#pragma once

#include "adc/sampling.h"

#include "tele/telemetry.h"
#include "util.h"

#include "adc/temperature.h"
#include "cooling.h"

#include "buck.h"
#include "pwm/backflow.h"
#include "battery.h"
#include "viz/lcd.h"
#include "tracker.h"
#include "pd_control.h"
#include "store.h"
#include "metering.h"
#include "charger.h"
#include "etc/plot.h"
#include "app_state.h"

struct Limits {
    const float Vin_max{};
    const float Vin_min{};

    const float Vout_max{};

    const float Iin_max{}, Ishort{};
    const float Iout_max{};

    const float P_max{};

    const float Temp_max{};

    const float Temp_derate{};

    const bool reverse_current_paranoia{};


    explicit Limits(const ConfFile &limits)
        : Vin_max(limits.getFloat("vin_max")), Vin_min(limits.getFloat("vin_min")),
          Vout_max(limits.getFloat("vout_max")),
          Iin_max(limits.getFloat("iin_max")), Ishort(limits.getFloat("iout_short")),
          Iout_max(limits.getFloat("iout_max")),
          P_max(limits.getFloat("p_max")), Temp_max(limits.getFloat("temp_max", 90.0f)),
          Temp_derate(limits.getFloat("temp_derate")),
          reverse_current_paranoia(limits.getByte("reverse_current_paranoia", 1) != 0) {
        assert_throw(Vin_max > Vin_min, "");
        assert_throw(Vin_max * Iin_max > P_max, "");
        assert_throw(Vin_max * Iin_max < P_max * 4, "");
        assert_throw(Temp_derate < Temp_max, "");
        assert_throw(20 < Temp_max and Temp_max < 120, "");
    }

    Limits() = default;

    Limits(const Limits &lim) = default;

    Limits &operator=(const Limits &right) {
        if (this == &right) return *this;
        this->~Limits();
        new(this) Limits(right);
        return *this;
    }

    //Limits &operator=(const Limits& other) = default;
};


struct TeleConf {
    IPAddress influxdbHost;

    TeleConf() : influxdbHost(0UL) {
    }

    TeleConf(const ConfFile &teleConf) {
        auto host = teleConf.getString("influxdb_host", "");
        influxdbHost = host.empty() ? IPAddress(0UL) : IPAddress(host.c_str());
    }
};

struct MpptParams : public BatChargerParams {
    float Vin_max = 80.f;
    float Vin_min = 10.5f;
    float Iin_max = 30.f;
    float P_max = 800.f;
};

enum class MpptControlMode : uint8_t {
    None = 0,
    CV,
    CC, // = 2
    CP,
    MPPT, // 4
    Sweep,
    Max,
};


// inline + const char* => single copy in flash .rodata, no per-TU std::string array or global ctor
inline constexpr std::array<const char *, (size_t) MpptControlMode::Max> MpptState2String{
    "N/A",
    "CV",
    "CC",
    "CP",
    "MPPT",
    "SWEEP"
};


struct TopologyConfig {
    bool backflowAtHV = false; //backflow switch is at solar input
};

template<typename T_NUM, T_NUM max = std::numeric_limits<T_NUM>::max()>
struct MinSampler {
    T_NUM min{max};

    void add(const T_NUM &v) { if (v < min) min = v; }

    void reset() { min = max; }

    bool empty() const { return min == max; }

    const T_NUM &get(bool &empty) const {
        if (min == max) empty = true;
        return min;
    }

    const T_NUM &get() const {
        return min;
    }

    bool tryGet(T_NUM &out) const {
        if (min == max)return false;
        out = min;
        return true;
    }

    T_NUM pop(bool &empty) {
        auto r = get(empty);
        reset();
        return r;
    }

    T_NUM pop() {
        auto r = get();
        reset();
        return r;
    }
};

/**
 * Implements
 * - Protection of DCDC converter
 * - Voltage and current control loop
 * - MPP global scan
 * - Telemetry
 */
class MpptController {
    ADC_Sampler &sampler;

public:
    SynchronousConverter &converter;
    LCD &lcd;

private:
    float cntrlValue = 0.0f;

    struct {
        MpptControlMode mode: 3 = MpptControlMode::None;
        bool _limiting: 1 = false; // control limited (no MPPT)
        uint8_t limIdx: 4 = 15;
    } ctrlState;

public:
    //MinSampler<MpptControlMode, MpptControlMode::Max> ctrlModeSampled{};
    MinSampler<uint8_t, 15> limIdxSampled{};
    uint16_t targetDutyCycle = 0; // one-shot ramp target from sweep/MPP (cleared on arrival)
    uint16_t manualTarget = 0;   // persistent manual-mode duty target (survives backoff, cleared only by dc 0)

private:
    bool _sweeping = false; // global scan


    struct {
        float power = 0;
        float voltage = 0;
        uint16_t dutyCycle = 0;
    } maxPowerPoint; // MPP during sweep

    // A sweep only commits its captured MPP as the operating target above this power. In marginal
    // light (e.g. a dawn cold-start sweep) the "peak" is just noise at near-max duty; committing it
    // strands the converter there until it's driven to the opposite rail. Below it: no target, fall
    // back to normal MPPT / the next periodic re-sweep. MPPT still harvests sub-threshold light.
    static constexpr float SweepMinPower = 5.0f; // W

    Plot sweepPlot{};

    time_ms lastTimeProtectPassed = 0;
    time_us _lastPointWrite = 0;
    time_us _backoffUntilUs = 0;
    uint32_t _backoffArmedSec = 0; // length of the running backoff, so a repeat can't extend it
    time_ms psuTripWindowStart = 0;
    uint8_t psuTripCount = 0;
    unsigned short _teleNumPoints = 0;

    const VIinVout<const Sensor *> &sensors;


    // Gains below are the defaults; converter.conf::ctrl_<name>_{kp,kd,td} override them (begin()).
    // They are not the whole loop gain: update() scales the result into a duty slew rate
    // by a further per-path constant (see kCtrlSlewLimit in mppt.cpp).
    PD_Control VinController{-100, -200, true}; // Vin under-voltage
    PD_Control VoutController{1500, /*100**/ 12 * 1000, true}; // Vout over-voltage  TODO 8k, 10k prevents full sweep
    PD_Control IinController{100, 200, true}; // Iin over-current
    PD_Control_SmoothSetpoint IoutCurrentController{200, 400, 200}; // Iout over-current // TODO PID?
    PD_Control_SmoothSetpoint powerController{20, 5, 200}; // over-power // TODO PID?
    //PD_Control LoadRegulationCTRL{5, -200, true}; //

public:
    const Sensor *sensorPhysicalI{nullptr};
    const Sensor *sensorPhysicalU{nullptr};
    Tracker tracker{};

private:
    TopologyConfig topologyConfig;

    uint8_t ledPinSimple = 255;

    uint16_t targetPwmCnt = 0; // config-derived boot duty; cleared on mppt/sweep to allow tracking

    float sweepSpeed = 4.0f; // global-sweep speed (tracker.conf::sweep_speed)

    struct flags_ {
        bool autoDetectVout_max: 1 = true;
    };

    flags_ flags;

public:
    //MpptParams params;
    Limits limits{};
    TeleConf tele{};
    BatteryCharger charger;
    BackflowDriver bflow{};
    SolarEnergyMeter meter{};
    TempSensorGPIO_NTC ntc;
    Esp32TempSensor ucTemp;
    Fan fan{};

    float speedScale = 1;

    explicit MpptController(ADC_Sampler &dcdcPwr, const VIinVout<const Sensor *> &sensors,
                            SynchronousConverter &converter, LCD &lcd)
        : sampler(dcdcPwr), converter(converter), lcd(lcd), sensors{sensors},
          charger{} {
    }

    void initSensors(const ConfFile &boardConf) {
        assert_throw(sensors.Vout, "");
        assert_throw(sensors.Iout, "");

        if (sensors.Iout->isVirtual) {
            ESP_LOGI("mppt", "Iout sensor is virtual, using Iin");
            sensorPhysicalI = sensors.Iin;
            sensorPhysicalU = sensors.Vin;
        } else {
            // use Iout by default
            sensorPhysicalI = sensors.Iout;
            sensorPhysicalU = sensors.Vout;
        }
        if (sensorPhysicalI->isVirtual) throw std::runtime_error("no physical I sensor");
        if (sensorPhysicalU->isVirtual) throw std::runtime_error("no physical U sensor");

        ntc.begin(boardConf);
        ucTemp.begin();
        ucTemp.read();
    }

    void begin(const ConfFile &trackerConf, const ConfFile &boardConf, const ConfFile &converterConf,
               const Limits &limits_, const TeleConf &tele_);

    [[nodiscard]] MpptControlMode getState() const { return ctrlState.mode; }

    struct TeleSnap {
        float Ui, Uo, I, P, mcuTemp, ntcTemp;
        uint16_t duty;
        uint8_t mode, limIdx;
    };

    TeleSnap teleSnap() {
        float i = sensorPhysicalI->med3.get();
        return {.Ui = sensors.Vin->med3.get(), .Uo = sensors.Vout->med3.get(), .I = i,
                .P = i * sensorPhysicalU->med3.get(), .mcuTemp = ucTemp.last(), .ntcTemp = ntc.last(),
                .duty = (uint16_t) converter.getCtrlOnPwmCnt(),
                .mode = (uint8_t) ctrlState.mode, .limIdx = ctrlState.limIdx};
    }
    [[nodiscard]] bool active() const { return _sweeping or sampler.isCalibrating() or !converter.disabled(); }
    [[nodiscard]] bool isSweeping() const { return _sweeping; }

    // Default backoff blocks startCondition() so MPPT doesn't re-poke pwmPerturb()
    // every tick after a protection trip — otherwise OV/OC violations spam the log
    // and toggle the converter at sample rate (see Vin-OV regression with Voc>vin_max).
    // Callers that want immediate-recovery semantics (calibration done, user `dc 0`)
    // must pass 0 explicitly. `who` tags the trip path in the backoff log so a
    // stuck post-sweep state names the responsible protect.
    void shutdownDcdc(const char *who, uint32_t backoffSec = 5) {
        if (topologyConfig.backflowAtHV) {
            converter.disable();
            bflow.enable(false);
        } else {
            bflow.enable(false);
            converter.disable();
        }
        if (backoffSec) {
            // PSU fast-retry: only for transient faults (Vout-OV, supply-UV). Other faults
            // (sensor-fail, revI, highI) keep their normal backoff even in PSU mode.
            // stopAndBackoff caller is maintenance, not a fault — don't count it.
            bool psuFastRetry = g_app.psuMode() && backoffSec <= 5
                && strcmp(who, "stopAndBackoff") != 0
                && (strstr(who, "Vout-OV") || strstr(who, "supply-UV"));
            if (psuFastRetry) {
                auto nowMs = wallClockMs();
                if (nowMs - psuTripWindowStart > 60000) {
                    psuTripWindowStart = nowMs;
                    psuTripCount = 0;
                }
                ++psuTripCount;
                if (psuTripCount > 8) {
                    psuLatched = true;
                    _backoffUntilUs = wallClockUs() + 30000000ULL;
                    _backoffArmedSec = 30;
                    ESP_LOGE("mppt", "PSU latch [%s] (%u trips)", who, psuTripCount);
                } else if (psuTripCount > 4) {
                    psuEscalated = true;
                    if (!inBackoff() || backoffSec > _backoffArmedSec) {
                        _backoffUntilUs = wallClockUs() + static_cast<time_us>(backoffSec) * 1000000ULL;
                        _backoffArmedSec = backoffSec;
                        ESP_LOGW("mppt", "PSU escalated backoff %lus [%s] (trip %u)",
                                 (unsigned long) backoffSec, who, psuTripCount);
                    }
                } else {
                    _backoffUntilUs = wallClockUs() + 100000ULL;
                    _backoffArmedSec = 0;
                    ESP_LOGW("mppt", "PSU fast retry [%s] (trip %u)", who, psuTripCount);
                }
            } else {
                if (!inBackoff() || backoffSec > _backoffArmedSec) {
                    _backoffUntilUs = wallClockUs() + static_cast<time_us>(backoffSec) * 1000000ULL;
                    _backoffArmedSec = backoffSec;
                    ESP_LOGW("mppt", "backoff %lus [%s]%s", (unsigned long) backoffSec, who,
                             _sweeping ? " mid-sweep" : "");
                }
            }
            _sweeping = false;
        }
    }

    // Manual override (console `sweep`): drop any pending backoff so the user's
    // intent isn't silently absorbed by a stale trip timer.
    void clearBackoff() {
        _backoffUntilUs = 0;
        _backoffArmedSec = 0;
    }

    // Discard a running scan without committing its MPP. active() stays true while _sweeping is
    // set, so a scan the user overrides (manual PWM) would otherwise keep the state machine in
    // sweep mode until the next `mppt`.
    void abortSweep() {
        _sweeping = false;
        maxPowerPoint = {};
        targetDutyCycle = 0;
    }

    // Break a CV-floor lockup in place (no reboot): release the charger's Vout pinning and reset the
    // Vout controller + limit/target state so normal MPPT can climb again. Returns true if the
    // charger pin actually moved (i.e. there was a latch to clear).
    bool releaseCvFloorLatch(const char *why) {
        bool changed = charger.releaseVoutPinning(why);
        VoutController.reset();
        ctrlState._limiting = false;
        ctrlState.limIdx = 15;
        if (ctrlState.mode == MpptControlMode::CV) ctrlState.mode = MpptControlMode::None;
        targetDutyCycle = 0;
        clearBackoff();
        return changed;
    }

    [[nodiscard]] bool inBackoff() const { return wallClockUs() < _backoffUntilUs; }

    [[nodiscard]] float boardPowerSupplyVoltage() const {
        constexpr auto diodeFwdVoltage = 0.3f;
        return std::max(sensors.Vin->last, sensors.Vout->last) - diodeFwdVoltage;
    }

    [[nodiscard]] bool boardPowerSupplyUnderVoltage(bool start = false) const {
        if (isnan(sensors.Vin->last) || isnan(sensors.Vout->last))
            return false;
        return boardPowerSupplyVoltage() < (start ? 9.5f : 9.f);
    }

    // First startCondition() clause currently blocking a start, or nullptr if clear.
    // Single source of truth for startCondition(); also logged while idle in START.
    [[nodiscard]] const char *startBlockReason() const {
        if (psuLatched) return "psu-latch";
        if (inBackoff()) return "backoff";
        if (ntc.last() > limits.Temp_max - 3 || !(ucTemp.last() < limits.Temp_max - 3)) return "temp";
        if (!g_app.psuMode() &&
            !(converter.boost()
                  ? sensors.Vin->ewm.avg.get() < sensors.Vout->ewm.avg.get() + 1
                  : sensors.Vin->ewm.avg.get() > sensors.Vout->ewm.avg.get() + 1))
            return "Vin-Vout";
        if (g_app.psuMode() && sensors.Vout && sensors.Vout->last > computeOvThreshold() * 0.98f)
            return "Vout-still-OV";
        if (boardPowerSupplyUnderVoltage(true)) return "supply-UV";
        if (sampler.isCalibrating()) return "calibrating";
        return nullptr;
    }

    [[nodiscard]] bool startCondition() const { return startBlockReason() == nullptr; }

    bool protectLf(bool ignoreUV) {
        //auto nowMs = loopWallClockMs();

        // power supply under-voltage shutdown
        if (boardPowerSupplyUnderVoltage() and not ignoreUV) {
            if (!converter.disabled())
                ESP_LOGW("mppt", "Supply under-voltage! Vin %.1f and Vout %.1f < 10", sensors.Vin->last,
                     sensors.Vout->last);
            shutdownDcdc("supply-UV");
            enqueue_task([&] { meter.commit(); });
            return false;
        }

        // detect battery voltage
        // TODO move this to charger ?
        if (!g_app.psuMode() && !charger.params.haveVbatMax()) {
            auto vout = sensors.Vout->calibrationAvg;
            float detectedVout_max = detectMaxBatteryVoltage(vout);
            if (std::isnan(detectedVout_max)) {
                ESP_LOGW("mppt", "Unable to detect battery voltage Vout=%.2fV", vout);
                converter.disable();
                enqueue_task([&] { sampler.startCalibration(); });
                return false;
            } else {
                ESP_LOGI("mppt", "Detected max battery voltage %.2fV (from Vout=%.2fV)", detectedVout_max, vout);
                charger.params.Vbat_max = min(limits.Vout_max, detectedVout_max);
            }
        }

        if (ntc.last() > limits.Temp_max || ucTemp.last() > limits.Temp_max) {
            ESP_LOGE("mppt", "Temp %.1f (or mcu %.1f) > %.1f°C, shutdown", ntc.last(), ucTemp.last(), limits.Temp_max);
            return false;
        }

        return true;
    }

    bool protect(bool ignoreUV) {
        auto nowMs = wallClockMs();

        // input over-voltage
        if (sensors.Vin->last > limits.Vin_max) {
            // input over-voltage
            ESP_LOGW("mppt", "Vin %.1f > %.1f!", sensors.Vin->last, limits.Vin_max);
            shutdownDcdc("Vin-OV");
            return false;
        }

        // output over-voltage
        // An explicit `ovset` overrides the derived threshold (issue #59). Without it the threshold
        // is derived from Vbat_max × a factor (1.03 with reverse_current_paranoia, 1.5 without).
        // Until the battery is identified, the hard configured ceiling is the only threshold we
        // have. Deriving one from an unusable Vbat_max instead would yield ~0 and trip OV on any
        // output voltage — and protectLf()'s re-detect never runs, because a trip returns before it.
        auto ovTh = computeOvThreshold();
        //if (adcSampler.med3.s.chVout.get() > ovTh) {
        if (sensors.Vout->last > ovTh) {
            //  && sensors.Vout->previous > ovTh * 0.9f
            bool wasDisabled = converter.disabled();
            shutdownDcdc("Vout-OV");

            auto vout = std::max(sensors.Vout->last, sensors.Vout->previous);

            if (!wasDisabled)
                ESP_LOGW("mppt", "Vout %.1fV (prev=%.1fV,ewma=%.1fV,std=%.4f,D=%hu) > %.1fV + 5pct!",
                     sensors.Vout->last, sensors.Vout->previous,
                     sensors.Vout->ewm.avg.get(), sensors.Vout->ewm.std.get(), converter.getCtrlOnPwmCnt(),
                     ovTh
            );


            if (flags.autoDetectVout_max && !g_app.psuMode() && nowMs - lastTimeProtectPassed > 20000) {
                // Persistent OV: auto-detect Vout_max by clearing the setpoint so protectLf()
                // re-detects it. But an explicitly commanded `vset` setpoint must not be silently
                // discarded — that widens the protection threshold (to the board ceiling) with no
                // feedback to the host (issue #59).
                if (charger.params.vbatMaxExplicit) {
                    ESP_LOGE("mppt", "Vout OV persists %.1fs, but Vbat_max=%.2fV was set via vset — NOT resetting",
                             (float) (nowMs - lastTimeProtectPassed) / 1000.f, charger.params.Vbat_max);
                } else {
                    ESP_LOGW("mppt", "Vout OV persists, clearing Vbat_max for auto-detect");
                    charger.params.Vbat_max = NAN;
                    sampler.startCalibration();
                }
            }

            enqueue_task([&] {
                lcd.displayMessageF("OV shutdown!\nVout=%.1fV max=%.1fV", 10000, vout, ovTh);
            });

            return false;
        }


        // input over current
        if (sensors.Iin->last > limits.Iin_max * 1.3f && !converter.disabled()) {
            shutdownDcdc("Iin-OC");
            ESP_LOGW("mppt", "Iin %.1f >1.3x lim (Iout=%.1f Vin=%.2f), shutdown",
                     sensors.Iin->last,
                     sensors.Iout->last, sensors.Vin->last);
            return false;
        }

        // output over current
        if ((sensors.Iout->last > limits.Iout_max * 1.5f
             or sensors.Iout->med3.get() > limits.Iout_max * 1.25f
             or sensors.Iout->ewm.avg.get() > limits.Iout_max * 1.15f
            ) and not converter.disabled()) {
            shutdownDcdc("Iout-OC", 30);
            ESP_LOGW("mppt", "Iout %.2f (med %.2f avg %.2f) >lim %.2f, shutdown", sensors.Iout->last,
                     sensors.Iout->med3.get(), sensors.Iout->ewm.avg.get(), limits.Iout_max);
            return false;
        }

        if (sensorPhysicalI->last < -1 && sensorPhysicalI->previous < -1 && !converter.forcedPwm_()) {
            if (sensors.Iout->ewm.avg.get() > 10) {
                //buck.halfDutyCycle();
                shutdownDcdc("revI-highAvg");
                ESP_LOGE("MPPT", "Reverse I %.2fA, noise? high avg, shutdown", sensorPhysicalI->last);
            } else {
                if (bflow.state() || converter.getRectOnPwmCnt() > converter.getRectOnPwmMin())
                    ESP_LOGE("MPPT", "Reverse I %.2fA, noise? disable BFC+LS FET (pwm=%hu)",
                         sensorPhysicalI->last, converter.getCtrlOnPwmCnt());
                bflow.enable(false); // reverse current
                converter.syncRectMinDuty();
            }
        }

        if (sensorPhysicalI->ewm.avg.get() < -1 /*&& !converter.forcedPwm_()*/) {
            if (!converter.disabled())
                ESP_LOGE("MPPT", "Reverse avg current %.1f A, shutdown!", sensorPhysicalI->ewm.avg.get());
            shutdownDcdc("revI-ewm");
            return false;
        }

        if (!converter.boost()) {
            if (sensors.Vout->ewm.avg.get() > (sensors.Vin->ewm.avg.get() + 1.0f) * 1.25f) {
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "Vout %.1f > Vin %.1f, shutdown duty=%i", sensors.Vout->ewm.avg.get(),
                         sensors.Vin->ewm.avg.get(), (int) converter.getCtrlOnPwmCnt());
                shutdownDcdc("Vout>Vin-avg");
                return false;
            }
            if (sensors.Vout->last > (sensors.Vin->last + .5f) * 2) {
                ESP_LOGE("MPPT", "Vout %.1f > 2x Vin %.1f, shutdown", sensors.Vout->last, sensors.Vin->last);
                shutdownDcdc("Vout>2Vin");
                return false;
            }

            // try to prevent voltage boost and disable low side for low currents
            auto currentFilt = fminf(sensorPhysicalI->ewm.avg.get(),
                                     std::max(sensorPhysicalI->last, sensorPhysicalI->previous));
            if (currentFilt < -0.05f && limits.reverse_current_paranoia) {
                if (converter.getRectOnPwmCnt() > converter.getRectOnPwmMax() / 2 &&
                    converter.getRectOnPwmCnt() > (converter.pwmRectMin + converter.pwmCtrlMax / 20)) {
                    ESP_LOGW("MPPT", "Low I, set LS min duty (ewm(%s)=%.2f, max=%.2f)",
                             sensorPhysicalI->params.teleName.c_str(),
                             sensorPhysicalI->ewm.avg.get(),
                             std::max(sensorPhysicalI->last, sensorPhysicalI->previous));
                }
                if (bflow.state())
                    ESP_LOGW("MPPT", "Low current %.2f, disable backflow", currentFilt);
                if (converter.getRectOnPwmCnt() > converter.getRectOnPwmMin())
                    ESP_LOGW("MPPT", "Low current %.2f, disable sync rect", currentFilt);
                converter.syncRectMinDuty();
                bflow.enable(false); // low current
            }
        } else {
            // TODO Vin
        }

        if (sensors.Iout->ewm.avg.get() > limits.Ishort and sensors.Vout->ewm.avg.get() < 1) {
            if (!converter.disabled())
                ESP_LOGE("MPPT", "Output short circuit detected! (V=%.2f, I= %.1fA)",
                     sensors.Vout->ewm.avg.get(), sensors.Iout->ewm.avg.get());
            shutdownDcdc("short", 30);
            return false;
        }

        // if bflow switch is powered by HS gate drive, need a min duty cycle
        // TODO lift this, bflow switch will be powered from bootstrap cap and not gate drive signal
        constexpr auto BflowMinDutyCycle = 0.1f;
        if (bflow && (!bflow.state() || converter.getDutyCycle() < BflowMinDutyCycle)) {
            if (sensorPhysicalI->ewm.avg.get() > 6) {
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "High-current through open backflow switch!");
                shutdownDcdc("highI-bflowOpen");
                return false;
            }

            if (converter.getDutyCycle() > 0.33f) {
                // in case the current sensor is wrong
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "High duty cycle with open backflow switch!");
                shutdownDcdc("highD-bflowOpen");
                return false;
            }
        }

        if (!converter.syncRectEnabled_()) {
            if (sensorPhysicalI->ewm.avg.get() > 6) {
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "High current without sync rectification!");
                shutdownDcdc("highI-noSyncRect");
                return false;
            }
        }

        // TODO move this to control loop
        //float vOut = fmaxf(sensors.Vout->med3.get(), sensors.Vout->ewm.avg.get());
        //float vIn = fminf(sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get());
        float vOut = sensors.Vout->ewm.avg.get();
        float vIn = sensors.Vin->ewm.avg.get();
        // TODO smoothing!
        auto vr = converter.updateSyncRectMaxDuty(
            vIn, vOut, converter.boost() ? sensors.Iin->ewm.avg.get() : sensors.Iout->ewm.avg.get());

        auto iOutSmall = sensorPhysicalI->ewm.avg.get() < (limits.Iout_max * 0.01f);

        if (iOutSmall && converter.getCtrlOnPwmCnt() > converter.pwmRectMin * 2 and
            (converter.forcedPwm_()
                 ? (vOut < 1 or (converter.getDutyCycle() * 0.5f) > vr)
                 : (converter.getDutyCycle() * 0.8f) > vr)
            and limits.reverse_current_paranoia
            and !(g_app.psuMode() && vOut > 0.5f * psuVsetpoint && vOut < psuVsetpoint)) {
            if (!converter.disabled())
                ESP_LOGE("MPPT",
                     "Buck D=%d%% but Vout(%.2f,vr=%.2f) Iout(%.2f,last=%.2f) low! sensor/HB fail",
                     100 * converter.getCtrlOnPwmCnt() / converter.pwmCtrlMax, vOut, vr,
                     sensors.Iout->ewm.avg.get(),
                     sensors.Iout->last
            );

            shutdownDcdc("Vr-sensor-fail");
            return false;
        }

        lastTimeProtectPassed = nowMs;

        return true;
    }


    /**
     * Start a global MPPT scan.
     */
    void startSweep() {
        _sweeping = true;

        converter.disable();
        ctrlState._limiting = false;
        targetDutyCycle = 0;

        VinController.reset();
        VoutController.reset();
        IinController.reset();
        IoutCurrentController.reset();
        //LoadRegulationCTRL.reset();

        ESP_LOGI("mppt", "Start sweep");


        maxPowerPoint = {};

        sampler.startCalibration();

        enqueue_task([&] {
            rtcount_en = false;
            vTaskDelay(10);
            sweepPlot.reserve();
            meter.commit(); // not real-time safe
            lcd.periodicInit(); // not real-time safe
            rtcount_en = true;
        });
    }

    // Called from the console task (core 0). Stores the target only; the RT loop
    // ramps pwmCtrl on core 1 — keeps all PWM writes on one core so a concurrent
    // disable() can't race an in-flight ledc_update_duty (LEDC has no force-low latch).
    void setManualTarget(uint16_t duty) {
        if (duty > converter.pwmCtrlMax) duty = converter.pwmCtrlMax;
        manualTarget = duty;
    }

    // Drop the config-derived boot duty cap so update() resumes normal MPPT tracking.
    void clearBootTarget() { targetPwmCnt = 0; }

    // One-shot automatic ramp target (consumed by update() sweep/MPP fade path).
    void setAutoRampTarget(uint16_t duty) {
        if (duty > converter.pwmCtrlMax) duty = converter.pwmCtrlMax;
        targetDutyCycle = duty;
    }

    float psuVsetpoint = NAN; // PSU CV setpoint (V); NAN = not commanded
    bool psuEscalated = false;
    bool psuLatched = false;

    void setPsuSetpoint(float v) {
        if (!std::isfinite(v) || v <= 0 || v > limits.Vout_max) return;
        VoutController.reset();
        psuVsetpoint = v;
    }

    void psuResetTripState() {
        psuTripCount = 0;
        psuTripWindowStart = 0;
        psuEscalated = false;
        psuLatched = false;
    }

    [[nodiscard]] uint8_t getPsuTripCount() const { return psuTripCount; }

    [[nodiscard]] float computeOvThreshold() const {
        if (std::isfinite(charger.params.Vout_ov_limit) && charger.params.Vout_ov_limit > 0)
            return std::min(charger.params.Vout_ov_limit, limits.Vout_max);
        if (g_app.psuMode() && std::isfinite(psuVsetpoint))
            return std::min(psuVsetpoint * (limits.reverse_current_paranoia ? 1.03f : 1.5f), limits.Vout_max);
        if (charger.params.haveVbatMax())
            return std::min(charger.params.Vbat_max * (limits.reverse_current_paranoia ? 1.03f : 1.5f),
                            limits.Vout_max);
        return limits.Vout_max;
    }

    struct CVP {
        MpptControlMode mode;
        PD_Control &crtl;

        struct {
            float actual, target;
        };
    };


    /**
     * Stops MPPT scan and set duty cycle to captured MPP
     * @param controlMode
     */
    void _stopSweep(MpptControlMode controlMode, int limIdx, CVP *limCtrl) {
        _sweeping = false;

        if (maxPowerPoint.power < SweepMinPower) {
            // Marginal light: no real MPP found. Don't commit a phantom target (would strand the
            // converter at near-max duty). Drop the target and back off briefly; the converter then
            // resumes via normal MPPT / the next periodic re-sweep once there's real power.
            targetDutyCycle = 0;
            ESP_LOGI("mppt", "Stop sweep: no MPP (best %.2fW < %.1fW), backing off", maxPowerPoint.power,
                     SweepMinPower);
            shutdownDcdc("sweep-no-mpp", 30);
            return;
        }

        targetDutyCycle = maxPowerPoint.dutyCycle;

        ESP_LOGI("mppt",
                 "Stop sweep %.2fs mode=%s (lim=%i tgt=%.2f act=%.2f) PWM=%hu MPP=(%.1fW,%hu,%.1fV)",
                 (wallClockUs() - sampler.getTimeLastCalibrationUs()) * 1e-6f,
                 MpptState2String[(uint8_t) controlMode], limIdx,
                 limCtrl ? limCtrl->target : NAN, limCtrl ? limCtrl->actual : NAN,
                 converter.getCtrlOnPwmCnt(), maxPowerPoint.power, maxPowerPoint.dutyCycle, maxPowerPoint.voltage
        );

        enqueue_task([&] {
            // TODO replace displayMessageF with a general message callback call
            lcd.displayMessageF("MPP Scan done\n%.1fW @ %.1fV", 6000, maxPowerPoint.power, maxPowerPoint.voltage);
            sweepPlot.plot();
        });
    }

    void telemetry();

    time_us lastUs = 0;

    void update(); // normal update
    void updateManual(); // manual mode
};
