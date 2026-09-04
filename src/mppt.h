#pragma once

#include <atomic>

#include "freertos/FreeRTOS.h"

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
#include "math/pv_model.h"

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

enum class PsuSetpointError : uint8_t {
    None = 0,
    OutOfRange,
    TelemetryUnavailable,
    BoostBelowInput,
    OvLimitConflict,
};

enum class PsuCommand : uint8_t {
    None = 0, Enable, DisableMppt, DisableManual, StartSweep, ShortLowSide, EnablePv
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
    // Written by console/self-test tasks and consumed by the RT loop. The target itself is the
    // mailbox; PWM hardware remains RT-owned.
    std::atomic<uint16_t> manualTarget{0};

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
    std::atomic<uint8_t> psuTripCount{0};
    std::atomic<float> pendingPsuSetpoint{NAN};
    // [31:8] request ticket, [7:0] PsuCommand. One atomic word binds the action to its
    // completion identity, so a later request cannot make an earlier RT completion wake it.
    std::atomic<uint32_t> pendingPsuCommandWord{0};
    std::atomic<uint32_t> nextPsuTicket{0};
    // More than one non-RT producer can wait concurrently. A single "last completed" ticket
    // loses A when B completes before A is scheduled. Keep a bounded result slot per ticket;
    // there are currently at most two producers (console and measure-coil), with ample margin.
    static constexpr size_t PsuResultSlots = 8;
    std::array<std::atomic<uint32_t>, PsuResultSlots> completedPsuResults{};
    std::atomic<uint32_t> psuMailboxSeq{0};
    portMUX_TYPE psuMailboxMux = portMUX_INITIALIZER_UNLOCKED;
    std::atomic<PsuSetpointError> lastPsuRequestError{PsuSetpointError::None};
    std::atomic<bool> pendingPsuBootRequest{false};
    std::atomic<float> publishedPsuSetpoint{NAN};
    // Runtime console setting: both CLI and RT protection read it, so it cannot live in the
    // otherwise single-core charger params object as an ordinary float.
    std::atomic<float> explicitOvLimit{NAN};
    std::atomic<uint16_t> pendingManualTarget{0};
    std::atomic<int> pendingManualRect{-1};
    std::atomic<float> pendingPvIsc{NAN}, pendingPvVoc{NAN}, pendingPvK{NAN};
    std::atomic<bool> pendingPvRebase{false};
    // Published PV curve snapshot for non-RT readers (status/OTA/measure-coil). Seqlock
    // (odd = write in progress) so a concurrent curve update can't yield a mixed set.
    std::atomic<uint32_t> pvPubSeq{0};
    std::atomic<float> pvPubIsc{NAN}, pvPubVoc{NAN}, pvPubK{NAN};
    std::atomic<bool> pvPubActive{false};
    std::atomic<float> pvBaseIsc{NAN}; // last full `pv` Isc, reference for `pv scale`
    unsigned short _teleNumPoints = 0;

    const VIinVout<const Sensor *> &sensors;

    void lockPsuMailbox() {
        // Producers can be separate tasks on NON_RT_CORE (console, OTA and measure-coil).
        // A plain atomic spin lock can deadlock if a higher-priority producer preempts its
        // owner on that same core. The IDF port mux disables local preemption while owned;
        // the RT consumer never takes it and only observes the odd/even sequence below.
        portENTER_CRITICAL(&psuMailboxMux);
        psuMailboxSeq.fetch_add(1, std::memory_order_acq_rel);
    }

    uint32_t postPsuCommand(PsuCommand command) {
        uint32_t ticket = (nextPsuTicket.fetch_add(1, std::memory_order_relaxed) + 1) & 0x00ffffffu;
        if (ticket == 0)
            ticket = (nextPsuTicket.fetch_add(1, std::memory_order_relaxed) + 1) & 0x00ffffffu;
        pendingPsuCommandWord.store((ticket << 8) | static_cast<uint8_t>(command),
                                    std::memory_order_release);
        psuMailboxSeq.fetch_add(1, std::memory_order_release);
        portEXIT_CRITICAL(&psuMailboxMux);
        return ticket;
    }

    void completePsuCommand(uint32_t ticket, PsuSetpointError error) {
        completedPsuResults[ticket % PsuResultSlots].store(
            (ticket << 8) | static_cast<uint8_t>(error), std::memory_order_release);
    }

    // RT-only seqlock writer for the published PV curve snapshot. The release fence after the
    // odd-marking increment is load-bearing: a release RMW alone does not keep the payload
    // stores from becoming visible before it.
    void publishPvState(bool active, float isc, float voc, float k) {
        pvPubSeq.fetch_add(1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        pvPubActive.store(active, std::memory_order_relaxed);
        pvPubIsc.store(isc, std::memory_order_relaxed);
        pvPubVoc.store(voc, std::memory_order_relaxed);
        pvPubK.store(k, std::memory_order_relaxed);
        pvPubSeq.fetch_add(1, std::memory_order_release);
    }

    void pvDeactivateRt() {
        if (!pvSim.active) return;
        pvSim.active = false;
        publishPvState(false, NAN, NAN, NAN);
    }


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
                    psuTripCount.store(0, std::memory_order_relaxed);
                }
                const uint8_t tripCount = psuTripCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (tripCount > 8) {
                    psuLatched = true;
                    _backoffUntilUs = wallClockUs() + 30000000ULL;
                    _backoffArmedSec = 30;
                    ESP_LOGE("mppt", "PSU latch [%s] (%u trips)", who, tripCount);
                } else if (tripCount > 4) {
                    psuEscalated = true;
                    if (!inBackoff() || backoffSec > _backoffArmedSec) {
                        _backoffUntilUs = wallClockUs() + static_cast<time_us>(backoffSec) * 1000000ULL;
                        _backoffArmedSec = backoffSec;
                        ESP_LOGW("mppt", "PSU escalated backoff %lus [%s] (trip %u)",
                                 (unsigned long) backoffSec, who, tripCount);
                    }
                } else {
                    _backoffUntilUs = wallClockUs() + 100000ULL;
                    _backoffArmedSec = 0;
                    ESP_LOGW("mppt", "PSU fast retry [%s] (trip %u)", who, tripCount);
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
        if (g_app.psuMode()) {
            const auto feasibility = currentPsuFeasibility();
            if (feasibility == PsuSetpointError::BoostBelowInput)
                return "psu-boost-setpoint";
            if (feasibility == PsuSetpointError::OvLimitConflict)
                return "psu-ov-conflict";
        }
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

        if (g_app.psuMode()) {
            const auto feasibility = currentPsuFeasibility();
            if (feasibility == PsuSetpointError::BoostBelowInput
                || feasibility == PsuSetpointError::OvLimitConflict) {
                if (!psuLatched.exchange(true, std::memory_order_relaxed)) {
                    ESP_LOGE("mppt", "active PSU setpoint %.2f V became infeasible (Vin %.2f V, OV %.2f V)",
                             psuVsetpoint, sensors.Vin ? sensors.Vin->ewm.avg.get() : NAN,
                             getExplicitOvLimit());
                    shutdownDcdc("psu-setpoint-infeasible", 30);
                }
                return false;
            }
        }

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

    // RT-CORE ONLY. Applies a queued `pwm-freq` change and carries the duty state that lives HERE
    // through it. manualTarget, the boot duty cap, the one-shot ramp target and the tracker's
    // captured MPP are all raw PWM counts: rescaling only the converter would leave the very next
    // control tick ramping back toward a count that no longer means the same duty — on a bench in
    // manual PWM that undoes the frequency change's operating point within milliseconds.
    void applyPendingPwmFreqRt() {
        const float r = converter.applyPendingPwmFreqRt();
        if (!(r > 0.f)) {
            converter.ackPendingPwmFreqRt();
            return;
        }
        const uint16_t hi = converter.pwmCtrlMax;
        auto rescale = [r, hi](uint16_t v) {
            return (uint16_t) std::min<int32_t>(std::lround((float) v * r), hi);
        };
        manualTarget.store(rescale(manualTarget.load(std::memory_order_relaxed)),
                           std::memory_order_relaxed);
        targetPwmCnt = rescale(targetPwmCnt);
        targetDutyCycle = rescale(targetDutyCycle);
        // Rescaled, not cleared: the operating point is preserved across the change, so a captured
        // MPP is still the same physical point — only its count changed. Clearing it would send a
        // sweep-driven fade to duty 0. There are TWO captures and both are raw counts: the tracker's
        // (P&O) and this class's own sweep capture, which _stopSweep() commits as targetDutyCycle.
        tracker.maxPowerPoint.dutyCycle = rescale(tracker.maxPowerPoint.dutyCycle);
        maxPowerPoint.dutyCycle = rescale(maxPowerPoint.dutyCycle);
        // Last: the console returns as soon as this lands, and the next command it dispatches must
        // find every count already in the new period's scale.
        converter.ackPendingPwmFreqRt();
    }

    // One-shot automatic ramp target (consumed by update() sweep/MPP fade path).
    void setAutoRampTarget(uint16_t duty) {
        if (duty > converter.pwmCtrlMax) duty = converter.pwmCtrlMax;
        targetDutyCycle = duty;
    }

    float psuVsetpoint = NAN; // PSU CV setpoint (V); NAN = not commanded
    std::atomic<bool> psuEscalated{false};
    std::atomic<bool> psuLatched{false};

    // PV-sim (solar-array-simulator): the output follows the panel curve V = f(Iout).
    // RT-owned; while active, psuVsetpoint is advanced along the curve each tick
    // (pvAdvanceSetpoint) instead of being a constant. slewVps/iout span are boot-config
    // (begin(), before the RT loop runs).
    struct {
        bool active = false;
        PvModel model{};
        float slewVps = 200.f;
        EWMA<float> iout{16};
    } pvSim;

    void setPsuSetpoint(float v) {
        if (!std::isfinite(v) || v <= 0 || v > limits.Vout_max) return;
        VoutController.reset();
        psuVsetpoint = v;
        publishedPsuSetpoint.store(v, std::memory_order_release);
    }

    // Shared boost headroom: setpoint feasibility gate and the PV-sim curve floor.
    static constexpr float BoostHeadroomV = 0.5f;

    static PsuSetpointError validatePsuSetpoint(float requested, float voutMax, bool boost,
                                                float vin, float explicitOvLimit) {
        if (!std::isfinite(requested) || requested <= 0 || requested > voutMax)
            return PsuSetpointError::OutOfRange;
        // An explicit OV threshold is a hard protection setting, not a second CV target. Leave
        // the same 2% approach margin startBlockReason() uses so the requested operating point
        // cannot be inside the trip band by construction.
        if (std::isfinite(explicitOvLimit) && explicitOvLimit > 0
            && requested >= 0.98f * explicitOvLimit)
            return PsuSetpointError::OvLimitConflict;
        // A boost can regulate only ABOVE its input. Unknown Vin is not evidence that the
        // setpoint is feasible; entry waits for a fresh sample and rejects an unusable one.
        if (boost && !std::isfinite(vin))
            return PsuSetpointError::TelemetryUnavailable;
        if (boost && requested <= vin + BoostHeadroomV)
            return PsuSetpointError::BoostBelowInput;
        return PsuSetpointError::None;
    }

    [[nodiscard]] uint32_t queuePsuSetpoint(float v, bool bootRequest = false) {
        const auto precheck = validatePsuSetpoint(v, limits.Vout_max, false, NAN,
                                                  getExplicitOvLimit());
        if (precheck != PsuSetpointError::None) {
            lastPsuRequestError.store(precheck, std::memory_order_relaxed);
            return 0;
        }
        lockPsuMailbox();
        pendingPsuSetpoint.store(v, std::memory_order_relaxed);
        pendingPsuBootRequest.store(bootRequest, std::memory_order_relaxed);
        return postPsuCommand(PsuCommand::Enable);
    }

    [[nodiscard]] bool requestPsuSetpoint(float v, bool bootRequest = false) {
        return queuePsuSetpoint(v, bootRequest) != 0;
    }

    // The alpha family degenerates below k=0.5 (solver floors, ~linear curve) and k→1 is
    // pathological, hence the narrower range than vconv's model.
    [[nodiscard]] static bool validPvParams(float isc, float voc, float k) {
        return std::isfinite(isc) && isc > 0 && std::isfinite(voc) && voc > 0
               && std::isfinite(k) && k >= 0.5f && k <= 0.95f;
    }

    // rebase=false (`pv scale`) keeps pvBaseIsc as the reference for later scales.
    [[nodiscard]] uint32_t queuePvCurve(float isc, float voc, float k,
                                        bool bootRequest = false, bool rebase = true) {
        if (!validPvParams(isc, voc, k)) {
            lastPsuRequestError.store(PsuSetpointError::OutOfRange, std::memory_order_relaxed);
            return 0;
        }
        const auto precheck = validatePsuSetpoint(voc, limits.Vout_max, false, NAN,
                                                  getExplicitOvLimit());
        if (precheck != PsuSetpointError::None) {
            lastPsuRequestError.store(precheck, std::memory_order_relaxed);
            return 0;
        }
        lockPsuMailbox();
        pendingPvIsc.store(isc, std::memory_order_relaxed);
        pendingPvVoc.store(voc, std::memory_order_relaxed);
        pendingPvK.store(k, std::memory_order_relaxed);
        // The base is committed by the RT consumer on success — a producer-side store would
        // survive an RT-side rejection and corrupt a later `pv scale`.
        pendingPvRebase.store(rebase, std::memory_order_relaxed);
        pendingPsuBootRequest.store(bootRequest, std::memory_order_relaxed);
        return postPsuCommand(PsuCommand::EnablePv);
    }

    // Coherent read of the published curve. Returns active; outputs are NAN when never set.
    // Callers are non-RT tasks (console/OTA/measure-coil). Bounded retry with a yield: an odd
    // seq can persist for a whole interrupt on the RT core, so spin briefly, then sleep-retry;
    // the final fallback (possibly-mixed read) only fires if RT died mid-publish — better than
    // wedging the caller forever.
    bool getPvCurve(float &isc, float &voc, float &k) const {
        uint32_t s1, s2;
        bool active;
        for (int tries = 0; tries < 64; ++tries) {
            if (tries >= 8) vTaskDelay(1);
            s1 = pvPubSeq.load(std::memory_order_acquire);
            if (s1 & 1u) continue;
            active = pvPubActive.load(std::memory_order_relaxed);
            isc = pvPubIsc.load(std::memory_order_relaxed);
            voc = pvPubVoc.load(std::memory_order_relaxed);
            k = pvPubK.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = pvPubSeq.load(std::memory_order_relaxed);
            if (s1 == s2) return active;
        }
        active = pvPubActive.load(std::memory_order_relaxed);
        isc = pvPubIsc.load(std::memory_order_relaxed);
        voc = pvPubVoc.load(std::memory_order_relaxed);
        k = pvPubK.load(std::memory_order_relaxed);
        return active;
    }

    [[nodiscard]] bool isPvActive() const {
        return pvPubActive.load(std::memory_order_relaxed);
    }
    [[nodiscard]] float getPvBaseIsc() const {
        return pvBaseIsc.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t requestPsuOff() {
        lockPsuMailbox();
        pendingPsuBootRequest.store(false, std::memory_order_relaxed);
        return postPsuCommand(PsuCommand::DisableMppt);
    }

    [[nodiscard]] uint32_t requestPsuManual(uint16_t duty, int manualRect) {
        lockPsuMailbox();
        pendingManualTarget.store(duty, std::memory_order_relaxed);
        pendingManualRect.store(manualRect, std::memory_order_relaxed);
        pendingPsuBootRequest.store(false, std::memory_order_relaxed);
        return postPsuCommand(PsuCommand::DisableManual);
    }

    [[nodiscard]] uint32_t requestPsuSweep() {
        lockPsuMailbox();
        pendingPsuBootRequest.store(false, std::memory_order_relaxed);
        return postPsuCommand(PsuCommand::StartSweep);
    }

    [[nodiscard]] uint32_t requestPsuShortLowSide() {
        lockPsuMailbox();
        pendingPsuBootRequest.store(false, std::memory_order_relaxed);
        return postPsuCommand(PsuCommand::ShortLowSide);
    }

    [[nodiscard]] bool hasPendingPsuCommand() const {
        return pendingPsuCommandWord.load(std::memory_order_acquire) != 0;
    }
    [[nodiscard]] bool isPsuCommandDone(uint32_t ticket) const {
        if (ticket == 0) return false;
        return (completedPsuResults[ticket % PsuResultSlots].load(std::memory_order_acquire) >> 8)
               == ticket;
    }
    [[nodiscard]] PsuSetpointError getPsuCommandError(uint32_t ticket) const {
        if (ticket == 0) return lastPsuRequestError.load(std::memory_order_relaxed);
        const uint32_t result =
            completedPsuResults[ticket % PsuResultSlots].load(std::memory_order_acquire);
        return (result >> 8) == ticket
               ? static_cast<PsuSetpointError>(result & 0xffu)
               : PsuSetpointError::None;
    }
    bool cancelPsuCommand(uint32_t ticket) {
        uint32_t expected = (ticket << 8)
                            | (pendingPsuCommandWord.load(std::memory_order_acquire) & 0xffu);
        if ((expected >> 8) != ticket) return false;
        return pendingPsuCommandWord.compare_exchange_strong(expected, 0,
                                                             std::memory_order_acq_rel);
    }
    [[nodiscard]] float getPsuSetpoint() const {
        return publishedPsuSetpoint.load(std::memory_order_acquire);
    }
    [[nodiscard]] float getExplicitOvLimit() const {
        return explicitOvLimit.load(std::memory_order_acquire);
    }
    void setExplicitOvLimit(float v) {
        explicitOvLimit.store(v, std::memory_order_release);
    }
    [[nodiscard]] float getRequestedPsuSetpoint() const {
        const auto command = static_cast<PsuCommand>(
            pendingPsuCommandWord.load(std::memory_order_acquire) & 0xffu);
        if (command == PsuCommand::Enable)
            return pendingPsuSetpoint.load(std::memory_order_relaxed);
        if (command == PsuCommand::EnablePv)
            return pendingPvVoc.load(std::memory_order_relaxed);
        // PV-sim active: the commanded top of the curve, not the moving setpoint — `ovset`
        // validates against this, and an OV limit inside (moving-Vset, Voc) would latch.
        if (pvPubActive.load(std::memory_order_relaxed))
            return pvPubVoc.load(std::memory_order_relaxed);
        return getPsuSetpoint();
    }
    [[nodiscard]] uint16_t getManualTarget() const {
        return manualTarget.load(std::memory_order_relaxed);
    }

    // RT-CORE ONLY. Console code posts a mailbox command; this is the single writer for the
    // controller resets, trip counters, sweep/backoff state and PSU setpoint transition.
    void applyPendingPsuCommandRt(bool freshTelemetry = true) {
        const uint32_t seqBefore = psuMailboxSeq.load(std::memory_order_acquire);
        if (seqBefore & 1u) return;
        auto word = pendingPsuCommandWord.load(std::memory_order_acquire);
        if (word == 0) return;
        const auto command = static_cast<PsuCommand>(word & 0xffu);
        const uint32_t ticket = word >> 8;
        // An Enable needs the current Vin for boost feasibility. Off/manual/sweep remain
        // available during ADC loss, but stale telemetry must never be used to energize.
        const bool pvCmd = command == PsuCommand::EnablePv;
        if ((command == PsuCommand::Enable || pvCmd) && !freshTelemetry) return;
        const float requested = pvCmd ? pendingPvVoc.load(std::memory_order_relaxed)
                                      : pendingPsuSetpoint.load(std::memory_order_relaxed);
        const float pvIsc = pendingPvIsc.load(std::memory_order_relaxed);
        const float pvK = pendingPvK.load(std::memory_order_relaxed);
        const bool pvRebase = pendingPvRebase.load(std::memory_order_relaxed);
        const uint16_t manualDuty = pendingManualTarget.load(std::memory_order_relaxed);
        const int manualRect = pendingManualRect.load(std::memory_order_relaxed);
        const bool bootRequest = pendingPsuBootRequest.load(std::memory_order_relaxed);
        if (psuMailboxSeq.load(std::memory_order_acquire) != seqBefore) return;
        if (!pendingPsuCommandWord.compare_exchange_strong(word, 0, std::memory_order_acq_rel))
            return;

        if (command != PsuCommand::Enable && !pvCmd) {
            abortSweep();
            clearBackoff();
            converter.setManualRect(-1);
            psuVsetpoint = NAN;
            publishedPsuSetpoint.store(NAN, std::memory_order_release);
            pvDeactivateRt();
            psuResetTripState();
            releaseCvFloorLatch("psu off");
            lastPsuRequestError.store(PsuSetpointError::None, std::memory_order_relaxed);
            if (command == PsuCommand::DisableManual) {
                setManualTarget(manualDuty);
                converter.setManualRect(manualRect);
                if (manualDuty != 0 && !limits.reverse_current_paranoia) {
                    converter.enableSyncRect(true);
                    bflow.enable(true);
                }
                g_app.opMode = OpMode::Manual;
                ESP_LOGI("mppt", "PSU off, manual PWM mode");
            } else if (command == PsuCommand::StartSweep) {
                g_app.opMode = OpMode::Mppt;
                clearBootTarget();
                startSweep();
                ESP_LOGI("mppt", "PSU off, sweep started");
            } else if (command == PsuCommand::ShortLowSide) {
                setManualTarget(0);
                g_app.opMode = OpMode::Manual;
                converter.shortLs();
                ESP_LOGI("mppt", "PSU off, low side shorted for diagnostic");
            } else {
                g_app.opMode = OpMode::Mppt;
                clearBootTarget();
                ESP_LOGI("mppt", "PSU off, MPPT mode");
            }
            completePsuCommand(ticket, PsuSetpointError::None);
            return;
        }

        const float vin = sensors.Vin ? sensors.Vin->ewm.avg.get() : NAN;
        // For a PV curve the no-load operating point IS Voc, so Voc carries the feasibility check.
        auto error = validatePsuSetpoint(requested, limits.Vout_max, converter.boost(), vin,
                                         getExplicitOvLimit());
        if (pvCmd && error == PsuSetpointError::None && !validPvParams(pvIsc, requested, pvK))
            error = PsuSetpointError::OutOfRange;
        lastPsuRequestError.store(error, std::memory_order_relaxed);
        if (error != PsuSetpointError::None) {
            const char *why = error == PsuSetpointError::TelemetryUnavailable
                              ? "fresh Vin telemetry unavailable"
                              : error == PsuSetpointError::BoostBelowInput
                              ? "boost setpoint must exceed Vin by 0.5 V"
                              : error == PsuSetpointError::OvLimitConflict
                                ? "setpoint conflicts with explicit OV threshold"
                                : "setpoint out of range";
            ESP_LOGE("mppt", "PSU request %.2f V rejected: %s (Vin %.2f V, OV %.2f V)",
                     requested, why, vin, getExplicitOvLimit());
            if (bootRequest)
                g_app.setupErr = true;
            completePsuCommand(ticket, error);
            return;
        }
        if (pvCmd && pvK * requested < vin + 2)
            ESP_LOGW("mppt", "PV MPP %.1f V is below the boost floor (Vin %.1f V) — knee not emulatable",
                     pvK * requested, vin);
        if (pvCmd && pvSim.active) {
            // In-place curve update: keep the current setpoint (the slew limiter walks it to
            // the new curve), no controller reset — a jump to Voc under load is the excursion
            // the limiter exists to prevent. Only a deliberate full `pv <isc> <voc>` re-issue
            // (rebase set) is the unlatch escape hatch, like `psu <v>`; a `pv scale` or a
            // save/restore must not silently cancel a fault backoff or the trip history.
            if (pvRebase) {
                psuResetTripState();
                clearBackoff();
            }
            pvSim.model.set(pvIsc, requested, pvK);
            if (pvRebase) pvBaseIsc.store(pvIsc, std::memory_order_relaxed);
            publishPvState(true, pvIsc, requested, pvK);
            ESP_LOGI("mppt", "PV curve update, Isc=%.2fA Voc=%.2fV k=%.2f", pvIsc, requested, pvK);
            completePsuCommand(ticket, PsuSetpointError::None);
            return;
        }
        abortSweep();
        clearBackoff();
        converter.setManualRect(-1);
        psuResetTripState();
        if (pvCmd) {
            pvSim.model.set(pvIsc, requested, pvK);
            pvSim.iout.reset();
            pvSim.active = true;
            if (pvRebase) pvBaseIsc.store(pvIsc, std::memory_order_relaxed);
            publishPvState(true, pvIsc, requested, pvK);
            setPsuSetpoint(requested);
            g_app.opMode = OpMode::Psu;
            ESP_LOGI("mppt", "PV-sim mode, Isc=%.2fA Voc=%.2fV k=%.2f", pvIsc, requested, pvK);
        } else {
            pvDeactivateRt(); // a plain `psu <V>` while PV is active reverts to fixed CV
            setPsuSetpoint(requested);
            g_app.opMode = OpMode::Psu;
            ESP_LOGI("mppt", "PSU mode, vset=%.2fV", requested);
        }
        completePsuCommand(ticket, PsuSetpointError::None);
    }

    // RT-only, PV-sim active: advance psuVsetpoint one tick along the curve.
    // No controller reset (a per-tick reset would kill the Vout PD's derivative).
    void pvAdvanceSetpoint(float iout, float vin, float dt);

    [[nodiscard]] PsuSetpointError getLastPsuRequestError() const {
        return lastPsuRequestError.load(std::memory_order_relaxed);
    }

    [[nodiscard]] PsuSetpointError currentPsuFeasibility() const {
        const float vin = sensors.Vin ? sensors.Vin->ewm.avg.get() : NAN;
        // PV-sim: judge feasibility at the curve top (Voc), not the moving setpoint — the
        // per-tick Vin-floor clamp keeps the moving value feasible by construction, so the
        // infeasibility latch fires only when Vin rises to within the headroom of Voc.
        const float v = pvSim.active ? pvSim.model.voc : psuVsetpoint;
        return validatePsuSetpoint(v, limits.Vout_max, converter.boost(), vin,
                                   getExplicitOvLimit());
    }

    void psuResetTripState() {
        psuTripCount.store(0, std::memory_order_relaxed);
        psuTripWindowStart = 0;
        psuEscalated = false;
        psuLatched = false;
    }

    [[nodiscard]] uint8_t getPsuTripCount() const {
        return psuTripCount.load(std::memory_order_relaxed);
    }
    [[nodiscard]] bool isPsuEscalated() const {
        return psuEscalated.load(std::memory_order_relaxed);
    }
    [[nodiscard]] bool isPsuLatched() const {
        return psuLatched.load(std::memory_order_relaxed);
    }

    [[nodiscard]] float computeOvThreshold() const {
        const float ovLimit = getExplicitOvLimit();
        if (std::isfinite(ovLimit) && ovLimit > 0)
            return std::min(ovLimit, limits.Vout_max);
        // PV-sim: pin the OV band to Voc so it doesn't follow the moving setpoint down the curve.
        const float psuBase = pvSim.active ? pvSim.model.voc : psuVsetpoint;
        if (g_app.psuMode() && std::isfinite(psuBase))
            return std::min(psuBase * (limits.reverse_current_paranoia ? 1.03f : 1.5f), limits.Vout_max);
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
