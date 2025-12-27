#pragma once

#include "adc/sampling.h"

#include "tele/telemetry.h"
#include "util.h"

#include "temperature.h"
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
    CC,
    CP,
    MPPT,
    Sweep,
    Max,
};


static const std::array<std::string, (size_t) MpptControlMode::Max> MpptState2String{
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
    uint16_t targetDutyCycle = 0; // MPP from global scan

private:
    bool _sweeping = false; // global scan


    struct {
        float power = 0;
        float voltage = 0;
        uint16_t dutyCycle = 0;
    } maxPowerPoint; // MPP during sweep

    Plot sweepPlot{};

    unsigned long lastTimeProtectPassed = 0;
    unsigned long _lastPointWrite = 0;

    const VIinVout<const Sensor *> &sensors;


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

    uint16_t targetPwmCnt = 0;

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

    void begin(const ConfFile &trackerConf, const ConfFile &boardConf, const Limits &limits_, const TeleConf &tele_);

    [[nodiscard]] MpptControlMode getState() const { return ctrlState.mode; }
    [[nodiscard]] bool active() const { return _sweeping or sampler.isCalibrating() or !converter.disabled(); }
    [[nodiscard]] bool isSweeping() const { return _sweeping; }

    void shutdownDcdc() {
        // TODO backoff time delay
        if (topologyConfig.backflowAtHV) {
            converter.disable();
            bflow.enable(false);
        } else {
            // disabling the backflow switch first to avoid any battery current into the converter
            // solar current is usually not harmful, so shutting down the converter can wait
            bflow.enable(false); // this is very fast
            converter.disable();
        }
    }

    [[nodiscard]] float boardPowerSupplyVoltage() const {
        constexpr auto diodeFwdVoltage = 0.3f;
        return std::max(sensors.Vin->last, sensors.Vout->last) - diodeFwdVoltage;
    }

    [[nodiscard]] bool boardPowerSupplyUnderVoltage(bool start = false) const {
        if (isnan(sensors.Vin->last) || isnan(sensors.Vout->last))
            return false;
        return boardPowerSupplyVoltage() < (start ? 9.5f : 9.f);
    }

    [[nodiscard]] bool startCondition() const {
        return !(ntc.last() > limits.Temp_derate) && ucTemp.last() < limits.Temp_derate
               && (converter.boost()
                       ? sensors.Vin->ewm.avg.get() < sensors.Vout->ewm.avg.get() + 1
                       : sensors.Vin->ewm.avg.get() > sensors.Vout->ewm.avg.get() + 1)
               && !boardPowerSupplyUnderVoltage(true) && !sampler.isCalibrating();
    }

    bool protectLf(bool ignoreUV) {
        //auto nowMs = loopWallClockMs();

        // power supply under-voltage shutdown
        if (boardPowerSupplyUnderVoltage() and not ignoreUV) {
            if (!converter.disabled())
                ESP_LOGW("mppt", "Supply under-voltage! Vin %.1f and Vout %.1f < 10", sensors.Vin->last,
                     sensors.Vout->last);
            shutdownDcdc();
            enqueue_task([&] { meter.commit(); });
            return false;
        }

        // detect battery voltage
        // TODO move this to charger ?
        if (std::isnan(charger.params.Vbat_max)) {
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
            shutdownDcdc();
            return false;
        }

        // output over-voltage
        // todo introduce separate variable for reverse_current_paranoia
        auto ovTh = std::min(charger.params.Vbat_max * (limits.reverse_current_paranoia ? 1.03f : 1.5f),
                             limits.Vout_max);
        //if (adcSampler.med3.s.chVout.get() > ovTh) {
        if (sensors.Vout->last > ovTh) {
            //  && sensors.Vout->previous > ovTh * 0.9f
            bool wasDisabled = converter.disabled();
            shutdownDcdc();

            auto vout = std::max(sensors.Vout->last, sensors.Vout->previous);

            if (!wasDisabled)
                ESP_LOGW("mppt", "Vout %.1fV (prev=%.1fV,ewma=%.1fV,std=%.4f,D=%hu) > %.1fV + 5pct!",
                     sensors.Vout->last, sensors.Vout->previous,
                     sensors.Vout->ewm.avg.get(), sensors.Vout->ewm.std.get(), converter.getCtrlOnPwmCnt(),
                     charger.params.Vbat_max
            );


            if (flags.autoDetectVout_max && nowMs - lastTimeProtectPassed > 20000) {
                // if the OV condition persists for some seconds, auto-detect Vout_max
                charger.params.Vbat_max = NAN;
                sampler.startCalibration();
            }

            enqueue_task([&] {
                lcd.displayMessageF("OV shutdown!\nVout=%.1fV max=%.1fV", 10000, vout, ovTh);
            });

            return false;
        }


        // input over current
        if (sensors.Iin->last > limits.Iin_max * 1.3f && !converter.disabled()) {
            shutdownDcdc();
            ESP_LOGW("mppt", "Input current %.1f > 1.5x limit (Iout=%.1f, Vin=%.2f, Iin=%.2f), shutdown",
                     sensors.Iin->last,
                     sensors.Iout->last, sensors.Vin->last, sensors.Iin->last);
            return false;
        }

        // output over current
        if ((sensors.Iout->last > limits.Iout_max * 1.5f
             or sensors.Iout->med3.get() > limits.Iout_max * 1.25f
             or sensors.Iout->ewm.avg.get() > limits.Iout_max * 1.15f
            ) and not converter.disabled()) {
            shutdownDcdc();
            ESP_LOGW("mppt", "Output Current %.2f (med %.2f, avg %.2f) above limit %.2f, shutdown", sensors.Iout->last,
                     sensors.Iout->med3.get(), sensors.Iout->ewm.avg.get(), limits.Iout_max);
            return false;
        }

        if (sensorPhysicalI->last < -1 && sensorPhysicalI->previous < -1 && !converter.forcedPwm_()) {
            if (sensors.Iout->ewm.avg.get() > 10) {
                //buck.halfDutyCycle();
                shutdownDcdc();
                ESP_LOGE("MPPT", "Reverse current %.2f A, noise? High avg current, shutdown", sensorPhysicalI->last);
            } else {
                if (bflow.state() || converter.getRectOnPwmCnt() > converter.getRectOnPwmMin())
                    ESP_LOGE("MPPT", "Reverse current %.2f A, noise? disable BFC and low-side FET (pwm=%hu)",
                         sensorPhysicalI->last, converter.getCtrlOnPwmCnt());
                bflow.enable(false); // reverse current
                converter.syncRectMinDuty();
            }
        }

        if (sensorPhysicalI->ewm.avg.get() < -1 /*&& !converter.forcedPwm_()*/) {
            if (!converter.disabled())
                ESP_LOGE("MPPT", "Reverse avg current %.1f A, shutdown!", sensorPhysicalI->ewm.avg.get());
            shutdownDcdc();
            return false;
        }

        if (!converter.boost()) {
            if (sensors.Vout->ewm.avg.get() > (sensors.Vin->ewm.avg.get() + 1.0f) * 1.25f) {
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "Vout %.1f > Vin %.1f, shutdown duty=%i", sensors.Vout->ewm.avg.get(),
                         sensors.Vin->ewm.avg.get(), (int) converter.getCtrlOnPwmCnt());
                shutdownDcdc();
                return false;
            }
            if (sensors.Vout->last > (sensors.Vin->last + .5f) * 2) {
                ESP_LOGE("MPPT", "Vout %.1f > 2x Vin %.1f, shutdown", sensors.Vout->last, sensors.Vin->last);
                shutdownDcdc();
                return false;
            }

            // try to prevent voltage boost and disable low side for low currents
            auto currentFilt = fminf(sensorPhysicalI->ewm.avg.get(),
                                     std::max(sensorPhysicalI->last, sensorPhysicalI->previous));
            if (currentFilt < -0.05f && limits.reverse_current_paranoia) {
                if (converter.getRectOnPwmCnt() > converter.getRectOnPwmMax() / 2 &&
                    converter.getRectOnPwmCnt() > (converter.pwmRectMin + converter.pwmCtrlMax / 20)) {
                    ESP_LOGW("MPPT", "Low current, set low-side min duty (ewm(%s)=%.2f, max(i[0],i[-1])=%.2f)",
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
                ESP_LOGE("MPPT", "Output short circuit detected! (V=%.2f, I= %.1fA",
                     sensors.Vout->ewm.avg.get(), sensors.Iout->ewm.avg.get());
            shutdownDcdc();
            // TODO delay
            return false;
        }

        // if bflow switch is powered by HS gate drive, need a min duty cycle
        // TODO lift this, bflow switch will be powered from bootstrap cap and not gate drive signal
        constexpr auto BflowMinDutyCycle = 0.1f;
        if (bflow && (!bflow.state() || converter.getDutyCycle() < BflowMinDutyCycle)) {
            if (sensorPhysicalI->ewm.avg.get() > 6) {
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "High-current through open backflow switch!");
                shutdownDcdc();
                return false;
            }

            if (converter.getDutyCycle() > 0.33f) {
                // in case the current sensor is wrong
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "High duty cycle with open backflow switch!");
                shutdownDcdc();
                return false;
            }
        }

        if (!converter.syncRectEnabled_()) {
            if (sensorPhysicalI->ewm.avg.get() > 6) {
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "High current without sync rectification!");
                shutdownDcdc();
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
            and limits.reverse_current_paranoia) {
            if (!converter.disabled())
                ESP_LOGE("MPPT",
                     "Buck running at D=%d %% but Vout (%.2f, vr=%.2f) and Iout (%.2f, last=%.2f) low! Sensor or half-bridge failure.",
                     100 * converter.getCtrlOnPwmCnt() / converter.pwmCtrlMax, vOut, vr,
                     sensors.Iout->ewm.avg.get(),
                     sensors.Iout->last
            );

            shutdownDcdc();
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

    void setTargetDutyCycle(uint16_t dutyCycle) {
        if (dutyCycle == 0) converter.disable(); // no need to fade
        if (dutyCycle > converter.pwmCtrlMax) dutyCycle = converter.pwmCtrlMax;
        targetDutyCycle = dutyCycle;
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
        targetDutyCycle = maxPowerPoint.dutyCycle;

        ESP_LOGI("mppt",
                 "Stop sweep after %.2fs at controlMode=%s (limIdx=%i, tgt=%.2f, act=%.2f) PWM=%hu, MPP=(%.1fW,PWM=%hu,%.1fV)",
                 (wallClockUs() - sampler.getTimeLastCalibrationUs()) * 1e-6f,
                 MpptState2String[(uint8_t) controlMode].c_str(), limIdx,
                 limCtrl ? limCtrl->target : NAN, limCtrl ? limCtrl->actual : NAN,
                 converter.getCtrlOnPwmCnt(), maxPowerPoint.power, maxPowerPoint.dutyCycle, maxPowerPoint.voltage
        );

        enqueue_task([&] {
            lcd.displayMessageF("MPP Scan done\n%.1fW @ %.1fV", 6000, maxPowerPoint.power, maxPowerPoint.voltage);
            sweepPlot.plot();
        });
    }

    void telemetry();

    unsigned long lastUs = 0;

    void update(); // normal update
    void updateCV(); // Constant-Voltage mode
    void updateManual(); // manual mode
};
