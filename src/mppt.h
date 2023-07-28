#pragma once

#include "adc/sampling.h"
#include "telemetry.h"

#include "temperature.h"
#include "fan.h"

#include "battery.h"
#include "lcd.h"
#include "tracker.h"
#include "control.h"
#include "store.h"
#include "metering.h"
#include "charger.h"


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




/**
 * Implements
 * - Protection of DCDC converter
 * - Voltage and current control loop
 * - MPP global scan
 * - Telemetry
 */
class MpptController {
    ADC_Sampler &dcdcPwr;
    HalfBridgePwm &pwm;
    LCD &lcd;
    MpptParams params;

    bool autoDetectVout_max = true;

    MpptControlMode state = MpptControlMode::None;
    bool _limiting = false;
    bool _sweeping = false;

    struct {
        float power = 0;
        uint16_t dutyCycle = 0;
        float voltage = 0;
    } maxPowerPoint;// MPP during sweep

    unsigned long lastTimeProtectPassed = 0;
    unsigned long _lastPointWrite = 0;

    VIinVout<const ADC_Sampler::Sensor*> sensors;

    PD_Control VinController{-100, -200, true}; // Vin under-voltage
    PD_Control VoutController{150, 600, true}; // Vout over-voltage
    PD_Control IinController{100, 400, true}; // Iin over-current
    PD_Control_SmoothTarget IoutCurrentController{30, 120, 200}; // Iout over-current
    PD_Control_SmoothTarget powerController{10, 20, 200}; // over-power // TODO PID?

    Tracker tracker{};
    BatteryCharger charger;

public:
    SolarEnergyMeter meter{};
    TempSensorGPIO_NTC ntc;
    float speedScale = 1;

    explicit MpptController(ADC_Sampler &dcdcPwr, HalfBridgePwm &pwm, LCD &lcd)
            : dcdcPwr(dcdcPwr), pwm(pwm), lcd(lcd), charger{params} {

        pinMode((uint8_t) PinConfig::LED, OUTPUT);

        digitalWrite((uint8_t) PinConfig::LED, false);

        fanInit();
    }

    void setSensors(const VIinVout<const ADC_Sampler::Sensor*> &channels_) {
        sensors = channels_;
    }

    void begin() {
        meter.load();
        startSweep();
    }

    MpptControlMode getState() const { return state; }

    void shutdownDcdc() { pwm.disable(); }

    bool boardPowerSupplyUnderVoltage(bool start = false) const {
        return std::max(sensors.Vin->last, sensors.Vout->last) < (start ? 9.5f : 9.f);
    }

    bool startCondition() const {
        return ntc.last() < 70.0f
               && sensors.Vin->ewm.avg.get() > sensors.Vout->ewm.avg.get() + 1
               && !boardPowerSupplyUnderVoltage(true);
    }

    bool protect() {

        // power supply under-voltage shutdown
        if (boardPowerSupplyUnderVoltage()) {
            if (!pwm.disabled())
                ESP_LOGW("mppt", "Vin %.1f and Vout %.1f < 10", sensors.Vin->last, sensors.Vout->last);
            shutdownDcdc();
            return false;
        }

        if (dcdcPwr.isCalibrating()) {
            shutdownDcdc();
            return false;
        }

        // detect battery voltage
        // TODO move this to charger ?
        if (std::isnan(params.Vout_max)) {
            auto vout = sensors.Vout->calibrationAvg;
            float detectedVout_max = detectMaxBatteryVoltage(vout);
            if (std::isnan(detectedVout_max)) {
                ESP_LOGW("mppt", "Unable to detect battery voltage Vout=%.2fV", vout);
                pwm.disable();
                dcdcPwr.startCalibration();
                return false;
            } else {
                ESP_LOGI("mppt", "Detected max battery voltage %.2fV (from Vout=%.2fV)", detectedVout_max, vout);
                params.Vout_max = detectedVout_max;
            }
        }


        // input over-voltage
        if (sensors.Vin->last > params.Vin_max) {
            // input over-voltage
            ESP_LOGW("mppt", "Vin %.1f > %.1f!", sensors.Vin->last, params.Vin_max);
            shutdownDcdc();
            return false;
        }

        // output over-voltage
        auto ovTh = params.Vout_max * 1.08;
        //if (adcSampler.med3.s.chVout.get() > ovTh) {
        if (sensors.Vout->last > ovTh && sensors.Vout->previous > ovTh) {
            bool wasDisabled = pwm.disabled();
            shutdownDcdc();

            auto vout = std::max(sensors.Vout->last, sensors.Vout->previous);

            if (!wasDisabled)
                ESP_LOGW("mppt", "Vout %.1fV (ewma=%.1fV,std=%.4f,pwm=%hu) > %.1fV + 8%%!",
                         vout,
                         sensors.Vout->ewm.avg.get(), sensors.Vout->ewm.std.get(), pwm.getBuckDutyCycle(),
                         params.Vout_max);


            if (autoDetectVout_max && millis() - lastTimeProtectPassed > 20000) {
                // if the OV condition persists for some seconds, auto detect Vout_max
                params.Vout_max = NAN;
                dcdcPwr.startCalibration();
            }

            lcd.displayMessageF("OV shutdown!\nVout=%.1fV max=%.1fV", 10000, vout, ovTh);

            return false;
        }


        // input over current
        if (sensors.Iin->last / params.Iin_max > 1.5) {
            shutdownDcdc();
            ESP_LOGW("mppt", "Current %.1f 50%% above limit, shutdown", sensors.Iin->last);
            return false;
        }

        if (sensors.Iin->last < -1 && sensors.Iin->previous < -1) {
            ESP_LOGE("MPPT", "Reverse current %.1f A, noise? disable BFC and low-side FET", sensors.Iin->last);
            //shutdownDcdc();
            pwm.enableBackflowMosfet(false);
            pwm.lowSideMinDuty();
            //pwm.halfDutyCycle();
        }

        if (sensors.Iin->ewm.avg.get() < -1) {
            ESP_LOGE("MPPT", "Reverse avg current shutdown %.1f A", sensors.Iin->ewm.avg.get());
            shutdownDcdc();
            return false;
        }

        if (sensors.Vout->ewm.avg.get() > sensors.Vin->ewm.avg.get() * 1.25f) {
            if (!pwm.disabled())
                ESP_LOGE("MPPT", "Vout %.1f > Vin %.1f, shutdown", sensors.Vout->ewm.avg.get(),
                         sensors.Vin->ewm.avg.get());
            shutdownDcdc();
            return false;
        }

        if (sensors.Vout->last > sensors.Vin->last * 2) {
            ESP_LOGE("MPPT", "Vout %.1f > 2x Vin %.1f, shutdown", sensors.Vout->last, sensors.Vin->last);
            shutdownDcdc();
            return false;
        }

        // try to prevent voltage boost and disable low side for low currents
        if (fminf(sensors.Iin->ewm.avg.get(), std::max(sensors.Iin->last, sensors.Iin->previous)) < 0.1f) {
            if (pwm.getBuckDutyCycleLS() > pwm.getDutyCycleLSMax() / 2 &&
                pwm.getBuckDutyCycleLS() > (pwm.pwmMax / 10)) {
                ESP_LOGW("MPPT", "Set low-side min duty (ewm(Iin)=%.2f, max(Iin,Iin[-1])=%.2f)",
                         sensors.Iin->ewm.avg.get(), std::max(sensors.Iin->last, sensors.Iin->previous));
            }
            pwm.lowSideMinDuty();
            pwm.enableBackflowMosfet(false);
        }

        if (ntc.last() > 95) {
            ESP_LOGE("MPPT", "Temp %.1f°C > 95°C, shutdown", ntc.last());
            return false;
        }


        // TODO move this to control loop
        float vOut = fmaxf(sensors.Vout->med3.get(), sensors.Vout->ewm.avg.get());
        float vIn = fminf(sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get());
        pwm.updateLowSideMaxDuty(vOut, vIn);

        lastTimeProtectPassed = millis();

        return true;
    }


    /**
     * Start a global MPPT scan.
     */
    void startSweep() {
        pwm.disable();
        _limiting = false;

        VinController.reset();
        VoutController.reset();
        IinController.reset();
        IoutCurrentController.reset();

        ESP_LOGI("mppt", "Start sweep");

        dcdcPwr.startCalibration();
        //if (!_sweeping)
        _sweeping = true;
        maxPowerPoint = {};

        meter.commit(); // not real-time safe
        lcd.periodicInit(); // not real-time safe
    }


    /**
     * Stops MPPT scan and set duty cycle to captured MPP
     * @param controlMode
     */
    void _stopSweep(MpptControlMode controlMode) {
        ESP_LOGI("mppt", "Stop sweep at controlMode=%s PWM=%hu, MPP=(%.1fW,PWM=%hu,%.1fV)",
                 MpptState2String[(uint8_t) controlMode].c_str(),
                 pwm.getBuckDutyCycle(), maxPowerPoint.power, maxPowerPoint.dutyCycle, maxPowerPoint.voltage
        );
        lcd.displayMessageF("MPP Scan done\n%.1fW @ %.1fV", 6000, maxPowerPoint.power, maxPowerPoint.voltage);
        _sweeping = false;
        pwm.pwmPerturb((int16_t) maxPowerPoint.dutyCycle - (int16_t) pwm.getBuckDutyCycle()); // jump to MPP
    }

    void telemetry() {
        if (!WiFi.isConnected())
            return;

        auto Iin = sensors.Iin->ewm.avg.get();
        auto Vin = sensors.Vin->ewm.avg.get();
        auto power = Iin * Vin;

        Point point("mppt");
        point.addTag("device", "fugu_" + String(getChipId()));
        point.addField("P", power, 2);
        point.addField("I", Iin, 3);
        point.addField("U", Vin, 3);

        point.addField("E", meter.totalEnergy.get(), 1);
        point.addField("E_today", meter.dailyEnergyMeter.today.energyDay.toFloat(), 1);

        point.addField("pwm_duty", pwm.getBuckDutyCycle());
        point.addField("pwm_ls_duty", pwm.getBuckDutyCycleLS());
        point.addField("pwm_ls_max", pwm.getDutyCycleLSMax());

        auto nowMs = millis();

        point.setTime(WritePrecision::MS);

        if (nowMs - _lastPointWrite > 10) {
            telemetryAddPoint(point, 40);
            _lastPointWrite = nowMs;
        }
    }

    float getIoutSmooth(float conversionEff = 0.97f) const {
        auto pin = sensors.Iin->ewm.avg.get() * sensors.Vin->ewm.avg.get();
        return pin * conversionEff / std::max(sensors.Vout->ewm.avg.get(), 2.f);
    }

    /**
     * - Energy counter
     * - voltage and current control
     * - calls mpp tracker
     */
    void update() {
        auto nowMs = millis();
        auto nowUs = micros();

        if (pwm.disabled() && !startCondition()) {
            pwm.enableBackflowMosfet(false);
            state = MpptControlMode::None;
            return;
        }

        constexpr float conversionEfficiency = 0.97f;

        auto Iin(sensors.Iin->ewm.avg.get());
        auto Vin(sensors.Vin->ewm.avg.get());
        auto Vout(sensors.Vout->ewm.avg.get());
        float power = sensors.Iin->ewm.avg.get() * sensors.Vin->ewm.avg.get();

        //avgIin.add(adcSampler.last.s.chIin);
        //avgVin.add(adcSampler.last.s.chVin);
        //float smoothPower = avgIin.get() * avgVin.get();

        auto Iout = getIoutSmooth(conversionEfficiency);
        meter.add(sensors.Iin->last * sensors.Vin->last * conversionEfficiency, power, nowUs);


        const float ntcTemp = ntc.last();
        fanUpdateTemp(ntcTemp, power);

        float powerLimit = params.P_max;
        if (ntcTemp > 75 or std::isnan(ntcTemp)) {
            powerLimit = 300;
        } else if (ntcTemp > 80) {
            powerLimit = 200;
        } else if (ntcTemp > 90) {
            powerLimit = 20;
        }

        // topping current
        float Iout_max = charger.getToppingCurrent(Vout);

        // periodic sweep / scan
        if (!_sweeping /*&& power < 30*/ && (nowMs - dcdcPwr.getTimeLastCalibration()) > (20 * 60000)) {
            ESP_LOGI("mppt", "periodic zero-current calibration");
            startSweep();
            return;
        }

        Point point("mppt");
        point.addTag("device", "fugu_" + String(getChipId()));
        point.addField("I", Iin, 2);
        point.addField("U", Vin, 2);
        point.addField("U_out", sensors.Vout->ewm.avg.get(), 2);
        point.addField("P", power, 2);
        point.addField("E", meter.totalEnergy.get(), 1);


        struct CVP {
            MpptControlMode mode;
            PD_Control &crtl;
            struct {
                float actual, target;
            };
        };

        constexpr auto CV = MpptControlMode::CV, CC = MpptControlMode::CC, CP = MpptControlMode::CP;

        std::array<CVP, 5> controlValues{
                CVP{CV, VinController, {sensors.Vin->med3.get(), params.Vin_min}},
                CVP{CV, VoutController, {sensors.Vout->med3.get(), params.Vout_max}},
                CVP{CC, IinController, {sensors.Iin->med3.get(), params.Iin_max}},
                CVP{CC, IoutCurrentController, {Iout, Iout_max}},
                CVP{CP, powerController, {power, powerLimit}},
        };

        CVP *limitingControl = nullptr;
        float limitingControlValue = std::numeric_limits<float>::infinity();
        for (auto &c: controlValues) {
            auto cv = c.crtl.update(c.actual, c.target);
            if (cv < limitingControlValue) {
                limitingControlValue = cv;
                limitingControl = &c;
            }
        }

        //auto limitingControl = std::min_element(controlValues.begin(), controlValues.end(),
        //                                        [](const CVP &a, const CVP &b) { return a.second < b.second; });

        MpptControlMode controlMode = MpptControlMode::None;
        float controlValue = 0;

        if (limitingControlValue <= 0) {
            // limit condition
            controlMode = limitingControl->mode;
            controlValue = limitingControlValue;

            auto limIdx = (int) (limitingControl - controlValues.begin());
            if (!_limiting && controlValue < -80) {
                ESP_LOGI("mppt", "Limiting! Control value %.2f, mode=%s, idx=%i (act=%.3f, tgt=%.3f)", controlValue,
                         MpptState2String[(int) controlMode].c_str(), limIdx, limitingControl->actual,
                         limitingControl->target);
                ESP_LOGI("mppt", "Iout_max=%.2f powerLimit=%.2f", Iout_max, powerLimit);
            }

            point.addField("cv_lim_idx", limIdx);

            _limiting = true;
        } else {
            // no limit condition
            if (_limiting) {
                // recover from limit condition
                _limiting = false;
                controlMode = MpptControlMode::MPPT;
                controlValue = limitingControlValue;
            }
        }

        // bounce at pwm boundary
        if (pwm.getBuckDutyCycle() == pwm.pwmMaxHS) {
            controlMode = MpptControlMode::CV;
            controlValue = -1;
        } else if (pwm.getBuckDutyCycle() == pwm.pwmMinHS && !_sweeping) {
            controlMode = MpptControlMode::CV;
            controlValue = 1;
        }

        assert((controlMode == MpptControlMode::None) == (controlValue == 0));


        if (_sweeping) {
            if (controlMode == MpptControlMode::None) {
                controlMode = MpptControlMode::Sweep;
                controlValue = 5; // sweep speed

                // capture MPP during sweep
                if (power > maxPowerPoint.power) {
                    maxPowerPoint.power = power;
                    maxPowerPoint.dutyCycle = pwm.getBuckDutyCycle();
                    maxPowerPoint.voltage = Vin;
                }
            } else {
                _stopSweep(controlMode);
            }
        }

        //
        if (controlMode == MpptControlMode::None) {
            controlMode = MpptControlMode::MPPT;
            controlValue = tracker.update(power, pwm.getBuckDutyCycle());
            controlValue *= speedScale;

            auto dP = tracker.dP;
            point.addField("P_prev", tracker._lastPower, 2);
            point.addField("dP", dP, 2);
            //point.addField("P_filt", tracker.pwmPowerTable[pwm.getBuckDutyCycle()].get(), 1);
            point.addField("P_filt", tracker._powerBuf.getMean(), 1);
            if (std::abs(dP) < tracker.minPowerStep) {
                point.addField("dP_thres", 0.0f, 2);
            } else {
                point.addField("dP_thres", dP, 2);
            }
        } else {
            tracker.resetTracker(power, controlValue > 0);
        }

        //assert(controlValue != 0 && controlMode != MpptControlMode::None);

        controlValue = constrain(controlValue, -(float) pwm.getBuckDutyCycle(), 5.0f);

        this->state = controlMode;


        pwm.pwmPerturbFractional(controlValue);

        point.addField("pwm_dir_f", controlValue, 2);
        // point.addField("pwm_dir", pwmDirection);

        pwm.enableBackflowMosfet((Iin > 0.2f));
        pwm.enableLowSide((Iin > 0.2f));


        bool ledState = (Iin > 0.2f && controlMode == MpptControlMode::MPPT && controlValue > 0);
        digitalWrite((uint8_t) PinConfig::LED, ledState);


        point.addField("mppt_state", int(controlMode));
        // point.addField("mcu_temp", mcu_temp.last(), 1);
        point.addField("ntc_temp", ntcTemp, 1);
        point.addField("pwm_duty", pwm.getBuckDutyCycle());
        point.addField("pwm_ls_duty", pwm.getBuckDutyCycleLS());
        point.addField("pwm_ls_max", pwm.getDutyCycleLSMax());

        point.setTime(WritePrecision::MS);

        if (nowMs - _lastPointWrite > 10) {
            telemetryAddPoint(point, 40);
            _lastPointWrite = nowMs;
        }
    }
};