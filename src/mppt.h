#pragma once

#include "adc/sampling.h"
#include "telemetry.h"

#include "temperature.h"
#include "fan.h"

#include "battery.h"
#include "lcd.h"
#include "tracker.h"
#include "control.h"

#ifndef FUGU_BAT_V
#define FUGU_BAT_V NAN
#endif

struct MpptParams {
    float Vout_max = FUGU_BAT_V; //14.6 * 2;
    float Vin_max = 80;
    float Iin_max = 30;
    float Iout_max = 27; // coil & fuse limited
    float P_max = 800;
    float Vin_min = 10.5f;
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
class MpptSampler {
    DCDC_PowerSampler &dcdcPwr;
    HalfBridgePwm &pwm;
    LCD &lcd;
    MpptParams params;

    bool autoDetectVout_max = true;

    unsigned long lastOcFastRecovery = 0;

    MpptControlMode state;
    bool _limiting = false;

    TrapezoidalIntegrator<float, unsigned long, double> _energy{1e-6f / 3600.f};

    bool _sweeping = false;
    struct {
        float power = 0;
        uint16_t dutyCycle = 0;
        float voltage = 0;
    } maxPowerPoint;

    unsigned long lastTimeProtectPassed = 0;
    unsigned long _lastPointWrite = 0;

    PD VinController{-100, -200, true}; // Vin under-voltage
    PD VoutController{150, 600, true}; // Vout over-voltage
    PD IinController{100, 400, true}; // Iin over-current
    PD IoutCurrentController{100, 400, true}; // Iout over-current
    PD powerController{100, 400, true}; // over-power


    Tracker tracker{};


public:
    TempSensorGPIO_NTC ntc;
    float Iout;
    float speedScale = 1;

    explicit MpptSampler(DCDC_PowerSampler &dcdcPwr, HalfBridgePwm &pwm, LCD &lcd)
            : dcdcPwr(dcdcPwr), pwm(pwm), lcd(lcd) {

        pinMode((uint8_t) PinConfig::LED, OUTPUT);

        digitalWrite((uint8_t) PinConfig::LED, false);

        fanInit();
    }

    MpptControlMode getState() const { return state; }

    bool protect() {
        if (dcdcPwr.isCalibrating()) {
            pwm.disable();
            return false;
        }

        // detect battery voltage
        if (std::isnan(params.Vout_max)) {
            auto vout = dcdcPwr.getBatteryIdleVoltage();
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

        if (std::max(dcdcPwr.last.s.chVin, dcdcPwr.last.s.chVout) < 10.f) {
            ESP_LOGW("mppt", "Vin %.1f and Vout %.1f < 10", dcdcPwr.last.s.chVin, dcdcPwr.last.s.chVout);
            //pwm.disable();
            startSweep();
            return false;
        }


        if (dcdcPwr.last.s.chVin > params.Vin_max) {
            // input over-voltage
            ESP_LOGW("mppt", "Vin %.1f > %.1f!", dcdcPwr.last.s.chVin, params.Vin_max);
            pwm.disable();
            return false;
        }

        // TODO shutdown on too samples in a row > 5%
        auto ovTh = params.Vout_max * 1.08;
        //if (dcdcPwr.med3.s.chVout.get() > ovTh) {
        if (dcdcPwr.last.s.chVout > ovTh && dcdcPwr.previous.s.chVout > ovTh) {
            bool wasDisabled = pwm.disabled();
            pwm.disable();

            auto vout = std::max(dcdcPwr.last.s.chVout, dcdcPwr.previous.s.chVout);
            // output over-voltage
            if (!wasDisabled)
                ESP_LOGW("mppt", "Vout %.1fV (ewma=%.1fV,std=%.4f,pwm=%hu) > %.1fV + 8%%!",
                         vout,
                         dcdcPwr.ewm.s.chVout.avg.get(), dcdcPwr.ewm.s.chVout.std.get(), pwm.getBuckDutyCycle(),
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
        if (dcdcPwr.last.s.chIin / params.Iin_max > 1.5) {
            pwm.disable();
            ESP_LOGW("mppt", "Current %.1f 50%% above limit, shutdown", dcdcPwr.last.s.chIin);
            return false;
        }

        if (dcdcPwr.last.s.chIin < -1 && dcdcPwr.previous.s.chIin < -1) {
            ESP_LOGE("MPPT", "Reverse current %.1f A, noise? disable BFC and low-side FET", dcdcPwr.last.s.chIin);
            //pwm.disable();
            pwm.enableBackflowMosfet(false);
            pwm.lowSideMinDuty();
            //pwm.halfDutyCycle();
        }

        if (dcdcPwr.ewm.s.chIin.avg.get() < -1) {
            ESP_LOGE("MPPT", "Reverse avg current shutdown %.1f A", dcdcPwr.ewm.s.chIin.avg.get());
            pwm.disable();
            return false;
        }

        if (dcdcPwr.ewm.s.chVout.avg.get() > dcdcPwr.ewm.s.chVin.avg.get() * 1.25f) {
            if (!pwm.disabled())
                ESP_LOGE("MPPT", "Vout %.1f > Vin %.1f, shutdown", dcdcPwr.ewm.s.chVout.avg.get(),
                         dcdcPwr.ewm.s.chVin.avg.get());
            pwm.disable();
            return false;
        }

        if (dcdcPwr.last.s.chVout > dcdcPwr.last.s.chVin * 2) {
            ESP_LOGE("MPPT", "Vout %.1f > 2x Vin %.1f, shutdown", dcdcPwr.last.s.chVout, dcdcPwr.last.s.chVin);
            pwm.disable();
            return false;
        }

        // try to prevent voltage boost and disable low side for low currents
        if (fminf(dcdcPwr.ewm.s.chIin.avg.get(), std::max(dcdcPwr.last.s.chIin, dcdcPwr.previous.s.chIin)) < 0.1f) {
            if (pwm.getBuckDutyCycleLS() > pwm.getDutyCycleLSMax() / 2 &&
                pwm.getBuckDutyCycleLS() > (pwm.pwmMax / 10)) {
                ESP_LOGW("MPPT", "Set low-side min duty (ewm(Iin)=%.2f, max(Iin,Iin[-1])=%.2f)",
                         dcdcPwr.ewm.s.chIin.avg.get(), std::max(dcdcPwr.last.s.chIin, dcdcPwr.previous.s.chIin));
            }
            pwm.lowSideMinDuty();
            pwm.enableBackflowMosfet(false);
        }

        if (dcdcPwr.ewm.s.chIin.avg.get() / params.Iin_max > 1.05) {
            // input over current (soft)
            auto timeNow = millis();
            auto debounceLog = (timeNow - lastOcFastRecovery > 1000);
            if (debounceLog) {
                ESP_LOGW("mppt", "Smooth current %.1f 5%% above limit!", dcdcPwr.ewm.s.chIin.avg.get());
                lastOcFastRecovery = timeNow;
            }
            pwm.pwmPerturb(-2);
        }

        if (ntc.read() > 95) {
            ESP_LOGE("MPPT", "Temp %.1f°C > 95°C, shutdown", ntc.last());
            return false;
        }


        float vOut = fmaxf(dcdcPwr.med3.s.chVout.get(), dcdcPwr.ewm.s.chVout.avg.get());
        float vIn = fminf(dcdcPwr.med3.s.chVin.get(), dcdcPwr.ewm.s.chVin.avg.get());

        pwm.updateLowSideMaxDuty(vOut, vIn);

        lastTimeProtectPassed = millis();

        return true;
    }

    bool startCondition() {
        return dcdcPwr.ewm.s.chVin.avg.get() > dcdcPwr.ewm.s.chVout.avg.get() + 1
               && ntc.read() < 70.0f;
    }

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

        lcd.periodicInit();
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
        pwm.pwmPerturb((int16_t) maxPowerPoint.dutyCycle - (int16_t) pwm.getBuckDutyCycle());
    }

    void telemetry() {
        auto Iin = dcdcPwr.ewm.s.chIin.avg.get();
        auto Vin = dcdcPwr.ewm.s.chVin.avg.get();
        auto power = Iin * Vin;

        Point point("mppt");
        point.addTag("device", "fugu_" + String(getChipId()));
        point.addField("P", power, 2);
        point.addField("I", Iin, 3);
        point.addField("U", Vin, 3);

        point.addField("E", _energy.get(), 1);

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

    //EWMA<float> avgIin{200}, avgVin{200};

    /**
     * - Energy counter
     * - voltage and current control
     * - calls mpp tracker
     */
    void update() {
        auto nowMs = millis();

        if (pwm.disabled() && !startCondition()) {
            pwm.enableBackflowMosfet(false);
            state = MpptControlMode::None;
            return;
        }


        float conversionEfficiency = 0.97f;
        auto Iin(dcdcPwr.ewm.s.chIin.avg.get());
        auto Vin(dcdcPwr.ewm.s.chVin.avg.get());
        float power = dcdcPwr.ewm.s.chIin.avg.get() * dcdcPwr.ewm.s.chVin.avg.get();

        //avgIin.add(dcdcPwr.last.s.chIin);
        //avgVin.add(dcdcPwr.last.s.chVin);
        //float smoothPower = avgIin.get() * avgVin.get();

        Iout = dcdcPwr.getIoutSmooth(conversionEfficiency);
        _energy.add(dcdcPwr.last.s.chIin * dcdcPwr.last.s.chVin * conversionEfficiency, micros());

        float ntcTemp = ntc.last();
        fanUpdateTemp(ntcTemp, power);

        float powerLimit = params.P_max;
        if (ntcTemp > 75 or std::isnan(ntcTemp)) {
            powerLimit = 300;
        } else if (ntcTemp > 80) {
            powerLimit = 200;
        } else if (ntcTemp > 90) {
            powerLimit = 20;
        }

        // periodic sweep
        if (!_sweeping /*&& power < 30*/ && (nowMs - dcdcPwr.getTimeLastCalibration()) > (15 * 60000)) {
            ESP_LOGI("mppt", "periodic zero-current calibration");
            startSweep();
            return;
        }

        Point point("mppt");
        point.addTag("device", "fugu_" + String(getChipId()));
        point.addField("I", Iin, 2);
        point.addField("U", Vin, 2);
        point.addField("U_out", dcdcPwr.ewm.s.chVout.avg.get(), 2);
        point.addField("P", power, 2);
        point.addField("E", _energy.get(), 1);


        float uCV_in = VinController.update(dcdcPwr.med3.s.chVin.get(), params.Vin_min);
        float uCV_out = VoutController.update(dcdcPwr.med3.s.chVout.get(), params.Vout_max);
        float uCC_in = IinController.update(dcdcPwr.med3.s.chIin.get(), params.Iin_max);
        float uCC_out = IoutCurrentController.update(Iout, params.Iout_max);
        float uCP = powerController.update(power, powerLimit);

        typedef std::pair<MpptControlMode, float> CVP;

        std::array<CVP, 5> controlValues{
                CVP{MpptControlMode::CV, uCV_in},
                CVP{MpptControlMode::CV, uCV_out},
                CVP{MpptControlMode::CC, uCC_in},
                CVP{MpptControlMode::CC, uCC_out},
                CVP{MpptControlMode::CP, uCP},
        };

        auto limitingControl = std::min_element(controlValues.begin(), controlValues.end(),
                                                [](const CVP &a, const CVP &b) { return a.second < b.second; });

        MpptControlMode controlMode = MpptControlMode::None;
        float controlValue = 0;

        if (limitingControl->second <= 0) {
            // limit condition
            controlMode = limitingControl->first;
            controlValue = limitingControl->second;

            auto limIdx = (int) (limitingControl - controlValues.begin());
            if (!_limiting && controlValue < -80) {
                ESP_LOGI("mppt", "Limiting! Control value %.2f, mode=%s, idx=%i", controlValue,
                         MpptState2String[(int) controlMode].c_str(), limIdx);
            }

            point.addField("cv_lim_idx", limIdx);

            _limiting = true;
        } else {
            // no limit condition
            if (_limiting) {
                // recover from limit condition
                _limiting = false;
                controlMode = MpptControlMode::MPPT;
                controlValue = limitingControl->second;
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
                controlValue = 4; // 3x sweep speed

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

        controlValue = constrain(controlValue, -(float) pwm.getBuckDutyCycle(), 4.0f);

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

        // nextUpdateTime = nowMs + (unsigned long) (20.f / speedScale);
    }
};