#pragma once

#include "sampling.h"
#include "telemetry.h"

#include "temperature.h"
#include "fan.h"

#include "battery.h"

struct MpptParams {
    float Vout_max = NAN; //14.6 * 2;
    float Vin_max = 80;
    float Iin_max = 30;
    float Iout_max = 32;
    float P_max = 800;
};

enum class MpptState : uint8_t {
    None = 0,
    CV,
    CC,
    CP,
    MPPT,
    Sweep,
    Max,
};

static const std::array<std::string, (size_t) MpptState::Max> MpptState2String{
        "N/A",
        "CV",
        "CC",
        "CP",
        "MPPT",
        "SWEEP"
};


class MpptSampler {
    DCDC_PowerSampler &dcdcPwr;
    HalfBridgePwm &pwm;
    MpptParams params;

    bool autoDetectVout_max = true;

    unsigned long lastOcFastRecovery = 0;
    float lastPower = 0;
    //float lastVoltage = 0;
    int8_t pwmDirection = 0;

    MpptState state;

    bool _sweeping = false;
    struct {
        float power = 0;
        uint16_t dutyCycle = 0;
        float voltage = 0;
    } maxPowerPoint;

    unsigned long nextUpdateTime = 0;
    unsigned long lastTimeProtectPassed = 0;

    //unsigned long lastTimeRandomPerturb = 0;




public:
    // Esp32TempSensor mcu_temp; // don't use this! poor real-time performance!
    TempSensorGPIO_NTC ntc;
    float Iout;

    explicit MpptSampler(DCDC_PowerSampler &dcdcPwr, HalfBridgePwm &pwm)
            : dcdcPwr(dcdcPwr), pwm(pwm) {
        pinMode((uint8_t) PinConfig::LED, OUTPUT);
        digitalWrite((uint8_t) PinConfig::LED, false);

        fanInit();
    }

    MpptState getState() const { return state; }

    float getPower() const { return lastPower; }

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
        if (dcdcPwr.last.s.chVout > ovTh && dcdcPwr.previous.s.chVout > ovTh) {
            // output over-voltage
            ESP_LOGW("mppt", "Vout %.1fV (ewma=%.1fV,std=%.4f,pwm=%hu,dir=%.1hhi) > %.1fV + 8%%!",
                     std::max(dcdcPwr.last.s.chVout, dcdcPwr.previous.s.chVout),
                     dcdcPwr.ewm.s.chVout.avg.get(), dcdcPwr.ewm.s.chVout.std.get(), pwm.getBuckDutyCycle(),
                     pwmDirection,
                     params.Vout_max);
            pwm.disable();

            if (autoDetectVout_max && millis() - lastTimeProtectPassed > 4000) {
                // if the OV condition persists for some seconds, auto detect Vout_max
                params.Vout_max = NAN;
                dcdcPwr.startCalibration();
            }

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

        if (dcdcPwr.ewm.s.chVout.avg.get() > dcdcPwr.ewm.s.chVin.avg.get()) {
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
        if (fminf(dcdcPwr.ewm.s.chIin.avg.get(), std::max(dcdcPwr.last.s.chIin, dcdcPwr.previous.s.chIin)) < 0.2f) {
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

        float voltageRatio = fmaxf(dcdcPwr.last.s.chVout, dcdcPwr.ewm.s.chVout.avg.get()) / dcdcPwr.last.s.chVin;
        //float voltageRatio = dcdcPwr.ewm.s.chVout.avg.get() / dcdcPwr.ewm.s.chVin.avg.get();
        pwm.updateLowSideMaxDuty(voltageRatio);

        lastTimeProtectPassed = millis();

        return true;
    }


    void startSweep() {
        pwm.disable();
        dcdcPwr.startCalibration();
        if (!_sweeping)
            ESP_LOGI("mppt", "Start sweep");
        _sweeping = true;
        maxPowerPoint = {};
    }

    bool isSweeping() const { return _sweeping; }


    void update(bool dryRun) {
        auto nowMs = millis();

        if (nowMs < nextUpdateTime)
            return;

        //float voltage = pwm.getBuckDutyCycle(); // #dcdcPwr.ewm.s.chVin.avg.get(); // todo rename pwm
        auto Iin(dcdcPwr.ewm.s.chIin.avg.get());
        auto Vin(dcdcPwr.ewm.s.chVin.avg.get());
        float power = dcdcPwr.ewm.s.chIin.avg.get() * dcdcPwr.ewm.s.chVin.avg.get();
        float ntcTemp = ntc.read();
        // mcu_temp.read();
        //float power = dcdcPwr.last.s.chIin * dcdcPwr.last.s.chVin;

        fanUpdateTemp( ntcTemp, power);

        float powerLimit = params.P_max;
        if (ntcTemp > 75 or std::isnan(ntcTemp)) {
            powerLimit = 300;
        } else if (ntcTemp > 80) {
            powerLimit = 200;
        } else if (ntcTemp > 90) {
            powerLimit = 20;
        }

        Point point("mppt");
        point.addTag("device", "fugu_" + String(getChipId()));
        point.addField("I", Iin, 2);
        //point.addField("I_raw", dcdcPwr.last.s.chIin, 2);

        point.addField("U", Vin, 2);
        //point.addField("U_in_raw", dcdcPwr.last.s.chVin, 2);

        point.addField("U_out", dcdcPwr.ewm.s.chVout.avg.get(), 2);
        //point.addField("U_out_raw", dcdcPwr.last.s.chVout, 2);

        point.addField("P", power, 2);
        point.addField("P_prev", lastPower, 2);
        //point.addField("P_raw", dcdcPwr.last.s.chIin * dcdcPwr.last.s.chVin, 2);

        MpptState state = MpptState::MPPT;

        float conversionEff = 0.97f;
        Iout = dcdcPwr.getIoutSmooth();

        if (dcdcPwr.last.s.chVout >= params.Vout_max) {
            pwmDirection = -8;
            if (dcdcPwr.last.s.chVout >= params.Vout_max * 1.01)
                pwmDirection = -16;
            state = MpptState::CV;
        } else if (dcdcPwr.last.s.chIin > params.Iin_max or Iout > params.Iout_max) {
            pwmDirection = -1;
            if (dcdcPwr.last.s.chIin / params.Iin_max > 1.2)
                pwmDirection = -16;
            state = MpptState::CC;
        } else if (power > powerLimit) {
            pwmDirection = -1;
            state = MpptState::CP;
            /*} else if (power < 1.f) {
                if (!dryRun)
                    startSweep();
                pwmDirection = 1;
                state = MpptState::Sweep; */
        } else if (_sweeping) {
            pwmDirection = 1;
            if (power > maxPowerPoint.power) {
                // capture MPP during sweep
                maxPowerPoint.power = power;
                maxPowerPoint.dutyCycle = pwm.getBuckDutyCycle();
                maxPowerPoint.voltage = Vin;
            }
            state = MpptState::Sweep;
        }

        this->state = state;

        if (!dryRun && _sweeping && (state != MpptState::Sweep || pwm.getBuckDutyCycle() == pwm.pwmMaxHS)) {
            ESP_LOGI("mppt", "Stop sweep at state=%s PWM=%hu, MPP=(%.1fW,PWM=%hu)",
                     MpptState2String[(uint8_t) state].c_str(),
                     pwm.getBuckDutyCycle(), maxPowerPoint.power, maxPowerPoint.dutyCycle
            );

            _sweeping = false;
        }

        if (state != MpptState::MPPT) {
            lastPower = power;
        } else {
            auto dP = power - lastPower;
            point.addField("dP", dP, 2);

            if (std::abs(dP) < 1.5f) {
                // insignificant power change / noise, do nothing
                point.addField("dP_thres", 0.0f, 2);
            } else {
                point.addField("dP_thres", dP, 2);
                if (dP >= 0) {
                    // power is increasing, we are on the right track
                } else {
                    pwmDirection *= -1;
                }
                lastPower = power;
            }
        }

        if (pwmDirection > 2)
            pwmDirection = 2;

        if (!dryRun && pwmDirection == 0)
            pwmDirection = 1;

        /*if (false && state == MpptState::MPPT && nowMs - lastTimeRandomPerturb > 60000 && power > 4 &&
            (power / powerLimit) < .9f) {
            pwmDirection = -25; //pwmDirection > 0 ? 20 : -20;
            ESP_LOGI("mppt", "Random perturb %hhi", pwmDirection);
            lastTimeRandomPerturb = nowMs;
        }
         */

        float speedScale = 1.0f;

        if (!_sweeping)
            speedScale *= 0.25;

        float vOut_pred =
                dcdcPwr.last.s.chVout + std::max<float>(0.0f, dcdcPwr.last.s.chVout - dcdcPwr.ewm.s.chVout.avg.get());
        if (vOut_pred >= params.Vout_max * 0.95f) {
            speedScale *= 0.5;
            if (vOut_pred >= params.Vout_max * 0.99f) {
                speedScale *= 0.5;
            }
        }
        point.addField("U_out_pred", vOut_pred, 2);

        // slow down on low power (and high output impedance?), to reduce ripple
        if (power < 0.8) speedScale *= (1 / 4.0f);

        point.addField("speed_scale", speedScale, 2);


        if (!dryRun) {
            pwm.pwmPerturb(pwmDirection);

            //point.addField("pwm_dir_f", pwmDirectionScaled, 2);
            point.addField("pwm_dir", pwmDirection);

            // "bounce" bottom
            if (pwmDirection <= 0 && pwm.getBuckDutyCycle() <= pwm.pwmMinLS + 1) {
                pwmDirection = 1;
            }

            // bounce top
            if (pwmDirection >= 0 && pwm.getBuckDutyCycle() == pwm.pwmMaxHS) {
                pwmDirection = -1;
            }

            pwm.enableBackflowMosfet((Iin > 0.2f));
        }

        bool ledState = (!dryRun && Iin > 0.2f && state == MpptState::MPPT && pwmDirection > 0);
        digitalWrite((uint8_t) PinConfig::LED, ledState);


        point.addField("mppt_state", int(state));
        // point.addField("mcu_temp", mcu_temp.last(), 1);
        point.addField("ntc_temp", ntcTemp, 1);
        point.addField("pwm_duty", pwm.getBuckDutyCycle());
        point.addField("pwm_ls_duty", pwm.getBuckDutyCycleLS());
        point.setTime(WritePrecision::MS);
        telemetryAddPoint(point, 40);


        nextUpdateTime = nowMs + (unsigned long) (20.f / speedScale);
    }

};