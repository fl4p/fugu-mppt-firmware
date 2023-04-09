#pragma once

#include "sampling.h"
#include "telemetry.h"

struct MpptParams {
    float Vout_max = 29;
    float Vin_max = 70;
    float Iin_max = 4;
};

enum MpptState :uint8_t {
    None = 0,
    CV,
    CC,
    MPPT,
};

static const std::array<std::string, 4> MpptState2String {"N/A", "CV", "CC", "MPPT"};

class MpptSampler {
    const DCDC_PowerSampler &dcdcPwr;
    HalfBridgePwm &pwm;
    MpptParams params;

    unsigned long lastOcFastRecovery = 0;
    float lastPower = 0;
    //float lastVoltage = 0;
    MpptState state = MpptState::None;
public:

    explicit MpptSampler(const DCDC_PowerSampler &dcdcPwr, HalfBridgePwm &pwm)
            : dcdcPwr(dcdcPwr), pwm(pwm) {}

    MpptState getState() const {return state;}
    float getPower() const {return lastPower;}

    bool protect() {
        if(dcdcPwr.isCalibrating()) {
            pwm.disable();
            return false;
        }

        if (dcdcPwr.last.s.chVin > params.Vin_max) {
            // input over-voltage
            ESP_LOGW("mppt", "Vin %f > %f!", dcdcPwr.last.s.chVin, params.Vin_max);
            pwm.disable();
            return false;
        }

        if (dcdcPwr.last.s.chVout > params.Vout_max * 1.05) {
            // output over-voltage
            ESP_LOGW("mppt", "Vout %f > %f + 5%%!", dcdcPwr.last.s.chVout, params.Vout_max);
            pwm.disable();
            return false;
        }

        if (dcdcPwr.last.s.chIin / params.Iin_max > 1.2) {
            // input over current
            if (dcdcPwr.last.s.chIin / params.Iin_max > 1.5) {
                pwm.disable();
                ESP_LOGW("mppt", "Current %.1f 50%% above limit, shutdown", dcdcPwr.last.s.chIin);
                return false;
            }
            ESP_LOGW("mppt", "Current %.1f 20%% above limit!", dcdcPwr.last.s.chIin);
            pwm.halfDutyCycle();
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


        if (dcdcPwr.last.s.chVout > dcdcPwr.last.s.chVin) {
            pwm.disable();
            return false;
        }

        // try to prevent voltage boost and disable low side for low currents
        if (fminf(dcdcPwr.ewm.s.chIin.avg.get(), dcdcPwr.last.s.chIin) < 0.2f) {
            pwm.lowSideMinDuty();
        }

        float voltageRatio = fmaxf(dcdcPwr.last.s.chVout,dcdcPwr.ewm.s.chVout.avg.get()) / dcdcPwr.last.s.chVin;
        pwm.updateLowSideMaxDuty(voltageRatio);

        return true;
    }

    int8_t pwmDirection = 0;

    void update() {
        //float voltage = pwm.getBuckDutyCycle(); // #dcdcPwr.ewm.s.chVin.avg.get(); // todo rename pwm
        auto Iin(dcdcPwr.ewm.s.chIin.avg.get());
        auto Vin ( dcdcPwr.ewm.s.chVin.avg.get());
        float power = dcdcPwr.ewm.s.chIin.avg.get() * dcdcPwr.ewm.s.chVin.avg.get();
        //float power = dcdcPwr.last.s.chIin * dcdcPwr.last.s.chVin;

        Point point("mppt");
        point.addTag("device", "esp32_proto_mppt");
        point.addField("I", Iin, 2);
        point.addField("I_raw", dcdcPwr.last.s.chIin, 2);
        point.addField("U", Vin, 2);
        point.addField("U_raw", dcdcPwr.last.s.chVin, 2);
        point.addField("P", power, 2);
        point.addField("P_prev", lastPower, 2);
        point.addField("P_raw", dcdcPwr.last.s.chIin * dcdcPwr.last.s.chVin, 2);

        if (dcdcPwr.last.s.chVout > params.Vout_max) {
            pwmDirection = -1;
            state = MpptState::CV;
        } else if (dcdcPwr.last.s.chIin > params.Iin_max) {
            pwmDirection = -1;
            state = MpptState::CC;
        } else {
            auto dP = power - lastPower;
            point.addField("dP", dP, 2);
            if(power < 1.f) {
                pwmDirection = 1;
            }
            if (std::abs(dP) < 1.5f) {
                // insignificant power change / noise
                // do nothing
                point.addField("dP_thres", 0.0f, 2);
                // ESP_LOGI("MPPT", "abs(dP)<1 DIR=%hhi", pwmDirection);
            } else {
                point.addField("dP_thres", dP, 2);
                if(dP >= 0) {
                    // power is increasing, we are on the right track
                    ESP_LOGI("MPPT", "dP=%.7f DIR=%hhi", dP, pwmDirection);
                } else {
                    ESP_LOGI("MPPT", "dP=%.7f DIR=%hhi REVERSE", dP, pwmDirection);
                    //pwmDirection *= -1;
                    pwmDirection = 1;
                }
                lastPower = power;
            }
            /*
            if(power < 20.0f) {
                pwmDirection = 1;
            } else if ((power - lastPower > 0) == (voltage - lastVoltage > 0)) {
                pwmDirection = -1; // decrease Vin
            } else {
                pwmDirection = 1; // increase Vin
            }
             */
            state = MpptState::MPPT;
        }

        //lastVoltage = voltage;

        pwm.pwmPerturb(pwmDirection);

        point.addField("pwm_dir", pwmDirection);
        point.addField("pwm_duty", pwm.getBuckDutyCycle());
        point.setTime(WritePrecision::MS);

        telemetryAddPoint(point);




        // TODO limit chIout
        // switch off dcdc if power < min_power
        // V_in < limit
        // temperature
        // output power < limit
        // output current < limit


        if (dcdcPwr.last.s.chIin < -1) {
            ESP_LOGE("MPPT", "Reverse current shutdown");
            pwm.disable();
        }
    }

};