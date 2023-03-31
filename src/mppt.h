#pragma once

#include "sampling.h"


struct MpptParams {
    float Vout_max = 42;
    float Vin_max = 80;
    float Iin_max = 3;
};

enum MpptState :uint8_t {
    None = 0,
    CV,
    CC,
    MPPT,
};

class MpptSampler {
    const DCDC_PowerSampler &dcdcPwr;
    HalfBridgePwm &pwm;
    MpptParams params;

    unsigned long lastOcFastRecovery = 0;
    float lastPower = 0;
    float lastVoltage = 0;
    MpptState state = MpptState::None;
public:

    explicit MpptSampler(const DCDC_PowerSampler &dcdcPwr, HalfBridgePwm &pwm)
            : dcdcPwr(dcdcPwr), pwm(pwm) {}

    MpptState getState() const {return state;}
    float getPower() const {return lastPower;}

    bool protect() {
        if (dcdcPwr.last.s.chVin > params.Vin_max) {
            ESP_LOGW("mppt", "Vin %f > %f!", dcdcPwr.last.s.chVin, params.Vin_max);
            pwm.disable();
            return false;
        }

        if (dcdcPwr.last.s.chVout > params.Vout_max * 1.05) {
            ESP_LOGW("mppt", "Vout %f > %f + 5%%!", dcdcPwr.last.s.chVout, params.Vout_max);
            pwm.disable();
            return false;
        }

        if (dcdcPwr.last.s.chIin / params.Iin_max > 1.2) {
            if (dcdcPwr.last.s.chIin / params.Iin_max > 1.5) {
                pwm.disable();
                ESP_LOGW("mppt", "Current %.1f 50%% above limit, shutdown", dcdcPwr.last.s.chIin);
                return false;
            }
            ESP_LOGW("mppt", "Current %.1f 20%% above limit!", dcdcPwr.last.s.chIin);
            pwm.halfDutyCycle();
        }


        if (dcdcPwr.ewm.s.chIin.avg.get() / params.Iin_max > 1.05) {
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
            pwm.setLowSideDutyMin();
        }

        return true;
    }

    void update() {
        int8_t pwmDirection = 0;

        float voltage = dcdcPwr.ewm.s.chVin.avg.get();
        float power = dcdcPwr.ewm.s.chIin.avg.get() * voltage;

        if (dcdcPwr.last.s.chVout > params.Vout_max) {
            pwmDirection = -1;
            state = MpptState::CV;
        } else if (dcdcPwr.last.s.chIin > params.Iin_max) {
            pwmDirection = -1;
            state = MpptState::CC;
        } else {
            if(power < 20.0f) {
                pwmDirection = 1;
            } else if ((power - lastPower > 0) == (voltage - lastVoltage > 0)) {
                pwmDirection = -1; // decrease Vin
            } else {
                pwmDirection = 1; // increase Vin
            }
            state = MpptState::MPPT;
        }

        lastPower = power;
        lastVoltage = voltage;

        pwm.pwmPerturb(pwmDirection);



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