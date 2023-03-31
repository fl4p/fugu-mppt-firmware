#pragma once

#include "sampling.h"


struct MpptParams {
    float Vout_max = 42;
    float Vin_max = 80;
    float Iin_max = 3;
};

class MpptSampler {
    const DCDC_PowerSampler &dcdcPwr;
    HalfBridgePwm &pwm;
    MpptParams params;

    unsigned long lastOcFastRecovery = 0;
public:

    explicit MpptSampler(const DCDC_PowerSampler &dcdcPwr, HalfBridgePwm &pwm)
            : dcdcPwr(dcdcPwr), pwm(pwm) {}

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

        if (dcdcPwr.last.s.chIin / params.Iin_max > 1.05) {
            if (dcdcPwr.last.s.chIin / params.Iin_max > 1.5) {
                pwm.disable();
                ESP_LOGW("mppt", "Current %.1f 50%% above limit, shutdown", dcdcPwr.last.s.chIin);
                return false;
            }
            auto duty = pwm.getBuckDutyCycle();
            duty -= 2;

            auto timeNow = millis();
            auto debounceLog = (timeNow - lastOcFastRecovery > 1000);
            lastOcFastRecovery = timeNow;

            if (dcdcPwr.last.s.chIin / params.Iin_max > 1.2) {
                if (debounceLog) {ESP_LOGW("mppt", "Current %.1f 20%% above limit!", dcdcPwr.last.s.chIin); }
                duty /= 2;
            } else if (debounceLog) {
                ESP_LOGW("mppt", "Current %.1f 5%% above limit!", dcdcPwr.last.s.chIin);
            }
            pwm.setBuckDutyCycle(duty);
        }


        if (dcdcPwr.last.s.chVout > dcdcPwr.last.s.chVin) {
            pwm.disable();
            return false;
        }

        return true;
    }

    void update() {
        int8_t pwmDirection = 0;

        if (dcdcPwr.last.s.chVout > params.Vout_max) {
            pwmDirection = -1;
        } else if (dcdcPwr.last.s.chIin > params.Iin_max) {
            pwmDirection = -1;
        } else {
            pwmDirection = 1;
        }

        auto duty = pwm.getBuckDutyCycle();
        if (duty + pwmDirection < 0) {
            pwmDirection = -duty;
        }

        pwm.setBuckDutyCycle(duty + pwmDirection);



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