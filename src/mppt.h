#pragma once

#include "sampling.h"
#include "telemetry.h"

struct MpptParams {
    float Vout_max = 14.3 * 2;
    float Vin_max = 80;
    float Iin_max = 20;
    float P_max = 550;
};

enum MpptState : uint8_t {
    None = 0,
    CV,
    CC,
    CP,
    MPPT,
    Sweep,
    Max,
};

static const std::array<std::string, MpptState::Max> MpptState2String{"N/A", "CV", "CC", "CP", "MPPT", "SWEEP"};

class MpptSampler {
    const DCDC_PowerSampler &dcdcPwr;
    HalfBridgePwm &pwm;
    MpptParams params;

    unsigned long lastOcFastRecovery = 0;
    float lastPower = 0;
    //float lastVoltage = 0;
    MpptState state = MpptState::None;
    float pwmDirection = 0;

    bool _sweeping = false;
public:

    explicit MpptSampler(const DCDC_PowerSampler &dcdcPwr, HalfBridgePwm &pwm)
            : dcdcPwr(dcdcPwr), pwm(pwm) {}

    MpptState getState() const { return state; }

    float getPower() const { return lastPower; }

    bool protect() {
        if (dcdcPwr.isCalibrating()) {
            pwm.disable();
            return false;
        }

        if (dcdcPwr.last.s.chVin > params.Vin_max) {
            // input over-voltage
            ESP_LOGW("mppt", "Vin %.1f > %.1f!", dcdcPwr.last.s.chVin, params.Vin_max);
            pwm.disable();
            return false;
        }

        if (dcdcPwr.last.s.chVout > params.Vout_max * 1.05) {
            // output over-voltage
            ESP_LOGW("mppt", "Vout %.1fV (ewma=%.1fV,std=%.4f,pwm=%hu,state=%s,dir=%.1f) > %.1fV + 5%%!",
                     dcdcPwr.last.s.chVout,
                     dcdcPwr.ewm.s.chVout.avg.get(), dcdcPwr.ewm.s.chVout.std.get(), pwm.getBuckDutyCycle(),
                     MpptState2String[state].c_str(), pwmDirection,
                     params.Vout_max);
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

        if (dcdcPwr.last.s.chIin < -1) {
            ESP_LOGE("MPPT", "Reverse current %.1f A, noise? disable BFC and low-side FET", dcdcPwr.last.s.chIin);
            //pwm.disable();
            pwm.enableBackflowMosfet(false);
            pwm.lowSideMinDuty();
            //pwm.halfDutyCycle();
        }

        if (dcdcPwr.ewm.s.chIin.avg.get() < -1) {
            ESP_LOGE("MPPT", "Reverse avg current shutdown %.1f A", dcdcPwr.ewm.s.chIin.avg.get());
            pwm.disable();
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
        if (fminf(dcdcPwr.ewm.s.chIin.avg.get(), dcdcPwr.last.s.chIin) < 0.1f) {
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
        pwm.updateLowSideMaxDuty(voltageRatio);

        return true;
    }


    void startSweep() {
        _sweeping = true;
    }


    void update(bool dryRun) {
        //float voltage = pwm.getBuckDutyCycle(); // #dcdcPwr.ewm.s.chVin.avg.get(); // todo rename pwm
        auto Iin(dcdcPwr.ewm.s.chIin.avg.get());
        auto Vin(dcdcPwr.ewm.s.chVin.avg.get());
        float power = dcdcPwr.ewm.s.chIin.avg.get() * dcdcPwr.ewm.s.chVin.avg.get();
        //float power = dcdcPwr.last.s.chIin * dcdcPwr.last.s.chVin;

        Point point("mppt");
        point.addTag("device", "esp32_proto_mppt_" + String(getChipId()));
        point.addField("I", Iin, 2);
        point.addField("I_raw", dcdcPwr.last.s.chIin, 2);

        point.addField("U", Vin, 2);
        point.addField("U_raw", dcdcPwr.last.s.chVin, 2);

        point.addField("U_out", dcdcPwr.ewm.s.chVout.avg.get(), 2);
        point.addField("U_out_raw", dcdcPwr.last.s.chVout, 2);

        point.addField("P", power, 2);
        point.addField("P_prev", lastPower, 2);
        point.addField("P_raw", dcdcPwr.last.s.chIin * dcdcPwr.last.s.chVin, 2);

        if (dcdcPwr.last.s.chVout >= params.Vout_max) {
            pwmDirection = -8;
            if (dcdcPwr.last.s.chVout >= params.Vout_max * 1.01)
                pwmDirection = -16;
            state = MpptState::CV;
        } else if (dcdcPwr.last.s.chIin > params.Iin_max) {
            pwmDirection = -1;
            state = MpptState::CC;
        } else if (power > params.P_max) {
            pwmDirection = -1;
            state = MpptState::CP;
        } else {
            auto dP = power - lastPower;
            point.addField("dP", dP, 2);

            if (power < 2.f) {
                pwmDirection = 1;
            }

            if (std::abs(dP) < 1.5f) {
                // insignificant power change / noise
                // do nothing
                point.addField("dP_thres", 0.0f, 2);
                // ESP_LOGI("MPPT", "abs(dP)<1 DIR=%hhi", pwmDirection);
            } else {
                point.addField("dP_thres", dP, 2);
                if (dP >= 0) {
                    // power is increasing, we are on the right track
                    //ESP_LOGI("MPPT", "dP=%.7f DIR=%hhi", dP, pwmDirection);
                } else {
                    // ESP_LOGI("MPPT", "dP=%.7f DIR=%hhi REVERSE", dP, pwmDirection);
                    pwmDirection *= -1;
                }
                //if(std::abs(pwmDirection) > 1)
                //    pwmDirection /= std::abs(pwmDirection);
                lastPower = power;
            }
            state = MpptState::MPPT;
        }


        auto pwmDirectionScaled = pwmDirection;

        float vOut_pred =
                dcdcPwr.last.s.chVout + std::max<float>(0.0f, dcdcPwr.last.s.chVout - dcdcPwr.ewm.s.chVout.avg.get());
        if (vOut_pred >= params.Vout_max * 0.95f) {
            pwmDirectionScaled *= 0.5;
            if (vOut_pred >= params.Vout_max * 0.99f) {
                pwmDirectionScaled *= 0.5;
            }
        }
        point.addField("U_out_pred", vOut_pred, 2);

// slow down on low power (and high output impedance?)
// this reduces ripple
//if (power < 1.2) pwmDirectionScaled *= (1 / 4.0f);
        if (power < 0.8) pwmDirectionScaled *= (1 / 4.0f);
        if (power > 100) pwmDirectionScaled *= (1 / 4.0f); // TODO noise?

        if (_sweeping) {
            if (state != MpptState::MPPT || pwm.getBuckDutyCycle() == pwm.pwmMaxHS) {
                ESP_LOGI("mppt", "Stop sweep");
                _sweeping = false;
            } else {
                pwmDirectionScaled = 1.f;
            }
            state = MpptState::Sweep;
        }


        if (!dryRun) {
            pwm.pwmPerturbFractional(pwmDirectionScaled);

            point.addField("pwm_dir_f", pwmDirectionScaled, 2);

            // "bounce" low pwm
            if (pwmDirection <= 0 && pwm.getBuckDutyCycle() < pwm.pwmStartHS + 10) {
                ESP_LOGI("MPPT", "PWM floor bounce %hu", pwm.getBuckDutyCycle());
                pwmDirection = 1;
            }
        }

        if (Iin > 0.2f)
            pwm.enableBackflowMosfet(true);


        point.addField("mppt_state", int(state));

        point.addField("pwm_duty", pwm.getBuckDutyCycle());
        point.addField("pwm_ls_duty", pwm.getBuckDutyCycleLS());
        point.setTime(WritePrecision::MS);
        telemetryAddPoint(point, 40);


        // TODO limit chIout
        // switch off dcdc if power < min_power
        // V_in < limit
        // temperature
        // output power < limit
        // output current < limit
    }

};