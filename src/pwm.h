#include <cstdint>
#include <cmath>
#include<algorithm>
#include <Arduino.h>

#include "pinconfig.h"
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"


class HalfBridgePwm {

    static constexpr uint8_t pwmCh_IN = 0;
    static constexpr uint8_t pwmCh_EN = 1;

    static constexpr float MinDutyCycleLS = 0.06f; // to keep the HS bootstrap circuit running

    const uint8_t pwmResolution = 11;
    const ledc_mode_t ledcMode = LEDC_LOW_SPEED_MODE;

    uint16_t pwmHS = 0;
    uint16_t pwmLS = 0;

    uint16_t pwmMaxLS = 0;

    float outInVoltageRatio = 0;

public:

    const uint16_t pwmMax, pwmMaxHS, pwmMinLS, pwmMinHS;


    HalfBridgePwm()
            : pwmMax((2 << (pwmResolution - 1)) - 1), pwmMaxHS(pwmMax * (1.0f - MinDutyCycleLS)),
              pwmMinLS(std::ceil((float) pwmMax * MinDutyCycleLS)), // keeping the bootstrap circuit powered
              pwmMinHS(pwmMinLS / 4) // everything else is too much!
              // note that MOSFETs have different Vg(th) and switchting times. worst case is Vi/o=60/12
            // ^ set pwmMinHS a bit lower than pwmMinLS (because pwmMinLS might be already too much for CV with no load)
    {

    }

    bool init() {

        // TODO pin self-check?

        //uint32_t pwmFrequency = 39000;           //  USER PARAMETER - PWM Switching Frequency - Hz (For Buck)
        uint32_t pwmFrequency = 39000;           //  USER PARAMETER - PWM Switching Frequency - Hz (For Buck)
        //float PPWM_margin = 99.5;          //  CALIB PARAMETER - Minimum Operating Duty Cycle for Predictive PWM (%)
        //float PWM_MaxDC = 95.0;            //  CALIB PARAMETER - Maximum Operating Duty Cycle (%) 90%-97% is good, 97 makes low-side turn-on too short for bootstrapping

        //uint16_t pwmMax = 0;
        //uint16_t pwmMaxLimited = 0;
        //uint16_t PWM = 0;

        ESP_LOGI("pwm", "pwmMax=%hu, pwmMinLS=%hu, pwmMinHS=%hu, pwmMaxHS=%hu", pwmMax, pwmMinLS, pwmMinHS, pwmMaxHS);




        //PWM INITIALIZATION
        init_pwm(pwmCh_IN, getBuckIN_PIN(), pwmFrequency);
        init_pwm(pwmCh_EN, (uint8_t) PinConfig::Bridge_EN, pwmFrequency);


        // pwmMax = pow(2, pwmResolution) - 1;                           //Get PWM Max Bit Ceiling
        //pwmMaxLimited =(PWM_MaxDC * pwmMax) / 100.0f;

        pinMode((uint8_t) PinConfig::Backflow_EN, OUTPUT);
        digitalWrite((uint8_t) PinConfig::Backflow_EN, false);

        return true;
    }

    bool disabled() {
        return pwmHS == 0;
    }


    void pwmPerturb(int16_t direction) {

        pwmHS = constrain(pwmHS + direction, pwmMinHS, pwmMaxHS);

        pwmMaxLS = computePwmMaxLs(pwmHS, pwmMax, outInVoltageRatio);
        pwmMaxLS = std::max(pwmMinLS, pwmMaxLS);

        if (pwmLS - pwmMaxLS > (pwmMax / 10)) {
            ESP_LOGW("pwm", "Set pwmLS %hu -> pwmMaxLS=%hu", pwmLS, pwmMaxLS);
        }

        // "fade-in" the low-side duty cycle
        pwmLS = constrain(pwmLS + 3, pwmMinLS, pwmMaxLS);

        update_pwm(pwmCh_IN, pwmHS);
        update_pwm(pwmCh_EN, pwmHS + pwmLS);
    }

    float directionFloatBuffer = 0.0f;

    void pwmPerturbFractional(float directionFloat) {
        assert(std::abs(directionFloat) < pwmMax);

        directionFloat += directionFloatBuffer;
        directionFloatBuffer = 0;
        auto directionInt = (int16_t)(directionFloat);
        if (directionInt != 0)
            pwmPerturb(directionInt);
        directionFloatBuffer += directionFloat - (float) directionInt;
    }

    uint16_t getBuckDutyCycle() const {
        return pwmHS;
    }

    uint16_t getBuckDutyCycleLS() const {
        return pwmLS;
    }

    uint16_t getDutyCycleLSMax() const {
        return pwmMaxLS;
    }

    /*void halfDutyCycle() {
        pwmHS /= 2;
        pwmLS /= 2;
        pwmPerturb(0);
    }*/

    void disable() {
        if (pwmHS > pwmMinHS)
            ESP_LOGW("pwm", "PWM disabled");
        pwmHS = 0;
        pwmLS = 0;
        update_pwm(pwmCh_IN, 0);
        update_pwm(pwmCh_EN, 0);
        enableBackflowMosfet(false);
    }

    void lowSideMinDuty() {
        if (pwmLS > pwmMinLS + pwmMinLS/2) {
            ESP_LOGW("pwm", "set low-side PWM to minimum %hu -> %hu (vRatio=%.3f, pwmMaxLS=%hu)", pwmLS, pwmMinLS,
                     outInVoltageRatio, pwmMaxLS);
        }
        pwmLS = pwmMinLS;
        update_pwm(pwmCh_EN, pwmHS + pwmLS);
    }

    static uint16_t computePwmMaxLs(uint16_t pwmHS, uint16_t pwmMax, float voltageRatio) {
        // prevents boosting and "low side burning" due to excessive LS current
        // (non-diode behavior, negative reverse inductor current)

        const float margin = 0.99f; // TODO what does this model or represent, remove? maybe coil losses?

        // this is very sensitive to voltageRatio errors! at VR=0.64 a -5% error causes a 13% deviation of pwmMaxLs !
        constexpr float voltageMaxErr = 0.02f; // inc -> safer, less efficient

        // WCEF = worst case error factor
        // compute the worst case error (Vout too low, Vin too high)
        constexpr float voltageRatioWCEF = (1.f - voltageMaxErr) / (1.f + voltageMaxErr); // < 1.0

        voltageRatio = constrain(voltageRatio, 1e-2f, 1.f - 1e-2f); // the greater, the safer

        const float pwmMaxLsWCEF = (1 / (voltageRatio * voltageRatioWCEF) - 1) / (1 / voltageRatio - 1); // > 1


        auto pwmMaxLs = (1 / voltageRatio - 1) * (float) pwmHS / pwmMaxLsWCEF; // the lower, the safer

        // pwmMaxLs = std::min<float>(pwmMaxLs, (float)pwmHS); // TODO explain why this is necessary
        // I guess it can be a little more
        // At which duty cycle (HS) does coil current stop touching zero? see if-block below vvv
        // ^^ https://github.com/fl4p/fugu-mppt-firmware/issues/1

        // the extra 5% below fixes reverse current at 39khz, Vin55, Vout27, dc1000
        // TODO pwm=1390 40.8/26.6

        // TODO remove the 1.05?
        if (pwmMaxLs < (pwmMax - pwmHS)) { // * 1.05f
            // DCM (Discontinuous Conduction Mode)
            // this is when the coil current is still touching zero
            // it'll stop for higher HS duty cycles
            pwmMaxLs = std::min<float>(pwmMaxLs, (float) pwmHS * 1.0f); // TODO explain why this is necessary
        } else {
            pwmMaxLs = (float) (pwmMax - pwmHS);
        }

        //ESP_LOGI("dbg", "pwmMaxLs=%f, (float) (pwmMax - pwmHS)=%f, pwmMax=%hu, pwmHS=%hu", pwmMaxLs, (float) (pwmMax - pwmHS), pwmMax, pwmHS);

        // todo beyond pwmMaxLs < (pwmMax - pwmHS), can we reduce the worst-case error assumption to boost eff?
        // or just replace  with pwmMaxLs < (pwmMax - pwmHS) * pwmMaxLsWCEF and remove scaling pwmMaxLs by pwmMaxLsWCEF

        return (uint16_t)(pwmMaxLs * margin);
    }

    void updateLowSideMaxDuty(float vout, float vin) {
        // voltageRatio = Vout/Vin

        if(vin > vout && vin > 0.1f) {
            outInVoltageRatio = vout / vin;
        } else {
            outInVoltageRatio = 1; // the safest (minimize LS duty cycle)
        }

        pwmMaxLS = computePwmMaxLs(pwmHS, pwmMax, outInVoltageRatio);
        pwmMaxLS = std::max(pwmMinLS, pwmMaxLS);

        if (pwmLS > pwmMaxLS) {
            if (pwmLS - pwmMaxLS > (pwmMax / 10)) {
                ESP_LOGW("pwm", "Set pwmLS %hu -> pwmMaxLS=%hu (VR=%.2f)", pwmLS, pwmMaxLS, outInVoltageRatio);
            }
            pwmLS = pwmMaxLS;
            update_pwm(pwmCh_EN, pwmHS + pwmLS); // instantly commit if limit decreases
        }
    }

    void enableBackflowMosfet(bool enable) {
        digitalWrite((uint8_t) PinConfig::Backflow_EN, enable);
    }


private:
    void init_pwm(int channel, int pin, uint32_t freq) {
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
                .speed_mode       = ledcMode,
                .duty_resolution  = (ledc_timer_bit_t) pwmResolution,
                .timer_num        = LEDC_TIMER_0,
                .freq_hz          = freq,
                .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        // Prepare and then apply the LEDC PWM channel configuration
        ledc_channel_config_t ledc_channel = {
                .gpio_num       = pin,
                .speed_mode     = ledcMode,
                .channel        = (ledc_channel_t) channel,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LEDC_TIMER_0,
                .duty           = 0, // Set duty to 0%
                .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    void update_pwm(int channel, uint32_t duty) {
        ESP_ERROR_CHECK(ledc_set_duty(ledcMode, (ledc_channel_t) channel, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(ledcMode, (ledc_channel_t) channel));
    }

};