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

    const uint16_t pwmMax, pwmMaxHS, pwmMinLS;
    uint16_t pwmStartHS = 0;


    HalfBridgePwm()
            : pwmMax(pow(2, pwmResolution) - 1), pwmMaxHS(pwmMax * (1.0f - MinDutyCycleLS)),
              pwmMinLS(std::ceil(pwmMax * MinDutyCycleLS)) {

    }

    bool init() {

        // TODO pin self-check?

        //uint32_t pwmFrequency = 39000;           //  USER PARAMETER - PWM Switching Frequency - Hz (For Buck)
        uint32_t pwmFrequency = 22000;           //  USER PARAMETER - PWM Switching Frequency - Hz (For Buck)
        //float PPWM_margin = 99.5;          //  CALIB PARAMETER - Minimum Operating Duty Cycle for Predictive PWM (%)
        //float PWM_MaxDC = 95.0;            //  CALIB PARAMETER - Maximum Operating Duty Cycle (%) 90%-97% is good, 97 makes low-side turn-on too short for bootstrapping

        //uint16_t pwmMax = 0;
        //uint16_t pwmMaxLimited = 0;
        //uint16_t PWM = 0;




        //PWM INITIALIZATION
        init_pwm(pwmCh_IN, (uint8_t) PinConfig::Bridge_IN, pwmFrequency);
        init_pwm(pwmCh_EN, (uint8_t) PinConfig::Bridge_EN, pwmFrequency);


        // pwmMax = pow(2, pwmResolution) - 1;                           //Get PWM Max Bit Ceiling
        //pwmMaxLimited =(PWM_MaxDC * pwmMax) / 100.0f;

        pinMode((uint8_t) PinConfig::Backflow_EN, OUTPUT);
        digitalWrite((uint8_t) PinConfig::Backflow_EN, false);

        return true;
    }


    void pwmPerturb(int16_t direction) {

        if (pwmHS <= 1 && pwmHS < pwmStartHS)
            pwmHS = pwmStartHS;

        // compute ordinary pwm update within bounds:
        pwmHS = constrain(pwmHS + direction, 0, pwmMaxHS);// pwmMinLS


        pwmMaxLS = computePwmMaxLs(pwmHS, pwmMax, outInVoltageRatio);

        // "fade-in" the low-side duty cycle
        pwmLS = constrain(pwmLS + 2, pwmMinLS, pwmMaxLS);

        update_pwm(pwmCh_IN, pwmHS);
        update_pwm(pwmCh_EN, pwmHS + pwmLS);
    }

    float directionFloatBuffer = 0.0f;

    void pwmPerturbFractional(float directionFloat) {
        assert(std::abs(directionFloat) < 128);

        directionFloat += directionFloatBuffer;
        directionFloatBuffer = 0;
        auto directionInt = (int8_t) (directionFloat);
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

    void halfDutyCycle() {
        pwmHS /= 2;
        pwmLS /= 2;
        pwmPerturb(0);
    }

    void disable() {
        if (pwmHS > 0)
            ESP_LOGW("pwm", "PWM disabled");
        pwmHS = 0;
        pwmLS = 0;
        update_pwm(pwmCh_IN, 0);
        update_pwm(pwmCh_EN, 0);
        enableBackflowMosfet(false);
    }

    void lowSideMinDuty() {
        if (pwmLS > pwmMinLS + 10)
            ESP_LOGW("pwm", "set low-side PWM to minimum %hu -> %hu", pwmLS, pwmMinLS);
        pwmLS = pwmMinLS;
        update_pwm(pwmCh_EN, pwmHS + pwmLS);
    }

    static uint16_t computePwmMaxLs(uint16_t pwmHS, uint16_t pwmMax, float voltageRatio) {
        // prevents boosting and "low side burning" due to excessive LS current
        // (non-diode behavior, negative reverse inductor current)
        const float margin = 0.99f;
        voltageRatio = std::max<float>(voltageRatio, 0.01f);
        auto pwmMaxLs = ((1 / voltageRatio - 1) * (float) pwmHS);

        // pwmMaxLs = std::min<float>(pwmMaxLs, (float)pwmHS); // TODO explain why this is necessary
        // I guess it can be a little more
        // At which duty cycle (HS) does coil current stop touching zero?
        // ^^ https://github.com/fl4p/fugu-mppt-firmware/issues/1

        if(pwmMaxLs < (pwmMax - pwmHS)) {
            // this is when the coil current is still touching zero
            // it'll stop for higher HS duty cycles
            pwmMaxLs = std::min<float>(pwmMaxLs, (float)pwmHS); // TODO explain why this is necessary
        }

        return (uint16_t)(std::min<float>(pwmMaxLs, (float)(pwmMax - pwmHS)) * margin);
    }

    void updateLowSideMaxDuty(float outInVoltageRatio_) {
        // voltageRatio = Vout/Vin

        outInVoltageRatio = outInVoltageRatio_;

        //auto idealBuckPwm = (uint16_t) ((float) pwmMax * (margin / 100.f) * std::min(1.f, voltageRatio));
        // idealBuckPwm / 4 is still too much for open circuit output
        pwmStartHS = pwmMinLS; // idealBuckPwm / 4; // k =.75

        pwmMaxLS = computePwmMaxLs(pwmHS, pwmMax, outInVoltageRatio);

        if (pwmLS > pwmMaxLS) {
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