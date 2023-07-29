#include <cstdint>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <Arduino.h>

#include "pinconfig.h"

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

    bool lowSideEnabled = false;

public:

    const uint16_t pwmMax, pwmMaxHS, pwmMinLS, pwmMinHS;


    HalfBridgePwm()
            : pwmMax((2 << (pwmResolution - 1)) - 1), pwmMaxHS(pwmMax * (1.0f - MinDutyCycleLS)),
              pwmMinLS(std::ceil((float) pwmMax * MinDutyCycleLS)), // keeping the bootstrap circuit powered
              pwmMinHS(pwmMinLS / 4) // everything else is too much!
    // note that MOSFETs have different Vg(th) and switching times. worst case is Vi/o=60/12
    // ^ set pwmMinHS a bit lower than pwmMinLS (might cause no-load output over-voltage otherwise)
    {}

    bool init() {

        // TODO pin self-check?

        uint32_t pwmFrequency = 39000; // buck converter switching frequency

        ESP_LOGI("pwm", "pwmMax=%hu, pwmMinLS=%hu, pwmMinHS=%hu, pwmMaxHS=%hu", pwmMax, pwmMinLS, pwmMinHS, pwmMaxHS);

        init_pwm(pwmCh_IN, getBuckIN_PIN(), pwmFrequency);
        init_pwm(pwmCh_EN, (uint8_t) PinConfig::Bridge_EN, pwmFrequency);


        pinMode((uint8_t) PinConfig::Backflow_EN, OUTPUT);
        digitalWrite((uint8_t) PinConfig::Backflow_EN, false);

        return true;
    }

    bool disabled() const { return pwmHS == 0; }

    void pwmPerturb(int16_t direction) {

        pwmHS = constrain(pwmHS + direction, pwmMinHS, pwmMaxHS);

        pwmMaxLS = computePwmMaxLs(pwmHS, pwmMax, outInVoltageRatio);
        pwmMaxLS = std::max(pwmMinLS, pwmMaxLS);

        if (pwmLS - pwmMaxLS > (pwmMax / 10)) {
            ESP_LOGW("pwm", "Set pwmLS %hu -> pwmMaxLS=%hu", pwmLS, pwmMaxLS);
        }

        // "fade-in" the low-side duty cycle
        pwmLS = lowSideEnabled ? constrain(pwmLS + 3, pwmMinLS, pwmMaxLS) : pwmMinLS;

        update_pwm(pwmCh_IN, pwmHS);
        update_pwm(pwmCh_EN, pwmHS + pwmLS);
    }

    float directionFloatBuffer = 0.0f;

    /**
     * Perturb by a fractional step.
     * Instantly perturbs the integer part and accumulates the remainder in a buffer
     *
     * @param directionFloat
     */
    void pwmPerturbFractional(float directionFloat) {
        assert(std::abs(directionFloat) < pwmMax);

        directionFloat += directionFloatBuffer;
        directionFloatBuffer = 0;
        auto directionInt = (int16_t) (directionFloat);
        if (directionInt != 0)
            pwmPerturb(directionInt);
        directionFloatBuffer += directionFloat - (float) directionInt;
    }

    //void pwmPerturbConstantRate(float normalizedDutyCycleStepPerSecond, unsigned long dt_us) {
    //    pwmPerturbFractional(normalizedDutyCycleStepPerSecond * (float)dt_us * 1e-6f * (float)pwmMax);
    //}

    uint16_t getBuckDutyCycle() const { return pwmHS; }

    uint16_t getBuckDutyCycleLS() const { return pwmLS; }

    uint16_t getDutyCycleLSMax() const { return pwmMaxLS; }

    void disable() {
        if (pwmHS > pwmMinHS)
            ESP_LOGW("pwm", "PWM disabled");
        pwmHS = 0;
        pwmLS = 0;
        update_pwm(pwmCh_IN, 0);
        update_pwm(pwmCh_EN, 0);
        enableBackflowMosfet(false);
    }


    /**
     * Compute low-side switch duty cycle to emulate a diode for synchronous buck
     * Prevents reverse current through the LS switch, which would lead to voltage boost and reverse current
     * and can eventually destroy the LS switch.
     * High-voltages at the output can toast anything connected (including the board itself!)
     *
     * The function does some error estimation and decides whether we are in DCM or CCM mode and limits the LS duty cycle accordingly.
     * See https://www.ti.com/seclit/ug/slyu036/slyu036.pdf#page=19 for more info about sync buck modes and timings
     *
     * @param pwmHS Duty cycle of the HS switch
     * @param pwmMax The maximum duty cycle value
     * @param voltageRatio Vout/Vin ratio (the greater, the safer but less efficient)
     * @return
     */
    static uint16_t computePwmMaxLs(uint16_t pwmHS, uint16_t pwmMax, float voltageRatio) {

        const float coilEfficiency = 0.99f; // TODO what does this model or represent, remove? maybe coil losses?

        // the computation below is very sensitive to voltageRatio errors!
        // at VR=0.64 a -5% error causes a 13% deviation of pwmMaxLs !
        // so we do some proper error computation here
        constexpr float voltageMaxErr = 0.02f; // inc -> safer, less efficient

        // WCEF = worst case error factor
        // compute the worst case error (Vout estimated too low, Vin too high)
        constexpr float voltageRatioWCEF = (1.f - voltageMaxErr) / (1.f + voltageMaxErr); // < 1.0

        voltageRatio = constrain(voltageRatio, 1e-2f, 1.f - 1e-2f); // constrain to reasonable range

        // compute worst-case error at current working point
        const float pwmMaxLsWCEF = (1 / (voltageRatio * voltageRatioWCEF) - 1) / (1 / voltageRatio - 1); // > 1

        auto pwmMaxLs = (1 / voltageRatio - 1) * (float) pwmHS / pwmMaxLsWCEF; // the lower, the safer

        if (pwmMaxLs < (float) (pwmMax - pwmHS)) {
            // DCM (Discontinuous Conduction Mode)
            // this is when the coil current is still touching zero, it'll stop for higher HS duty cycles
            // Allowing higher duty cycles here would result a synchronous forced PWM.
            // (forced PWM reduces output ripple?) TODO
            pwmMaxLs = std::min<float>(pwmMaxLs, (float) pwmHS);
        } else {
            // CCM (Continuous Conduction Mode)
            // coil current never becomes zero
            pwmMaxLs = (float) (pwmMax - pwmHS);
        }

        //ESP_LOGI("dbg", "pwmMaxLs=%f, (float) (pwmMax - pwmHS)=%f, pwmMax=%hu, pwmHS=%hu", pwmMaxLs, (float) (pwmMax - pwmHS), pwmMax, pwmHS);

        // todo beyond pwmMaxLs < (pwmMax - pwmHS), can we reduce the worst-case error assumption to boost eff?
        // or just replace  with pwmMaxLs < (pwmMax - pwmHS) * pwmMaxLsWCEF and remove scaling pwmMaxLs by pwmMaxLsWCEF

        return (uint16_t) (pwmMaxLs * coilEfficiency);
    }

    void updateLowSideMaxDuty(float vout, float vin) {
        // voltageRatio = Vout/Vin

        if (vin > vout && vin > 0.1f) {
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

    /**
     * Permanently set enable/disable low-side switch
     * @param enable
     */
    void enableLowSide(bool enable) {
        if (enable != lowSideEnabled) {
            ESP_LOGI("pwm", "Low-side switch %s", enable ? "enabled" : "disabled");
        }
        lowSideEnabled = enable;
        if (!enable && pwmLS > pwmMinLS) {
            ESP_LOGI("pwm", "Set Low-side pwm min");
            pwmLS = pwmMinLS;
            update_pwm(pwmCh_EN, pwmHS + pwmLS);
        }
    }

    /**
     * Set low-side duty cycle to minimum (e.g. disable power conduction, just keep the bootstrapping powered for HS drive)
     *
     */
    void lowSideMinDuty() {
        if (pwmLS > pwmMinLS) {
            if (pwmLS > pwmMinLS + pwmMinLS / 2) {
                ESP_LOGW("pwm", "set low-side PWM to minimum %hu -> %hu (vRatio=%.3f, pwmMaxLS=%hu)", pwmLS, pwmMinLS,
                         outInVoltageRatio, pwmMaxLS);
            }
            pwmLS = pwmMinLS;
            update_pwm(pwmCh_EN, pwmHS + pwmLS);
        }
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
                .hpoint         = 0,
                .flags          = {.output_invert = 0},
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    void update_pwm(int channel, uint32_t duty) {
        ESP_ERROR_CHECK(ledc_set_duty(ledcMode, (ledc_channel_t) channel, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(ledcMode, (ledc_channel_t) channel));
    }

};