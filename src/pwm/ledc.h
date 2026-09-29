#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

class PWM_ESP32_ledc {
    //const uint8_t pwmResolution = 10;
#if SOC_LEDC_SUPPORT_HS_MODE
    static constexpr ledc_mode_t ledcMode = LEDC_HIGH_SPEED_MODE;
#else
    static constexpr ledc_mode_t ledcMode = LEDC_LOW_SPEED_MODE;
#endif

public:
    const char *name = "ledc";
    uint16_t pwmMax;

    PWM_ESP32_ledc() : pwmMax(0) {}


    void init_pwm(int channel, int pin, uint32_t freq) {
        // see idf5.3 ledc_find_suitable_duty_resolution()
        uint32_t div = getApbFrequency() / freq; // + pwmFrequency / 2
        auto resolution = min((int) log2(div), SOC_LEDC_TIMER_BIT_WIDTH);
        auto pm = ((2 << (resolution - 1)) - 1);
        if (pwmMax == 0) pwmMax = pm;
        assert(pm == pwmMax);

        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
                .speed_mode       = ledcMode,
                .duty_resolution  = (ledc_timer_bit_t) resolution,
                .timer_num        = LEDC_TIMER_0,
                .freq_hz          = freq,
                .clk_cfg          = LEDC_AUTO_CLK,
                .deconfigure      = false,
        };
        ESP_ERROR_CHECK_THROW(ledc_timer_config(&ledc_timer));

        // Prepare and then apply the LEDC PWM channel configuration
        // assigned, not designated: intr_type (deprecated) and deconfigure differ between IDF 5 and 6
        ledc_channel_config_t ledc_channel = {};
        ledc_channel.gpio_num = pin;
        ledc_channel.speed_mode = ledcMode;
        ledc_channel.channel = (ledc_channel_t) channel;
        ledc_channel.timer_sel = LEDC_TIMER_0;
        ledc_channel.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
        ESP_ERROR_CHECK_THROW(ledc_channel_config(&ledc_channel));
    }

    //void set_freq() {
    //     ledc_set_freq(ledcMode, LEDC_TIMER_0, freq);
    //}

    void update_pwm(int channel, uint32_t duty) {
        ESP_ERROR_CHECK(ledc_set_duty(ledcMode, (ledc_channel_t) channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(ledcMode, (ledc_channel_t) channel));
    }

    void update_pwm(int channel, uint32_t hpoint, uint32_t duty) {
        ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(ledcMode, (ledc_channel_t) channel, duty, hpoint));
        ESP_ERROR_CHECK(ledc_update_duty(ledcMode, (ledc_channel_t) channel));
    }

    void stop(int channel, uint8_t idleLevel) {
        ESP_ERROR_CHECK(ledc_stop(ledcMode,  (ledc_channel_t) channel, idleLevel ));
    }
};