#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

class PWM_ESP32_ledc {
    const uint8_t pwmResolution = 11;
    const ledc_mode_t ledcMode = LEDC_LOW_SPEED_MODE;

public:
    const uint16_t pwmMax;

    PWM_ESP32_ledc() : pwmMax((2 << (pwmResolution - 1)) - 1) {}

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