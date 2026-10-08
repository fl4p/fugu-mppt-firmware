#pragma once

#include <cmath>
#include <cstdint>
#include <Arduino.h>
#include "esp_private/esp_gpio_reserve.h"

#include "etc/pinconfig.h"
#include "util.h"

/**
 * Drives the LV power-good output. High = the LV terminal is a good aux supply source,
 * which disables the HV aux supply path. Low (and the pin's pull-down) = HV path enabled.
 * Asserts after the LV voltage stayed >= vOn for holdMs of continuous fresh samples. Releases at
 * once below vOn - hyst, on a non-finite or stale reading, or on release().
 */
class LvPgood {
    static constexpr float hyst = 0.5f, vOnMin = 5.f;
    static constexpr uint32_t holdMs = 5000, staleMs = 500, gapMs = 200;

    uint8_t pin = 255;
    bool _state = false;
    float vOn = 10.f;
    uint32_t goodSinceMs = 0, lastCallMs = 0, freshMs = 0, lastN = 0;

    void set(bool on) {
        digitalWrite(pin, on);
        if (on == _state) return;
        _state = on;
        UART_LOG_ASYNC("LV pgood %s", on ? "on" : "off");
    }

public:
    void init(const ConfFile &board) {
        pin = board.getByte("lv_pgood", 255);
        float v = board.getFloat("lv_pgood_v", 10.f);
        vOn = std::isfinite(v) && v >= vOnMin ? v : 10.f;
        if (pin == 255) return;
        if (!GPIO_IS_VALID_OUTPUT_GPIO(pin) || esp_gpio_is_reserved(BIT64(pin))) {
            ESP_LOGE("pgood", "lv_pgood pin %u unusable, disabled", pin);
            pin = 255;
            return;
        }
        pinMode(pin, OUTPUT);
        set(false);
        goodSinceMs = 0;
    }

    void update(float vLvFast, float vLvAvg, uint32_t nSamples, bool allow, uint32_t nowMs) {
        if (pin == 255) return;
        if (nowMs - lastCallMs > gapMs) goodSinceMs = 0;
        lastCallMs = nowMs;
        if (nSamples != lastN) {
            lastN = nSamples;
            freshMs = nowMs;
        }
        if (!allow || nowMs - freshMs > staleMs || !std::isfinite(vLvFast) || !std::isfinite(vLvAvg)
            || vLvFast < vOn - hyst) {
            release();
            return;
        }
        if (_state) return;
        if (vLvFast < vOn || vLvAvg < vOn) {
            goodSinceMs = 0;
            return;
        }
        if (!goodSinceMs) goodSinceMs = nowMs | 1u;
        else if (nowMs - goodSinceMs >= holdMs) set(true);
    }

    // Writes the pin unconditionally: a racing set(true) on the other core must not leave it high.
    void release() {
        if (pin == 255) return;
        goodSinceMs = 0;
        set(false);
    }

    bool state() const { return _state; }
};
