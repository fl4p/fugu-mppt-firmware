#pragma once

#include <cstdint>
#include <Arduino.h>

#include "pinconfig.h"
#include "util.h"

/**
 * Drives the backflow switch
 * aka anti-back-feed, ideal diode, panel switch)
 */
class BackflowDriver {
    bool _state = false;
public:
    void init() {
        if ((bool) PinConfig::Backflow_EN) {
            assert(!(bool) PinConfig::Backflow_SD);
            pinMode((uint8_t) PinConfig::Backflow_EN, OUTPUT);
            digitalWrite((uint8_t) PinConfig::Backflow_EN, false);
        } else {
            assert((bool) PinConfig::Backflow_SD);
            pinMode((uint8_t) PinConfig::Backflow_SD, OUTPUT);
            digitalWrite((uint8_t) PinConfig::Backflow_SD, true);
        }
        _state = false;
    }

    void enable(bool enable) {
        if ((bool) PinConfig::Backflow_EN) {
            digitalWrite((uint8_t) PinConfig::Backflow_EN, enable);
        } else {
            digitalWrite((uint8_t) PinConfig::Backflow_SD, !enable);
        }
        if(_state != enable) {
            UART_LOG_ASYNC("Backflow switch %s", enable ? "enabled" : "disabled");
        }
        _state = enable;
    }

    bool state() const { return _state; }
};