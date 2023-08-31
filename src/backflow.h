#pragma once

#include <cstdint>
#include <Arduino.h>

#include "pinconfig.h"

/**
 * Drives the backflow switch
 * aka anti-back-feed, ideal diode
 */
class BackflowDriver {

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
    }

    void enable(bool enable) {
        if ((bool) PinConfig::Backflow_EN) {
            digitalWrite((uint8_t) PinConfig::Backflow_EN, enable);
        } else {
            digitalWrite((uint8_t) PinConfig::Backflow_SD, !enable);
        }
    }
};