#pragma once

#include <cstdint>
#include <Arduino.h>

#include "etc/pinconfig.h"
#include "util.h"

/**
 * Drives the backflow switch
 * aka anti-back-feed, ideal diode, panel switch)
 */
class BackflowDriver {
    bool _state = false;
    int8_t panelEN;
    int8_t panelSD;
public:
    void init(const ConfFile &pin) {
        panelEN = pin.getByte("panel_en", 0);
        panelSD = pin.getByte("panel_sd", 0);

        if (panelEN) {
            assert(!panelSD);
            pinMode(panelEN, OUTPUT);
            digitalWrite(panelEN, false);
        } else if (panelSD) {
            pinMode(panelSD, OUTPUT);
            digitalWrite(panelSD, true);
        } else {
            // nothing
        }
        _state = false;
    }

    void enable(bool enable) {
        if (panelEN) {
            digitalWrite(panelEN, enable);
        } else if (panelSD) {
            digitalWrite(panelSD, !enable);
        } else {
            if (_state != enable) {
                UART_LOG_ASYNC("Back-flow switch ignored");
            }
        }
        if (_state != enable) {
            UART_LOG_ASYNC("Back-flow switch %s", enable ? "enabled" : "disabled");
        }
        _state = enable;
    }

    bool state() const { return _state; }

    explicit operator bool() const {
        return panelSD != 0 and panelEN != 0;
    }
};