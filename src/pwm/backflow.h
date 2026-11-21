#pragma once

#include <cstdint>
#include <Arduino.h>

#include "etc/pinconfig.h"
#include "util.h"
#include <atomic>
#if WITH_VCONV
#include "sim/vconv.h"
#endif

/**
 * Drives the backflow switch
 * aka anti-back-feed, ideal diode, panel switch)
 */
class BackflowDriver {
    bool _state = false;
    int8_t panelEN;
    int8_t panelSD;

public:
    static constexpr uint8_t LockHsShort = 1, LockSelftest = 2;
    std::atomic<uint8_t> lockOff{0}; // owner bits; enable(true) is refused while any is set

    void init(const ConfFile &pin) {
#if WITH_VCONV
        // simulated plant: keep the logical state, never drive a real board's panel switch
        panelEN = panelSD = 0;
#else
        panelEN = pin.getByte("panel_en", 0);
        panelSD = pin.getByte("panel_sd", 0);
#endif

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
#ifdef FUGU_FORCE_BFLOW_OFF
        // Bench-safety override: keep the panel/backflow switch open so battery current
        // cannot flow into the converter while testing. The MPPT loop re-drives this
        // every tick, so a runtime `bf 0` can't hold; this is the only reliable hold.
        enable = false;
#endif
        if (lockOff.load(std::memory_order_relaxed)) enable = false;
#if WITH_VCONV
        g_vconv.bfOn = enable;
#endif
        if (panelEN) {
            digitalWrite(panelEN, !!enable);
        } else if (panelSD) {
            digitalWrite(panelSD, !enable);
        } else {
            //if (_state != enable) {
            //  UART_LOG_ASYNC("Back-flow switch ignored");
            //}
        }
        if (_state != enable && (panelEN or panelSD)) {
            UART_LOG_ASYNC("Back-flow switch %s", enable ? "enabled" : "disabled");
        }
        _state = enable;
    }

    bool state() const { return _state; }

    explicit operator bool() const {
#if WITH_VCONV
        if (g_vconv.bfModel) return true;
#endif
        return panelSD != 0 or panelEN != 0;   // init() asserts the two are mutually exclusive
    }
};
