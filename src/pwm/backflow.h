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
 * (aka anti-back-feed, ideal diode, panel switch)
 */
class BackflowDriver {
    bool _state = false;
    int8_t pinEN;
    int8_t pinSD;

public:
    static constexpr uint8_t LockHsShort = 1, LockSelftest = 2;
    std::atomic<uint8_t> lockOff{0}; // owner bits; enable(true) is refused while any is set

    void init(const ConfFile &pin) {
        // panel_en/panel_sd were renamed; an old key alone fails setup (etc/migrate_side_keys.py)
        for (auto k: {"en", "sd"}) {
            const std::string o = std::string("panel_") + k, n = std::string("bflow_") + k;
            if (!pin.has(o)) continue;
            if (!pin.has(n)) throw std::runtime_error("board.conf: " + o + " was renamed to " + n);
            ESP_LOGW("bflow", "board.conf: %s ignored, %s is set; remove %s", o.c_str(), n.c_str(), o.c_str());
        }
#if WITH_VCONV
        // simulated plant: keep the logical state, never drive a real board's switch
        pinEN = pinSD = 0;
#else
        pinEN = pin.getByte("bflow_en", 0);
        pinSD = pin.getByte("bflow_sd", 0);
#endif

        if (pinEN) {
            assert(!pinSD);
            pinMode(pinEN, OUTPUT);
            digitalWrite(pinEN, false);
        } else if (pinSD) {
            pinMode(pinSD, OUTPUT);
            digitalWrite(pinSD, true);
        } else {
            // nothing
        }
        _state = false;
    }

    void enable(bool enable) {
#ifdef FUGU_FORCE_BFLOW_OFF
        // Bench-safety override: keep the backflow switch open so battery current
        // cannot flow into the converter while testing. The MPPT loop re-drives this
        // every tick, so a runtime `bf 0` can't hold; this is the only reliable hold.
        enable = false;
#endif
        if (lockOff.load(std::memory_order_relaxed)) enable = false;
#if WITH_VCONV
        g_vconv.bfOn = enable;
#endif
        if (pinEN) {
            digitalWrite(pinEN, !!enable);
        } else if (pinSD) {
            digitalWrite(pinSD, !enable);
        } else {
            //if (_state != enable) {
            //  UART_LOG_ASYNC("Back-flow switch ignored");
            //}
        }
        if (_state != enable && (pinEN or pinSD)) {
            UART_LOG_ASYNC("Back-flow switch %s", enable ? "enabled" : "disabled");
        }
        _state = enable;
    }

    bool state() const { return _state; }

    explicit operator bool() const {
#if WITH_VCONV
        if (g_vconv.bfModel) return true;
#endif
        return pinSD != 0 or pinEN != 0;   // init() asserts the two are mutually exclusive
    }
};
