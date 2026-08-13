#pragma once
// Bundled mode/state flags owned by main.cpp. Single struct so cli.cpp and other consumers see a
// documented surface (`g_app.manualPwm`) instead of a sprawl of file-scope externs. Components
// (mppt, converter, adcSampler, sensors, led, nvs) stay as named globals — they're the right
// granularity already.

#include <cstdint>

#include "util.h"

enum class OpMode : uint8_t { Mppt = 0, Manual, Psu };

struct AppState {
    OpMode opMode = OpMode::Mppt;
    [[nodiscard]] bool manualPwm() const { return opMode == OpMode::Manual; }
    [[nodiscard]] bool psuMode()   const { return opMode == OpMode::Psu; }
#ifdef WITH_NETW
    bool disableWifi = false;
    time_ms wifiReenableMs = 0; // wallClockMs() deadline to auto re-enable WiFi (0 = never)
#endif
    bool usbConnected = false;
    bool setupErr = false;
    uint16_t loopRateMin = 0;
    uint32_t maxLoopLag = 0; // peak RT-loop lag (µs), a time delta, not a timestamp
#if CAPTURE_LOOP_DT
    uint32_t maxLoopDT = 0; // peak loop dt (µs), a time delta
#endif
};

extern AppState g_app;
