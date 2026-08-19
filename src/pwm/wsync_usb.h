#pragma once

// Wired sync on a USB D-/D+ pad. Boards without a GPIO header (flu) can only reach the sync
// wire through the USB connector, but that pad is either the USB-Serial-JTAG PHY or a GPIO,
// never both. Qualify the line first, hand the pad over only if a leader is really driving it.
//
// Kept out of buck.h so the USB/PCNT headers don't land in every TU that includes it.

#if WITH_WSYNC

// Why the run ended up where it did. Reported by `wsync`, which otherwise cannot tell a USB
// fallback from a configured sync_role=none.
enum class WsyncMode : uint8_t {
    configured_none,   // sync_role=none, or not a USB pin: nothing was probed
    usb_host_active,   // a host was talking to us; the pad was never touched
    probe_no_edges,    // pad taken, line silent
    probe_bad_rate,    // edges present but wrong rate or irregular spacing
    armed_follower,    // qualified, pad kept for the GPIO matrix
    leader,
};

inline const char *wsyncModeStr(WsyncMode m) {
    switch (m) {
        case WsyncMode::usb_host_active: return "usb (host active)";
        case WsyncMode::probe_no_edges:  return "usb (no sync edges)";
        case WsyncMode::probe_bad_rate:  return "usb (sync rate/spacing bad)";
        case WsyncMode::armed_follower:  return "follower";
        case WsyncMode::leader:          return "leader";
        default:                         return "none";
    }
}

#include <cstdint>

#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED

#include <driver/pulse_cnt.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <hal/usb_serial_jtag_ll.h>
#include <soc/usb_pins.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>

// USBPHY_DM_NUM/USBPHY_DP_NUM (19/20 on S3) are what soc/usb_pins.h actually defines; there is
// no USB_DP_GPIO_NUM in IDF 5.5.
inline bool wsyncPinIsUsb(uint8_t pin) {
    return pin == USBPHY_DM_NUM || pin == USBPHY_DP_NUM;
}

inline void wsyncUsbPadEnable(bool en) {
    usb_serial_jtag_ll_phy_enable_pad(en);
}

struct WsyncProbeResult {
    bool qualified = false;
    float rateHz = 0.f;
    int edges = 0;
};

// One counting window. Returns edges seen and the rate over the *measured* interval -- a
// preempted caller overruns the delay, and dividing by the requested ms reports a healthy wire
// as over-rate (same reason cmdWsync measures).
inline bool wsyncCountWindow(uint8_t pin, uint32_t ms, int &edges, float &rateHz) {
    pcnt_unit_config_t upc = {.low_limit = -32768, .high_limit = 32767,
                              .intr_priority = 0, .flags = {}};
    // Accumulate across the wrap. A healthy 39 kHz line cannot reach 32767 in 20 ms, but the
    // counter RESETS at high_limit, so without this a fast train aliases down into the accept
    // band: (32768 + 780) / 20 ms ~ 1.68 MHz reads back as a plausible 39 kHz, and a 25 ns
    // filter does not reject 1.68 MHz. Qualification must not be foolable by a faster signal.
    upc.flags.accum_count = 1;
    pcnt_unit_handle_t unit = nullptr;
    if (pcnt_new_unit(&upc, &unit) != ESP_OK) return false;
    if (pcnt_unit_add_watch_point(unit, 32767) != ESP_OK) { pcnt_del_unit(unit); return false; }

    bool ok = false;
    pcnt_channel_handle_t ch = nullptr;
    pcnt_chan_config_t cpc = {.edge_gpio_num = pin, .level_gpio_num = -1,
                              .flags = {.invert_edge_input = 0, .invert_level_input = 0,
                                        .virt_edge_io_level = 0, .virt_level_io_level = 1,
                                        .io_loop_back = 0}};
    if (pcnt_new_channel(unit, &cpc, &ch) == ESP_OK) {
        // pcnt_new_channel() unconditionally does gpio_pullup_en() + gpio_pulldown_dis()
        // (esp_driver_pcnt/src/pulse_cnt.c). The AC-coupled receiver biases against the
        // internal pull-DOWN; a pull-up parks the node near mid-rail, so we would be
        // qualifying a different circuit than the one initSyncIn() later runs. Undo it here.
        // Every setup step is a precondition of a POSITIVE result, so none of them may fail
        // silently: a missed pulldown or filter would qualify a different circuit than the one
        // initSyncIn() then runs. Fail closed -- ok stays false and the caller falls back to USB.
        esp_err_t e = gpio_set_pull_mode((gpio_num_t) pin, GPIO_PULLDOWN_ONLY);
        // Match the operational filter, do not beat it. MCPWM gets the S3 fixed PIN filter
        // (~25 ns); qualifying through a wider window would pass a line whose sub-window
        // ringing then re-phases the timer once armed.
        pcnt_glitch_filter_config_t gf = {.max_glitch_ns = 25};
        if (e == ESP_OK) e = pcnt_unit_set_glitch_filter(unit, &gf);
        if (e == ESP_OK) e = pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                         PCNT_CHANNEL_EDGE_ACTION_HOLD);
        if (e == ESP_OK) e = pcnt_channel_set_level_action(ch, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                          PCNT_CHANNEL_LEVEL_ACTION_KEEP);
        if (e == ESP_OK) e = pcnt_unit_enable(unit);
        if (e == ESP_OK) e = pcnt_unit_clear_count(unit);
        if (e == ESP_OK) e = pcnt_unit_start(unit);
        if (e == ESP_OK) {
            int64_t t0 = esp_timer_get_time();
            vTaskDelay(pdMS_TO_TICKS(ms));
            esp_err_t er = pcnt_unit_get_count(unit, &edges);
            int64_t us = esp_timer_get_time() - t0;
            pcnt_unit_stop(unit);
            pcnt_unit_disable(unit);
            rateHz = us > 0 ? (float) edges * 1e6f / (float) us : 0.f;
            ok = er == ESP_OK && us > 0;
        }
    }

    // Teardown order matters: the unit must be back in INIT state with no channels before
    // deletion, and the pad must not go back to the PHY while we are still touching its pulls.
    if (ch) pcnt_del_channel(ch);
    pcnt_del_unit(unit);
    return ok;
}

// Qualify the sync line on a USB pad. Caller must have disabled the PHY pad first.
//
// Two windows within +-10% of nominal that also agree with each other. This bounds the RATE and
// its stability over 40 ms; it does NOT prove per-pulse spacing -- two edge totals cannot tell
// evenly spaced pulses from missing pulses plus compensating ringing. Proving regularity needs
// edge timestamps (capture/RMT) and is still open; the external Schmitt buffer that
// doc/dev-notes/wired-sync.md calls mandatory remains the real defence against a ringing line.
inline WsyncProbeResult wsyncQualifyLine(uint8_t pin, float expectHz, uint32_t ms = 20) {
    WsyncProbeResult r{};
    int e1 = 0, e2 = 0;
    float f1 = 0.f, f2 = 0.f;
    if (!wsyncCountWindow(pin, ms, e1, f1)) return r;
    if (!wsyncCountWindow(pin, ms, e2, f2)) return r;

    r.edges = e1 + e2;
    r.rateHz = (f1 + f2) * 0.5f;
    if (r.edges == 0) return r;

    const float tol = 0.10f;
    bool inBand = std::fabs(f1 - expectHz) <= expectHz * tol &&
                  std::fabs(f2 - expectHz) <= expectHz * tol;
    // The windows must also agree with each other: a wandering rate is not a locked leader.
    bool agree = std::fabs(f1 - f2) <= expectHz * tol;
    r.qualified = inBand && agree;
    return r;
}

#else // no USB-Serial-JTAG on this target

inline bool wsyncPinIsUsb(uint8_t) { return false; }

#endif // CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED

#endif // WITH_WSYNC
