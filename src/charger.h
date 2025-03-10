#pragma once

#include "mqtt.h"

#ifndef FUGU_BAT_V
#define FUGU_BAT_V NAN
#endif


struct BatChargerParams {
    float Vbat_max = FUGU_BAT_V; //FUGU_BAT_V; //14.6 * 2;
    //float Iout_max = 27; // HW1: coil & fuse limited
    //float Iout_max = 32; // HW2, backflow mosfet gets hot!

    //float Vout_top() const { return Vbat_max * 0.98f; } //0.96 0.975f;
    //float Vout_top_release() const { return Vbat_max * 0.94f; }

    // float Iout_top = .5;
};

class BatteryCharger {
    // unsigned long timeTopUntil = 0; // charger
    //float vbat_cap = NAN;

public:
    BatChargerParams params{};

    volatile float vcell_max = 0;
    volatile unsigned long vcell_max_t = 0;

    explicit BatteryCharger() = default;

    void begin() {
        mqtt_subscribe_topic("bat_caravan/cell_voltages/max", [&](const char *dat, int len) {
            this->vcell_max = strtof(dat, nullptr);
            this->vcell_max_t = wallClockUs();
        });
    }

    /*
    float getToppingCurrent(float Vout) {
        auto nowMs = millis();

        // Topping-Mode reduces current with rising output-voltage
        // https://www.toolfk.com/online-plotter-frame#W3sidHlwZSI6MCwiZXEiOiIyMC0oMjAtMikqbWluKDEsKHgtMjcpLygyOC0yNykpKiouNSIsImNvbG9yIjoiIzAwMDAwMCJ9LHsidHlwZSI6MTAwMCwid2luZG93IjpbIjI1IiwiMzAiLCIwIiwiMTAwIl19XQ--

        float Iout_max = params.Iout_max;
        if (Vout >= params.Vout_top()) {
            if (!timeTopUntil)
                ESP_LOGI("chg", "Begin topping mode Iout_max %.2f (start=%.2f V, release=%.2f V, Vbat_max=%.2f)", params.Iout_top,
                         params.Vout_top(), params.Vout_top_release(), params.Vbat_max);
            timeTopUntil = nowMs + 1000 * 60 * 5;
        } else if (Vout <= params.Vout_top_release()) {
            if (timeTopUntil) ESP_LOGI("chg", "End topping mode, Vout %.2f", Vout);
            timeTopUntil = 0;
        }

        if (nowMs < timeTopUntil) {
            auto v0 = params.Vout_top_release(), v1 = params.Vout_top();
            auto i0 = params.Iout_max, i1 = params.Iout_top;
            // ramp up topping current when we reach release voltage
            if (Vout > v0)
                Iout_max = std::min(Iout_max, i0 - (i0 - i1) * sqrtf(min((Vout - v0) / (v1 - v0), 1.f)));
        }

        return Iout_max;
    }
     */

    float vcell_eoc = 3.6f; // cell end-of-charge voltage

    EWMA<volatile float, float> vbat_cap{60};

    void update(float vbat) {
        if (std::isfinite(vcell_max) and vcell_max > vcell_eoc and (vcell_max_t - wallClockUs()) < 200000000) {
            auto vbat_cap2 = vbat - (vcell_max - vcell_eoc);
            if (vbat_cap2 > params.Vbat_max) vbat_cap2 = params.Vbat_max;
            vbat_cap.add(vbat_cap2);
        } else {
            vbat_cap.add(params.Vbat_max);
        }
    }

    void reset() {
        vbat_cap.reset();
    }

    [[nodiscard]] float Vbat_max() const {
        if (vbat_cap.get() > 0) return vbat_cap.get();
        return params.Vbat_max;
    }
};