#pragma once

#ifndef FUGU_BAT_V
#define FUGU_BAT_V NAN
#endif


struct BatChargerParams {
    float Vout_max = FUGU_BAT_V; //FUGU_BAT_V; //14.6 * 2;
    //float Iout_max = 27; // HW1: coil & fuse limited
    float Iout_max = 32; // HW2, backflow mosfet gets hot!

    float Vout_top() const { return Vout_max * 0.96f; } //0.975f;
    float Vout_top_release() const { return Vout_max * 0.94f; }
    float Iout_top = .5;
};

class BatteryCharger {

    unsigned long timeTopUntil = 0; // charger

    const BatChargerParams &params;

public:

    explicit BatteryCharger(const BatChargerParams &params) : params{params} {
    }

    float getToppingCurrent(float Vout) {
        auto nowMs = millis();

        // Topping-Mode reduces current with rising output-voltage
        // https://www.toolfk.com/online-plotter-frame#W3sidHlwZSI6MCwiZXEiOiIyMC0oMjAtMikqbWluKDEsKHgtMjcpLygyOC0yNykpKiouNSIsImNvbG9yIjoiIzAwMDAwMCJ9LHsidHlwZSI6MTAwMCwid2luZG93IjpbIjI1IiwiMzAiLCIwIiwiMTAwIl19XQ--

        float Iout_max = params.Iout_max;
        if (Vout >= params.Vout_top()) {
            if (!timeTopUntil)
                ESP_LOGI("chg", "Begin topping mode Iout_max %.2f (start=%.2f V, release=%.2f V, Vbat_max=%.2f)", params.Iout_top,
                         params.Vout_top(), params.Vout_top_release(), params.Vout_max);
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
};