#pragma once

#include "tele/mqtt.h"
#include "util.h"

#ifndef FUGU_BAT_V
#define FUGU_BAT_V NAN
#endif


struct BatChargerParams {
    float Vbat_max = NAN; //FUGU_BAT_V; //14.6 * 2;
    float Iout_max = NAN; // HW1: coil & fuse limited
    float vcell_eoc = 3.55f; // cell end-of-charge voltage
    float vcell_float = 3.5f; // cell end-of-charge voltage
    float cv_min = 3.37;

    //float vcell_stop = 3.59f; // cell end-of-charge voltage

    //float Iout_max = 32; // HW2, backflow mosfet gets hot!

    //float Vout_top() const { return Vbat_max * 0.98f; } //0.96 0.975f;
    //float Vout_top_release() const { return Vbat_max * 0.94f; }

    // float Iout_top = .5;
};

class LFP_FloatMode {
    bool floatMode = false;

    const BatChargerParams &p;

public:
    explicit LFP_FloatMode(const BatChargerParams &params) : p(params) {
    }

    bool update(const volatile float &vcell_max, const float &ibat) {
        // check charge termination criteria
        // https://github.com/fl4p/fugu-mppt-firmware/issues/31
        float k = 0.05f; // LFP
        float cv = vcell_max - ibat * (p.vcell_eoc - p.cv_min);
        if (!floatMode and cv / k > p.cv_min) {
            floatMode = true;
            //ESP_LOGI("charger", "float mode %hhu (ibat=%.3f)", floatMode, ibat);
        } else if (floatMode and vcell_max < p.cv_min) {
            floatMode = false;
            //ESP_LOGI("charger", "float mode %hhu (ibat=%.3f)", floatMode, ibat);
        }
        ESP_LOGI("charger", "float mode %hhu (ibat=%.3f, vcell_max=%.3f, cv=%.3f, cv_min=%.3f)", floatMode, ibat, vcell_max, cv, p.cv_min);    
        return floatMode;
    }
};

class BatteryCharger {
    // unsigned long timeTopUntil = 0; // charger
    EWMA<volatile float, float> vbat_avg{60}; // update freq is 3s
    float vbat_cap = NAN;

    MeanAccumulator ibat_mean;
    MeanAccumulator iout_mean;

    float ioutLim = NAN;

public:
    BatChargerParams params{};
    LFP_FloatMode floatMode{params};

    volatile float vcell_max = 0;
    volatile unsigned long vcell_max_t = 0;


    explicit BatteryCharger() = default;

    void begin(ConfFile &chargerConf) {
        params.Vbat_max = chargerConf.getFloat("vout_max", NAN);
        params.vcell_eoc = chargerConf.getFloat("cell_voltage_eoc", 3.6f);
        params.vcell_float = chargerConf.getFloat("cell_voltage_float", 3.55f);
        params.Iout_max = chargerConf.getFloat("ibat_max", 40.f); // iout = ibat + iload
    }


    void _update(float vbat = INFINITY) {
        if (vcell_max > params.vcell_eoc /* and (vcell_max_t - wallClockUs()) < 180000000*/) {
            vbat_cap = fmin(vbat_avg.get(), vbat) - (vcell_max - params.vcell_eoc) * 4;
            if (vbat_cap > params.Vbat_max) vbat_cap = params.Vbat_max;
        } else {
            vbat_cap = params.Vbat_max;
        }

        // LFP cutoff (see https://github.com/fl4p/fugu-mppt-firmware/issues/31 )
        if (ibat_mean.num >= 8) {
            float ibat = ibat_mean.pop(), iout = iout_mean.pop();
            if (floatMode.update(vcell_max, ibat)) {
                // the highest cell triggers termination
                // now compute the new output current limit (which is not necessarily the battery current)
                float alpha = .3; // fade-in the limit exponentially to stabilize in case of many chargers
                ioutLim = iout * (alpha - 1) + (iout - ibat) * alpha;
                ESP_LOGI("charger", "float update: ibat=%.3f iout=%.3f iout_lim:=%.3f", ibat, iout, ioutLim);
            } else {
                // unlimited
                ioutLim = NAN;
            }
        }
    }


    void beginMqtt(const ConfFile &mqttConf) {
        // TODO conffile
        auto topic = mqttConf.getString("cell_voltages_max_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                this->vcell_max = strntof(dat, len);
                this->vcell_max_t = wallClockUs();
                ESP_LOGI("charger",
                         "avg(vbat)=%.4fV vcell_max(mqtt)=%.4fV vcell_eoc=%.4fV vbat_cap=%.4fV vbat_max=%.4fV",
                         this->vbat_avg.get(),
                         this->vcell_max, params.vcell_eoc, this->Vbat_max(), params.Vbat_max);
                _update();
            });

        topic = mqttConf.getString("ibat_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                ibat_mean.add(strntof(dat, len));
            });

        topic = mqttConf.getString("ibat_lim_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                params.Iout_max = strntof(dat, len);
                ESP_LOGI("charger", "I_lim %.3f A", params.Iout_max);
                _update();
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


    void update(float vbat, float iout) {
        vbat_avg.add(vbat);
        iout_mean.add(iout);
        _update();
    }

    void reset() {
        vbat_cap = NAN;
    }

    [[nodiscard]] float Vbat_max() const {
        if (vbat_cap > 0) return vbat_cap;
        return params.Vbat_max;
    }

    [[nodiscard]] float Ibat_max() const {
        // TODO this should take the acquired ibat - iout delta into account
        // ibat != iout (generally)
        
        auto lim = params.Iout_max;

        // eoc regulation
        auto s = min(max(0.f, (params.vcell_eoc - vcell_max) / (params.vcell_eoc - params.vcell_float)), 1.f);
        if (isfinite(s)) lim *= s;

        // float mode limit
        if (isfinite(ioutLim)) lim = min(lim, ioutLim);

        return lim;
    }
};
