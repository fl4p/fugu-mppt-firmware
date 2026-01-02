#pragma once

#include "tele/mqtt.h"
#include "util.h"

#ifndef FUGU_BAT_V
#define FUGU_BAT_V NAN
#endif


struct BatChargerParams {
    float Vbat_max = NAN; //FUGU_BAT_V; //14.6 * 2;
    float Ibat_lim = NAN; // [A] Max bat charge current (Ibat = Iout - Iload)
    float Cbat = NAN; // [C] Battery capacity
    float cv_eoc = NAN; // cell end-of-charge voltage. LFP: @Ibat=0.05C
    float cv_min = NAN; // where the termination line starts @Ibat=0
};

class Li_ChgTerminationCondition {
    /**
     * Charge termination condition for LFP (LiFePo4, Lithium Iron Phosphate) and other (?) Lithium Batteries
     * as described in https://nordkyndesign.com/charging-marine-lithium-battery-banks/
     * also see discussion https://github.com/fl4p/fugu-mppt-firmware/issues/31
     */

    const BatChargerParams &p;
    bool terminated = false;

public:
    float v_term; // termination cell voltage

    explicit operator bool() const { return terminated; }


    explicit Li_ChgTerminationCondition(const BatChargerParams &params)
        : p(params),
          v_term(params.cv_min) {
    }

    bool update(const volatile float &vcell_high, const float &ibat) {
        constexpr float k = 0.05f;
        // k is the ratio of cut-off current and capacity at EOC voltage (LFP: 3.65V, NCR: 4.2V)
        //  LFP:0.05
        // NCR: 0.02? https://www.orbtronic.com/content/Datasheet-specs-Sanyo-Panasonic-NCR18650GA-3500mah.pdf 67mA
        // EVE INR18650: k=0.033
        // k: the higher, the safer, so for now just leave it at 0.05
        float r = (p.cv_eoc - p.cv_min) / (k * p.Cbat); // some form of resistance (for 280Ah this is ~20mΩ)
        float vo = ibat * r;
        v_term = fminf(p.cv_min + fmax(0, vo), p.cv_eoc); // dont go beyond cv_eoc to avoid BMS cut-offr
        if (!terminated and ibat > 0 and vcell_high > p.cv_min + vo) {
            terminated = true;
        } else if (terminated and vcell_high < p.cv_min) {
            terminated = false;
        }
        ESP_LOGI("charger", "termination %hhu (iBat=%.2f, vcHigh=%.3f, vcTerm=%.3f, vcD=%.3f)", terminated, ibat,
                 vcell_high, v_term, v_term - vcell_high

                 // ", vhc*=%.3f", vcell_high - vo /* vhc* is the 'internal' cell voltage, I compensated */ useful?
        );
        return terminated;
    }
};

class BatteryCharger {
    EWMA<volatile float, float> vbat_avg{60}; // update freq is 3s
    float vpack_pin = NAN;

    MeanAccumulator ibat_mean{};
    MeanAccumulator iout_mean{};

    float ioutLim = NAN;

public:
    BatChargerParams params{};
    Li_ChgTerminationCondition termCond{params};

    volatile float vcell_max = 0;
    volatile unsigned long vcell_max_t = 0;


    explicit BatteryCharger() = default;

    void begin(ConfFile &chargerConf) {
        params.Vbat_max = chargerConf.getFloat("vout_max", NAN);
        params.cv_eoc = chargerConf.getFloat("cv_eoc", 3.6f);
        params.cv_min = chargerConf.getFloat("cv_min", 3.37f);
        params.Ibat_lim = chargerConf.getFloat("ibat_max", 40.f); // note: iout = ibat + iload
        params.Cbat = chargerConf.getFloat("bat_c", NAN);
        termCond.v_term = params.cv_min;
    }


    void _updateTermination() {
        constexpr auto MEAN_NUM = 8;

        // update termination and current regulation
        if (ibat_mean.num >= MEAN_NUM and iout_mean.num >= MEAN_NUM) {
            const float ibat = ibat_mean.pop(), iout = iout_mean.pop();
            const float cv_float = (params.cv_min + params.cv_eoc) * .5f;
            const bool rampOff = vcell_max >= cv_float;

            // the highest cell triggers termination
            if (termCond.update(vcell_max, ibat) or rampOff) {
                // ramp down current on voltage interval [(cv_min+cv_eoc)/2, cv_eoc] (s -> [1, 0])
                auto s = min(max(0.f, (params.cv_eoc - vcell_max) / (params.cv_eoc - cv_float)), 1.f);

                const float ioutLimTarget = fmax(
                    .0f, termCond
                             ? iout - ibat // we want ibat to be zero and only supply load current
                             : params.Ibat_lim * s + (iout - ibat) * (1.f - s) // ramp-off
                );

                constexpr float alpha = .3; // fade-in the limit exponentially to stabilize in case of many chargers
                ioutLim = iout * (1 - alpha) + ioutLimTarget * alpha;
                ESP_LOGI("charger", "%s: iBat=%.2f iOut=%.2f iOutLim:=%.2f iOutLimTgt=%.2f s=%.2f iBatLim=%.2f",
                         termCond ? "terminated" : "ramp-off", ibat, iout, ioutLim, ioutLimTarget, s,
                         params.Ibat_lim);
            } else {
                // unlimited
                ioutLim = NAN;
            }
        }
    }

    void _updatePackVoltagePinning(float vbat = INFINITY) {
        // EOC voltage regulation "pack voltage pinning"
        // once a cell reaches termination voltage we capture pack voltage and set it as max output voltage

        float v_eoc = fmin(params.cv_eoc, termCond.v_term);
        //  ^ v_eoc: theoretically we could go beyond cv_eoc if ibat is sufficiently high. however a "dumb" BMS will
        //  cut us off, causing a voltage transient which we like to avoid. so never go beyond cv_eoc

        if (vcell_max >= v_eoc  and (wallClockUs() - vcell_max_t) < 180000000) {
            constexpr auto OV_FEEDBACK = 2; // 4
            float vPin = fmin(vbat_avg.get(), vbat) - (vcell_max - v_eoc) * OV_FEEDBACK;
            if (vPin < vpack_pin)
                ESP_LOGI("charger", "vpPin:=%.3fV (cvHigh=%.3f v_term=%.3f vbat_avg=%.3f)", vPin, vcell_max,
                     v_eoc, vbat_avg.get());
            vpack_pin = vPin;
            if (vpack_pin > params.Vbat_max) vpack_pin = params.Vbat_max;
        } else {
            vpack_pin = params.Vbat_max;
        }
    }


    void beginMqtt(const ConfFile &mqttConf) {
        auto topic = mqttConf.getString("cell_voltages_max_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                this->vcell_max = strntof(dat, len);
                this->vcell_max_t = wallClockUs();
                ESP_LOGD("charger",
                         "avg(vbat)=%.3fV cv_max(mqtt)=%.3fV cv_term=%.3fV vbat_lim=%.3fV vbat_max=%.3fV",
                         this->vbat_avg.get(),
                         this->vcell_max, termCond.v_term, this->Vbat_max(), params.Vbat_max);
                _updatePackVoltagePinning();
            });

        topic = mqttConf.getString("ibat_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                ibat_mean.add(strntof(dat, len));
            });

        topic = mqttConf.getString("ibat_lim_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                params.Ibat_lim = strntof(dat, len);
                ESP_LOGI("charger", "Ibat_lim= %.3f A", params.Ibat_lim);
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
        // note: iout = ibat + iload
        //if (iout > params.Ibat_lim * 0.005f)
        if (isfinite(iout))iout_mean.add(iout);
        else iout_mean.clear();
        _updateTermination();
        _updatePackVoltagePinning();
    }

    void reset() {
        vpack_pin = NAN;
    }

    [[nodiscard]] float Vbat_max() const {
        if (vpack_pin > 0) return vpack_pin;
        return params.Vbat_max;
    }

    [[nodiscard]] float Ibat_max() const {
        // TODO this should take the acquired ibat - iout delta into account
        // ibat != iout (generally)

        auto lim = params.Ibat_lim;

        // float current regulation
        const float cv_float = (params.cv_min + params.cv_eoc) * .5f;
        auto s = min(max(0.f, (params.cv_eoc - vcell_max) / (params.cv_eoc - cv_float)), 1.f);
        if (isfinite(s)) lim *= s;

        // termination mode limit (keep Ibat~0 and supply loads)
        if (isfinite(ioutLim)) lim = min(lim, ioutLim);

        return lim;
    }
};
