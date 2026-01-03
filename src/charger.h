#pragma once

#include "tele/mqtt.h"
#include "util.h"


// [V] cell voltage when to start the CV-phase (absorption)

struct BatChargerParams {
    float Vbat_max = NAN; // [V] max bat pack voltage = output voltage
    float Vbat_fallback = NAN; // [V] max bat pack voltage if bms data is n/a
    float Ibat_lim = NAN; // [A] Max bat charge current (Ibat = Iout - Iload)
    float Cbat = NAN; // [C] Battery capacity

    float cv_min = NAN; // where the termination line starts @Ibat=0 (LFP: 3.37V)
    float cv_eoc = NAN; // termination line ending (LFP: 3.65V@Ibat=0.05C)


    void load(const ConfFile &chargerConf) {
        Vbat_max = chargerConf.getFloat("vout_max", NAN);
        Vbat_fallback = chargerConf.getFloat("vout_max_fallback", NAN);
        cv_eoc = chargerConf.getFloat("cv_eoc", 3.6f);
        cv_min = chargerConf.getFloat("cv_min", 3.37f);
        Ibat_lim = chargerConf.getFloat("ibat_max", 40.f); // note: iout = ibat + iload
        Cbat = chargerConf.getFloat("bat_c", NAN);
    }
};

struct BatteryState {
    static constexpr auto VCELL_EXPIRATION_TIME_SEC = 180;

    volatile float vcell_high = 0; // voltage of highest cell reported by BMS
    volatile unsigned long vcell_high_t = 0; // timestamp of highest cell voltage

    EWMA<volatile float, float> vbat_avg{60}; // time-averaged pack voltage
    MeanAccumulator ibat_mean{}; // time-averaged battery current (from BMS)
    MeanAccumulator iout_mean{}; // time-averaged out current (from this charger)

    void setVcellHigh(const float &vcell_high_) {
        vcell_high = vcell_high_;
        vcell_high_t = wallClockUs();
    }

    [[nodiscard]] bool haveValidCellVoltage() const {
        return vcell_high > 0 and wallClockUs() - vcell_high_t < (VCELL_EXPIRATION_TIME_SEC * 1000000);
    }

    void update(float vbat, float iout) {
        vbat_avg.add(vbat);
        // note: iout = ibat + iload
        //if (iout > params.Ibat_lim * 0.005f)
        if (isfinite(iout))iout_mean.add(iout);
        else iout_mean.clear();
    }
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
    float vpack_pin = NAN;
    float ioutLim = NAN;

public:
    BatChargerParams params{};
    Li_ChgTerminationCondition termCond{params};
    BatteryState batSt{};


    explicit BatteryCharger() = default;

    void begin(const ConfFile &chargerConf) {
        params.load(chargerConf);
        termCond.v_term = params.cv_min;
    }


    void _updateTermination() {
        constexpr auto MEAN_NUM = 8;

        // update termination and current regulation
        if (batSt.ibat_mean.num >= MEAN_NUM and batSt.iout_mean.num >= MEAN_NUM) {
            const float ibat = batSt.ibat_mean.pop(), iout = batSt.iout_mean.pop();
            const float cv_float = (params.cv_min + params.cv_eoc) * .5f;
            const bool rampOff = batSt.vcell_high >= cv_float;

            // only the highest cell triggers termination
            if (termCond.update(batSt.vcell_high, ibat) or rampOff) {
                // this branch is likely not very useful. once a cell reaches termination voltage
                // we pin pack voltage so charger goes into CV mode. current will decrease, also decreasing termination
                // voltage
                /*
                // ramp down current on voltage interval [(cv_min+cv_eoc)/2, cv_eoc] (s -> [1, 0])
                auto s = min(max(0.f, (params.cv_eoc - batSt.vcell_high) / (params.cv_eoc - cv_float)), 1.f);

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
                         */
                // no need to set current limit. once we reach termination voltage, we go into CV mode
                // and current will fall off
                ioutLim = NAN;
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
        //  cut us off at max 3.65V, causing a voltage transient which we like to avoid. so never go beyond cv_eoc

        bool vcell_valid = batSt.haveValidCellVoltage();

        if (vcell_valid and batSt.vcell_high >= v_eoc) {
            constexpr auto OV_FEEDBACK_GAIN = 2; // 4
            // TODO run average filter on vPin (or vcell_high?)
            float vPin = fmin(batSt.vbat_avg.get(), vbat) - (batSt.vcell_high - v_eoc) * OV_FEEDBACK_GAIN;
            if (vPin < vpack_pin)
                ESP_LOGI("charger", "vpPin:=%.3fV (cvHigh=%.3f v_term=%.3f vbat_avg=%.3f)", vPin, batSt.vcell_high,
                     v_eoc, batSt.vbat_avg.get());
            vpack_pin = vPin;
        } else if (!vcell_valid && params.Vbat_fallback > 0) {
            ESP_LOGW("charger", "Cell Voltage n/a, fall back to VbatMax=%.3fV", params.Vbat_fallback);
            vpack_pin = params.Vbat_fallback;
        } else {
            vpack_pin = params.Vbat_max;
        }
    }


    void beginMqtt(const ConfFile &mqttConf) {
        auto topic = mqttConf.getString("cell_voltages_max_topic", "");
        if (!topic.empty()) {
            if (params.Vbat_fallback > 0) vpack_pin = params.Vbat_fallback;
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                batSt.setVcellHigh(strntof(dat, len));
                ESP_LOGD("charger",
                         "avg(vbat)=%.3fV cv_max(mqtt)=%.3fV cv_term=%.3fV vbat_lim=%.3fV vbat_max=%.3fV",
                         batSt.vbat_avg.get(),
                         batSt.vcell_high, termCond.v_term, Vout_max(), params.Vbat_max);
                _updatePackVoltagePinning();
            });
        }

        topic = mqttConf.getString("ibat_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                batSt.ibat_mean.add(strntof(dat, len));
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
        batSt.update(vbat, iout);
        _updateTermination();
        _updatePackVoltagePinning();
    }

    // TODO
    //void reset() {
    //    vpack_pin = NAN;
    //}

    [[nodiscard]] float Vout_max() const {
        float v_max = params.Vbat_max;
        if (vpack_pin > 0 and vpack_pin < v_max) v_max = vpack_pin;
        return v_max;
    }

    [[nodiscard]] float Iout_max() const {
        // TODO this should take the acquired ibat - iout delta into account
        // TODO2: is this really necessary?
        // ibat != iout (generally)

        auto lim = params.Ibat_lim;

        if (false) {
            // float current regulation
            const float cv_float = (params.cv_min + params.cv_eoc) * .5f;
            auto s = min(max(0.f, (params.cv_eoc - batSt.vcell_high) / (params.cv_eoc - cv_float)), 1.f);
            if (isfinite(s)) lim *= s;
        }

        // termination mode limit (keep Ibat~0 and supply loads)
        if (isfinite(ioutLim)) lim = min(lim, ioutLim);

        return lim;
    }
};
