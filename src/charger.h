#pragma once

#include <atomic>

#include "console.h"
#include "etc/coulomb_counter.h"
#include "etc/linear_glide.h"
#include "tele/mqtt.h"
#include "util.h"

struct BatChargerParams {
    float Vbat_max = NAN; // [V] max bat pack voltage = output voltage
    float Vbat_fallback = NAN; // [V] max bat pack voltage if bms data is n/a
    float Ibat_lim = NAN; // [A] Max bat charge current (Ibat = Iout - Iload)
    float Cbat = NAN; // [Ah] Effective pack capacity. For parallel packs use the summed Ah (e.g. 2P 280Ah → 560).

    float cv_min = NAN; // "float" where the termination line starts @Ibat=0 (LFP: 3.37V, LFP_longevity: 3.325V)
    float cv_eoc = NAN; // termination line ending (LFP_longevity: 3.5V @ Ibat = tail_c_rate * Cbat)
    float cv_ceiling = NAN; // [V] hard per-cell ceiling: latch termination if the highest cell reaches this,
    // regardless of charge current. Backstops cv_eoc on imbalanced packs (one cell runs away while current
    // is still high). Stay well below the BMS over-voltage cut-off. Default cv_eoc + 0.05.
    uint8_t n_cells = 0; // number of series cells, inferred from Vbat_max / cv_eoc
    float tail_c_rate = 0.05f; // [1/h] ratio of EOC tail current to capacity.
    // ^ LFP: 0.05. NCR (Sanyo NCR18650GA, 67mA on 3500mAh): ~0.02. EVE INR18650: 0.033. Higher = safer (terminates earlier).
    float recharge_dod = 0.20f; // DoD-since-EoC to release termination. LFP  ~0.20. See doc/Termination.md.
    float recharge_vfloor_band = 0.05f; // [V] cell-voltage drop below cv_min to release termination (fallback to DoD). See doc/Termination.md.
    float vout_offset_max = 0.6f; // [V] worst-case Vout-sensor error to tolerate during terminated float.
    float partial_charge = 0.f; // SoC fraction to stop at between periodic full charges (0 = always charge to full)
    uint32_t full_charge_interval_s = 7 * 86400; // [s] charge to full (BMS balancing) at least this often
    float bat_temp_min = 0.f; // [°C] no charging below (Li plating)
    float bat_temp_derate = 45.f; // [°C] charge current derates linearly from here ...
    float bat_temp_max = 55.f; // [°C] ... to zero here

    // True when Vbat_max was set explicitly via `vset` (not from config or auto-detect). The
    // persistent-OV auto-detect reset must not discard an explicitly commanded setpoint (issue #59).
    bool vbatMaxExplicit = false;
    // The EOC feedback loop drives the highest cell (BMS, accurate) down to its target by lowering vpack_pin;
    // this is how far below the nominal float floor (Vbat_fallback, expressed in this converter's possibly-
    // offset Vout frame) it may pull. Without it a converter that reads Vout high pins a full pack above EOC
    // and trickles current into it indefinitely. See doc/Termination.md.

    // Vbat_max doubles as the "battery not identified yet" flag (boot before auto-detect, a failed
    // detect, a bad `vset`). Test it here rather than for NAN at each use: a 0 or negative is just
    // as unusable, and treating it as a real ceiling clamps the OV threshold to ~0, which reads
    // every output voltage as an over-voltage and locks the converter out of ever starting.
    [[nodiscard]] bool haveVbatMax() const { return std::isfinite(Vbat_max) && Vbat_max > 0; }

    void load(const ConfFile &chargerConf) {
        Vbat_max = chargerConf.getFloat("vout_max", NAN, true);
        cv_eoc = chargerConf.getFloat("cv_eoc", 3.5f);
        cv_min = chargerConf.getFloat("cv_float", 3.325);
        assert_throw(cv_eoc >= cv_min, "cv_eoc must be >= cv_min");
        cv_ceiling = chargerConf.getFloat("cv_ceiling", cv_eoc + 0.05f);
        assert_throw(cv_ceiling >= cv_eoc, "cv_ceiling must be >= cv_eoc");
        assert_throw(std::isfinite(Vbat_max) && Vbat_max > 0, "vout_max must be a positive finite voltage");
        n_cells = (uint8_t) floorf(Vbat_max / cv_eoc);
        assert_throw(n_cells > 0, "vout_max must be >= cv_eoc (could not determine cell count)");
        float vout_fallback = n_cells * cv_min;
        Vbat_fallback = chargerConf.getFloat("vout_max_fallback", vout_fallback);
        ESP_LOGI("charger", "N_cells=%u (Vbat_max=%.2f / cv_eoc=%.3f), Vbat_fallback=%.3fV cv_ceiling=%.3fV",
                 (unsigned) n_cells, Vbat_max, cv_eoc, Vbat_fallback, cv_ceiling);

        Ibat_lim = chargerConf.getFloat("ibat_max", 20.f, true); // note: iout = ibat + iload
        Cbat = chargerConf.getFloat("bat_c", NAN, true); // larger->"safer" value, doesn't overcharge small bats
        // Optional: a PSU/non-battery topology omits it. NAN disables termination + DoD recharge
        // (downstream guards isfinite(Cbat)); warn instead of refusing to start.
        if (!(std::isfinite(Cbat) && Cbat > 0.f))
            ESP_LOGW("charger", "bat_c missing/invalid -> termination + EOC feedback disabled (Cbat=NAN)");
        tail_c_rate = chargerConf.getFloat("tail_c_rate", 0.05f);
        assert_throw(tail_c_rate > 0.f, "tail_c_rate must be > 0");
        recharge_dod = chargerConf.getFloat("recharge_dod", 0.20f);
        recharge_vfloor_band = chargerConf.getFloat("recharge_vfloor_band", 0.05f);
        assert_throw(recharge_vfloor_band >= 0.f, "recharge_vfloor_band must be >= 0");
        vout_offset_max = chargerConf.getFloat("vout_offset_max", 0.6f);
        assert_throw(vout_offset_max >= 0.f, "vout_offset_max must be >= 0");
        partial_charge = chargerConf.getFloat("partial_charge", 0.f);
        assert_throw(partial_charge >= 0.f && partial_charge < 1.f, "partial_charge must be in [0, 1)");
        float days = chargerConf.getFloat("full_charge_interval", 7.f);
        assert_throw(days > 0.f && days < 365.f, "full_charge_interval must be in (0, 365) days");
        full_charge_interval_s = (uint32_t) (days * 86400.f);
        bat_temp_min = chargerConf.getFloat("bat_temp_min", 0.f);
        bat_temp_derate = chargerConf.getFloat("bat_temp_derate", 45.f);
        bat_temp_max = chargerConf.getFloat("bat_temp_max", 55.f);
        assert_throw(bat_temp_min < bat_temp_derate && bat_temp_derate < bat_temp_max,
                     "need bat_temp_min < bat_temp_derate < bat_temp_max");
    }
};

struct BatteryState {
    static constexpr auto VCELL_EXPIRATION_TIME_SEC = 180;
    static constexpr uint16_t IBAT_MIN_SAMPLES = 8; // smoothing warm-up before ibat is published
    static constexpr uint8_t TEMP_SENSORS = 4; // max bat_temp_topic entries

    volatile float vcell_high = 0; // voltage of highest cell reported by BMS
    volatile uint32_t vcell_high_t = 0; // 32-bit lower half of wallClockUs(). 32-bit single-store/load is atomic across cores on Xtensa; 64-bit would not be.
    volatile uint32_t ibat_t = 0; // last ibat frame, same clock as vcell_high_t
    volatile float temp[TEMP_SENSORS]{NAN, NAN, NAN, NAN}; // [°C] pack sensors from BMS
    volatile uint32_t temp_t = 0;

    EWMA<volatile float, float> vout_avg{60}; // time-averaged pack voltage
    CoulombCounter coulombCounter{}; // Ah-since-last-full tracker, used for recharge hysteresis

    void setVcellHigh(const float &vcell_high_) {
        vcell_high = vcell_high_;
        vcell_high_t = static_cast<uint32_t>(wallClockUs());
    }

    void setTemp(uint8_t i, float t) {
        if (i >= TEMP_SENSORS || !(t > -40.f && t < 100.f)) return; // NAN and sensor-fault values

        temp[i] = t;
        temp_t = static_cast<uint32_t>(wallClockUs());
    }

    [[nodiscard]] bool haveTemp() const {
        for (auto t: temp) if (std::isfinite(t)) return true;
        return false;
    }

    [[nodiscard]] float tempMin() const {
        float m = INFINITY;
        for (auto t: temp) if (std::isfinite(t) && t < m) m = t;
        return m;
    }

    [[nodiscard]] float tempMax() const {
        float m = -INFINITY;
        for (auto t: temp) if (std::isfinite(t) && t > m) m = t;
        return m;
    }

    [[nodiscard]] bool haveValidCellVoltage() const {
        // Compare as 32-bit so the 71-minute wrap-around of vcell_high_t is handled correctly.
        return vcell_high > 0 and (static_cast<uint32_t>(wallClockUs()) - vcell_high_t) < (VCELL_EXPIRATION_TIME_SEC * 1000000ULL);
    }

    // producer (MQTT task): smooth ibat here and publish a single lock-free
    // snapshot the loop-task consumer only loads. NAN until IBAT_MIN_SAMPLES seen.
    void updateBatCurrent(const float &ibat) {
        _ibatEwma.add(ibat);
        ibat_t = static_cast<uint32_t>(wallClockUs());
        if (_ibatSamples < IBAT_MIN_SAMPLES) ++_ibatSamples;
        if (_ibatSamples >= IBAT_MIN_SAMPLES)
            _ibatSmoothed.store(_ibatEwma.get(), std::memory_order_relaxed);
        coulombCounter.updateBatCurrent(ibat);
    }

    // consumer (loop task): NAN until smoothing is warm
    [[nodiscard]] float ibatSmoothed() const { return _ibatSmoothed.load(std::memory_order_relaxed); }

    void update(float vout, float /*iout*/) {
        vout_avg.add(vout);
        // note: iout = ibat + iload, but only BMS-reported ibat is used for termination
    }

private:
    EWMA<float> _ibatEwma{IBAT_MIN_SAMPLES}; // producer-private smoothing
    uint16_t _ibatSamples = 0; // producer-private warm-up counter
    std::atomic<float> _ibatSmoothed{NAN}; // single writer: producer
};

class Li_ChgTerminationCondition {
    /**
     * Charge termination condition for LFP (LiFePo4, Lithium Iron Phosphate) and other (?) Lithium Batteries
     * as described in https://nordkyndesign.com/charging-marine-lithium-battery-banks/
     * also see discussion https://github.com/fl4p/fugu-mppt-firmware/issues/31
     *
     * when full charged, the battery voltage is pinned. todo regulate battery current towards 0?
     */

    static constexpr uint8_t VFLOOR_STREAK_REQ = 4; // consecutive sub-threshold BMS frames to release on voltage
    static constexpr uint8_t VCEIL_STREAK_REQ = 2; // consecutive over-ceiling frames to latch (ignore 1-frame I·R spikes)
    static constexpr uint8_t VTERM_STREAK_REQ = 2; // consecutive over-line frames to latch

    const BatChargerParams &p;
    bool terminated = false;
    float _v_term;
    uint8_t _vfloorStreak = 0; // sustained sub-threshold counter (transient I·R sag → 1-frame dips, ignore)
    uint8_t _ceilStreak = 0; // sustained over-ceiling counter (transient charge I·R spike → 1-frame, ignore)
    uint8_t _lineStreak = 0;

public:
    [[nodiscard]] float v_term() const { return _v_term; }

    explicit operator bool() const { return terminated; }


    explicit Li_ChgTerminationCondition(const BatChargerParams &params)
        : p(params),
          _v_term(params.cv_min) {
    }

    void reset() {
        terminated = false;
        _v_term = p.cv_min;
        _vfloorStreak = 0;
        _ceilStreak = 0;
        _lineStreak = 0;
    }

    // One call per BMS cell frame (the streaks count frames).
    bool update(float vcell_high, float ibat, float ahSinceFull) {
        // Termination line: at ibat = tail_c_rate * Cbat the cell sits at cv_eoc; at ibat = 0 it sits at cv_min.
        // r models the apparent cell resistance implied by that line (for 280Ah / 0.05 → ~20mΩ).
        // See doc/Termination.md.
        float r = (p.cv_eoc - p.cv_min) / (p.tail_c_rate * p.Cbat);
        float vo = ibat * r;
        _v_term = fminf(p.cv_min + fmaxf(0.f, vo), p.cv_eoc); // don't go beyond cv_eoc to avoid BMS cut-off
        float iBatFloor = -p.tail_c_rate * p.Cbat * 0.02f;

        // Hard per-cell ceiling backstop. The normal trigger compares against _v_term, which is capped at
        // cv_eoc so the cell is never intentionally pushed above the EOC voltage regardless of charge current.
        // The ceiling latches termination if the highest cell still reaches cv_ceiling (e.g. a transient I·R
        // spike or a misconfigured cv_eoc), current-independent; a short streak rejects a one-frame spike.
        bool ceilingLatch = false;
        if (!terminated and std::isfinite(p.cv_ceiling) and vcell_high >= p.cv_ceiling) {
            if (++_ceilStreak >= VCEIL_STREAK_REQ) ceilingLatch = true;
        } else {
            _ceilStreak = 0;
        }

        bool lineLatch = false;
        if (!terminated and ibat > iBatFloor and vcell_high > _v_term) {
            if (++_lineStreak >= VTERM_STREAK_REQ) lineLatch = true;
        } else {
            _lineStreak = 0;
        }

        if (!terminated and (ceilingLatch or lineLatch)) {
            terminated = true;
            _vfloorStreak = 0;
            _ceilStreak = 0;
            _lineStreak = 0;
            if (ceilingLatch)
                ESP_LOGW("charger", "Termination latched on cell ceiling: vcHigh(%.3f) >= cv_ceiling(%.3f) @ iBat=%.2f",
                         vcell_high, p.cv_ceiling, ibat);
        } else if (terminated and shouldRelease(vcell_high, ahSinceFull)) {
            terminated = false;
        }
        ESP_LOGD("charger", "term %u (iBat=%.2f vcHigh=%.3f vcTerm=%.3f vcD=%.3f ahSF=%.2f)",
                 terminated, ibat, vcell_high, _v_term, _v_term - vcell_high, ahSinceFull);
        return terminated;
    }

private:
    bool shouldRelease(float vcell_high, float ahSinceFull) {
        // Voltage-floor backstop in case the coulomb counter is misbehaving.
        // Require a sustained streak so a single I·R sag (50 A load spike →
        // vcell drops a few hundred mV for one BMS frame) doesn't release a
        // still-near-full pack.
        if (vcell_high < p.cv_min - p.recharge_vfloor_band) {
            if (++_vfloorStreak >= VFLOOR_STREAK_REQ) {
                ESP_LOGW("charger", "Termination release due to vcell_high(%.3f)<%.3f - %.3f (sustained %u frames)",
                         vcell_high, p.cv_min, p.recharge_vfloor_band, (unsigned) _vfloorStreak);
                _vfloorStreak = 0;
                return true;
            }
        } else {
            _vfloorStreak = 0;
        }
        if (std::isfinite(p.Cbat) && p.recharge_dod > 0.f && ahSinceFull > p.recharge_dod * p.Cbat) {
            ESP_LOGW("charger", "Termination release due to DoD: ahSinceFull(%.2f)>%.2f Ah (recharge_dod=%.2f)",
                     ahSinceFull, p.recharge_dod * p.Cbat, p.recharge_dod);
            return true;
        }
        return false;
    }
};

class BatteryCharger {
    float vpack_pin = NAN;
    float ioutLim = NAN; // [A] cap from the battery-temperature policy (NAN = none)

    // Smooths the OV-feedback pin against per-BMS-message noise. LFP's flat
    // discharge curve means mV-level cell-voltage noise gets multiplied by
    // OV_FEEDBACK_GAIN and would otherwise twitch the setpoint. Span chosen
    // to filter a single outlier without adding more than a few BMS-cycles
    // of latency.
    EWMA_N<4> _vPinFilt{};

    // 5 s linear glide of vpack_pin into Vbat_fallback when BMS data goes stale
    // (avoids a ~1 V step on the converter setpoint).
    LinearGlide _fallbackGlide{5'000'000};

    // 5 s linear glide of vpack_pin on termination transitions. Rising edge:
    // current pin → Vbat_fallback (LFP "float" voltage ≈ N_cells × cv_min) so
    // the converter holds the pack at a fixed reference and supplies the load
    // (i_bat ≈ 0). Falling edge: Vbat_fallback → Vbat_max so the recharge ramp
    // doesn't step the converter setpoint.
    LinearGlide _floatGlide{5'000'000};
    bool _wasTerminated = false;

    // Integrator step gate: advance vpack_pin only on a fresh BMS frame so the
    // step rate matches the cell-voltage update rate (otherwise loopLF at 1 Hz
    // runs the integrator 15× per BMS cycle and overshoots).
    uint32_t _lastBmsFrameUs = 0;

    uint32_t _lastTermFrameUs = 0; // termCond runs once per cell frame
    uint32_t _lastIbatFrameUs = 0; // load-following steps once per ibat frame

    bool _bmsCellSource = false; // a BMS cell-voltage topic was configured (termination can be evaluated)
    bool _termDecided = false; // termCond has been evaluated at least once (needs cell voltage + warm ibat)
    bool _coldBlocked = false; // pack below bat_temp_min: hold ibat at 0 (loads are still served)
    uint16_t _tempStaleTicks = 0; // update() ticks since the last temperature frame
    uint32_t _lastTempFrameUs = 0;
    bool _partialHold = false; // Ah ceiling reached: hold SoC by following the load (ibat -> 0)
    bool _wasPartial = false;
    time_us _lastFullUs = 0; // last termination latch (0 = none since boot)

    static constexpr float BAT_TEMP_HYST = 2.f; // [°C] cold-block release band
    static constexpr uint16_t TEMP_EXPIRE_TICKS = 3600; // ~1 h without a frame: policy off (as without a sensor)
    static constexpr float IOUT_LIM_FLOOR = 0.25f; // the CC limiter normalizes by its setpoint; never 0

public:
    BatChargerParams params{};
    Li_ChgTerminationCondition termCond{params};
    BatteryState batSt{};


    explicit BatteryCharger() = default;

    void begin(const ConfFile &chargerConf) {
        params.load(chargerConf);
        termCond.reset(); // propagate just-loaded cv_min into termCond's v_term (was NAN at ctor time)
    }


    void _updateTermination() {
        // Gate on the battery-current snapshot from the BMS/MQTT producer. It is
        // already smoothed over IBAT_MIN_SAMPLES there; no need to pace on the
        // converter's own iout (which is ibat + iload and cannot be used alone).
        if (!batSt.haveValidCellVoltage()) return;

        float ibat = batSt.ibatSmoothed();
        if (!std::isfinite(ibat)) return; // smoothing not warm yet

        uint32_t frame = batSt.vcell_high_t;
        if (frame == _lastTermFrameUs) return; // streaks inside termCond count cell frames, not loop ticks
        _lastTermFrameUs = frame;

        bool wasTerm = bool(termCond);
        termCond.update(batSt.vcell_high, ibat, batSt.coulombCounter.ahSinceFull());
        _termDecided = true; // termCond now reflects a real evaluation (cell voltage + warm ibat)
        if (!wasTerm && bool(termCond)) {
            // Rising edge — pack is full, re-zero the coulomb counter so the
            // recharge_dod hysteresis measures against this full point.
            batSt.coulombCounter.markFull();
            _lastFullUs = wallClockUs();
            ESP_LOGI("charger", "Termination latched: ahSinceFull reset to 0");
        }
    }

    // Battery-temperature policy. No sensor data (or none for ~1 h): no limit, as without the feature.
    // Cold: charge current is held at 0 by the load-follower (the converter still serves the loads;
    // discharging a cold pack is fine). Hot: the *battery* current limit derates, so the output limit
    // is the estimated load current (iout - ibat) plus the derated ibat_max.
    void _updateTempLimit(float iout) {
        ioutLim = NAN;
        uint32_t frame = batSt.temp_t;
        if (frame != _lastTempFrameUs) { _lastTempFrameUs = frame; _tempStaleTicks = 0; }
        else if (_tempStaleTicks < UINT16_MAX) ++_tempStaleTicks;
        if (!batSt.haveTemp() || _tempStaleTicks > TEMP_EXPIRE_TICKS) {
            if (_coldBlocked) ESP_LOGW("charger", "pack temperature stale, cold block dropped");
            _coldBlocked = false;
            return;
        }
        float tmin = batSt.tempMin(), tmax = batSt.tempMax();
        float release = params.bat_temp_min + BAT_TEMP_HYST;
        if (tmin < (_coldBlocked ? release : params.bat_temp_min)) {
            if (!_coldBlocked)
                ESP_LOGW("charger", "pack %.1f°C < %.1f°C: charging blocked until %.1f°C", tmin, params.bat_temp_min, release);
            _coldBlocked = true;
            if (!std::isfinite(batSt.ibatSmoothed())) ioutLim = IOUT_LIM_FLOOR; // no ibat to follow: idle
            return;
        }
        if (_coldBlocked) ESP_LOGI("charger", "pack %.1f°C: charging released", tmin);
        _coldBlocked = false;
        if (tmax > params.bat_temp_derate) {
            float scale = fmaxf(0.f, (params.bat_temp_max - tmax) / (params.bat_temp_max - params.bat_temp_derate));
            float ibat = batSt.ibatSmoothed();
            float iload = (std::isfinite(ibat) && std::isfinite(iout)) ? fmaxf(0.f, iout - ibat) : 0.f;
            ioutLim = fmaxf(iload + params.Ibat_lim * scale, IOUT_LIM_FLOOR);
        }
    }

    [[nodiscard]] uint16_t tempStaleS() const { return _tempStaleTicks; }

    // Partial-charge ceiling: after a full charge, stop at partial_charge (Ah-counted from that
    // full) and hold there by load-following until the pack has discharged recharge_dod below the
    // ceiling, or until full_charge_interval expires (then charge to full again for balancing).
    // Needs a full event since boot: the Ah counter is a deficit-since-full, unknown before that.
    void _updatePartialHold() {
        bool enabled = params.partial_charge > 0.f && std::isfinite(params.Cbat) && _lastFullUs != 0
                       && (wallClockUs() - _lastFullUs) < (time_us) params.full_charge_interval_s * 1000000ULL;
        if (!enabled || bool(termCond)) {
            if (_partialHold) ESP_LOGI("charger", "partial hold off (%s)", bool(termCond) ? "terminated" : "full charge due");
            _partialHold = false;
            return;
        }
        float ah = batSt.coulombCounter.ahSinceFull();
        float ceilAh = (1.f - params.partial_charge) * params.Cbat;
        if (!_partialHold && ah <= ceilAh) {
            _partialHold = true;
            ESP_LOGI("charger", "partial hold: %.0f%% reached (%.1f Ah since full)", params.partial_charge * 100.f, ah);
        } else if (_partialHold && ah > ceilAh + params.recharge_dod * params.Cbat) {
            _partialHold = false;
            ESP_LOGI("charger", "partial hold released: %.1f Ah since full", ah);
        }
    }

    // Load-following: nudge vpack_pin so the BMS pack current goes to ibatTarget (~0: the converter
    // covers the load, nothing goes into the pack). One capped step per ibat frame; ibat is already
    // smoothed over IBAT_MIN_SAMPLES frames, so the step must stay small against the stiff pack
    // (tens of mΩ including cables) or the loop hunts. The floor is the termination-release voltage
    // minus the Vout tolerance, where a pack at partial SoC rests. Only steps while this converter
    // drives the bus: a disabled converter (night) would otherwise integrate the house load up to
    // Vbat_max and charge for minutes at dawn.
    void _loadFollowStep(float ibat, float ibatTarget, bool authority) {
        constexpr float DEADBAND_A = 0.2f, GAIN_V_PER_A = 0.004f, STEP_MAX_V = 0.02f;
        uint32_t frame = batSt.ibat_t;
        if (!authority || frame == _lastIbatFrameUs) return;
        _lastIbatFrameUs = frame;
        float floor = params.n_cells * (params.cv_min - params.recharge_vfloor_band) - params.vout_offset_max;
        float err = ibat - ibatTarget;
        float step = fabsf(err) > DEADBAND_A ? fminf(fmaxf(-err * GAIN_V_PER_A, -STEP_MAX_V), STEP_MAX_V) : 0.f;
        vpack_pin = fminf(fmaxf(vpack_pin + step, floor), params.Vbat_max);
    }

    // Partial hold: trim towards the Ah ceiling so a BMS current offset inside the deadband can't
    // walk the SoC away over a week (0.2 A is 34 Ah / 12 % in 7 days on 280 Ah). ±1 A at 2 Ah off.
    [[nodiscard]] float _holdIbatTarget() const {
        if (!_partialHold) return 0.f;
        float ceilAh = (1.f - params.partial_charge) * params.Cbat;
        return fminf(fmaxf((batSt.coulombCounter.ahSinceFull() - ceilAh) * 0.5f, -1.f), 1.f);
    }

    void _updatePackVoltagePinning(bool voutAuthority = true, float vbat = INFINITY) {
        // Pack-voltage-pinning regimes (first match wins):
        //   1) EOC feedback — cells above v_eoc: closed loop on vcell_high → v_eoc.
        //      Doubles as the over-target corrector during terminated float; pulls
        //      vpack_pin down until vcell_high reaches v_eoc (≈ cv_min at ibat≈0),
        //      self-correcting a Vout-ADC offset up to params.vout_offset_max (the
        //      depth it may pull below the nominal float floor). Suppressed when balancingMode
        //      is on, to let a passive balancer act at a fixed float voltage
        //      (unhealthy for an extended period of time, so only when explicitly
        //      requested).
        //   2) Terminated float — open-loop hold at Vbat_fallback.
        //   3) BMS-stale — glide to Vbat_fallback.
        //   4) Bulk — push to Vbat_max (or ride the falling-edge glide).
        // Termination transitions are glided to avoid stepping the setpoint.

        bool balancingMode = false;

        bool batDataOk = batSt.haveValidCellVoltage() and std::isfinite(params.Cbat);
        bool nowTerm = bool(termCond);
        auto nowUs = wallClockUs();

        // Absorption target: hold the highest cell at cv_eoc and let the current taper; the
        // termination line decides when it's done. Once terminated, hold it at cv_min (no trickle).
        // The target must not follow v_term(ibat): lowering the pin drops ibat, which lowers v_term,
        // which lowers the pin again, until ibat=0 and termination latches at cv_min (premature
        // termination seen on fry/flat 2026-07).
        float v_eoc = nowTerm ? params.cv_min : params.cv_eoc;

        float ibat = batSt.ibatSmoothed();
        bool hold = std::isfinite(ibat) && ((_partialHold && batDataOk) || _coldBlocked);
        if (hold != _wasPartial) {
            if (hold) {
                // start at the bus voltage the pack sits at now (the bulk pin is far above it)
                float bus = batSt.vout_avg.get();
                if (std::isfinite(bus)) vpack_pin = std::isfinite(vpack_pin) ? fminf(vpack_pin, bus) : bus;
                ESP_LOGI("charger", "hold (%s): load-following from vpPin %.3fV", _coldBlocked ? "cold" : "partial", vpack_pin);
            } else {
                float from = std::isfinite(vpack_pin) ? vpack_pin : params.Vbat_max;
                _floatGlide.start(from, params.Vbat_max, nowUs);
                ESP_LOGI("charger", "hold end, gliding vpPin %.3fV -> %.3fV", from, params.Vbat_max);
            }
            _wasPartial = hold;
        }
        if (hold) {
            _vPinFilt.reset();
            _fallbackGlide.reset();
            _floatGlide.reset();
            _wasTerminated = false;
            if (std::isnan(vpack_pin)) vpack_pin = params.Vbat_max;
            _loadFollowStep(ibat, _holdIbatTarget(), voutAuthority);
            return;
        }

        // No authority over the (shared) bus — floored / ~0 output, no plant response.
        if (!voutAuthority && batDataOk) {
            if (nowTerm) {
                // Terminated/full pack: yield LOW. Climbing to Vbat_max here would push current into a
                // full pack and limit-cycle against the EOC re-clamp (sibling converter already holds the
                // bus and regulates termination). Pin at the EOC float floor so this converter targets the
                // resting full-pack voltage and sits at ~0 current; it re-takes the bus once termination
                // releases (recharge) via the branch below.
                float floor = params.Vbat_fallback - params.vout_offset_max;
                if (std::isnan(vpack_pin) || vpack_pin > floor + 0.01f)
                    ESP_LOGW("charger", "no Vout authority (terminated): yield %.3f -> %.3f", vpack_pin, floor);
                vpack_pin = floor;
                _vPinFilt.reset();
                _fallbackGlide.reset();
                _floatGlide.reset();
                _wasTerminated = nowTerm;
            } else {
                // Not full: release the pin upward so this converter keeps a real target and can climb
                // back to re-take the bus; the controlling converter still regulates it meanwhile.
                releaseVoutPinning("no Vout authority");
            }
            return;
        }

        if (nowTerm != _wasTerminated) {
            float from = std::isfinite(vpack_pin) ? vpack_pin : params.Vbat_fallback;
            float to = nowTerm ? params.Vbat_fallback : params.Vbat_max;
            _floatGlide.start(from, to, nowUs);
            ESP_LOGI("charger", "Term %s, gliding vpPin %.3fV -> %.3fV over %ums",
                     nowTerm ? "latched" : "released", from, to,
                     (unsigned) (_floatGlide.durationUs() / 1000));
            _wasTerminated = nowTerm;
        }

        bool eocFeedback = voutAuthority && batDataOk && batSt.vcell_high >= v_eoc && !(balancingMode && nowTerm);

        if (eocFeedback) {
            // Integrate the cell-overvoltage error into vpack_pin itself; drives
            // vcell_high → v_eoc with no steady-state floor. Step only when a new
            // BMS frame arrives so the per-frame drop = gain·err regardless of
            // loopLF cadence; the EWMA on vPin_raw still damps single-frame outliers.
            _fallbackGlide.reset();
            _floatGlide.reset();
            constexpr float OV_FEEDBACK_GAIN = 1.f;
            uint32_t bmsFrameUs = batSt.vcell_high_t;
            bool newBmsFrame = bmsFrameUs != _lastBmsFrameUs;
            if (newBmsFrame || std::isnan(vpack_pin)) {
                _lastBmsFrameUs = bmsFrameUs;
                float base = std::isfinite(vpack_pin) ? vpack_pin : fminf(batSt.vout_avg.get(), vbat);
                // Floor below the nominal float voltage by the tolerated Vout offset so this BMS-driven loop
                // can still pull the highest cell down to v_eoc when our Vout reads high (otherwise the float
                // floor, in our offset Vout frame, pins a full pack above EOC and trickles current into it).
                // The per-cell voltage error must be scaled by the number of cells to become a pack-voltage correction.
                float vPin_raw = fmaxf(base - (batSt.vcell_high - v_eoc) * params.n_cells * OV_FEEDBACK_GAIN,
                                       params.Vbat_fallback - params.vout_offset_max);
                _vPinFilt.add(vPin_raw);
                float vPin = _vPinFilt.get();
                if (std::isnan(vpack_pin) or vPin < vpack_pin - 0.01f)
                    ESP_LOGI("charger", "update vpPin:=%.3fV (raw=%.3f cvHigh=%.3f v_eoc=%.3f vbat_avg=%.3f)",
                             vPin, vPin_raw, batSt.vcell_high, v_eoc, batSt.vout_avg.get());
                vpack_pin = vPin;
            }
        } else if (nowTerm && batDataOk) {
            // balancing mode / float
            // terminated float: hold an absolute target, ignore feedback/EWMA
            // this keep LFP float, with a small charge current, considered unhealthy for an extended period of time
            _vPinFilt.reset();
            _fallbackGlide.reset();
            vpack_pin = _floatGlide.value(nowUs);
        } else if (!batDataOk && params.Vbat_fallback >= 0) {
            // missing bat data, and we have a fallback -> glide there
            _vPinFilt.reset();
            _floatGlide.reset();
            if (!_fallbackGlide.active()) {
                // entering fallback — capture current pin as the glide origin
                float from = std::isfinite(vpack_pin) ? vpack_pin : params.Vbat_fallback;
                _fallbackGlide.start(from, params.Vbat_fallback, nowUs);
                auto what = !batSt.haveValidCellVoltage() ? "Cell Voltage" : "Pack Capacity";
                ESP_LOGW("charger", "%s n/a, gliding vpPin %.3fV -> %.3fV over %ums",
                         what, from, params.Vbat_fallback,
                         (unsigned) (_fallbackGlide.durationUs() / 1000));
            }
            vpack_pin = _fallbackGlide.value(nowUs);
        } else {
            // bulk charging or (missing batData and no fallback)
            _vPinFilt.reset();
            _fallbackGlide.reset();
            // ride the falling-edge glide if it's still ramping, otherwise jump
            vpack_pin = _floatGlide.active() ? _floatGlide.value(nowUs) : params.Vbat_max;
        }
    }


    void beginMqtt(const ConfFile &mqttConf) {
        auto topic = mqttConf.getString("cell_voltages_max_topic", "");
        if (!topic.empty()) {
            _bmsCellSource = true;
            if (params.Vbat_fallback > 0)
                vpack_pin = params.Vbat_fallback;
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                batSt.setVcellHigh(strntof(dat, len));
                ESP_LOGD("charger",
                         "avg(vbat)=%.3fV cv_max(mqtt)=%.3fV cv_term=%.3fV vbat_lim=%.3fV vbat_max=%.3fV",
                         batSt.vout_avg.get(),
                         batSt.vcell_high, termCond.v_term(), Vout_max(), params.Vbat_max);
            });
        }

        topic = mqttConf.getString("ibat_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                float i = strntof(dat, len);
                if (!std::isfinite(i)) {
                    LOG_VALUE_IGNORED("charger", "Ibat", len, dat);
                    return;
                }
                batSt.updateBatCurrent(i);
            });

        // up to TEMP_SENSORS comma-separated topics (batmon-ha: <dev>/temperatures/1,<dev>/temperatures/2)
        topic = mqttConf.getString("bat_temp_topic", "");
        for (uint8_t i = 0; i < BatteryState::TEMP_SENSORS && !topic.empty(); ++i) {
            auto comma = topic.find(',');
            std::string one = topic.substr(0, comma);
            topic = comma == std::string::npos ? "" : topic.substr(comma + 1);
            auto b = one.find_first_not_of(' '), e = one.find_last_not_of(' ');
            if (b == std::string::npos) continue;
            one = one.substr(b, e - b + 1);
            MQTT.subscribeTopic(one, [this, i](const char *dat, int len) {
                float t = strntof(dat, len);
                if (!std::isfinite(t)) {
                    LOG_VALUE_IGNORED("charger", "bat_temp", len, dat);
                    return;
                }
                batSt.setTemp(i, t);
            });
        }

        topic = mqttConf.getString("ibat_lim_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                float v = strntof(dat, len);
                if (!std::isfinite(v) || v < 0) {
                    LOG_VALUE_IGNORED("charger", "Ibat_lim", len, dat);
                    return;
                }
                auto prev = params.Ibat_lim;
                params.Ibat_lim = v;
                ESP_LOGI("charger", "Ibat_lim= %.3f A (was %.3f)", params.Ibat_lim, prev);
            });
    }

    void update(float vout, float iout, bool voutAuthority = true) {
        batSt.update(vout, iout);
        _updateTermination();
        _updateTempLimit(iout);
        _updatePartialHold();
        _updatePackVoltagePinning(voutAuthority);
    }

    [[nodiscard]] bool partialHold() const { return _partialHold; }
    [[nodiscard]] bool chargeBlocked() const { return _coldBlocked; }
    // the pack wants no charge: terminated, at the partial ceiling, or too cold
    [[nodiscard]] bool chargeHold() const { return bool(termCond) || _partialHold || _coldBlocked; }
    [[nodiscard]] time_us lastFullUs() const { return _lastFullUs; }

    // Release the EOC vpack_pin latch upward to Vbat_max and clear the filter/glide state. Used when
    // this converter has no authority over a (shared) bus so it stops targeting a ratcheted-down Vout
    // and can climb back to re-take the bus, and as the in-place recovery for a CV-floor lockup. Cells
    // are still monitored on these paths, so the EOC feedback / termination re-clamp once it climbs;
    // releasing to Vbat_fallback (≈ the resting pack voltage on a shared bus) would instead pin it at
    // the bus voltage and throttle a battery that isn't full. Returns true if moved.
    bool releaseVoutPinning(const char *why = nullptr) {
        float target = std::isfinite(params.Vbat_max) ? params.Vbat_max : params.Vbat_fallback;
        bool changed = !std::isfinite(vpack_pin) || fabsf(vpack_pin - target) > 0.01f
                       || _fallbackGlide.active() || _floatGlide.active();
        float prev = vpack_pin;
        vpack_pin = target;
        _vPinFilt.reset();
        _fallbackGlide.reset();
        _floatGlide.reset();
        _wasTerminated = false; // force a fresh float glide on re-entry instead of jumping to the stale target
        if (changed)
            ESP_LOGW("charger", "release Vout pinning%s%s%s: %.3f -> %.3f",
                     why ? " [" : "", why ? why : "", why ? "]" : "", prev, target);
        return changed;
    }

    [[nodiscard]] float Vout_max() const {
        float v_max = params.Vbat_max;
        if (vpack_pin > 0 and vpack_pin < v_max) v_max = vpack_pin;
        return v_max;
    }

    // A BMS cell-voltage topic is configured, so termCond reflects the real pack state once a frame
    // has arrived. Lets the sweep trigger defer a boot sweep until that first frame instead of pulsing
    // a possibly-full pack. False when no BMS is wired (then termination can't be known from cells).
    [[nodiscard]] bool hasBmsCellSource() const { return _bmsCellSource; }

    // termCond has run at least once, so it reflects the real pack state. Distinct from
    // hasBmsCellSource(): a BMS topic may be subscribed but its cell-voltage/ibat frames not yet in.
    [[nodiscard]] bool terminationDecided() const { return _termDecided; }

    [[nodiscard]] float Iout_max() const {
        // TODO this should take the acquired ibat - iout delta into account
        // TODO2: is this really necessary?
        // ibat != iout (generally)

        auto lim = params.Ibat_lim;

        // termination mode limit (keep Ibat~0 and supply loads)
        if (std::isfinite(ioutLim)) lim = min(lim, ioutLim);

        return lim;
    }
};
