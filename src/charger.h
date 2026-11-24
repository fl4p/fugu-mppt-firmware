#pragma once

#include <atomic>
#include <cstddef>
#include <cstring>

#include "console.h"
#include "etc/coulomb_counter.h"
#include "etc/linear_glide.h"
#include "out_impedance.h"
#include "tele/mqtt.h"
#include "util.h"
#include "wire_guard.h"

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
    float recharge_dod = 0.20f; // DoD-since-EoC to release termination. LFP  ~0.20. See website/docs/guide/charging/termination.md.
    float recharge_vfloor_band = 0.05f; // [V] cell-voltage drop below cv_min to release termination (fallback to DoD). See website/docs/guide/charging/termination.md.
    uint32_t cv_hold_max_s = 0; // [s] CV hold at cv_eoc before termination, max duration (0 = off: latch on the line)
    uint32_t tail_hold_s = 180; // [s] the hold latches once ibat stays below the tail this long
    float cv_hold_band = 0.02f; // [V] cell band below cv_eoc for the hold's tail test
    float vout_offset_max = 0.6f; // [V] worst-case Vout-sensor error to tolerate during terminated float.
    float partial_charge = 0.f; // SoC fraction to stop at between periodic full charges (0 = always charge to full)
    uint32_t full_charge_interval_s = 7 * 86400; // [s] charge to full (BMS balancing) at least this often
    uint32_t balance_interval_s = 0; // [s] top-balancing run at least this often (0 = off)
    uint32_t balance_hold_s = 7200; // [s] in-band time at cv_eoc that completes a balance run
    float balance_delta = 0.01f; // [V] cell delta that completes a run early
    float holiday_charge = 0.f; // SoC ceiling once the pack barely cycles (0 = auto mode off)
    float holiday_ibat = NAN; // [A] quiet day: mean discharge current below this
    float holiday_band = 0.1f; // SoC band below holiday_charge before the hold releases
    uint8_t holiday_days = 3; // consecutive quiet days to enter holiday mode
    float bat_temp_min = 0.f; // [°C] no charging below (Li plating)
    float bat_temp_derate = 45.f; // [°C] charge current derates linearly from here ...
    float bat_temp_max = 55.f; // [°C] ... to zero here

    // True when Vbat_max was set explicitly via `vset` (not from config or auto-detect). The
    // persistent-OV auto-detect reset must not discard an explicitly commanded setpoint (issue #59).
    bool vbatMaxExplicit = false;
    // The EOC feedback loop drives the highest cell (BMS, accurate) down to its target by lowering vpack_pin;
    // this is how far below the nominal float floor (Vbat_fallback, expressed in this converter's possibly-
    // offset Vout frame) it may pull. Without it a converter that reads Vout high pins a full pack above EOC
    // and trickles current into it indefinitely. See website/docs/guide/charging/termination.md.

    // Vbat_max doubles as the "battery not identified yet" flag (boot before auto-detect, a failed
    // detect, a bad `vset`). Test it here rather than for NAN at each use: a 0 or negative is just
    // as unusable, and treating it as a real ceiling clamps the OV threshold to ~0, which reads
    // every output voltage as an over-voltage and locks the converter out of ever starting.
    [[nodiscard]] bool haveVbatMax() const { return std::isfinite(Vbat_max) && Vbat_max > 0; }

    __attribute__((cold)) void load(const ConfFile &chargerConf) {
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
        assert_throw(std::isfinite(Vbat_fallback) && Vbat_fallback > 0, "vout_max_fallback must be > 0");
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
        long hs = chargerConf.getLong("cv_hold_max_s", 0);
        assert_throw(hs >= 0 && hs <= 86400, "cv_hold_max_s must be in [0, 86400]");
        cv_hold_max_s = (uint32_t) hs;
        hs = chargerConf.getLong("tail_hold_s", 180);
        assert_throw(hs > 0 && hs <= 7200, "tail_hold_s must be in (0, 7200]");
        tail_hold_s = (uint32_t) hs;
        cv_hold_band = chargerConf.getFloat("cv_hold_band", 0.02f);
        float bd = chargerConf.getFloat("balance_interval", 0.f);
        hs = chargerConf.getLong("balance_hold_s", 7200);
        balance_delta = chargerConf.getFloat("balance_delta", 0.01f);
        assert_throw(bd >= 0.f && bd < 365.f && hs > 0 && hs <= 86400 && balance_delta >= 0.f, "balance_* out of range");
        balance_interval_s = (uint32_t) (bd * 86400.f);
        balance_hold_s = (uint32_t) hs;
        assert_throw(cv_hold_band > 0.f && (!(cv_hold_max_s || balance_interval_s) || cv_hold_band < cv_eoc - cv_min),
                     "need 0 < cv_hold_band < cv_eoc - cv_float");
        vout_offset_max = chargerConf.getFloat("vout_offset_max", 0.6f);
        assert_throw(vout_offset_max >= 0.f, "vout_offset_max must be >= 0");
        assert_throw(Vbat_fallback > vout_offset_max, "vout_max_fallback must be > vout_offset_max");
        assert_throw(recharge_dod >= 0.f && recharge_dod < 1.f, "recharge_dod must be in [0, 1)");
        partial_charge = chargerConf.getFloat("partial_charge", 0.f);
        assert_throw(partial_charge >= 0.f && partial_charge < 1.f, "partial_charge must be in [0, 1)");
        if (partial_charge > 0.f) {
            // the hold releases recharge_dod below the ceiling; that band must stay above 0 % SoC
            assert_throw(recharge_dod > 0.f && recharge_dod < partial_charge, "need 0 < recharge_dod < partial_charge");
            if (!(std::isfinite(Cbat) && Cbat > 0.f)) {
                ESP_LOGW("charger", "partial_charge needs bat_c, disabled");
                partial_charge = 0.f;
            }
        }
        holiday_charge = chargerConf.getFloat("holiday_charge", 0.f);
        assert_throw(holiday_charge >= 0.f && holiday_charge < 1.f, "holiday_charge must be in [0, 1)");
        long hd = chargerConf.getLong("holiday_days", 3);
        assert_throw(hd >= 1 && hd <= 60, "holiday_days must be in [1, 60]");
        holiday_days = (uint8_t) hd;
        if (holiday_charge > 0.f) {
            holiday_ibat = chargerConf.getFloat("holiday_ibat", NAN);
            assert_throw(std::isfinite(holiday_ibat) && holiday_ibat > 0.f, "holiday_charge needs holiday_ibat > 0 [A]");
            holiday_band = chargerConf.getFloat("holiday_band", 0.1f);
            assert_throw(holiday_band > 0.f && holiday_band < holiday_charge, "need 0 < holiday_band < holiday_charge");
            if (!(std::isfinite(Cbat) && Cbat > 0.f)) {
                ESP_LOGW("charger", "holiday_charge needs bat_c, disabled");
                holiday_charge = 0.f;
            }
        }
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

// Charger state kept across esp_restart/panic/OTA in pinned RTC slow RAM (not across power loss).
// The first 20 B are the v2 layout (older builds read it unchanged); the balance part validates alone, the
// appended ceiling wait with balSum (a 40 B build leaves it stale, balSum moves on).
struct ChargerRtc {
    uint32_t magic;
    uint32_t fullAgeS; // age of the last full charge, UINT32_MAX = unknown
    float bmsRef; // BMS remaining charge at that full, NAN = none
    uint8_t holiday, streak, gaps, override_;
    uint32_t sum;
    uint32_t balMagic;
    uint32_t balAgeS; // age of the last balance run, UINT32_MAX = forced due
    uint32_t balInBandS; // in-band time of the unfinished run
    uint32_t balSessS; // time in the current balance hold session
    uint32_t balSum;
    uint32_t balCeilS; // age of a run stopped on the cell ceiling, UINT32_MAX = none
    uint32_t balCeilSum;

    static constexpr uint32_t MAGIC = 0x48444c32, BAL_MAGIC = 0x42414c32;
    [[nodiscard]] __attribute__((noinline)) static uint32_t fnv(const void *p, size_t words) {
        uint32_t h = 0x811c9dc5, w;
        for (size_t i = 0; i < words; ++i) {
            memcpy(&w, (const char *) p + 4 * i, 4);
            h = (h ^ w) * 16777619u;
        }
        return h;
    }
    [[nodiscard]] uint32_t checksum() const { return fnv(this, 4); }
    [[nodiscard]] uint32_t balChecksum() const { return fnv(&balMagic, 4); }
    [[nodiscard]] bool valid() const {
        return magic == MAGIC && sum == checksum() && holiday <= 1 && override_ <= 2;
    }
    [[nodiscard]] bool balValid() const { return balMagic == BAL_MAGIC && balSum == balChecksum(); }
    [[nodiscard]] uint32_t ceilChecksum() const { return fnv(&balSum, 2); }
    [[nodiscard]] bool ceilValid() const { return balValid() && balCeilSum == ceilChecksum(); }
};
static_assert(offsetof(ChargerRtc, sum) == 16 && offsetof(ChargerRtc, balMagic) == 20
              && offsetof(ChargerRtc, balCeilS) == 40 && sizeof(ChargerRtc) == 48, "ChargerRtc layout");

struct BatteryState {
    static constexpr auto VCELL_EXPIRATION_TIME_SEC = 180;
    static constexpr uint16_t IBAT_MIN_SAMPLES = 8; // smoothing warm-up before ibat is published
    static constexpr uint8_t TEMP_SENSORS = 4; // max bat_temp_topic entries

    volatile float vcell_high = 0; // voltage of highest cell reported by BMS
    volatile uint32_t vcell_high_t = 0; // frame id, low 32 bits of wallClockUs()
    volatile uint32_t vcell_high_ticks = 0; // freshness stamp, wallClockTicks(); the us frame id wraps every ~71.6 min
    volatile uint32_t ibat_t = 0; // last ibat frame, same clock as vcell_high_t
    volatile float ibat_raw = NAN; // last ibat frame, unsmoothed
    volatile float vpack = NAN; // [V] BMS pack voltage
    volatile uint32_t vpack_t = 0; // last vpack frame, same clock as vcell_high_t
    volatile float temp[TEMP_SENSORS]{NAN, NAN, NAN, NAN}; // [°C] pack sensors from BMS
    volatile uint32_t temp_t[TEMP_SENSORS]{0, 0, 0, 0};
    volatile float cell_delta = NAN; // [V] BMS max - min cell voltage
    volatile uint32_t cell_delta_t = 0; // last delta frame, same clock as vcell_high_t

    EWMA<volatile float, float> vout_avg{60}; // time-averaged pack voltage
    CoulombCounter coulombCounter{}; // Ah-since-last-full tracker, used for recharge hysteresis

    void setVcellHigh(const float &vcell_high_) {
        vcell_high = vcell_high_;
        vcell_high_t = static_cast<uint32_t>(wallClockUs());
        vcell_high_ticks = wallClockTicks();
        coulombCounter.sampleOnCellFrame(vcell_high_ticks);
    }

    void setTemp(uint8_t i, float t) {
        if (i >= TEMP_SENSORS) return;
        temp[i] = (t > -40.f && t < 100.f) ? t : NAN; // NAN and sensor-fault values drop the sensor
        temp_t[i] = static_cast<uint32_t>(wallClockUs());
    }

    void setCellDelta(float d) {
        cell_delta = d;
        cell_delta_t = static_cast<uint32_t>(wallClockUs());
    }

    [[nodiscard]] bool haveValidCellVoltage() const {
        return vcell_high > 0 and cellAgeTicks() < secToCoarseTicks(VCELL_EXPIRATION_TIME_SEC);
    }

    [[nodiscard]] uint32_t cellAgeTicks() const { return wallClockTicks() - vcell_high_ticks; }

    // producer (MQTT task): smooth ibat here and publish a single lock-free
    // snapshot the loop-task consumer only loads. NAN until IBAT_MIN_SAMPLES seen.
    void updateBatCurrent(const float &ibat) {
        _ibatEwma.add(ibat);
        ibat_raw = ibat;
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
    // The cell voltage relaxes over minutes after a drop in charge current (cloud), the line would drop
    // at once: place the line with the decaying peak of the current instead.
    static constexpr float ILINE_TAU_S = 120.f;
    static constexpr uint32_t STALE_PAUSE_S = 600; // balance run: longest stale pause before the latch

    const BatChargerParams &p;
    bool terminated = false;
    float _v_term;
    uint8_t _vfloorStreak = 0; // sustained sub-threshold counter (transient I·R sag → 1-frame dips, ignore)
    uint8_t _ceilStreak = 0; // sustained over-ceiling counter (transient charge I·R spike → 1-frame, ignore)
    uint8_t _lineStreak = 0;
    float _iHold = 0; // recent peak of ibat, decaying with ILINE_TAU_S
    time_us _iHoldUs = 0;
    bool _cvHold = false, _tailOn = false;
    uint32_t _cvHoldTicks = 0, _tailTicks = 0; // wallClockTicks() at hold entry / tail start
    uint32_t _staleT = 0; // wallClockTicks() | 1 at a stale pause of a balance hold, 0 = none

    void _latch() {
        terminated = true;
        _cvHold = false;
        _staleT = 0;
        _vfloorStreak = 0;
        _ceilStreak = 0;
        _lineStreak = 0;
    }

public:
    bool balance = false; // balance run: a line crossing enters the hold even with cv_hold_max_s = 0; no tail/timeout exit

    [[nodiscard]] float v_term() const { return _v_term; }
    [[nodiscard]] bool cvHold() const { return _cvHold; }
    [[nodiscard]] uint32_t cvHoldS() const { return _cvHold ? coarseTicksToSec(wallClockTicks() - _cvHoldTicks) : 0; }
    [[nodiscard]] bool stalePaused() const { return _staleT; }

    // BMS cell or ibat data went stale during the CV hold: latch (the stale path holds the float cap anyway).
    // A balance hold pauses instead (left, the charger holds float as if latched) and resumes on the next
    // fresh frame; it latches once stale STALE_PAUSE_S longer.
    void latchStale() {
        if (_staleT) {
            if (wallClockTicks() - _staleT < secToCoarseTicks(STALE_PAUSE_S)) return;
        } else if (!_cvHold) {
            return;
        } else if (balance) {
            ESP_LOGW("charger", "cv hold paused: BMS data stale");
            _staleT = wallClockTicks() | 1;
            _cvHold = false;
            return;
        }
        ESP_LOGW("charger", "cv hold ended: BMS data stale after %u s%s", (unsigned) cvHoldS(),
                 _staleT ? " (balance paused)" : "");
        _latch();
    }

    explicit operator bool() const { return terminated; }


    explicit Li_ChgTerminationCondition(const BatChargerParams &params)
        : p(params),
          _v_term(params.cv_min) {
    }

    void reset() {
        terminated = false;
        _v_term = p.cv_min;
        _iHoldUs = 0;
        _vfloorStreak = 0;
        _ceilStreak = 0;
        _lineStreak = 0;
        _cvHold = false;
        _staleT = 0;
    }

    // One call per BMS cell frame (the streaks count frames).
    bool update(float vcell_high, float ibat, float ahSinceFull) {
        // Termination line: at ibat = tail_c_rate * Cbat the cell sits at cv_eoc; at ibat = 0 it sits at cv_min.
        // r models the apparent cell resistance implied by that line (defaults 3.5/3.325 V, 280Ah / 0.05 → 12.5mΩ).
        // See website/docs/guide/charging/termination.md.
        float r = (p.cv_eoc - p.cv_min) / (p.tail_c_rate * p.Cbat);
        auto now = wallClockUs();
        _iHold = fmaxf(ibat, _iHoldUs ? _iHold * expf((float) (now - _iHoldUs) * (-1e-6f / ILINE_TAU_S)) : ibat);
        _iHoldUs = now;
        float vo = _iHold * r;
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
        if (!terminated and !_cvHold and ibat > iBatFloor and vcell_high > _v_term) {
            if (++_lineStreak >= VTERM_STREAK_REQ) lineLatch = true;
        } else {
            _lineStreak = 0;
        }

        const bool resume = _staleT; // fresh after a stale pause: back in the hold, its clock kept
        _staleT = 0;
        if (!ceilingLatch and (resume or (lineLatch and (p.cv_hold_max_s or balance)))) {
            // CV hold: the charger holds the cell at cv_eoc and the current tapers; latch on the tail
            lineLatch = false;
            _lineStreak = 0;
            _cvHold = true;
            _tailOn = false;
            _vfloorStreak = 0;
            if (!resume) _cvHoldTicks = wallClockTicks();
        } else if (_cvHold and !ceilingLatch) {
            uint32_t now = wallClockTicks(), heldS = coarseTicksToSec(now - _cvHoldTicks);
            if (ibat < p.tail_c_rate * p.Cbat and vcell_high >= p.cv_eoc - p.cv_hold_band) {
                if (!_tailOn) { _tailOn = true; _tailTicks = now; }
            } else {
                _tailOn = false;
            }
            uint32_t tailS = _tailOn ? coarseTicksToSec(now - _tailTicks) : 0;
            if (!balance and tailS >= p.tail_hold_s) {
                ESP_LOGW("charger", "cv hold ended: tail %u s @ iBat %.2f", (unsigned) tailS, ibat);
                lineLatch = true;
            } else if (!balance and heldS > p.cv_hold_max_s) {
                ESP_LOGW("charger", "cv hold timeout after %u s", (unsigned) heldS);
                lineLatch = true;
            } else if (vcell_high < p.cv_min) {
                // sun gone: back to charging, not "full"
                if (++_vfloorStreak >= VFLOOR_STREAK_REQ) {
                    ESP_LOGW("charger", "cv hold left: vcHigh %.3f < cv_min after %u s", vcell_high, (unsigned) heldS);
                    _cvHold = false;
                    _vfloorStreak = 0;
                }
            } else {
                _vfloorStreak = 0;
            }
        }

        if (!terminated and (ceilingLatch or lineLatch)) {
            _latch();
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
    float _staleCap = NAN; // [V] bus cap latched for the cell-data gap that began at _staleCapTicks
    uint32_t _staleCapTicks = 0;
    bool _termJoin = false; // terminated without authority, joined at float because the pack discharges
    bool _wasCvHold = false, _apArm = false; // hold entered without an approach limit: cut pending
    float _cvIlim = NAN, _cvIbat = NAN; // [A] CV hold: output-current limit (NAN = none), pack-current target

    // Integrator step gate: advance vpack_pin only on a fresh BMS frame so the
    // step rate matches the cell-voltage update rate (otherwise loopLF at 1 Hz
    // runs the integrator 15× per BMS cycle and overshoots).
    uint32_t _lastBmsFrameUs = 0;

    uint32_t _lastTermFrameUs = 0; // termCond runs once per cell frame
    uint32_t _lastIbatFrameUs = 0; // load-following steps once per ibat frame

    bool _bmsCellSource = false; // a BMS cell-voltage topic was configured (termination can be evaluated)
    bool _termDecided = false; // termCond has seen enough cell frames to have latched (needs warm, fresh ibat)
    uint8_t _termEvals = 0;
    volatile bool _coldBlocked = false; // pack below bat_temp_min: hold ibat at 0 (loads are still served)
    volatile bool _partialHold = false; // Ah ceiling reached: hold SoC by following the load (ibat -> 0)
    float _hotScale = NAN; // derate factor for Ibat_lim above bat_temp_derate (NAN = none)
    bool _wasHold = false;
    time_us _lastFullUs = 0; // last termination latch (0 = none since boot)
    time_us _hStartUs = 0, _hLastUs = 0; // holiday: current 24 h bucket start, last tick
    float _hCovS = 0.f, _hDisS = 0.f, _hDisAh = 0.f; // bucket coverage [s], discharge time [s] and charge [Ah]
    uint32_t _hLastT = 0, _hFrames = 0, _hChg = 0; // ibat frames in the bucket, and how many changed value
    float _hLastI = NAN;
    uint8_t _hStreak = 0, _hGaps = 0; // consecutive quiet / unevaluable buckets
    volatile uint8_t _hOverride = 0; // 0 auto, 1 on, 2 off
    bool _holiday = false, _hEff = false; // detected, effective last tick
    uint32_t _fullAgeBootS = UINT32_MAX; // restored full-charge age at boot (no full since boot)
    bool _refFull = false; // the counter's BMS reference was taken at a real full (since boot or restored)
    ChargerRtc *_rtc = nullptr;
    // wallClockTicks(): tear-free on core 0
    uint32_t _balDoneT = 0, _balLastT = 0; // last completed run (0 = none since boot), last balance tick
    uint32_t _balAgeBootS = 0; // restored balance age at boot; unknown counts from boot
    float _balInBandS = 0.f, _balSessS = 0.f; // in-band time of the pending run, time in the current hold session
    uint32_t _balDeltaLastT = 0, _balDeltaSeenT = 0, _balCeilT = 0; // _balCeilT: run stopped on the cell ceiling
    uint8_t _balDeltaStreak = 0;
    bool _balForce = false, _balRun = false, _balHot = false, _balWasHold = false, _tempSource = false;
    volatile uint8_t _balReq = 0; // console request: 1 now, 2 skip

    // Consumer-side freshness of the producer's 32-bit frame stamps: the stamp only tells us a
    // frame arrived; when it changed is noted here on the 64-bit loop clock (wrap-free, no torn read).
    uint32_t _ibatLastT = 0, _tempLastT[BatteryState::TEMP_SENSORS]{};
    time_us _ibatSeenUs = 0, _tempSeenUs[BatteryState::TEMP_SENSORS]{};
    float _tempMin = NAN, _tempMax = NAN; // over fresh sensors, NAN when none
    uint32_t _wireLastT = 0;
    float _wirePrevI = NAN;

    static constexpr float BAT_TEMP_HYST = 2.f; // [°C] cold-block release band
    static constexpr time_us TEMP_EXPIRE_US = 3600ULL * 1000000ULL; // policy off without a frame (as without a sensor)
    static constexpr time_us IBAT_EXPIRE_US = 180ULL * 1000000ULL; // like VCELL_EXPIRATION_TIME_SEC
    static constexpr float IOUT_LIM_FLOOR = 0.25f; // the CC limiter normalizes by its setpoint; never 0
    static constexpr float APPROACH_V = 0.03f; // [V] CV hold approach band below cv_eoc (> one cell frame's rise)
    static constexpr time_us BOOT_IDLE_US = 120ULL * 1000000ULL;
    static constexpr time_us HOLIDAY_DAY_US = 86400ULL * 1000000ULL;
    static constexpr time_us HOLIDAY_GAP_US = 60ULL * 1000000ULL; // longer tick or ibat-frame gap: not covered
    static constexpr float HOLIDAY_DIS_A = 0.2f; // [A] discharge below this counts
    static constexpr float HOLIDAY_MIN_DIS_S = 2 * 3600; // discharge time for an evaluable bucket
    static constexpr float HOLIDAY_EXIT_MIN_S = 3 * 3600, HOLIDAY_EXIT_K = 1.2f; // in-bucket exit: mean > K * holiday_ibat
    static constexpr float BAL_TEMP_MAX = 35.f; // [°C] balance runs deferred above (LFP storage range 0..35 °C)
    static constexpr uint32_t BAL_GAP_T = secToCoarseTicks(60); // older cell frames, longer tick/delta gaps: not counted
    static constexpr float BAL_TICK_S = 1048576e-6f; // [s] per coarse tick
    static constexpr uint8_t BAL_DELTA_FRAMES = 3; // consecutive delta frames <= balance_delta to finish early

    // Notes when a frame stamp changes; true while the last change is younger than maxAge.
    static bool _fresh(uint32_t stamp, uint32_t &last, time_us &seen, time_us now, time_us maxAge) {
        if (stamp != last) { last = stamp; seen = now; }
        return seen != 0 && (now - seen) < maxAge;
    }

public:
    BatChargerParams params{};
    Li_ChgTerminationCondition termCond{params};
    BatteryState batSt{};
    WireResistance wire{};
    WireGuard wireGuard{};


    explicit BatteryCharger() = default;

    __attribute__((cold)) void begin(const ConfFile &chargerConf) {
        params.load(chargerConf);
        termCond.reset(); // propagate just-loaded cv_min into termCond's v_term (was NAN at ctor time)
    }


    void _updateTermination() {
        // Gate on the battery-current snapshot from the BMS/MQTT producer. It is
        // already smoothed over IBAT_MIN_SAMPLES there; no need to pace on the
        // converter's own iout (which is ibat + iload and cannot be used alone).
        float ibat = batSt.ibatSmoothed();
        bool ibatOk = std::isfinite(ibat) && ibatFresh();
        if (!batSt.haveValidCellVoltage() || !ibatOk) {
            termCond.latchStale(); // not a verified full: the Ah reference stays
            return;
        }

        uint32_t frame = batSt.vcell_high_t;
        if (frame == _lastTermFrameUs) return; // streaks inside termCond count cell frames, not loop ticks
        _lastTermFrameUs = frame;

        bool wasTerm = bool(termCond);
        termCond.update(batSt.vcell_high, ibat, batSt.coulombCounter.updateDepth(wasTerm));
        // the line needs VTERM_STREAK_REQ frames to latch, so the boot sweep must not trust
        // "not terminated" before that many have been seen
        if (_termEvals < UINT8_MAX) ++_termEvals;
        _termDecided = _termEvals >= 2;
        if (!wasTerm && bool(termCond)) {
            // Rising edge — pack is full, re-zero the coulomb counter so the
            // recharge_dod hysteresis measures against this full point.
            batSt.coulombCounter.markFull();
            _refFull = std::isfinite(batSt.coulombCounter.bmsRef());
            _lastFullUs = wallClockUs();
            ESP_LOGI("charger", "Termination latched: ahSinceFull reset to 0");
            // in the hold or its approach
            if (termCond.balance && batSt.vcell_high >= params.cv_ceiling) {
                _balCeilT = wallClockTicks() | 1;
                ESP_LOGW("charger", "balance stopped on the cell ceiling, deferred");
            }
        }
    }

    // Age of the last completed balance run [s]; unknown counts from boot.
    [[nodiscard]] uint32_t balanceAgeS() const {
        return (_balDoneT ? 0 : _balAgeBootS) + coarseTicksToSec(wallClockTicks() - _balDoneT);
    }

    [[nodiscard]] bool balanceDue() const {
        return params.balance_interval_s && (_balForce || balanceAgeS() >= params.balance_interval_s);
    }

    // why a due run waits, nullptr when it may run
    [[nodiscard]] const char *balanceDeferWhy() {
        if (holiday()) return "holiday";
        if (_balHot) return "warm";
        if (_tempSource && !haveTemp()) return "temp n/a";
        // a run stopped on the cell ceiling waits interval/4, at least 20 h (not the next charge)
        if (_balCeilT && wallClockTicks() + 1 - _balCeilT
                         < secToCoarseTicks(std::max<uint32_t>(params.balance_interval_s / 4, 72000)))
            return "ceiling";
        if (!batSt.haveValidCellVoltage() || !ibatFresh()) return "BMS stale";
        return nullptr;
    }

    [[nodiscard]] bool balanceRunning() const { return termCond.balance && termCond.cvHold(); }
    [[nodiscard]] uint32_t balanceInBandS() const { return (uint32_t) _balInBandS; }

    // console requests, applied on the next update()
    void balanceNow() { _balReq = 1; }
    void balanceSkip() { _balReq = 2; }

    // Top-balancing run: while due and not deferred, a line crossing enters the CV hold (termCond.balance) and
    // it holds the highest cell at cv_eoc. Done after balance_hold_s in band or balance_delta on
    // BAL_DELTA_FRAMES delta frames; the hold then ends through its normal exits (tail, cv_hold_max_s), as it
    // does when the run is deferred or a session exceeds 2 balance_hold_s. The in-band time is kept until done.
    void _updateBalance() {
        uint32_t now = wallClockTicks(), dt = now - _balLastT;
        _balLastT = now;
        uint8_t q = _balReq;
        _balReq = 0;
        _balForce |= q == 1;
        if (q == 1) _balCeilT = 0;
        _balHot = haveTemp() && _tempMax > (_balHot ? BAL_TEMP_MAX - BAT_TEMP_HYST : BAL_TEMP_MAX);
        _balRun = balanceDue() && !balanceDeferWhy();
        bool hold = termCond.cvHold() || termCond.stalePaused(); // a stale pause keeps the session
        if (_balWasHold && !hold) _balSessS = 0.f; // session over (a reboot keeps it)
        _balWasHold = hold;
        if (hold && _balRun && dt <= BAL_GAP_T) _balSessS += dt * BAL_TICK_S;
        termCond.balance = _balRun && _balSessS <= 2 * params.balance_hold_s;
        bool newDelta = batSt.cell_delta_t != _balDeltaLastT;
        if (newDelta) {
            _balDeltaLastT = batSt.cell_delta_t;
            if (now - _balDeltaSeenT > BAL_GAP_T) _balDeltaStreak = 0;
            _balDeltaSeenT = now;
        }
        if (!(termCond.balance && hold && dt <= BAL_GAP_T && batSt.cellAgeTicks() < BAL_GAP_T
              && batSt.vcell_high >= params.cv_eoc - params.cv_hold_band)) {
            _balDeltaStreak = 0;
        } else {
            _balInBandS += dt * BAL_TICK_S;
            if (newDelta) _balDeltaStreak = batSt.cell_delta <= params.balance_delta ? _balDeltaStreak + 1 : 0;
        }
        if (q == 2 || _balInBandS >= params.balance_hold_s || _balDeltaStreak >= BAL_DELTA_FRAMES) {
            ESP_LOGI("charger", "balance %s: %u s in band", q == 2 ? "skipped" : "done", (unsigned) _balInBandS);
            _balDoneT = now | 1;
            _balInBandS = 0.f;
            _balForce = false;
            _balDeltaStreak = 0;
            termCond.balance = false;
        }
    }

    // Battery-temperature policy. Sensors expire individually after an hour without a frame; with none
    // fresh the policy is off, as without the feature. Cold: the pack current is held at 0 by the
    // load-follower (loads are still served; discharging a cold pack is fine). Hot: the follower
    // holds the pack current at the derated ibat_max instead; that regulates the *pack* current even
    // when a sibling converter feeds it too. Without a fresh ibat both fall back to an output limit.
    void _updateTempLimit() {
        ioutLim = NAN;
        _hotScale = NAN;
        time_us now = wallClockUs();
        _tempMin = INFINITY; _tempMax = -INFINITY;
        for (uint8_t i = 0; i < BatteryState::TEMP_SENSORS; ++i) {
            float t = batSt.temp[i];
            if (!_fresh(batSt.temp_t[i], _tempLastT[i], _tempSeenUs[i], now, TEMP_EXPIRE_US) || !std::isfinite(t)) continue;
            if (t < _tempMin) _tempMin = t;
            if (t > _tempMax) _tempMax = t;
        }
        if (!std::isfinite(_tempMin)) {
            _tempMin = _tempMax = NAN;
            if (_coldBlocked) ESP_LOGW("charger", "pack temperature stale, cold block dropped");
            _coldBlocked = false;
            return;
        }
        float release = params.bat_temp_min + BAT_TEMP_HYST;
        if (_tempMin < (_coldBlocked ? release : params.bat_temp_min)) {
            if (!_coldBlocked)
                ESP_LOGW("charger", "pack %.1f°C < %.1f°C: charging blocked until %.1f°C", _tempMin, params.bat_temp_min, release);
            _coldBlocked = true;
            if (!ibatFresh()) ioutLim = IOUT_LIM_FLOOR; // no ibat to follow: idle
            return;
        }
        if (_coldBlocked) ESP_LOGI("charger", "pack %.1f°C: charging released", _tempMin);
        _coldBlocked = false;
        if (_tempMax > params.bat_temp_derate) {
            _hotScale = fmaxf(0.f, (params.bat_temp_max - _tempMax) / (params.bat_temp_max - params.bat_temp_derate));
            if (!ibatFresh()) ioutLim = fmaxf(params.Ibat_lim * _hotScale, IOUT_LIM_FLOOR); // assumes iload ~ 0
        }
    }

    [[nodiscard]] bool ibatFresh() {
        return std::isfinite(batSt.ibatSmoothed())
               && _fresh(batSt.ibat_t, _ibatLastT, _ibatSeenUs, wallClockUs(), IBAT_EXPIRE_US);
    }
    [[nodiscard]] bool haveTemp() const { return std::isfinite(_tempMin); }
    [[nodiscard]] float tempMin() const { return _tempMin; }
    [[nodiscard]] float tempMax() const { return _tempMax; }
    [[nodiscard]] uint32_t ibatAgeS() const { return _ibatSeenUs ? (uint32_t) ((wallClockUs() - _ibatSeenUs) / 1000000ULL) : 0; }
    [[nodiscard]] uint32_t tempAgeS() const {
        time_us newest = 0;
        for (auto t: _tempSeenUs) if (t > newest) newest = t;
        return newest ? (uint32_t) ((wallClockUs() - newest) / 1000000ULL) : 0;
    }

    // Holiday detection, per 24 h bucket: mean discharge current (ibat < -0.2 A) over the time it
    // discharged (at least 2 h). Charging and the partial hold's own drain are ignored. A bucket is evaluable
    // with >= 80 % ibat coverage and, after >= 2 h of discharge, a changing ibat (a stuck reading is not quiet);
    // an unevaluable one pauses the streak, two in a row reset it unless on holiday. holiday_days quiet buckets enter; a
    // mean above holiday_ibat ends it (in the running bucket after 3 h above 1.2 holiday_ibat).
    void _updateHoliday() {
        if (!(params.holiday_charge > 0.f)) {
            _holiday = false;
            _hStreak = _hGaps = 0;
            _hStartUs = 0;
            _hEff = false;
            return;
        }
        time_us now = wallClockUs();
        if (!_hStartUs) _hNewBucket(now);
        time_us dt = now - _hLastUs;
        _hLastUs = now;
        float ibat = batSt.ibatSmoothed();
        float t = _partialHold ? _holdIbatTarget() : 0.f;
        if (t < 0.f && ibat >= t - HOLIDAY_DIS_A) ibat = 0.f; // the hold's own drain; beyond it the converter is out
        bool fresh = ibatFresh();
        if (batSt.ibat_t != _hLastT) {
            _hLastT = batSt.ibat_t;
            ++_hFrames;
            if (batSt.ibat_raw != _hLastI) ++_hChg;
            _hLastI = batSt.ibat_raw;
        }
        if (dt <= HOLIDAY_GAP_US && fresh && now - _ibatSeenUs <= HOLIDAY_GAP_US && std::isfinite(ibat)) {
            float s = dt * 1e-6f;
            _hCovS += s;
            if (ibat < -HOLIDAY_DIS_A) { _hDisS += s; _hDisAh -= ibat * s * (1.f / 3600.f); }
        }
        float mean = holidayMeanA();
        if (_holiday && _hDisS >= HOLIDAY_EXIT_MIN_S && mean > HOLIDAY_EXIT_K * params.holiday_ibat) {
            _hStreak = 0;
            _hSet(false, mean);
        }
        if (now - _hStartUs >= HOLIDAY_DAY_US) {
            bool evaluable = _hCovS >= 0.8f * (HOLIDAY_DAY_US * 1e-6f)
                             && (_hDisS < HOLIDAY_MIN_DIS_S || _hChg * 20 >= _hFrames);
            mean = _hDisAh * 3600.f / fmaxf(_hDisS, HOLIDAY_MIN_DIS_S);
            if (!evaluable) {
                if (_hGaps < UINT8_MAX) ++_hGaps;
                if (_hGaps >= 2 && !_holiday) _hStreak = 0; // gaps never end an active holiday
            } else {
                _hGaps = 0;
                _hStreak = mean < params.holiday_ibat ? (uint8_t) (_hStreak < UINT8_MAX ? _hStreak + 1 : _hStreak) : 0;
                _hSet(_hStreak >= params.holiday_days, mean);
            }
            _hNewBucket(now);
        }
        bool eff = holiday();
        if (_hEff && !eff) _partialHold = false; // re-evaluate against the raised ceiling
        _hEff = eff;
    }

    void _hNewBucket(time_us now) {
        _hStartUs = _hLastUs = now;
        _hCovS = _hDisS = _hDisAh = 0.f;
        _hFrames = _hChg = 0;
    }

    void _hSet(bool h, float mean) {
        if (h != _holiday)
            ESP_LOGI("charger", "holiday %s: %u quiet days (mean discharge %.2f A, %.1f h)", h ? "on" : "off",
                     (unsigned) _hStreak, mean, _hDisS * (1.f / 3600.f));
        _holiday = h;
    }

    [[nodiscard]] float holidayMeanA() const { return _hDisS > 0.f ? _hDisAh * 3600.f / _hDisS : 0.f; }

    // effective: detection, or the console override
    [[nodiscard]] bool holiday() const {
        return params.holiday_charge > 0.f && (_hOverride == 1 || (_hOverride == 0 && _holiday));
    }

    // SoC ceiling in effect (0 = charge to full)
    [[nodiscard]] float ceiling() const {
        float c = params.partial_charge;
        if (holiday()) c = c > 0.f ? fminf(c, params.holiday_charge) : params.holiday_charge;
        return c;
    }

    // Age of the last full charge [s], from a reboot-surviving age when there was none since boot;
    // UINT32_MAX when unknown (a full charge is due).
    [[nodiscard]] uint32_t fullAgeS() const {
        if (_lastFullUs) return (uint32_t) ((wallClockUs() - _lastFullUs) / 1000000ULL);
        if (_fullAgeBootS == UINT32_MAX) return UINT32_MAX;
        return _fullAgeBootS + (uint32_t) (wallClockUs() / 1000000ULL);
    }

    // The deficit counts from a real full: one since boot, or the restored BMS reference while it is live.
    [[nodiscard]] bool deficitKnown() const {
        return _lastFullUs != 0 || (_refFull && batSt.coulombCounter.bmsSource());
    }

    // Partial-charge ceiling: after a full charge, stop at ceiling() (Ah-counted from that full) and
    // hold there by load-following until the pack has discharged recharge_dod (holiday_band on
    // holiday) below it, or until full_charge_interval expires (then charge to full for balancing).
    void _updatePartialHold() {
        if (_refFull && !_lastFullUs && batSt.coulombCounter.bmsSource()) {
            float d = batSt.coulombCounter.deficit();
            if (!(d >= -0.05f * params.Cbat && d <= params.Cbat)) {
                ESP_LOGW("charger", "restored BMS ref dropped: deficit %.1f Ah", d);
                _refFull = false;
                batSt.coulombCounter.restoreRef(NAN);
            }
        }
        bool dataOk = batSt.haveValidCellVoltage() && ibatFresh(); // the deficit counter is only trusted live
        float lim = ceiling();
        bool enabled = lim > 0.f && deficitKnown() && dataOk && fullAgeS() < params.full_charge_interval_s && !_balRun;
        if (!enabled || bool(termCond)) {
            if (_partialHold)
                ESP_LOGI("charger", "partial hold off (%s)", bool(termCond) ? "terminated" : !dataOk ? "BMS stale" : "full charge due");
            _partialHold = false;
            return;
        }
        float ah = batSt.coulombCounter.deficit();
        float ceilAh = (1.f - lim) * params.Cbat;
        float band = holiday() ? params.holiday_band : params.recharge_dod;
        if (!_partialHold && ah <= ceilAh) {
            _partialHold = true;
            ESP_LOGI("charger", "partial hold: %.0f%% reached (%.1f Ah since full)", lim * 100.f, ah);
        } else if (_partialHold && ah > ceilAh + band * params.Cbat) {
            _partialHold = false;
            ESP_LOGI("charger", "partial hold released: %.1f Ah since full", ah);
        }
    }

    // Restores from r if valid (else starts unknown), then keeps r up to date on every update().
    void attachRtc(ChargerRtc *r) {
        _rtc = r;
        if (!r->valid()) {
            ESP_LOGI("charger", "no retained state (power-on)");
            return;
        }
        if (r->fullAgeS < 365U * 86400U) {
            _fullAgeBootS = r->fullAgeS;
            if (std::isfinite(r->bmsRef)) {
                batSt.coulombCounter.restoreRef(r->bmsRef);
                _refFull = true;
            }
        }
        _holiday = r->holiday;
        _hStreak = r->streak;
        _hGaps = r->gaps;
        _hOverride = r->override_;
        ESP_LOGI("charger", "retained: full %lus ago%s, holiday %u (streak %u, override %u)",
                 (unsigned long) _fullAgeBootS, _refFull ? " (BMS ref)" : "", (unsigned) _holiday,
                 (unsigned) _hStreak, (unsigned) _hOverride);
        if (r->balValid()) {
            _balForce = r->balAgeS == UINT32_MAX;
            _balAgeBootS = _balForce ? 0 : r->balAgeS;
            _balInBandS = (float) (r->balInBandS < 86400U ? r->balInBandS : 86400U);
            _balSessS = (float) (r->balSessS < 2 * 86400U ? r->balSessS : 2 * 86400U);
            if (r->ceilValid() && r->balCeilS != UINT32_MAX)
                _balCeilT = (wallClockTicks() - secToCoarseTicks(r->balCeilS)) | 1;
        }
    }

    void _saveRtc() {
        if (!_rtc) return;
        ChargerRtc r{};
        r.magic = ChargerRtc::MAGIC;
        r.fullAgeS = fullAgeS();
        r.bmsRef = r.fullAgeS != UINT32_MAX && _refFull ? batSt.coulombCounter.bmsRef() : NAN;
        r.holiday = _holiday;
        r.streak = _hStreak;
        r.gaps = _hGaps;
        r.override_ = _hOverride;
        r.sum = r.checksum();
        r.balMagic = ChargerRtc::BAL_MAGIC;
        r.balAgeS = _balForce ? UINT32_MAX : balanceAgeS();
        r.balInBandS = (uint32_t) _balInBandS;
        r.balSessS = (uint32_t) _balSessS;
        r.balSum = r.balChecksum();
        r.balCeilS = _balCeilT ? coarseTicksToSec(wallClockTicks() + 1 - _balCeilT) : UINT32_MAX;
        r.balCeilSum = r.ceilChecksum();
        *_rtc = r;
    }

    // 0 auto, 1 on, 2 off
    void setHolidayOverride(uint8_t o) { _hOverride = o; }
    [[nodiscard]] uint8_t holidayOverride() const { return _hOverride; }

    // Load-following: nudge vpack_pin so the BMS pack current goes to ibatTarget (0: the converter
    // covers the load, nothing goes into the pack). One capped step per ibat frame; ibat is already
    // smoothed over IBAT_MIN_SAMPLES frames, so the step must stay small against the stiff pack
    // (tens of mΩ including cables) or the loop hunts. Only steps while this converter drives the
    // bus: a disabled converter (night) would otherwise integrate the house load up to Vbat_max and
    // charge for minutes at dawn. The partial hold has a floor (the termination-release voltage minus
    // the Vout tolerance: a pack at partial SoC rests above it); a cold pack may sit at any SoC, so
    // the cold/hot holds have none.
    void _loadFollowStep(float ibat, float ibatTarget, bool authority) {
        constexpr float DEADBAND_A = 0.2f, GAIN_V_PER_A = 0.004f, STEP_MAX_V = 0.02f;
        uint32_t frame = batSt.ibat_t;
        if (!authority || frame == _lastIbatFrameUs) return;
        _lastIbatFrameUs = frame;
        // keep > 0: Vout_max() ignores a non-positive pin and would release to Vbat_max
        float floor = _partialHold && !_coldBlocked
                          ? params.n_cells * (params.cv_min - params.recharge_vfloor_band) - params.vout_offset_max
                          : 0.1f;
        floor = fmaxf(floor, 0.1f);
        float err = ibat - ibatTarget;
        float step = fabsf(err) > DEADBAND_A ? fminf(fmaxf(-err * GAIN_V_PER_A, -STEP_MAX_V), STEP_MAX_V) : 0.f;
        vpack_pin = fminf(fmaxf(vpack_pin + step, floor), params.Vbat_max);
    }

    // CV hold, BMS frames only (no Vout reading). Cell frame: pack-current target = measured ibat + K (cv_eoc -
    // vcell_high) / r, r the line's cell resistance; the cell loop gain per frame is K R / r with R the cell's
    // short-term dI->dV, < 1 also with a frame of lag. K_DOWN: R < r / 1.5 (8.3 mOhm at 280 Ah with the defaults; field 2-3 mOhm), fast
    // down so a cell above cv_eoc sheds the current before the steep top runs away. K_UP: R < 2 r, slow up, the
    // polarization relaxing after a cut is not room. Units on one BMS compute the same target, so it does not add
    // up. Ibat frame: the output-current limit moves G (target - ibat); N units sum to N G (dIbat/dIlim = 1), 2 G < 1
    // also with one frame of BMS lag. Up steps <= STEP_UP_A; the limit stays <= HEADROOM_A above the own iout
    // (PV-limited: no wind-up). BMS frames older than BLIND_S: the limit decays from ibat_max towards the floor
    // (tau BLIND_TAU_S), as the expiry latch would end it; the floor holds for any number of units on the bus.
    void _cvHoldStep(bool step, float iout, float ibatCap) {
        constexpr float K_UP = 0.5f, K_DOWN = 1.5f, G = 0.4f, STEP_UP_A = 2.f, HEADROOM_A = 2.f, BLIND_TAU_S = 20.f;
        constexpr uint32_t BLIND_S = 20; // 5 ibat frames
        const uint32_t age = std::max(ibatAgeS(), coarseTicksToSec(batSt.cellAgeTicks()));
        if (step && age > BLIND_S)
            _cvIlim = fmaxf(fminf(_cvIlim, params.Ibat_lim * expf(((float) BLIND_S - (float) age) / BLIND_TAU_S)),
                            IOUT_LIM_FLOOR);
        const float ibat = batSt.ibat_raw;
        uint32_t frame = batSt.vcell_high_t;
        if (frame != _lastBmsFrameUs || std::isnan(_cvIbat)) {
            _lastBmsFrameUs = frame;
            const float err = params.cv_eoc - batSt.vcell_high;
            _cvIbat = fminf(ibat + (err > 0 ? K_UP : K_DOWN) * err * params.tail_c_rate * params.Cbat
                                   / (params.cv_eoc - params.cv_min), ibatCap);
        }
        frame = batSt.ibat_t;
        if (frame == _lastIbatFrameUs) return;
        _lastIbatFrameUs = frame;
        if (!step) return;
        float lim = fminf(fminf(_cvIlim + fminf(G * (_cvIbat - ibat), STEP_UP_A), iout + HEADROOM_A), params.Ibat_lim);
        lim = fmaxf(lim, IOUT_LIM_FLOOR);
        ESP_LOGD("charger", "cv hold: ibat %.2f -> %.2f A, ilim %.2f -> %.2f A", ibat, _cvIbat, _cvIlim, lim);
        _cvIlim = lim;
    }

    // Pack-current target of the hold, the strictest of: cold 0 A; hot the derated ibat_max; partial
    // a trim towards the Ah ceiling so a BMS current offset inside the deadband can't walk the SoC
    // away over a week (0.2 A is 34 Ah / 12 % in 7 days on 280 Ah), ±1 A at 2 Ah off.
    [[nodiscard]] float _holdIbatTarget() const {
        float t = INFINITY;
        if (_coldBlocked) t = 0.f;
        if (std::isfinite(_hotScale)) t = fminf(t, params.Ibat_lim * _hotScale);
        if (_partialHold) {
            float ceilAh = (1.f - ceiling()) * params.Cbat;
            t = fminf(t, fminf(fmaxf((batSt.coulombCounter.deficit() - ceilAh) * 0.5f, -1.f), 1.f));
        }
        return t;
    }

    void _updatePackVoltagePinning(bool voutAuthority = true, float vbat = INFINITY, float voutNow = NAN,
                                   bool psu = false, float ioutNow = NAN, bool sweep = false) {
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
        //   3) BMS-stale — glide to Vbat_fallback (after a plateau reading: from the bus voltage, over 10 min).
        //   4) Bulk — push to Vbat_max (or ride the falling-edge glide).
        // Termination transitions are glided to avoid stepping the setpoint.

        bool balancingMode = false;

        bool batDataOk = batSt.haveValidCellVoltage() and std::isfinite(params.Cbat);
        bool nowTerm = bool(termCond) || termCond.stalePaused(); // a stale pause holds float as a latch does
        if (!nowTerm) _termJoin = false;
        auto nowUs = wallClockUs();

        // Absorption target: hold the highest cell at cv_eoc and let the current taper; the
        // termination line decides when it's done. Once terminated, hold it at cv_min (no trickle).
        // The target must not follow v_term(ibat): lowering the pin drops ibat, which lowers v_term,
        // which lowers the pin again, until ibat=0 and termination latches at cv_min (premature
        // termination seen on fry/flat 2026-07).
        float v_eoc = nowTerm ? params.cv_min : params.cv_eoc;

        // CV hold before termination: the cell loop sets an output-current limit from BMS frames. The pin stays at
        // Vbat_max (a protection ceiling only), so a Vout-sensor error cannot keep power flowing. PSU mode freezes it.
        float ibat = batSt.ibatSmoothed();
        bool cvHold = termCond.cvHold();
        // Approach (hold mode): a cell within APPROACH_V of cv_eoc above the tail current overshoots (cell frames
        // seconds apart, steep top of charge). Cut the pack current to the tail current, the line's current at cv_eoc,
        // and let the hold loop take it from there; kept until the hold, a latch, stale data or a discharging pack.
        // A hold entered without that cut (line low at low ibat) keeps it armed: it cuts once on reaching the band.
        const float iTail = params.tail_c_rate * params.Cbat, ibatNow = batSt.ibat_raw;
        const float band = params.cv_eoc - APPROACH_V;
        const bool apOk = !psu && !nowTerm && (params.cv_hold_max_s || termCond.balance) && batDataOk && ibatFresh()
                          && !_partialHold && !_coldBlocked && !(ibat < 0.f);
        const bool ap = !cvHold && apOk;
        if (cvHold && !_wasCvHold) _apArm = !std::isfinite(_cvIlim);
        const bool cut = apOk && (cvHold ? _apArm : !std::isfinite(_cvIlim)) && batSt.vcell_high >= band
                         && ibatNow > iTail;
        if (cut) {
            const float lim = std::isfinite(ioutNow) && !sweep ? fmaxf(ioutNow * iTail / ibatNow, IOUT_LIM_FLOOR) : iTail;
            _cvIlim = std::isfinite(_cvIlim) ? fminf(_cvIlim, lim) : lim; // a cut never raises a hold's limit
            _cvIbat = fminf(iTail, _holdIbatTarget());
            _lastBmsFrameUs = batSt.vcell_high_t;
            _lastIbatFrameUs = batSt.ibat_t;
            _apArm = false;
            ESP_LOGW("charger", "cv approach%s: vcHigh %.3f @ iBat %.2f, ilim %.2f A", cvHold ? " (hold)" : "",
                     batSt.vcell_high, ibatNow, _cvIlim);
        }
        // Boot: cell frames near the top before termination could be evaluated (ibat warm-up): idle. Without an ibat
        // source it ends BOOT_IDLE_US after boot.
        bool boot = !_termDecided && batSt.haveValidCellVoltage()
                    && (std::isfinite(_cvIlim) || batSt.vcell_high >= band)
                    && (batSt.ibat_t || nowUs < BOOT_IDLE_US);
        if (boot) {
            if (!std::isfinite(_cvIlim)) ESP_LOGW("charger", "boot: vcHigh %.3f, idle until the BMS frames", batSt.vcell_high);
            _cvIlim = IOUT_LIM_FLOOR;
            _cvIbat = NAN;
        } else if (!cvHold && !ap) {
            _cvIlim = NAN;
        }
        if (cvHold != _wasCvHold) {
            _wasCvHold = cvHold;
            if (cvHold) {
                if (!std::isfinite(_cvIlim))
                    _cvIlim = sweep ? params.tail_c_rate * params.Cbat
                              : std::isfinite(ioutNow) ? fmaxf(ioutNow, IOUT_LIM_FLOOR) : params.Ibat_lim;
                if (!cut) _lastBmsFrameUs = ~batSt.vcell_high_t; // target from this frame
                ESP_LOGW("charger", "cv hold: vcHigh %.3f > line %.3f @ iBat %.2f, ilim %.2f A",
                         batSt.vcell_high, termCond.v_term(), ibat, _cvIlim);
            } else {
                // a latch glides to float from the bus as read now: no output step
                if (nowTerm && std::isfinite(voutNow)) vpack_pin = fminf(voutNow, params.Vbat_max);
                _cvIlim = NAN;
                ESP_LOGI("charger", "cv hold left (%s), vpPin %.3fV",
                         termCond.stalePaused() ? "paused" : nowTerm ? "terminated" : "charging", vpack_pin);
            }
        }
        if (cvHold || (ap && !boot && std::isfinite(_cvIlim))) {
            vpack_pin = params.Vbat_max;
            _vPinFilt.reset();
            _fallbackGlide.reset();
            _floatGlide.reset();
            _wasTerminated = false;
            _wasHold = false; // a load-follow hold after this one starts at the bus
            _cvHoldStep(!psu && !sweep, ioutNow, _holdIbatTarget()); // a sweep's own current dip must not pull it down
            return;
        }

        bool hold = ibatFresh() && (_partialHold || _coldBlocked || std::isfinite(_hotScale));
        if (hold != _wasHold) {
            if (hold) {
                // start at the bus voltage the pack sits at now (the bulk pin is far above it)
                float bus = batSt.vout_avg.get();
                if (std::isfinite(bus)) vpack_pin = std::isfinite(vpack_pin) ? fminf(vpack_pin, bus) : bus;
                ESP_LOGI("charger", "hold (%s): load-following from vpPin %.3fV, ibat -> %.1fA",
                         _coldBlocked ? "cold" : _partialHold ? "partial" : "hot", vpack_pin, _holdIbatTarget());
            } else {
                float from = std::isfinite(vpack_pin) ? vpack_pin : params.Vbat_max;
                _floatGlide.start(from, params.Vbat_max, nowUs);
                ESP_LOGI("charger", "hold end, gliding vpPin %.3fV -> %.3fV", from, params.Vbat_max);
            }
            _wasHold = hold;
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
                // While the pack discharges, nothing covers the load: join at the float target the
                // sibling holds. A fixed target, so a disabled converter cannot wind it up.
                constexpr float JOIN_A = 1.0f, LEAVE_A = 0.5f;
                bool fresh = ibatFresh();
                if (!fresh || ibat > LEAVE_A) _termJoin = false;
                else if (ibat < -JOIN_A) _termJoin = true;
                float target = _termJoin ? params.Vbat_fallback : params.Vbat_fallback - params.vout_offset_max;
                if (std::isnan(vpack_pin) || fabsf(vpack_pin - target) > 0.01f)
                    ESP_LOGW("charger", "no Vout authority (terminated): %s %.3f -> %.3f (ibat %.1fA)",
                             _termJoin ? "join" : "yield", vpack_pin, target, ibat);
                vpack_pin = target;
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
            // No glide when termination latched without authority: hold the float target directly.
            _vPinFilt.reset();
            _fallbackGlide.reset();
            vpack_pin = _floatGlide.active() ? _floatGlide.value(nowUs) : params.Vbat_fallback;
        } else if (!batDataOk) {
            // missing bat data -> glide to the fallback
            _vPinFilt.reset();
            _floatGlide.reset();
            // A cell-data gap after a plateau reading: cap at the bus voltage it left, ramping down to the
            // float voltage over STALE_HOLD_SEC, instead of dropping below a charging pack at once.
            // The cap is latched once per gap so a pin release cannot raise it.
            constexpr float STALE_HOLD_CELL_MARGIN = 0.05f, STALE_HOLD_HEADROOM = 0.1f; // [V]
            constexpr uint32_t STALE_HOLD_SEC = 600;
            const uint32_t age = batSt.cellAgeTicks(),
                    t0 = secToCoarseTicks(BatteryState::VCELL_EXPIRATION_TIME_SEC),
                    span = secToCoarseTicks(STALE_HOLD_SEC);
            float from = std::isfinite(vpack_pin) ? vpack_pin : params.Vbat_fallback;
            if (_staleCapTicks != batSt.vcell_high_ticks) {
                _staleCapTicks = batSt.vcell_high_ticks;
                const float bus = fminf(batSt.vout_avg.get(), voutNow); // voutNow NAN: the average alone
                _staleCap = (!nowTerm && std::isfinite(params.Cbat) && batSt.vcell_high > 0 &&
                             batSt.vcell_high < params.cv_min + STALE_HOLD_CELL_MARGIN &&
                             std::isfinite(batSt.vout_avg.get()) && bus > 0 && age < t0 + span)
                                ? fmaxf(params.Vbat_fallback, fminf(from, bus + STALE_HOLD_HEADROOM))
                                : NAN;
            }
            float to = params.Vbat_fallback;
            if (std::isfinite(_staleCap) && !nowTerm)
                to -= (params.Vbat_fallback - _staleCap) *
                        (1.f - fminf(1.f, (float) (age > t0 ? age - t0 : 0) / (float) span));
            if (!_fallbackGlide.active()) {
                // entering fallback — capture current pin as the glide origin
                _fallbackGlide.start(from, to, nowUs);
                auto what = !batSt.haveValidCellVoltage() ? "Cell Voltage" : "Pack Capacity";
                ESP_LOGW("charger", "%s n/a, gliding vpPin %.3fV -> %.3fV over %ums%s",
                         what, from, to, (unsigned) (_fallbackGlide.durationUs() / 1000),
                         std::isfinite(_staleCap) ? ", then to the fallback" : "");
            }
            const float g0 = _fallbackGlide.from();
            vpack_pin = g0 + (to - g0) * _fallbackGlide.progress(nowUs);
        } else {
            // bulk charging
            _vPinFilt.reset();
            _fallbackGlide.reset();
            // ride the falling-edge glide if it's still ramping, otherwise jump
            vpack_pin = _floatGlide.active() ? _floatGlide.value(nowUs) : params.Vbat_max;
        }
    }


    __attribute__((cold)) void beginMqtt(const ConfFile &mqttConf) {
        auto topic = mqttConf.getString("cell_voltages_max_topic", "");
        if (!topic.empty()) {
            _bmsCellSource = true;
            if (std::isnan(vpack_pin)) vpack_pin = params.Vbat_fallback; // an MQTT restart keeps the live pin
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                float v = strntof(dat, len);
                if (!std::isfinite(v) || v <= 0) {
                    LOG_VALUE_IGNORED("charger", "Vcell_high", len, dat);
                    return;
                }
                batSt.setVcellHigh(v);
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
        _tempSource = !topic.empty();
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

        // BMS remaining charge [Ah]: a deficit shared by all chargers on this BMS
        topic = mqttConf.getString("bat_charge_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                float ah = strntof(dat, len);
                if (!std::isfinite(ah) || ah < 0) {
                    LOG_VALUE_IGNORED("charger", "bat_charge", len, dat);
                    return;
                }
                batSt.coulombCounter.setBmsCharge(ah, wallClockTicks());
            });

        // batmon-ha: <dev>/cell_voltages/delta [V]
        topic = mqttConf.getString("cell_voltages_delta_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                float d = strntof(dat, len);
                if (!std::isfinite(d) || d < 0) {
                    LOG_VALUE_IGNORED("charger", "cell_delta", len, dat);
                    return;
                }
                batSt.setCellDelta(d);
            });

        topic = mqttConf.getString("bat_voltage_topic", "");
        if (!topic.empty())
            MQTT.subscribeTopic(topic, [&](const char *dat, int len) {
                float v = strntof(dat, len);
                if (!std::isfinite(v) || v <= 0) {
                    LOG_VALUE_IGNORED("charger", "Vpack", len, dat);
                    return;
                }
                batSt.vpack = v;
                batSt.vpack_t = static_cast<uint32_t>(wallClockUs());
            });

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

    void update(float vout, float iout, bool voutAuthority = true, bool psu = false, bool sweep = false) {
        batSt.update(vout, iout);
        _updateTermination();
        _updateTempLimit();
        _updateHoliday();
        _updateBalance();
        _updatePartialHold();
        _updatePackVoltagePinning(voutAuthority, INFINITY, vout, psu, iout, sweep);
        _saveRtc();
    }

    // One wire fit sample per pack-voltage frame, while switching at a current that held steady
    // since the previous frame (BMS and converter samples are seconds apart).
    void updateWire(float vout, float iout, bool switching) {
        const uint32_t t = batSt.vpack_t;
        if (t == _wireLastT) return;
        _wireLastT = t;
        const bool use = switching && iout > 0.3f && std::abs((int32_t) (t - batSt.ibat_t)) < 2000000;
        if (use && std::fabs(iout - _wirePrevI) < 0.5f) {
            wire.add(vout - batSt.vpack, iout, batSt.ibat_raw, (uint32_t) (wallClockUs() / 1000000));
            wireGuard.frame(vout - batSt.vpack, iout, batSt.ibat_raw);
        }
        _wirePrevI = use ? iout : NAN;
    }

    [[nodiscard]] bool partialHold() const { return _partialHold; }
    [[nodiscard]] uint8_t holidayStreak() const { return _hStreak; }
    [[nodiscard]] uint8_t holidayGaps() const { return _hGaps; }
    [[nodiscard]] float holidayDischargeH() const { return _hDisS * (1.f / 3600.f); }
    [[nodiscard]] bool chargeBlocked() const { return _coldBlocked; }
    // the pack wants no charge (terminated, at the partial ceiling, or too cold) or is held at cv_eoc
    [[nodiscard]] bool chargeHold() const {
        return bool(termCond) || _partialHold || _coldBlocked || termCond.cvHold() || termCond.stalePaused();
    }
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

        if (std::isfinite(ioutLim)) lim = min(lim, ioutLim);
        if (std::isfinite(_cvIlim)) lim = min(lim, _cvIlim);
        const float wireCap = wireGuard.ioutCap();
        if (std::isfinite(wireCap)) lim = min(lim, wireCap);
        return fmaxf(lim, IOUT_LIM_FLOOR); // the CC limiter normalizes by its setpoint; 0 (e.g. ibat_lim_topic) would divide by 0
    }
};
