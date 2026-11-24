#pragma once

#include <atomic>
#include <cstring>

#include "esp_log.h"

#include "math/statmath.h"
#include "util.h"


/**
 * Coulomb counter tracking battery DoD via trapezoidal integration.
 * maxDt = 30 s drops integration across BMS dropouts longer than that.
 *
 * Lock-free single-core SPSC. The integrator is mutated only by the producer
 * (BMS/MQTT task) in updateBatCurrent(); the consumer (loop task) reads the
 * published Ah snapshot and requests a reset via markFull(). Producer and
 * consumer are both pinned to core 0, so a 32-bit atomic load/store (lock-free
 * on Xtensa) is enough — no critical section.
 *
 * Optional BMS remaining-charge source (setBmsCharge): while fresh it replaces the integrator, so
 * parallel chargers on one BMS share the same deficit. The charge is sampled on each cell frame
 * (sampleOnCellFrame) and the full-point reference is taken from that sample, so chargers that
 * latch on the same cell frame hold the same reference. Signed, never clamped. While terminated the
 * BMS reference follows the charge up, so chargers that latched on different frames and see the
 * same peak converge on it.
 *
 * The BMS may re-seed its charge (e.g. to 0.9 x nominal). A charge step larger than BAT_I_MAX_A
 * can move within BRIDGE_MAX_S of the previous sample (at least DELIVERY_S, + 0.5 Ah) is a re-seed: the step is replaced
 * by the ibat estimate for that window and the difference kept as an offset. A re-seed within that
 * bound or after a longer gap is taken as real. Rtc keeps the last raw sample, its RTC slow-clock time and the
 * offset across a soft reset, so the first sample after boot gets the same check (the ibat estimate then has only
 * the ibat since boot, else 0 A) and the offset scale continues: a ref restored in it stays consistent.
 */
class CoulombCounter {
    TrapezoidalIntegrator<> _integrator{1.f / 3600e6f, 30'000'000ULL}; // producer-only
    std::atomic<float> _ahSinceFull{0.f}; // single writer: producer
    std::atomic<bool> _resetRequested{false}; // consumer -> producer
    std::atomic<float> _bmsCharge{NAN}; // [Ah] remaining, producer
    std::atomic<uint32_t> _bmsChargeTicks{0}; // wallClockTicks() of _bmsCharge
    std::atomic<float> _bmsAtCell{NAN}; // _bmsCharge sampled on the last cell frame, NAN if stale
    float _bmsRef = NAN; // [Ah] charge at the last full (or first sample), consumer
    float _depth = 0.f; // deepest deficit since full, consumer
    std::atomic<float> _bmsOffset{0.f}; // [Ah] added to the raw BMS charge, producer
    float _intAtCharge = 0.f; // integrator at the last charge sample, producer
    float _covS = 0.f; // integrated ibat time since the last charge sample, producer

public:
    // Last raw BMS charge, kept in RTC RAM across a soft reset (not a power loss). Producer only.
    struct Rtc {
        float ah, off; // raw charge and the offset at that sample
        uint64_t t; // RTC slow-clock ticks of ah
        uint32_t sum;
        [[nodiscard]] __attribute__((noinline)) uint32_t checksum() const {
            uint32_t h = 0x42514332u, w[4];
            memcpy(w, this, sizeof w);
            for (uint32_t x: w) h = (h ^ x) * 16777619u;
            return h;
        }
        [[nodiscard]] bool valid() const { return sum == checksum() && std::isfinite(ah) && std::isfinite(off); }
    };
private:
    Rtc *_rtc = nullptr; // producer

public:
    static constexpr uint32_t BMS_CHARGE_EXPIRE_S = 120;
    static constexpr float BAT_I_MAX_A = 200.f; // pack fuse
    static constexpr float BRIDGE_MAX_S = 120.f;
    static constexpr float DELIVERY_S = 12.f; // one BMS publish period: late, batched or repeated values
    static constexpr float RTC_TOL_S = 3.f; // RTC vs uptime clock mismatch the restart check tolerates

    void updateBatCurrent(float ibat) { updateBatCurrent(ibat, wallClockUs()); }

    // producer thread
    void updateBatCurrent(float ibat, time_us nowUs) {
        if (_resetRequested.exchange(false, std::memory_order_relaxed)) {
            _intAtCharge -= (float) _integrator.get();
            _integrator.reset();
        }
        _covS += (float) _integrator.add(-ibat, nowUs) * 1e-6f;
        _ahSinceFull.store((float) _integrator.get(), std::memory_order_relaxed);
    }

    // consumer thread. A pending markFull() reads as 0 until the producer
    // honors it, so the integrator stays single-writer without a revert window.
    [[nodiscard]] float ahSinceFull() const {
        if (_resetRequested.load(std::memory_order_relaxed)) return 0.f;
        return _ahSinceFull.load(std::memory_order_relaxed);
    }

    // consumer thread: zero now (via the pending flag) and reset the integrator
    // on the next producer update (~1 BMS cycle later).
    void markFull() {
        _resetRequested.store(true, std::memory_order_relaxed);
        _bmsRef = _bmsAtCell.load(std::memory_order_relaxed);
        _depth = 0.f;
    }

    // Before the first setBmsCharge. refOff: the offset the restored ref was saved with, NAN if unknown (then
    // r is dropped: it may predate the ref). The offset continues from r, else from refOff.
    void attachRtc(Rtc *r, float refOff) {
        _rtc = r;
        if (r && std::isnan(refOff)) r->sum = ~r->checksum();
        _bmsOffset.store(r && r->valid() ? r->off : std::isfinite(refOff) ? refOff : 0.f, std::memory_order_relaxed);
    }

    // producer thread. rtc: RTC slow-clock ticks (reset only with the RTC domain) of tickUs each, 0 = none
    void setBmsCharge(float ah, uint32_t ticks, uint64_t rtc = 0, float tickUs = 0.f) {
        float in = (float) _integrator.get(), off = _bmsOffset.load(std::memory_order_relaxed);
        float prev = _bmsCharge.load(std::memory_order_relaxed) - off;
        float dtS = (float) (int32_t) (ticks - _bmsChargeTicks.load(std::memory_order_relaxed)) * COARSE_TICK_S;
        if (std::isnan(prev) && rtc && _rtc && _rtc->valid()) {
            // first sample since boot. The RTC gap must span this boot's uptime, else the RTC clock restarted.
            dtS = (float) (rtc - _rtc->t) * tickUs * 1e-6f;
            if (dtS >= (float) ticks * COARSE_TICK_S - RTC_TOL_S) prev = _rtc->ah;
        }
        float lim = BAT_I_MAX_A * fmaxf(dtS, DELIVERY_S) * (1.f / 3600.f), dc = ah - prev;
        float est = NAN;
        if (dtS <= BRIDGE_MAX_S && std::fabs(dc) > lim + 0.5f) {
            float x = _integrator.last(); // -ibat
            est = _intAtCharge - in + (std::isfinite(x) ? -x * fmaxf(dtS - _covS, 0.f) * (1.f / 3600.f) : 0.f);
            est = fmaxf(-lim, fminf(est, lim));
            off += est - dc;
        }
        // the sample (with its offset) is committed before the offset is visible to the consumer's ChargerRtc save
        if (_rtc && rtc) {
            _rtc->ah = ah;
            _rtc->off = off;
            _rtc->t = rtc;
            _rtc->sum = _rtc->checksum();
        }
        _bmsOffset.store(off, std::memory_order_relaxed);
        _bmsCharge.store(ah + off, std::memory_order_relaxed);
        _bmsChargeTicks.store(ticks, std::memory_order_relaxed);
        _intAtCharge = in;
        _covS = 0.f;
        if (std::isfinite(est))
            ESP_LOGW("charger", "BMS charge re-seed %.2f -> %.2f Ah in %.0f s, taken as %+.2f Ah", prev, ah, dtS, est);
    }

    // producer thread, on each BMS cell frame
    void sampleOnCellFrame(uint32_t ticks) {
        float c = _bmsCharge.load(std::memory_order_relaxed);
        bool fresh = ticks - _bmsChargeTicks.load(std::memory_order_relaxed) < secToCoarseTicks(BMS_CHARGE_EXPIRE_S);
        _bmsAtCell.store(fresh ? c : NAN, std::memory_order_relaxed);
    }

    [[nodiscard]] bool bmsSource() const {
        return std::isfinite(_bmsRef) && std::isfinite(_bmsAtCell.load(std::memory_order_relaxed));
    }

    // consumer thread: signed deficit since full, from the BMS charge while fresh, else the integrator
    [[nodiscard]] float deficit() const {
        return bmsSource() ? _bmsRef - _bmsAtCell.load(std::memory_order_relaxed) : ahSinceFull();
    }

    [[nodiscard]] float depth() const { return _depth; }

    // consumer: BMS charge at the last full in the BMS's raw scale (bmsRef = ref - offset), and the restore of ref
    [[nodiscard]] float bmsRef() const { return _bmsRef - offset(); }
    [[nodiscard]] float ref() const { return _bmsRef; }
    [[nodiscard]] float offset() const { return _bmsOffset.load(std::memory_order_relaxed); }
    void restoreRef(float ref) { _bmsRef = ref; }

    // consumer thread, once per cell frame: deepest deficit since full (the recharge release depth).
    // While terminated the full point follows the charge up (a sibling charging, or a latch on it only).
    float updateDepth(bool terminated = false) {
        float c = _bmsAtCell.load(std::memory_order_relaxed); // one snapshot for the decision and the ref
        if (!std::isfinite(_bmsRef)) _bmsRef = c; // no full since boot
        bool bms = std::isfinite(_bmsRef) && std::isfinite(c);
        float d = bms ? _bmsRef - c : ahSinceFull();
        if (terminated && bms && d < 0.f) _bmsRef = c, _depth = d = 0.f;
        if (d > _depth) _depth = d;
        return _depth;
    }
};
