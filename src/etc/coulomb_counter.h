#pragma once

#include <atomic>

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
 * bound, after a longer gap or during a reboot is taken as real.
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
    static constexpr uint32_t BMS_CHARGE_EXPIRE_S = 120;
    static constexpr float BAT_I_MAX_A = 200.f; // pack fuse
    static constexpr float BRIDGE_MAX_S = 120.f;
    static constexpr float DELIVERY_S = 12.f; // one BMS publish period: late, batched or repeated values

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

    // producer thread
    void setBmsCharge(float ah, uint32_t ticks) {
        float in = (float) _integrator.get(), off = _bmsOffset.load(std::memory_order_relaxed);
        float prev = _bmsCharge.load(std::memory_order_relaxed) - off;
        float dtS = (float) (int32_t) (ticks - _bmsChargeTicks.load(std::memory_order_relaxed)) * COARSE_TICK_S;
        float lim = BAT_I_MAX_A * fmaxf(dtS, DELIVERY_S) * (1.f / 3600.f), dc = ah - prev;
        if (dtS <= BRIDGE_MAX_S && std::fabs(dc) > lim + 0.5f) {
            float x = _integrator.last(); // -ibat
            float est = _intAtCharge - in + (std::isfinite(x) ? -x * fmaxf(dtS - _covS, 0.f) * (1.f / 3600.f) : 0.f);
            est = fmaxf(-lim, fminf(est, lim));
            off += est - dc;
            _bmsOffset.store(off, std::memory_order_relaxed);
            ESP_LOGW("charger", "BMS charge re-seed %.2f -> %.2f Ah in %.0f s, taken as %+.2f Ah", prev, ah, dtS, est);
        }
        _bmsCharge.store(ah + off, std::memory_order_relaxed);
        _bmsChargeTicks.store(ticks, std::memory_order_relaxed);
        _intAtCharge = in;
        _covS = 0.f;
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

    // consumer: BMS charge at the last full in the BMS's raw scale, and its restore after a reboot
    [[nodiscard]] float bmsRef() const { return _bmsRef - _bmsOffset.load(std::memory_order_relaxed); }
    void restoreRef(float ref) { _bmsRef = ref + _bmsOffset.load(std::memory_order_relaxed); }

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
