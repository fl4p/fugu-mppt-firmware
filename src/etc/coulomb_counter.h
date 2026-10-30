#pragma once

#include <atomic>

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
 * latch on the same cell frame hold the same reference. Signed, never clamped.
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

public:
    static constexpr uint32_t BMS_CHARGE_EXPIRE_S = 120;

    void updateBatCurrent(float ibat) { updateBatCurrent(ibat, wallClockUs()); }

    // producer thread
    void updateBatCurrent(float ibat, time_us nowUs) {
        if (_resetRequested.exchange(false, std::memory_order_relaxed))
            _integrator.reset();
        _integrator.add(-ibat, nowUs);
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
        _bmsCharge.store(ah, std::memory_order_relaxed);
        _bmsChargeTicks.store(ticks, std::memory_order_relaxed);
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

    // consumer thread, once per cell frame: deepest deficit since full (the recharge release depth)
    float updateDepth() {
        if (!std::isfinite(_bmsRef)) _bmsRef = _bmsAtCell.load(std::memory_order_relaxed); // no full since boot
        float d = deficit();
        if (d > _depth) _depth = d;
        return _depth;
    }
};
