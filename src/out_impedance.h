#pragma once

#include <cmath>
#include <cstdint>

/**
 * Online estimate of the output source resistance R = dV/dI (wiring + battery ohmic part).
 * Samples are averaged into fixed-length blocks; differences of consecutive block means enter an
 * exponentially-forgetting instrumental-variable fit R = Σ(dV·dD) / Σ(dI·dD), with the duty
 * step dD as instrument. Voltage changes from other sources on the bus move I as well (through the
 * converter's input characteristic) but are uncorrelated with the own duty steps, so they don't
 * bias R; current noise doesn't attenuate it either. Block differencing removes slow drift.
 * get() is NAN without recent, sufficiently correlated excitation.
 */
class OutputImpedance {
    float sV = 0, sI = 0, sD = 0;   // current block sums
    float vPrev = NAN, iPrev = NAN, dPrev = NAN; // previous block means
    float svd = 0, sid = 0, sdd = 0, sii = 0;
    uint32_t blockStartUs = 0, lastUs = 0, lastStepUs = 0;
    uint16_t nBlk = 0;
    uint16_t nSteps = 0;            // accepted steps, saturating
    float r = NAN;

public:
    static constexpr uint32_t BLOCK_US = 20000;
    static constexpr uint32_t STALE_US = 60'000'000;
    static constexpr uint16_t MIN_STEPS = 32;
    static constexpr float FORGET = 1.f - 1.f / 256.f;
    static constexpr float MIN_CORR2 = 0.25f; // squared correlation of dI and dD

    // drops the open block and the difference reference, keeps the fit
    void reset() {
        sV = sI = sD = 0;
        nBlk = 0;
        vPrev = iPrev = dPrev = NAN;
    }

    // d: duty (PWM counts) commanded during the sample
    void add(float v, float i, float d, uint32_t nowUs) {
        if (!std::isfinite(v) || !std::isfinite(i) || nowUs - lastUs > 2 * BLOCK_US) reset();
        lastUs = nowUs;
        if (!std::isfinite(v) || !std::isfinite(i)) return;

        if (!nBlk) blockStartUs = nowUs;
        sV += v;
        sI += i;
        sD += d;
        if (++nBlk < 2 || nowUs - blockStartUs < BLOCK_US) return;

        const float n = (float) nBlk;
        float vm = sV / n, im = sI / n, dm = sD / n;
        sV = sI = sD = 0;
        nBlk = 0;
        float dV = vm - vPrev, dI = im - iPrev, dD = dm - dPrev;
        vPrev = vm;
        iPrev = im;
        dPrev = dm;
        if (!(std::fabs(dD) >= 0.5f)) return; // no own step (or first block, NAN)

        svd = svd * FORGET + dV * dD;
        sid = sid * FORGET + dI * dD;
        sdd = sdd * FORGET + dD * dD;
        sii = sii * FORGET + dI * dI;
        lastStepUs = nowUs;
        if (nSteps < 0xffff) ++nSteps;
        r = (nSteps >= MIN_STEPS && sid > 0 && sid * sid >= MIN_CORR2 * sdd * sii) ? svd / sid : NAN;
    }

    // [Ω]
    [[nodiscard]] float get(uint32_t nowUs) const { return nowUs - lastStepUs > STALE_US ? NAN : r; }

    [[nodiscard]] uint16_t steps() const { return nSteps; }
};
