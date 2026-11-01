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
 * get() is NAN without recent, sufficiently correlated excitation. hold() suspends the fit for
 * HOLD_US, for periods where the duty is not an independent input (closed-loop regulation).
 */
class OutputImpedance {
    float sV = 0, sI = 0, sD = 0;   // current block sums
    float vPrev = NAN, iPrev = NAN, dPrev = NAN; // previous block means
    float svd = 0, sid = 0, sdd = 0, sii = 0;
    uint32_t blockStartUs = 0, lastUs = 0, lastStepUs = 0, holdUs = 0;
    uint16_t nBlk = 0;
    uint16_t nSteps = 0;            // accepted steps, saturating
    float r = NAN;

public:
    static constexpr uint32_t BLOCK_US = 20000;
    static constexpr uint32_t STALE_US = 60'000'000;
    static constexpr uint32_t HOLD_US = 2'000'000;
    static constexpr uint16_t MIN_STEPS = 32;
    static constexpr float FORGET = 1.f - 1.f / 256.f;
    static constexpr float MIN_CORR2 = 0.25f; // squared correlation of dI and dD

    // drops the open block and the difference reference, keeps the fit
    void reset() {
        sV = sI = sD = 0;
        nBlk = 0;
        vPrev = iPrev = dPrev = NAN;
    }

    void hold(uint32_t nowUs) {
        reset();
        holdUs = nowUs;
    }

    // d: duty (PWM counts) commanded during the sample
    void add(float v, float i, float d, uint32_t nowUs) {
        const bool bad = !std::isfinite(v) || !std::isfinite(i) || nowUs - holdUs < HOLD_US;
        if (bad || nowUs - lastUs > 2 * BLOCK_US) reset();
        lastUs = nowUs;
        if (bad) return;

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

/**
 * Wire resistances from an external pack voltage: exponentially-forgetting least squares of
 * dv = a + R1·i1 + R2·i2, with dv the own output voltage minus the pack voltage, i1 the own
 * output current and i2 the battery current. a absorbs a constant calibration offset. Results are
 * NAN until the fit has enough weight and both its residual and the standard error of R1 are small,
 * so a sensor whose error is not a constant offset reports nothing rather than a resistance.
 * Also NAN after STALE_S without a sample.
 */
class WireResistance {
    double s[6]{}; // XᵀX upper triangle, x = (1, i1, i2): 00 01 02 11 12 22
    double xy[3]{}, yy = 0;
    float r1 = NAN, r2 = NAN, sigma = NAN;
    uint32_t lastS = 0;

public:
    static constexpr double FORGET = 1. - 1. / 2048.;
    static constexpr double MIN_WEIGHT = 200;
    static constexpr float MAX_SIGMA = 0.04f; // [V] residual
    static constexpr float MAX_SE = 0.002f;   // [Ω] standard error of R1
    static constexpr uint32_t STALE_S = 3600;

    void add(float dv, float i1, float i2, uint32_t nowS) {
        if (!std::isfinite(dv) || !std::isfinite(i1) || !std::isfinite(i2)) return;
        lastS = nowS;
        const double x[3]{1, i1, i2};
        for (int k = 0, a = 0; a < 3; ++a)
            for (int b = a; b < 3; ++b, ++k) s[k] = s[k] * FORGET + x[a] * x[b];
        for (int a = 0; a < 3; ++a) xy[a] = xy[a] * FORGET + x[a] * dv;
        yy = yy * FORGET + (double) dv * dv;
        solve();
    }

    [[nodiscard]] float own(uint32_t nowS) const { return nowS - lastS > STALE_S ? NAN : r1; }    // R1 [Ω]
    [[nodiscard]] float shared(uint32_t nowS) const { return nowS - lastS > STALE_S ? NAN : r2; } // R2 [Ω]
    [[nodiscard]] float residual() const { return sigma; }

private:
    void solve() {
        r1 = r2 = sigma = NAN;
        const double a = s[0], b = s[1], c = s[2], d = s[3], e = s[4], f = s[5];
        if (a < MIN_WEIGHT) return;
        const double c00 = d * f - e * e, c01 = c * e - b * f, c02 = b * e - c * d;
        const double det = a * c00 + b * c01 + c * c02;
        if (!(std::fabs(det) > 1e-9 * a * d * f)) return; // singular: currents never varied
        const double c11 = a * f - c * c, c12 = b * c - a * e, c22 = a * d - b * b;
        const double p0 = (c00 * xy[0] + c01 * xy[1] + c02 * xy[2]) / det;
        const double p1 = (c01 * xy[0] + c11 * xy[1] + c12 * xy[2]) / det;
        const double p2 = (c02 * xy[0] + c12 * xy[1] + c22 * xy[2]) / det;
        const double sse = yy - (p0 * xy[0] + p1 * xy[1] + p2 * xy[2]);
        const double s2 = (sse > 0 ? sse : 0) / (a - 3);
        sigma = (float) std::sqrt(s2);
        if (!(sigma <= MAX_SIGMA) || !(std::sqrt(s2 * c11 / det) <= MAX_SE)) return;
        r1 = (float) p1;
        r2 = (float) p2;
    }
};
