#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

/**
 * Output source resistance R = dV/dI (wiring, BMS path, battery ohmic part) from one monotonic
 * current ramp: a least-squares line V = a + R·I over BLOCK_US block means of the samples between
 * reset() and finish(). The result replaces the previous one only if the fit passes the span,
 * residual and standard-error gates; get() is NAN once it is older than STALE_S.
 * add()/reset()/finish() run on one core; get() may run on another.
 */
class OutputImpedance {
    float bV = 0, bI = 0;           // open block sums, relative to (v0, i0)
    float v0 = 0, i0 = 0;
    uint32_t blkUs = 0;
    uint16_t nBlk = 0, n = 0;
    double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
    float iMin = 0, iMax = 0;
    std::atomic<float> r{NAN};
    std::atomic<uint32_t> rS{0};    // finish() time of r [s]

    void closeBlock() {
        const float k = 1.f / (float) nBlk;
        const double x = bI * k, y = bV * k;
        nBlk = 0;
        bV = bI = 0;
        if (!n++) iMin = iMax = (float) x;
        else if (x < iMin) iMin = (float) x;
        else if (x > iMax) iMax = (float) x;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        syy += y * y;
    }

public:
    static constexpr uint32_t BLOCK_US = 100'000;
    static constexpr uint32_t STALE_S = 6 * 3600; // sweeps run every 30 min while charging only
    static constexpr uint16_t MIN_N = 20;
    static constexpr float MIN_SPAN = 3.f;     // [A]
    static constexpr float MAX_RES = 0.04f;    // residual RMS [V]
    static constexpr float MAX_SE = 0.003f;    // standard error of R [Ω]

    struct Fit {
        float r, se, res, span;
        uint16_t n;
        bool ok;
    };

    void reset() { n = nBlk = 0; }

    void add(float v, float i, uint32_t nowUs) {
        if (!std::isfinite(v) || !std::isfinite(i)) return;
        if (!n && !nBlk) {
            v0 = v;
            i0 = i;
            bV = bI = 0;
            sx = sy = sxx = sxy = syy = 0;
        } else if (nBlk && nowUs - blkUs >= BLOCK_US) closeBlock();
        if (!nBlk) blkUs = nowUs;
        bV += v - v0;
        bI += i - i0;
        ++nBlk;
    }

    // evaluates the ramp since reset(), publishes R if it passes the gates, then resets
    Fit finish(uint32_t nowS) {
        if (nBlk) closeBlock();
        Fit f{NAN, NAN, NAN, n ? iMax - iMin : 0.f, n, false};
        if (n >= 3) {
            const double m = n, cxx = sxx - sx * sx / m, cxy = sxy - sx * sy / m, cyy = syy - sy * sy / m;
            const double b = cxy / cxx, ssr = std::fmax(cyy - b * cxy, 0.);
            f.r = (float) b;
            f.res = (float) std::sqrt(ssr / (m - 2));
            f.se = (float) std::sqrt(ssr / (m - 2) / cxx);
            f.ok = n >= MIN_N && f.span >= MIN_SPAN && f.res <= MAX_RES && f.se <= MAX_SE && f.r > 0 &&
                   std::isfinite(f.r);
        }
        if (f.ok) {
            r.store(f.r, std::memory_order_relaxed);
            rS.store(nowS ? nowS : 1, std::memory_order_release);
        }
        reset();
        return f;
    }

    // [Ω]
    [[nodiscard]] float get(uint32_t nowS) const {
        const uint32_t t = rS.load(std::memory_order_acquire);
        return t && nowS - t <= STALE_S ? r.load(std::memory_order_relaxed) : NAN;
    }

    // finish() time of the value get() returns, 0 = none; changes once per qualifying sweep
    [[nodiscard]] uint32_t stamp() const { return rS.load(std::memory_order_acquire); }
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
    float r0 = NAN, r1 = NAN, r2 = NAN, sigma = NAN;
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
    [[nodiscard]] float offset(uint32_t nowS) const { return nowS - lastS > STALE_S ? NAN : r0; } // a [V]
    [[nodiscard]] float residual() const { return sigma; }

private:
    void solve() {
        r0 = r1 = r2 = sigma = NAN;
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
        r0 = (float) p0;
        r1 = (float) p1;
        r2 = (float) p2;
    }
};
