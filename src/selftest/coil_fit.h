#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

// Coil inductance from a DCM duty sweep of a buck (etc/measure_coil.py is the reference, same algorithm).
// Per point the DCM transfer relation gives 1/L = 2*Vout*fsw*Iout / ((Vin-Vout)*Vin*D^2). The switch
// node rings after the coil current reaches zero (L against the node capacitance) and the ring phase at
// HS turn-on moves Iout by +-10..25 %; the phase is the ring frequency times the dead time
// t = (1 - D*Vin/Vout)/fsw. So 1/L_i is fitted as c0 + sum_h (a_h cos(2 pi h f t_i) + b_h sin(...)),
// f scanned between the span and the sampling limit, L = 1/c0 (the mean over whole ring periods).
// Without enough points or a significant (>= 1 %) periodic term the median of the points is returned.
namespace coilfit {

constexpr int kMaxPts = 64;
constexpr int kMinFitPts = 16;
constexpr float kFLo = 80e3f, kFHi = 1.2e6f; // ring frequency search range, Hz

// DCM duty band [lo, hi] (HS counts) and step for `steps` points between lo and hi * M * pwmMax
inline bool band(float vin, float vout, uint16_t pwmMax, uint16_t pwmCtrlMax, int steps, int &lo, int &hi,
                 int &step) {
    if (!(vin > vout + 1.0f) || !(vout > 0.0f) || steps < 2) return false;
    const float m = vout / vin;
    lo = std::max(2, (int) std::lround(0.25f * m * (float) pwmMax));
    hi = std::min((int) pwmCtrlMax, (int) std::lround(0.9f * m * (float) pwmMax));
    step = std::max(1, (hi - lo) / (steps - 1));
    return hi > lo;
}

struct Result {
    float L = NAN;       // H
    float Lmed = NAN;    // median of the per-point values, H
    float f = 0;         // ring frequency, Hz (0 = not resolved)
    float amp = 0;       // first-harmonic ripple of 1/L, relative
    float periods = 0;   // dead-time span in ring periods
    float se = NAN;      // standard error of L, relative
    int n = 0, nh = 0;
};

namespace detail {
// least squares y ~ A c (A columns: 1, cos/sin of h*f*t), returns the residual sum of squares
inline float solve(const float *y, const float *t, int n, float f, int nh, float c[5]) {
    const int p = 1 + 2 * nh;
    float N[5][6] = {};
    for (int i = 0; i < n; ++i) {
        float a[5];
        a[0] = 1;
        const float w = 6.2831853f * f * t[i];
        const float cw = std::cos(w), sw = std::sin(w);
        a[1] = cw, a[2] = sw;
        if (nh > 1) a[3] = cw * cw - sw * sw, a[4] = 2 * sw * cw;
        for (int r = 0; r < p; ++r) {
            for (int k = 0; k < p; ++k) N[r][k] += a[r] * a[k];
            N[r][p] += a[r] * y[i];
        }
    }
    for (int k = 0; k < p; ++k) { // Gauss-Jordan, partial pivoting
        int piv = k;
        for (int r = k + 1; r < p; ++r) if (std::fabs(N[r][k]) > std::fabs(N[piv][k])) piv = r;
        if (std::fabs(N[piv][k]) < 1e-9f) return INFINITY;
        if (piv != k) for (int j = 0; j <= p; ++j) std::swap(N[k][j], N[piv][j]);
        for (int r = 0; r < p; ++r) {
            if (r == k) continue;
            const float m = N[r][k] / N[k][k];
            for (int j = k; j <= p; ++j) N[r][j] -= m * N[k][j];
        }
    }
    for (int k = 0; k < p; ++k) c[k] = N[k][p] / N[k][k];
    float rss = 0;
    for (int i = 0; i < n; ++i) {
        const float w = 6.2831853f * f * t[i];
        float e = y[i] - c[0] - c[1] * std::cos(w) - c[2] * std::sin(w);
        if (nh > 1) e -= c[3] * std::cos(2 * w) + c[4] * std::sin(2 * w);
        rss += e * e;
    }
    return rss;
}

inline float median(float *a, int n) { // sorts a
    for (int i = 1; i < n; ++i) {
        const float v = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > v) a[j + 1] = a[j], --j;
        a[j + 1] = v;
    }
    return n ? (n & 1 ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2])) : NAN;
}
}

// d: HS duty fraction, vin/vout [V], iout [A] of the accepted DCM points (n <= kMaxPts); fsw [Hz]
inline Result fit(const float *d, const float *vin, const float *vout, const float *iout, int n, float fsw) {
    Result r;
    if (n > kMaxPts) n = kMaxPts;
    float y[kMaxPts], t[kMaxPts], s[kMaxPts];
    int m = 0;
    for (int i = 0; i < n; ++i) {
        const float x = (vin[i] - vout[i]) * vin[i] * d[i] * d[i];
        if (!(x > 0) || !(iout[i] > 0) || !(vout[i] > 0)) continue;
        y[m] = 2 * vout[i] * fsw * iout[i] / x;
        t[m] = (1 - d[i] * vin[i] / vout[i]) / fsw;
        s[m] = y[m];
        ++m;
    }
    r.n = m;
    if (m < 3) return r;
    const float ymed = detail::median(s, m);
    r.Lmed = r.L = 1 / ymed;
    if (m < kMinFitPts) return r;
    float y0 = 0, tMin = t[0], tMax = t[0];
    for (int i = 0; i < m; ++i) {
        y[i] /= ymed; // O(1) for float sums
        y0 += y[i];
        tMin = std::min(tMin, t[i]), tMax = std::max(tMax, t[i]);
    }
    y0 /= (float) m;
    float rss0 = 0;
    for (int i = 0; i < m; ++i) rss0 += (y[i] - y0) * (y[i] - y0);
    const float span = tMax - tMin;
    for (int i = 0; i < m; ++i) s[i] = t[i];
    detail::median(s, m);
    for (int i = 0; i + 1 < m; ++i) s[i] = s[i + 1] - s[i];
    const float dt = detail::median(s, m - 1);
    if (!(span > 0) || !(dt > 0)) return r;
    const int nh = m >= 24 ? 2 : 1;
    const float fLo = std::max(kFLo, 1.0f / span), fHi = std::min(kFHi, 0.45f / dt);
    if (!(fHi > fLo)) return r;
    float df = 0.1f / span, best = INFINITY, fBest = 0, c[5];
    for (float f = fLo; f < fHi; f += df) {
        const float e = detail::solve(y, t, m, f, nh, c);
        if (e < best) best = e, fBest = f;
    }
    for (int k = 0; k < 2; ++k) { // refine around the grid minimum
        const float f0 = fBest;
        for (int j = -5; j <= 5; ++j) {
            const float f = f0 + (float) j * df * 0.2f;
            const float e = detail::solve(y, t, m, f, nh, c);
            if (e < best) best = e, fBest = f;
        }
        df *= 0.2f;
    }
    if (!(best < 0.5f * rss0)) return r;
    detail::solve(y, t, m, fBest, nh, c);
    if (!(c[1] * c[1] + c[2] * c[2] > 1e-4f * c[0] * c[0])) return r; // < 1 % ripple: no ring
    const int p = 1 + 2 * nh;
    r.L = 1 / (c[0] * ymed);
    r.f = fBest;
    r.amp = std::sqrt(c[1] * c[1] + c[2] * c[2]) / c[0];
    r.periods = span * fBest;
    r.se = std::sqrt(best / (float) (m - p) / (float) m) / c[0];
    r.nh = nh;
    return r;
}

}
