#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

// Coil inductance from a DCM duty sweep of a buck (etc/measure_coil.py is the reference, same algorithm).
// Per point the DCM transfer relation gives 1/L = 2*Vout*fsw*Iout / ((Vin-Vout)*Vin*D^2). The switch
// node rings after the coil current reaches zero (L against the node capacitance) and the ring phase at
// HS turn-on moves Iout by +-10..25 %; the phase is the ring frequency times the dead time
// t = (1 - D*Vin/Vout)/fsw. So 1/L_i is fitted as c0 + sum_h (a_h cos(2 pi h f t_i) + b_h sin(...)) + s*x_i
// + x_i*(a cos + b sin), x_i the peak coil current Ipk_i = (Vin-Vout)*D/(fsw*L) minus its mean (L bias and ring
// decay), f scanned between the span and the sampling limit, L = 1/c0 (over whole ring periods, at the mean Ipk).
// Without enough points or a significant (>= 1 %) periodic term the median of the points is returned.
// L(Ipk): the line c0 + s*x, and bins of equal point count, each spanning >= ~0.8 ring periods, of the
// ring-corrected 1/L_i.
namespace coilfit {

constexpr int kMaxPts = 64;
constexpr int kMinFitPts = 16;
constexpr float kFLo = 80e3f, kFHi = 1.2e6f; // ring frequency search range, Hz
constexpr int kMaxBins = 5, kMinBinPts = 6;

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
    // L(Ipk), needs kMinFitPts points and >= 2 bins
    float L0 = NAN;      // line at Ipk -> 0, H
    float k = NAN, kse = NAN; // dL/dIpk / L0 and its standard error (with the Iout zero and node charge), 1/A
    int nb = 0;          // bins, 0 = none
    float lo[kMaxBins], hi[kMaxBins], Lb[kMaxBins], seb[kMaxBins]; // Ipk range (A), L (H), se relative
};

// Iout read with the converter off before (z0) and after (z1) the sweep: subtracts the zero, interpolated
// over the n points in sweep order; returns its uncertainty (A). A zero that moved by more than kZeroTol marks
// the run as disturbed.
constexpr float kZeroTol = 0.02f, kZeroNoise = 0.002f; // A
inline float zeroCorrect(float *iout, int n, float z0, float z1) {
    for (int i = 0; i < n; ++i) iout[i] -= z0 + (z1 - z0) * (float) i / (float) std::max(1, n - 1);
    return std::max(kZeroNoise, 0.5f * std::fabs(z1 - z0));
}

namespace detail {
// fitted ring term at phase w (x: envelope)
inline float ring(const float *c, float w, float x, int nh) {
    if (!nh) return 0;
    const float cw = std::cos(w), sw = std::sin(w);
    float v = c[1] * cw + c[2] * sw + x * (c[2 * nh + 2] * cw + c[2 * nh + 3] * sw);
    if (nh > 1) v += c[3] * (cw * cw - sw * sw) + c[4] * 2 * sw * cw;
    return v;
}

// least squares y ~ A c, A columns: 1, cos/sin of h*f*t (h <= nh), x; returns the residual sum of squares.
// vx: adds the ring envelope x*cos, x*sin (if nh) and returns (A^T A)^-1 of the x coefficient
inline float solve(const float *y, const float *t, const float *x, int n, float f, int nh, float c[8],
                   float *vx = nullptr) {
    const int sx = 1 + 2 * nh, p = sx + (nh && vx ? 3 : 1), q = p + (vx != nullptr);
    float N[8][10] = {};
    for (int i = 0; i < n; ++i) {
        float a[8];
        a[0] = 1;
        const float w = 6.2831853f * f * t[i];
        const float cw = std::cos(w), sw = std::sin(w);
        a[1] = cw, a[2] = sw;
        if (nh > 1) a[3] = cw * cw - sw * sw, a[4] = 2 * sw * cw;
        a[sx] = x[i];
        if (nh && vx) a[sx + 1] = x[i] * cw, a[sx + 2] = x[i] * sw;
        for (int r = 0; r < p; ++r) {
            for (int k = 0; k < p; ++k) N[r][k] += a[r] * a[k];
            N[r][p] += a[r] * y[i];
        }
    }
    if (vx) N[sx][q] = 1;
    for (int k = 0; k < p; ++k) { // Gauss-Jordan, partial pivoting
        int piv = k;
        for (int r = k + 1; r < p; ++r) if (std::fabs(N[r][k]) > std::fabs(N[piv][k])) piv = r;
        if (std::fabs(N[piv][k]) < 1e-9f) return INFINITY;
        if (piv != k) for (int j = 0; j <= q; ++j) std::swap(N[k][j], N[piv][j]);
        for (int r = 0; r < p; ++r) {
            if (r == k) continue;
            const float m = N[r][k] / N[k][k];
            for (int j = k; j <= q; ++j) N[r][j] -= m * N[k][j];
        }
    }
    for (int k = 0; k < p; ++k) c[k] = N[k][p] / N[k][k];
    if (vx) *vx = N[sx][q] / N[sx][sx];
    else c[sx + 1] = c[sx + 2] = 0;
    float rss = 0;
    for (int i = 0; i < n; ++i) {
        const float e = y[i] - c[0] - ring(c, 6.2831853f * f * t[i], x[i], nh) - c[sx] * x[i];
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

// d: HS duty fraction, vin/vout [V], iout [A] of the accepted DCM points (n <= kMaxPts); fsw [Hz]; zsig: Iout
// zero uncertainty (A, zeroCorrect()), added to the se
inline Result fit(const float *d, const float *vin, const float *vout, const float *iout, int n, float fsw,
                  float zsig = 0) {
    Result r;
    if (n > kMaxPts) n = kMaxPts;
    float y[kMaxPts], t[kMaxPts], s[kMaxPts], x[kMaxPts];
    auto ok = [&](int i) { return (vin[i] - vout[i]) * vin[i] * d[i] * d[i] > 0 && iout[i] > 0 && vout[i] > 0; };
    int m = 0;
    float zi = 0, vm = 0;
    for (int i = 0; i < n; ++i) {
        if (!ok(i)) continue;
        y[m] = 2 * vout[i] * fsw * iout[i] / ((vin[i] - vout[i]) * vin[i] * d[i] * d[i]);
        t[m] = (1 - d[i] * vin[i] / vout[i]) / fsw;
        x[m] = (vin[i] - vout[i]) * d[i] / fsw; // Ipk * L
        s[m] = y[m];
        zi += 1 / iout[i], vm += std::max(vout[i], vin[i] * vin[i] / (2 * vout[i]) - vout[i]);
        ++m;
    }
    r.n = m;
    if (m < 3) return r;
    const float ymed = detail::median(s, m);
    r.Lmed = r.L = 1 / ymed;
    if (m < kMinFitPts) return r;
    float xm = 0, tMin = t[0], tMax = t[0];
    for (int i = 0; i < m; ++i) {
        y[i] /= ymed; // O(1) for float sums
        x[i] *= ymed;
        xm += x[i];
        tMin = std::min(tMin, t[i]), tMax = std::max(tMax, t[i]);
    }
    xm /= (float) m, zi /= (float) m, vm /= (float) m;
    for (int i = 0; i < m; ++i) x[i] -= xm;
    float c[8], vs;
    const float rss0 = detail::solve(y, t, x, m, 0, 0, c);
    const float span = tMax - tMin;
    for (int i = 0; i < m; ++i) s[i] = t[i];
    detail::median(s, m);
    for (int i = 0; i + 1 < m; ++i) s[i] = s[i + 1] - s[i];
    const float dt = detail::median(s, m - 1);
    const int nh = m >= 24 ? 2 : 1;
    const float fLo = std::max(kFLo, 1.0f / span), fHi = std::min(kFHi, 0.45f / dt);
    float best = INFINITY, fBest = 0;
    if (span > 0 && dt > 0 && fHi > fLo) {
        float df = 0.1f / span;
        for (float f = fLo; f < fHi; f += df) {
            const float e = detail::solve(y, t, x, m, f, nh, c);
            if (e < best) best = e, fBest = f;
        }
        for (int k = 0; k < 2; ++k) { // refine around the grid minimum
            const float f0 = fBest;
            for (int j = -5; j <= 5; ++j) {
                const float f = f0 + (float) j * df * 0.2f;
                const float e = detail::solve(y, t, x, m, f, nh, c);
                if (e < best) best = e, fBest = f;
            }
            df *= 0.2f;
        }
    }
    float rss = INFINITY;
    if (best < 0.5f * rss0) {
        rss = detail::solve(y, t, x, m, fBest, nh, c, &vs);
        if (c[1] * c[1] + c[2] * c[2] > 1e-4f * c[0] * c[0]) { // >= 1 % ripple: ring
            r.L = 1 / (c[0] * ymed);
            r.f = fBest;
            r.amp = std::sqrt(c[1] * c[1] + c[2] * c[2]) / c[0];
            r.periods = span * fBest;
            r.nh = nh;
        }
    }
    if (!r.nh) rss = detail::solve(y, t, x, m, 0, 0, c, &vs);
    const int sx = 1 + 2 * r.nh, p = sx + (r.nh ? 3 : 1);
    if (!(rss < INFINITY) || !(c[0] > 0)) return r;
    float xn = 0; // Ipk at L(Ipk) of the line rather than at the mean L, refit
    for (int i = 0; i < m; ++i) x[i] = (x[i] + xm) * (1 + c[sx] * x[i] / c[0]), xn += x[i];
    xm = xn / (float) m;
    for (int i = 0; i < m; ++i) x[i] -= xm;
    rss = detail::solve(y, t, x, m, r.f, r.nh, c, &vs);
    if (!(rss < INFINITY) || !(c[0] > 0)) return r;
    const float sig = std::sqrt(rss / (float) (m - p));
    if (r.nh) r.se = std::hypot(sig / std::sqrt((float) m) / c[0], zsig * zi);
    // L(Ipk). The Iout zero and the net node charge per cycle (ring -C*Vout, HS turn-off +C*Vin^2/(2*Vout)) act
    // as Iout offsets; the larger term bounds the latter.
    for (int i = 0, j = 0; i < n; ++i)
        if (ok(i)) s[j] = y[j] / iout[i], ++j; // dy/dIout
    float cz[8], vz;
    if (!(detail::solve(s, t, x, m, r.f, r.nh, cz, &vz) < INFINITY)) return r;
    const float rho = r.L * ymed, g = c[0] + c[sx] * xm; // rho: Ipk scale, x = rho * Ipk
    const float zt = std::hypot(zsig, r.f > 0 ? vm * fsw / (39.478418f * r.f * r.f * r.L) : 0);
    r.L0 = g / (c[0] * c[0] * ymed);
    r.k = -c[sx] * rho / g;
    r.kse = std::hypot(sig * std::sqrt(vs), cz[sx] * zt) * rho / g;
    for (int i = 0; i < m; ++i) {
        const float u = y[i] - detail::ring(c, 6.2831853f * r.f * t[i], x[i], r.nh), ip = (x[i] + xm) / rho;
        const float z = s[i];
        int j = i - 1; // insertion sort by Ipk
        while (j >= 0 && x[j] > ip) y[j + 1] = y[j], x[j + 1] = x[j], s[j + 1] = s[j], --j;
        y[j + 1] = u, x[j + 1] = ip, s[j + 1] = z;
    }
    int nb = std::min(kMaxBins, m / kMinBinPts);
    if (r.f > 0) nb = std::min(nb, (int) (r.periods * 1.25f));
    if (nb < 2) { // under ~1.6 ring periods the slope is not separable from the ring
        r.L0 = r.k = r.kse = NAN;
        return r;
    }
    for (int b = 0; b < nb; ++b) {
        const int i0 = b * m / nb, i1 = (b + 1) * m / nb;
        float u = 0, z = 0;
        for (int i = i0; i < i1; ++i) u += y[i], z += s[i];
        u /= (float) (i1 - i0), z /= (float) (i1 - i0);
        r.lo[b] = x[i0], r.hi[b] = x[i1 - 1];
        r.Lb[b] = 1 / (u * ymed);
        r.seb[b] = std::hypot(sig / std::sqrt((float) (i1 - i0)), z * zt) / u;
    }
    r.nb = nb;
    return r;
}

}
