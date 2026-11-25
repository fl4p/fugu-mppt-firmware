#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

// Coil inductance from a DCM duty sweep of a buck (etc/measure_coil.py is the reference, same algorithm; a boost
// through asBuck()).
// Per point the DCM transfer relation gives 1/L = 2*Vout*fsw*Iout / ((Vin-Vout)*Vin*D^2). The switch
// node rings after the coil current reaches zero (L against the node capacitance) and the ring phase at
// HS turn-on moves Iout by +-10..25 %; the phase is the ring frequency times the dead time
// t = (1 - D*Vin/Vout)/fsw. So 1/L_i is fitted as c0 + sum_h (a_h cos(2 pi h f t_i) + b_h sin(...)) + s*x_i
// + x_i*(a cos + b sin), x_i the peak coil current Ipk_i = (Vin-Vout)*D/(fsw*L) minus its mean (L bias and ring
// decay), f scanned between the span and the sampling limit, L = 1/c0 (over whole ring periods, at the mean Ipk).
// Without enough points or a significant (>= 1 %) periodic term the median of the points is returned.
// L(Ipk): the slope s, and bins of equal point count, each spanning >= ~0.8 ring periods, of the
// ring-corrected 1/L_i. Errors: noise (residuals, Iout zero read) and systematic (node charge, on-time offset,
// ring-model mismatch, Iout gain). An Iout or on-time offset also tilts L(Ipk): the slope is not the coil's alone.
// On-time offset dl (effective on-time = commanded + dl): every point uses D + dl*fsw. Its estimate drops the slope
// (L constant over Ipk) and fits 1/L_i = c0 (1 + 2 dl fsw/D_i) with the ring terms; dl and the slope are nearly
// collinear over one sweep, so dl carries the slope tolerance kSlopeTol as a systematic error.
namespace coilfit {

constexpr int kMaxPts = 64;
constexpr int kMinFitPts = 16;
constexpr float kFLo = 80e3f, kFHi = 1.2e6f; // ring frequency search range, Hz
constexpr int kMaxBins = 5, kMinBinPts = 6;
constexpr float kOnTimeMin = 50e-9f; // on-time offset bound (s) when the dead time is shorter
constexpr float kGainTol = 0.01f;    // assumed Iout gain tolerance (shunt), relative
constexpr float kSlopeTol = 0.005f;  // coil L(Ipk) slope tolerance of the on-time estimate, 1/A
constexpr float kDlMax = 400e-9f;    // plausible on-time offset (s): beyond it the DCM model does not fit
constexpr float kDlTol = 80e-9f;     // on-time offset error (s, noise and systematic) up to which it is applied

// A boost (vin the LV input, vout the HV output, its input current the coil current) is the buck with input vout and
// output vout - vin at the same control duty: mirroring the switch node (v -> vout - v) keeps the coil's voltage and
// current. Maps a boost point in place, so everything below applies unchanged.
inline void asBuck(float &vin, float &vout) {
    const float vh = vout;
    vout -= vin, vin = vh;
}

// DCM peak coil current from the output average, L-free: Iout = Ipk*D*Vin/(2*Vout) (CCM: >= the real peak)
inline float ipkDcm(float d, float vin, float vout, float iout) { return 2.0f * iout * vout / (d * vin); }

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
    float se = NAN;      // noise error of L (residuals, Iout zero read), relative 1 sigma
    float sys = NAN;     // systematic error of L, relative 1 sigma (bounds taken as uniform)
    int n = 0, nh = 0;
    // L(Ipk), needs kMinFitPts points and >= 2 bins
    float k = NAN, kse = NAN; // apparent dL/dIpk / L(Ipk -> 0) and its noise error, 1/A
    float ksys = NAN;    // slope the Iout and on-time offset bounds can produce, 1/A
    float dl = NAN, dlse = NAN, dlsys = NAN; // on-time offset estimate at this fit's points, its noise and systematic (s)
    float dspan = 0;     // duty span of the points, max/min
    int nb = 0;          // bins, 0 = none
    // Ipk range (A), L (H), noise and offset-bound errors (relative)
    float lo[kMaxBins], hi[kMaxBins], Lb[kMaxBins], seb[kMaxBins], sysb[kMaxBins];
};

// Iout read with the converter off before (z0) and after (z1) the sweep: subtracts the zero, interpolated
// over the sweep rows (row[i] of rows; none: the n points are the rows); returns its uncertainty (A, 1 sigma).
// A zero that moved by more than kZeroTol marks the run as disturbed.
constexpr float kZeroTol = 0.02f, kZeroNoise = 0.002f; // A
inline float zeroCorrect(float *iout, int n, float z0, float z1, const uint8_t *row = nullptr, int rows = 0) {
    if (!row) rows = n;
    for (int i = 0; i < n; ++i) iout[i] -= z0 + (z1 - z0) * (float) (row ? row[i] : i) / (float) std::max(1, rows - 1);
    return std::hypot(kZeroNoise, 0.2887f * std::fabs(z1 - z0)); // read noise, drift bound |z1-z0|/2 as uniform
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
// v: adds the ring envelope xe*cos, xe*sin (if nh; xe defaults to x) and returns the diagonal of (A^T A)^-1 for c0
// and x
inline float solve(const float *y, const float *t, const float *x, int n, float f, int nh, float c[8],
                   float *v = nullptr, const float *xe = nullptr) {
    if (!xe) xe = x;
    const int sx = 1 + 2 * nh, p = sx + (nh && v ? 3 : 1), q = p + (v ? 2 : 0);
    float N[8][11] = {};
    for (int i = 0; i < n; ++i) {
        float a[8];
        a[0] = 1;
        const float w = 6.2831853f * f * t[i];
        const float cw = std::cos(w), sw = std::sin(w);
        a[1] = cw, a[2] = sw;
        if (nh > 1) a[3] = cw * cw - sw * sw, a[4] = 2 * sw * cw;
        a[sx] = x[i];
        if (nh && v) a[sx + 1] = xe[i] * cw, a[sx + 2] = xe[i] * sw;
        for (int r = 0; r < p; ++r) {
            for (int k = 0; k < p; ++k) N[r][k] += a[r] * a[k];
            N[r][p] += a[r] * y[i];
        }
    }
    if (v) N[0][p + 1] = N[sx][p + 2] = 1;
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
    if (v) v[0] = N[0][p + 1] / N[0][0], v[1] = N[sx][p + 2] / N[sx][sx];
    else c[sx + 1] = c[sx + 2] = 0;
    float rss = 0;
    for (int i = 0; i < n; ++i) {
        const float e = y[i] - c[0] - ring(c, 6.2831853f * f * t[i], xe[i], nh) - c[sx] * x[i];
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
// zero uncertainty (A, zeroCorrect()); ton: bound of the HS on-time offset (s), e.g. the dead time; dl: on-time
// offset applied to every point (s); fr >= 0: ring frequency (Hz, 0 = none) instead of the scan
inline Result fit(const float *d, const float *vin, const float *vout, const float *iout, int n, float fsw,
                  float zsig = 0, float ton = kOnTimeMin, float dl = 0, float fr = -1) {
    Result r;
    if (n > kMaxPts) n = kMaxPts;
    float y[kMaxPts], t[kMaxPts], s[kMaxPts], x[kMaxPts], w[kMaxPts], u[kMaxPts];
    auto de = [&](int i) { return d[i] + dl * fsw; };
    auto ok = [&](int i) { return (vin[i] - vout[i]) * vin[i] > 0 && de(i) > 0 && iout[i] > 0 && vout[i] > 0; };
    int m = 0;
    float vm = 0, dlo = 1, dhi = 0;
    for (int i = 0; i < n; ++i) {
        if (!ok(i)) continue;
        const float D = de(i);
        y[m] = 2 * vout[i] * fsw * iout[i] / ((vin[i] - vout[i]) * vin[i] * D * D);
        t[m] = (1 - D * vin[i] / vout[i]) / fsw;
        x[m] = (vin[i] - vout[i]) * D / fsw; // Ipk * L
        s[m] = y[m];
        vm += std::max(vout[i], vin[i] * vin[i] / (2 * vout[i]) - vout[i]);
        dlo = std::min(dlo, D), dhi = std::max(dhi, D);
        ++m;
    }
    r.n = m;
    r.dspan = dhi / dlo;
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
    xm /= (float) m, vm /= (float) m;
    for (int i = 0; i < m; ++i) x[i] -= xm;
    float c[8], v[2];
    const float rss0 = detail::solve(y, t, x, m, 0, 0, c);
    const float span = tMax - tMin;
    for (int i = 0; i < m; ++i) s[i] = t[i];
    detail::median(s, m);
    for (int i = 0; i + 1 < m; ++i) s[i] = s[i + 1] - s[i];
    const float dt = detail::median(s, m - 1);
    const int nh = m >= 24 ? 2 : 1;
    const float fLo = std::max(kFLo, 1.0f / span), fHi = std::min(kFHi, 0.45f / dt);
    float best = INFINITY, fBest = fr;
    if (fr >= 0) best = fr > 0 ? 0 : INFINITY;
    else if (span > 0 && dt > 0 && fHi > fLo) {
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
        rss = detail::solve(y, t, x, m, fBest, nh, c, v);
        if (fr > 0 || c[1] * c[1] + c[2] * c[2] > 1e-4f * c[0] * c[0]) { // >= 1 % ripple: ring
            r.L = 1 / (c[0] * ymed);
            r.f = fBest;
            r.amp = std::sqrt(c[1] * c[1] + c[2] * c[2]) / c[0];
            r.periods = span * fBest;
            r.nh = nh;
        }
    }
    if (!r.nh) rss = detail::solve(y, t, x, m, 0, 0, c, v);
    const int sx = 1 + 2 * r.nh, p = sx + (r.nh ? 3 : 1);
    if (!(rss < INFINITY) || !(c[0] > 0)) return r;
    float xn = 0; // Ipk at L(Ipk) of the line rather than at the mean L, refit
    for (int i = 0; i < m; ++i) x[i] = (x[i] + xm) * (1 + c[sx] * x[i] / c[0]), xn += x[i];
    xm = xn / (float) m;
    for (int i = 0; i < m; ++i) x[i] -= xm;
    rss = detail::solve(y, t, x, m, r.f, r.nh, c, v);
    if (!(rss < INFINITY) || !(c[0] > 0)) return r;
    const float sig = std::sqrt(rss / (float) (m - p));
    // Offsets as responses of the fit: s = dy/dIout, w = dy/dton = 1e6 u y. Bounds: the zero read zsig (noise), the net
    // node charge per cycle (ring -C*Vout, HS turn-off +C*Vin^2/(2*Vout); the larger term) and ton (systematic).
    for (int i = 0, j = 0; i < n; ++i)
        if (ok(i)) s[j] = y[j] / iout[i], u[j] = 2e-6f * fsw / de(i), w[j] = 1e6f * u[j] * y[j], ++j;
    float cz[8], cd[8], vz[2], q = 0, e0 = 0;
    if (!(detail::solve(s, t, x, m, r.f, r.nh, cz, vz) < INFINITY) ||
        !(detail::solve(w, t, x, m, r.f, r.nh, cd, vz) < INFINITY))
        return r;
    for (int i = 0; i < m; ++i) { // residuals correlated in sweep order: ring-model mismatch
        const float e = y[i] - c[0] - detail::ring(c, 6.2831853f * r.f * t[i], x[i], r.nh) - c[sx] * x[i];
        if (i) q += e * e0;
        e0 = e;
    }
    const float a = std::min(0.9f, std::max(0.f, q / rss)), infl = std::sqrt((1 + a) / (1 - a));
    const float zn = r.f > 0 ? vm * fsw / (39.478418f * r.f * r.f * r.L) : 0;
    const float sn = sig * std::sqrt(v[0]) / c[0];
    const float rho = r.L * ymed; // Ipk scale, x = rho * Ipk
    { // on-time offset: 1/L_i = c0 + c0 dl 2 fsw/D_i (u, dl in us) + ring, no slope; responses to the Iout zero and to
        // the slope (w = y Ipk)
        float cu[8], cs[8], ck[8], vu[2];
        for (int i = 0; i < m; ++i) w[i] = y[i] * (x[i] + xm) / rho;
        const float e = detail::solve(y, t, u, m, r.f, r.nh, cu, vu, x);
        if (e < INFINITY && cu[0] > 0 && detail::solve(s, t, u, m, r.f, r.nh, cs, vz, x) < INFINITY &&
            detail::solve(w, t, u, m, r.f, r.nh, ck, vz, x) < INFINITY) {
            const float g = 1e-6f / cu[0];
            r.dl = dl + cu[sx] * g;
            r.dlse = std::hypot(std::sqrt(e / (float) (m - p) * vu[1]) * infl, cs[sx] * zsig) * g;
            const float bz = cs[sx] * zn, bk = ck[sx] * kSlopeTol; // bounds, uniform
            r.dlsys = std::sqrt((bz * bz + bk * bk) / 3) * std::fabs(g);
        }
    }
    if (r.nh) {
        r.se = std::hypot(sn, cz[0] / c[0] * zsig);
        const float bz = cz[0] / c[0] * zn, bd = cd[0] / c[0] * ton; // bounds, independent and uniform
        r.sys = std::sqrt((bz * bz + bd * bd + kGainTol * kGainTol) / 3 + sn * sn * (infl * infl - 1));
    }
    // L(Ipk)
    const float g = c[0] + c[sx] * xm;
    r.k = -c[sx] * rho / g;
    r.kse = std::hypot(sig * std::sqrt(v[1]) * infl, cz[sx] * zsig) * rho / g;
    r.ksys = (std::fabs(cz[sx]) * zn + std::fabs(cd[sx]) * ton) * rho / g;
    for (int i = 0; i < m; ++i) {
        const float yr = y[i] - detail::ring(c, 6.2831853f * r.f * t[i], x[i], r.nh), ip = (x[i] + xm) / rho;
        const float z = s[i], zd = 1e6f * u[i] * y[i];
        int j = i - 1; // insertion sort by Ipk
        while (j >= 0 && x[j] > ip) y[j + 1] = y[j], x[j + 1] = x[j], s[j + 1] = s[j], u[j + 1] = u[j], --j;
        y[j + 1] = yr, x[j + 1] = ip, s[j + 1] = z, u[j + 1] = zd;
    }
    int nb = std::min(kMaxBins, m / kMinBinPts);
    if (r.f > 0) nb = std::min(nb, (int) (r.periods * 1.25f));
    if (nb < 2) { // under ~1.6 ring periods the slope is not separable from the ring
        r.k = r.kse = r.ksys = NAN;
        return r;
    }
    for (int b = 0; b < nb; ++b) {
        const int i0 = b * m / nb, i1 = (b + 1) * m / nb;
        float yr = 0, z = 0, zd = 0;
        for (int i = i0; i < i1; ++i) yr += y[i], z += s[i], zd += u[i];
        yr /= (float) (i1 - i0), z /= (float) (i1 - i0), zd /= (float) (i1 - i0);
        r.lo[b] = x[i0], r.hi[b] = x[i1 - 1];
        r.Lb[b] = 1 / (yr * ymed);
        r.seb[b] = std::hypot(sig / std::sqrt((float) (i1 - i0)), z * zsig) / yr;
        r.sysb[b] = (z * zn + zd * ton) / yr;
    }
    r.nb = nb;
    return r;
}

// on-time offset of r: 0 applicable; 1 not estimated (too few points), 2 duty span under 2x, 3 error (noise and
// systematic) over kDlTol, 4 out of range (beyond kDlMax by 2 sigma)
inline int dlStatus(const Result &r) {
    const float u = std::hypot(r.dlse, r.dlsys);
    if (!(u == u) || !(r.dl == r.dl)) return 1;
    if (std::fabs(r.dl) > kDlMax + 2 * u) return 4;
    if (r.dspan < 2) return 2;
    return u > kDlTol ? 3 : 0;
}

// fit() with the on-time offset applied, iterated on its own estimate from r0 (the fit at dl 0) at r0's ring frequency;
// st: dlStatus(). The on-time bound of the stated errors becomes the estimate's uncertainty.
inline Result fitOnTime(const float *d, const float *vin, const float *vout, const float *iout, int n, float fsw,
                        float zsig, const Result &r0, int &st) {
    // under ~1.6 periods a ring is not separable from a smooth trend (as for L(Ipk)): the offset fit drops it
    const float fr = r0.periods >= 1.6f ? r0.f : 0;
    Result r = r0;
    if (r0.nh && !fr) r = fit(d, vin, vout, iout, n, fsw, zsig, kOnTimeMin, 0, 0);
    for (int i = 0; i < 4 && dlStatus(r) != 1 && std::fabs(r.dl) < 3 * kDlMax; ++i) {
        const float dl = r.dl;
        r = fit(d, vin, vout, iout, n, fsw, zsig, 1.7320508f * std::hypot(r.dlse, r.dlsys), dl, fr);
        if (std::fabs(r.dl - dl) < 1e-9f) break;
    }
    st = dlStatus(r);
    return r;
}

}
