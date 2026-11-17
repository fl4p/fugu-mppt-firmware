#pragma once

#include <cmath>

// Solve the augmented 3x3 system M in place (Gauss-Jordan); false if singular.
inline bool solve3(double M[3][4]) {
    for (int i = 0; i < 3; ++i) {
        double p = M[i][i];
        if (std::fabs(p) < 1e-12) return false;
        for (int j = 0; j < 4; ++j) M[i][j] /= p;
        for (int k = 0; k < 3; ++k)
            if (k != i) {
                double f = M[k][i];
                for (int j = 0; j < 4; ++j) M[k][j] -= f * M[i][j];
            }
    }
    return true;
}

// Least-squares fit of two half-parabolas sharing an apex (x0, y0): y = y0 - a*(x0-x)^2 left of x0,
// y = y0 - b*(x-x0)^2 right of it, a, b >= 0. x0 is grid-searched over the interior of the
// ascending xs. x0Out/bOut are written only on success.
inline bool asymPeakFit(const float *xs, const float *ys, int n, float &x0Out, double &bOut) {
    if (n < 5) return false;
    double syy = 0, r0 = 0, bestSse = INFINITY;
    for (int i = 0; i < n; ++i) { syy += (double) ys[i] * ys[i]; r0 += ys[i]; }
    constexpr int GRID = 400;
    for (int k = 0; k <= GRID; ++k) {
        double x0 = xs[1] + (double) (xs[n - 2] - xs[1]) * k / GRID;
        double su = 0, sv = 0, suu = 0, svv = 0, r1 = 0, r2 = 0;
        int nl = 0, nr = 0;
        for (int i = 0; i < n; ++i) {
            double d = xs[i] - x0, q = d * d;
            if (d < 0) { su += q; suu += q * q; r1 -= q * ys[i]; ++nl; }
            else { sv += q; svv += q * q; r2 -= q * ys[i]; ++nr; }
        }
        if (nl < 2 || nr < 2) continue;
        double M[3][4] = {{(double) n, -su, -sv, r0}, {-su, suu, 0, r1}, {-sv, 0, svv, r2}};
        if (!solve3(M) || M[1][3] < 0 || M[2][3] < 0) continue;
        double sse = syy - (M[0][3] * r0 + M[1][3] * r1 + M[2][3] * r2);
        if (sse < bestSse) { bestSse = sse; x0Out = (float) x0; bOut = M[2][3]; }
    }
    return bestSse < INFINITY;
}

// Least-squares knee over ascending xs: y = c0 + c1*d + c2*min(0,d)^2 + c3*max(0,d)^2, d = (x - x0) / span,
// c2 <= 0 (left side straight or bending down, as the body-diode loss does), c3 < c2 (the curve bends down
// at x0). x0 is grid-searched with >= 3 points left and >= 2 right of it. The line takes any baseline
// slope, so x0 is the onset of the bend. up: c3 > max(c2, 0) instead, c2 free (the curve bends up at x0, from
// a left side that may already curve either way). Out: x0, drop = (c2 - c3), up: (c3 - max(c2, 0)), times
// d(xs[n-1])^2 (the added bend at the last point, not the left side's own curvature), rms residual. Written
// only on success.
inline bool kneeFit(const float *xs, const float *ys, int n, float &x0Out, float &dropOut, float &rmsOut,
                    bool up = false) {
    if (n < 6) return false;
    const double span = (double) xs[n - 1] - xs[0];
    if (!(span > 0)) return false;
    double best = INFINITY;
    constexpr int GRID = 400;
    for (int g = 0; g <= GRID; ++g) {
        const double x0 = xs[2] + ((double) xs[n - 2] - xs[2]) * g / GRID;
        int nr = 0;
        for (int i = 0; i < n; ++i) nr += xs[i] > x0;
        if (nr < 2 || n - nr < 3) continue;
        double c[4] = {};
        for (int m = 4; m >= 3; --m) { // m = 3: c2 = 0 (constraint active)
            double A[4][5] = {};
            for (int i = 0; i < n; ++i) {
                const double d = (xs[i] - x0) / span, f[4] = {1, d, d < 0 ? d * d : 0, d > 0 ? d * d : 0};
                const double fm[4] = {f[0], f[1], m == 4 ? f[2] : f[3], f[3]};
                for (int j = 0; j < m; ++j) {
                    for (int k = 0; k < m; ++k) A[j][k] += fm[j] * fm[k];
                    A[j][m] += fm[j] * ys[i];
                }
            }
            bool ok = true;
            for (int i = 0; i < m && ok; ++i) { // Gauss-Jordan
                const double p = A[i][i];
                if (std::fabs(p) < 1e-12) { ok = false; break; }
                for (int j = 0; j <= m; ++j) A[i][j] /= p;
                for (int k = 0; k < m; ++k)
                    if (k != i) {
                        const double f = A[k][i];
                        for (int j = 0; j <= m; ++j) A[k][j] -= f * A[i][j];
                    }
            }
            if (!ok) break;
            if (m == 4) { c[0] = A[0][4]; c[1] = A[1][4]; c[2] = A[2][4]; c[3] = A[3][4]; }
            else { c[0] = A[0][3]; c[1] = A[1][3]; c[2] = 0; c[3] = A[2][3]; }
            if (c[2] <= 0 || up) break;
        }
        if (!(up ? c[3] > std::max(c[2], 0.0) : c[2] <= 0 && c[3] < c[2])) continue;
        double sse = 0;
        for (int i = 0; i < n; ++i) {
            const double d = (xs[i] - x0) / span;
            const double e = ys[i] - (c[0] + c[1] * d + (d < 0 ? c[2] : c[3]) * d * d);
            sse += e * e;
        }
        if (sse < best) {
            best = sse;
            const double dl = (xs[n - 1] - x0) / span;
            x0Out = (float) x0;
            dropOut = (float) ((up ? c[3] - std::max(c[2], 0.0) : c[2] - c[3]) * dl * dl);
            rmsOut = (float) std::sqrt(sse / (n - 4));
        }
    }
    return best < INFINITY;
}
