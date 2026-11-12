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
