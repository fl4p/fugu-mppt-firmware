// vconv buck plant (src/sim/vconv.cpp) with the fry/flat board parameters (plant_boards.h) against the LTspice
// half-bridge bench numbers of doc/research/vconv-vs-spice.md: dawn Pout(HS) ripple, falling-step share and
// efficiency, DCM near Vin ~ Vout, the ~100 W DCM/CCM staircase and CCM efficiency. Prints the tables and
// exits 1 when a check is out of its band. The reference rows below are doc/research/vconv-spice-data/*.csv.
//   test/host-stub/vconv-spice-check.sh [-v]        SIM_PLANT="fry:t_sw=30e-9" overrides board fields
#include "sim/vconv.h"
#include "plant_boards.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {
constexpr int kPmax = 4103;
constexpr uint32_t kFreq = 38996; // 160 MHz / 4103 (fry/flat MCPWM)
constexpr float kTick = 1.f / 160e6f, kT = kPmax * kTick;
bool g_verbose = false;
int g_fail = 0;

struct Pt {
    double pin, pout;
};

// the bench's coil L with the board's ring frequency (b = nullptr: ideal switches, no ring)
void setup(VirtualConverter &p, const PlantBoard *b, float L) {
    p.setPv(1e-6f, 500.f, 0.8f);
    p.setPvBlocking(true);
    p.setPassives(10.f, 470e-6f, L); // stiff Vin (bench source): the plant moves Vin within the cycle
    if (b) {
        p.setRing(b->ringC() * b->lPlant / L, b->ringQ);
        p.setRingNode(b->ringV, b->ringCexp, b->ringCt);
        p.setDiodeVf(b->vf);
        p.setSwitching(b->tDead, b->tHsExt, b->rHs, b->rLs, b->tSw);
    }
    p.setPwmMax(kPmax), p.setPwmFreq(kFreq);
}

// stiff Vin and Vout, 400 cycles, mean of the second half (as the bench)
Pt runStiff(const PlantBoard *b, float L, float vin, float vout, int hs, int ls) {
    VirtualConverter p;
    setup(p, b, L);
    p.setBat(vout, 1e-6f);
    p.setPwmCount(0, hs), p.setPwmCount(1, ls);
    double si = 0, so = 0;
    for (int k = 0; k < 400; ++k) {
        p.setVin(vin), p.setVout(vout);
        p.stepSeconds(1.f / kFreq, kFreq);
        if (k >= 200) si += p.getIinAvg(), so += p.getIoutAvg();
    }
    return {si / 200 * vin, so / 200 * vout};
}

float benchL(const char *board) { return !strcmp(board, "fry") ? 80e-6f : 51e-6f; }

int fwLs(float vin, float vout, int hs) {
    const float m = std::min((vout / vin) / (0.99f / 1.01f), 0.99f);
    return std::min(std::max(320, (int) std::lround(hs * (1 / m - 1))), kPmax - hs - 1);
}

int hsForP(float L, float vin, float vout, float p) {
    const float io = p / vout;
    return (int) (std::sqrt(io * 2 * L * kT * vout / ((vin - vout) * vin)) * 160e6f);
}

double pct(std::vector<double> v, double q) {
    std::sort(v.begin(), v.end());
    const double k = q * (double) (v.size() - 1);
    const size_t i = (size_t) k;
    return i + 1 < v.size() ? v[i] + (v[i + 1] - v[i]) * (k - (double) i) : v[i];
}

// least-squares polynomial of degree 4 in scaled x, evaluated at x
std::vector<double> polySmooth(const std::vector<double> &x, const std::vector<double> &y) {
    constexpr int D = 5;
    double mx = 0, sx = 0;
    for (double v: x) mx += v;
    mx /= (double) x.size();
    for (double v: x) sx += (v - mx) * (v - mx);
    sx = std::sqrt(sx / (double) x.size());
    long double A[D][D + 1] = {};
    for (size_t n = 0; n < x.size(); ++n) {
        long double u[D], t = 1;
        for (int k = 0; k < D; ++k) u[k] = t, t *= (x[n] - mx) / sx;
        for (int r = 0; r < D; ++r) {
            for (int c = 0; c < D; ++c) A[r][c] += u[r] * u[c];
            A[r][D] += u[r] * y[n];
        }
    }
    for (int c = 0; c < D; ++c) {
        int pv = c;
        for (int r = c + 1; r < D; ++r) if (std::fabs((double) A[r][c]) > std::fabs((double) A[pv][c])) pv = r;
        for (int k = 0; k <= D; ++k) std::swap(A[c][k], A[pv][k]);
        for (int r = 0; r < D; ++r) {
            if (r == c) continue;
            const long double f = A[r][c] / A[c][c];
            for (int k = c; k <= D; ++k) A[r][k] -= f * A[c][k];
        }
    }
    std::vector<double> out;
    for (double v: x) {
        long double s = 0, t = 1;
        for (int k = 0; k < D; ++k) s += A[k][D] / A[k][k] * t, t *= (v - mx) / sx;
        out.push_back((double) s);
    }
    return out;
}

struct DawnStats {
    double amp = NAN, period = NAN, neg = NAN, eta13 = NAN, eta830 = NAN;
};

DawnStats dawnSet(const PlantBoard *b, const char *board, float vin) {
    constexpr float vout = 26.5f;
    const float Lg = benchL(board);
    const float iob = vout * (1 - vout / vin) / (2 * Lg / kT), pcap = std::min(30.f, 0.9f * iob * vout);
    std::vector<double> h, p, e13, e830;
    for (int hs = hsForP(Lg, vin, vout, 0.5f); hs <= hsForP(Lg, vin, vout, pcap); hs += 45) {
        const Pt r = runStiff(b, Lg, vin, vout, hs, fwLs(vin, vout, hs));
        if (r.pout > 0.8 && r.pout < 3) e13.push_back(r.pout / r.pin);
        if (r.pout > 8 && r.pout < 30) e830.push_back(r.pout / r.pin);
        if (r.pout > 1) h.push_back(hs), p.push_back(r.pout);
    }
    DawnStats s;
    auto mean = [](const std::vector<double> &v) {
        double a = 0;
        for (double x: v) a += x;
        return v.empty() ? NAN : a / (double) v.size();
    };
    s.eta13 = mean(e13), s.eta830 = mean(e830);
    if (h.size() < 8) return s;
    std::vector<double> lh, lp, r;
    for (size_t i = 0; i < h.size(); ++i) lh.push_back(std::log(h[i])), lp.push_back(std::log(p[i]));
    const auto sm = polySmooth(lh, lp);
    for (size_t i = 0; i < h.size(); ++i) r.push_back(p[i] / std::exp(sm[i]) - 1);
    std::vector<double> zc;
    for (size_t i = 0; i + 1 < r.size(); ++i) if ((r[i] > 0) != (r[i + 1] > 0) && r[i] != 0) zc.push_back(h[i]);
    if (zc.size() >= 3) s.period = 2 * (zc.back() - zc.front()) / (double) (zc.size() - 1);
    s.amp = 0.5 * (pct(r, 0.95) - pct(r, 0.05));
    int neg = 0;
    for (size_t i = 0; i + 1 < p.size(); ++i) neg += p[i + 1] < p[i];
    s.neg = (double) neg / (double) (p.size() - 1);
    return s;
}

// SPICE reference (vconv-spice-data/dawn.csv): B = bench, Brp = bench with the board's Q
struct DawnRef {
    const char *board;
    float vin, ampB, perB, negB, eta13B, eta830B, ampRp, negRp, eta13Rp, eta830Rp;
};
const DawnRef kDawnRef[] = {
    {"fry", 30, .196f, 900, .51f, .940f, NAN, NAN, NAN, NAN, NAN},
    {"fry", 35, .208f, 645, .28f, .910f, .998f, .174f, .17f, .900f, .993f},
    {"fry", 45, .286f, 465, .22f, .779f, .997f, .223f, .03f, .838f, .989f},
    {"fry", 55, .338f, 345, .04f, .871f, .993f, NAN, NAN, NAN, NAN},
    {"flat", 30, .141f, 756, .36f, .955f, .998f, NAN, NAN, NAN, NAN},
    {"flat", 35, .165f, 472, .16f, .974f, .998f, NAN, NAN, NAN, NAN},
    {"flat", 45, .256f, 330, .23f, .866f, .997f, .103f, .00f, .870f, .983f},
    {"flat", 55, .287f, 244, .25f, .969f, .993f, NAN, NAN, NAN, NAN},
};

float lo(float a, float b) { return std::isnan(b) ? a : std::min(a, b); }
float hi(float a, float b) { return std::isnan(b) ? a : std::max(a, b); }

void check(bool ok, const char *what) {
    if (!ok) ++g_fail, printf("FAIL: %s\n", what);
}

// field Q: ripple 0.80..0.98 x the bench with that damping (Brp); where Brp was not run, between half its estimate
// and B. -hq: 0.7..1.05 x B
void dawn(bool hq) {
    printf("\n## dawn, %s (stiff Vin, Vout 26.5 V, firmware DCM LS rule, 45-count HS steps, Pout > 1 W)\n",
           hq ? "high-Q plant (-hq)" : "field-Q plant");
    printf("set      | ripple amp: ideal  plant  SPICE B  Brp | period: plant  B | falling steps: ideal plant  B   Brp |"
           " eta 1-3 W: ideal plant  B     Brp  | eta 8-30 W: plant  B     Brp\n");
    double negSum = 0, e13Err = 0;
    int nNeg = 0, nNegPos = 0, nE = 0;
    for (auto &r: kDawnRef) {
        const PlantBoard b = hq ? highQ(*plantBoard(r.board)) : *plantBoard(r.board);
        const DawnStats s = dawnSet(&b, r.board, r.vin), i = dawnSet(nullptr, r.board, r.vin);
        printf("%-4s-%-3.0f | %17.3f %6.3f %8.3f %5.3f | %13.0f %4.0f | %20.2f %5.2f %4.2f %4.2f | %16.3f %6.3f %5.3f %5.3f |"
               " %17.4f %5.3f %5.3f\n", r.board, r.vin, i.amp, s.amp, r.ampB, r.ampRp, s.period, r.perB, i.neg, s.neg, r.negB,
               r.negRp, i.eta13, s.eta13, r.eta13B, r.eta13Rp, s.eta830, r.eta830B, r.eta830Rp);
        // Brp estimate where it was not run: its ratio to B at 45 V on the same board
        float rp = r.ampRp;
        for (auto &q: kDawnRef) if (std::isnan(rp) && !strcmp(q.board, r.board) && q.vin == 45) rp = r.ampB * q.ampRp / q.ampB;
        const float aLo = hq ? 0.7f * r.ampB : std::isnan(r.ampRp) ? 0.5f * rp : 0.80f * rp;
        const float aHi = hq ? 1.05f * r.ampB : std::isnan(r.ampRp) ? r.ampB : 0.98f * rp;
        char m[96];
        snprintf(m, sizeof m, "%s-%.0f ripple amp %.3f outside %.3f..%.3f", r.board, r.vin, s.amp, aLo, aHi);
        check(s.amp >= aLo && s.amp <= aHi, m);
        if (!std::isnan(s.eta830) && !std::isnan(r.eta830B)) {
            // without a damped bench run: the damping costs 0.5-1.4 pp at 35-45 V
            const float eLo = std::isnan(r.eta830Rp) ? r.eta830B - 0.010f : lo(r.eta830B, r.eta830Rp) - 0.006f;
            snprintf(m, sizeof m, "%s-%.0f eta 8-30 W %.4f outside %.4f..%.4f", r.board, r.vin, s.eta830, eLo,
                     hi(r.eta830B, r.eta830Rp) + 0.003f);
            check(s.eta830 >= eLo && s.eta830 <= hi(r.eta830B, r.eta830Rp) + 0.003f, m);
        }
        negSum += s.neg, ++nNeg, nNegPos += s.neg > 0;
        if (!std::isnan(s.eta13)) e13Err += std::fabs(s.eta13 - r.eta13B), ++nE;
    }
    printf("falling-step share: plant mean %.2f (%d of %d sets > 0), SPICE B mean 0.26, Brp 0.00-0.17; eta 1-3 W mean"
           " |plant - B| %.3f\n", negSum / nNeg, nNegPos, nNeg, e13Err / nE);
    check(negSum / nNeg >= (hq ? 0.10 : 0.05) && negSum / nNeg <= 0.45 && nNegPos >= 3, "falling-step share");
    check(e13Err / nE <= 0.045, "eta 1-3 W");
}

// DCM near Vin ~ Vout, LS near its minimum pulse: SPICE B Pout (bench L 80 / 51 uH, Vout 26.5 V), near_boundary.csv
struct NearRef {
    const char *board;
    int hs, ls;
    float pout;
};
const NearRef kNearRef[] = {
    {"fry", 2732, 320, 5.625f}, {"fry", 2867, 320, 5.516f}, {"fry", 3002, 329, 5.400f}, {"fry", 3137, 344, 5.327f},
    {"fry", 3182, 349, 5.314f}, {"fry", 3272, 359, 5.258f}, {"fry", 3407, 374, 5.075f},
    {"flat", 2724, 320, 11.751f}, {"flat", 2859, 320, 11.827f}, {"flat", 2994, 328, 11.653f},
    {"flat", 3129, 343, 11.459f}, {"flat", 3264, 358, 11.251f}, {"flat", 3399, 373, 11.052f},
};

void nearBoundary() {
    printf("\n## DCM near Vin ~ Vout (Vin 30 V, Vout 26.5 V)\n");
    for (const char *bn: {"fry", "flat"}) {
        double rs = 0, ri = 0;
        int n = 0;
        printf("%-4s HS:LS  ideal  plant  SPICE W\n", bn);
        for (auto &r: kNearRef) {
            if (strcmp(r.board, bn)) continue;
            const float Lg = benchL(bn);
            const Pt s = runStiff(plantBoard(bn), Lg, 30, 26.5f, r.hs, r.ls), i = runStiff(nullptr, Lg, 30, 26.5f, r.hs, r.ls);
            if (g_verbose || r.hs == 3182 || r.hs == 3129) printf("     %d:%d %6.2f %6.2f %6.2f\n", r.hs, r.ls, i.pout, s.pout, r.pout);
            rs += s.pout / r.pout, ri += i.pout / r.pout, ++n;
        }
        printf("%-4s mean Pout vs SPICE: ideal %+.0f %%, plant %+.0f %%\n", bn, 100 * (ri / n - 1), 100 * (rs / n - 1));
        char m[64];
        snprintf(m, sizeof m, "%s near-boundary Pout %+.0f %%", bn, 100 * (rs / n - 1));
        check(std::fabs(rs / n - 1) <= 0.15, m);
    }
}

// ~100 W at the DCM/CCM boundary, firmware DCM LS rule (SPICE B, undamped), p2.csv
struct P2Ref {
    const char *board;
    float vin;
    int hs, ls;
    float pout;
};
const P2Ref kP2Ref[] = {
    {"flat", 60, 996, 1214, 31.22f}, {"flat", 60, 1059, 1291, 31.51f}, {"flat", 60, 1122, 1368, 33.48f},
    {"flat", 60, 1184, 1444, 46.18f}, {"flat", 60, 1247, 1520, 46.65f}, {"flat", 60, 1310, 1597, 47.08f},
    {"flat", 60, 1373, 1674, 62.38f}, {"flat", 60, 1435, 1750, 64.71f}, {"flat", 60, 1498, 1827, 65.17f},
    {"flat", 60, 1561, 1903, 66.27f}, {"flat", 60, 1623, 1979, 85.60f}, {"flat", 60, 1686, 2056, 86.48f},
    {"flat", 60, 1749, 2133, 87.00f}, {"flat", 60, 1812, 2209, 97.63f},
    {"fry", 75, 797, 1414, 23.84f}, {"fry", 75, 847, 1503, 24.32f}, {"fry", 75, 897, 1591, 25.02f},
    {"fry", 75, 947, 1680, 28.72f}, {"fry", 75, 998, 1771, 39.14f}, {"fry", 75, 1048, 1859, 40.07f},
    {"fry", 75, 1098, 1948, 40.68f}, {"fry", 75, 1148, 2037, 41.59f}, {"fry", 75, 1198, 2125, 48.93f},
    {"fry", 75, 1248, 2214, 59.34f}, {"fry", 75, 1299, 2305, 60.48f}, {"fry", 75, 1349, 2393, 61.23f},
    {"fry", 75, 1399, 2482, 62.33f},
    // not {"fry", 75, 1449, 2571, 75.56f}: D Vin = Vout within 0.05 %, a stiff-source CCM runaway in the plant
};

void p2() {
    printf("\n## ~100 W DCM/CCM boundary staircase (flat 60 V, fry 75 V -> 26.5 V)\n");
    for (const char *bn: {"flat", "fry"}) {
        double es = 0, es2 = 0, ei = 0, ei2 = 0, ets = 0;
        int n = 0;
        for (auto &r: kP2Ref) {
            if (strcmp(r.board, bn)) continue;
            const float Lg = benchL(bn);
            const Pt s = runStiff(plantBoard(bn), Lg, r.vin, 26.5f, r.hs, r.ls), i = runStiff(nullptr, Lg, r.vin, 26.5f, r.hs, r.ls);
            const double e = s.pout / r.pout - 1, f = i.pout / r.pout - 1;
            if (g_verbose) printf("  %s %d:%d ideal %.1f plant %.1f SPICE %.1f W\n", bn, r.hs, r.ls, i.pout, s.pout, r.pout);
            es += e, es2 += e * e, ei += f, ei2 += f * f, ets += s.pout / s.pin, ++n;
        }
        printf("%-4s Pout vs SPICE mean / rms: ideal %+.1f / %.1f %%, plant %+.1f / %.1f %%; plant eta %.2f %%\n", bn,
               100 * ei / n, 100 * std::sqrt(ei2 / n), 100 * es / n, 100 * std::sqrt(es2 / n), 100 * ets / n);
        char m[64];
        snprintf(m, sizeof m, "%s P2 rms %.1f %%", bn, 100 * std::sqrt(es2 / n));
        check(std::fabs(es / n) <= 0.03 && std::sqrt(es2 / n) <= 0.05, m);
    }
}

// CCM, synchronous (LS = pmax - HS - 1), soft battery so the current settles: SPICE B efficiency incl. coil DCR and
// the avalanche loss where ccm.csv lists it (W at ~Pout); the plant has no avalanche term
struct CcmRef {
    const char *board;
    float vin, iout, eta, pAval, pout;
};
const CcmRef kCcmRef[] = {
    {"flat", 60, 3.8f, .9894f, .36f, 101}, {"flat", 60, 11.3f, .9888f, .83f, 299}, {"flat", 60, 22.6f, .9838f, 1.74f, 599},
    {"flat", 30, 3.0f, .9952f, 0, 80}, {"flat", 30, 11.3f, .9931f, .27f, 299}, {"fry", 75, 3.8f, .9875f, .37f, 101},
    {"fry", 60, 11.3f, .9911f, .36f, 299}, {"fry", 60, 22.6f, .9874f, .77f, 599}, {"fry", 30, 3.0f, .9954f, 0, 80},
    {"fry", 30, 11.3f, .9948f, 0, 299},
};

void ccm() {
    printf("\n## CCM efficiency (Vout ~26.5 V)\n  point          Iout  plant eta  SPICE eta\n");
    double err = 0;
    for (auto &r: kCcmRef) {
        const PlantBoard *b = plantBoard(r.board);
        constexpr float rb = 0.05f;
        double best = 1e9, eta = 0, iBest = 0;
        const int h0 = (int) (26.5f / r.vin * kPmax);
        for (int hs = h0 - 20; hs < h0 + 120; ++hs) {
            VirtualConverter p;
            setup(p, b, benchL(r.board));
            p.setBat(26.5f - r.iout * rb, rb);
            p.setVout(26.5f);
            p.setPwmCount(0, hs), p.setPwmCount(1, kPmax - hs - 1);
            double si = 0, so = 0, vo = 0;
            for (int k = 0; k < 6000; ++k) {
                p.setVin(r.vin);
                p.stepSeconds(1.f / kFreq, kFreq);
                if (k >= 4000) si += p.getIinAvg(), so += p.getIoutAvg() * p.getVout(), vo += p.getIoutAvg();
            }
            const double io = vo / 2000;
            if (std::fabs(io - r.iout) < best) best = std::fabs(io - r.iout), eta = so / (si * r.vin), iBest = io;
            if (io > r.iout + 1) break;
        }
        printf("  %-4s %2.0f V %5.1f A %5.2f A  %7.2f %%  %7.2f %%\n", r.board, r.vin, r.iout, iBest, 100 * eta, 100 * r.eta);
        err += std::fabs(eta - r.eta);
        char m[64];
        snprintf(m, sizeof m, "%s %.0f V %.1f A CCM eta %.2f %%", r.board, r.vin, r.iout, 100 * eta);
        // -0.2 pp .. +0.15 pp + the avalanche share: at 3.8 A r_*/t_sw (fitted at the higher currents) cannot carry it
        check(eta - r.eta >= -0.002 && eta - r.eta <= 0.0015f + r.pAval / r.pout, m);
    }
    printf("  mean |plant - SPICE| %.2f pp\n", 100 * err / (sizeof kCcmRef / sizeof kCcmRef[0]));
}
}

int main(int argc, char **argv) {
    g_verbose = argc > 1 && !strcmp(argv[1], "-v");
    plantBoardsFromEnv();
    for (auto &b: kPlantBoards)
        printf("# %s: L %.0f uH, ring %.0f kHz at %.0f V (%.2f nF, C ~ Vin^-%.2f), Q %.0f, edge C x%.2f, Vf %.2f V, dead %.0f ns, HS +%.0f ns,"
               " R hs/ls %.0f/%.0f mOhm, t_sw %.0f ns\n", b.name, b.lPlant * 1e6f, b.ringF * 1e-3f, b.ringV, b.ringC() * 1e9f,
               b.ringCexp, b.ringQ, b.ringCt, b.vf, b.tDead * 1e9f, b.tHsExt * 1e9f, b.rHs * 1e3f, b.rLs * 1e3f, b.tSw * 1e9f);
    dawn(false);
    dawn(true);
    nearBoundary();
    p2();
    ccm();
    printf("\n%s (%d checks out of band)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail ? 1 : 0;
}
