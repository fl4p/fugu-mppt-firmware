// Host test of the coil-inductance estimator (src/selftest/coil_fit.h) on synthetic sweeps from the
// VirtualConverter plant with the switch-node ring on: known L, ring 200..400 kHz, Q 10..40, sensor
// noise, stiff and soft (low-light PV) sources. Compares the old estimator (10 steps, median of the
// per-point L) with the new one (48 steps, ring fit); checks the L(Ipk) split (plant L = L0*(1 - k*Ipk):
// false positives at k = 0, recovery of -1 and -3 %/A) and the Iout zero reads (a stale zero offset biases
// L by +14 % with a small stated error unless corrected; a drifting zero flags the run; an irradiance ramp
// does not bias). `--fit <file>` instead fits the datasets of a whitespace file (rows "D Vin Vout Iout",
// blank line between datasets, first line of each "fsw") and prints "L f Lmed L0 k kse nb [lo hi Lb seb]..."
// per dataset (parity check against etc/measure_coil.py, see etc/test_measure_coil_fit.py).
//
//   clang++ -std=gnu++20 -O2 -I src test/host-stub/coil-fit-test.cpp src/sim/vconv.cpp -o /tmp/coil-fit-test
//   /tmp/coil-fit-test [trials] [split trials]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "sim/vconv.h"
#include "selftest/coil_fit.h"

static constexpr uint16_t kPmax = 4103;
static constexpr uint32_t kFsw = 39000;

struct Pt { float d, vin, vout, iout; };

// steady plant point at HS count h: LS count as the firmware sets it in DCM (ratio from the voltages,
// 1 % worst-case error each side), the PV source sags with the load through c_in
static Pt point(VirtualConverter &v, uint16_t h, bool soft) {
    const float r = v.getVin() / v.getVout() * (0.99f / 1.01f) - 1.f;
    const uint16_t ls = (uint16_t) std::min<long>(std::lround(h * std::max(0.f, r)), kPmax - h - 1);
    v.setPwmCount(0, h), v.setPwmCount(1, ls);
    v.stepSeconds(soft ? 0.25f : 0.01f, kFsw);
    double vi = 0, vo = 0, io = 0;
    const int n = 400;
    for (int k = 0; k < n; ++k) {
        v.stepSeconds(1.f / kFsw, kFsw);
        vi += v.getVin(), vo += v.getVout(), io += v.getIoutAvg();
    }
    return {(float) h / kPmax, (float) (vi / n), (float) (vo / n), (float) (io / n)};
}

// k: plant L DC bias (1/A); ioff: Iout zero error at the sweep start (A), decaying with tau (s, 0 = constant);
// irr: PV irradiance factor reached at the last band step; zero: the firmware's Iout zero reads before and
// after the sweep (z[0], z[1]) and their correction
struct Opt { float k = 0, ioff = 0, tau = 0, irr = 1; bool zero = false; };
struct Zero { float z0 = 0, z1 = 0, sig = 0; };
static constexpr float kStepS = 1.3f; // firmware time per step

// one sweep as measure_coil does it: band from the idle voltages, stop on CCM or Iout > 2 A, floor
static std::vector<Pt> sweep(std::mt19937 &rng, float L, float fr, float q, float voc, float vbat, float isc,
                             int steps, const Opt &o = {}, Zero *z = nullptr) {
    std::normal_distribution<float> N(0, 1);
    VirtualConverter v;
    v.setPv(isc, voc, 0.86f);
    v.setBat(vbat, 0.03f);
    v.setPassives(940e-6f, 470e-6f, L);
    v.setDiodeVf(0.7f);
    v.setRing(1 / (39.478418f * fr * fr * L), q);
    v.setPwmMax(kPmax), v.setPwmFreq(kFsw);
    v.setVin(voc), v.setVout(vbat);
    v.setLBias(o.k);
    const bool soft = isc < 5;
    int lo, hi, step;
    std::vector<Pt> all, out;
    if (!coilfit::band(voc - 0.3f, vbat, kPmax, kPmax - 330, steps, lo, hi, step)) return out;
    const float off = 0.004f * N(rng); // residual current-sensor offset after zero calibration
    auto zoff = [&](float ts) { return o.ioff * (o.tau > 0 ? std::exp(-ts / o.tau) : 1.f); };
    float iMax = 0;
    int j = 0;
    for (int h = lo; h <= hi; h += step, ++j) {
        if (o.irr != 1) v.setPv(isc * (1 + (o.irr - 1) * (float) (h - lo) / (float) (hi - lo)), voc, 0.86f);
        Pt p = point(v, (uint16_t) h, soft);
        if (p.d * p.vin / p.vout > 0.97f) break; // firmware reports CCM
        p.iout = p.iout * (1 + 0.005f * N(rng)) + 0.003f * N(rng) + off + zoff(kStepS * (float) (j + 1));
        p.vin += 0.03f * N(rng), p.vout += 0.01f * N(rng);
        all.push_back(p);
        iMax = std::max(iMax, p.iout);
        if (p.iout > 2.0f) break;
    }
    if (o.zero) {
        const float z0 = off + zoff(0) + 0.001f * N(rng), z1 = off + zoff(kStepS * (float) (j + 2)) + 0.001f * N(rng);
        std::vector<float> io(all.size());
        for (size_t i = 0; i < all.size(); ++i) io[i] = all[i].iout;
        const float sig = coilfit::zeroCorrect(io.data(), (int) io.size(), z0, z1);
        iMax = 0;
        for (size_t i = 0; i < all.size(); ++i) all[i].iout = io[i], iMax = std::max(iMax, io[i]);
        if (z) *z = {z0, z1, sig};
    }
    const float floor = std::max(0.15f, 0.2f * iMax);
    for (auto &p: all) if (p.iout >= floor && p.vin > p.vout) out.push_back(p);
    return out;
}

static coilfit::Result fitPts(const std::vector<Pt> &p, float zsig = 0) {
    float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
    const int n = std::min<int>((int) p.size(), coilfit::kMaxPts);
    for (int i = 0; i < n; ++i) d[i] = p[i].d, vi[i] = p[i].vin, vo[i] = p[i].vout, io[i] = p[i].iout;
    return coilfit::fit(d, vi, vo, io, n, (float) kFsw, zsig);
}

struct Rnd { // random board and source
    float L, fr, q, voc, vbat;
    explicit Rnd(std::mt19937 &rng) {
        std::uniform_real_distribution<float> U(0, 1);
        L = 40e-6f + 60e-6f * U(rng), fr = 200e3f + 200e3f * U(rng), q = 10 + 30 * U(rng);
        voc = 55 + 25 * U(rng), vbat = 25.5f + 3 * U(rng);
    }
};

static float rmsOf(const std::vector<float> &e) {
    float s = 0;
    for (float x: e) s += x * x;
    return e.empty() ? NAN : std::sqrt(s / (float) e.size());
}

static float meanOf(const std::vector<float> &e) {
    float s = 0;
    for (float x: e) s += x;
    return e.empty() ? NAN : s / (float) e.size();
}

// L(Ipk) split: false positives at k = 0, recovery of -1 and -3 %/A (soft and stiff source, zero-corrected)
static int testSplit(int trials) {
    int fail = 0;
    const float ks[] = {0.f, 0.01f, 0.03f}, iscs[] = {1.5f, 8.f};
    for (float k: ks)
        for (float isc: iscs) {
            std::mt19937 rng(777);
            std::vector<float> e, zs;
            int sig = 0, nbSum = 0, nbMin = 99;
            for (int t = 0; t < trials; ++t) {
                const Rnd b(rng);
                Opt o;
                o.k = k, o.zero = true;
                Zero z;
                const auto p = sweep(rng, b.L, b.fr, b.q, b.voc, b.vbat, isc, 48, o, &z);
                if ((int) p.size() < coilfit::kMinFitPts) continue;
                const auto sp = fitPts(p, z.sig);
                if (!(sp.k == sp.k)) continue;
                e.push_back((sp.k + k) * 100), zs.push_back((sp.k + k) / sp.kse);
                sig += std::fabs(sp.k) > 2 * sp.kse;
                nbSum += sp.nb, nbMin = std::min(nbMin, sp.nb);
            }
            const float rate = (float) sig / (float) e.size();
            printf("split k=%.0f %%/A, %s source, %zu sweeps: slope err mean %+.2f rms %.2f %%/A, rms z %.2f, "
                   "%s %.1f%%, bins %.1f (min %d)\n", -k * 100, isc < 5 ? "soft" : "stiff", e.size(), meanOf(e),
                   rmsOf(e), rmsOf(zs), k == 0 ? "false positive" : "significant", 100 * rate,
                   (float) nbSum / (float) e.size(), nbMin);
            // slope within 0.5 %/A; -1 %/A is near the 2-sigma limit, -3 %/A must be found
            const bool ok = std::fabs(meanOf(e)) < 0.5f && rmsOf(e) < 0.5f && (k == 0 ? rate <= 0.05f : k < 0.02f || rate > 0.95f);
            if (!ok) ++fail, printf("  FAIL\n");
        }
    return fail;
}

// Iout zero error during the sweep (stale offset calibration, thermal drift) and a PV irradiance ramp
static int testDisturbed(int trials) {
    struct Case { const char *name; Opt o; float isc; bool flag; };
    const Case cases[] = {
        {"stale zero -0.12 A", {0, -0.12f, 0, 1, false}, 8, false},
        {"stale zero -0.12 A, zero-corrected", {0, -0.12f, 0, 1, true}, 8, false},
        {"zero -0.12 A decaying 20 s, zero-corrected", {0, -0.12f, 20, 1, true}, 8, true},
        {"irradiance -40 %, soft, zero-corrected", {0, 0, 0, 0.6f, true}, 1.5f, false},
        {"irradiance -40 %, stiff, zero-corrected", {0, 0, 0, 0.6f, true}, 8, false},
    };
    int fail = 0;
    for (const auto &c: cases) {
        std::mt19937 rng(4242);
        std::vector<float> e, se;
        int flagged = 0;
        for (int t = 0; t < trials; ++t) {
            const Rnd b(rng);
            Zero z;
            const auto p = sweep(rng, b.L, b.fr, b.q, b.voc, b.vbat, c.isc, 48, c.o, &z);
            if ((int) p.size() < coilfit::kMinFitPts) continue;
            const auto r = fitPts(p, z.sig);
            e.push_back((r.L / b.L - 1) * 100), se.push_back(100 * r.se);
            flagged += std::fabs(z.z1 - z.z0) > coilfit::kZeroTol;
        }
        const float rate = (float) flagged / (float) e.size();
        std::vector<float> ae = e;
        for (float &x: ae) x = std::fabs(x);
        std::sort(ae.begin(), ae.end());
        const float p95 = ae[(size_t) (0.95f * (float) (ae.size() - 1))];
        printf("%s: %zu sweeps, L err mean %+.1f%% p95 %.1f%%, stated +-%.1f%%, flagged disturbed %.0f%%\n", c.name,
               e.size(), meanOf(e), p95, meanOf(se), 100 * rate);
        bool ok;
        if (!c.o.zero) ok = meanOf(e) > 5 && meanOf(se) < 2; // reproduces the biased-but-confident result
        else if (c.flag) ok = rate > 0.95f;
        else ok = p95 < 4 && rate < 0.05f;
        if (!ok) ++fail, printf("  FAIL\n");
    }
    return fail;
}

static int fitFile(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return 2;
    char line[256];
    std::vector<Pt> p;
    float fsw = 0;
    auto flush = [&]() {
        if (fsw <= 0) return;
        float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
        const int n = std::min<int>((int) p.size(), coilfit::kMaxPts);
        for (int i = 0; i < n; ++i) d[i] = p[i].d, vi[i] = p[i].vin, vo[i] = p[i].vout, io[i] = p[i].iout;
        const auto r = coilfit::fit(d, vi, vo, io, n, fsw);
        printf("%.9g %.9g %.9g %.9g %.9g %.9g %d", r.L, r.f, r.Lmed, r.L0, r.k, r.kse, r.nb);
        for (int b = 0; b < r.nb; ++b) printf(" %.9g %.9g %.9g %.9g", r.lo[b], r.hi[b], r.Lb[b], r.seb[b]);
        printf("\n");
        p.clear(), fsw = 0;
    };
    while (fgets(line, sizeof line, f)) {
        Pt q{};
        const int k = sscanf(line, "%f %f %f %f", &q.d, &q.vin, &q.vout, &q.iout);
        if (k == 4) p.push_back(q);
        else if (k == 1 && fsw <= 0) fsw = q.d;
        else flush();
    }
    flush();
    fclose(f);
    return 0;
}

int main(int argc, char **argv) {
    if (argc > 2 && std::string(argv[1]) == "--fit") return fitFile(argv[2]);
    const int trials = argc > 1 ? atoi(argv[1]) : 24;
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> U(0, 1);
    const float iscs[] = {0.6f, 1.5f, 8.f};
    int fail = 0;
    for (float isc: iscs) {
        std::vector<float> eOld, eNew, eF;
        int nNew = 0, ring = 0;
        for (int t = 0; t < trials; ++t) {
            const float L = 40e-6f + 60e-6f * U(rng), C = 3e-9f + 5e-9f * U(rng);
            const float fr = 1 / (6.2831853f * std::sqrt(L * C)), q = 10 + 30 * U(rng);
            const float voc = 62 + 18 * U(rng), vbat = 25.5f + 3 * U(rng);
            const auto po = sweep(rng, L, fr, q, voc, vbat, isc, 10);
            const auto pn = sweep(rng, L, fr, q, voc, vbat, isc, 48);
            if (po.size() < 3 || pn.size() < 3) continue;
            eOld.push_back(fitPts(po).Lmed / L - 1);
            const auto r = fitPts(pn);
            eNew.push_back(r.L / L - 1);
            nNew += (int) pn.size();
            if (r.f > 0) eF.push_back(r.f / fr - 1), ++ring;
        }
        auto stat = [](std::vector<float> e, float &mean, float &rms, float &p95) {
            mean = rms = 0;
            for (float x: e) mean += x, rms += x * x;
            mean /= (float) e.size(), rms = std::sqrt(rms / (float) e.size());
            for (float &x: e) x = std::fabs(x);
            std::sort(e.begin(), e.end());
            p95 = e[(size_t) (0.95f * (float) (e.size() - 1))];
        };
        float m0, r0, p0, m1, r1, p1, mf = 0, rf = 0, pf = 0;
        stat(eOld, m0, r0, p0);
        stat(eNew, m1, r1, p1);
        if (!eF.empty()) stat(eF, mf, rf, pf);
        printf("Isc %.1f A (%s source), %zu sweeps: old 10-step median: mean %+.1f%% rms %.1f%% p95 %.1f%% | "
               "new 48-step ring fit (%.0f pts): mean %+.1f%% rms %.1f%% p95 %.1f%% | ring found %d/%zu, f err rms %.1f%%\n",
               isc, isc < 5 ? "soft" : "stiff", eNew.size(), 100 * m0, 100 * r0, 100 * p0, (float) nNew / (float) eNew.size(),
               100 * m1, 100 * r1, 100 * p1, ring, eNew.size(), 100 * rf);
        if (!(r1 < r0) || !(p1 < 0.04f)) ++fail, printf("  FAIL: new estimator not better / p95 >= 4%%\n");
    }
    fail += testSplit(argc > 2 ? atoi(argv[2]) : 100);
    fail += testDisturbed(trials);
    printf("coil-fit-test: %s\n", fail ? "FAIL" : "PASS");
    return fail ? 1 : 0;
}
