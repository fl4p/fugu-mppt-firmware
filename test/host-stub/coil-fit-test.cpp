// Host test of the coil-inductance estimator (src/selftest/coil_fit.h) on synthetic sweeps from the
// VirtualConverter plant with the switch-node ring on: known L, ring 200..400 kHz, Q 10..40, sensor
// noise, stiff and soft (low-light PV) sources. Compares the old estimator (10 steps, median of the
// per-point L) with the new one (48 steps, ring fit). `--fit <file>` instead fits the datasets of a
// whitespace file (rows "D Vin Vout Iout", blank line between datasets, first line of each "fsw") and
// prints "L f" per dataset (parity check against etc/measure_coil.py, see etc/test_measure_coil_fit.py).
//
//   clang++ -std=gnu++20 -O2 -I src test/host-stub/coil-fit-test.cpp src/sim/vconv.cpp -o /tmp/coil-fit-test
//   /tmp/coil-fit-test [trials]
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

// one sweep as measure_coil does it: band from the idle voltages, stop on CCM or Iout > 2 A, floor
static std::vector<Pt> sweep(std::mt19937 &rng, float L, float fr, float q, float voc, float vbat, float isc,
                             int steps) {
    std::normal_distribution<float> N(0, 1);
    VirtualConverter v;
    v.setPv(isc, voc, 0.86f);
    v.setBat(vbat, 0.03f);
    v.setPassives(940e-6f, 470e-6f, L);
    v.setDiodeVf(0.7f);
    v.setRing(1 / (39.478418f * fr * fr * L), q);
    v.setPwmMax(kPmax), v.setPwmFreq(kFsw);
    v.setVin(voc), v.setVout(vbat);
    const bool soft = isc < 5;
    int lo, hi, step;
    std::vector<Pt> all, out;
    if (!coilfit::band(voc - 0.3f, vbat, kPmax, kPmax - 330, steps, lo, hi, step)) return out;
    const float off = 0.004f * N(rng); // residual current-sensor offset after zero calibration
    float iMax = 0;
    for (int h = lo; h <= hi; h += step) {
        Pt p = point(v, (uint16_t) h, soft);
        if (p.d * p.vin / p.vout > 0.97f) break; // firmware reports CCM
        p.iout = p.iout * (1 + 0.005f * N(rng)) + 0.003f * N(rng) + off;
        p.vin += 0.03f * N(rng), p.vout += 0.01f * N(rng);
        all.push_back(p);
        iMax = std::max(iMax, p.iout);
        if (p.iout > 2.0f) break;
    }
    const float floor = std::max(0.15f, 0.2f * iMax);
    for (auto &p: all) if (p.iout >= floor && p.vin > p.vout) out.push_back(p);
    return out;
}

static coilfit::Result fitPts(const std::vector<Pt> &p) {
    float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
    const int n = std::min<int>((int) p.size(), coilfit::kMaxPts);
    for (int i = 0; i < n; ++i) d[i] = p[i].d, vi[i] = p[i].vin, vo[i] = p[i].vout, io[i] = p[i].iout;
    return coilfit::fit(d, vi, vo, io, n, (float) kFsw);
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
        printf("%.9g %.9g %.9g\n", r.L, r.f, r.Lmed);
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
    printf("coil-fit-test: %s\n", fail ? "FAIL" : "PASS");
    return fail ? 1 : 0;
}
