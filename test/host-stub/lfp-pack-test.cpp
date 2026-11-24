// Host test: topOfCharge() (lfp_pack.h) against the 2026-11-23 BMS record (data/bms-top-2026-11-23.csv), run from the
// repo root:
//
//   clang++ -std=gnu++20 -I test/host-stub test/host-stub/lfp-pack-test.cpp -o /tmp/lfp-pack-test && /tmp/lfp-pack-test
//
// replay of the measured pack current (highest cell and pack voltage), CV holds at 3.500 V from the field state at
// four hold starts, the taper of one long hold, the relaxation after the 13:33 hold and a 0.5 A trickle.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "lfp_pack.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { printf("  FAIL: " __VA_ARGS__); printf("\n"); ++g_fail; } } while (0)

struct Row { double t; float i, v, vp; };
static std::vector<Row> g_d;
static constexpr float DT = 20.f, VHOLD = 3.5f;
static constexpr double T_REST = 13 * 3600 + 44 * 60 - 10 * 3600; // ah 0 of the fit: the 13:44 rest

static bool load(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return false;
    char ln[512];
    while (fgets(ln, sizeof ln, f)) {
        if (ln[0] < '0' || ln[0] > '9') continue;
        Row r{0, NAN, NAN, NAN};
        char *p = ln, *e;
        r.t = strtod(p, &e), p = e + 1;
        float *col[3] = {&r.i, &r.v, &r.vp};
        for (float *c: col) {
            if (*p != ',' && *p != '\n' && *p) *c = strtof(p, &e), p = e;
            if (*p == ',') ++p;
        }
        g_d.push_back(r);
    }
    fclose(f);
    return !g_d.empty();
}

static size_t idx(double t) { return std::min(g_d.size() - 1, (size_t) (t / DT)); }
static double clock(int h, int m, int s = 0) { return h * 3600 + m * 60 + s - 10 * 3600; }

// the pack at row k after replaying the measured current from the record's start
static LfpPack stateAt(size_t k) {
    double q = 0;
    for (size_t j = 0; j < idx(T_REST); ++j) q += g_d[j].i * DT / 3600;
    LfpPack p;
    topOfCharge(p, (float) -q);
    p.y = g_d[0].i, p.step(g_d[0].i, 0);
    for (size_t j = 0; j < k; ++j) p.step(g_d[j].i, DT);
    return p;
}

// one DT step with the current that puts the highest cell at VHOLD at the step's end
static float holdStep(LfpPack &p) {
    float lo = -50, hi = 200;
    for (int it = 0; it < 40; ++it) {
        LfpPack q = p;
        const float i = (lo + hi) / 2;
        q.step(i, DT);
        (q.vcellHigh(i) < VHOLD ? lo : hi) = i;
    }
    const float i = (lo + hi) / 2;
    p.step(i, DT);
    return i;
}

static float fieldMean(size_t a, size_t b) { // mean pack current over rows a..b
    double s = 0;
    for (size_t j = a; j <= b; ++j) s += g_d[j].i;
    return (float) (s / (b - a + 1));
}

static void test_replay() {
    LfpPack p = stateAt(0);
    double se = 0, sp = 0;
    int ne = 0, np = 0;
    for (auto &r: g_d) {
        p.step(r.i, DT);
        if (!std::isnan(r.v)) se += std::pow(p.vcellHigh(r.i) - r.v, 2), ++ne;
        if (!std::isnan(r.vp)) sp += std::pow((p.emf() + r.i * LfpPack::N * p.r0 - r.vp) / LfpPack::N, 2), ++np;
    }
    const float ec = (float) std::sqrt(se / ne) * 1e3f, ep = (float) std::sqrt(sp / np) * 1e3f;
    printf("  replay 10:00..16:20: highest cell rms %.1f mV (%d frames), pack per cell rms %.1f mV\n", ec, ne, ep);
    CHECK(ec < 15 && ep < 10, "replay rms %.1f / %.1f mV", ec, ep);
}

static void test_holds() {
    struct H { int h, m, mins; } hs[] = {{10, 54, 20}, {11, 50, 16}, {12, 30, 14}, {13, 5, 23}};
    for (auto &h: hs) {
        const size_t k0 = idx(clock(h.h, h.m)), k1 = k0 + h.mins * 60 / (int) DT;
        LfpPack p = stateAt(k0);
        std::vector<float> I;
        for (size_t k = k0; k <= k1; ++k) I.push_back(holdStep(p));
        const float f0 = fieldMean(k0 + 3, k0 + 9), f1 = fieldMean(k1 - 6, k1);
        const float m0 = (I[3] + I[6] + I[9]) / 3, m1 = (I[I.size() - 7] + I[I.size() - 4] + I.back()) / 3;
        printf("  hold %02d:%02d, %2d min, ah %5.1f: field %5.1f -> %5.1f A, model %5.1f -> %5.1f A\n", h.h, h.m, h.mins,
               (float) stateAt(k0).ah[0], f0, f1, m0, m1);
        CHECK(std::fabs(m1 - f1) < 1.5f, "hold %02d:%02d end current %.1f vs field %.1f A", h.h, h.m, m1, f1);
    }
}

// field taper: 20 -> 15 A in ~20 min (tau 1.2 h), 9.2 -> 7.8 A in 23 min (tau 2.45 h)
static void test_taper() {
    LfpPack p = stateAt(idx(clock(10, 54)));
    double t = 0, t20 = -1, t15 = -1, t92 = -1, t78 = -1, ah0 = p.ah[0];
    float i = 99;
    for (; t < 6 * 3600 && i > 7.8f; t += DT) {
        i = holdStep(p);
        if (t20 < 0 && i <= 20.f) t20 = t;
        if (t15 < 0 && i <= 15.f) t15 = t;
        if (t92 < 0 && i <= 9.2f) t92 = t;
        if (t78 < 0 && i <= 7.8f) t78 = t;
    }
    const double tauA = (t15 - t20) / 3600 / std::log(20. / 15), tauB = (t78 - t92) / 3600 / std::log(9.2 / 7.8);
    printf("  long hold from 10:54: 20 -> 15 A in %.0f min (tau %.2f h), 9.2 -> 7.8 A in %.0f min (tau %.2f h), "
           "%.1f Ah in %.1f h\n", (t15 - t20) / 60, tauA, (t78 - t92) / 60, tauB, p.ah[0] - ah0, t / 3600);
    CHECK(tauA > 0.8 && tauA < 2.5 && tauB > 1.5 && tauB < 4, "taper tau %.2f h / %.2f h", tauA, tauB);
}

// 13:33:10 hold end, 7.1 -> -2.2 A: field -55 mV in 60 s, then ~93 mV / tau 228 s to ~3.346 V
static void test_relax() {
    const size_t k0 = idx(clock(13, 33, 0));
    LfpPack p = stateAt(k0);
    float worst = 0;
    printf("  relaxation after 13:33:");
    for (size_t k = k0; k < k0 + 31; ++k) {
        p.step(g_d[k].i, DT);
        const float m = p.vcellHigh(g_d[k].i), f = g_d[k].v;
        if ((k - k0) % 6 == 0) printf(" +%.0fs %.3f/%.3f", (k - k0 + 1) * DT, f, m);
        if (!std::isnan(f) && k > k0) worst = std::max(worst, std::fabs(m - f));
    }
    printf(" (field/model)\n");
    CHECK(worst < 0.02f, "relaxation off by %.0f mV", worst * 1e3f);
}

// both chargers at their 0.25 A floor: 0.5 A net into the pack after the 13:33 hold, 2 h
static void test_trickle() {
    LfpPack p = stateAt(idx(clock(13, 33, 20)));
    float v1 = 0, v2 = 0;
    for (int s = 0; s < 7200; s += (int) DT) {
        p.step(0.5f, DT);
        if (s == 3600) v1 = p.vcellHigh(0.5f);
    }
    v2 = p.vcellHigh(0.5f);
    printf("  0.5 A after the 13:33 hold: cell %.4f V at 1 h, %.4f V at 2 h (%+.1f mV/h)\n", v1, v2, (v2 - v1) * 1e3f);
    CHECK(v2 - v1 < 0.001f && v2 < 3.38f, "0.5 A creeps %+.1f mV/h to %.3f V", (v2 - v1) * 1e3f, v2);
}

int main() {
    const char *path = "test/host-stub/data/bms-top-2026-11-23.csv";
    if (!load(path)) {
        printf("cannot read %s (run from the repo root)\n", path);
        return 2;
    }
    test_replay();
    test_holds();
    test_taper();
    test_relax();
    test_trickle();
    printf("%s (%d failures)\n", g_fail ? "FAILED" : "OK", g_fail);
    return g_fail != 0;
}
