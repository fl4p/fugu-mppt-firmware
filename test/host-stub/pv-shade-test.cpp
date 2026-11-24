// PvShade (PV emulator string model) against vconv's exact series-string model.
//   clang++ -std=gnu++20 -fexceptions -I test/host-stub -I src \
//       -o /tmp/pv-shade-test test/host-stub/pv-shade-test.cpp src/sim/vconv.cpp && /tmp/pv-shade-test
#include <cmath>
#include <cstdio>

#include "../../src/sim/vconv.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

static int countPeaks(const PvModel &m, const PvShade &sh, float *vPk = nullptr) {
    int n = 0;
    float p1 = 0, p2 = -1, v1 = m.voc;
    for (int j = 1; j <= 2000; ++j) {
        const float i = m.isc * (float) j / 2000, v = sh.voltage(m, i), p = v * i;
        if (p1 > p && p1 > p2 && p1 > 0.5f) { if (vPk) vPk[n] = v1; ++n; }
        p2 = p1, p1 = p, v1 = v;
    }
    return n;
}

int main() {
    PvShade sh;
    CHECK(sh.parse("1,1,0.4") && sh.n == 3 && sh.g[0] == 100 && sh.g[2] == 40);
    CHECK(sh.parse("off") && sh.n == 0);
    CHECK(sh.parse("") && sh.n == 0);
    CHECK(sh.parse("1") && sh.n == 0);
    CHECK(sh.parse("1.0") && sh.n == 0);
    PvShade keep;
    CHECK(keep.parse("1,0.5"));
    CHECK(!keep.parse("0.5") && keep.n == 2);      // one factor: no string, unchanged
    CHECK(!keep.parse("1,0") && keep.n == 2);
    CHECK(!keep.parse("1,1.2") && keep.n == 2);
    CHECK(!keep.parse("1,x") && keep.n == 2);
    CHECK(!keep.parse("1,1,1,1,1,1,1") && keep.n == 2);
    CHECK(!keep.parse("1,1,") && keep.n == 2);
    CHECK(!keep.parse("1, 0.5x") && keep.n == 2);
    CHECK(keep.parse("1,0.5") && keep.iscScale() == 1.0f);
    CHECK(keep.parse("0.5,0.4") && std::fabs(keep.iscScale() - 0.5f) < 1e-6f);
    CHECK(keep.parse("0.004,1") && keep.g[0] == 1);  // rounds to at least 1 %

    PvModel m;
    m.set(8.f, 40.f, 0.8f);
    PvShade none;
    for (float i = 0; i < 8.f; i += 0.37f) CHECK(none.voltage(m, i) == m.voltage(i));
    CHECK(countPeaks(m, none) == 1);

    // Equal factors: the single-panel curve below the knee.
    PvShade eq;
    eq.parse("1,1,1");
    for (float i = 0; i < 7.6f; i += 0.37f) CHECK(std::fabs(eq.voltage(m, i) - m.voltage(i)) < 1e-3f);

    const char *cases[] = {"1,1,0.4", "1,0.5", "1,0.7,0.3", "1,1,1,1,0.5,0.5", "0.6,1"};
    for (const char *cs : cases) {
        sh.parse(cs);
        // V(I) is non-increasing, so the emulator's setpoint is a function of Iout.
        float vPrev = 1e9f;
        for (int j = 0; j <= 4000; ++j) {
            const float v = sh.voltage(m, m.isc * (float) j / 4000);
            CHECK(v <= vPrev + 1e-4f);
            vPrev = v;
        }
        // Same curve as vconv's exact string model (equal Voc shares, bypass drop 0.5 V).
        VirtualConverter vc;
        vc.setPv(m.isc, m.voc, m.k);
        float g[PvShade::kMax];
        for (uint8_t k = 0; k < sh.n; ++k) g[k] = sh.g[k] * 0.01f;
        vc.setPvString(sh.n, g, nullptr, 0.5f);
        vc.setPvExact(true);
        // Outside the knee bands (last kKnee of each substring's Isc) the curves agree.
        float worst = 0, slope = 0, vS = sh.voltage(m, 0);
        int cmp = 0;
        for (int j = 1; j < 4000; ++j) {
            const float i = m.isc * (float) j / 4000, v = sh.voltage(m, i);
            slope = std::max(slope, (vS - v) / (m.isc / 4000));
            vS = v;
            bool knee = false;
            for (uint8_t k = 0; k < sh.n; ++k) {
                const float x = i / (sh.g[k] * 0.01f * m.isc);
                knee |= x > 1 - PvShade::kKnee - 1e-3f && x < 1.0f + 1e-3f;
            }
            if (v < 0.5f || knee) continue;
            worst = std::max(worst, std::fabs(vc.pvCurrent(v) - i));
            ++cmp;
        }
        float vPk[8];
        const int pk = countPeaks(m, sh, vPk);
        int distinct = 1;
        for (uint8_t a = 1; a < sh.n; ++a) {
            bool seen = false;
            for (uint8_t b = 0; b < a; ++b) seen |= sh.g[a] == sh.g[b];
            distinct += !seen;
        }
        printf("%-16s peaks %d (distinct factors %d), max |I_vconv - I| %.5f A over %d points, max slope %.0f ohm\n",
               cs, pk, distinct, worst, cmp, slope);
        CHECK(worst < 0.02f && cmp > 1000);
        CHECK(slope < 200.0f);
        CHECK(pk >= 1 && pk <= distinct);
    }
    sh.parse("1,1,0.4");
    float vPk[8];
    CHECK(countPeaks(m, sh, vPk) == 2);
    printf("1,1,0.4 peaks at %.1f V and %.1f V\n", vPk[0], vPk[1]);
    printf(fails ? "%d FAILED\n" : "all passed\n", fails);
    return fails != 0;
}
