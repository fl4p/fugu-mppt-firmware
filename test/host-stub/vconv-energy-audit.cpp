// Energy audit of the vconv buck plant: with stiff Vin and Vout the plant must never deliver more power than it
// draws (Pout <= Pin, also for reverse flow). Sweeps HS over the whole range with LS 0, 20, 40, 320, 1000 counts and
// synchronous CCM, Vin 10..90 V, Vout 26.5 V, for the ideal plant and the fry/flat board plants (field Q and -hq).
// Exits 1 on a violation.
//   clang++ -std=gnu++20 -O2 -I src test/host-stub/vconv-energy-audit.cpp src/sim/vconv.cpp -o /tmp/vconv-energy-audit
#include "sim/vconv.h"
#include "plant_boards.h"

#include <cmath>
#include <cstdio>

namespace {
constexpr int kPmax = 4103;
constexpr uint32_t kFreq = 38996;

struct Pw {
    double pin, pout;
    bool finite;
};

Pw run(const PlantBoard *b, float L, float vin, float vout, int hs, int ls) {
    VirtualConverter p;
    p.setPv(1e-6f, 500.f, 0.8f);
    p.setPvBlocking(true);
    p.setPassives(470e-6f, 470e-6f, L);
    if (b) {
        p.setRing(b->ringC() * b->lPlant / L, b->ringQ);
        p.setRingNode(b->ringV, b->ringCexp, b->ringCt);
        p.setDiodeVf(b->vf);
        p.setSwitching(b->tDead, b->tHsExt, b->rHs, b->rLs, b->tSw);
    }
    p.setPwmMax(kPmax), p.setPwmFreq(kFreq);
    p.setBat(vout, 1e-6f);
    p.setPwmCount(0, hs), p.setPwmCount(1, ls);
    double si = 0, so = 0;
    bool fin = true;
    for (int k = 0; k < 600; ++k) {
        p.setVin(vin), p.setVout(vout);
        p.stepSeconds(1.f / kFreq, kFreq);
        fin = fin && std::isfinite(p.getIinAvg()) && std::isfinite(p.getIoutAvg());
        if (k >= 300) si += p.getIinAvg(), so += p.getIoutAvg();
    }
    return {si / 300 * vin, so / 300 * vout, fin};
}
}

int main() {
    plantBoardsFromEnv();
    int fails = 0;
    struct Case {
        const char *name;
        const PlantBoard *b;
        float L;
    };
    const PlantBoard fry = *plantBoard("fry"), flat = *plantBoard("flat"), fryHq = highQ(fry), flatHq = highQ(flat);
    const Case cases[] = {{"ideal", nullptr, 80e-6f}, {"fry", &fry, 80e-6f}, {"fry-hq", &fryHq, 80e-6f},
                          {"flat", &flat, 51e-6f}, {"flat-hq", &flatHq, 51e-6f}};
    const float vins[] = {10, 20, 26, 26.5f, 27, 28, 30, 35, 45, 60, 75, 90};
    const int lsm[] = {0, 20, 40, 320, 1000, -1};
    for (auto &c: cases) {
        double worst = -1e9;
        char at[160] = "";
        int n = 0, bad = 0;
        for (float vin: vins)
            for (int hs = 1; hs < kPmax; hs += 37)
                for (int m: lsm) {
                    const int ls = m < 0 ? kPmax - hs - 1 : m;
                    if (hs + ls > kPmax) continue;
                    const Pw r = run(c.b, c.L, vin, 26.5f, hs, ls);
                    ++n;
                    const double gain = r.pout - r.pin, tol = 1e-4 + 1e-5 * std::fabs(r.pin);
                    if (!r.finite || gain > tol) {
                        if (++bad <= 3) printf("  %s FAIL vin %.1f hs %d ls %d pin %.4f pout %.4f%s\n", c.name, vin, hs, ls,
                                               r.pin, r.pout, r.finite ? "" : " (not finite)");
                    }
                    if (gain > worst) worst = gain, snprintf(at, sizeof at, "vin %.1f hs %d ls %d", vin, hs, ls);
                }
        printf("%-8s %d points, %d with Pout > Pin, worst Pout - Pin %+.5f W at %s\n", c.name, n, bad, worst, at);
        fails += bad;
    }
    printf("%s\n", fails ? "FAIL" : "PASS");
    return fails ? 1 : 0;
}
