// plant zero crossing of the coil current at LS turn-off vs ideal, stiff sources, per board
#include "sim/vconv.h"
#include "plant_boards.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
constexpr int kPmax = 4103; constexpr uint32_t kFreq = 38996; constexpr float kTick = 1.f / 160e6f, kT = kPmax * kTick;
struct R { double iout, ils, il0; };
static R run(const PlantBoard &b, bool ideal, float vin, float vout, int hs, int ls) {
    VirtualConverter p;
    p.setPv(1e-6f, 500.f, 0.8f); p.setPvBlocking(true);
    p.setPassives(470e-6f, 470e-6f, b.lPlant);
    if (!ideal) {
        p.setRing(b.ringC(), b.ringQ); p.setRingNode(b.ringV, b.ringCexp, b.ringCt);
        p.setDiodeVf(b.vf); p.setSwitching(b.tDead, b.tHsExt, b.rHs, b.rLs, b.tSw);
    }
    p.setPwmMax(kPmax); p.setPwmFreq(kFreq); p.setBat(vout, 1e-6f);
    p.setPwmCount(0, hs); p.setPwmCount(1, ls);
    double si = 0, sc = 0, s0 = 0;
    for (int k = 0; k < 600; ++k) {
        p.setVin(vin); p.setVout(vout);
        const float a = p.getIL();
        p.stepSeconds(1.f / kFreq, kFreq);
        if (k >= 300) si += p.getIoutAvg(), sc += p.getILsOff(), s0 += a;
    }
    return {si / 300, sc / 300, s0 / 300};
}
// first + -> - crossing of the LS turn-off current, fractional LS count; NAN if none
static double zcTrue(const PlantBoard &b, bool ideal, float vin, float vout, int hs, double &il0) {
    const double id = (vin / vout - 1) * hs;
    int lo = std::max(1, (int) (0.7 * id));
    double prev = run(b, ideal, vin, vout, hs, lo).ils;
    for (int ls = lo + 1; ls < kPmax - hs - 1 && ls < id + 800; ++ls) {
        R r = run(b, ideal, vin, vout, hs, ls);
        if (prev > 0 && r.ils <= 0) { il0 = r.il0; return ls - r.ils / (r.ils - prev); }
        prev = r.ils;
    }
    return NAN;
}
int main(int argc, char **argv) {
    const char *bn = argc > 1 ? argv[1] : "fry";
    const bool hq = argc > 2 && !strcmp(argv[2], "hq");
    const bool ideal = argc > 2 && !strcmp(argv[2], "ideal");
    PlantBoard b = *plantBoard(bn); if (hq) b = highQ(b);
    const float vout = argc > 3 ? atof(argv[3]) : 26.8f;
    const float Lm = 0.95f * b.l0Conf;
    printf("# %s%s Vout %.1f  cols: vin hs ideal zc-ideal(ns) il0(A) zcQ-ideal(ns, model L) zcQ(plant L)\n", bn, hq ? "-hq" : ideal ? "-ideal" : "", vout);
    for (float vin: {35.f, 40.f, 45.f, 50.f, 55.f, 60.f, 66.f, 72.f, 78.f}) {
        const float M = vout / vin;
        for (float fr: {0.3f, 0.4f, 0.55f, 0.7f, 0.85f}) {
            const int hs = (int) std::lround(fr * M * kPmax);
            const double id = (vin / vout - 1) * hs;
            if (id + hs > 0.95 * kPmax) continue;
            double il0 = 0;
            const double zc = zcTrue(b, ideal, vin, vout, hs, il0);
            // charge-based zc from the mean current at LS 0.8..1 x ideal
            double iq = 0; int nq = 0;
            for (double x = 0.8 * id; x <= id; x += 0.05 * id) iq += run(b, ideal, vin, vout, hs, (int) std::lround(x)).iout, ++nq;
            iq /= nq;
            auto zq = [&](float L) { return std::sqrt(std::max(0.0, 2 * iq * kT * L * (vin - vout) / (vout * vin))) * 160e6; };
            printf("%5.0f %5d %6.0f %+7.0f %+6.3f %+7.0f %+7.0f\n", vin, hs, id, (zc - id) * 6.25, il0, (zq(Lm) - id) * 6.25, (zq(b.lPlant) - id) * 6.25);
        }
    }
}
