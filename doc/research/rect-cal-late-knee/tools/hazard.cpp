// reverse current and loss of a late LS offset over the DCM operating range, stiff sources, field timing (4103 @ 39 kHz)
//   hazard <board> <off_ns> [wce|ideal] [hq|-] [hs step, default 1]
#include "sim/vconv.h"
#include "plant_boards.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>
constexpr int kPmax = 4103; constexpr uint32_t kFreq = 38996; constexpr double kTickNs = 1e9 / 160e6;
struct R { double pin, pout, ils, il0, ilmin; bool dcm; };
static R run(const PlantBoard &b, float vin, float vout, int hs, int ls) {
    VirtualConverter p;
    p.setPv(1e-6f, 500.f, 0.8f); p.setPvBlocking(true); p.setPassives(470e-6f, 470e-6f, b.lPlant);
    p.setRing(b.ringC(), b.ringQ); p.setRingNode(b.ringV, b.ringCexp, b.ringCt);
    p.setDiodeVf(b.vf); p.setSwitching(b.tDead, b.tHsExt, b.rHs, b.rLs, b.tSw);
    p.setPwmMax(kPmax); p.setPwmFreq(kFreq); p.setBat(vout, 1e-6f);
    p.setPwmCount(0, hs); p.setPwmCount(1, std::min(ls, kPmax - hs - 1));
    R r{0, 0, 0, 0, 1e9, true};
    for (int k = 0; k < 800; ++k) {
        p.setVin(vin); p.setVout(vout);
        const float a = p.getIL();
        p.stepSeconds(1.f / kFreq, kFreq);
        if (k >= 400) r.pin += p.getIinAvg() * vin, r.pout += p.getIoutAvg() * vout, r.ils += p.getILsOff(), r.il0 += a,
                      r.ilmin = std::min(r.ilmin, (double) a), r.dcm &= p.inDcm();
    }
    r.pin /= 400, r.pout /= 400, r.ils /= 400, r.il0 /= 400;
    return r;
}
int main(int argc, char **argv) {
    const double offNs = argc > 2 ? atof(argv[2]) : 0;
    const bool wce = argc > 3 && !strcmp(argv[3], "wce");
    const int step = argc > 5 ? atoi(argv[5]) : 1;
    PlantBoard b = *plantBoard(argv[1]);
    if (argc > 4 && !strcmp(argv[4], "hq")) b = highQ(b);
    double worstI = 0, worstE = 0, worstDp = 0, worstCarry = 0, worstDpRel = 0;
    char wI[128] = "", wDp[128] = "", wC[128] = "";
    int n = 0;
    for (float vout: {26.8f, 29.2f})
        for (float vin = 31.f; vin <= 80.f; vin += 1.f) {
            const float M = vout / vin;
            for (int hs = 300; hs < M * kPmax; hs += step) {
                const double ideal = (vin / vout - 1) * hs;
                const double lsF = wce ? hs * (0.980198 / M - 1) : ideal;
                if (hs + (int) std::lround(lsF) >= kPmax - 1) continue;
                double t = lsF + offNs / kTickNs; // SynchronousConverter::computePwmRectMax: T/32 idle before HS
                if (offNs > 0) t = std::min(t, std::max(lsF, (double) (kPmax - hs - 1 - (kPmax >> 5))));
                const int ls = std::min(kPmax - hs - 1, std::max(320, (int) std::lround(t)));
                const R r = run(b, vin, vout, hs, ls), r0 = run(b, vin, vout, hs, std::max(320, (int) std::lround(lsF)));
                if (!r0.dcm) continue;
                ++n;
                if (ls <= 320) continue; // LS minimum pulse: the offset does not act
                const double irev = -std::min(0.0, r.ils), e = 0.5 * b.lPlant * irev * irev;
                const double dp = (r0.pout - r0.pin) - (r.pout - r.pin); // extra loss vs offset 0 (W)
                if (irev > worstI) worstI = irev, worstE = e, snprintf(wI, sizeof wI, "Vin %.0f Vout %.1f HS %d LS %d", vin, vout, hs, ls);
                if (dp > worstDp) worstDp = dp, worstDpRel = dp / std::max(1e-3, r0.pout), snprintf(wDp, sizeof wDp, "Vin %.0f Vout %.1f HS %d (Pout %.1f W)", vin, vout, hs, r0.pout);
                if (-r.ilmin > worstCarry) worstCarry = -r.ilmin, snprintf(wC, sizeof wC, "Vin %.0f Vout %.1f HS %d LS %d", vin, vout, hs, ls);
            }
        }
    printf("%-8s off %+4.0f ns %s: %d DCM pts | max reverse at LS off %.3f A (%.2f uJ, %.0f mW circ.) at %s | max coil current at HS on %+.3f A at %s | max extra loss vs off 0: %.1f mW at %s\n",
           argv[1], offNs, wce ? "WCE" : "LS=ideal+off", n, worstI, worstE * 1e6, worstE * kFreq * 1e3, wI, -worstCarry, wC, worstDp * 1e3, wDp);
}
