// Late LS offset over the whole DCM range (HS from the LS min pulse to M*T: 1-count steps in the last 600 counts, step_far
// below), Vin 31-80 V, Vout 26.8 / 29.2 V, stiff sources, 4103 ct at 39 kHz. The LS window comes from biased readings
// (vout_err, vin_err: relative) with the 2 % WCE margin, plus the offset under a rule:
//   old     min(base + off, T - HS - 1)
//   gap     keep LS off T/32 before the next HS
//   v4      keep it off T/32 + off/ratio (the reverse current's drain through the ctrl body diode)
//   cap<M>  T/32 gap, no positive offset where M_WCE > M
//   fw      = v4cap0.8, SynchronousConverter::computePwmRectMax
// Each point is compared with offset 0 under the same readings ("made Iout<0": offset 0 >= 0, offset < 0).
//   band <board[-hq]> <off_ns> <vout_err> <vin_err> <rule> [step_far]
#include "sim/vconv.h"
#include "plant_boards.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <algorithm>
constexpr int kPmax = 4103, kLsMin = 320; constexpr uint32_t kFreq = 38996; constexpr double kTickNs = 1e9 / 160e6;
int main(int argc, char **argv) {
    std::string name = argv[1];
    const bool hq = name.size() > 3 && name.compare(name.size() - 3, 3, "-hq") == 0;
    if (hq) name.resize(name.size() - 3);
    PlantBoard b = *plantBoard(name.c_str());
    if (hq) b = highQ(b);
    const double offNs = atof(argv[2]), voErr = atof(argv[3]), viErr = atof(argv[4]);
    const std::string rule = !strcmp(argv[5], "fw") ? "v4cap0.8" : argv[5];
    const int stepFar = argc > 6 ? atoi(argv[6]) : 1;
    const double off = offNs / kTickNs;
    const bool v4 = rule.rfind("v4", 0) == 0, gapR = rule == "gap" || v4 || rule.rfind("cap", 0) == 0;
    const size_t cp = rule.find("cap");
    const double mcap = cp == std::string::npos ? 2 : atof(rule.c_str() + cp + 3);
    auto lsOf = [&](double base, double comp, double o, double ratio, double mm) {
        double t = base + o;
        if (o > 0 && mm > mcap) t = base;
        if (gapR && o > 0) t = std::min(t, std::max(base, comp - (kPmax >> 5) - (v4 ? o / ratio : 0)));
        return std::min<int>(std::max<long>(kLsMin, std::lround(t)), (int) comp);
    };
    auto iout = [&](float vin, float vout, int hs, int ls) {
        VirtualConverter p;
        p.setPv(1e-6f, 500.f, 0.8f); p.setPvBlocking(true); p.setPassives(470e-6f, 470e-6f, b.lPlant);
        p.setRing(b.ringC(), b.ringQ); p.setRingNode(b.ringV, b.ringCexp, b.ringCt);
        p.setDiodeVf(b.vf); p.setSwitching(b.tDead, b.tHsExt, b.rHs, b.rLs, b.tSw);
        p.setPwmMax(kPmax); p.setPwmFreq(kFreq); p.setBat(vout, 1e-6f);
        p.setPwmCount(0, hs); p.setPwmCount(1, ls);
        double io = 0;
        for (int k = 0; k < 1000; ++k) {
            p.setVin(vin); p.setVout(vout);
            p.stepSeconds(1.f / kFreq, kFreq);
            if (k >= 900) io += p.getIoutAvg();
        }
        return io / 100;
    };
    double worst = 0, worst0 = 0, worstD = 0; char w[160] = "-", w0[160] = "-";
    int npts = 0, nneg = 0, nneg0 = 0, nband = 0;
    for (float vout: {26.8f, 29.2f})
        for (float vin = 31.f; vin <= 80.f; vin += 1.f) {
            const float M = vout / vin;
            const double Mm = std::min(0.99, vout * (1 + voErr) / (vin * (1 + viErr)) / 0.980198), ratio = 1 / Mm - 1;
            const int top = (int) (M * kPmax);
            for (int hs = 200; hs <= top; hs += (hs < top - 600 ? stepFar : 1)) {
                const double base = hs * ratio, comp = kPmax - hs - 1;
                if (std::lround(base) <= kLsMin || comp <= kLsMin) continue;
                const int ls = lsOf(base, comp, off, ratio, Mm), ls0 = lsOf(base, comp, 0, ratio, Mm);
                ++npts;
                if (ls >= comp && ls0 < comp) ++nband;
                const double io0 = iout(vin, vout, hs, ls0), io = ls == ls0 ? io0 : iout(vin, vout, hs, ls);
                if (io0 < 0) { ++nneg0; if (-io0 > worst0) worst0 = -io0, snprintf(w0, sizeof w0, "Vin %.0f Vo %.1f HS %d", vin, vout, hs); }
                if (io < 0 && io0 >= 0) ++nneg;
                if (io0 >= 0 && -io > worst) worst = -io, snprintf(w, sizeof w, "Vin %.0f Vo %.1f HS %d (M*T %d) LS %d", vin, vout, hs, top, ls);
                worstD = std::max(worstD, io0 - io);
            }
        }
    printf("%-7s %-7s off %+4.0f Vo %+4.1f%% Vi %+4.1f%%: %6d pts, %3d clamp, %4d made Iout<0 (min %+.3f A [%s]), max drop %.3f A | off 0: %d neg (min %+.3f [%s])\n",
           argv[1], argv[5], offNs, voErr * 100, viErr * 100, npts, nband, nneg, -worst, w, worstD, nneg0, -worst0, w0);
}
