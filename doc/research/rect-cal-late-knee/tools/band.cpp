// LS offset near D = M at 1-count HS steps (last 60 counts below M*T): WCE window from a biased Vout reading plus the
// offset, then the old rule (min(base + off, complementary)) or the firmware rule (a positive offset keeps the LS off for
// T/32 before the next HS, as SynchronousConverter::computePwmRectMax). Stiff sources. Points where
// the LS min pulse sets the LS are skipped. Compares each point against offset 0 at the same reading error.
//   band <board> <off_ns> <vout_err> [old|new] [cycles]
#include "sim/vconv.h"
#include "plant_boards.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <string>
constexpr int kPmax = 4103, kLsMin = 320; constexpr uint32_t kFreq = 38996; constexpr double kTickNs = 1e9 / 160e6;
int main(int argc, char **argv) {
    std::string name = argv[1];
    const bool hq = name.size() > 3 && name.compare(name.size() - 3, 3, "-hq") == 0;
    if (hq) name.resize(name.size() - 3);
    PlantBoard b = *plantBoard(name.c_str());
    if (hq) b = highQ(b);
    const double offNs = atof(argv[2]), voErr = atof(argv[3]);
    const bool fw = !(argc > 4 && !strcmp(argv[4], "old"));
    const int cycles = argc > 5 ? atoi(argv[5]) : 1000;
    const double off = offNs / kTickNs;
    auto lsOf = [&](double base, double comp, double o) {
        double t = base + o;
        if (fw && o > 0) t = std::min(t, std::max(base, comp - (kPmax >> 5)));
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
        for (int k = 0; k < cycles; ++k) {
            p.setVin(vin); p.setVout(vout);
            p.stepSeconds(1.f / kFreq, kFreq);
            if (k >= cycles - 100) io += p.getIoutAvg();
        }
        return io / 100;
    };
    double worst = 0, worstD = 0; char w[160] = "-", wd[160] = "-";
    int nband = 0, npts = 0, nneg = 0;
    for (float vout: {26.8f, 29.2f})
        for (float vin = 31.f; vin <= 80.f; vin += 1.f) {
            const float M = vout / vin;
            const double Mm = vout * (1 + voErr) / vin / 0.980198;
            for (int hs = (int) (M * kPmax) - 60; hs <= (int) (M * kPmax); ++hs) {
                const double base = hs * (1 / Mm - 1), comp = kPmax - hs - 1;
                if (std::lround(base) <= kLsMin || comp <= kLsMin) continue;
                const int ls = lsOf(base, comp, off), ls0 = lsOf(base, comp, 0);
                ++npts;
                if (ls >= comp && ls0 < comp) ++nband; // the offset, not the reading, reached the clamp
                const double io = iout(vin, vout, hs, ls), io0 = ls == ls0 ? io : iout(vin, vout, hs, ls0);
                if (io < 0 && io0 >= 0) ++nneg;
                if (io0 >= 0 && -io > worst)
                    worst = -io, snprintf(w, sizeof w, "Vin %.0f Vout %.1f HS %d (M*T %.0f) LS %d", vin, vout, hs, M * kPmax, ls);
                if (io0 - io > worstD)
                    worstD = io0 - io, snprintf(wd, sizeof wd, "Vin %.0f Vout %.1f HS %d LS %d/%d: %.3f/%.3f A", vin, vout, hs, ls, ls0, io, io0);
            }
        }
    printf("%-7s %s off %+4.0f ns Vout err %+5.1f %%: %5d pts, %4d into clamp, %4d made Iout<0, min Iout %+.3f A [%s], max drop vs off 0 %.3f A [%s]\n",
           argv[1], fw ? "new" : "old", offNs, voErr * 100, npts, nband, nneg, -worst, w, worstD, wd);
}
