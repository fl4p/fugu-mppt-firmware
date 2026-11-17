// Diode-emulation LS window over the whole DCM range against the board plants (offset 0), stiff sources.
// Vout 26.8 / 29.2 V; Vin from Vout + 0.6 V (0.2 V steps up to 31 V, then 1 V) to vinMax; HS from 200 counts to M*T
// (1-count steps in the last `near` counts, `stepFar` below), 4103 ct at 39 kHz, LS min pulse 2 us (320 ct).
// Readings Vout x (1 + vo_err), Vin x (1 + vi_err), Iout read 0.05 A (DCM, sync rect on).
// Rules (LS count per point):
//   fw        the real SynchronousConverter (MOCK driver) of the tree this is built against; DEWIN_OFF_NS sets rect_offset_ns
//   emu       the pre-fix rule: lround(HS * r), r = 1/M_WCE - 1, M_WCE = M_read / 0.980198 (<= 0.99)
//   gap<K>    emu, LS ends K/r counts before the next HS (drain of K counts of late conduction)
//   gapT<K>   gap<K> plus T/32
//   cap<M>    emu, LS at the min pulse where M_WCE > M
//   abs<K>    emu - K counts
// Per rule: points with mean Iout < 0 split by LS above the min pulse ("window") or at it ("min"), the coil current at
// HS turn-on, and the loss (Vin*Iin - Vout*Iout) against the first rule per M bin (cost).
//   dewin <board[-hq]> <vo_err> <vi_err> <stepFar> <near> <vinMax> <rule>...
// Build: clang++ -std=gnu++20 -O2 -fexceptions -DMOCK=1 -I <tree>/test/host-stub -I <tree>/src dewin.cpp <tree>/src/sim/vconv.cpp
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include "esp_log.h"
#undef ESP_LOGI
#define ESP_LOGI(...) ((void) 0)
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define unlikely(x) (x)
#define OUTPUT 1
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t, int) {}
#include "buck.h"
#include "sim/vconv.h"
#include "plant_boards.h"
void assertPinState(uint8_t, bool, const char *, bool) {}
void UART_LOG(const char *, ...) {}

static constexpr uint16_t PM = 4103;
static constexpr uint32_t FSW = 39000;
static char g_l0[24];
static int g_lsMin = -1;

static void initConv(SynchronousConverter &c) {
    ConfFile conv{}; conv.set("topo", "buck");
    ConfFile board{}; board.set("pwm_freq", "39000"); board.set("pwm_driver_logic", "HiLi");
    board.set("pwm_hi", "21"); board.set("pwm_li", "14"); board.set("pwm_sd", "47"); board.set("skip_assert", "1");
    ConfFile coil{}; coil.set("L0", g_l0);
    if (const char *o = getenv("DEWIN_OFF_NS")) coil.set("rect_offset_ns", o), coil.set("rect_offset_drv", "mock");
    c.init(conv, board, coil);
}

static int lsFw(float vi, float vo, int hs, float il) {
    SynchronousConverter c{};
    initConv(c);
    c.enableSyncRect(true);
    c.pwmPerturb((int16_t) hs);
    for (int k = 0; k < 300; ++k) { c.updateSyncRectMaxDuty(vi, vo, il); c.pwmPerturb(0); }
    return c.getRectOnPwmCnt();
}

struct Pt { double io, iin, i0; };
static Pt plant(const PlantBoard &b, float vin, float vout, int hs, int ls) {
    VirtualConverter p;
    p.setPv(1e-6f, 500.f, 0.8f); p.setPvBlocking(true); p.setPassives(470e-6f, 470e-6f, b.lPlant);
    p.setRing(b.ringC(), b.ringQ); p.setRingNode(b.ringV, b.ringCexp, b.ringCt);
    p.setDiodeVf(b.vf); p.setSwitching(b.tDead, b.tHsExt, b.rHs, b.rLs, b.tSw);
    p.setPwmMax(PM); p.setPwmFreq(FSW); p.setBat(vout, 1e-6f);
    p.setPwmCount(0, hs); p.setPwmCount(1, ls);
    Pt r{0, 0, 0};
    for (int k = 0; k < 1000; ++k) {
        p.setVin(vin); p.setVout(vout);
        p.stepSeconds(1.f / FSW, FSW);
        if (k >= 900) r.io += p.getIoutAvg(), r.iin += p.getIinAvg();
    }
    r.io /= 100, r.iin /= 100, r.i0 = p.getIL();
    return r;
}

struct Rule {
    std::string name;
    int kind = 0; // 0 fw, 1 emu, 2 gap, 3 gapT, 4 cap, 5 abs
    double k = 0;
    int nWin = 0, nMin = 0, nCarry = 0, nPts = 0;
    double worstWin = 0, worstMin = 0;
    char wWin[128] = "-", wMin[128] = "-";
    double dLossMax[5] = {}, dLossSum[5] = {}, dEtaMax[5] = {}, lossSum[5] = {};
    int nBin[5] = {};
};

static int lsRule(const Rule &R, float vir, float vor, float vin, float vout, int hs) {
    if (R.kind == 0) return lsFw(vir, vor, hs, 0.05f);
    const double Mm = std::min(0.99, (double) std::clamp(vor / vir, 1e-2f, 0.99f) / 0.980198), r = 1 / Mm - 1;
    const int comp = PM - hs - 1;
    double t = hs * r;
    if (R.kind == 2) t = std::min(t, comp - R.k / r);
    if (R.kind == 3) t = std::min(t, comp - (PM >> 5) - R.k / r);
    if (R.kind == 4 && Mm > R.k) t = 0;
    if (R.kind == 5) t -= R.k;
    (void) vin, (void) vout;
    return std::min<long>(std::max<long>(g_lsMin, std::lround(t)), comp);
}

static int mBin(double M) { return M < 0.5 ? 0 : M < 0.7 ? 1 : M < 0.8 ? 2 : M < 0.9 ? 3 : 4; }

int main(int argc, char **argv) {
    if (argc < 8) {
        fprintf(stderr, "usage: dewin <board[-hq]> <vo_err> <vi_err> <stepFar> <near> <vinMax> <rule>...\n");
        return 2;
    }
    std::string name = argv[1];
    const bool hq = name.size() > 3 && name.compare(name.size() - 3, 3, "-hq") == 0;
    if (hq) name.resize(name.size() - 3);
    PlantBoard b = *plantBoard(name.c_str());
    if (hq) b = highQ(b);
    snprintf(g_l0, sizeof g_l0, "%g", b.l0Conf);
    PWM_Mock::pwmMaxOverride = PM;
    const double voErr = atof(argv[2]), viErr = atof(argv[3]);
    const int stepFar = atoi(argv[4]), nearCt = atoi(argv[5]);
    const float vinMax = (float) atof(argv[6]);
    std::vector<Rule> rules;
    for (int a = 7; a < argc; ++a) {
        Rule R; R.name = argv[a];
        const std::string s = argv[a];
        if (s == "fw") R.kind = 0;
        else if (s == "emu") R.kind = 1;
        else if (s.rfind("gapT", 0) == 0) R.kind = 3, R.k = atof(s.c_str() + 4);
        else if (s.rfind("gap", 0) == 0) R.kind = 2, R.k = atof(s.c_str() + 3);
        else if (s.rfind("cap", 0) == 0) R.kind = 4, R.k = atof(s.c_str() + 3);
        else if (s.rfind("abs", 0) == 0) R.kind = 5, R.k = atof(s.c_str() + 3);
        else { fprintf(stderr, "unknown rule %s\n", argv[a]); return 2; }
        rules.push_back(R);
    }
    { SynchronousConverter c{}; initConv(c); g_lsMin = c.getRectOnPwmMin(); }
    const bool dump = getenv("DEWIN_DUMP") != nullptr;
    for (float vout: {26.8f, 29.2f})
        for (float vin = vout + 0.6f; vin <= vinMax + 1e-3f; vin = vin < 31.f ? vin + 0.2f : vin + 1.f) {
            const float M = vout / vin;
            const int top = (int) (M * PM);
            const float vir = vin * (float) (1 + viErr), vor = vout * (float) (1 + voErr);
            for (int hs = 200; hs <= top; hs += (hs < top - nearCt ? stepFar : 1)) {
                if (hs > (int) PM - g_lsMin - 1) break; // above pwmCtrlMax
                int ls0 = -1; Pt r0{};
                for (size_t i = 0; i < rules.size(); ++i) {
                    Rule &R = rules[i];
                    const int ls = lsRule(R, vir, vor, vin, vout, hs);
                    Pt r = (i > 0 && ls == ls0) ? r0 : plant(b, vin, vout, hs, ls);
                    if (i == 0) ls0 = ls, r0 = r;
                    ++R.nPts;
                    if (r.i0 < -0.05) ++R.nCarry;
                    if (dump) printf("P %s %.1f %.1f %d %d %d %.4f %.4f %.4f\n", R.name.c_str(), vin, vout, hs, ls, top, r.io, r.iin, r.i0);
                    if (r.io < 0) {
                        char w[128];
                        snprintf(w, sizeof w, "Vin %.1f Vo %.1f HS %d (M*T %d) LS %d i0 %+.2f", vin, vout, hs, top, ls, r.i0);
                        if (ls <= g_lsMin) { ++R.nMin; if (-r.io > R.worstMin) R.worstMin = -r.io, strcpy(R.wMin, w); }
                        else { ++R.nWin; if (-r.io > R.worstWin) R.worstWin = -r.io, strcpy(R.wWin, w); }
                    }
                    const int bn = mBin(M);
                    const double loss0 = vin * r0.iin - vout * r0.io, loss = vin * r.iin - vout * r.io;
                    const double eta0 = r0.iin > 1e-3 ? vout * r0.io / (vin * r0.iin) : 0, eta = r.iin > 1e-3 ? vout * r.io / (vin * r.iin) : 0;
                    R.dLossMax[bn] = std::max(R.dLossMax[bn], loss - loss0);
                    R.dLossSum[bn] += loss - loss0;
                    R.lossSum[bn] += loss;
                    if (r0.io > 0.05 && r.io > 0.05) R.dEtaMax[bn] = std::max(R.dEtaMax[bn], (eta0 - eta) * 100);
                    ++R.nBin[bn];
                }
            }
        }
    for (auto &R: rules) {
        printf("%-8s %-8s Vo %+4.1f%% Vi %+4.1f%%: %6d pts | Iout<0 window %5d (min %+.3f A [%s]) | at min pulse %5d (min %+.3f A [%s]) | i0<-0.05 %d\n",
               argv[1], R.name.c_str(), voErr * 100, viErr * 100, R.nPts, R.nWin, -R.worstWin, R.wWin, R.nMin, -R.worstMin, R.wMin, R.nCarry);
        if (&R != &rules[0]) {
            printf("%-8s %-8s   extra loss vs %s, M <0.5 / 0.5-0.7 / 0.7-0.8 / 0.8-0.9 / >0.9: max [mW]", argv[1], R.name.c_str(), rules[0].name.c_str());
            for (int k = 0; k < 5; ++k) printf(" %.1f", R.dLossMax[k] * 1e3);
            printf(" | mean [mW]");
            for (int k = 0; k < 5; ++k) printf(" %.2f", R.nBin[k] ? R.dLossSum[k] / R.nBin[k] * 1e3 : 0);
            printf(" | max eta drop [pp]");
            for (int k = 0; k < 5; ++k) printf(" %.3f", R.dEtaMax[k]);
            printf("\n");
        }
    }
}
