// Host test for the buck LS min pulse ratio limit (SynchronousConverter::minPulseRatioMax): the real class at the MCPWM
// resolution (4103 counts) against the vconv cycle model of the field boards (plant_boards.h: ring, dead time, diode
// drop, losses), stiff Vin and Vout, LS at the min pulse, every HS count up to D = M. Checks
//   - the limit follows from the config (boot_refresh_ns, period) and start < stop
//   - up to the highest true ratio the running limit admits (reading 2 % off), the reverse current of the min pulse
//     drains within the period: the mean output current is bounded and does not grow
//   - known bad: a little above the carry-over onset 1 - pwmRectMin/T the same plant pumps amps from the output back
//     into the input, so the check above would catch a limit set too high
//
// Build & run (repo root):
//   clang++ -std=gnu++20 -O2 -fexceptions -DMOCK=1 -I test/host-stub -I src \
//       test/host-stub/minpulse-test.cpp src/sim/vconv.cpp -o /tmp/minpulse-test && /tmp/minpulse-test
// MINPULSE_LIMIT=<M> replaces the stop ratio (known-bad calibration: 0.93 and 0.90 must fail).

#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <cstdlib>

#include "esp_log.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define unlikely(x) (x)
#define OUTPUT 1
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t, int) {}

#include "../../src/buck.h"
#include "../../src/sim/vconv.h"
#include "plant_boards.h"

void assertPinState(uint8_t, bool, const char *, bool) {}
void UART_LOG(const char *, ...) {}

static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { ++g_fail; printf("FAIL %s:%d %s  ", __FILE__, __LINE__, #c); printf(__VA_ARGS__); putchar('\n'); } } while (0)

static constexpr uint32_t FSW = 39000;

static void initConv(SynchronousConverter &c, const char *topo = "buck") {
    PWM_Mock::pwmMaxOverride = 4103;
    ConfFile conv{}; conv.set("topo", topo);
    ConfFile board{}; board.set("pwm_freq", "39000"); board.set("pwm_driver_logic", "HiLi");
    board.set("pwm_hi", "21"); board.set("pwm_li", "14"); board.set("pwm_sd", "47"); board.set("skip_assert", "1");
    ConfFile coil{}; coil.set("L0", "80e-6");
    c.init(conv, board, coil);
}

// C_in 1 F: a stiff source within the cycle too (the plant steps Vin mid-cycle, so 470 uF pinned only at the cycle
// start lets the pumped charge raise Vin and self-limit the pump)
static void board(VirtualConverter &p, const PlantBoard &b, bool hq) {
    p.setPassives(1.f, 470e-6f, b.lPlant);
    p.setRing(b.ringC(), hq ? b.qHigh : b.ringQ);
    p.setRingNode(b.ringV, b.ringCexp, b.ringCt);
    p.setSwitching(b.tDead, b.tHsExt, b.rHs, b.rLs, b.tSw);
    p.setDiodeVf(b.vf);
}

struct Pt {
    float iMean, iLate, iL; // mean output current [A] over cycles 200..400 and 400..600, coil current at HS turn-on
};

// stiff source and battery: the plant's voltages are pinned each cycle (the worst case, a battery bus at the input)
static Pt run(const PlantBoard &b, bool hq, float vin, float vout, uint16_t hs, uint16_t ls, uint16_t pwmMax) {
    VirtualConverter p{};
    board(p, b, hq);
    p.setPvBlocking(false), p.setBat(vout, 1e-6f);
    p.setPwm({pwmMax, hs, ls, FSW});
    double s1 = 0, s2 = 0;
    for (int k = 0; k < 600; ++k) {
        p.setVin(vin), p.setVout(vout);
        p.stepSeconds(1.f / (float) FSW, FSW);
        if (k >= 200) (k < 400 ? s1 : s2) += p.getIoutAvg();
    }
    return {(float) (s1 / 200), (float) (s2 / 200), p.getIL()};
}

int main() {
    SynchronousConverter c{};
    initConv(c);
    const uint16_t pm = c.pwmMaxDriver(), lsMin = c.getRectOnPwmMin();
    const float T = (float) pm + (float) c.getDtLhTicks();
    // the reading-error factor the class applies on top of the T/32 margin (1 % per sensor: 0.980)
    const float wcef = c.minPulseRatioMax(false) / (1.f - ((float) lsMin + T / 32) / T);
    const float mStop = getenv("MINPULSE_LIMIT") ? (float) atof(getenv("MINPULSE_LIMIT")) : c.minPulseRatioMax(false);
    const float mCrit = 1.f - (float) lsMin / T;
    printf("pwmMax %u, LS min pulse %u counts (%.1f %% of T): carry-over onset M %.4f, stop ratio %.4f (true M up to"
           " %.4f with the reading 2 %% off), start ratio %.4f\n", pm, lsMin, 100 * lsMin / T, mCrit, mStop, mStop / wcef,
           c.minPulseRatioMax(true));

    // from the config: 2 us + dead time (the default boot_refresh_ns), T/32, a reading-error factor of a few percent
    CHECK(lsMin >= (uint16_t) (2e-6f * FSW * (float) pm), "LS min pulse %u counts below 2 us", lsMin);
    CHECK(wcef > 0.97f && wcef < 0.99f, "reading-error factor %.4f", wcef);
    CHECK(c.minPulseRatioMax(true) < c.minPulseRatioMax(false) - 0.01f, "no start hysteresis");
    CHECK(c.minPulseRatioMax(false) / wcef < mCrit, "the limit must sit below the carry-over onset");
    // reading errors beyond 1 % per sensor (Vout reads up to 1.7 % low while switching): >= 0.02 in M left to the onset
    CHECK(mCrit - mStop / wcef >= 0.02f, "stop ratio %.4f leaves %.4f to the onset at the 1 %% reading limit", mStop,
          mCrit - mStop / wcef);
    {
        // in DCM above the stop ratio (no load: DCM by current) a buck is exposed, a boost is not
        const float vin = 20.f, vout = vin * (c.minPulseRatioMax(false) + 0.01f);
        SynchronousConverter bk{}, bst{};
        initConv(bk), initConv(bst, "boost");
        CHECK(!bk.minPulseExposed(), "fresh buck starts in CCM");
        CHECK(bk.computeDCM(vin, vout, 0.f) && bst.computeDCM(vin, vout, 0.f), "no DCM at zero current");
        CHECK(bk.minPulseExposed(), "buck in DCM above the stop ratio not exposed");
        CHECK(!bst.minPulseExposed(), "boost has no rect min pulse");
    }

    const char *names[] = {"fry", "flat"};
    for (const char *n: names) {
        for (bool hq: {false, true}) {
            const PlantBoard &b = *plantBoard(n);
            for (float vout: {26.8f, 13.4f}) {
                // admitted: every HS count at the highest true ratio the limit lets run
                const float mTrue = mStop / wcef, vin = vout / mTrue;
                float worst = 0, drift = 0;
                for (uint16_t hs = 1; hs + lsMin + 1 < pm && hs < mTrue * T; hs += 4) {
                    const Pt r = run(b, hq, vin, vout, hs, lsMin, pm);
                    worst = std::min(worst, r.iMean);
                    drift = std::max(drift, r.iMean - r.iLate);
                }
                printf("%s%s Vout %.1f: true M %.4f (Vin %.2f): mean Iout min %.3f A, largest fall %.4f A\n", n,
                       hq ? "-hq" : "", vout, mTrue, vin, worst, drift);
                CHECK(worst > -0.6f, "%s%s Vout %.1f M %.4f: min-pulse reverse current %.3f A", n, hq ? "-hq" : "", vout,
                      mTrue, worst);
                CHECK(drift < 0.02f, "%s%s Vout %.1f M %.4f: reverse current still growing (%.4f A)", n, hq ? "-hq" : "",
                      vout, mTrue, drift);

                // known bad: above the onset the min pulse alone pumps amps back into the input
                const float mBad = mCrit + (getenv("MP_BAD") ? (float) atof(getenv("MP_BAD")) : 0.015f);
                float bad = 0;
                for (uint16_t hs = 1; hs < mBad * T && hs + lsMin + 1 < pm; hs += 64)
                    bad = std::min(bad, run(b, hq, vout / mBad, vout, hs, lsMin, pm).iMean);
                printf("%s%s Vout %.1f: true M %.4f (above onset): mean Iout min %.2f A\n", n, hq ? "-hq" : "", vout,
                       mBad, bad);
                CHECK(bad < -1.f, "%s%s Vout %.1f: no pump above the onset (%.3f A)?", n, hq ? "-hq" : "", vout, bad);
            }
        }
    }
    printf(g_fail ? "FAILED (%d)\n" : "all ok\n", g_fail);
    return g_fail != 0;
}
