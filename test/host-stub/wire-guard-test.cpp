// Host test for wire_guard.h: a synthetic charging plant (day current profile, sibling charger,
// varying load, BMS frame noise, sweeps) feeding the real WireResistance and WireGuard.
//
//   clang++ -std=gnu++20 -O2 -fexceptions -I src test/host-stub/wire-guard-test.cpp \
//       -o /tmp/wire-guard-test && /tmp/wire-guard-test
//
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>
#include "out_impedance.h"
#include "wire_guard.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { printf("  FAIL: " __VA_ARGS__); printf("\n"); ++g_fail; } } while (0)

static constexpr uint32_t DAY = 86400, TICK = 3;

struct Sim {
    WireGuard g{};
    WireResistance w{};
    float rc = 0.014f, a = 0.020f, rb = 0.003f, rbat = 0.006f;
    float dRc = 0, dRb = 0;     // injected fault [Ω]
    float dRcPerDay = 0;        // slow ramp [Ω/day] from faultT
    uint32_t faultT = UINT32_MAX;
    // converter-vs-BMS voltage offset drift from driftT: ramp [V/day] up to ±aRampMax, plus a 6 h wave [V]
    float aRampPerDay = 0, aRampMax = INFINITY, aWave = 0;
    uint32_t driftT = UINT32_MAX;
    float ipk = 28.f, flat = 0.8f, sigV = 0.015f;
    float loadAmp = 3.f;
    bool bms = true, rcFeed = true, roFeed = true, honorCap = true;
    uint32_t sweepS = 1800, nSweep = 0;
    // sweep outliers of roOut [Ω]: bit (sweep # mod 5) of outMask, or with probability outP
    float roOut = 0, outP = 0;
    uint8_t outMask = 0;
    uint32_t t = 0, roStamp = 0;
    // restarts: the guard sees uptime t - bootT; persist emulates the NVS save and the RTC day copy
    uint32_t bootT = 0;
    bool persist = false, haveFlash = false;
    WireGuard::Blob flash{};
    WireGuard::DayRtc rtc{};
    WireGuard::FitRtc fit{};
    float ro = NAN, prevI = NAN;
    std::mt19937 rng{1};

    // first time effLevel reached each level, and when wf first showed it
    uint32_t firstEff[4]{UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    uint8_t maxEff = 0, maxLvl = 0;
    bool rbSeen = false;
    float minCap = INFINITY;
    uint8_t dbgL = 0;
    bool dbgRb = false;

    explicit Sim(uint8_t mode = 2, uint32_t seed = 1) : rng(seed) { g.conf.mode = mode; }

    float profile(uint32_t s) const {
        const float h = (float) (s % DAY) / 3600.f;
        return h > 7 && h < 17 ? ipk * std::sin(3.14159f * (h - 7) / 10) : 0.f;
    }

    float offset(uint32_t s) const {
        if (s < driftT) return a;
        const float d = (float) (s - driftT), r = aRampPerDay * d / DAY;
        return a + (std::fabs(r) > aRampMax ? std::copysign(aRampMax, r) : r) + aWave * std::sin(6.2832f * d / (6 * 3600));
    }

    float faultRc(uint32_t s) const {
        if (s < faultT) return 0;
        return dRc + dRcPerDay * (float) (s - faultT) / DAY;
    }

    void step() {
        std::normal_distribution<float> nv(0, sigV), nro(0, 0.0005f);
        const uint32_t day = t / DAY;
        // copper temperature, day to day
        const float rcDay = rc + 0.0007f * std::sin((float) day * 1.7f);
        float i = profile(t);
        const float cap = g.ioutCap();
        if (honorCap && std::isfinite(cap)) { i = std::min(i, cap); minCap = std::min(minCap, cap); }
        if (g.tripped()) i = 0;
        const bool running = i > 0.3f;
        const float load = 2.f + loadAmp * (0.5f + 0.5f * std::sin((float) t / 1700.f));
        const float ib = i + flat * profile(t) - load;
        const float fr = faultRc(t), fb = t >= faultT ? dRb : 0.f;
        const float dv = offset(t) + (rcDay + fr) * i + (rb + fb) * ib + nv(rng);

        if (bms && running && i > 0.3f) {
            if (std::fabs(i - prevI) < 0.5f) {
                if (rcFeed) w.add(dv, i, ib, t - bootT);
                g.frame(dv, i, ib);
            }
            prevI = i;
        } else prevI = NAN;

        if (roFeed && running && i >= 3 && t % sweepS == 0) {
            ro = rcDay + fr + rb + fb + rbat + 0.001f * std::sin((float) t / 9000.f) + nro(rng);
            if ((outMask >> (nSweep % 5) & 1) || (outP > 0 && std::uniform_real_distribution<float>()(rng) < outP))
                ro += roOut;
            ++nSweep;
            roStamp = t;
        }
        const uint32_t up = t - bootT;
        const float rcv = rcFeed ? w.own(up) : NAN;
        const bool roOk = roFeed && roStamp > bootT && t - roStamp < 6 * 3600;
        g.tick(up, i, running, rcv, rcFeed ? w.offset(up) : NAN, rcFeed ? w.shared(up) : NAN, roOk ? ro : NAN,
               roOk ? roStamp - bootT : 0);
        if (persist) {
            if (g.saveDue(running)) {
                flash = g.blob();
                haveFlash = true;
                g.saved();
            }
            g.saveDay(rtc);
            g.saveFit(fit);
        }

        if (getenv("WG_DEBUG") && (g.level() != dbgL || g.rbWarn() != dbgRb)) {
            dbgL = g.level(); dbgRb = g.rbWarn();
            auto f = g.fastPath(), r = g.rcPath(), o = g.roPath(), b = g.rbPath();
            printf("    d%.2f lvl=%u rb=%d fast %d %.2f/%.2f rc %d %.2f/%.2f ro %d %.2f/%.2f rb %d %.2f/%.2f iref=%.1f\n",
                   t / (float) DAY, dbgL, dbgRb, f.fresh, f.dr * 1e3, f.pw, r.fresh, r.dr * 1e3, r.pw, o.fresh,
                   o.dr * 1e3, o.pw, b.fresh, b.dr * 1e3, b.pw, g.iRef());
        }
        const uint8_t e = g.effLevel();
        for (uint8_t l = 1; l <= e; ++l)
            if (firstEff[l] == UINT32_MAX) firstEff[l] = t;
        maxEff = std::max(maxEff, e);
        maxLvl = std::max(maxLvl, g.level());
        rbSeen |= g.rbWarn();
        t += TICK;
    }

    void runUntil(uint32_t end) { while (t < end) step(); }

    // restart as the firmware does: baseline from the last save, open day and regression from RTC if kept
    void reboot(bool rtcKept, bool fitKept = true) {
        const auto c = g.conf;
        const uint8_t p = g.paths;
        g.~WireGuard();
        new(&g) WireGuard();
        g.conf = c;
        g.paths = p;
        if (haveFlash) g.load(&flash, sizeof flash);
        if (rtcKept) g.restoreDay(rtc);
        if (rtcKept && fitKept) g.restoreFit(fit);
        w = WireResistance{};
        prevI = NAN;
        bootT = t - 8;
    }

    float det(uint8_t l) const { // detection delay [min] after the fault, NAN = never
        return firstEff[l] == UINT32_MAX || firstEff[l] < faultT ? NAN : (float) (firstEff[l] - faultT) / 60.f;
    }
};

static constexpr uint32_t ON = 9 * DAY + 12 * 3600; // fault onset: day 9, noon at peak current (learning over)

static void faultRun(Sim &s, float dR, bool fast, bool rcp, bool rop) {
    s.runUntil(ON);
    s.g.paths = (fast ? 1 : 0) | (rcp ? 2 : 0) | (rop ? 4 : 0);
    s.faultT = ON;
    s.dRc = dR;
    s.runUntil(ON + 2 * DAY);
}

static void printRun(const char *name, const Sim &s) {
    printf("  %-28s maxEff=%u maxLvl=%u warn=%7.1f derate=%7.1f trip=%7.1f min  cap=%.1fA\n", name, s.maxEff,
           s.maxLvl, s.det(1), s.det(2), s.det(3), std::isfinite(s.minCap) ? s.minCap : NAN);
}

static void testClean() {
    printf("clean 30 days, all paths\n");
    for (uint32_t seed = 1; seed <= 3; ++seed) {
        Sim s{2, seed};
        s.runUntil(30 * DAY);
        printf("  seed %u: maxLvl=%u days=%u base Rc=%.2f a=%.1fmV Rb=%.2f Ro=%.2f mΩ wf=%d\n", seed, s.maxLvl,
               s.g.dayCount(), s.g.baseRc() * 1e3, s.g.baseA() * 1e3, s.g.baseRb() * 1e3, s.g.baseRo() * 1e3,
               s.g.wf());
        CHECK(s.maxLvl == 0, "clean run raised level %u", s.maxLvl);
        CHECK(!s.rbSeen, "clean run raised Rb warn");
        CHECK(!s.g.learning(), "still learning after 30 days");
        CHECK(std::fabs(s.g.baseRc() - 0.014f) < 0.0015f, "Rc baseline %.4f", s.g.baseRc());
        CHECK(std::fabs(s.g.baseRo() - 0.023f) < 0.0015f, "Ro baseline %.4f", s.g.baseRo());
    }
}

static void testSteps() {
    printf("resistance steps per path (fault on day 9 at the 28 A noon peak)\n");
    struct P { const char *n; bool f, r, o; } paths[]{{"fast", 1, 0, 0}, {"Rc", 0, 1, 0}, {"Ro", 0, 0, 1}, {"all", 1, 1, 1}};
    for (auto &p: paths) {
        for (float dR: {0.003f, 0.006f, 0.015f}) {
            char name[48];
            snprintf(name, sizeof name, "%s +%.0f mOhm", p.n, dR * 1e3);
            Sim s{2};
            faultRun(s, dR, p.f, p.r, p.o);
            printRun(name, s);
            // the slope path alone learns a fault only through minutes close to its regression line and
            // only until the first warning: +6 warns after days, a +15 step stays out of the regression
            const bool slopeOnly = p.r && !p.f && !p.o;
            if (dR == 0.003f) CHECK(s.maxEff == 0, "%s: +3 mOhm below floor raised %u", name, s.maxEff);
            if (dR == 0.006f && slopeOnly) CHECK(s.maxEff == 1, "%s: +6 mOhm expected warn, got %u", name, s.maxEff);
            else if (dR == 0.006f) {
                CHECK(s.maxEff == 2, "%s: +6 mOhm expected derate, got %u", name, s.maxEff);
                CHECK(!s.g.tripped(), "%s: +6 mOhm tripped", name);
                const float want = std::sqrt(4.f / 0.006f);
                CHECK(std::fabs(s.minCap - want) < 0.15f * want, "%s: cap %.1f want ~%.1f", name, s.minCap, want);
            }
            if (dR == 0.015f && !slopeOnly) CHECK(s.maxEff == 3 && s.g.tripped(), "%s: +15 mOhm no trip", name);
            if (p.f && dR == 0.015f) CHECK(s.det(3) <= 8, "fast +15 trip after %.1f min", s.det(3));
            if (p.f && dR == 0.006f) CHECK(s.det(1) <= 8, "fast +6 warn after %.1f min", s.det(1));
        }
    }
}

// the converter-vs-BMS offset drifts by tens of mV over hours and days (fry 11-04..11-15: 120 mV
// range); against a frozen baseline offset that reads as excess resistance at high current
static void testOffsetDrift() {
    printf("offset drift at 30 A (learning over on day 7, drift from day 9)\n");
    struct D { const char *n; float perDay, max, wave; } cases[]{
        {"+30 mV over 3 days", 0.010f, 0.030f, 0},
        {"+120 mV over 3 days", 0.040f, 0.120f, 0},
        {"-120 mV over 3 days", -0.040f, -0.120f, 0},
        {"+-30 mV 6 h wave", 0, 0, 0.030f},
        {"+80 mV ramp + 30 mV wave", 0.040f, 0.080f, 0.030f},
    };
    for (auto &c: cases) {
        Sim s{2};
        s.ipk = 30.f;
        s.driftT = 9 * DAY;
        s.aRampPerDay = c.perDay;
        s.aRampMax = std::fabs(c.max);
        s.aWave = c.wave;
        s.runUntil(16 * DAY);
        // known-bad: the same drift graded against the frozen baseline offset at 30 A
        const float ex = std::fabs(c.max) + c.wave, raw = ex * 30;
        printf("  %-26s maxLvl=%u dayWarn=%d wf=%d  frozen-offset excess at 30 A: %.1f mOhm %.1f W\n", c.n, s.maxLvl,
               s.g.dayWarn(), s.g.wf(), ex / 30 * 1e3, raw);
        CHECK(s.maxLvl == 0 && !s.g.dayWarn(), "%s raised level %u day %d", c.n, s.maxLvl, s.g.dayWarn());
        CHECK(s.g.wf() == 0, "%s: wf %d", c.n, s.g.wf());
    }
    printf("  a fault on top of the drift is still found\n");
    for (float dR: {0.006f, 0.015f}) {
        Sim s{2};
        s.ipk = 30.f;
        s.driftT = 9 * DAY;
        s.aRampPerDay = 0.040f;
        s.aRampMax = 0.120f;
        s.aWave = 0.030f;
        s.runUntil(12 * DAY + 12 * 3600);
        s.faultT = s.t;
        s.dRc = dR;
        s.runUntil(s.t + DAY);
        char name[48];
        snprintf(name, sizeof name, "drift + %.0f mOhm", dR * 1e3);
        printRun(name, s);
        CHECK(s.det(1) <= 8, "%s: warn after %.1f min", name, s.det(1));
        if (dR > 0.01f) CHECK(s.g.tripped(), "%s: no trip", name);
        else CHECK(s.maxEff == 2, "%s: want derate, got %u", name, s.maxEff);
    }
}

// a resistance rise that stays below the power thresholds because the current is low
static void testDayWarn() {
    printf("current-independent day warning (12 A days)\n");
    uint8_t prev = 0;
    printf("  dR 0..20 mOhm from day 9: ");
    for (int mo = 0; mo <= 20; mo += 2) {
        Sim s{2};
        s.ipk = 12.f;
        s.runUntil(9 * DAY);
        s.faultT = s.t;
        s.dRc = (float) mo * 1e-3f;
        s.runUntil(s.t + 3 * DAY);
        const uint8_t l = std::max<uint8_t>(s.maxEff, s.g.dayWarn());
        printf("%u", l);
        CHECK(l >= prev, "+%d mOhm gave %u < %u", mo, l, prev);
        prev = l;
        if (mo <= 4) CHECK(!s.g.dayWarn() && s.maxEff == 0, "+%d mOhm at 12 A warned", mo);
        if (mo >= 10) {
            CHECK(s.g.dayWarn() && s.g.wf() >= 1, "+%d mOhm at 12 A: no day warning (wf %d)", mo, s.g.wf());
            // 12 A·12 A·dR stays below pDerate up to +27 mOhm: a day warning must never limit
            CHECK(!std::isfinite(s.g.ioutCap()) && !s.g.tripped(), "+%d mOhm: day warning acted", mo);
        }
    }
    printf("\n");
    {   // without BMS the day check falls back to Ro
        Sim s{2};
        s.ipk = 12.f;
        s.bms = s.rcFeed = false;
        s.runUntil(9 * DAY);
        s.faultT = s.t;
        s.dRc = 0.012f;
        s.runUntil(s.t + 3 * DAY);
        printf("  no BMS, +12 mOhm: dayWarn=%d maxEff=%u wf=%d\n", s.g.dayWarn(), s.maxEff, s.g.wf());
        CHECK(s.g.dayWarn(), "no day warning from Ro");
    }
    {   // repaired: the next judged day clears it
        Sim s{2};
        s.ipk = 12.f;
        s.runUntil(9 * DAY);
        s.faultT = s.t;
        s.dRc = 0.012f;
        s.runUntil(s.t + 2 * DAY);
        CHECK(s.g.dayWarn(), "precondition: day warning");
        s.dRc = 0;
        s.runUntil(s.t + 2 * DAY);
        CHECK(!s.g.dayWarn() && s.g.wf() == 0, "day warning did not clear after repair");
        WireGuard g2{};
        s.dRc = 0.012f;
        s.runUntil(s.t + 2 * DAY);
        CHECK(g2.load(&s.g.blob(), sizeof(WireGuard::Blob)) && g2.dayWarn(), "day warning not persisted");
    }
}

static void testTripLatch() {
    printf("trip latch holds until reset\n");
    Sim s{2};
    faultRun(s, 0.015f, true, true, true);
    CHECK(s.g.tripped(), "no trip");
    s.dRc = 0; // repaired, but no reset yet
    s.runUntil(s.t + 3 * DAY);
    CHECK(s.g.tripped() && s.g.effLevel() == 3, "trip released without reset");
    s.g.reset();
    CHECK(!s.g.tripped() && s.g.level() == 0 && s.g.learning() && !std::isfinite(s.g.ioutCap()),
          "reset did not clear latch/baseline");
    s.runUntil(s.t + 2 * DAY);
    CHECK(s.g.level() == 0, "level after reset %u", s.g.level());
}

static void testMonotonic() {
    printf("monotonicity over fault size (all paths, and fast only)\n");
    for (int pathSel = 0; pathSel < 2; ++pathSel) {
        uint8_t prevLvl = 0;
        float prevDet = INFINITY;
        printf("  %s: ", pathSel ? "fast" : "all ");
        for (int mo = 0; mo <= 30; mo += 1) {
            Sim s{2};
            faultRun(s, (float) mo * 1e-3f, true, !pathSel, !pathSel);
            printf("%u", s.maxLvl);
            CHECK(s.maxLvl >= prevLvl, "+%d mOhm gave level %u < %u", mo, s.maxLvl, prevLvl);
            if (s.maxLvl == prevLvl && s.maxLvl > 0) {
                const float d = s.det(s.maxLvl);
                CHECK(!(d > prevDet + 3), "+%d mOhm detected later (%.1f > %.1f min)", mo, d, prevDet);
                prevDet = d;
            } else prevDet = s.maxLvl ? s.det(s.maxLvl) : INFINITY;
            prevLvl = s.maxLvl;
        }
        printf("\n");
    }
}

static void testMissing() {
    printf("missing data\n");
    {   // no BMS, no sweeps: never verified
        Sim s{2};
        s.bms = s.rcFeed = s.roFeed = false;
        s.runUntil(8 * DAY);
        CHECK(s.g.wf() == WireGuard::WF_UNVERIFIED, "wf %d, want unverified", s.g.wf());
        CHECK(s.g.wf() == WireGuard::WF_UNVERIFIED, "no long-unverified alert after 8 days");
        CHECK(s.maxLvl == 0, "level without data");
        printf("  no data 8 days: wf=%d unverifiedLong=%d\n", s.g.wf(), s.g.unverifiedLong());
    }
    {   // data stops while derated: level and cap hold
        Sim s{2};
        s.runUntil(ON);
        s.faultT = ON;
        s.dRc = 0.006f;
        while (s.g.effLevel() < 2 && s.t < ON + DAY) s.step();
        CHECK(s.g.effLevel() == 2, "precondition: derated, got %u", s.g.effLevel());
        const float cap = s.g.ioutCap();
        s.dRc = 0;
        s.bms = s.rcFeed = s.roFeed = false;
        s.runUntil(s.t + 3 * DAY);
        CHECK(s.g.effLevel() == 2 && s.g.ioutCap() == cap, "missing data released derate (lvl %u cap %.1f)",
              s.g.effLevel(), s.g.ioutCap());
        printf("  derate then no data 3 days: lvl=%u cap=%.1f wf=%d\n", s.g.effLevel(), s.g.ioutCap(), s.g.wf());
    }
    {   // a night does not invalidate the day's verdict
        Sim s{2};
        s.runUntil(10 * DAY + 3 * 3600);
        CHECK(s.g.wf() == 0, "wf at night %d", s.g.wf());
    }
    {   // boot: no data yet is unverified, not ok; BMS and sweeps lost while charging: unverified again
        Sim s{2};
        s.step();
        CHECK(s.g.wf() == WireGuard::WF_UNVERIFIED, "boot wf %d", s.g.wf());
        s.runUntil(10 * DAY + 12 * 3600);
        CHECK(s.g.wf() == 0, "charging wf %d", s.g.wf());
        s.bms = s.rcFeed = s.roFeed = false;
        s.runUntil(s.t + 180);
        CHECK(!std::isfinite(s.g.pw()) && s.g.fastPath().fresh, "pw %.2f reported without new data", s.g.pw());
        s.runUntil(s.t + 2 * DAY); // the last sweep's Ro counts for STALE_RO charging minutes
        printf("  data lost while charging, 2 days: wf=%d pw=%.2f unverified days %u\n", s.g.wf(), s.g.pw(),
               s.g.blob().unvDays);
        CHECK(s.g.wf() == WireGuard::WF_UNVERIFIED && !s.g.unverifiedLong(), "data lost: wf %d, want unverified",
              s.g.wf());
    }
    {   // with data again after a fault is repaired, derate releases
        Sim s{2};
        faultRun(s, 0.006f, true, true, true);
        s.dRc = 0;
        s.w = WireResistance{};
        s.runUntil(s.t + 2 * DAY);
        printf("  repaired derate: lvl=%u\n", s.g.level());
        CHECK(s.g.level() == 0, "derate did not release after repair");
    }
}

static void testLearning() {
    printf("learning phase warns only\n");
    Sim s{2};
    s.runUntil(3 * DAY + 10 * 3600);
    s.faultT = s.t;
    s.dRc = 0.015f;
    s.runUntil(s.t + DAY);
    printf("  +15 on day 3: maxEff=%u lvl=%u learning=%d tripped=%d\n", s.maxEff, s.g.level(), s.g.learning(),
           s.g.tripped());
    CHECK(s.maxEff <= 1 && !s.g.tripped() && !std::isfinite(s.g.ioutCap()), "learning acted beyond warn");
    CHECK(s.maxEff == 1, "learning did not warn");
}

static void testRamp() {
    printf("slow ramp is not learned in\n");
    Sim s{2};
    s.runUntil(9 * DAY);
    s.faultT = s.t;
    s.dRcPerDay = 0.0005f; // 0.5 mΩ/day
    s.runUntil(s.t + 30 * DAY);
    printf("  0.5 mOhm/day x30: base Rc=%.2f mΩ maxEff=%u warn at +%.1f mΩ\n", s.g.baseRc() * 1e3, s.maxEff,
           s.firstEff[1] == UINT32_MAX ? NAN : (s.firstEff[1] - s.faultT) / (float) DAY * 0.5f);
    CHECK(s.g.baseRc() < 0.014f + 0.0035f, "baseline crept to %.2f mΩ", s.g.baseRc() * 1e3);
    CHECK(s.maxEff >= 1, "ramp to +15 mOhm never warned");
}

static void testRb() {
    printf("shared-path (Rb) rise\n");
    for (int withFast = 0; withFast < 2; ++withFast) {
        Sim s{2};
        s.roFeed = false;
        s.loadAmp = 15.f; // varying load makes Rb identifiable
        s.runUntil(ON);
        s.g.paths = withFast ? 7 : 0;
        s.faultT = ON;
        s.dRb = 0.010f;
        s.runUntil(ON + 2 * DAY);
        printf("  +10 mOhm Rb, own paths %s: rbWarn=%d maxEff=%u tripped=%d wf=%d\n", withFast ? "graded" : "off   ",
               s.rbSeen, s.maxEff, s.g.tripped(), s.g.wf());
        // with own paths graded the excess can't be told apart from an own-wire fault when the
        // output and battery currents move together: they act first (documented limitation)
        if (!withFast) {
            CHECK(s.rbSeen, "Rb rise not warned");
            CHECK(s.maxEff == 0 && !s.g.tripped(), "Rb rise raised own level %u", s.maxEff);
        }
    }
}

static void testModes() {
    printf("wire_protect modes, +15 mOhm all paths\n");
    for (uint8_t m = 0; m <= 2; ++m) {
        Sim s{m};
        faultRun(s, 0.015f, true, true, true);
        printf("  mode %u: maxEff=%u level=%u tripped=%d cap=%.1f wf=%d\n", m, s.maxEff, s.g.level(), s.g.tripped(),
               s.g.ioutCap(), s.g.wf());
        if (m == 0) CHECK(s.maxEff == 0 && s.g.level() == 0 && !s.g.tripped() && s.g.dayCount() == 0, "mode 0 acted");
        if (m == 1) CHECK(s.maxEff == 1 && s.g.level() == 3 && !s.g.tripped() && !std::isfinite(s.g.ioutCap()),
                          "mode 1 acted beyond warn");
        if (m == 2) CHECK(s.maxEff == 3 && s.g.tripped(), "mode 2 did not trip");
    }
}

static void testKnownBad() {
    printf("known-bad calibration: the guard must fire\n");
    // a fault whose excess power exceeds the trip threshold at a modest current
    Sim s{2};
    s.ipk = 20.f;
    s.runUntil(ON);
    s.faultT = ON;
    s.dRc = 0.025f; // 0.025·20² = 10 W
    s.runUntil(ON + DAY);
    printRun("+25 mOhm at 20 A", s);
    CHECK(s.g.tripped(), "known-bad fault did not trip");
    // and a disabled persistence (fault on only 3 min) must not fire
    Sim b{2};
    b.runUntil(ON);
    b.faultT = ON;
    b.dRc = 0.015f;
    b.runUntil(ON + 180);
    b.dRc = 0;
    b.runUntil(ON + DAY);
    printRun("+15 mOhm for 3 min", b);
    CHECK(b.maxEff == 0, "3-minute blip raised level %u", b.maxEff);
}

// sweeps every 2 min (fry 11-14: ~30/h), Ro path only
static void testRoNoise() {
    printf("Ro path: sweep bursts, median + spacing\n");
    struct C { const char *n; uint8_t mask; float p; } cases[]{
        {"2 of every 5 sweeps +12 mOhm", 0b00011, 0},
        {"20 % random +12 mOhm", 0, 0.2f},
    };
    for (auto &c: cases) {
        Sim s{2};
        s.sweepS = 120;
        s.roOut = 0.012f;
        s.runUntil(ON);
        s.g.paths = 4;
        s.outMask = c.mask;
        s.outP = c.p;
        s.runUntil(ON + 3 * DAY);
        printf("  %-30s maxLvl=%u Ro fresh=%d\n", c.n, s.maxLvl, s.g.roPath().fresh);
        CHECK(s.maxLvl == 0, "%s raised level %u", c.n, s.maxLvl);
    }
    for (int noisy = 0; noisy < 2; ++noisy) { // +15 mOhm at 15 A: 3.4 W
        Sim s{2};
        s.sweepS = 120;
        s.ipk = 15.f;
        s.roOut = -0.012f; // outliers low: they must not hide the step either
        s.runUntil(ON);
        s.g.paths = 4;
        s.outP = noisy ? 0.2f : 0;
        s.faultT = ON;
        s.dRc = 0.015f;
        s.runUntil(ON + DAY);
        char name[48];
        snprintf(name, sizeof name, "+15 mOhm at 15 A%s", noisy ? ", 20 % low" : "");
        printRun(name, s);
        CHECK(s.maxEff == 1, "%s: want warn, got %u", name, s.maxEff);
        CHECK(s.det(1) >= 2 * WireGuard::RO_GAP && s.det(1) <= 90, "%s: warn after %.1f min", name, s.det(1));
    }
    {   // repaired: the Ro path alone releases once all sweeps of its window are low
        Sim s{2};
        s.sweepS = 120;
        faultRun(s, 0.006f, false, false, true);
        CHECK(s.maxLvl == 2 && s.g.level() >= 1, "Ro +6 mOhm: max %u now %u", s.maxLvl, s.g.level());
        s.dRc = 0;
        s.runUntil(s.t + DAY);
        printf("  Ro +6 mOhm repaired: lvl=%u\n", s.g.level());
        CHECK(s.g.level() == 0, "Ro-only derate did not release after repair");
    }
    printf("  monotonic, Ro only, 2-min sweeps, 0..100 mOhm: ");
    uint8_t prev = 0;
    for (int mo: {0, 2, 4, 6, 8, 10, 15, 20, 30, 50, 100}) {
        Sim s{2};
        s.sweepS = 120;
        faultRun(s, (float) mo * 1e-3f, false, false, true);
        printf("%u", s.maxLvl);
        CHECK(s.maxLvl >= prev, "+%d mOhm gave %u < %u", mo, s.maxLvl, prev);
        if (mo == 100) CHECK(s.g.tripped() && s.det(3) <= 90, "+100 mOhm: trip after %.1f min", s.det(3));
        prev = s.maxLvl;
    }
    printf("\n");
    {   // fewer than RO_MED_MIN sweeps: no Ro value, unverified rather than ok
        Sim s{2};
        s.bms = s.rcFeed = false;
        s.sweepS = 120;
        s.runUntil(3 * DAY + 8 * 3600);
        const auto b = s.g.blob(); // reboot with a baseline
        s.g.reset();
        CHECK(s.g.load(&b, sizeof b) && s.g.roRef(), "reload");
        for (uint32_t k = 0; k < WireGuard::RO_MED_MIN; ++k) {
            const uint32_t n = s.nSweep;
            while (s.nSweep == n) s.step();
            s.runUntil(s.t + 60);
            if (k + 1 < WireGuard::RO_MED_MIN)
                CHECK(!s.g.roPath().fresh && s.g.wf() == WireGuard::WF_UNVERIFIED, "%u sweeps: Ro fresh %d wf %d",
                      k + 1, s.g.roPath().fresh, s.g.wf());
        }
        CHECK(s.g.roPath().fresh && s.g.wf() == 0, "%u sweeps: Ro fresh %d wf %d", WireGuard::RO_MED_MIN,
              s.g.roPath().fresh, s.g.wf());
    }
}

static void testPersist() {
    printf("blob round trip\n");
    Sim s{2};
    s.runUntil(10 * DAY);
    WireGuard g2{};
    CHECK(g2.load(&s.g.blob(), sizeof(WireGuard::Blob)), "load rejected");
    CHECK(g2.baseRc() == s.g.baseRc() && g2.baseRo() == s.g.baseRo() && !g2.learning(), "restored baseline differs");
    CHECK(!g2.tripped() && g2.level() == 0, "latch restored");
    auto bad = s.g.blob();
    bad.ver = 99;
    CHECK(!g2.load(&bad, sizeof bad), "bad version accepted");
    CHECK(!g2.load(&bad, sizeof bad - 1), "bad size accepted");
    printf("  sizeof(WireGuard)=%zu sizeof(Blob)=%zu\n", sizeof(WireGuard), sizeof(WireGuard::Blob));
}


// field case: one dim partial day left Rc 3.13 mOhm (true ~15) as a 1-day baseline; later correct
// days were rejected against it, so learning never progressed and the slope path warned (now: not
// graded below BASE_MIN_DAYS)
static void testBadFirstDay() {
    printf("learning: a wrong first-day baseline is outvoted\n");
    {
        Sim s{1};
        WireGuard::Blob b{WireGuard::VER, 1, 1, 1, 1, 1, 0, 0, WireGuard::NONE, WireGuard::NONE, {313}, {257}, {500}, {2016}};
        CHECK(s.g.load(&b, sizeof b) && s.g.learning(), "load fry-like blob");
        s.runUntil(DAY);
        const uint8_t first = s.maxLvl;
        s.maxLvl = 0;
        s.runUntil(2 * DAY);
        const float rc2 = s.g.baseRc();
        s.runUntil(3 * DAY); // a held level releases in the morning of day 2
        s.maxLvl = 0;
        s.runUntil(5 * DAY);
        printf("  base Rc 3.13 -> %.2f (after 2 good days) -> %.2f mOhm, days %u, level day 1 %u, days 3-5 %u, learning %d\n",
               rc2 * 1e3, s.g.baseRc() * 1e3, s.g.dayCount(), first, s.maxLvl, s.g.learning());
        CHECK(first == 0, "a 1-day baseline was graded on day 1: level %u", first);
        CHECK(std::fabs(s.g.baseRc() - 0.014f) < 0.0015f, "baseline did not converge: %.2f mOhm", s.g.baseRc() * 1e3);
        CHECK(s.maxLvl == 0, "false warning stays after convergence: level %u", s.maxLvl);
        CHECK(s.g.dayCount() == 6 && s.g.learning() && !s.g.armed(), "days %u learning %d", s.g.dayCount(), s.g.learning());
    }
    {   // a dim first day (peak 7 A) doesn't enter at all; the next normal days do
        Sim s{2};
        s.ipk = 7.f;
        s.runUntil(DAY);
        printf("  dim first day (7 A): days %u, Rc days %u\n", s.g.dayCount(), s.g.blob().nRc);
        CHECK(s.g.dayCount() == 0 && s.g.blob().nRc == 0 && s.g.learning(), "dim day entered the baseline");
        s.ipk = 28.f;
        s.runUntil(3 * DAY);
        CHECK(s.g.dayCount() == 2 && std::fabs(s.g.baseRc() - 0.014f) < 0.0015f, "normal days: %u, Rc %.2f",
              s.g.dayCount(), s.g.baseRc() * 1e3);
    }
    {   // too little data: a few minutes above I_MIN, no current spread
        Sim s{2};
        s.ipk = 5.4f;
        s.runUntil(4 * DAY);
        printf("  5.4 A peak, 4 days: days %u Rc days %u learning %d wf %d\n", s.g.dayCount(), s.g.blob().nRc,
               s.g.learning(), s.g.wf());
        CHECK(s.g.dayCount() == 0 && s.g.blob().nRc == 0 && s.g.learning(), "thin days entered the baseline");
    }
    printf("  counted days over peak current 4..28 A (3 days each): ");
    int prev = 0;
    for (int ipk = 4; ipk <= 28; ipk += 2) {
        Sim s{2};
        s.ipk = (float) ipk;
        s.runUntil(3 * DAY);
        printf("%u", s.g.dayCount());
        CHECK(s.g.dayCount() >= prev, "peak %d A counted %u < %d days", ipk, s.g.dayCount(), prev);
        prev = s.g.dayCount();
    }
    printf("\n");
}

// after learning a fault still stays out of the baseline and warns
static void testFaultNotLearned() {
    printf("fault after learning: rejected from the baseline\n");
    Sim s{1};
    s.runUntil(ON);
    const float rc0 = s.g.baseRc(), ro0 = s.g.baseRo();
    s.faultT = ON;
    s.dRc = 0.010f;
    s.runUntil(ON + 5 * DAY);
    printf("  +10 mOhm for 5 days: base Rc %.2f -> %.2f, Ro %.2f -> %.2f mOhm, level %u wf %d\n", rc0 * 1e3,
           s.g.baseRc() * 1e3, ro0 * 1e3, s.g.baseRo() * 1e3, s.g.level(), s.g.wf());
    CHECK(std::fabs(s.g.baseRc() - rc0) < 0.001f && std::fabs(s.g.baseRo() - ro0) < 0.001f, "fault learned in");
    CHECK(s.g.level() >= 1 && s.g.wf() == 1, "fault not warned: level %u wf %d", s.g.level(), s.g.wf());
}

// the firmware restarts with the baseline from NVS and, after a software reset, the open day from RTC RAM
static void testReboots() {
    printf("restarts during learning (10 days)\n");
    struct C { const char *n; uint32_t every, first; bool rtc; uint8_t minDays; } cases[]{
        {"every 3 h, RTC kept", 3 * 3600, 3600, true, 9},
        {"every 2.5 min 9-11 h, RTC kept", 150, 0, true, 9},
        {"10:00 and 13:00, RTC lost", 0, 0, false, 9},
        {"every 3 h, RTC lost", 3 * 3600, 3600, false, 0},
    };
    for (auto &c: cases) {
        Sim s{2};
        s.persist = true;
        uint32_t next = c.first;
        while (s.t < 10 * DAY) {
            s.step();
            const uint32_t h = s.t % DAY;
            bool boot;
            if (c.every == 150) boot = h >= 9 * 3600 && h < 11 * 3600 && h % 150 < TICK;
            else if (c.every) boot = s.t >= next;
            else boot = h == 10 * 3600 || h == 13 * 3600;
            if (boot) {
                if (c.every && c.every != 150) next += c.every;
                s.reboot(c.rtc);
            }
        }
        printf("  %-32s days %2u learning %d base Rc %.2f Ro %.2f mOhm maxLvl %u\n", c.n, s.g.dayCount(),
               s.g.learning(), s.g.baseRc() * 1e3, s.g.baseRo() * 1e3, s.maxLvl);
        if (c.minDays) {
            CHECK(s.g.dayCount() >= c.minDays && s.g.dayCount() <= 10, "%s: %u days", c.n, s.g.dayCount());
            CHECK(std::fabs(s.g.baseRc() - 0.014f) < 0.0015f, "%s: Rc %.2f", c.n, s.g.baseRc() * 1e3);
            CHECK(s.maxLvl == 0, "%s: level %u", c.n, s.maxLvl);
        }
    }
    {   // RTC copy integrity
        Sim s{2};
        s.runUntil(DAY + 12 * 3600);
        WireGuard::DayRtc r{};
        s.g.saveDay(r);
        WireGuard g2{};
        CHECK(g2.restoreDay(r) && g2.daySamples() == s.g.daySamples() && g2.daySamples() > 0, "round trip");
        auto bad = r;
        bad.d.rc[0] ^= 1;
        CHECK(!WireGuard{}.restoreDay(bad), "corrupt day accepted");
        bad = r;
        ++bad.magic;
        CHECK(!WireGuard{}.restoreDay(bad), "foreign magic accepted");
        WireGuard::DayRtc z{};
        CHECK(!WireGuard{}.restoreDay(z), "zeroed RAM accepted");
    }
    {   // the restored day keeps its age across the uptime reset and closes as without the restart
        Sim a{2}, b{2};
        a.persist = b.persist = true;
        a.runUntil(DAY + 12 * 3600);
        b.runUntil(DAY + 12 * 3600);
        b.reboot(true);
        a.runUntil(3 * DAY);
        b.runUntil(3 * DAY);
        printf("  reboot at noon, RTC kept: days %u vs %u without, Rc %.3f vs %.3f mOhm\n", b.g.dayCount(),
               a.g.dayCount(), b.g.baseRc() * 1e3, a.g.baseRc() * 1e3);
        CHECK(a.g.dayCount() == b.g.dayCount() && std::fabs(a.g.baseRc() - b.g.baseRc()) < 3e-4f, "restored day differs");
    }
}

// a baseline of 1 stored day, 5 mOhm low: not graded until BASE_MIN_DAYS days; a fault after that is
static void testBaseMinDays() {
    printf("baseline below %u days is not graded\n", WireGuard::BASE_MIN_DAYS);
    using B = WireGuard::Blob;
    // Ro true ~23 mOhm, Rc ~14 mOhm (Sim)
    const B roLow{WireGuard::VER, 0, 0, 1, 1, 1, 0, 0, WireGuard::NONE, WireGuard::NONE, {}, {}, {}, {1800}};
    const B rcLow{WireGuard::VER, 1, 1, 0, 0, 1, 0, 0, WireGuard::NONE, WireGuard::NONE, {900}, {200}, {300}, {}};
    struct C { const char *n; const B &b; uint8_t paths; float rMax; } cases[]{
        {"Ro 1 day 18 mOhm, Ro path", roLow, 4, 0},
        {"Rc 1 day 9 mOhm, fast+Rc", rcLow, 3, 0},
    };
    for (auto &c: cases) {
        Sim s{2};
        s.sweepS = 120;
        s.g.paths = c.paths;
        CHECK(s.g.load(&c.b, sizeof c.b), "%s: load", c.n);
        s.runUntil(15 * 3600);
        const int8_t wf0 = s.g.wf();
        const bool ref0 = c.paths == 4 ? s.g.roRef() : s.g.rcRef();
        s.runUntil(3 * DAY);
        const bool ref = c.paths == 4 ? s.g.roRef() : s.g.rcRef();
        const uint8_t clean = s.maxLvl;
        s.faultT = s.t + 12 * 3600;
        s.dRc = 0.010f;
        s.runUntil(s.faultT + DAY);
        printf("  %-26s day 0: ref %d wf %d; days 0-2 maxLvl %u, ref %d; +10 mOhm day 3: warn after %.1f min\n", c.n,
               ref0, wf0, clean, ref, s.det(1));
        CHECK(!ref0 && wf0 == WireGuard::WF_UNVERIFIED, "%s: graded on day 0 (wf %d)", c.n, wf0);
        CHECK(clean == 0, "%s: 1-day baseline raised level %u", c.n, clean);
        CHECK(ref, "%s: no reference after 3 days", c.n);
        CHECK(s.det(1) <= 90, "%s: fault after 3 days not warned (%.1f min)", c.n, s.det(1));
    }
    {   // a configured rMax is a reference from the start; the 1-day Rc below it is not used
        Sim s{2};
        s.g.conf.rMax = 0.016f;
        s.g.paths = 3;
        CHECK(s.g.load(&rcLow, sizeof rcLow) && s.g.rcRef() && !s.g.rbRef(), "rMax: load");
        s.runUntil(DAY);
        const uint8_t clean = s.maxLvl;
        s.faultT = s.t + 12 * 3600;
        s.dRc = 0.010f;
        s.runUntil(s.faultT + 12 * 3600);
        printf("  rMax 16 mOhm, Rc 1 day 9 mOhm: day 0 maxLvl %u; +10 mOhm day 1: warn after %.1f min\n", clean, s.det(1));
        CHECK(clean == 0, "rMax: clean day raised level %u", clean);
        CHECK(s.det(1) <= 15, "rMax: fault on day 1 not warned (%.1f min)", s.det(1));
    }
}

// the offset regression survives a software reset in RTC RAM; anything odd there starts it cold
static void testFitRestore() {
    printf("offset regression across a restart\n");
    constexpr uint32_t T = 10 * DAY + 12 * 3600;
    auto firstFast = [](Sim &s) {
        const uint32_t t0 = s.t;
        while (!s.g.fastPath().fresh && s.t < t0 + 3600) s.step();
        return (float) (s.t - t0) / 60.f;
    };
    Sim a{2}, b{2}, c{2};
    for (auto *s: {&a, &b, &c}) { s->persist = true; s->runUntil(T); }
    b.reboot(true);
    c.reboot(true, false);
    const float wb = firstFast(b), wc = firstFast(c);
    a.runUntil(b.t);
    printf("  first fast-path minute after restart: %.1f min restored, %.1f min cold; slope %.3f vs %.3f mOhm\n", wb, wc,
           b.g.slope() * 1e3, a.g.slope() * 1e3);
    CHECK(wb <= 2 && wc >= 5, "warm %.1f cold %.1f min", wb, wc);
    CHECK(std::fabs(a.g.slope() - b.g.slope()) < 5e-5f, "restored slope %.3f vs %.3f mOhm", b.g.slope() * 1e3,
          a.g.slope() * 1e3);

    WireGuard::FitRtc r{};
    a.g.saveFit(r);
    WireGuard g2{};
    CHECK(g2.restoreFit(r), "round trip rejected");
    auto bad = r;
    bad.rg[3] *= 1.001f;
    CHECK(!WireGuard{}.restoreFit(bad), "corrupt sums accepted");
    WireGuard::FitRtc z{};
    CHECK(!WireGuard{}.restoreFit(z), "zeroed RAM accepted");
    auto sign = [](WireGuard::FitRtc &f) { // the guard's FNV-1a over the sums, seeded with FIT_MAGIC
        uint32_t h = WireGuard::FIT_MAGIC, x;
        for (float v: f.rg) { memcpy(&x, &v, 4); h = (h ^ x) * 16777619u; }
        f.sum = h;
    };
    bad = r;
    sign(bad);
    CHECK(WireGuard{}.restoreFit(bad), "re-signed copy rejected");
    for (float x: {NAN, INFINITY, -1.f}) {
        bad = r;
        bad.rg[x < 0 ? 0 : 4] = x;
        sign(bad);
        CHECK(!WireGuard{}.restoreFit(bad), "signed sums with %g accepted", x);
    }

    // a fault shortly before or after the restart is still found as fast as without one
    for (int32_t dt: {-120, 120}) {
        Sim f{2};
        f.persist = true;
        f.runUntil(T - 3600);
        f.faultT = f.t + 3600 + dt;
        f.dRc = 0.006f;
        f.runUntil(T);
        f.reboot(true);
        f.runUntil(T + 3 * 3600);
        printf("  +6 mOhm %+d s from the restart: warn after %.1f min\n", dt, f.det(1));
        CHECK(f.det(1) <= 8, "fault %+d s from restart: warn after %.1f min", dt, f.det(1));
    }
}

int main() {
    testClean();
    testSteps();
    testOffsetDrift();
    testDayWarn();
    testTripLatch();
    testMonotonic();
    testMissing();
    testLearning();
    testRamp();
    testRb();
    testModes();
    testKnownBad();
    testRoNoise();
    testPersist();
    testBadFirstDay();
    testFaultNotLearned();
    testReboots();
    testBaseMinDays();
    testFitRestore();
    printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
    return g_fail != 0;
}
