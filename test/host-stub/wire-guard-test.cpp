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
    uint32_t t = 0, roStamp = 0;
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
                if (rcFeed) w.add(dv, i, ib, t);
                g.frame(dv, i, ib);
            }
            prevI = i;
        } else prevI = NAN;

        if (roFeed && running && i >= 3 && t % 1800 == 0) {
            ro = rcDay + fr + rb + fb + rbat + 0.001f * std::sin((float) t / 9000.f) + nro(rng);
            roStamp = t;
        }
        const float rcv = rcFeed ? w.own(t) : NAN;
        g.tick(t, i, running, rcv, rcFeed ? w.offset(t) : NAN, rcFeed ? w.shared(t) : NAN,
               roFeed && t - roStamp < 6 * 3600 ? ro : NAN, roFeed ? roStamp : 0);

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
    testPersist();
    printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
    return g_fail != 0;
}
