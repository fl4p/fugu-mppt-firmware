// Host test for CoulombCounter (src/etc/coulomb_counter.h): two chargers on one BMS.
//
//   clang++ -std=gnu++20 -fexceptions -I test/host-stub -I src test/host-stub/coulomb-test.cpp \
//       -o /tmp/coulomb-test && /tmp/coulomb-test
//
#include <cassert>
#include <cmath>
#include <cstdio>
#include <utility>
#include "mock.h"
#include "etc/coulomb_counter.h"

time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_fail; } } while (0)

static constexpr float CBAT = 280.f, DOD = 0.15f, THR = DOD * CBAT; // 42 Ah
static constexpr time_us FRAME_US = 12'000'000ULL;

// One charger: its counter plus the release decision of Li_ChgTerminationCondition (depth > thr).
struct Unit {
    CoulombCounter cc;
    bool terminated = false;
    float ibatGain; // per-unit ibat integration error (frame timing, dropped frames)
    explicit Unit(float g) : ibatGain(g) {}
    void cellFrame(time_us t, float ibat, bool bms) {
        cc.updateBatCurrent(ibat * ibatGain, t);
        if (bms) cc.sampleOnCellFrame(coarseTicks(t));
        float depth = cc.updateDepth();
        if (terminated && depth > THR) terminated = false;
    }
};

// Pack discharges at -4 A until the first unit releases, then that unit charges at +10 A net.
// Returns the frame count the second unit needed to release, or -1 if it never did.
static int run(bool bms, Unit &a, Unit &b) {
    float charge = 200.f;
    time_us t = 1'000'000ULL;
    auto frame = [&](float ibat) {
        charge += ibat * (float) FRAME_US / 3600e6f;
        a.cc.setBmsCharge(charge, coarseTicks(t));
        b.cc.setBmsCharge(charge, coarseTicks(t));
        a.cellFrame(t, ibat, bms);
        b.cellFrame(t, ibat, bms);
        t += FRAME_US;
    };
    frame(0.f);
    a.cc.markFull(); b.cc.markFull();
    a.terminated = b.terminated = true;
    frame(0.f);
    int firstRelease = -1;
    for (int k = 0; k < 20000; ++k) {
        bool anyOn = !a.terminated || !b.terminated;
        frame(anyOn ? 10.f : -4.f);
        if (anyOn && firstRelease < 0) firstRelease = k;
        if (!a.terminated && !b.terminated) return firstRelease < 0 ? 0 : k - firstRelease;
        if (firstRelease >= 0 && k - firstRelease > 3000) return -1;
    }
    return -1;
}

static void test_integrators_deadlock() {
    Unit a{1.01f}, b{0.99f};
    int lag = run(false, a, b);
    printf("integrator mode: second unit released after %d frames\n", lag);
    CHECK(lag == -1, "baseline should reproduce the deadlock (one unit stays terminated)");
}

static void test_bms_releases_together() {
    Unit a{1.01f}, b{0.99f};
    int lag = run(true, a, b);
    printf("bms mode: second unit released after %d frames\n", lag);
    CHECK(lag == 0 || lag == 1, "both units must release within one frame");
    CHECK(a.cc.bmsSource() && b.cc.bmsSource(), "bms source active");
}

static void test_signed_not_clamped() {
    CoulombCounter cc;
    time_us t = 1'000'000ULL;
    float q = 200.f;
    auto sample = [&](float ah) { cc.setBmsCharge(ah, coarseTicks(t)); cc.sampleOnCellFrame(coarseTicks(t)); t += FRAME_US; };
    auto at = [&](float ah) { // no ibat: walk in steps under the 0.5 Ah continuity tolerance
        while (std::fabs(ah - q) > 1e-4f) { q += fmaxf(-0.4f, fminf(ah - q, 0.4f)); sample(q); }
    };
    sample(q); cc.markFull();
    at(205.f); // sibling charged past the full point
    CHECK(std::fabs(cc.deficit() + 5.f) < 1e-3f, "deficit goes negative, no clamp");
    CHECK(std::fabs(cc.updateDepth()) < 1e-3f, "depth stays at 0 while above full");
    at(190.f);
    CHECK(std::fabs(cc.updateDepth() - 10.f) < 1e-3f, "depth measured from the full point, not the peak");
    at(198.f);
    CHECK(std::fabs(cc.updateDepth() - 10.f) < 1e-3f, "depth keeps the deepest point");
    CHECK(std::fabs(cc.deficit() - 2.f) < 1e-3f, "deficit follows the charge");
}

static void test_stale_bms_falls_back() {
    CoulombCounter cc;
    time_us t = 1'000'000ULL;
    cc.setBmsCharge(200.f, coarseTicks(t));
    cc.sampleOnCellFrame(coarseTicks(t));
    cc.updateBatCurrent(0.f, t);
    cc.markFull();
    CHECK(cc.bmsSource(), "fresh bms charge is the source");
    for (int i = 1; i <= 20; ++i) { // 240 s of -6 A, no BMS charge frames
        t += FRAME_US;
        cc.updateBatCurrent(-6.f, t);
        cc.sampleOnCellFrame(coarseTicks(t));
    }
    CHECK(!cc.bmsSource(), "stale bms charge is dropped");
    float d = cc.updateDepth();
    CHECK(std::fabs(d - 6.f * 228.f / 3600.f) < 0.005f, "integrator deficit used while bms stale"); // reset drops one interval
}

static void test_no_charge_topic_is_integrator() {
    CoulombCounter cc;
    time_us t = 1'000'000ULL;
    cc.updateBatCurrent(0.f, t);
    cc.markFull();
    for (int i = 1; i <= 10; ++i) { t += FRAME_US; cc.updateBatCurrent(-3.f, t); cc.sampleOnCellFrame(coarseTicks(t)); }
    CHECK(!cc.bmsSource(), "no bms source without frames");
    CHECK(std::fabs(cc.updateDepth() - cc.ahSinceFull()) < 1e-6f, "depth equals integrator");
}

// BMS frames every 4 s with the charge following ibat (both sides of the BMS agree).
struct Feed {
    CoulombCounter cc;
    time_us t = 1'000'000ULL;
    float q; // [Ah] true remaining charge
    float scale = 0.f; // raw = q + scale: the BMS's re-seed shifts this
    explicit Feed(float q0) : q(q0) {}
    void frame(float ibat, bool charge = true) {
        t += 4'000'000ULL;
        q += ibat * 4.f / 3600.f;
        cc.updateBatCurrent(ibat, t);
        if (charge) cc.setBmsCharge(q + scale, coarseTicks(t));
        cc.sampleOnCellFrame(coarseTicks(t));
        cc.updateDepth();
    }
    void gap(time_us us, float ibat) { t += us; q += ibat * (float) us / 3600e6f; }
    void reseed(float raw) { scale = raw - q; frame(0.f); }
};

// 11-23 12:58: 276.8 -> 257.02 at -3 A, 69 s without data, then the BMS re-seeds to 0.9 x 245 Ah.
static void test_reseed_after_gap() {
    Feed f{276.8f};
    f.frame(0.f); f.cc.markFull(); f.frame(0.f);
    while (f.q > 257.02f) f.frame(-3.f);
    float d0 = f.cc.deficit();
    f.gap(69'000'000ULL, -3.f);
    f.scale = 220.5f - f.q;
    f.frame(-3.f);
    float want = d0 + 3.f * 73.f / 3600.f;
    printf("re-seed after gap: deficit %.2f -> %.2f Ah (true %.2f), depth %.2f\n", d0, f.cc.deficit(), 276.8f - f.q, f.cc.depth());
    CHECK(std::fabs(f.cc.deficit() - want) < 0.05f, "re-seed absorbed: deficit stays ~19.8 + gap discharge");
    CHECK(f.cc.depth() < want + 0.05f, "depth not raised by the re-seed");
    for (int i = 0; i < 450; ++i) f.frame(+20.f); // 30 min charging, +10 Ah
    CHECK(std::fabs(f.cc.deficit() - (276.8f - f.q)) < 0.3f, "smooth rise afterwards follows the charge");
    CHECK(f.cc.depth() < want + 0.05f, "depth unchanged while charging");

    // reboot: the saved ref is in the BMS's raw scale, the new counter starts without an offset
    float ref = f.cc.bmsRef();
    CoulombCounter n;
    n.restoreRef(ref);
    n.setBmsCharge(f.q + f.scale, coarseTicks(f.t));
    n.sampleOnCellFrame(coarseTicks(f.t));
    printf("reboot: deficit %.2f vs %.2f before\n", n.deficit(), f.cc.deficit());
    CHECK(std::fabs(n.deficit() - f.cc.deficit()) < 0.05f, "restored ref keeps the deficit across a reboot");
}

// a re-seed between two 4 s frames, no gap (11-23 12:11, 14:09)
static void test_reseed_no_gap() {
    Feed f{279.32f};
    f.frame(0.f); f.cc.markFull();
    for (int i = 0; i < 10; ++i) f.frame(-5.f);
    float d0 = f.cc.deficit();
    f.reseed(252.01f);
    CHECK(std::fabs(f.cc.deficit() - d0) < 0.05f, "step absorbed in full");
    f.reseed(f.q + f.scale + 30.f);
    CHECK(std::fabs(f.cc.deficit() - d0) < 0.05f, "upward step absorbed");
}

// 11-22: seven re-seeds 253.4 -> 252.0 with charging in between
static void test_repeated_reseeds_no_accumulation() {
    Feed f{253.4f};
    f.frame(0.f); f.cc.markFull(); f.frame(0.f);
    float q0 = f.q;
    for (int k = 0; k < 7; ++k) {
        while (f.q + f.scale < 253.4f) f.frame(+10.f);
        f.reseed(252.0f);
        for (int i = 0; i < 30; ++i) f.frame(-2.f);
    }
    float truth = q0 - f.q;
    printf("7 re-seeds: deficit %.3f Ah, true %.3f Ah, raw shift %.2f Ah\n", f.cc.deficit(), truth, f.scale);
    CHECK(std::fabs(f.cc.deficit() - truth) < 0.1f, "no accumulated error after 7 re-seeds");
}

// BMS SOC correction without current: -1 Ah over 20 min must pass unchanged
static void test_slow_drift_passes() {
    Feed f{250.f};
    f.frame(0.f); f.cc.markFull(); f.frame(0.f);
    for (int i = 0; i < 300; ++i) { f.scale -= 1.f / 300.f; f.frame(0.f); }
    printf("slow drift: deficit %.3f depth %.3f\n", f.cc.deficit(), f.cc.depth());
    CHECK(std::fabs(f.cc.deficit() - 1.f) < 0.01f, "slow drift passes");
    CHECK(std::fabs(f.cc.depth() - 1.f) < 0.01f, "slow drift deepens depth");
}

// A gap in both streams: any step up to BAT_I_MAX_A x gap is real discharge and passes whole.
static void test_real_discharge_in_long_gap() {
    for (auto [gapS, amps]: {std::pair{600.f, 30.f}, std::pair{7200.f, 30.f}}) {
        Feed f{250.f};
        f.frame(0.f); f.cc.markFull();
        for (int i = 0; i < 10; ++i) f.frame(-3.f);
        float d0 = f.cc.deficit(), truth0 = 250.f - f.q;
        f.gap((time_us) (gapS * 1e6f), -amps);
        f.frame(-3.f);
        float got = f.cc.deficit() - d0, want = 250.f - f.q - truth0;
        printf("gap %.0f s at %.0f A: true %.2f Ah, credited %.2f Ah, depth %.2f\n", gapS, amps, want, got, f.cc.depth());
        CHECK(std::fabs(got - want) < 0.05f, "real discharge in a gap credited in full");
        CHECK(std::fabs(f.cc.depth() - (250.f - f.q)) < 0.05f, "depth follows it");
        float d1 = f.cc.deficit();
        for (int i = 0; i < 90; ++i) f.frame(-30.f);
        CHECK(std::fabs(f.cc.deficit() - d1 - 3.f) < 0.05f, "discharge after the gap counted in full");
    }
}

// one sibling misses 2 h of data (30 A discharge), the other sees it all: both end on the same deficit
static void test_sibling_misses_gap() {
    CoulombCounter a, b;
    time_us t = 60'000'000ULL;
    float q = 250.f;
    auto both = [&](float ibat) {
        for (auto c: {&a, &b}) {
            c->updateBatCurrent(ibat, t); c->setBmsCharge(q, coarseTicks(t));
            c->sampleOnCellFrame(coarseTicks(t)); c->updateDepth();
        }
    };
    both(0.f); a.markFull(); b.markFull();
    t += 4'000'000ULL; both(0.f);
    for (int i = 0; i < 1800; ++i) {
        t += 4'000'000ULL; q -= 30.f * 4.f / 3600.f;
        a.updateBatCurrent(-30.f, t);
        if (i % 3 == 2) a.setBmsCharge(q, coarseTicks(t));
        a.sampleOnCellFrame(coarseTicks(t)); a.updateDepth();
    }
    t += 4'000'000ULL; both(0.f);
    printf("siblings after a 2 h outage of one: %.2f / %.2f Ah (true %.2f)\n", a.deficit(), b.deficit(), 250.f - q);
    CHECK(std::fabs(a.deficit() - (250.f - q)) < 0.1f && std::fabs(b.deficit() - (250.f - q)) < 0.1f,
          "both siblings see the discharge");
    CHECK(a.depth() > 42.f && b.depth() > 42.f, "both release");
}

// the first ibat after boot has no prior interval: a charge at 24 s and ibat at 28 s leave the ref alone
static void test_first_ibat_after_boot() {
    CoulombCounter c;
    c.restoreRef(250.f);
    c.setBmsCharge(250.f, coarseTicks(24'000'000ULL)); c.sampleOnCellFrame(coarseTicks(24'000'000ULL));
    c.updateBatCurrent(-200.f, 28'000'000ULL);
    c.setBmsCharge(250.f - 200.f * 4.f / 3600.f, coarseTicks(28'000'000ULL)); c.sampleOnCellFrame(coarseTicks(28'000'000ULL));
    printf("first ibat after boot: deficit %.3f, ref %.3f\n", c.deficit(), c.bmsRef());
    CHECK(std::fabs(c.bmsRef() - 250.f) < 1e-3f && std::fabs(c.deficit() - 200.f * 4.f / 3600.f) < 1e-3f,
          "no integration from t=0, no false step");
}

// -180 A with charge every 12 s and ibat every 4 or 24 s, either callback order: the ibat interval
// that straddles a charge sample must not be counted in two charge windows
static void test_sparse_ibat_no_drift() {
    for (int ibatEvery: {1, 2, 6})
        for (bool ibatFirst: {true, false}) {
            CoulombCounter c;
            time_us t = 60'000'000ULL;
            float q = 250.f;
            int absorbed = 0;
            auto step = [&](int k) {
                bool ib = k % ibatEvery == 0, ch = k % 3 == 0;
                float off0 = c.bmsRef();
                if (ib && ibatFirst) c.updateBatCurrent(-180.f, t);
                if (ch) c.setBmsCharge(q, coarseTicks(t));
                if (ib && !ibatFirst) c.updateBatCurrent(-180.f, t);
                if (ch) { c.sampleOnCellFrame(coarseTicks(t)); c.updateDepth(); }
                if (std::isfinite(off0) && std::fabs(c.bmsRef() - off0) > 1e-3f) ++absorbed;
            };
            step(0); c.markFull();
            for (int k = 1; k <= 144; ++k) { t += 4'000'000ULL; q -= 180.f * 4.f / 3600.f; step(k); }
            CHECK(absorbed == 0 && std::fabs(c.deficit() - 28.8f) < 0.05f && std::fabs(c.depth() - 28.8f) < 0.05f,
                  "sparse ibat: no false steps");
            if (absorbed || std::fabs(c.deficit() - 28.8f) >= 0.05f)
                printf("  ibat every %d frames, ibat first %d: deficit %.3f, %d absorbed\n", ibatEvery, ibatFirst, c.deficit(), absorbed);
        }
    printf("sparse ibat at -180 A: checked 6 cadences/orders\n");
}

// a re-seed in a gap: bridged with the last ibat up to BRIDGE_MAX_S, taken raw beyond it
static void test_gap_limit() {
    for (float gapS: {100.f, 140.f}) {
        Feed f{250.f};
        f.frame(0.f); f.cc.markFull();
        while (f.q > 230.f) f.frame(-40.f);
        f.frame(+40.f);
        float d0 = 250.f - f.q;
        f.gap((time_us) (gapS * 1e6f), -180.f);
        f.scale -= 36.f;
        f.cc.setBmsCharge(f.q + f.scale, coarseTicks(f.t)); // charge before the first ibat after the gap
        f.cc.sampleOnCellFrame(coarseTicks(f.t)); f.cc.updateDepth();
        f.cc.updateBatCurrent(-180.f, f.t);
        float truth = 250.f - f.q, got = f.cc.deficit();
        printf("gap %.0f s, -180 A real, last ibat +40 A, -36 Ah re-seed: deficit %.2f (true %.2f, from %.2f)\n",
               gapS, got, truth, d0);
        if (gapS <= CoulombCounter::BRIDGE_MAX_S)
            CHECK(std::fabs(got - truth) <= 220.f * gapS / 3600.f + 0.1f, "bridge error bounded by |dI| x gap");
        else
            CHECK(std::fabs(got - truth - 36.f) < 0.1f, "beyond the limit the raw value is taken");
    }
}

// 10 days of 2..8 s ramps between 0 and +-180 A, ibat every 4 s and charge every 12 s with +-1 s jitter,
// charge on its own clock or coincident with ibat in either callback order: nothing is absorbed
static void test_ten_days_no_drift() {
    for (int mode = 0; mode < 3; ++mode) {
        CoulombCounter c;
        uint32_t rng = 12345u + mode;
        auto rnd = [&]() { rng = rng * 1664525u + 1013904223u; return (float) (rng >> 8) / 16777216.f; };
        const int STEP_MS = 50;
        double q = 250.;
        float i = 0.f, i0 = 0.f, i1 = 0.f;
        long segStart = 0, segLen = 1;
        long nextI = 1000, nextC = 1000, nI = 0, nC = 0;
        int absorbed = 0;
        float ref = NAN, lastRef = NAN, maxErr = 0.f;
        bool full = false;
        for (long ms = 1000; ms <= 10L * 86400 * 1000; ms += STEP_MS) {
            if (ms - segStart >= segLen) {
                segStart = ms; segLen = (long) (2000 + rnd() * 6000) / STEP_MS * STEP_MS;
                i0 = i1; i1 = 180.f * (float) ((int) (rnd() * 3.f) - 1);
            }
            float in = i0 + (i1 - i0) * (float) (ms - segStart) / (float) segLen;
            q += (double) (i + in) * 0.5 * STEP_MS / 3.6e6;
            i = in;
            time_us t = (time_us) ms * 1000ULL;
            bool doI = ms >= nextI, doC = mode == 0 ? ms >= nextC : doI && nI % 3 == 0;
            if (doI) nextI = 1000 + (++nI) * 4000 + (long) ((rnd() * 2.f - 1.f) * 1000.f) / STEP_MS * STEP_MS;
            if (mode == 0 && doC) nextC = 1000 + (++nC) * 12000 + (long) ((rnd() * 2.f - 1.f) * 1000.f) / STEP_MS * STEP_MS;
            bool ibatFirst = mode != 2;
            if (doI && ibatFirst) c.updateBatCurrent(i, t);
            if (doC) {
                c.setBmsCharge((float) q, coarseTicks(t));
                c.sampleOnCellFrame(coarseTicks(t));
                if (!full && ms > 60000) { c.markFull(); full = true; ref = lastRef = (float) q; }
                c.updateDepth();
                if (full) {
                    if (std::fabs(c.bmsRef() - lastRef) > 1e-4f) { ++absorbed; lastRef = c.bmsRef(); }
                    maxErr = fmaxf(maxErr, std::fabs(c.deficit() - (ref - (float) q)));
                }
            }
            if (doI && !ibatFirst) c.updateBatCurrent(i, t);
        }
        printf("10 days, mode %d: %ld ibat, %ld charge, %d absorbed, max deficit error %.4f Ah\n", mode, nI, mode ? nI / 3 : nC, absorbed, maxErr);
        CHECK(absorbed == 0 && maxErr < 2e-3f, "no absorption, no drift over 10 days");
    }
}

int main() {
    test_integrators_deadlock();
    test_bms_releases_together();
    test_signed_not_clamped();
    test_stale_bms_falls_back();
    test_no_charge_topic_is_integrator();
    test_reseed_after_gap();
    test_reseed_no_gap();
    test_repeated_reseeds_no_accumulation();
    test_slow_drift_passes();
    test_real_discharge_in_long_gap();
    test_sibling_misses_gap();
    test_first_ibat_after_boot();
    test_sparse_ibat_no_drift();
    test_gap_limit();
    test_ten_days_no_drift();
    printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
    return g_fail ? 1 : 0;
}
