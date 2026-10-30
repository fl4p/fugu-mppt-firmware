// Host test for CoulombCounter (src/etc/coulomb_counter.h): two chargers on one BMS.
//
//   clang++ -std=gnu++20 -fexceptions -I test/host-stub -I src test/host-stub/coulomb-test.cpp \
//       -o /tmp/coulomb-test && /tmp/coulomb-test
//
#include <cassert>
#include <cmath>
#include <cstdio>
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
    auto at = [&](float ah) { cc.setBmsCharge(ah, coarseTicks(t)); cc.sampleOnCellFrame(coarseTicks(t)); t += FRAME_US; };
    at(200.f); cc.markFull();
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
    CHECK(std::fabs(d - 6.f * 240.f / 3600.f) < 0.02f, "integrator deficit used while bms stale");
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

int main() {
    test_integrators_deadlock();
    test_bms_releases_together();
    test_signed_not_clamped();
    test_stale_bms_falls_back();
    test_no_charge_topic_is_integrator();
    printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
    return g_fail ? 1 : 0;
}
