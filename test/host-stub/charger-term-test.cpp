// Host test for the terminated / no-Vout-authority pinning in BatteryCharger (src/charger.h).
//
//   clang++ -std=gnu++20 -fexceptions -I test/host-stub -I test/host-stub/arduino-shim -I src \
//       test/host-stub/charger-term-test.cpp -o /tmp/charger-term-test && /tmp/charger-term-test
//
#include <cassert>
#include <algorithm>
#include <cstdio>
using std::min;
using std::max;
#include "mock.h"
#include "charger.h"

time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { printf("  FAIL: " __VA_ARGS__); printf("\n"); ++g_fail; } } while (0)

static BatChargerParams params() {
    BatChargerParams p;
    p.Vbat_max = 29.0f;
    p.Vbat_fallback = 26.8f;
    p.cv_eoc = 3.5f;
    p.cv_min = 3.35f;
    p.n_cells = 8;
    p.Cbat = 280.f;
    p.tail_c_rate = 0.05f;
    p.Ibat_lim = 40.f;
    p.recharge_dod = 0.15f;
    p.vout_offset_max = 0.6f;
    return p;
}

static void frames(BatteryCharger &c, float vcell, float ibat, int n, bool authority) {
    for (int i = 0; i < n; ++i) {
        setWallClockUs(loopWallClockUs_ + 4'000'000ULL);
        c.batSt.setVcellHigh(vcell);
        c.batSt.updateBatCurrent(ibat);
        c.update(26.7f, 0.f, authority);
    }
}

static void latch(BatteryCharger &c, bool authority) {
    c.termCond.reset();
    frames(c, c.params.cv_min + 0.10f, 0.1f, 20, authority);
}

static const float FLOOR = 26.8f - 0.6f;

static void test_idle_full_pack_yields() {
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    latch(c, true);
    CHECK((bool) c.termCond, "terminated");
    frames(c, 3.36f, 0.1f, 10, false);
    CHECK(std::fabs(c.Vout_max() - FLOOR) < 0.02f, "yield to floor at ~0 A, got %.3f", c.Vout_max());
}

static void test_discharging_pack_joins_with_hysteresis() {
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    latch(c, true);
    frames(c, 3.33f, -4.f, 15, false);
    CHECK(std::fabs(c.Vout_max() - 26.8f) < 0.02f, "join at float while discharging, got %.3f", c.Vout_max());
    frames(c, 3.33f, -0.3f, 15, false);
    CHECK(std::fabs(c.Vout_max() - 26.8f) < 0.02f, "stay joined inside the hysteresis band, got %.3f", c.Vout_max());
    frames(c, 3.36f, 2.f, 15, false);
    CHECK(std::fabs(c.Vout_max() - FLOOR) < 0.02f, "leave when the pack charges, got %.3f", c.Vout_max());
    CHECK((bool) c.termCond, "still terminated");
}

static void test_join_never_exceeds_float() {
    // disabled converter at night (no authority), long discharge: the pin must not wind up
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    latch(c, true);
    float hi = 0;
    for (int k = 0; k < 200; ++k) { frames(c, 3.31f, -8.f, 1, false); hi = std::max(hi, c.Vout_max()); }
    CHECK(hi <= 26.8f + 1e-3f, "pin never above float, max %.3f", hi);
}

static void test_stale_ibat_yields() {
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    latch(c, true);
    frames(c, 3.33f, -4.f, 15, false);
    for (int i = 0; i < 60; ++i) { // cell frames continue, ibat stops for 240 s
        setWallClockUs(loopWallClockUs_ + 4'000'000ULL);
        c.batSt.setVcellHigh(3.33f);
        c.update(26.7f, 0.f, false);
    }
    CHECK(std::fabs(c.Vout_max() - FLOOR) < 0.02f, "stale ibat falls back to yield, got %.3f", c.Vout_max());
}

static void test_regain_authority_after_unglided_latch() {
    // termination latched while this unit had no authority: no float glide was ever started
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    latch(c, false);
    CHECK((bool) c.termCond, "terminated without authority");
    frames(c, 3.34f, 0.f, 3, true); // regains authority, cells below cv_min (no EOC feedback)
    CHECK(c.Vout_max() <= 26.8f + 0.02f, "float target, not Vbat_max, got %.3f", c.Vout_max());
}

static void test_not_terminated_releases_high() {
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    frames(c, 3.30f, -4.f, 10, false);
    CHECK(!(bool) c.termCond, "not terminated");
    CHECK(std::fabs(c.Vout_max() - 29.f) < 0.02f, "no authority, not full: release to Vbat_max, got %.3f", c.Vout_max());
}

int main() {
    test_idle_full_pack_yields();
    test_discharging_pack_joins_with_hysteresis();
    test_join_never_exceeds_float();
    test_stale_ibat_yields();
    test_regain_authority_after_unglided_latch();
    test_not_terminated_releases_high();
    printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
    return g_fail ? 1 : 0;
}
