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

// cell frames stop while bulk charging at vout; returns the pin after `sec` seconds of staleness.
// releaseEvery: a stuck-watchdog pin release every n s; voutAfter: bus once frames stop
static float staleAfter(float vcell, float vout, int sec, bool term = false, int releaseEvery = 0,
                        float voutAfter = NAN, float *maxPin = nullptr) {
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    if (term) latch(c, true); else c.termCond.reset();
    for (int i = 0; i < 40; ++i) {
        setWallClockUs(loopWallClockUs_ + 4'000'000ULL);
        c.batSt.setVcellHigh(vcell);
        c.batSt.updateBatCurrent(term ? 0.f : 20.f);
        c.update(vout, 0.f, true);
    }
    if (std::isfinite(voutAfter)) vout = voutAfter;
    float hi = 0;
    for (int t = 1; t <= sec; ++t) {
        setWallClockUs(loopWallClockUs_ + 1'000'000ULL);
        if (releaseEvery && t % releaseEvery == 0) c.releaseVoutPinning("test");
        c.update(t > 200 && releaseEvery ? c.Vout_max() : vout, 0.f, true); // bus follows the cap
        // settled value: just before the next release (a release jumps to Vbat_max and glides back, as before)
        if (t > 200 && (!releaseEvery || t % releaseEvery == releaseEvery - 1)) hi = std::max(hi, c.Vout_max());
    }
    if (maxPin) *maxPin = hi;
    return c.Vout_max();
}

static float rampAt(int sec) { // expected cap: 27.4 V at 180 s ramping to 26.8 V at 780 s
    return 27.4f - 0.6f * std::min(1.f, std::max(0.f, (sec * 1.f - 180.f) / 600.f));
}

static void test_stale_cells_hold_bus() {
    float v = staleAfter(3.38f, 27.3f, 120);
    CHECK(v > 28.9f, "fresh within expiry: bulk pin, got %.3f", v);
    for (int sec: {200, 400, 700}) {
        v = staleAfter(3.38f, 27.3f, sec);
        CHECK(std::fabs(v - rampAt(sec)) < 0.04f, "gap %d s: ramp %.3f, got %.3f", sec, rampAt(sec), v);
    }
    v = staleAfter(3.38f, 27.3f, 800);
    CHECK(std::fabs(v - 26.8f) < 0.02f, "long gap: float fallback, got %.3f", v);
    v = staleAfter(3.40f, 27.3f, 200);
    CHECK(std::fabs(v - 26.8f) < 0.02f, "cell at cv_min + 0.05: float fallback, got %.3f", v);
    v = staleAfter(3.45f, 27.3f, 200);
    CHECK(std::fabs(v - 26.8f) < 0.02f, "cell above the plateau: float fallback, got %.3f", v);
    v = staleAfter(3.38f, 26.5f, 200);
    CHECK(std::fabs(v - 26.8f) < 0.02f, "bus below float: never under the fallback, got %.3f", v);
    v = staleAfter(3.36f, 27.3f, 200, true);
    CHECK(std::fabs(v - 26.8f) < 0.02f, "terminated: float fallback, got %.3f", v);
    float hi = 0;
    staleAfter(3.38f, 27.3f, 760, false, 36, NAN, &hi);
    CHECK(hi <= 27.4f + 0.01f, "pin releases must not ratchet the cap, max %.3f", hi);
    v = staleAfter(3.38f, 27.3f, 200, false, 0, 27.0f);
    CHECK(v < 27.4f - 0.05f, "bus dropping as the gap begins: lower cap, got %.3f", v);
}

// 11-14: a cloud cut the charge current from ~45 A to ~1 A; the highest cell was still relaxing
// (3.362 V) and the line had dropped to cv_min at once, so both chargers latched "full".
static void test_cloud_dip_does_not_latch() {
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    frames(c, 3.41f, 45.f, 150, true);
    CHECK(!(bool) c.termCond, "bulk at 45 A below cv_eoc keeps charging");
    frames(c, 3.362f, 1.f, 25, true); // 100 s into the dip
    CHECK(!(bool) c.termCond, "relaxing cell in a dip does not latch");
    frames(c, 3.345f, 1.f, 150, true);
    CHECK(!(bool) c.termCond, "relaxed below cv_min: still charging");
}

static void test_low_current_above_line_still_latches() {
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    frames(c, 3.41f, 45.f, 150, true);
    frames(c, 3.362f, 1.f, 90, true); // 6 min: line back at ~3.37
    CHECK(!(bool) c.termCond, "not yet at 6 min");
    frames(c, 3.375f, 1.f, 60, true);
    CHECK((bool) c.termCond, "a cell that stays above the line at tail current latches");
}

// ---- CV hold ----

static BatChargerParams holdParams(uint32_t maxS = 5400) {
    BatChargerParams p = params();
    p.cv_ceiling = 3.55f;
    p.cv_hold_max_s = maxS;
    return p;
}

// 8s pack + cable: vcell_high = voc + ibat * R_CELL, bus = 8 vcell + iout * R_BUS. Each converter is a CV
// source at its pin (R_SRC output resistance) limited to its PV current. 1 s steps (loopLF), ibat frames
// every 4 s, cell frames every 12 s.
struct Plant {
    static constexpr float R_CELL = 0.004f, R_BUS = 0.02f, R_SRC = 0.005f, DV_DAH = 0.0032f;
    float voc = 3.38f, iload = 0.f, ipv[2] = {35.f, 35.f};
    float ibat = 0, vbus = 0, iout[2] = {0, 0};
    bool cells = true; // BMS cell frames flowing
    int t = 0;

    void solve(BatteryCharger **c, int n) {
        float lo = 8 * voc - 5, hi = 8 * voc + 5;
        for (int k = 0; k < 60; ++k) {
            float v = 0.5f * (lo + hi), sum = 0;
            for (int i = 0; i < n; ++i) sum += std::clamp((c[i]->Vout_max() - v) / R_SRC, 0.f, ipv[i]);
            float vb = v - sum * R_BUS;
            float need = (vb - 8 * voc) / (8 * R_CELL) + iload;
            (sum > need ? lo : hi) = v;
        }
        vbus = 0.5f * (lo + hi);
        float sum = 0;
        for (int i = 0; i < n; ++i) sum += (iout[i] = std::clamp((c[i]->Vout_max() - vbus) / R_SRC, 0.f, ipv[i]));
        ibat = sum - iload;
    }

    [[nodiscard]] float vcell() const { return voc + ibat * R_CELL; }

    void run(BatteryCharger **c, int n, int sec, bool psu = false) {
        for (int k = 0; k < sec; ++k) {
            ++t;
            setWallClockUs(loopWallClockUs_ + 1'000'000ULL);
            solve(c, n);
            voc += ibat * (1.f / 3600.f) * DV_DAH;
            for (int i = 0; i < n; ++i) {
                if (t % 4 == 0) c[i]->batSt.updateBatCurrent(ibat);
                if (cells && t % 12 == 0) c[i]->batSt.setVcellHigh(vcell());
                c[i]->update(vbus, iout[i], iout[i] > 0.3f, psu);
            }
        }
    }
    void run(BatteryCharger &c, int sec, bool psu = false) { BatteryCharger *a[1] = {&c}; run(a, 1, sec, psu); }

    int untilHold(BatteryCharger &c, int maxS = 3600) {
        for (int k = 0; k < maxS; ++k) {
            run(c, 1);
            if (c.termCond.cvHold() || (bool) c.termCond) return k;
        }
        return -1;
    }
};

static void test_hold_off_latches_on_second_frame() {
    for (uint32_t maxS: {0u, 5400u}) {
        BatteryCharger c; c.params = holdParams(maxS); setWallClockUs(1'000'000ULL);
        c.termCond.reset();
        frames(c, 3.45f, 30.f, 10, true);
        CHECK(!(bool) c.termCond && !c.termCond.cvHold(), "below the line: charging (max %u)", maxS);
        frames(c, 3.52f, 30.f, 1, true);
        CHECK(!(bool) c.termCond && !c.termCond.cvHold(), "1 frame over the line: nothing yet (max %u)", maxS);
        frames(c, 3.52f, 30.f, 1, true);
        if (!maxS) CHECK((bool) c.termCond && !c.termCond.cvHold(), "off: latched on the 2nd frame");
        else CHECK(!(bool) c.termCond && c.termCond.cvHold(), "on: hold entered on the 2nd frame");
    }
}

static void test_hold_converges_and_tail_latches() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    Plant pl;
    int k = pl.untilHold(c);
    CHECK(k >= 0 && c.termCond.cvHold(), "hold entered (after %d s, ibat %.1f A)", k, pl.ibat);
    float pinIn = c.Vout_max();
    CHECK(pinIn < 29.f - 0.1f, "entry pin at the bus, not Vbat_max: %.3f", pinIn);
    pl.run(c, 600);
    float vmax = 0, vmin = 9, iAtLatch = NAN;
    int tailT = 0, latchedAt = -1;
    for (int s = 0; s < 7200 && latchedAt < 0; s += 12) {
        pl.run(c, 12);
        if (c.termCond.cvHold()) { vmax = std::max(vmax, pl.vcell()); vmin = std::min(vmin, pl.vcell()); }
        tailT = pl.ibat < 14.f ? tailT + 12 : 0;
        if ((bool) c.termCond) { latchedAt = s; iAtLatch = pl.ibat; }
    }
    printf("  hold: cell %.3f..%.3f, tail latch %d s after settling at %.1f A\n", vmin, vmax, latchedAt, iAtLatch);
    CHECK(vmax <= 3.5f + 0.02f && vmin >= 3.5f - 0.02f, "cell held at cv_eoc +- band: %.3f..%.3f", vmin, vmax);
    CHECK(latchedAt >= 0, "tail latched");
    CHECK(iAtLatch < 14.f && tailT >= 170, "latched on the tail: ibat %.1f A, below tail for %d s", iAtLatch, tailT);
    CHECK(c.batSt.coulombCounter.updateDepth() < 1.f, "tail latch marks full");
    pl.run(c, 60);
    CHECK(c.Vout_max() <= 26.8f + 0.02f, "float (or below) after the latch: %.3f", c.Vout_max());
}

static void test_hold_load_step_no_latch() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    Plant pl;
    pl.untilHold(c);
    pl.run(c, 300);
    float pin0 = c.Vout_max();
    pl.iload = 30.f; // PV-limited: ibat ~5 A, the cell drops below the band
    float vhi = 0;
    for (int s = 0; s < 1800; s += 12) { pl.run(c, 12); vhi = std::max(vhi, pl.vcell()); }
    CHECK(pl.ibat < 14.f && pl.vcell() < 3.48f, "load: ibat %.1f A, cell %.3f", pl.ibat, pl.vcell());
    CHECK(!(bool) c.termCond && c.termCond.cvHold(), "low current below the band does not latch");
    CHECK(c.Vout_max() <= pin0 + 1e-3f, "pin not wound up: %.3f -> %.3f (bus %.3f)", pin0, c.Vout_max(), pl.vbus);
    pl.iload = 0.f;
    float vmax = 0;
    for (int s = 0; s < 600; s += 12) { pl.run(c, 12); vmax = std::max(vmax, pl.vcell()); }
    CHECK(vmax < 3.55f && !(bool) c.termCond, "load gone: no ceiling trip, max %.3f", vmax);
}

static void test_hold_timeout() {
    BatteryCharger c; c.params = holdParams(600); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    frames(c, 3.45f, 30.f, 10, true); // ibat smoothing warm-up
    frames(c, 3.52f, 30.f, 2, true);
    CHECK(c.termCond.cvHold(), "in hold");
    int n = 0;
    while (!(bool) c.termCond && n < 400) { frames(c, 3.50f, 30.f, 1, true); ++n; } // 4 s frames
    CHECK((bool) c.termCond && n * 4 > 600 && n * 4 <= 612, "timeout latched after %d s", n * 4);
}

static void test_hold_stale_latches() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    Plant pl;
    pl.untilHold(c);
    pl.run(c, 120);
    pl.cells = false;
    pl.run(c, 170);
    CHECK(c.termCond.cvHold() && !(bool) c.termCond, "still holding within expiry");
    pl.run(c, 30);
    CHECK((bool) c.termCond && !c.termCond.cvHold(), "stale cell data latches");
    pl.run(c, 30);
    CHECK(c.Vout_max() < 28.f, "stale fallback towards float: %.3f", c.Vout_max());
}

static void test_hold_ceiling_latches() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    frames(c, 3.45f, 30.f, 10, true); // ibat smoothing warm-up
    frames(c, 3.52f, 30.f, 2, true);
    CHECK(c.termCond.cvHold(), "in hold");
    frames(c, 3.56f, 30.f, 1, true);
    CHECK(!(bool) c.termCond, "1 frame at the ceiling: not yet");
    frames(c, 3.56f, 30.f, 1, true);
    CHECK((bool) c.termCond && !c.termCond.cvHold(), "ceiling latches in the hold");
}

static void test_hold_cloudy_exit() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    Plant pl;
    pl.untilHold(c);
    pl.run(c, 300);
    pl.ipv[0] = 0.f;
    pl.iload = 5.f;
    pl.voc = 3.33f; // sun gone, relaxed pack discharging
    pl.run(c, 12 * 6);
    CHECK(!(bool) c.termCond && !c.termCond.cvHold(), "left the hold without latching");
    pl.run(c, 10);
    CHECK(c.Vout_max() > 28.9f, "back to bulk: %.3f", c.Vout_max());
}

static void test_hold_two_units_converge() {
    BatteryCharger a, b; a.params = holdParams(); b.params = holdParams(); setWallClockUs(1'000'000ULL);
    a.termCond.reset(); b.termCond.reset();
    Plant pl;
    pl.ipv[0] = 25.f; pl.ipv[1] = 25.f;
    BatteryCharger *u[2] = {&a, &b};
    for (int k = 0; k < 3600 && !(a.termCond.cvHold() && b.termCond.cvHold()); ++k) pl.run(u, 2, 1);
    CHECK(a.termCond.cvHold() && b.termCond.cvHold(), "both in hold");
    pl.run(u, 2, 600);
    float vmax = 0, vmin = 9;
    for (int s = 0; s < 1800 && !(bool) a.termCond; s += 12) {
        pl.run(u, 2, 12);
        vmax = std::max(vmax, pl.vcell()); vmin = std::min(vmin, pl.vcell());
    }
    printf("  two units: cell %.3f..%.3f, pins %.3f/%.3f\n", vmin, vmax, a.Vout_max(), b.Vout_max());
    CHECK(vmax <= 3.5f + 0.02f && vmin >= 3.5f - 0.02f, "two units hold cv_eoc +- band: %.3f..%.3f", vmin, vmax);
    for (int s = 0; s < 7200 && !((bool) a.termCond && (bool) b.termCond); s += 12) pl.run(u, 2, 12);
    CHECK((bool) a.termCond && (bool) b.termCond, "both latch on the tail (ibat %.1f A)", pl.ibat);
}

static void test_hold_psu_freezes_pin() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    Plant pl;
    pl.untilHold(c);
    float pin = c.Vout_max();
    pl.run(c, 240, true);
    CHECK(c.Vout_max() == pin, "psu mode: hold pin frozen %.3f -> %.3f", pin, c.Vout_max());
}

int main() {
    test_cloud_dip_does_not_latch();
    test_low_current_above_line_still_latches();
    test_stale_cells_hold_bus();
    test_idle_full_pack_yields();
    test_discharging_pack_joins_with_hysteresis();
    test_join_never_exceeds_float();
    test_stale_ibat_yields();
    test_regain_authority_after_unglided_latch();
    test_not_terminated_releases_high();
    test_hold_off_latches_on_second_frame();
    test_hold_converges_and_tail_latches();
    test_hold_load_step_no_latch();
    test_hold_timeout();
    test_hold_stale_latches();
    test_hold_ceiling_latches();
    test_hold_cloudy_exit();
    test_hold_two_units_converge();
    test_hold_psu_freezes_pin();
    printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
    return g_fail ? 1 : 0;
}
