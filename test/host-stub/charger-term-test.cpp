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
#include "lfp_pack.h"

time_us loopWallClockUs_ = 0;
MqttService MQTT;
void MqttService::subscribeTopic(const std::string &, MqttMsgCallback) {}
bool MqttService::init(const ConfFile &) { return true; }
void MqttService::close() {}
bool MqttService::onStart() { return false; }
void MqttService::onStop() {}
void MqttService::onTick() {}
void MqttService::_handleEvent(esp_event_base_t, int32_t, void *) {}
void MqttService::_invokeForTest(const std::string &, const char *, int) {}
float strntof(const char *dat, int len) { return strtof(std::string(dat, len).c_str(), nullptr); }
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

// 8s pack (lfp_pack.h) + cable: bus = pack.emf() + ibat * 8 r0 + iout * R_BUS. Each converter is a CV source at its pin
// (R_SRC output resistance) limited to its PV current and Iout_max(). Its Vout sensor reads off[i] from the bus, so it
// regulates the true bus to Vout_max() - off[i]. 1 s steps (loopLF), ibat frames every 4 s, cell frames every 12 s.
struct Plant {
    static constexpr float R_BUS = 0.02f, R_SRC = 0.005f;
    LfpPack pack;
    BmsFeed bms;
    float iload = 0.f, ipv[2] = {35.f, 35.f};
    float off[2] = {0.f, 0.f};
    bool flatOff = false; // unit 1 reads -0.1 V while unit 0 converts, else -0.9 V (fry/flat 11-23)
    float ibat = 0, vbus = 0, iout[2] = {0, 0};
    bool sweep = false; // mppt.isSweeping() of every unit
    int t = 0;

    void solve(BatteryCharger **c, int n) {
        const float e = pack.emf(), rPack = LfpPack::N * pack.r0;
        float lo = e - 5, hi = e + 5;
        auto src = [&](int i, float v) {
            return std::clamp((c[i]->Vout_max() - off[i] - v) / R_SRC, 0.f, std::min(ipv[i], c[i]->Iout_max()));
        };
        for (int k = 0; k < 60; ++k) {
            float v = 0.5f * (lo + hi), sum = 0;
            for (int i = 0; i < n; ++i) sum += src(i, v);
            float vb = v - sum * R_BUS;
            float need = (vb - e) / rPack + iload;
            (sum > need ? lo : hi) = v;
        }
        vbus = 0.5f * (lo + hi);
        float sum = 0;
        for (int i = 0; i < n; ++i) sum += (iout[i] = src(i, vbus));
        ibat = sum - iload;
    }

    [[nodiscard]] float vcell() const { return pack.vcellHigh(ibat); }

    void run(BatteryCharger **c, int n, int sec, bool psu = false) {
        for (int k = 0; k < sec; ++k) {
            ++t;
            setWallClockUs(loopWallClockUs_ + 1'000'000ULL);
            if (flatOff) off[1] = iout[0] > 1.f ? -0.1f : -0.9f;
            solve(c, n);
            pack.step(ibat, 1.f);
            bms.poll(t, ibat, vcell());
            for (int i = 0; i < n; ++i) {
                bms.deliver(*c[i]);
                // main.cpp lfControl(): any running converter has authority while the charger holds
                c[i]->update(vbus + off[i], iout[i], iout[i] > 0.3f || (c[i]->chargeHold() && ipv[i] > 0), psu, sweep);
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
    CHECK(c.Vout_max() == 29.f && std::fabs(c.Iout_max() - pl.iout[0]) < 2.f,
          "entry: pin at Vbat_max, current limit at the output current (%.1f / %.1f A)", c.Iout_max(), pl.iout[0]);
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
    pl.iload = 30.f; // PV-limited: ibat ~5 A, the cell drops below the band
    float limHi = 0;
    for (int s = 0; s < 1800; s += 12) { pl.run(c, 12); limHi = std::max(limHi, c.Iout_max() - pl.iout[0]); }
    CHECK(pl.ibat < 14.f && pl.vcell() < 3.48f, "load: ibat %.1f A, cell %.3f", pl.ibat, pl.vcell());
    CHECK(!(bool) c.termCond && c.termCond.cvHold(), "low current below the band does not latch");
    CHECK(limHi <= 2.f + 0.01f, "limit not wound up: at most %.2f A above the output current", limHi);
    pl.iload = 0.f;
    float vmax = 0;
    int out = 0;
    for (int s = 0; s < 600; ++s) {
        pl.run(c, 1);
        vmax = std::max(vmax, pl.vcell());
        if (s >= 60 && std::fabs(pl.vcell() - 3.5f) > 0.015f) ++out;
    }
    printf("  load off: cell peak %.3f, %d s out of +-15 mV after 60 s\n", vmax, out);
    CHECK(vmax < 3.55f && !(bool) c.termCond && out == 0, "load gone: no ceiling trip (max %.3f), back in band", vmax);

    // load step during the hold with PV to spare: the current limit follows, the cell stays near cv_eoc
    pl.iload = 5.f;
    float vlo = 9;
    vmax = 0;
    for (int s = 0; s < 600; s += 12) {
        pl.run(c, 12);
        if (s >= 60) vlo = std::min(vlo, pl.vcell()), vmax = std::max(vmax, pl.vcell());
    }
    printf("  5 A load: cell %.3f..%.3f, iout %.1f A\n", vlo, vmax, pl.iout[0]);
    CHECK(vlo >= 3.5f - 0.015f && vmax <= 3.5f + 0.015f, "load on: cell back at cv_eoc: %.3f..%.3f", vlo, vmax);
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
    pl.bms.cells = false;
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
    pl.pack.setMean(3.33f); // sun gone, relaxed pack discharging
    pl.run(c, 12 * 6);
    CHECK(!(bool) c.termCond && !c.termCond.cvHold(), "left the hold without latching");
    pl.run(c, 10);
    CHECK(c.Vout_max() > 28.9f, "back to bulk: %.3f", c.Vout_max());
}

static void test_hold_two_units_converge(bool lag) {
    BatteryCharger a, b; a.params = holdParams(); b.params = holdParams(); setWallClockUs(1'000'000ULL);
    a.termCond.reset(); b.termCond.reset();
    Plant pl;
    pl.bms.lagI = lag;
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
    printf("  two units%s: cell %.3f..%.3f, ilim %.1f/%.1f A\n", lag ? " (ibat 1 frame late)" : "", vmin, vmax,
           a.Iout_max(), b.Iout_max());
    CHECK(vmax <= 3.5f + 0.015f && vmin >= 3.5f - 0.015f, "two units hold cv_eoc +- band: %.3f..%.3f", vmin, vmax);
    for (int s = 0; s < 7200 && !((bool) a.termCond && (bool) b.termCond); s += 12) pl.run(u, 2, 12);
    CHECK((bool) a.termCond && (bool) b.termCond, "both latch on the tail (ibat %.1f A)", pl.ibat);
}

static void test_hold_psu_freezes_pin() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    Plant pl;
    pl.untilHold(c);
    pl.run(c, 1);
    float lim = c.Iout_max();
    pl.run(c, 240, true);
    CHECK(c.Iout_max() == lim && c.Vout_max() == 29.f, "psu mode: hold limit frozen %.2f -> %.2f A", lim, c.Iout_max());
}

// a sweep in the hold (converter off, then ramping from 0) must not pull the limit down to iout + 2 A; without the
// sweep flag the same dip does (the PV-limited anti-wind-up)
static void test_hold_sweep_keeps_limit(bool flag) {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    Plant pl;
    pl.untilHold(c);
    pl.run(c, 600);
    const float lim = c.Iout_max(), i0 = pl.iout[0];
    pl.sweep = flag;
    pl.ipv[0] = 0.f;
    pl.run(c, 6); // calibration + the low part of the ramp
    const float limSw = c.Iout_max();
    pl.ipv[0] = 35.f;
    pl.run(c, 4);
    pl.sweep = false;
    pl.run(c, 4);
    if (flag) {
        CHECK(limSw == lim, "sweep: limit kept %.2f -> %.2f A", lim, limSw);
        CHECK(pl.iout[0] > 0.9f * i0 && c.Iout_max() <= lim + 2.f, "sweep: back at %.2f A (before %.2f A), limit %.2f A",
              pl.iout[0], i0, c.Iout_max());
    } else CHECK(limSw <= 2.f + 0.01f, "no sweep flag: limit collapses to %.2f A (iout + 2 A)", limSw);
}

// clouds in the hold: no wind-up of the current limit, no overshoot when the sun returns
static void test_hold_pv_limited_recovers() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    Plant pl;
    pl.untilHold(c);
    pl.run(c, 600);
    pl.ipv[0] = 4.f;
    float limHi = 0;
    for (int s = 0; s < 1800; s += 4) { pl.run(c, 4); limHi = std::max(limHi, c.Iout_max() - pl.iout[0]); }
    CHECK(c.termCond.cvHold() && pl.vcell() < 3.49f, "cloud: in hold, cell %.3f below target", pl.vcell());
    CHECK(limHi <= 2.f + 0.01f && c.Iout_max() < 7.f, "cloud: limit %.2f A, at most %.2f A above iout", c.Iout_max(), limHi);
    pl.ipv[0] = 35.f;
    float vmax = 0;
    int s = 0, back = -1;
    for (; s < 1200; s += 12) {
        pl.run(c, 12);
        vmax = std::max(vmax, pl.vcell());
        if (back < 0 && pl.vcell() >= 3.5f - 0.005f) back = s;
    }
    printf("  sun back: cell max %.3f, at cv_eoc after %d s\n", vmax, back);
    CHECK(vmax <= 3.5f + 0.015f, "sun back: no overshoot, max %.3f", vmax);
    CHECK(back >= 0 && back < 600, "sun back: cell at cv_eoc after %d s", back);
}

// cell frames stop in the hold: stale latch, the limit goes, the existing stale fallback holds the bus
static void test_hold_stale_releases_limit() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    Plant pl;
    pl.untilHold(c);
    pl.run(c, 300);
    CHECK(c.Iout_max() < 30.f, "hold limits the current: %.1f A", c.Iout_max());
    float lim = c.Iout_max();
    pl.bms.cells = false;
    pl.run(c, 170);
    CHECK(c.termCond.cvHold() && c.Iout_max() <= lim + 2.f, "within expiry: still limited %.1f A", c.Iout_max());
    pl.run(c, 30);
    CHECK((bool) c.termCond && c.Iout_max() >= 40.f - 1e-3f, "stale: latched, limit released (%.1f A)", c.Iout_max());
    float bus = pl.vbus;
    pl.run(c, 2);
    CHECK(c.Vout_max() <= bus + 0.05f, "stale: pin at the bus, no output step (%.3f, bus %.3f)", c.Vout_max(), bus);
    pl.run(c, 30);
    CHECK(std::fabs(c.Vout_max() - 26.8f) < 0.02f, "stale: float fallback %.3f", c.Vout_max());
}

// ---- top balancing ----

static BatChargerParams balParams(uint32_t holdS = 1800, uint32_t maxS = 0) {
    BatChargerParams p = holdParams(maxS);
    p.balance_interval_s = 30 * 86400;
    p.balance_hold_s = holdS;
    return p;
}

static void test_hold_entry_40a_two_units() {
    BatteryCharger a, b; a.params = balParams(7200); b.params = balParams(7200); setWallClockUs(1'000'000ULL);
    a.termCond.reset(); b.termCond.reset();
    a.balanceNow(); b.balanceNow();
    Plant pl;
    topOfCharge(pl.pack);
    pl.ipv[0] = pl.ipv[1] = 20.f;
    BatteryCharger *u[2] = {&a, &b};
    float vmax = 0, vlo = 9, vhi = 0;
    int hold = -1;
    for (int t = 0; t < 1200; ++t) {
        pl.run(u, 2, 1);
        vmax = std::max(vmax, pl.vcell());
        if (hold < 0 && a.termCond.cvHold() && b.termCond.cvHold()) hold = t;
        if (hold >= 0 && t > hold + 300) vlo = std::min(vlo, pl.vcell()), vhi = std::max(vhi, pl.vcell());
    }
    printf("  40 A entry: cell peak %.3f, hold after %d s, then %.3f..%.3f at %.1f A\n", vmax, hold, vlo, vhi, pl.ibat);
    CHECK(vmax <= 3.5f + 0.015f, "40 A into the top: cell peak %.3f <= cv_eoc + 15 mV", vmax);
    CHECK(hold >= 0 && !(bool) a.termCond && !(bool) b.termCond, "40 A: both in the hold, no latch");
    CHECK(vlo >= 3.5f - 0.015f && vhi <= 3.5f + 0.015f, "40 A: held at cv_eoc +- 15 mV: %.3f..%.3f", vlo, vhi);
}

// 11-23 14:14: fry holds (16 A) as flat reboots and runs unlimited at 20 A before its ibat smoothing is warm.
static void test_hold_sibling_boots_unlimited() {
    BatteryCharger a, b; a.params = balParams(7200); b.params = balParams(7200); setWallClockUs(1'000'000ULL);
    a.termCond.reset(); b.termCond.reset();
    a.balanceNow(); b.balanceNow();
    Plant pl;
    topOfCharge(pl.pack);
    pl.ipv[0] = 16.f; pl.ipv[1] = 0.f;
    BatteryCharger *u[2] = {&a, &b};
    for (int k = 0; k < 3600 && pl.vcell() < 3.5f; ++k) pl.run(u, 1, 1);
    pl.run(u, 2, 1); // b's first update before it may convert
    pl.ipv[1] = 20.f;
    float vmax = 0, idle = 0;
    for (int t = 0; t < 900 && !(bool) a.termCond && !(bool) b.termCond; ++t) {
        pl.run(u, 2, 1);
        vmax = std::max(vmax, pl.vcell());
        if (t < 30) idle = std::max(idle, pl.iout[1]);
    }
    printf("  sibling boots: cell peak %.3f, booted unit <= %.2f A in its first 30 s\n", vmax, idle);
    CHECK(!(bool) a.termCond && !(bool) b.termCond, "sibling boot: no ceiling latch (peak %.3f)", vmax);
    CHECK(vmax <= 3.5f + 0.015f && idle < 0.5f, "sibling boot: peak %.3f, booted unit idle (%.2f A)", vmax, idle);
}

// cell frames near the top but no ibat source: idle for BOOT_IDLE_US after boot only
static void test_boot_idle_without_ibat() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    for (int k = 0; k < 10; ++k) {
        setWallClockUs(loopWallClockUs_ + 4'000'000ULL);
        c.batSt.setVcellHigh(3.49f);
        c.update(27.6f, 0.f, false);
    }
    CHECK(c.Iout_max() < 0.3f, "boot near the top, no ibat yet: idle (%.2f A)", c.Iout_max());
    for (int k = 0; k < 30; ++k) {
        setWallClockUs(loopWallClockUs_ + 4'000'000ULL);
        c.batSt.setVcellHigh(3.49f);
        c.update(27.6f, 0.f, false);
    }
    CHECK(c.Iout_max() >= 40.f - 1e-3f, "no ibat source: released after the boot window (%.1f A)", c.Iout_max());
    BatteryCharger d; d.params = holdParams(); setWallClockUs(1'000'000ULL);
    d.termCond.reset();
    frames(d, 3.40f, 20.f, 12, true);
    CHECK(d.Iout_max() >= 40.f - 1e-3f, "boot below the top: no idle (%.1f A)", d.Iout_max());
}

static void tick(BatteryCharger &c, float vout = 27.5f) {
    setWallClockUs(loopWallClockUs_ + 1'000'000ULL);
    c.update(vout, 0.f, true);
}

// frames() helper reaching the hold: warm-up below the line, then 2 frames above it
static void enterHold(BatteryCharger &c) {
    frames(c, 3.45f, 30.f, 10, true);
    frames(c, 3.52f, 30.f, 2, true);
}

static void test_bal_not_due_normal() {
    for (uint32_t maxS: {0u, 5400u}) {
        BatteryCharger c; c.params = balParams(1800, maxS); setWallClockUs(1'000'000ULL);
        c.termCond.reset();
        CHECK(!c.balanceDue() && c.balanceAgeS() < 5, "boot: unknown age counts from boot, not due (age %u)",
              c.balanceAgeS());
        Plant pl;
        pl.untilHold(c);
        if (!maxS) CHECK((bool) c.termCond && !c.termCond.cvHold(), "not due, hold off: latched on the line");
        else {
            CHECK(c.termCond.cvHold() && !c.balanceRunning(), "not due: regular hold");
            for (int s = 0; s < 7200 && !(bool) c.termCond; s += 12) pl.run(c, 12);
            CHECK((bool) c.termCond && c.termCond.cvHoldS() == 0, "not due: tail latches as before");
        }
        CHECK(c.balanceInBandS() == 0, "not due: no in-band time counted");
    }
    BatteryCharger c; c.params = balParams(); c.params.balance_interval_s = 86400; setWallClockUs(1'000'000ULL);
    tick(c);
    CHECK(!c.balanceDue(), "1 d interval: not due at boot");
    setWallClockUs(loopWallClockUs_ + 86400ULL * 1000000ULL);
    tick(c);
    CHECK(c.balanceDue(), "1 d interval: due a day after boot");
}

static void test_bal_completes() {
    BatteryCharger c; c.params = balParams(7200); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    c.balanceNow();
    tick(c);
    CHECK(c.balanceDue(), "balance now: due");
    Plant pl;
    int k = pl.untilHold(c);
    CHECK(k >= 0 && c.termCond.cvHold() && c.balanceRunning(), "hold entered with cv_hold_max_s = 0");
    int inBand = 0, tail = 0, tailMax = 0, held = 0;
    float vmax = 0, vmin = 9;
    while (!(bool) c.termCond && held < 20000) {
        pl.run(c, 1);
        ++held;
        if (pl.vcell() >= 3.48f) ++inBand;
        if (held > 600) { vmax = std::max(vmax, pl.vcell()); vmin = std::min(vmin, pl.vcell()); }
        tail = pl.ibat < 14.f ? tail + 1 : 0;
        tailMax = std::max(tailMax, tail);
    }
    printf("  balance: latched after %d s (cell in band ~%d s, tail streak %d s, cell %.3f..%.3f)\n", held, inBand,
           tailMax, vmin, vmax);
    CHECK((bool) c.termCond && !c.termCond.cvHold(), "run latched");
    CHECK(tailMax > 1000, "tail exit ignored during the run (tail streak %d s)", tailMax);
    CHECK(held >= 7200 && inBand >= 7200 - 30 && held <= 7200 + 600, "completes at balance_hold_s: %d s, %d in band",
          held, inBand);
    CHECK(vmax <= 3.5f + 0.02f && vmin >= 3.5f - 0.02f, "cell held at cv_eoc +- band: %.3f..%.3f", vmin, vmax);
    CHECK(!c.balanceDue() && c.balanceAgeS() < 15 && c.balanceInBandS() == 0, "done: age 0, progress 0 (age %u)", c.balanceAgeS());
    CHECK(c.batSt.coulombCounter.updateDepth() < 1.f, "completion marks full");
    pl.run(c, 60);
    CHECK(c.Vout_max() <= 26.8f + 0.02f, "float after the run: %.3f", c.Vout_max());
}

static void test_bal_progress_survives_sun_gone_and_cap() {
    BatteryCharger c; c.params = balParams(1800); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    c.balanceNow();
    Plant pl;
    pl.untilHold(c);
    pl.run(c, 600);
    pl.ipv[0] = 0.f; pl.iload = 5.f; pl.pack.setMean(3.33f); // sun gone
    pl.run(c, 12 * 6);
    CHECK(!(bool) c.termCond && !c.termCond.cvHold(), "sun gone: hold left, not terminated");
    uint32_t p1 = c.balanceInBandS();
    CHECK(p1 > 450 && p1 <= 600 + 12, "first session in band %u s", p1);
    pl.run(c, 3600);
    CHECK(c.balanceInBandS() == p1 && c.balanceDue(), "progress kept overnight: %u s", c.balanceInBandS());
    pl.ipv[0] = 35.f; pl.iload = 0.f; pl.pack.setMean(3.40f); // next day
    pl.untilHold(c, 7200);
    CHECK(c.balanceRunning(), "next day: run resumes");
    int held = 0;
    while (!(bool) c.termCond && held < 5000) { pl.run(c, 1); ++held; }
    CHECK((bool) c.termCond && !c.balanceDue(), "completes in the second session");
    CHECK(held >= (int) (1800 - p1) - 5 && held < (int) (1800 - p1) + 300, "second session %d s, needed %u s", held,
          1800 - p1);

    // session cap: PV-limited under a load, the cell sits below the band
    BatteryCharger d; d.params = balParams(1800); setWallClockUs(loopWallClockUs_ + 1'000'000ULL);
    d.termCond.reset();
    d.balanceNow();
    Plant q;
    q.untilHold(d);
    q.iload = 30.f;
    held = 0;
    while (!(bool) d.termCond && held < 8000) { q.run(d, 1); ++held; }
    uint32_t p2 = d.balanceInBandS();
    printf("  session cap after %d s, %u s in band\n", held, p2);
    CHECK((bool) d.termCond && held > 3400 && held < 3700, "session capped at 2 balance_hold_s (%d s)", held);
    CHECK(d.balanceDue() && p2 < 1800, "cap keeps the run pending (%u s in band)", p2);
    CHECK(d.batSt.coulombCounter.updateDepth() < 1.f, "cap terminates as a normal full");
}

static void test_bal_delta() {
    for (int variant = 0; variant < 4; ++variant) {
        BatteryCharger c; c.params = balParams(7200); setWallClockUs(1'000'000ULL);
        c.termCond.reset();
        c.balanceNow();
        enterHold(c);
        CHECK(c.balanceRunning(), "delta %d: run", variant);
        int n = 0;
        for (; n < 60 && !(bool) c.termCond; ++n) {
            float vc = variant == 3 ? 3.45f : 3.50f; // 3: out of band
            if (variant == 0 || variant == 3 || (variant == 1 && n == 0)) c.batSt.setCellDelta(0.005f);
            if (variant == 2) c.batSt.setCellDelta(n % 2 ? 0.005f : 0.02f); // never 3 in a row
            frames(c, vc, 10.f, 1, true);
        }
        if (variant == 0) CHECK((bool) c.termCond && n <= 5 && !c.balanceDue(), "delta: early finish after %d frames", n);
        else CHECK(!(bool) c.termCond && c.balanceDue(), "delta %d: no early finish (%d frames)", variant, n);
    }
    BatteryCharger c; c.params = balParams(7200); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    c.balanceNow();
    enterHold(c);
    for (int n = 0; n < 5; ++n) { c.batSt.setCellDelta(NAN); frames(c, 3.50f, 10.f, 1, true); }
    CHECK(!(bool) c.termCond && c.balanceDue(), "NAN delta is not balanced");
}

static void test_bal_defer() {
    for (int why = 0; why < 2; ++why) {
        BatteryCharger c; c.params = balParams(1800);
        if (!why) { c.params.holiday_charge = 0.6f; c.params.holiday_ibat = 1.f; }
        setWallClockUs(1'000'000ULL);
        c.termCond.reset();
        c.balanceNow();
        enterHold(c);
        CHECK(c.balanceRunning(), "defer %d: running", why);
        frames(c, 3.50f, 10.f, 30, true);
        uint32_t p = c.balanceInBandS();
        if (!why) c.setHolidayOverride(1);
        else c.batSt.setTemp(0, 36.f);
        frames(c, 3.50f, 10.f, 2, true);
        CHECK(!c.balanceRunning() && !c.termCond.cvHold() && c.balanceDue(), "defer %d: hold ended", why);
        CHECK((bool) c.termCond && c.batSt.coulombCounter.updateDepth() < 1.f, "defer %d: normal full", why);
        CHECK(c.balanceDeferWhy() != nullptr, "defer %d: reason given", why);
        frames(c, 3.50f, 10.f, 10, true);
        CHECK(c.balanceInBandS() <= p + 4, "defer %d: progress kept, not counting (%u -> %u)", why, p, c.balanceInBandS());
        if (why) {
            c.batSt.setTemp(0, 34.f);
            frames(c, 3.40f, 10.f, 1, true);
            CHECK(c.balanceDeferWhy() != nullptr, "34 °C: still warm (hysteresis)");
            c.batSt.setTemp(0, 32.f);
            frames(c, 3.40f, 10.f, 1, true);
            CHECK(c.balanceDeferWhy() == nullptr, "32 °C: may run again");
        }
    }
    BatteryCharger c; c.params = balParams(1800); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    c.balanceNow();
    c.batSt.setTemp(0, 40.f);
    enterHold(c);
    CHECK((bool) c.termCond && !c.termCond.cvHold(), "warm pack: no run, normal termination");
    c.batSt.setTemp(0, NAN);
    tick(c);
    CHECK(c.balanceDeferWhy() == nullptr, "no fresh temperature: not deferred on temperature");
}

static void test_bal_ceiling_and_stale() {
    BatteryCharger c; c.params = balParams(1800); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    c.balanceNow();
    enterHold(c);
    frames(c, 3.50f, 10.f, 20, true);
    uint32_t p = c.balanceInBandS();
    frames(c, 3.56f, 10.f, 2, true);
    CHECK((bool) c.termCond && c.balanceDue() && c.balanceInBandS() >= p, "ceiling latches, run stays pending");
    CHECK(c.balanceDeferWhy() && !strcmp(c.balanceDeferWhy(), "ceiling"), "ceiling: deferred");
    c.termCond.reset(); // next charge
    enterHold(c);
    CHECK(!c.balanceRunning() && !c.termCond.cvHold() && (bool) c.termCond, "next charge: no run, normal termination");
    setWallClockUs(loopWallClockUs_ + (30ULL * 86400ULL / 4) * 1000000ULL);
    frames(c, 3.40f, 10.f, 2, true);
    CHECK(c.balanceDeferWhy() == nullptr && c.balanceDue(), "interval / 4 later: may run again");
    c.termCond.reset();
    enterHold(c);
    CHECK(c.balanceRunning(), "interval / 4 later: runs");
    frames(c, 3.56f, 10.f, 2, true);
    CHECK(c.balanceDeferWhy() != nullptr, "ceiling again: deferred");
    c.balanceNow();
    tick(c);
    CHECK(c.balanceDeferWhy() == nullptr, "balance now: deferral cleared");

    BatteryCharger d; d.params = balParams(1800); setWallClockUs(loopWallClockUs_ + 1'000'000ULL);
    d.termCond.reset();
    d.balanceNow();
    enterHold(d);
    frames(d, 3.50f, 10.f, 20, true);
    p = d.balanceInBandS();
    for (int s = 0; s < 400; ++s) tick(d); // cell + ibat frames stop
    CHECK(!(bool) d.termCond && d.termCond.stalePaused() && d.balanceDue(), "stale 400 s: paused, not latched");
    for (int s = 0; s < 600; ++s) tick(d);
    CHECK((bool) d.termCond && d.balanceDue() && d.balanceInBandS() <= p + 64, "stale: latched, run pending (%u -> %u)",
          p, d.balanceInBandS());
    CHECK(d.balanceDeferWhy() != nullptr, "stale: deferred");
}

// BMS frames (cells and ibat) stop for gapS mid run (fry/flat 11-23: ~3 min, both units latched)
static void test_bal_stale_gap(int gapS, bool bal = true) {
    BatteryCharger c; c.params = bal ? balParams(7200) : holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    if (bal) c.balanceNow();
    Plant pl;
    pl.untilHold(c);
    CHECK(c.termCond.cvHold() && c.balanceRunning() == bal, "gap %d: in the hold", gapS);
    pl.run(c, 1800);
    uint32_t p0 = c.balanceInBandS();
    float limBefore = c.Iout_max(), ibatBefore = pl.ibat;
    pl.bms.cells = pl.bms.ibatOn = false;
    int s = 0, latchedAt = -1, pausedS = 0;
    float limMin = 99.f, vmaxGap = 0.f, ibatGapEnd = 0.f;
    for (; s < gapS; ++s) {
        pl.run(c, 1);
        vmaxGap = std::max(vmaxGap, pl.vcell());
        if (c.termCond.stalePaused()) {
            ++pausedS;
            CHECK(!c.termCond.cvHold() && c.Vout_max() <= pl.vbus + 0.05f, "gap %d: paused at %d s, pin %.3f above the bus",
                  gapS, s, c.Vout_max());
            limMin = std::min(limMin, c.Iout_max());
        }
        if (latchedAt < 0 && (bool) c.termCond) latchedAt = s;
    }
    ibatGapEnd = pl.ibat;
    uint32_t pGap = c.balanceInBandS();
    printf("  gap %d s%s: paused %d s, latched at %d s, in band %u -> %u s, ibat %.1f -> %.1f A, lim %.1f -> %.1f A, cell max %.3f\n",
           gapS, bal ? "" : " (normal hold)", pausedS, latchedAt, p0, pGap, ibatBefore, ibatGapEnd, limBefore, limMin,
           vmaxGap);
    CHECK(pGap <= p0 + 60, "gap %d: in-band time not counted past the 60 s cell-age limit (%u -> %u)", gapS, p0, pGap);
    if (!bal) {
        CHECK(latchedAt >= 175 && latchedAt <= 200 && pausedS == 0, "normal hold: latches at expiry as before (%d s)",
              latchedAt);
        return;
    }
    if (gapS > 800) {
        CHECK(latchedAt >= 775 && latchedAt <= 800, "long gap: latched after expiry + pause (%d s)", latchedAt);
        CHECK(!c.termCond.stalePaused() && c.balanceDue() && pGap >= p0, "long gap: run pending, progress kept");
        pl.bms.cells = pl.bms.ibatOn = true;
        pl.run(c, 120);
        CHECK((bool) c.termCond && !c.termCond.cvHold(), "long gap: still terminated with fresh data");
        return;
    }
    CHECK(latchedAt < 0 && pausedS > 0 && c.termCond.stalePaused(), "short gap: paused, not latched (latched at %d s)",
          latchedAt);
    CHECK(ibatGapEnd < 2.f, "short gap: pack current off at float (%.1f A)", ibatGapEnd);
    pl.bms.cells = pl.bms.ibatOn = true;
    int back = -1;
    float ibatMax = -99.f, vmax = 0.f;
    for (s = 0; s < 120; ++s) {
        pl.run(c, 1);
        if (back < 0 && c.balanceRunning()) back = s;
        if (back >= 0) ibatMax = std::max(ibatMax, pl.ibat), vmax = std::max(vmax, pl.vcell());
    }
    CHECK(back >= 0 && back <= 13 && !(bool) c.termCond, "short gap: hold resumed after %d s", back);
    CHECK(ibatMax <= ibatBefore + 8.f + 0.1f, "resume: no current surge (ibat %.1f A, %.1f A before)", ibatMax, ibatBefore);
    int held = 0;
    while (!(bool) c.termCond && held < 20000) {
        pl.run(c, 1);
        ++held;
        vmax = std::max(vmax, pl.vcell());
    }
    printf("  gap %d s: resumed after %d s, completed %d s later, cell max %.3f\n", gapS, back, held + 120, vmax);
    CHECK((bool) c.termCond && !c.balanceDue(), "short gap: run completes");
    CHECK(held + 120 >= (int) (7200 - pGap) - 30 && held + 120 <= (int) (7200 - pGap) + 300,
          "short gap: completes on the kept time (%d s after the gap, %u s left)", held + 120, 7200 - pGap);
    CHECK(vmax <= 3.5f + 0.02f, "short gap: cell held at cv_eoc after the resume (max %.3f)", vmax);
}

static void test_bal_partial_hold_suspended() {
    BatteryCharger c; c.params = balParams(); c.params.partial_charge = 0.8f; setWallClockUs(1'000'000ULL);
    latch(c, true);
    c.termCond.reset();
    frames(c, 3.34f, 0.f, 5, true);
    CHECK(c.partialHold(), "partial hold after a full");
    c.balanceNow();
    frames(c, 3.34f, 0.f, 2, true);
    CHECK(!c.partialHold(), "balance due: partial ceiling suspended");
    c.balanceSkip();
    frames(c, 3.34f, 0.f, 2, true);
    CHECK(c.partialHold() && !c.balanceDue(), "skipped: partial hold again");
}

static void test_bal_rtc() {
    ChargerRtc r{};
    BatteryCharger a; a.params = balParams(1800);
    a.params.holiday_charge = 0.6f; a.params.holiday_ibat = 1.f;
    setWallClockUs(1'000'000ULL);
    a.attachRtc(&r);
    a.termCond.reset();
    a.setHolidayOverride(2);
    a.balanceNow();
    enterHold(a);
    frames(a, 3.50f, 10.f, 25, true);
    uint32_t p = a.balanceInBandS();
    CHECK(r.valid() && r.balValid() && r.magic == 0x48444c32, "saved, v2 prefix intact");
    CHECK(r.balAgeS == UINT32_MAX && r.balInBandS == p, "forced + progress saved");

    BatteryCharger b; b.params = balParams(1800); b.params.holiday_charge = 0.6f; b.params.holiday_ibat = 1.f;
    b.attachRtc(&r);
    CHECK(b.holidayOverride() == 2 && b.balanceInBandS() == p, "restored: holiday override, progress %u", b.balanceInBandS());
    tick(b);
    CHECK(b.balanceDue(), "restored: still forced due");

    BatteryCharger e; e.params = balParams(1800);
    setWallClockUs(loopWallClockUs_ + 1'000'000ULL);
    e.attachRtc(&r);
    tick(e);
    e.balanceSkip();
    tick(e);
    setWallClockUs(loopWallClockUs_ + 10ULL * 86400ULL * 1000000ULL);
    tick(e);
    BatteryCharger f; f.params = balParams(1800);
    f.attachRtc(&r);
    CHECK(f.balanceAgeS() >= 10 * 86400 && !f.balanceDue(), "age restored: %u s", f.balanceAgeS());

    // v2 layout (older build): holiday restored, balance unknown (from boot)
    ChargerRtc o{};
    o.magic = ChargerRtc::MAGIC; o.fullAgeS = 3600; o.bmsRef = NAN; o.holiday = 1; o.override_ = 1;
    o.sum = o.checksum();
    o.balMagic = 0xdeadbeef; o.balAgeS = 5; o.balInBandS = 1234; o.balSum = 7;
    BatteryCharger g; g.params = balParams(1800); g.params.holiday_charge = 0.6f; g.params.holiday_ibat = 1.f;
    g.attachRtc(&o);
    CHECK(g.holidayOverride() == 1 && g.balanceInBandS() == 0, "v2: holiday kept, no balance progress");
}

static void test_bal_two_units() {
    BatteryCharger a, b; a.params = balParams(3600); b.params = balParams(3600); setWallClockUs(1'000'000ULL);
    a.termCond.reset(); b.termCond.reset();
    a.balanceNow(); b.balanceNow();
    Plant pl;
    pl.ipv[0] = 25.f; pl.ipv[1] = 25.f;
    BatteryCharger *u[2] = {&a, &b};
    for (int k = 0; k < 3600 && !(a.balanceRunning() && b.balanceRunning()); ++k) pl.run(u, 2, 1);
    CHECK(a.balanceRunning() && b.balanceRunning(), "both run");
    pl.run(u, 2, 600);
    float vmax = 0, vmin = 9;
    int s = 0;
    for (; s < 6000 && !((bool) a.termCond && (bool) b.termCond); s += 12) {
        pl.run(u, 2, 12);
        if (!(bool) a.termCond) { vmax = std::max(vmax, pl.vcell()); vmin = std::min(vmin, pl.vcell()); }
    }
    printf("  two units balance: cell %.3f..%.3f, both latched %d s later\n", vmin, vmax, s);
    CHECK(vmax <= 3.5f + 0.02f && vmin >= 3.5f - 0.02f, "two units hold cv_eoc +- band: %.3f..%.3f", vmin, vmax);
    CHECK((bool) a.termCond && (bool) b.termCond && !a.balanceDue() && !b.balanceDue(), "both complete");
}

// 11-23: flat's Vout reads -0.1 V while fry converts and -0.9 V alone. fry backs off, flat then holds in its own
// low-reading frame and the runaway cell walks to the ceiling.
static void test_hold_vout_offset_two_units() {
    BatteryCharger a, b; a.params = balParams(7200); b.params = balParams(7200); setWallClockUs(1'000'000ULL);
    a.termCond.reset(); b.termCond.reset();
    a.balanceNow(); b.balanceNow();
    Plant pl;
    pl.ipv[0] = pl.ipv[1] = 30.f;
    pl.pack.set(3.38f, 0.06f); pl.pack.kImb = 0.01f; pl.flatOff = true;
    BatteryCharger *u[2] = {&a, &b};
    for (int k = 0; k < 3600 && !(a.balanceRunning() && b.balanceRunning()); ++k) pl.run(u, 2, 1);
    CHECK(a.balanceRunning() && b.balanceRunning(), "both run");
    pl.run(u, 2, 600);
    float vmax = 0, vmin = 9, vend = 0;
    int s = 0;
    for (; s < 5400 && !(bool) a.termCond && !(bool) b.termCond; s += 12) {
        pl.run(u, 2, 12);
        vmax = std::max(vmax, pl.vcell()); vmin = std::min(vmin, pl.vcell());
    }
    vend = pl.vcell();
    printf("  offset: cell %.3f..%.3f (end %.3f) over %d s, iout %.1f/%.1f A, lim %.1f/%.1f A\n", vmin, vmax, vend, s,
           pl.iout[0], pl.iout[1], a.Iout_max(), b.Iout_max());
    CHECK(!(bool) a.termCond && !(bool) b.termCond, "no ceiling stop after %d s (cell %.3f)", s, vend);
    CHECK(vmax <= 3.5f + 0.015f && vmin >= 3.5f - 0.015f, "offset: cell held at cv_eoc +- 15 mV: %.3f..%.3f", vmin, vmax);
}

// a reboot mid-hold must not restart the session cap (cell just below the band: no progress)
static void test_bal_cap_survives_reboot() {
    ChargerRtc r{};
    setWallClockUs(1'000'000ULL);
    uint32_t total = 0;
    bool capped = false;
    for (int boot = 0; boot < 3 && !capped; ++boot) {
        BatteryCharger c; c.params = balParams(1800);
        c.attachRtc(&r);
        c.termCond.reset();
        if (!boot) c.balanceNow();
        enterHold(c);
        for (int n = 0; n < 500 && c.termCond.cvHold(); ++n) { frames(c, 3.47f, 10.f, 1, true); total += 4; }
        capped = (bool) c.termCond;
        CHECK(c.balanceInBandS() < 30, "below band: little progress (%u s)", c.balanceInBandS());
    }
    CHECK(capped && total <= 3600 + 60, "session cap across reboots after %u s in the hold", total);
}

// a low-delta streak must not span a gap without ticks
static void test_bal_delta_gap() {
    BatteryCharger c; c.params = balParams(7200); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    c.balanceNow();
    enterHold(c);
    for (int n = 0; n < 2; ++n) { c.batSt.setCellDelta(0.005f); frames(c, 3.50f, 10.f, 1, true); }
    setWallClockUs(loopWallClockUs_ + 120'000'000ULL);
    c.batSt.setCellDelta(0.005f);
    frames(c, 3.50f, 10.f, 1, true);
    CHECK(c.balanceDue(), "2 frames, 120 s gap, 1 frame: not done");
}

// a configured temperature sensor that went stale defers (no sensor configured: no deferral)
static void test_bal_temp_stale() {
    BatteryCharger c; c.params = balParams(1800); setWallClockUs(1'000'000ULL);
    ConfFile m;
    m.set("bat_temp_topic", "bms/t1");
    c.beginMqtt(m);
    c.termCond.reset();
    c.balanceNow();
    c.batSt.setTemp(0, 40.f);
    frames(c, 3.40f, 10.f, 2, true);
    CHECK(c.balanceDeferWhy() != nullptr, "40 C: deferred");
    frames(c, 3.40f, 10.f, 1000, true); // > 1 h without a temperature frame
    CHECK(!c.haveTemp() && c.balanceDeferWhy() != nullptr, "configured sensor stale: still deferred");
    enterHold(c);
    CHECK(!c.balanceRunning(), "no run with a stale temperature");
}

// a 64-bit wall clock read torn on core 0 must not make the run due (balance timestamps use the tick clock)
static void test_bal_torn_clock() {
    BatteryCharger c; c.params = balParams(1800); setWallClockUs(1'000'000ULL);
    tick(c);
    c.balanceSkip();
    time_us t = loopWallClockUs_ + 1'000'000ULL;
    setWallClockUs(t);
    loopWallClockUs_ = t + (1ULL << 32); // torn: high word one ahead, ticks right
    c.update(27.5f, 0.f, true);
    loopWallClockUs_ = t;
    tick(c);
    CHECK(!c.balanceDue(), "torn us read during skip: not due (age %u)", c.balanceAgeS());
}

// BMS frames stop mid-hold for gapS (11-23: 150-211 s gaps): the limit decays instead of freezing open loop,
// the expiry latch is unchanged, a short gap resumes from the decayed limit without overshoot. units: 1 or 2 on a bus.
static void test_hold_blind_decay(int units, int gapS) {
    BatteryCharger a, b; a.params = holdParams(); b.params = holdParams(); setWallClockUs(1'000'000ULL);
    a.termCond.reset(); b.termCond.reset();
    Plant pl;
    if (units == 2) pl.ipv[0] = pl.ipv[1] = 25.f;
    BatteryCharger *u[2] = {&a, &b};
    for (int k = 0; k < 3600 && !(a.termCond.cvHold() && (units == 1 || b.termCond.cvHold())); ++k) pl.run(u, units, 1);
    pl.run(u, units, 120);
    CHECK(a.termCond.cvHold() && !(bool) a.termCond, "blind %d/%d: in the hold", units, gapS);
    float lim0 = a.Iout_max(), ibat0 = pl.ibat;
    pl.bms.cells = pl.bms.ibatOn = false;
    pl.run(u, units, 18);
    CHECK(a.Iout_max() >= lim0 - 0.01f, "blind %d/%d: 18 s, limit kept %.2f -> %.2f A", units, gapS, lim0, a.Iout_max());
    float limMin = 99.f, ibatEnd = 0.f;
    int latchedAt = -1;
    for (int s = 18; s < gapS; ++s) {
        pl.run(u, units, 1);
        if (s == 100) ibatEnd = pl.ibat, limMin = a.Iout_max();
        if (latchedAt < 0 && (bool) a.termCond) latchedAt = s;
    }
    printf("  blind %d unit(s), %d s: ibat %.1f -> %.1f A, lim %.1f -> %.2f A at 100 s, latched at %d s\n", units, gapS,
           ibat0, ibatEnd, lim0, limMin, latchedAt);
    CHECK(limMin < 1.f && ibatEnd < 1.f * units, "blind %d/%d: decayed below 1 A by 100 s (%.2f A, ibat %.1f A)",
          units, gapS, limMin, ibatEnd);
    if (gapS > 180) {
        CHECK(latchedAt >= 175 && latchedAt <= 200, "blind %d/%d: expiry latch as before (%d s)", units, gapS, latchedAt);
        return;
    }
    CHECK(latchedAt < 0 && a.termCond.cvHold(), "blind %d/%d: short gap, still in the hold", units, gapS);
    pl.bms.cells = pl.bms.ibatOn = true;
    float vmax = 0;
    int back = -1;
    for (int s = 0; s < 900; ++s) {
        pl.run(u, units, 1);
        vmax = std::max(vmax, pl.vcell());
        if (back < 0 && pl.vcell() >= 3.5f - 0.005f) back = s;
    }
    printf("  blind %d unit(s) resume: cell at cv_eoc after %d s, max %.3f, ilim %.1f A\n", units, back, vmax, a.Iout_max());
    CHECK(vmax <= 3.5f + 0.015f && back >= 0 && back < 600 && !(bool) a.termCond,
          "blind %d/%d: resumed from the decayed limit, max %.3f, back after %d s", units, gapS, vmax, back);
}

// outside the hold a BMS gap does not touch the output limit
static void test_bulk_blind_untouched() {
    BatteryCharger c; c.params = holdParams(); setWallClockUs(1'000'000ULL);
    c.termCond.reset();
    frames(c, 3.40f, 30.f, 10, true);
    for (int s = 0; s < 120; ++s) tick(c);
    CHECK(!c.termCond.cvHold() && c.Iout_max() >= 40.f - 1e-3f, "bulk, BMS gap 120 s: no limit (%.1f A)", c.Iout_max());
}

// a ceiling latch in the approach (before the hold) defers the run; the wait survives a restart (RTC)
static void test_bal_ceiling_approach_rtc() {
    ChargerRtc r{};
    BatteryCharger c; c.params = balParams(1800); setWallClockUs(1'000'000ULL);
    c.attachRtc(&r);
    c.termCond.reset();
    c.balanceNow();
    frames(c, 3.40f, 30.f, 10, true);
    frames(c, 3.48f, 30.f, 2, true);
    CHECK(!c.termCond.cvHold() && c.Iout_max() < 30.f, "approach: no hold, limit cut (%.1f A)", c.Iout_max());
    frames(c, 3.56f, 30.f, 2, true);
    CHECK((bool) c.termCond && c.balanceDeferWhy() && !strcmp(c.balanceDeferWhy(), "ceiling"),
          "ceiling in the approach: deferred");
    CHECK(r.ceilValid() && r.balCeilS < 30, "ceiling wait saved (%u s)", r.balCeilS);
    frames(c, 3.40f, 10.f, 900, true); // 1 h
    BatteryCharger d; d.params = balParams(1800);
    setWallClockUs(loopWallClockUs_ + 1'000'000ULL);
    d.attachRtc(&r);
    d.termCond.reset();
    tick(d);
    CHECK(d.balanceDue() && d.balanceDeferWhy() && !strcmp(d.balanceDeferWhy(), "ceiling"), "restart: still deferred");
    setWallClockUs(loopWallClockUs_ + (30ULL * 86400ULL / 4 - 3600) * 1000000ULL);
    frames(d, 3.40f, 10.f, 10, true);
    CHECK(d.balanceDeferWhy() == nullptr, "restored wait ends at interval / 4 after the stop (%s)", d.balanceDeferWhy());
    frames(d, 3.56f, 10.f, 2, true);
    CHECK(d.balanceDeferWhy() != nullptr, "ceiling again: deferred");
    d.balanceNow();
    tick(d);
    CHECK(d.balanceDeferWhy() == nullptr && r.ceilValid() && r.balCeilS == UINT32_MAX, "balance now: cleared and saved");

    // a 40 B build (older) rewrote the balance part and left a stale ceiling tail: ignored
    ChargerRtc o = r;
    o.balCeilS = 10;
    o.balCeilSum = o.ceilChecksum();
    o.balInBandS += 1;
    o.balSum = o.balChecksum();
    BatteryCharger e; e.params = balParams(1800);
    e.attachRtc(&o);
    e.termCond.reset();
    frames(e, 3.40f, 10.f, 10, true);
    CHECK(e.balanceDeferWhy() == nullptr && e.balanceDue(), "stale ceiling tail ignored, balance part kept");
}

// 11-23 12:58: the BMS re-seeded its remaining charge to 0.9 x 245 Ah after a 69 s data gap; the
// deficit jumped 19.8 -> 56.3 Ah and released termination by DoD (threshold 42 Ah).
static void test_bms_charge_reseed_no_release() {
    BatteryCharger c; c.params = params(); setWallClockUs(1'000'000ULL);
    float q = 276.8f, scale = 0.f;
    auto bframe = [&](float vcell, float ibat) {
        setWallClockUs(loopWallClockUs_ + 4'000'000ULL);
        q += ibat * 4.f / 3600.f;
        c.batSt.coulombCounter.setBmsCharge(q + scale, wallClockTicks());
        c.batSt.setVcellHigh(vcell);
        c.batSt.updateBatCurrent(ibat);
        c.update(26.7f, 0.f, true);
    };
    c.termCond.reset();
    for (int i = 0; i < 20; ++i) bframe(c.params.cv_min + 0.10f, 0.f);
    CHECK((bool) c.termCond && c.batSt.coulombCounter.bmsSource(), "latched on the BMS charge");
    while (q > 257.02f) bframe(c.params.cv_min + 0.02f, -3.f);
    setWallClockUs(loopWallClockUs_ + 69'000'000ULL);
    q -= 3.f * 69.f / 3600.f;
    scale = 220.5f - q;
    for (int i = 0; i < 75; ++i) bframe(c.params.cv_min + 0.02f, -3.f);
    float d = c.batSt.coulombCounter.deficit();
    printf("  re-seed: deficit %.2f Ah, true %.2f Ah, terminated %d\n", d, 276.8f - q, (int) (bool) c.termCond);
    CHECK((bool) c.termCond, "no release due to DoD after the BMS re-seed");
    CHECK(std::fabs(d - (276.8f - q)) < 0.3f, "deficit follows the true charge (%.2f vs %.2f)", d, 276.8f - q);
}

int main() {
    test_hold_blind_decay(1, 120);
    test_hold_blind_decay(2, 120);
    test_hold_blind_decay(1, 240);
    test_bulk_blind_untouched();
    test_bal_ceiling_approach_rtc();
    test_hold_vout_offset_two_units();
    test_bal_cap_survives_reboot();
    test_bal_delta_gap();
    test_bal_temp_stale();
    test_bal_torn_clock();
    test_bal_two_units();
    test_bal_not_due_normal();
    test_bal_completes();
    test_bal_progress_survives_sun_gone_and_cap();
    test_bal_delta();
    test_bal_defer();
    test_bal_ceiling_and_stale();
    test_bal_stale_gap(240);
    test_bal_stale_gap(1200);
    test_bal_stale_gap(240, false);
    test_bal_partial_hold_suspended();
    test_bal_rtc();
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
    test_hold_two_units_converge(false);
    test_hold_two_units_converge(true);
    test_hold_psu_freezes_pin();
    test_hold_sweep_keeps_limit(true);
    test_hold_sweep_keeps_limit(false);
    test_hold_pv_limited_recovers();
    test_hold_stale_releases_limit();
    test_hold_entry_40a_two_units();
    test_hold_sibling_boots_unlimited();
    test_boot_idle_without_ibat();
    test_bms_charge_reseed_no_release();
    printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
    return g_fail ? 1 : 0;
}
