#include <unity.h>
#include <array>
#include <memory>
#include <esp_log.h>
#include "util.h"   // loopWallClockUs_, wallClockMs() — tracker.h relies on its includer for these
#include "tracker.h"
#include "peak_scan.h"
#include "sim/vconv.h"
#include <algorithm>

// Tracker embeds two 2048-entry tables (~16 KB) — far over the task stack, so heap-allocate it
// (the firmware holds it as a static MpptController member, never on the stack).

// The Tracker is time-gated: a perturbation ("tick") only happens when
// now - _time > 1000/frequency. wallClockMs() reads loopWallClockUs_, which the
// test build leaves at 0, so we drive it here. Each step advances 100 ms, comfortably
// above the 50 ms (freq=20) / 33 ms (freq=30, normal) tick period, so every update ticks.
// All times stay < 30 s, keeping the tracker in normal mode (slow-mode needs no-new-max
// for >30 s). vin=0 neutralises the cloud-recovery branch (abs(0-0)/0.1 < 5%) so these
// tests isolate the pure power-based perturb&observe; the vin path has its own test.

static void advanceMs(unsigned long ms) { loopWallClockUs_ += ms * 1000ULL; }

static float kSettleMs() { return 150; } // power-filter settling (fmetal)

// power < 1: tracker forces direction=true (pump more power) regardless of prior direction.
void test_tracker_low_power_forces_pump() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(0.f, /*direction=*/false);
    advanceMs(100);
    float step = t.update(0.5f, 100, 0.f, kSettleMs);
    TEST_ASSERT_TRUE(t._direction);
    TEST_ASSERT_TRUE(step > 0.f);
}

// dP < 0 beyond the deadband -> reverse perturbation direction.
void test_tracker_reverses_on_power_drop() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    advanceMs(100);
    float step = t.update(90.f, 100, 0.f, kSettleMs);   // dP = -10
    TEST_ASSERT_FALSE(t._direction);
    TEST_ASSERT_TRUE(step < 0.f);
}

// dP > 0: keep climbing in the same direction.
void test_tracker_keeps_direction_on_power_rise() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    advanceMs(100);
    float step = t.update(110.f, 100, 0.f, kSettleMs);  // dP = +10
    TEST_ASSERT_TRUE(t._direction);
    TEST_ASSERT_TRUE(step > 0.f);
}

// A drop smaller than minPowerStep (1.0) and below minPowerStepRel must not reverse.
void test_tracker_deadband_ignores_small_drop() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    advanceMs(100);
    float step = t.update(99.5f, 100, 0.f, kSettleMs);  // dP=-0.5: abs<1.0 and 0.5/105 < 0.015
    TEST_ASSERT_TRUE(t._direction);
    TEST_ASSERT_TRUE(step > 0.f);
}

// Cloud recovery: a >5% Vin change since the last reversal flips direction even when
// power is rising (which on its own would not reverse).
void test_tracker_cloud_recovery_reverses_on_vin_jump() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    advanceMs(100);
    t.update(100.f, 100, 30.f, kSettleMs);  // primes _lastVin (0->30 reverses once); dir now false
    advanceMs(100);
    t.update(105.f, 100, 30.f, kSettleMs);  // vin stable, power up -> no reversal, dir stays false
    TEST_ASSERT_FALSE(t._direction);
    advanceMs(100);
    t.update(106.f, 100, 33.f, kSettleMs);  // vin 30->33 = ~10% -> reverse despite rising power
    TEST_ASSERT_TRUE(t._direction);
}

// Closed-loop hill-climb: drive a synthetic parabolic P(duty) peaking at duty=1000,
// applying the sign of the returned step to the duty each tick. The tracker must climb
// out of the zero-power region (power<1 forces pump) and settle near the peak.
void test_tracker_converges_to_peak() {
    loopWallClockUs_ = 1'000'000;
    const int peak = 1000;
    auto P = [&](int d) {
        float x = (float) (d - peak) / 400.f;
        float p = 200.f * (1.f - x * x);
        return p < 0.f ? 0.f : p;
    };
    int duty = 200;                       // start ~800 below the peak, in the zero-power region
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(P(duty), /*direction=*/true);
    for (int i = 0; i < 140; ++i) {
        advanceMs(100);
        float step = t.update(P(duty), (uint16_t) duty, 0.f, kSettleMs);
        duty += (step > 0.f ? 1 : -1) * 8;
        if (duty < 0) duty = 0;
        if (duty > 2047) duty = 2047;
    }
    // converged near the peak (deadband near the flat top bounds the residual dither)
    TEST_ASSERT_INT_WITHIN(120, peak, duty);
}

// Within 150 ms of a reversal the filtered power still reflects the old direction: no power-based
// decision, the reference follows the power instead.
void test_tracker_settles_after_reversal() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    advanceMs(100);
    t.update(90.f, 100, 0.f, kSettleMs);   // reverse
    TEST_ASSERT_FALSE(t._direction);
    advanceMs(50);
    t.update(80.f, 100, 0.f, kSettleMs);   // drop inside the settle window: ignored
    TEST_ASSERT_FALSE(t._direction);
    TEST_ASSERT_EQUAL_FLOAT(80.f, t._lastPower);
    advanceMs(200);
    t.update(70.f, 100, 0.f, kSettleMs);   // drop against the settled reference
    TEST_ASSERT_TRUE(t._direction);
}

// The window is the caller's filter settling time, not a constant.
void test_tracker_settle_window_follows_filter() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    advanceMs(100);
    t.update(90.f, 100, 0.f, [] { return 400.f; });   // reverse
    advanceMs(300);
    t.update(80.f, 100, 0.f, [] { return 400.f; });   // 300 ms < 400 ms: ignored
    TEST_ASSERT_FALSE(t._direction);
    advanceMs(200);
    t.update(70.f, 100, 0.f, [] { return 400.f; });
    TEST_ASSERT_TRUE(t._direction);
}

// The window lasts until Vin is back at the reversal point (the input cap lags the duty), at most 1 s.
void test_tracker_settle_waits_for_vin() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    advanceMs(100);
    t.update(90.f, 100, 50.f, [] { return 0.f; });    // reverse at Vin 50, Vin should rise now
    TEST_ASSERT_FALSE(t._direction);
    advanceMs(100);
    t.update(80.f, 100, 49.f, [] { return 0.f; });    // Vin still falling: ignored
    TEST_ASSERT_FALSE(t._direction);
    advanceMs(100);
    t.update(70.f, 100, 50.2f, [] { return 0.f; });   // Vin back: decide
    TEST_ASSERT_TRUE(t._direction);
    // Vin never comes back: decisions resume after 1 s
    advanceMs(500);
    t.update(60.f, 100, 50.5f, [] { return 0.f; });
    TEST_ASSERT_TRUE(t._direction);
    advanceMs(600);
    t.update(50.f, 100, 50.5f, [] { return 0.f; });
    TEST_ASSERT_FALSE(t._direction);
}

// Slow mode has no settle window: a window could only end at a 500 ms decision and would skip it, whether held
// by Vin or by a slow power filter.
static void slowModeDecidesAfterReversal(float settleMs) {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    t.maxPowerPoint = {200.f, wallClockMs(), 100, 50.f};
    t.setSlowMode(true);
    auto settle = [settleMs] { return settleMs; };
    advanceMs(600);
    t.update(90.f, 100, 50.f, settle);    // reverse at Vin 50
    TEST_ASSERT_FALSE(t._direction);
    advanceMs(600);
    t.update(80.f, 100, 49.f, settle);    // Vin not back, filter not settled: slow mode decides anyway
    TEST_ASSERT_TRUE(t.slowMode);
    TEST_ASSERT_TRUE(t._direction);
}

void test_tracker_slow_mode_settle_ignores_vin() { slowModeDecidesAfterReversal(0.f); }

void test_tracker_slow_mode_settle_ignores_filter() { slowModeDecidesAfterReversal(900.f); }

// A limiter hand-back (resetDirection) starts the window like a reversal.
void test_tracker_handback_starts_window() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    advanceMs(2000);
    t.resetDirection(false, 50.f);
    advanceMs(100);
    t.update(80.f, 100, 50.f, kSettleMs);   // stale reference, inside the window: ignored
    TEST_ASSERT_FALSE(t._direction);
    TEST_ASSERT_EQUAL_FLOAT(80.f, t._lastPower);
    advanceMs(100);
    t.update(70.f, 100, 50.f, kSettleMs);
    TEST_ASSERT_TRUE(t._direction);
}

// The 5 % Vin band is 5 % of its higher end in both directions, so a down leg followed by an up leg
// returns to the start instead of ratcheting down by 0.25 % per cycle.
void test_tracker_vin_band_symmetric() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(10.f, /*direction=*/true);
    auto step = [&](float vin) { advanceMs(200); t.update(10.f, 100, vin, kSettleMs); return t._direction; };
    bool d = step(40.f);                    // primes _lastVin (0 -> 40 reverses)
    TEST_ASSERT_TRUE(d == step(38.1f));     // 1.9 V < 5 % of 40
    TEST_ASSERT_TRUE(d != step(37.9f));
    d = t._direction;
    TEST_ASSERT_TRUE(d == step(39.85f));    // 1.95 V < 5 % of 39.85 (5 % of 37.9 would reverse here)
    TEST_ASSERT_TRUE(d != step(39.95f));
}

// A long limit (derate, CV hold) is not a reversal: on release near a stale MPP the tracker stays in fast mode,
// and a NaN Vin at hand-back keeps the Vin reference.
void test_tracker_long_limit_release_stays_fast() {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.resetTracker(100.f, /*direction=*/true);
    for (int i = 0; i <= 20; ++i) {
        advanceMs(100);
        t.update(100.f + (float) i, 100, 50.f, kSettleMs); // climbing: new MPP each period
    }
    for (int i = 0; i < 1200; ++i) { // 2 min limiter
        advanceMs(100);
        t.resetDirection(false, i == 1199 ? NAN : 50.f);
    }
    TEST_ASSERT_EQUAL_FLOAT(50.f, t._lastVin);
    advanceMs(100);
    t.update(118.f, 100, 50.f, kSettleMs); // within 15 % of the 2 min old MPP
    TEST_ASSERT_FALSE(t.slowMode);
}

// Steady 98 W at Vin 50 after a takeover at 1.0 s: MPP record 100 W at duty 800 (1.1 s), a reversal at 2.1 s.
static void toSteady(Tracker &t) {
    loopWallClockUs_ = 1'000'000;
    t.resetTracker(0.f, true);
    t.resetDirection(true, 50.f);
    advanceMs(100);
    t.update(100.f, 800, 50.f, kSettleMs);
    advanceMs(1000);
    t.update(98.f, 810, 50.f, kSettleMs);
}

// Ticks every 100 ms up to toMs (absolute); returns the value of the slow-mode entry tick, else NAN.
static float runTo(Tracker &t, uint64_t toMs, float p = 98.f) {
    float entry = NAN;
    while (loopWallClockUs_ < toMs * 1000) {
        advanceMs(100);
        bool s = t.slowMode;
        float r = t.update(p, 810, 50.f, kSettleMs);
        if (t.slowMode && !s) entry = r;
    }
    return entry;
}

// Slow mode after 30 s without a new MPP; the entry returns the distance [counts] to the MPP duty.
void test_tracker_slow_entry_after_30s() {
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    toSteady(t);
    TEST_ASSERT_TRUE(t._timeLastReverse == 2100);
    TEST_ASSERT_TRUE(std::isnan(runTo(t, 30'900)));
    TEST_ASSERT_FALSE(t.slowMode);
    TEST_ASSERT_EQUAL_FLOAT(-10.f, runTo(t, 31'500));
    TEST_ASSERT_TRUE(t.slowMode);
    TEST_ASSERT_EQUAL_FLOAT(2.f, t.frequency);
}

// No slow mode without a reversal after the takeover; a hand-back restarts the 30 s wait and needs a new reversal.
void test_tracker_slow_entry_needs_reversal_after_takeover() {
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    loopWallClockUs_ = 1'000'000;
    t.resetTracker(0.f, true);
    t.resetDirection(true, 50.f);
    runTo(t, 40'000, 100.f); // no reversal
    TEST_ASSERT_FALSE(t.slowMode);
    t.resetDirection(false, 50.f); // hand-back at 40 s: not a reversal
    runTo(t, 42'000, 98.f); // inside the settle window: no reversal
    runTo(t, 42'100, 96.f); // reversal inside the 30 s wait
    TEST_ASSERT_TRUE(t._timeLastReverse > t._timeTakeover);
    runTo(t, 70'000, 96.f);
    TEST_ASSERT_FALSE(t.slowMode);
    runTo(t, 70'200, 96.f);
    TEST_ASSERT_TRUE(t.slowMode);
}

// A hand-back (limiter release, end of the fade after a sweep, start-up) ends slow mode; re-entry needs a
// reversal and 30 s after the last hand-back.
void test_tracker_handback_ends_slow_mode() {
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    toSteady(t);
    runTo(t, 32'000);
    TEST_ASSERT_TRUE(t.slowMode);
    t.resetDirection(true, 50.f); // limiter hand-back
    TEST_ASSERT_FALSE(t.slowMode);
    TEST_ASSERT_EQUAL_FLOAT(30.f, t.frequency);
    for (int i = 0; i < 50; ++i) { // fade to the sweep MPP: hand-back every tick for 5 s
        advanceMs(100);
        t.resetDirection(true, 50.f);
    }
    runTo(t, 38'000); // no reversal yet
    TEST_ASSERT_FALSE(t.slowMode);
    runTo(t, 39'000, 96.f); // reversal at 38.1 s
    runTo(t, 67'000, 96.f); // 30 s after the last hand-back (37 s) not yet passed
    TEST_ASSERT_FALSE(t.slowMode);
    runTo(t, 68'000, 96.f);
    TEST_ASSERT_TRUE(t.slowMode);
}

// The MPP record reset (below 85 % once 30 s old) is evaluated before the entry: the reset and the new record in
// the same period keep the tracker in fast mode.
void test_tracker_reset_before_entry() {
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    toSteady(t);
    runTo(t, 40'000, 80.f); // the drop falls into the settle window of the 2.1 s reversal: no new reversal
    TEST_ASSERT_FALSE(t.slowMode);
    TEST_ASSERT_EQUAL_FLOAT(80.f, t.maxPowerPoint.power);
    TEST_ASSERT_TRUE(t.maxPowerPoint.timestamp > 31'000);
}

// Slow mode ends 45 s after the last reversal.
void test_tracker_slow_mode_ends_45s_after_reversal() {
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    toSteady(t); // reversal at 2.1 s
    runTo(t, 46'500);
    TEST_ASSERT_TRUE(t.slowMode);
    runTo(t, 48'000);
    TEST_ASSERT_FALSE(t.slowMode);
    TEST_ASSERT_TRUE(std::isnan(runTo(t, 100'000))); // no re-entry without a reversal
}

// ---- partial global-peak scan (peak_scan.h) ----

// fry-like string: 6 substrings (bypass diode 0.45 V), Voc 80 V, Vmpp 68.8 V; ideal CCM buck on 26 V: Vin = 26 V * 2047 / d
struct ScanRig {
    VirtualConverter pv;
    PeakScan s;
    float g[6] = {1, 1, 1, 1, 1, 1}, light = 1, d;
    uint32_t now = 1000;
    int active = 0, ended = 0;

    explicit ScanRig(float vStart, uint8_t n = 6) {
        pv.setPv(4.f, 80.f, 0.86f);
        shade();
        d = 26.f * 2047 / vStart;
        s.n = n, s.interval = 20;
        s.onSweep(80.f, now);
    }
    void shade() {
        float x[6];
        for (int i = 0; i < 6; ++i) x[i] = g[i] * light;
        pv.setPvString(6, x, nullptr, 0.45f);
    }
    // the sweep's current samples, Voc down to the battery
    void sweep() {
        for (float v = 80; v > 29; v -= 0.5f) s.sweepSample(pv.pvCurrent(v));
    }
    [[nodiscard]] float vin() const { return 26.f * 2047 / d; }
    [[nodiscard]] float p() const { return vin() * pv.pvCurrent(vin()); }
    template<class F>
    void run(uint32_t ms, F f) {
        for (uint32_t end = now + ms; now < end;) {
            now += 2;
            f(*this);
            const float t = s.update(now, vin(), vin(), p(), (uint16_t) d, 29.f, 5.f, [] { return 50.f; });
            if (!std::isnan(t)) d += std::clamp((t - d) * 0.2f, -10.f, 10.f), ++active;
            if (s.ended) ended = s.ended, s.ended = 0;
        }
    }
    void run(uint32_t ms) { run(ms, [](ScanRig &) {}); }
};

// Known-bad case: on the full-string peak while one shaded substring makes the 57 V peak 18 % better. Without the
// scan the operating point stays; the scan moves to the better peak.
void test_scan_moves_to_better_peak() {
    auto off = std::make_unique<ScanRig>(68.8f, 0);
    off->g[5] = 0.7f, off->shade();
    off->run(30'000);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 68.8f, off->vin());

    auto r = std::make_unique<ScanRig>(68.8f);
    r->g[5] = 0.7f, r->shade(), r->sweep();
    const float p0 = r->p();
    r->run(30'000);
    ESP_LOGI("test", "scan: %.1f W at 68.8 V -> %.1f W at %.1f V", p0, r->p(), r->vin());
    TEST_ASSERT_TRUE(r->active > 0);
    TEST_ASSERT_FLOAT_WITHIN(3.f, 55.5f, r->vin());
    TEST_ASSERT_TRUE(r->p() > 1.1f * p0);
}

// On the global peak already: probes, returns and stays.
void test_scan_stays_on_global_peak() {
    auto r = std::make_unique<ScanRig>(56.9f);
    r->g[5] = 0.7f, r->shade(), r->sweep();
    r->run(30'000);
    TEST_ASSERT_TRUE(r->active > 0);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 56.9f, r->vin());
}

// Light rises 40 % while the worse peak is probed: compared with the start it looks 15 % better. The power at the
// start point measured again on return shows the change, so the scan stays.
void test_scan_ignores_light_change() {
    auto r = std::make_unique<ScanRig>(56.9f);
    r->g[5] = 0.7f, r->shade(), r->sweep();
    r->run(30'000, [](ScanRig &x) {
        if (x.s.phase == PeakScan::Probe && x.light == 1) x.light = 1.4f, x.shade();
    });
    TEST_ASSERT_EQUAL_FLOAT(1.4f, r->light);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 56.9f, r->vin());
}

// Clear sky on the full-string peak: no lower peak can beat it at the current seen in the sweep, so no probing.
void test_scan_skips_clear_sky() {
    auto r = std::make_unique<ScanRig>(68.8f);
    r->sweep();
    r->run(30'000);
    TEST_ASSERT_EQUAL_INT(0, r->active);
    TEST_ASSERT_TRUE(r->s.pRef > 0);
}

// Trigger only (no period): a flickering cloud keeps the power off the reference but never steady, so no scan;
// once the power settles at the new level, the scan runs.
void test_scan_trigger_waits_for_steady_power() {
    auto r = std::make_unique<ScanRig>(56.9f);
    r->s.interval = 0;
    r->g[5] = 0.7f, r->shade(), r->sweep();
    r->run(6'000);
    TEST_ASSERT_TRUE(r->s.pRef > 0);
    r->run(90'000, [](ScanRig &x) {
        const float l = (x.now / 2000) % 2 ? 0.5f : 0.65f;
        if (l != x.light) x.light = l, x.shade();
    });
    TEST_ASSERT_EQUAL_INT(0, r->active);
    r->light = 0.6f, r->shade();
    r->run(30'000);
    TEST_ASSERT_TRUE(r->active > 0);
}

// Global peak at 56.9 V, the full-string probe 9 % worse. Light rises 14 % while it is probed and stays: against the
// start-point power before the probe it looks 4 % better. The reference is the larger of the two start-point
// measurements, so the scan stays.
void test_scan_reference_catches_light_ramp() {
    auto r = std::make_unique<ScanRig>(56.9f);
    r->g[5] = 0.81f, r->shade(), r->sweep();
    uint32_t tP = 0;
    r->run(30'000, [&tP](ScanRig &x) {
        if (x.s.phase == PeakScan::Probe && !tP) tP = x.now;
        if (tP && x.light < 1.14f) x.light = std::min(1.14f, 1 + 0.14f * (float) (x.now - tP) / 200.f), x.shade();
    });
    TEST_ASSERT_EQUAL_FLOAT(1.14f, r->light);
    TEST_ASSERT_TRUE(r->active > 0);
    TEST_ASSERT_EQUAL_INT(1, r->ended);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 56.9f, r->vin());
}

// A better peak (+18 %), but the light steps up 30 % during the probe: the start point measured again differs by
// more than 15 %, so the scan keeps the start point and reports the light change.
void test_scan_aborts_on_light_step() {
    auto r = std::make_unique<ScanRig>(68.8f);
    r->g[5] = 0.7f, r->shade(), r->sweep();
    r->run(30'000, [](ScanRig &x) {
        if (x.s.phase == PeakScan::Probe && x.light == 1) x.light = 1.3f, x.shade();
    });
    TEST_ASSERT_EQUAL_INT(3, r->ended);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 68.8f, r->vin());
}

// Trigger only: the power off the reference for 12 s (steady) does not scan, for 20 s it does.
void test_scan_trigger_needs_15s() {
    auto r = std::make_unique<ScanRig>(56.9f);
    r->s.interval = 0;
    r->g[5] = 0.7f, r->shade(), r->sweep();
    r->run(61'000);
    TEST_ASSERT_TRUE(r->s.pRef > 0);
    r->light = 0.6f, r->shade();
    r->run(12'000);
    r->light = 1, r->shade();
    r->run(10'000);
    TEST_ASSERT_EQUAL_INT(0, r->active);
    r->light = 0.6f, r->shade();
    r->run(20'000);
    TEST_ASSERT_TRUE(r->active > 0);
}

// Voc at the sweep read 12 % low: the substring share also follows the operating point (Vin / m0), so the probe
// still lands on the 57 V peak's flat side and the scan moves there from the 68.8 V one.
void test_scan_probe_anchor_on_vin() {
    auto r = std::make_unique<ScanRig>(68.8f);
    r->s.onSweep(70.f, r->now);
    r->g[5] = 0.7f, r->shade(), r->sweep();
    const float p0 = r->p();
    r->run(30'000);
    TEST_ASSERT_EQUAL_INT(2, r->ended);
    TEST_ASSERT_TRUE(r->vin() > 49 && r->vin() < 57);
    TEST_ASSERT_TRUE(r->p() > 1.03f * p0);
}

// Vmpp / Voc from the sweep MPP: an unshaded string's ratio is taken, a shaded maximum's (outside 0.74..0.86) is not.
void test_scan_learns_vmpp_ratio() {
    PeakScan s{};
    s.onSweep(80.f, 0);
    s.onSweepMpp(62.4f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.78f, s.kVmp);
    s.onSweepMpp(74.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.78f, s.kVmp);
    s.onSweepMpp(45.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.78f, s.kVmp);
}

// A scan stuck for 30 s (a stalled loop) ends with ended == 4 and the start duty, and stops steering.
void test_scan_timeout_returns_to_start() {
    auto r = std::make_unique<ScanRig>(68.8f);
    r->g[5] = 0.7f, r->shade(), r->sweep();
    const uint16_t dStart = (uint16_t) r->d;
    r->run(60'000, [](ScanRig &x) { if (x.s.phase == PeakScan::Probe) x.now += 31'000; });
    TEST_ASSERT_EQUAL_INT(4, r->ended);
    TEST_ASSERT_EQUAL_INT((int) dStart, (int) r->s.d0);
    TEST_ASSERT_TRUE(r->s.pRef <= 0);
}
