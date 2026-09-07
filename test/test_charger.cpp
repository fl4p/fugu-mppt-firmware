// Tests for src/charger.h (BatChargerParams, BatteryState, Li_ChgTerminationCondition,
// BatteryCharger MQTT callbacks) and src/etc/coulomb_counter.h.
//
// Termination/coulomb tests are pure-function — they construct the units under test
// directly and never touch MQTT.
//
// MQTT-callback tests synthesise messages via MqttService::_invokeForTest, which
// delivers a buffer to the registered handler without going through the broker.
// Tests use distinct topic names per logical channel but rely on subscribeTopic's
// overwrite semantics across tests (each test re-subscribes its own handler).

#include <unity.h>
#include <Arduino.h>

#include <cstring>
#include <unordered_map>

#include "charger.h"
#include "tele/mqtt.h"

// ------------------------------------------------------------------
//  Helpers
// ------------------------------------------------------------------

static BatChargerParams makeLfpParams() {
    BatChargerParams p;
    p.Vbat_max = 14.6f;        // 4S LFP
    p.Vbat_fallback = 13.4f;
    p.cv_eoc = 3.65f;
    p.cv_min = 3.37f;
    p.n_cells = 4;             // must match Vbat_max / cv_eoc
    p.Cbat = 280.0f;           // 280 Ah pack
    p.tail_c_rate = 0.05f;
    p.Ibat_lim = 40.0f;
    p.recharge_dod = 0.20f;
    return p;
}

// Drive the integrator with a constant current over a duration in 10s steps so we
// stay well under TrapezoidalIntegrator's 30s maxDt guard.
static void driveCounter(CoulombCounter &cc, float ibat,
                         time_us startUs, time_us endUs,
                         time_us stepUs = 10'000'000ULL) {
    for (time_us t = startUs; t <= endUs; t += stepUs) cc.updateBatCurrent(ibat, t);
}

// The line trigger latches only after two consecutive over-line frames (like the ceiling).
static void updateFrames(Li_ChgTerminationCondition &tc, float vcell, float ibat, float ah, int n = 2) {
    for (int i = 0; i < n; ++i) tc.update(vcell, ibat, ah);
}

// ------------------------------------------------------------------
//  Li_ChgTerminationCondition — termination-line math
// ------------------------------------------------------------------

void test_termination_line_at_zero_current() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    tc.update(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.cv_min, tc.v_term());
}

void test_termination_line_at_tail_current() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    const float tailA = p.tail_c_rate * p.Cbat; // 14 A
    tc.update(0.0f, tailA, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.cv_eoc, tc.v_term());
}

void test_termination_line_clamps_above_cv_eoc() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    const float tailA = p.tail_c_rate * p.Cbat;
    tc.update(0.0f, 2.0f * tailA, 0.0f); // 2× tail — line would be above cv_eoc
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.cv_eoc, tc.v_term());
}

void test_termination_line_midpoint() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    const float halfTailA = 0.5f * p.tail_c_rate * p.Cbat;
    tc.update(0.0f, halfTailA, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f * (p.cv_min + p.cv_eoc), tc.v_term());
}

// ------------------------------------------------------------------
//  Li_ChgTerminationCondition — latch & release behaviour
// ------------------------------------------------------------------

void test_termination_does_not_latch_below_line() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    tc.update(3.40f, 14.0f, 0.0f); // vcell 3.40 < v_term=3.65
    TEST_ASSERT_FALSE(bool(tc));
}

void test_termination_latches_above_line() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    tc.update(3.70f, 14.0f, 0.0f); // vcell 3.70 > v_term=3.65
    TEST_ASSERT_FALSE(bool(tc)); // one frame is a spike, not a latch
    updateFrames(tc, 3.70f, 14.0f, 0.0f);
    TEST_ASSERT_TRUE(bool(tc));
}

// Regression: at high current the old uncapped termination line was cv_min + ibat*r, which
// far exceeded cv_eoc and allowed cells to be driven well above cv_eoc before the ceiling
// backstop finally latched. The normal trigger must use the capped v_term so any cell above
// cv_eoc terminates immediately, regardless of charge current.
void test_termination_latches_when_vcell_above_cv_eoc_at_high_current() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    const float highA = 4.0f * p.tail_c_rate * p.Cbat; // 56 A
    // v_term is capped at cv_eoc even at high current; old code would have needed ~4.45 V.
    updateFrames(tc, p.cv_eoc + 0.02f, highA, 0.0f);
    TEST_ASSERT_TRUE(bool(tc));
}

void test_termination_release_via_dod() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    updateFrames(tc, 3.70f, 14.0f, 0.0f);
    TEST_ASSERT_TRUE(bool(tc));

    // Below the DoD threshold: still terminated
    tc.update(3.40f, 0.0f, 0.9f * p.recharge_dod * p.Cbat); // 50.4 Ah
    TEST_ASSERT_TRUE(bool(tc));

    // Past the DoD threshold: released
    tc.update(3.40f, 0.0f, 1.1f * p.recharge_dod * p.Cbat); // 61.6 Ah
    TEST_ASSERT_FALSE(bool(tc));
}

void test_termination_release_via_voltage_floor() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    updateFrames(tc, 3.70f, 14.0f, 0.0f);
    TEST_ASSERT_TRUE(bool(tc));

    // Release floor is cv_min - 0.1V, debounced over several BMS frames so a single
    // I·R sag doesn't release. Just above the floor: stays terminated.
    tc.update(p.cv_min - 0.08f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(bool(tc));

    // A single sub-threshold dip is ignored; going back above the floor resets the streak.
    tc.update(p.cv_min - 0.15f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(bool(tc));
    tc.update(p.cv_min - 0.08f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(bool(tc));

    // Sustained below the floor: released even without any Ah counted.
    for (int i = 0; i < 6; ++i) tc.update(p.cv_min - 0.15f, 0.0f, 0.0f);
    TEST_ASSERT_FALSE(bool(tc));
}

void test_termination_dod_release_skipped_when_cbat_missing() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    updateFrames(tc, 3.70f, 14.0f, 0.0f); // latch with valid params
    TEST_ASSERT_TRUE(bool(tc));

    p.Cbat = NAN; // simulate config drop / unset bat_c

    // Large Ah-since-full but Cbat=NaN → Ah branch is skipped, still terminated
    tc.update(3.40f, 0.0f, 1000.0f);
    TEST_ASSERT_TRUE(bool(tc));

    // Voltage floor still works regardless of Cbat (sustained sub-threshold).
    for (int i = 0; i < 6; ++i) tc.update(p.cv_min - 0.15f, 0.0f, 1000.0f);
    TEST_ASSERT_FALSE(bool(tc));
}

void test_termination_reset_clears_latch() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    updateFrames(tc, 3.70f, 14.0f, 0.0f);
    TEST_ASSERT_TRUE(bool(tc));

    tc.reset();
    TEST_ASSERT_FALSE(bool(tc));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.cv_min, tc.v_term());
}

// ------------------------------------------------------------------
//  CoulombCounter
// ------------------------------------------------------------------

void test_coulomb_counter_starts_at_zero() {
    CoulombCounter cc;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cc.ahSinceFull());
}

void test_coulomb_counter_discharge_accumulates() {
    CoulombCounter cc;
    // 60 s @ 10 A discharge = 10 * 60 / 3600 ≈ 0.1667 Ah
    driveCounter(cc, -10.0f, 0, 60'000'000UL);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 10.0f / 60.0f, cc.ahSinceFull());
}

void test_coulomb_counter_charging_decrements() {
    CoulombCounter cc;
    driveCounter(cc, -10.0f, 0, 60'000'000UL);
    const float afterDischarge = cc.ahSinceFull();
    TEST_ASSERT_TRUE(afterDischarge > 0.1f);

    driveCounter(cc, +10.0f, 70'000'000UL, 130'000'000UL);
    TEST_ASSERT_TRUE(cc.ahSinceFull() < afterDischarge);
}

void test_coulomb_counter_markfull_resets() {
    CoulombCounter cc;
    driveCounter(cc, -10.0f, 0, 60'000'000UL);
    TEST_ASSERT_TRUE(cc.ahSinceFull() > 0.1f);

    cc.markFull();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cc.ahSinceFull());
}

void test_coulomb_counter_drops_gap_over_maxdt() {
    CoulombCounter cc;
    cc.updateBatCurrent(-10.0f, 0); // seed lastTime/lastX
    cc.updateBatCurrent(-10.0f, 31'000'000UL); // gap > 30s maxDt → drop
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cc.ahSinceFull());

    // Next sample within maxDt resumes integration
    cc.updateBatCurrent(-10.0f, 41'000'000UL); // 10 s after the prior sample
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f * 10.0f / 3600.0f, cc.ahSinceFull());
}

// ------------------------------------------------------------------
//  MQTT-callback integration
// ------------------------------------------------------------------

void test_mqtt_vcell_message_updates_state() {
    BatteryCharger charger;
    charger.params = makeLfpParams();

    ConfFile mqttConf{{
        {"cell_voltages_max_topic", "test/vcell"},
    }};
    charger.beginMqtt(mqttConf);

    const char *msg = "3.512";
    MQTT._invokeForTest("test/vcell", msg, std::strlen(msg));

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.512f, (float) charger.batSt.vcell_high);
    TEST_ASSERT_TRUE(charger.batSt.haveValidCellVoltage());
}

// Test/main.cpp owns this variable in the test build. The MQTT-driven ibat path
// uses wallClockUs() (which reads loopWallClockUs_) for integration timestamps,
// so we must advance it ourselves — nothing else does during tests.
extern time_us loopWallClockUs_;

void test_mqtt_ibat_message_drives_coulomb_counter() {
    BatteryCharger charger;
    charger.params = makeLfpParams();

    ConfFile mqttConf{{
        {"ibat_topic", "test/ibat"},
    }};
    charger.beginMqtt(mqttConf);

    // Two discharge samples spaced by a real ~20ms wall-clock gap. We advance
    // loopWallClockUs_ ourselves before each delivery so the integrator sees
    // a non-zero dt.
    loopWallClockUs_ = micros();
    MQTT._invokeForTest("test/ibat", "-5", 2);
    delay(20);
    loopWallClockUs_ = micros();
    MQTT._invokeForTest("test/ibat", "-5", 2);

    TEST_ASSERT_TRUE(charger.batSt.coulombCounter.ahSinceFull() > 0.0f);
}

void test_mqtt_ibat_lim_accepts_valid() {
    BatteryCharger charger;
    charger.params = makeLfpParams();

    ConfFile mqttConf{{
        {"ibat_lim_topic", "test/ibatlim"},
    }};
    charger.beginMqtt(mqttConf);

    MQTT._invokeForTest("test/ibatlim", "30", 2);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, charger.params.Ibat_lim);

    // Zero is valid — BMS can legitimately signal "no charging permitted"
    MQTT._invokeForTest("test/ibatlim", "0", 1);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, charger.params.Ibat_lim);
}

void test_mqtt_ibat_lim_rejects_negative() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    const float initial = charger.params.Ibat_lim;

    ConfFile mqttConf{{
        {"ibat_lim_topic", "test/ibatlim"},
    }};
    charger.beginMqtt(mqttConf);

    MQTT._invokeForTest("test/ibatlim", "-5", 2);
    TEST_ASSERT_EQUAL_FLOAT(initial, charger.params.Ibat_lim);
}

void test_mqtt_ibat_lim_rejects_nan() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    const float initial = charger.params.Ibat_lim;

    ConfFile mqttConf{{
        {"ibat_lim_topic", "test/ibatlim"},
    }};
    charger.beginMqtt(mqttConf);

    MQTT._invokeForTest("test/ibatlim", "nan", 3);
    TEST_ASSERT_EQUAL_FLOAT(initial, charger.params.Ibat_lim);
}

// A zero-length payload (e.g. a retained-message clear) used to read dat[-1] inside strntof (OOB).
// All three BMS callbacks must now survive it and leave state untouched.
void test_mqtt_empty_payload_is_safe() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    const float limInit = charger.params.Ibat_lim;

    ConfFile mqttConf{{
        {"cell_voltages_max_topic", "test/vcell"},
        {"ibat_topic", "test/ibat"},
        {"ibat_lim_topic", "test/ibatlim"},
    }};
    charger.beginMqtt(mqttConf);

    MQTT._invokeForTest("test/vcell", "", 0);
    MQTT._invokeForTest("test/ibat", "", 0);
    MQTT._invokeForTest("test/ibatlim", "", 0);

    TEST_ASSERT_FALSE(charger.batSt.haveValidCellVoltage());        // NAN vcell not treated as valid
    TEST_ASSERT_EQUAL_FLOAT(limInit, charger.params.Ibat_lim);      // empty ignored, limit unchanged
    TEST_ASSERT_EQUAL_FLOAT(0.0f, charger.batSt.coulombCounter.ahSinceFull()); // ibat untouched
}

// releaseVoutPinning must release the pack-voltage pin UP to Vbat_max (so a converter that lost
// authority on a shared bus can climb back and re-take it), NOT down to Vbat_fallback — which would
// pin it at the resting bus voltage and throttle a battery that isn't full.
void test_release_vout_pinning_goes_to_vbat_max() {
    BatteryCharger charger;
    charger.params = makeLfpParams();   // Vbat_max=14.6, Vbat_fallback=13.4
    loopWallClockUs_ = 1'000'000;

    // No BMS cell data -> the charger pins Vout down to Vbat_fallback.
    charger.update(13.4f, 0.0f, /*voutAuthority*/ true);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, charger.params.Vbat_fallback, charger.Vout_max());

    // Release must move the target UP to Vbat_max, not leave it at the fallback.
    bool changed = charger.releaseVoutPinning("test");
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, charger.params.Vbat_max, charger.Vout_max());
}

// Drive the EOC feedback loop with the highest cell held above v_eoc, as if this converter's
// Vout reads high so lowering vpack_pin never actually brings the cell down. The BMS-driven loop
// must keep pulling vpack_pin below the nominal float floor (Vbat_fallback) by up to
// params.vout_offset_max, so a full pack stops being trickle-charged despite the Vout offset.
static void driveEocFeedback(BatteryCharger &charger, float vcellHigh, int frames) {
    // termCond.v_term() is cv_min only after _updateTermination runs; reset() seeds it now so v_eoc
    // isn't NaN-> cv_eoc before the ibat smoothing warms up.
    charger.termCond.reset();
    for (int i = 0; i < frames; ++i) {
        loopWallClockUs_ += 1'000'000;        // a fresh BMS frame each iteration (advances vcell_high_t)
        charger.batSt.setVcellHigh(vcellHigh);
        charger.batSt.updateBatCurrent(0.1f); // small +ibat: warms smoothing, latches termination (v_term~cv_min)
        charger.update(charger.params.Vbat_fallback, 0.1f, /*voutAuthority*/ true);
    }
}

void test_eoc_floor_allows_vout_offset_correction() {
    BatteryCharger charger;
    charger.params = makeLfpParams();          // Vbat_fallback=13.4, cv_min=3.37
    charger.params.vout_offset_max = 0.6f;
    loopWallClockUs_ = 1'000'000;

    driveEocFeedback(charger, charger.params.cv_min + 0.10f, 200); // cell stuck above v_eoc

    const float floor = charger.params.Vbat_fallback - charger.params.vout_offset_max; // 12.8
    TEST_ASSERT_TRUE(charger.Vout_max() < charger.params.Vbat_fallback - 0.05f); // used the headroom
    TEST_ASSERT_FLOAT_WITHIN(0.05f, floor, charger.Vout_max());                  // settled at the floor
    TEST_ASSERT_TRUE(charger.Vout_max() >= floor - 0.01f);                       // never below it
}

void test_eoc_floor_zero_offset_stops_at_fallback() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    charger.params.vout_offset_max = 0.0f;     // legacy behaviour: floor == Vbat_fallback
    loopWallClockUs_ = 1'000'000;

    driveEocFeedback(charger, charger.params.cv_min + 0.10f, 200);

    TEST_ASSERT_FLOAT_WITHIN(0.05f, charger.params.Vbat_fallback, charger.Vout_max());
}

// Terminated pack + no Vout authority (sibling holds the shared bus): this converter must yield
// LOW (pin at the EOC float floor) rather than climbing to Vbat_max and pushing current into a full
// pack — the limit-cycle that kept a terminated pack trickle-charging.
void test_terminated_no_authority_yields_low() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    charger.params.vout_offset_max = 0.6f;
    loopWallClockUs_ = 1'000'000;
    driveEocFeedback(charger, charger.params.cv_min + 0.10f, 40); // latch termination (with authority)

    loopWallClockUs_ += 1'000'000;                                // now lose authority while terminated
    charger.batSt.setVcellHigh(charger.params.cv_min + 0.10f);
    charger.batSt.updateBatCurrent(0.1f);
    charger.update(charger.params.Vbat_fallback, 0.1f, /*voutAuthority*/ false);

    const float floor = charger.params.Vbat_fallback - charger.params.vout_offset_max;
    TEST_ASSERT_FLOAT_WITHIN(0.05f, floor, charger.Vout_max());   // yielded low, NOT Vbat_max
}

// Not full + no authority: release UP to Vbat_max so the converter can climb back and re-take the
// bus (unchanged behaviour — only the terminated case yields low).
void test_not_terminated_no_authority_releases_high() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    loopWallClockUs_ = 1'000'000;
    charger.batSt.setVcellHigh(charger.params.cv_min - 0.20f);    // low cell -> not terminated
    charger.batSt.updateBatCurrent(5.0f);
    charger.update(charger.params.Vbat_fallback, 5.0f, /*voutAuthority*/ false);

    TEST_ASSERT_FLOAT_WITHIN(0.05f, charger.params.Vbat_max, charger.Vout_max());
}

// The float floor tracks vout_offset_max: a 0.3 V budget floors 0.3 V below Vbat_fallback.
void test_eoc_floor_scales_with_offset() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    charger.params.vout_offset_max = 0.3f;
    loopWallClockUs_ = 1'000'000;

    driveEocFeedback(charger, charger.params.cv_min + 0.10f, 200);

    TEST_ASSERT_FLOAT_WITHIN(0.05f, charger.params.Vbat_fallback - 0.3f, charger.Vout_max());
}

// releaseVoutPinning must clear the terminated-state memory so that, after the converter
// climbs back to Vbat_max, the next update starts a fresh float glide rather than jumping
// immediately to the stale Vbat_fallback target (which would step the setpoint).
void test_release_vout_pinning_restarts_float_glide() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    loopWallClockUs_ = 1'000'000;

    // Terminate and settle to the float setpoint. driveEocFeedback holds the cell above v_eoc, so the
    // EOC loop drives to the floor (Vbat_fallback - vout_offset_max), not Vbat_fallback itself.
    driveEocFeedback(charger, charger.params.cv_min + 0.10f, 200);
    TEST_ASSERT_TRUE(bool(charger.termCond));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, charger.params.Vbat_fallback - charger.params.vout_offset_max, charger.Vout_max());

    // Release the latch; vpack_pin should move UP to Vbat_max.
    TEST_ASSERT_TRUE(charger.releaseVoutPinning("test"));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, charger.params.Vbat_max, charger.Vout_max());

    // Drop the cell below v_eoc so EOC feedback does not override the glide.
    charger.batSt.setVcellHigh(charger.params.cv_min - 0.05f);
    loopWallClockUs_ += 1'000; // 1 ms: glide is still at its Vbat_max start
    charger.update(charger.params.Vbat_max, 0.1f, /*voutAuthority*/ true);

    // Without the _wasTerminated reset, Vout_max would immediately snap back to Vbat_fallback.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, charger.params.Vbat_max, charger.Vout_max());
}


// A single over-line frame followed by a normal one must not latch (streak resets).
void test_termination_line_streak_resets() {
    auto p = makeLfpParams();
    Li_ChgTerminationCondition tc{p};
    tc.update(3.70f, 14.0f, 0.0f);
    tc.update(3.40f, 14.0f, 0.0f);
    tc.update(3.70f, 14.0f, 0.0f);
    TEST_ASSERT_FALSE(bool(tc));
}

// Drive absorption: cell held at vcell with a constant ibat and fresh cell + ibat frames.
static void driveFrames(BatteryCharger &charger, float vcellHigh, float ibat, int frames, bool authority = true,
                        float vout = NAN) {
    for (int i = 0; i < frames; ++i) {
        loopWallClockUs_ += 4'000'000;
        charger.batSt.setVcellHigh(vcellHigh);
        charger.batSt.updateBatCurrent(ibat);
        charger.update(std::isfinite(vout) ? vout : charger.params.Vbat_max, ibat, authority);
    }
}

// Regression (fry/flat 2026-07): the EOC feedback target used to be v_term(ibat). A cell above
// cv_min but well below cv_eoc at moderate current then pulled the pin down, the current fell,
// v_term fell with it, and the pack "terminated" at 3.40 V/cell. The absorption target is now the
// fixed cv_eoc: such a cell must neither be pulled down nor terminate.
void test_eoc_feedback_does_not_chase_termination_line() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    charger.termCond.reset();
    loopWallClockUs_ = 1'000'000;
    driveFrames(charger, 3.45f, 10.0f, 40); // v_term(10A) = 3.57 > 3.45: below the line, below cv_eoc
    TEST_ASSERT_FALSE(bool(charger.termCond));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, charger.params.Vbat_max, charger.Vout_max()); // bulk, not pulled down
    // At cv_eoc the feedback engages: the pin comes down.
    driveFrames(charger, charger.params.cv_eoc + 0.01f, 10.0f, 10);
    TEST_ASSERT_TRUE(charger.Vout_max() < charger.params.Vbat_max - 0.05f);
}

// Once terminated the feedback target is cv_min: a terminated pack still above cv_min gets the
// pin pulled down (no trickle), even at a small positive ibat that used to raise v_term.
void test_terminated_target_is_cv_min() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    loopWallClockUs_ = 1'000'000;
    driveEocFeedback(charger, charger.params.cv_min + 0.10f, 10); // latches
    TEST_ASSERT_TRUE(bool(charger.termCond));
    float before = charger.Vout_max();
    driveFrames(charger, charger.params.cv_min + 0.02f, 2.0f, 20); // v_term(2A) would be 3.39 > cell
    TEST_ASSERT_TRUE(charger.Vout_max() < before - 0.01f);
}

// Battery temperature: below bat_temp_min charging is blocked (released with hysteresis). Without
// BMS current to follow, the output limit drops to the small floor; with it, the load-follower
// holds the pack current at 0 and the output limit stays free (loads are served).
void test_bat_temp_cold_block_and_hysteresis() {
    BatteryCharger charger;
    charger.params = makeLfpParams(); // Ibat_lim 40, temps 0/45/55
    loopWallClockUs_ = 1'000'000;
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, charger.Iout_max()); // no sensor: no limit
    TEST_ASSERT_FALSE(charger.chargeBlocked());
    charger.batSt.setTemp(0, -1.0f);
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_TRUE(charger.chargeBlocked());
    TEST_ASSERT_TRUE(charger.chargeHold());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.25f, charger.Iout_max()); // no ibat: idle at the floor, never 0
    charger.batSt.setTemp(0, 1.0f); // inside the 2 °C band: still blocked
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_TRUE(charger.chargeBlocked());
    charger.batSt.setTemp(0, 2.5f);
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_FALSE(charger.chargeBlocked());
    TEST_ASSERT_EQUAL_FLOAT(40.0f, charger.Iout_max());
    // Out-of-range sensor values are ignored.
    charger.batSt.setTemp(0, -60.0f);
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_FALSE(charger.chargeBlocked());
}

void test_bat_temp_cold_hold_follows_load() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    loopWallClockUs_ = 1'000'000;
    charger.batSt.setTemp(0, -5.0f);
    driveFrames(charger, 3.30f, 6.0f, 12); // charging 6 A into a cold pack
    TEST_ASSERT_TRUE(charger.chargeBlocked());
    TEST_ASSERT_EQUAL_FLOAT(40.0f, charger.Iout_max()); // loads may still be served
    float pin0 = charger.Vout_max();
    driveFrames(charger, 3.30f, 6.0f, 10);
    TEST_ASSERT_TRUE(charger.Vout_max() < pin0 - 0.05f); // pin walks down to stop the charge current
    // Expiry: an hour without a temperature frame drops the policy.
    loopWallClockUs_ += 3601ULL * 1'000'000ULL;
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_FALSE(charger.chargeBlocked());
}

// A cold pack can sit at any SoC: the hold must be able to pull the pin below the partial-hold floor
// (12.68 V for this 4S config) down to the actual bus voltage, or it would charge a low pack.
void test_bat_temp_cold_low_soc_pack() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    loopWallClockUs_ = 1'000'000;
    charger.batSt.setTemp(0, -5.0f);
    driveFrames(charger, 3.10f, 6.0f, 12, true, 12.4f); // bus 12.4 V, 6 A still flowing in
    TEST_ASSERT_TRUE(charger.chargeBlocked());
    TEST_ASSERT_TRUE(charger.Vout_max() <= 12.4f + 0.01f); // seeded at the bus, never clamped up
    driveFrames(charger, 3.10f, 6.0f, 10, true, 12.4f);
    TEST_ASSERT_TRUE(charger.Vout_max() < 12.4f - 0.05f);
}

// Sensors expire individually: a dead cold sensor must not block forever while another one stays live.
void test_bat_temp_per_sensor_expiry() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    loopWallClockUs_ = 1'000'000;
    charger.batSt.setTemp(0, -5.0f);
    charger.batSt.setTemp(1, 20.0f);
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_TRUE(charger.chargeBlocked());
    for (int i = 0; i < 80; ++i) { // sensor 1 keeps reporting for ~1.3 h, sensor 0 is silent
        loopWallClockUs_ += 60ULL * 1'000'000ULL;
        charger.batSt.setTemp(1, 20.0f);
        charger.update(13.3f, 0.0f, true);
    }
    TEST_ASSERT_FALSE(charger.chargeBlocked());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, charger.tempMin());
}

// A stopped ibat stream must clear the partial hold (the deficit counter is dead) so the converter
// falls back to normal charging instead of freezing the pin and suppressing every recovery path.
void test_partial_hold_clears_on_stale_ibat() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    charger.params.partial_charge = 0.8f;
    loopWallClockUs_ = 1'000'000;
    driveEocFeedback(charger, charger.params.cv_min + 0.10f, 12);
    driveCounter(charger.batSt.coulombCounter, 60.0f, loopWallClockUs_, loopWallClockUs_ + 3600ULL * 1'000'000ULL);
    loopWallClockUs_ += 3600ULL * 1'000'000ULL;
    driveFrames(charger, 3.30f, -1.0f, 4);
    driveCounter(charger.batSt.coulombCounter, -60.0f, loopWallClockUs_, loopWallClockUs_ + 300ULL * 1'000'000ULL);
    loopWallClockUs_ += 300ULL * 1'000'000ULL;
    driveFrames(charger, 3.33f, 8.0f, 4);
    TEST_ASSERT_TRUE(charger.partialHold());
    for (int i = 0; i < 50; ++i) { // cell frames continue, ibat frames stop
        loopWallClockUs_ += 4'000'000;
        charger.batSt.setVcellHigh(3.33f);
        charger.update(charger.params.Vbat_max, 0.0f, true);
    }
    TEST_ASSERT_FALSE(charger.partialHold());
    TEST_ASSERT_FALSE(charger.chargeHold());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, charger.params.Vbat_max, charger.Vout_max());
}

// The boot sweep trusts terminationDecided(); the line needs two frames, so one frame is not a decision.
void test_termination_decided_after_two_frames() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    charger.termCond.reset();
    loopWallClockUs_ = 1'000'000;
    for (int i = 0; i < 8; ++i) charger.batSt.updateBatCurrent(0.1f); // warm the smoothing
    driveFrames(charger, charger.params.cv_min + 0.10f, 0.1f, 1);
    TEST_ASSERT_FALSE(charger.terminationDecided());
    TEST_ASSERT_FALSE(bool(charger.termCond));
    driveFrames(charger, charger.params.cv_min + 0.10f, 0.1f, 1);
    TEST_ASSERT_TRUE(charger.terminationDecided());
    TEST_ASSERT_TRUE(bool(charger.termCond));
}

// The output limit is floored on the final value: a BMS may publish ibat_lim = 0.
void test_iout_max_never_zero() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    charger.params.Ibat_lim = 0.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.25f, charger.Iout_max());
}

void test_partial_charge_config_validation() {
    ConfFile bad{{{"vout_max", "14.6"}, {"cv_eoc", "3.65"}, {"cv_float", "3.37"}, {"bat_c", "280"},
                  {"partial_charge", "0.5"}, {"recharge_dod", "0.6"}}};
    BatChargerParams p;
    bool threw = false;
    try { p.load(bad); } catch (...) { threw = true; }
    TEST_ASSERT_TRUE(threw);
    ConfFile noCap{{{"vout_max", "14.6"}, {"cv_eoc", "3.65"}, {"cv_float", "3.37"}, {"partial_charge", "0.5"}}};
    p.load(noCap); // no bat_c: warns and disables the ceiling instead of refusing to start
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.partial_charge);
}

void test_bat_temp_hot_derate() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    loopWallClockUs_ = 1'000'000;
    charger.batSt.setTemp(0, 20.0f);
    charger.batSt.setTemp(1, 50.0f); // the hottest sensor derates
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, charger.Iout_max()); // no ibat: output cap, halfway 45..55 -> 50 %
    charger.batSt.setTemp(1, 60.0f);
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.25f, charger.Iout_max()); // never 0
    TEST_ASSERT_FALSE(charger.chargeBlocked()); // the coldest sensor (20 °C) is fine
    // With a live ibat the derate regulates the *pack* current through the pin (shared-bus safe):
    // 30 A into the pack at 50 °C against a 20 A target -> the pin walks down, the output cap is free.
    charger.batSt.setTemp(1, 50.0f);
    driveFrames(charger, 3.30f, 30.0f, 12);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, charger.Iout_max());
    float pin0 = charger.Vout_max();
    driveFrames(charger, 3.30f, 30.0f, 10);
    TEST_ASSERT_TRUE(charger.Vout_max() < pin0 - 0.05f);
    TEST_ASSERT_FALSE(charger.chargeHold()); // hot is a derate, not a hold: sweeps stay allowed
}

// Partial-charge ceiling: needs a full event first; then holds at partial_charge (Ah-counted),
// releases recharge_dod below the ceiling, and is dropped when the full-charge interval expires.
void test_partial_hold_cycle() {
    BatteryCharger charger;
    charger.params = makeLfpParams(); // Cbat 280, recharge_dod 0.2
    charger.params.partial_charge = 0.8f; // ceiling deficit 56 Ah, release at 112 Ah
    charger.params.full_charge_interval_s = 86400;
    loopWallClockUs_ = 1'000'000;
    // Move Ah with a moderate current over time so the producer-side ibat EWMA isn't left with a
    // huge residual that the frames below would have to wash out.
    auto moveAh = [&](float ah) {
        unsigned long secs = (unsigned long) (fabsf(ah) / 60.0f * 3600.0f);
        driveCounter(charger.batSt.coulombCounter, ah > 0 ? -60.0f : 60.0f, loopWallClockUs_, loopWallClockUs_ + secs * 1'000'000UL);
        loopWallClockUs_ += secs * 1'000'000UL;
    };

    // No full event yet: charging normally even with ahSinceFull = 0.
    driveFrames(charger, 3.30f, 5.0f, 12);
    TEST_ASSERT_FALSE(charger.partialHold());
    TEST_ASSERT_FALSE(charger.chargeHold());

    // Full charge terminates -> full event.
    driveEocFeedback(charger, charger.params.cv_min + 0.10f, 12);
    TEST_ASSERT_TRUE(bool(charger.termCond));
    TEST_ASSERT_FALSE(charger.partialHold()); // termination has precedence
    TEST_ASSERT_TRUE(charger.lastFullUs() != 0);

    // Discharge 60 Ah: termination releases at 56 Ah, hold is not yet on (deficit > ceiling).
    moveAh(60.0f);
    driveFrames(charger, 3.30f, -1.0f, 4);
    TEST_ASSERT_FALSE(bool(charger.termCond));
    TEST_ASSERT_FALSE(charger.partialHold());

    // Charge 5 Ah back: deficit 55 Ah <= 56 -> hold engages and follows the load.
    moveAh(-5.0f);
    driveFrames(charger, 3.33f, 8.0f, 4);
    TEST_ASSERT_TRUE(charger.partialHold());
    TEST_ASSERT_TRUE(charger.chargeHold());
    float pin0 = charger.Vout_max();
    driveFrames(charger, 3.33f, 8.0f, 10); // pack still charging: pin steps down
    TEST_ASSERT_TRUE(charger.Vout_max() < pin0 - 0.05f);
    float pin1 = charger.Vout_max();
    driveFrames(charger, 3.33f, -8.0f, 12); // load exceeds PV: pin steps up
    TEST_ASSERT_TRUE(charger.Vout_max() > pin1 + 0.05f);
    // 1 Ah above the ceiling the Ah trim asks for -0.5 A; ±0.2 A around that is the deadband.
    driveFrames(charger, 3.33f, -0.5f, 20);
    float pin2 = charger.Vout_max();
    driveFrames(charger, 3.33f, -0.4f, 5);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, pin2, charger.Vout_max());
    // Without output authority (converter off at night) the pin must not integrate the load.
    driveFrames(charger, 3.33f, -8.0f, 10, /*authority*/ false);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, pin2, charger.Vout_max());

    // Discharge past the release band (deficit > 112 Ah): hold releases, pin glides to Vbat_max.
    moveAh(70.0f);
    driveFrames(charger, 3.30f, -1.0f, 4);
    TEST_ASSERT_FALSE(charger.partialHold());
    loopWallClockUs_ += 10'000'000UL; // past the 5 s glide
    driveFrames(charger, 3.30f, 5.0f, 2);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, charger.params.Vbat_max, charger.Vout_max());

    // Charge back to the ceiling: hold again. Then let the full-charge interval expire: dropped.
    moveAh(-70.0f);
    driveFrames(charger, 3.33f, 5.0f, 4);
    TEST_ASSERT_TRUE(charger.partialHold());
    loopWallClockUs_ += 86400ULL * 1'000'000ULL;
    driveFrames(charger, 3.33f, 5.0f, 2);
    TEST_ASSERT_FALSE(charger.partialHold());
}

void test_partial_hold_disabled_by_default() {
    BatteryCharger charger;
    charger.params = makeLfpParams(); // partial_charge 0
    loopWallClockUs_ = 1'000'000;
    driveEocFeedback(charger, charger.params.cv_min + 0.10f, 12);
    driveCounter(charger.batSt.coulombCounter, -3600.0f, loopWallClockUs_, loopWallClockUs_ + 60'000'000UL);
    loopWallClockUs_ += 60'000'000UL;
    driveFrames(charger, 3.30f, -1.0f, 4);
    driveCounter(charger.batSt.coulombCounter, 3600.0f, loopWallClockUs_, loopWallClockUs_ + 5'000'000UL);
    loopWallClockUs_ += 5'000'000UL;
    driveFrames(charger, 3.33f, 8.0f, 4);
    TEST_ASSERT_FALSE(charger.partialHold());
}

void test_mqtt_bat_temp_topics() {
    BatteryCharger charger;
    charger.params = makeLfpParams();
    ConfFile mqttConf{{
        {"bat_temp_topic", "test/t1, test/t2"},
    }};
    charger.beginMqtt(mqttConf);
    MQTT._invokeForTest("test/t1", "12.5", 4);
    MQTT._invokeForTest("test/t2", "-3", 2);
    MQTT._invokeForTest("test/t2", "", 0); // empty payload: ignored
    loopWallClockUs_ = 1'000'000;
    charger.update(13.3f, 0.0f, true);
    TEST_ASSERT_TRUE(charger.haveTemp());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -3.0f, charger.tempMin());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.5f, charger.tempMax());
}
