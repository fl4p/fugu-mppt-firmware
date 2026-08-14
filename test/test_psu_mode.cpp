#include <unity.h>
#include <cmath>
#include "mppt.h"
#include "app_state.h"

extern MpptController mppt;
extern time_us loopWallClockUs_;

static void setPsuMode() {
    g_app.opMode = OpMode::Psu;
    mppt.psuResetTripState();
}

static void setMpptMode() {
    g_app.opMode = OpMode::Mppt;
    mppt.psuVsetpoint = NAN;
    mppt.psuResetTripState();
}

static void setupLimits(float vout_max, bool rcp) {
    const_cast<float&>(mppt.limits.Vout_max) = vout_max;
    const_cast<bool&>(mppt.limits.reverse_current_paranoia) = rcp;
}

// --- 1. OV threshold derivation ----------------------------------------------

void test_psu_ov_threshold_from_setpoint() {
    setPsuMode();
    setupLimits(100.0f, true);
    mppt.setExplicitOvLimit(NAN);
    mppt.charger.params.Vbat_max = NAN;
    mppt.psuVsetpoint = 50.0f;
    auto ovTh = mppt.computeOvThreshold();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f * 1.03f, ovTh);
}

void test_psu_ov_threshold_clamped_to_vout_max() {
    setPsuMode();
    setupLimits(60.0f, false);
    mppt.setExplicitOvLimit(NAN);
    mppt.charger.params.Vbat_max = NAN;
    mppt.psuVsetpoint = 80.0f;
    auto ovTh = mppt.computeOvThreshold();
    TEST_ASSERT_EQUAL_FLOAT(60.0f, ovTh);
}

void test_psu_ov_threshold_explicit_ovset_overrides() {
    setPsuMode();
    setupLimits(100.0f, true);
    mppt.setExplicitOvLimit(55.0f);
    mppt.psuVsetpoint = 50.0f;
    auto ovTh = mppt.computeOvThreshold();
    TEST_ASSERT_EQUAL_FLOAT(55.0f, ovTh);
}

void test_psu_ov_threshold_no_setpoint_falls_back() {
    setPsuMode();
    setupLimits(100.0f, false);
    mppt.setExplicitOvLimit(NAN);
    mppt.charger.params.Vbat_max = NAN;
    mppt.psuVsetpoint = NAN;
    auto ovTh = mppt.computeOvThreshold();
    TEST_ASSERT_EQUAL_FLOAT(100.0f, ovTh);
}

// --- 2. Trip escalation ------------------------------------------------------

static void simulateTrips(int n, time_us baseUs) {
    for (int i = 0; i < n; ++i) {
        loopWallClockUs_ = baseUs + (time_us)i * 200000ULL;
        mppt.shutdownDcdc("Vout-OV-test", 5);
    }
}

void test_psu_trip_escalates_after_repeated_trips() {
    setPsuMode();
    loopWallClockUs_ = 1000000ULL;
    simulateTrips(6, 1000000ULL);
    TEST_ASSERT_TRUE(mppt.psuEscalated);
    TEST_ASSERT_FALSE(mppt.psuLatched);
}

void test_psu_trip_latches_after_many_trips() {
    setPsuMode();
    loopWallClockUs_ = 1000000ULL;
    simulateTrips(10, 1000000ULL);
    TEST_ASSERT_TRUE(mppt.psuLatched);
}

void test_psu_trip_sparse_does_not_escalate() {
    setPsuMode();
    loopWallClockUs_ = 1000000ULL;
    mppt.shutdownDcdc("Vout-OV-test", 5);
    loopWallClockUs_ = 70000000ULL;
    mppt.shutdownDcdc("Vout-OV-test", 5);
    loopWallClockUs_ = 140000000ULL;
    mppt.shutdownDcdc("Vout-OV-test", 5);
    TEST_ASSERT_FALSE(mppt.psuEscalated);
    TEST_ASSERT_FALSE(mppt.psuLatched);
}

void test_psu_trip_permanent_fault_reaches_latch() {
    setPsuMode();
    loopWallClockUs_ = 1000000ULL;
    simulateTrips(20, 1000000ULL);
    TEST_ASSERT_TRUE(mppt.psuLatched);
}

void test_psu_serious_fault_stays_out_of_fast_retry_bucket() {
    setPsuMode();
    loopWallClockUs_ = 1000000ULL;
    for (int i = 0; i < 10; ++i) {
        loopWallClockUs_ += 200000ULL;
        mppt.shutdownDcdc("sensor-fail", 5);
    }
    TEST_ASSERT_EQUAL_UINT8(0, mppt.getPsuTripCount());
    TEST_ASSERT_FALSE(mppt.psuEscalated);
    TEST_ASSERT_FALSE(mppt.psuLatched);
}

void test_psu_trip_latch_blocks_start() {
    setPsuMode();
    mppt.psuLatched = true;
    TEST_ASSERT_EQUAL_STRING("psu-latch", mppt.startBlockReason());
}

void test_psu_reset_clears_latch() {
    setPsuMode();
    mppt.psuLatched = true;
    mppt.psuEscalated = true;
    mppt.psuResetTripState();
    TEST_ASSERT_FALSE(mppt.psuLatched);
    TEST_ASSERT_FALSE(mppt.psuEscalated);
    TEST_ASSERT_EQUAL_UINT8(0, mppt.getPsuTripCount());
}

// --- 3. setPsuSetpoint range rejection ---------------------------------------

void test_psu_setpoint_rejects_negative() {
    setMpptMode();
    setupLimits(100.0f, false);
    mppt.psuVsetpoint = NAN;
    mppt.setPsuSetpoint(-5.0f);
    TEST_ASSERT_TRUE(std::isnan(mppt.psuVsetpoint));
}

void test_psu_setpoint_rejects_zero() {
    setMpptMode();
    setupLimits(100.0f, false);
    mppt.psuVsetpoint = NAN;
    mppt.setPsuSetpoint(0.0f);
    TEST_ASSERT_TRUE(std::isnan(mppt.psuVsetpoint));
}

void test_psu_setpoint_rejects_nan() {
    setMpptMode();
    setupLimits(100.0f, false);
    mppt.psuVsetpoint = 42.0f;
    mppt.setPsuSetpoint(NAN);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, mppt.psuVsetpoint);
}

void test_psu_setpoint_rejects_above_vout_max() {
    setMpptMode();
    setupLimits(80.0f, false);
    mppt.psuVsetpoint = NAN;
    mppt.setPsuSetpoint(81.0f);
    TEST_ASSERT_TRUE(std::isnan(mppt.psuVsetpoint));
}

void test_psu_setpoint_accepts_valid() {
    setMpptMode();
    setupLimits(100.0f, false);
    mppt.psuVsetpoint = NAN;
    mppt.setPsuSetpoint(72.5f);
    TEST_ASSERT_EQUAL_FLOAT(72.5f, mppt.psuVsetpoint);
}

void test_psu_boost_rejects_setpoint_below_input() {
    auto e = MpptController::validatePsuSetpoint(48.0f, 80.0f, true, 48.0f, NAN);
    TEST_ASSERT_EQUAL_INT((int) PsuSetpointError::BoostBelowInput, (int) e);
}

void test_psu_boost_accepts_setpoint_with_headroom() {
    auto e = MpptController::validatePsuSetpoint(50.0f, 80.0f, true, 48.0f, NAN);
    TEST_ASSERT_EQUAL_INT((int) PsuSetpointError::None, (int) e);
}

void test_psu_boost_rejects_unknown_input() {
    auto e = MpptController::validatePsuSetpoint(50.0f, 80.0f, true, NAN, NAN);
    TEST_ASSERT_EQUAL_INT((int) PsuSetpointError::TelemetryUnavailable, (int) e);
}

void test_psu_setpoint_rejects_explicit_ov_conflict() {
    auto e = MpptController::validatePsuSetpoint(50.0f, 80.0f, false, 24.0f, 50.0f);
    TEST_ASSERT_EQUAL_INT((int) PsuSetpointError::OvLimitConflict, (int) e);
}

void test_psu_enable_waits_for_fresh_telemetry() {
    setMpptMode();
    setupLimits(80.0f, false);
    mppt.setExplicitOvLimit(NAN);
    const auto ticket = mppt.queuePsuSetpoint(50.0f);
    TEST_ASSERT_NOT_EQUAL_UINT32(0, ticket);
    mppt.applyPendingPsuCommandRt(false);
    TEST_ASSERT_TRUE(mppt.hasPendingPsuCommand());
    TEST_ASSERT_FALSE(mppt.isPsuCommandDone(ticket));
    TEST_ASSERT_FALSE(g_app.psuMode());
}

void test_psu_later_override_wins_before_rt_apply() {
    setMpptMode();
    setupLimits(80.0f, false);
    mppt.setExplicitOvLimit(NAN);
    const auto enableTicket = mppt.queuePsuSetpoint(50.0f);
    const auto manualTicket = mppt.requestPsuManual(0, -1);
    mppt.applyPendingPsuCommandRt(false);
    TEST_ASSERT_FALSE(mppt.isPsuCommandDone(enableTicket));
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(manualTicket));
    TEST_ASSERT_TRUE(g_app.manualPwm());
}

void test_psu_completion_keeps_earlier_concurrent_ticket() {
    setMpptMode();
    setupLimits(80.0f, false);
    mppt.setExplicitOvLimit(NAN);
    const auto first = mppt.queuePsuSetpoint(50.0f);
    mppt.applyPendingPsuCommandRt(true);
    const auto second = mppt.requestPsuManual(0, -1);
    mppt.applyPendingPsuCommandRt(false);
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(first));
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(second));
    TEST_ASSERT_EQUAL_INT((int) PsuSetpointError::None,
                          (int) mppt.getPsuCommandError(first));
}

void test_psu_short_low_side_transition_is_rt_owned() {
    setPsuMode();
    const auto ticket = mppt.requestPsuShortLowSide();
    TEST_ASSERT_TRUE(mppt.hasPendingPsuCommand());
    TEST_ASSERT_TRUE(g_app.psuMode());
    mppt.applyPendingPsuCommandRt(false);
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(ticket));
    TEST_ASSERT_TRUE(g_app.manualPwm());
    TEST_ASSERT_EQUAL_UINT16(0, mppt.getManualTarget());
}

// --- 4. OpMode flags ---------------------------------------------------------

void test_psu_mode_flag_correct() {
    g_app.opMode = OpMode::Psu;
    TEST_ASSERT_TRUE(g_app.psuMode());
    TEST_ASSERT_FALSE(g_app.manualPwm());

    g_app.opMode = OpMode::Manual;
    TEST_ASSERT_TRUE(g_app.manualPwm());
    TEST_ASSERT_FALSE(g_app.psuMode());

    g_app.opMode = OpMode::Mppt;
    TEST_ASSERT_FALSE(g_app.psuMode());
    TEST_ASSERT_FALSE(g_app.manualPwm());
}
