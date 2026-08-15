#include <unity.h>
#include <cmath>
#include "mppt.h"
#include "app_state.h"

extern MpptController mppt;
extern time_us loopWallClockUs_;

static void pvSetup(float vout_max = 80.0f) {
    g_app.opMode = OpMode::Mppt;
    mppt.psuVsetpoint = NAN;
    mppt.pvSim.active = false; // a leftover active would turn pvEnable into an in-place update
    mppt.pvSim.iout.reset();
    mppt.psuResetTripState();
    mppt.setExplicitOvLimit(NAN);
    const_cast<float &>(mppt.limits.Vout_max) = vout_max;
    const_cast<bool &>(mppt.limits.reverse_current_paranoia) = false;
}

// Fresh curve entry: queue + RT apply, asserts the transition happened.
static void pvEnable(float isc = 5.0f, float voc = 60.0f, float k = 0.8f) {
    const auto ticket = mppt.queuePvCurve(isc, voc, k);
    TEST_ASSERT_NOT_EQUAL_UINT32(0, ticket);
    mppt.applyPendingPsuCommandRt(true);
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(ticket));
    TEST_ASSERT_EQUAL_INT((int) PsuSetpointError::None, (int) mppt.getPsuCommandError(ticket));
}

// --- 1. Parameter validation --------------------------------------------------

void test_pv_rejects_bad_params() {
    pvSetup();
    TEST_ASSERT_EQUAL_UINT32(0, mppt.queuePvCurve(NAN, 60, 0.8f));
    TEST_ASSERT_EQUAL_UINT32(0, mppt.queuePvCurve(0, 60, 0.8f));
    TEST_ASSERT_EQUAL_UINT32(0, mppt.queuePvCurve(5, NAN, 0.8f));
    TEST_ASSERT_EQUAL_UINT32(0, mppt.queuePvCurve(5, 60, 0.4f));  // k below realizable range
    TEST_ASSERT_EQUAL_UINT32(0, mppt.queuePvCurve(5, 60, 0.96f)); // k pathological
    TEST_ASSERT_EQUAL_UINT32(0, mppt.queuePvCurve(5, 60, NAN));
    TEST_ASSERT_EQUAL_UINT32(0, mppt.queuePvCurve(5, 90, 0.8f));  // voc > Vout_max
    TEST_ASSERT_FALSE(mppt.hasPendingPsuCommand());
}

void test_pv_rejects_ovset_conflict() {
    pvSetup();
    mppt.setExplicitOvLimit(60.0f);
    TEST_ASSERT_EQUAL_UINT32(0, mppt.queuePvCurve(5, 60, 0.8f));
    TEST_ASSERT_EQUAL_INT((int) PsuSetpointError::OvLimitConflict,
                          (int) mppt.getLastPsuRequestError());
    mppt.setExplicitOvLimit(NAN);
}

// --- 2. Entry / mode transitions ----------------------------------------------

void test_pv_enable_enters_psu_mode_at_voc() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    TEST_ASSERT_TRUE(g_app.psuMode());
    TEST_ASSERT_TRUE(mppt.pvSim.active);
    TEST_ASSERT_TRUE(mppt.isPvActive());
    TEST_ASSERT_EQUAL_FLOAT(60.0f, mppt.psuVsetpoint);
    float isc, voc, k;
    TEST_ASSERT_TRUE(mppt.getPvCurve(isc, voc, k));
    TEST_ASSERT_EQUAL_FLOAT(5.0f, isc);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, voc);
    TEST_ASSERT_EQUAL_FLOAT(0.8f, k);
}

void test_pv_enable_waits_for_fresh_telemetry() {
    pvSetup();
    const auto ticket = mppt.queuePvCurve(5, 60, 0.8f);
    TEST_ASSERT_NOT_EQUAL_UINT32(0, ticket);
    mppt.applyPendingPsuCommandRt(false);
    TEST_ASSERT_TRUE(mppt.hasPendingPsuCommand());
    TEST_ASSERT_FALSE(mppt.isPsuCommandDone(ticket));
    TEST_ASSERT_FALSE(g_app.psuMode());
    // drain so the next test starts clean
    mppt.applyPendingPsuCommandRt(true);
}

void test_pv_inplace_update_keeps_setpoint() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    mppt.psuVsetpoint = 40.0f; // mid-curve under load
    const auto ticket = mppt.queuePvCurve(2.5f, 60, 0.8f);
    TEST_ASSERT_NOT_EQUAL_UINT32(0, ticket);
    mppt.applyPendingPsuCommandRt(true);
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(ticket));
    TEST_ASSERT_EQUAL_FLOAT(40.0f, mppt.psuVsetpoint); // no jump to Voc
    TEST_ASSERT_TRUE(mppt.pvSim.active);
    TEST_ASSERT_EQUAL_FLOAT(2.5f, mppt.pvSim.model.isc);
}

void test_pv_scale_rebase_semantics() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, mppt.getPvBaseIsc());
    // rebase=false (the `pv scale` path) must not move the base
    const auto ticket = mppt.queuePvCurve(2.5f, 60, 0.8f, false, false);
    TEST_ASSERT_NOT_EQUAL_UINT32(0, ticket);
    mppt.applyPendingPsuCommandRt(true);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, mppt.getPvBaseIsc());
    TEST_ASSERT_EQUAL_FLOAT(2.5f, mppt.pvSim.model.isc);
}

void test_pv_disable_paths_clear_active() {
    pvSetup();
    pvEnable();
    auto t1 = mppt.requestPsuOff();
    mppt.applyPendingPsuCommandRt(true);
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(t1));
    TEST_ASSERT_FALSE(mppt.pvSim.active);
    TEST_ASSERT_FALSE(mppt.isPvActive());

    pvSetup();
    pvEnable();
    auto t2 = mppt.requestPsuManual(0, -1);
    mppt.applyPendingPsuCommandRt(true);
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(t2));
    TEST_ASSERT_FALSE(mppt.pvSim.active);
    TEST_ASSERT_TRUE(g_app.manualPwm());
}

void test_pv_plain_psu_enable_reverts_to_cv() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    const auto ticket = mppt.queuePsuSetpoint(50.0f);
    TEST_ASSERT_NOT_EQUAL_UINT32(0, ticket);
    mppt.applyPendingPsuCommandRt(true);
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(ticket));
    TEST_ASSERT_TRUE(g_app.psuMode());
    TEST_ASSERT_FALSE(mppt.pvSim.active);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, mppt.psuVsetpoint);
}

void test_pv_pending_overridden_by_manual() {
    pvSetup();
    const auto pvTicket = mppt.queuePvCurve(5, 60, 0.8f);
    const auto manualTicket = mppt.requestPsuManual(0, -1);
    mppt.applyPendingPsuCommandRt(true);
    TEST_ASSERT_FALSE(mppt.isPsuCommandDone(pvTicket)); // overwritten, never completes
    TEST_ASSERT_TRUE(mppt.isPsuCommandDone(manualTicket));
    TEST_ASSERT_FALSE(mppt.pvSim.active);
    TEST_ASSERT_TRUE(g_app.manualPwm());
}

// --- 3. Protection pinned at Voc ----------------------------------------------

void test_pv_ov_threshold_pinned_at_voc() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    mppt.psuVsetpoint = 35.0f; // dragged down the curve
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f * 1.5f > 80.0f ? 80.0f : 60.0f * 1.5f,
                             mppt.computeOvThreshold());
}

void test_pv_requested_setpoint_reports_voc() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    mppt.psuVsetpoint = 35.0f;
    // `ovset` validates against this — it must see the top of the curve, not the moving Vset
    TEST_ASSERT_EQUAL_FLOAT(60.0f, mppt.getRequestedPsuSetpoint());
}

void test_pv_trips_reach_latch() {
    pvSetup();
    pvEnable();
    loopWallClockUs_ = 1000000ULL;
    for (int i = 0; i < 10; ++i) {
        loopWallClockUs_ += 200000ULL;
        mppt.shutdownDcdc("Vout-OV-test", 5);
    }
    TEST_ASSERT_TRUE(mppt.psuLatched);
    TEST_ASSERT_EQUAL_STRING("psu-latch", mppt.startBlockReason());
    mppt.psuResetTripState();
}

// --- 4. Per-tick curve law ------------------------------------------------------

void test_pv_advance_slew_clamp() {
    pvSetup();
    pvEnable(5, 60, 0.8f); // Vset = 60, slew default 200 V/s
    mppt.pvAdvanceSetpoint(5.0f, NAN, 0.001f); // Iout = Isc -> curve wants 0 V
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 59.8f, mppt.psuVsetpoint); // one 0.2 V step only
}

void test_pv_advance_dt_clamp() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    mppt.pvAdvanceSetpoint(5.0f, NAN, 5.0f); // stale dt (post-backoff) clamps to 10 ms
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 58.0f, mppt.psuVsetpoint);
}

void test_pv_advance_vin_floor() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    for (int i = 0; i < 5000; ++i)
        mppt.pvAdvanceSetpoint(5.0f, 30.0f, 0.01f); // curve wants 0 V, floor Vin+0.5
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f + MpptController::BoostHeadroomV, mppt.psuVsetpoint);
}

void test_pv_advance_voc_ceiling_holds_on_empty_interval() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    // Vin risen into the headroom of Voc: floor > ceiling, hold the ceiling
    for (int i = 0; i < 100; ++i)
        mppt.pvAdvanceSetpoint(0.0f, 59.9f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, mppt.psuVsetpoint);
    // and the Voc-pinned feasibility now reports the fault (Vin + 0.5 > Voc is not yet
    // true at 59.9 -> still feasible; at 59.7 it is the sensor's call, skip that here)
}

void test_pv_advance_tracks_curve_point() {
    pvSetup();
    pvEnable(5, 60, 0.8f);
    const float iOp = 2.0f;
    const float vExpect = mppt.pvSim.model.voltage(iOp);
    for (int i = 0; i < 5000; ++i)
        mppt.pvAdvanceSetpoint(iOp, 20.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, vExpect, mppt.psuVsetpoint);
    TEST_ASSERT_TRUE(vExpect > 20.5f); // operating point genuinely on the curve, not the floor
}
