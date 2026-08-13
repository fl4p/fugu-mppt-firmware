// Tests for src/pd_control.h (PD_Control, PD_Control_SmoothSetpoint). Pure math.
// pd_control.h relies on the includer for EWMA / std::isnan / NAN.

#include <unity.h>
#include <cmath>
#include <esp_log.h>
#include <Arduino.h>

#include "math/statmath.h"
#include "pd_control.h"

// dt is ignored while Td is NaN (legacy per-sample Kd), so the value is arbitrary here.
static constexpr float DT = 1e-3f;

void test_pd_proportional_only() {
    PD_Control pd{2.f, 0.f, false};
    // first update: de forced to 0 -> output is purely Kp*e
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f * (5.f - 3.f), pd.update(3.f, 5.f, DT));
    // steady measurement: de == 0, same output
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 4.f, pd.update(3.f, 5.f, DT));
}

void test_pd_first_tick_has_zero_derivative() {
    PD_Control pd{0.f, 1.f, false}; // derivative-only
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.f, pd.update(0.f, 0.f, DT)); // _prevE NaN -> de=0
}

void test_pd_derivative_on_step() {
    PD_Control pd{0.f, 1.f, false};
    pd.update(0.f, 0.f, DT);                                  // e=0, primes _prevE
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, pd.update(0.f, 2.f, DT)); // de = 2-0 = 2
}

void test_pd_normalize_relative_error() {
    PD_Control pd{1.f, 0.f, true};
    // normalize: measurement/=setpoint, setpoint=1 -> e = 1 - 0.5 = 0.5
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, pd.update(4.f, 8.f, DT));
}

void test_pd_reset_clears_derivative() {
    PD_Control pd{0.f, 1.f, false};
    pd.update(0.f, 0.f, DT);
    pd.update(0.f, 5.f, DT); // _prevE now 5
    pd.reset();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.f, pd.update(0.f, 5.f, DT)); // de=0 again after reset
}

void test_pd_smooth_setpoint_lags_step() {
    // SmoothSetpoint forces normalize=true, so output is the relative error
    // e = 1 - measurement/smoothed_setpoint. With a fixed measurement, a setpoint
    // step appears only through the EWMA-smoothed setpoint, which lags.
    PD_Control_SmoothSetpoint pd{1.f, 0.f, 10};
    float first = pd.update(5.f, 10.f, DT);   // seeds smoothed setpoint at 10: e = 1 - 5/10 = 0.5
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, first);
    float stepped = pd.update(5.f, 20.f, DT); // un-smoothed would give 1 - 5/20 = 0.75
    TEST_ASSERT_TRUE(stepped > 0.5f && stepped < 0.75f); // lag: between old and new
}

// --- Td (sample-rate invariant derivative) ------------------------------------------------

// Legacy Kd applies to the raw per-sample difference, so the SAME error step yields the same D
// output no matter how much time passed -- that is the coupling Td exists to remove.
// Kp is deliberately non-zero: with Kp=0 the Td path (Kp*Td*de/dt) is 0 for ANY Td, so the test
// would pass whether Td defaulted to NaN, 0 or 5ms -- blind to the regression it exists to catch.
void test_pd_legacy_kd_is_sample_rate_coupled() {
    PD_Control fast{1.f, 1.f, false}, slow{1.f, 1.f, false};
    TEST_ASSERT_TRUE(std::isnan(fast.Td)); // legacy path is the default
    fast.update(0.f, 0.f, 1e-3f);
    slow.update(0.f, 0.f, 4e-3f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, fast.update(0.f, 2.f, 1e-3f), slow.update(0.f, 2.f, 4e-3f));
}

// With Td set, the D component is a slope (de/dt), so quadrupling dt quarters it.
void test_pd_td_is_sample_rate_invariant() {
    PD_Control fast{1.f, 0.f, false}, slow{1.f, 0.f, false};
    fast.Td = 2e-3f;
    slow.Td = 2e-3f;
    fast.update(0.f, 0.f, 1e-3f);
    slow.update(0.f, 0.f, 4e-3f);
    // Kp*e is 2 in both; D adds Kp*Td*de/dt = 1*2e-3*2/dt
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f + 4.f, fast.update(0.f, 2.f, 1e-3f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f + 1.f, slow.update(0.f, 2.f, 4e-3f));
}

// Migration check: Td = (Kd/Kp)*Ts reproduces the legacy output exactly at that Ts. This is how
// the per-board defaults were derived, so a board can move to Td without changing behaviour.
void test_pd_td_matches_legacy_at_nominal_ts() {
    constexpr float Kp = 1500.f, Kd = 12000.f, Ts = 576e-6f;
    PD_Control legacy{Kp, Kd, false}, timed{Kp, Kd, false};
    timed.Td = (Kd / Kp) * Ts;
    legacy.update(0.f, 0.f, Ts);
    timed.update(0.f, 0.f, Ts);
    float a = legacy.update(0.f, 0.02f, Ts), b = timed.update(0.f, 0.02f, Ts);
    TEST_ASSERT_FLOAT_WITHIN(fabsf(a) * 1e-5f, a, b);
}

// --- pdLoadGains -------------------------------------------------------------------------

// The load-bearing claim of the conf refactor: a board that sets none of the keys is unchanged.
void test_pd_load_gains_empty_conf_is_noop() {
    PD_Control pd{1500.f, 12000.f, true};
    ConfFile empty{};
    pdLoadGains(empty, pd, "vout");
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1500.f, pd.Kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12000.f, pd.Kd);
    TEST_ASSERT_TRUE(std::isnan(pd.Td)); // absent _td must leave the legacy sentinel intact
}

void test_pd_load_gains_applies_keys() {
    PD_Control pd{1500.f, 12000.f, true};
    ConfFile c{};
    c.set("ctrl_vout_kp", "300");
    c.set("ctrl_vout_td", "0.0046");
    pdLoadGains(c, pd, "vout");
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 300.f, pd.Kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0046f, pd.Td);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12000.f, pd.Kd); // untouched key keeps its default
}

// A negative Td inverts the D term, which on a limiter flips "not limiting" into "limit hard".
void test_pd_load_gains_rejects_negative_td() {
    PD_Control pd{1500.f, 12000.f, true};
    ConfFile c{};
    c.set("ctrl_vout_td", "-0.0046");
    pdLoadGains(c, pd, "vout");
    TEST_ASSERT_TRUE(std::isnan(pd.Td)); // rejected -> falls back to legacy Kd
}

// "nan"/"inf" parse cleanly through strtof; a non-finite gain would latch ctrl-nan forever.
void test_pd_load_gains_rejects_nonfinite_kp() {
    PD_Control pd{1500.f, 12000.f, true};
    ConfFile c{};
    c.set("ctrl_vout_kp", "nan");
    c.set("ctrl_vout_kd", "inf");
    pdLoadGains(c, pd, "vout");
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1500.f, pd.Kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12000.f, pd.Kd);
}

// Calibration against a known-bad input: a negative Td, if it were accepted, flips the sign of a
// limiter's verdict. Pins the consequence, not just the guard.
void test_pd_negative_td_would_flip_limiter_sign() {
    constexpr float Kp = 1500.f, Ts = 576e-6f;
    PD_Control good{Kp, 0.f, true}, bad{Kp, 0.f, true};
    good.Td = 8.f * Ts;
    bad.Td = -8.f * Ts;
    good.update(1.f, 1.f, Ts);
    bad.update(1.f, 1.f, Ts);
    float g = good.update(0.98f, 1.f, Ts), b = bad.update(0.98f, 1.f, Ts);
    TEST_ASSERT_TRUE(g > 0.f);
    TEST_ASSERT_TRUE(b < 0.f); // same input, opposite verdict
}

// dt<=0 (first tick, or a stalled clock) must drop the D component, not divide by zero.
void test_pd_td_zero_dt_suppresses_derivative() {
    PD_Control pd{1.f, 0.f, false};
    pd.Td = 2e-3f;
    pd.update(0.f, 0.f, 0.f);
    float v = pd.update(0.f, 2.f, 0.f); // de=2 but dt=0 -> D dropped, only Kp*e remains
    TEST_ASSERT_TRUE(std::isfinite(v));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, v);
}
