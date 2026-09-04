// Tests for src/buck.h SynchronousConverter diode-emulation math (the safety-critical
// reverse-current-prevention path): ripple current, CCM/DCM decision + hysteresis, and
// the rect/ctrl duty ratio. init() is driven from in-memory ConfFiles; skip_assert=1
// bypasses the GPIO pin-state checks, and init_pwm configures the real LEDC on-target.

#include <unity.h>
#include <esp_log.h>
#include <Arduino.h>

#include <string>
#include <stdexcept>

#include "util.h" // defines ESP_ERROR_CHECK_THROW used by pwm/ledc.h (included from buck.h)
#include "buck.h"

// fL = pwm_freq * L0 * 0.95 = 39000 * 50e-6 * 0.95 = 1.8525
// rippleCurrent(60,30) = 30/fL * (1 - 30/60) ~= 8.1 A
static constexpr float kFsw = 39000.f;

static void initConvEx(SynchronousConverter &c, const char *topo, const char *bootRefreshNs) {
    // Pin the MCPWM driver so the diode-emulation math is exercised against it (the production
    // driver on the s3 boards). Ignored on builds that compile only one driver.
    ConfFile converterConf{{{"topo", topo}}};
    converterConf.set("pwm_driver", "mcpwm"); // exercise the diode-emulation math on the MCPWM driver
    ConfFile coilConf{{{"L0", "50e-6"}}};
    ConfFile boardConf = bootRefreshNs
        ? ConfFile{{
              {"pwm_freq", "39000"}, {"pwm_driver_logic", "HiLi"},
              {"pwm_hi", "1"}, {"pwm_li", "2"}, {"skip_assert", "1"},
              {"boot_refresh_ns", bootRefreshNs},
          }}
        : ConfFile{{
              {"pwm_freq", "39000"}, {"pwm_driver_logic", "HiLi"},
              {"pwm_hi", "1"}, {"pwm_li", "2"}, {"skip_assert", "1"},
          }};
    c.init(converterConf, boardConf, coilConf);
}

static void initConv(SynchronousConverter &c) { initConvEx(c, "buck", nullptr); }

void test_buck_ripple_current() {
    SynchronousConverter c;
    initConv(c);
    TEST_ASSERT_FALSE(c.boost());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 8.1f, c.rippleCurrent(60.f, 30.f));
}

void test_buck_dcm_ccm_transition_and_hysteresis() {
    SynchronousConverter c;
    initConv(c);
    // ripple ~8.1 A; DCM while ripple > il*2, CCM above, with 1.8x release hysteresis
    c.updateSyncRectMaxDuty(60.f, 30.f, 1.f);
    TEST_ASSERT_TRUE(c.inDCM());
    c.updateSyncRectMaxDuty(60.f, 30.f, 4.f);
    TEST_ASSERT_TRUE(c.inDCM());          // 8.1 > 4*1.8=7.2 -> stays DCM
    c.updateSyncRectMaxDuty(60.f, 30.f, 5.f);
    TEST_ASSERT_FALSE(c.inDCM());         // 8.1 < 5*1.8=9 -> CCM
    c.updateSyncRectMaxDuty(60.f, 30.f, 10.f);
    TEST_ASSERT_FALSE(c.inDCM());
}

void test_buck_rect_ctrl_ratio() {
    SynchronousConverter c;
    initConv(c);
    // buck: rectCtrlRatio(m) = 1/m - 1
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, c.rectCtrlRatio(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 3.0f, c.rectCtrlRatio(0.25f));
}

void test_buck_current_sweep_no_crash() {
    SynchronousConverter c;
    initConv(c);
    // mirror the host-stub sweep: must not trip any internal assert / range error
    for (float il = 0.1f; il < 30.f; il *= 1.1f)
        c.updateSyncRectMaxDuty(60.f, 30.f, il);
    TEST_PASS();
}

// --- regression: convRatioWCE clamp. The WCEF division (/0.98) inflates M; without the clamp it
//     can push M past its physical bound (buck: >1), making rectCtrlRatio() = 1/M-1 negative. ---

void test_buck_ratio_clamped_below_unity() {
    SynchronousConverter c;
    initConv(c);
    // M = vout/vin ~ 0.997 -> constrain 0.99 -> /WCEF ~1.01 -> clamp back to <1
    c.updateSyncRectMaxDuty(30.f, 29.9f, 1.f);
    TEST_ASSERT_TRUE(c.voltageRatio() > 0.f);
    TEST_ASSERT_TRUE(c.voltageRatio() < 1.f); // not pushed to/over unity
    TEST_ASSERT_TRUE(c.rectCtrlRatio(c.voltageRatio()) >= 0.f);
}

void test_buck_ratio_clamped_when_vout_ge_vin() {
    SynchronousConverter c;
    initConv(c);
    // vout >= vin -> fallback M=1.0 -> /WCEF ~1.02 -> clamp back to 0.99
    c.updateSyncRectMaxDuty(30.f, 30.f, 1.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.99f, c.voltageRatio());
    TEST_ASSERT_TRUE(c.rectCtrlRatio(c.voltageRatio()) >= 0.f);
}

void test_buck_ratio_never_negative_sweep() {
    SynchronousConverter c;
    initConv(c);
    for (float vout = 3.f; vout <= 36.f; vout += 1.f) {
        c.updateSyncRectMaxDuty(30.f, vout, 1.f);
        TEST_ASSERT_TRUE(c.voltageRatio() > 0.f && c.voltageRatio() < 1.f);
        TEST_ASSERT_TRUE(c.rectCtrlRatio(c.voltageRatio()) >= 0.f);
    }
}

// In DCM near M=1 with a nonzero duty, the (tiny, positive) ratio must floor pwmRectMax at the
// bootstrap min. Without the clamp the negative ratio would wrap (uint16) to the full CCM
// complement (pwmMax - pwmCtrl) — the opposite of the safe short LS time.
void test_buck_dcm_rectmax_not_full_ccm_near_unity() {
    SynchronousConverter c;
    initConv(c);
    c.pwmPerturb(c.pwmCtrlMax / 2); // nonzero duty so the DCM ratio actually multiplies
    c.updateSyncRectMaxDuty(30.f, 29.9f, 0.05f); // il<0.1 -> DCM, il>=0.01 & vl>=1 -> ratio path
    TEST_ASSERT_TRUE(c.inDCM());
    TEST_ASSERT_EQUAL_UINT(c.getRectOnPwmMin(), c.getRectOnPwmMax());
    TEST_ASSERT_TRUE(c.getRectOnPwmMax() < (uint16_t) (c.pwmMaxDriver() - c.getCtrlOnPwmCnt()));
}

// --- DCM sync-rect-off thresholds (SyncRectOffCurrent / SyncRectOffVoltage). ---

void test_buck_sync_rect_off_below_min_current() {
    SynchronousConverter c;
    initConv(c);
    c.pwmPerturb(c.pwmCtrlMax / 2);
    c.updateSyncRectMaxDuty(60.f, 30.f, 0.005f); // il < SyncRectOffCurrent -> disable sync rect
    TEST_ASSERT_TRUE(c.inDCM());
    TEST_ASSERT_EQUAL_UINT(c.getRectOnPwmMin(), c.getRectOnPwmMax());
}

void test_buck_sync_rect_active_in_dcm_with_current() {
    SynchronousConverter c;
    initConv(c);
    c.pwmPerturb(c.pwmCtrlMax / 2);
    c.updateSyncRectMaxDuty(60.f, 30.f, 1.f); // ripple 8.1 > 2*1 -> DCM, current healthy
    TEST_ASSERT_TRUE(c.inDCM());
    TEST_ASSERT_TRUE(c.getRectOnPwmMax() > c.getRectOnPwmMin()); // sync rect conducting
}

// --- boot_refresh_ns -> pwmRectMin count conversion (replaces the old fixed 6%). ---

void test_buck_bootstrap_min_default() {
    // Assert the INTENT, not the literal. The previous version of this test recomputed the
    // expectation with the same `500e-9` constant the code used, so it compared the default
    // against itself and could never detect a wrong one -- and it duly held green while the
    // default sat at a third of the required refresh (measured on fbuck: Lm=80, starved HS
    // gate drive, no fast turn-on at all with the output open).
    //
    // The bound is EMPIRICAL, not the 6% of the MinDutyCycleLS this conf replaced: 6% is ~244
    // counts here and fbuck was measured to need >= 300. Below that the HS gate drive does not
    // fail cleanly, it fires in bursts, which is far harder to spot than a dead gate.
    //
    // Asserted as a floor, not an equality, so raising the default for more margin does not
    // break the test while lowering it back into the burst region does.
    SynchronousConverter c;
    initConv(c); // no boot_refresh_ns -> default from buck.h
    const float minLsDutyMeasured = 300.f / 4069.f; // fbuck 2026-08-11, 39.3 kHz, MCPWM
    auto floorCounts = (float) c.pwmMaxDriver() * minLsDutyMeasured;
    TEST_ASSERT_TRUE_MESSAGE((float) c.getRectOnPwmMin() >= floorCounts,
                             "default boot_refresh_ns is below the measured HS-bootstrap floor");
}

void test_buck_bootstrap_min_scales_with_conf() {
    SynchronousConverter c;
    initConvEx(c, "buck", "3000"); // double the refresh time -> ~double the count
    auto expect = (uint16_t) std::ceil(3000e-9f * kFsw * (float) c.pwmMaxDriver());
    TEST_ASSERT_EQUAL_UINT(expect, c.getRectOnPwmMin());
}

void test_boost_bootstrap_min_is_zero() {
    SynchronousConverter c;
    initConvEx(c, "boost", nullptr); // boost: ctrl=LS pulls node low every cycle, no LS floor
    TEST_ASSERT_TRUE(c.boost());
    TEST_ASSERT_EQUAL_UINT(0, c.getRectOnPwmMin());
}

// boost-side symmetric clamp: M = vout/vin must stay > 1 so rectCtrlRatio() = 1/(M-1) is finite.
void test_boost_ratio_clamped_above_unity() {
    SynchronousConverter c;
    initConvEx(c, "boost", nullptr);
    c.updateSyncRectMaxDuty(30.f, 30.3f, 1.f); // vin=30, vout=30.3, M just above 1
    TEST_ASSERT_TRUE(c.voltageRatio() > 1.f);
    TEST_ASSERT_TRUE(std::isfinite(c.rectCtrlRatio(c.voltageRatio())));
    TEST_ASSERT_TRUE(c.rectCtrlRatio(c.voltageRatio()) >= 0.f);
}

#if defined(HAVE_MCPWM) && defined(HAVE_LEGACY)
// Both drivers compiled: converter.conf::pwm_driver picks the active one at runtime. At the same
// fsw the LEDC and MCPWM timers land on different resolutions, so pwmCounts() differs — a cheap
// observable that the selected driver actually came up (rather than always falling to the default).
static void initConvDriver(SynchronousConverter &c, const char *drv, int pinHi, int pinLi) {
    ConfFile cc{{{"topo", "buck"}}};
    cc.set("pwm_driver", drv);
    ConfFile coil{{{"L0", "50e-6"}}};
    ConfFile bc{{{"pwm_freq", "39000"}, {"pwm_driver_logic", "HiLi"}, {"skip_assert", "1"}}};
    bc.set("pwm_hi", std::to_string(pinHi));
    bc.set("pwm_li", std::to_string(pinLi));
    c.init(cc, bc, coil);
}

void test_buck_pwm_driver_runtime_select() {
    SynchronousConverter cl, cm;
    initConvDriver(cl, "ledc", 1, 2);
    initConvDriver(cm, "mcpwm", 4, 5);
    TEST_ASSERT_TRUE(cl.pwmCounts() > 0);
    TEST_ASSERT_TRUE(cm.pwmCounts() > 0);
    // distinct resolutions -> the runtime choice took effect on each converter
    TEST_ASSERT_TRUE(cm.pwmCounts() != cl.pwmCounts());
    TEST_ASSERT_TRUE(cl.pwmCounts() < 2500);  // ledc ~2047 at 39 kHz
    TEST_ASSERT_TRUE(cm.pwmCounts() > 3000);  // mcpwm ~4103 at 39 kHz
}

// Absent pwm_driver -> default ledc (an MCPWM board must opt in via converter.conf).
void test_pwm_driver_defaults_to_ledc() {
    SynchronousConverter c;
    ConfFile cc{{{"topo", "buck"}}};   // no pwm_driver key
    ConfFile coil{{{"L0", "50e-6"}}};
    ConfFile bc{{{"pwm_freq", "39000"}, {"pwm_driver_logic", "HiLi"}, {"skip_assert", "1"}}};
    bc.set("pwm_hi", "1");
    bc.set("pwm_li", "2");
    c.init(cc, bc, coil);
    TEST_ASSERT_TRUE(c.pwmCounts() < 2500);   // ledc resolution, not mcpwm
}

// An unrecognized pwm_driver value is rejected (throws) rather than silently picking a driver.
void test_pwm_driver_invalid_throws() {
    SynchronousConverter c;
    ConfFile cc{{{"topo", "buck"}}};
    cc.set("pwm_driver", "bogus");
    ConfFile coil{{{"L0", "50e-6"}}};
    ConfFile bc{{{"pwm_freq", "39000"}, {"pwm_driver_logic", "HiLi"}, {"skip_assert", "1"}}};
    bc.set("pwm_hi", "1");
    bc.set("pwm_li", "2");
    bool threw = false;
    try { c.init(cc, bc, coil); } catch (const std::exception &) { threw = true; }
    TEST_ASSERT_TRUE(threw);
}
#endif

// ---- runtime switching-frequency change (console `pwm-freq`) --------------------------------

// The whole design rests on the tick rate NOT moving with the frequency: bestTiming keeps the
// prescaler at 1 across the legal range on a 160 MHz source, so everything stored as a TIME
// (dead-time, rect_offset_ns, boot_refresh_ns) stays correct without being re-derived. If this
// ever fails, applyPendingPwmFreqRt() silently rescales those calibrations.
void test_pwm_freq_prescaler_invariant() {
    for (uint32_t f = 5100; f <= 500000; f += 1100) {
        PwmTiming t = bestTiming(f);
        char msg[64];
        snprintf(msg, sizeof msg, "f=%u res=%u ticks=%u", (unsigned) f,
                 (unsigned) t.resolution_hz, (unsigned) t.period_ticks);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(160000000u, t.resolution_hz, msg);
        TEST_ASSERT_TRUE_MESSAGE(t.period_ticks >= 320 && t.period_ticks <= 32000, msg);
    }
}

// Every refusal leaves the converter untouched and queues nothing. L0=50e-6 here, so the
// fsw*L0*0.95 window closes below ~21.05 kHz.
void test_pwm_freq_refusals() {
    SynchronousConverter c;
    initConv(c);
    const uint16_t period0 = c.getPeriodTicks(), max0 = c.pwmMaxDriver(), ctrlMax0 = c.pwmCtrlMax;
    uint16_t ticks = 0;
    TEST_ASSERT_NOT_NULL(c.requestPwmFrequency(4000, ticks));
    TEST_ASSERT_NOT_NULL(c.requestPwmFrequency(600000, ticks));
    TEST_ASSERT_NOT_NULL(c.requestPwmFrequency(20000, ticks));   // fsw*L0 <= 1
    TEST_ASSERT_TRUE(c.pwmFreqIdle());
    TEST_ASSERT_EQUAL_UINT16(period0, c.getPeriodTicks());
    TEST_ASSERT_EQUAL_UINT16(max0, c.pwmMaxDriver());
    TEST_ASSERT_EQUAL_UINT16(ctrlMax0, c.pwmCtrlMax);
}

// A round trip preserves the duty RATIO and leaves every ns-derived count bit-identical.
void test_pwm_freq_roundtrip_rescales_duty() {
    SynchronousConverter c;
    initConv(c);
    if (!c.getPeriodTicks()) TEST_IGNORE_MESSAGE("no MCPWM leg in this build");
    const uint16_t p39 = c.getPeriodTicks();
    TEST_ASSERT_EQUAL_UINT16(4103, p39);
    const uint16_t hl0 = c.getDtHlTicks(), lh0 = c.getDtLhTicks(), rectMin0 = c.getRectOnPwmMin();
    const int16_t off0 = c.getRectOnOffset();

    c.pwmPerturb((int16_t) (p39 / 4));
    const uint16_t duty0 = c.getCtrlOnPwmCnt();
    TEST_ASSERT_GREATER_THAN_UINT16(0, duty0);
    const float ratio0 = (float) duty0 / (float) p39;

    uint16_t ticks = 0;
    TEST_ASSERT_NULL(c.requestPwmFrequency(75000, ticks));
    TEST_ASSERT_EQUAL_UINT16(2133, ticks);
    TEST_ASSERT_TRUE(c.applyPendingPwmFreqRt() > 0.f);
    c.ackPendingPwmFreqRt();   // the RT caller owns the ack; without it the next request is refused
    TEST_ASSERT_TRUE(c.pwmFreqIdle());
    TEST_ASSERT_EQUAL_UINT16(2133, c.getPeriodTicks());
    TEST_ASSERT_EQUAL_UINT16((uint16_t) (2133 - lh0), c.pwmMaxDriver());
    // one count of quantization on a 2133-tick period
    TEST_ASSERT_FLOAT_WITHIN(1.f / 2133.f, ratio0, (float) c.getCtrlOnPwmCnt() / 2133.f);
    // fixed times at a fixed tick: untouched by the frequency change
    TEST_ASSERT_EQUAL_UINT16(hl0, c.getDtHlTicks());
    TEST_ASSERT_EQUAL_UINT16(lh0, c.getDtLhTicks());
    TEST_ASSERT_EQUAL_UINT16(rectMin0, c.getRectOnPwmMin());
    TEST_ASSERT_EQUAL_INT16(off0, c.getRectOnOffset());

    // A second request while the first is still un-acked is refused, not queued behind it: that is
    // what keeps one transaction in the mailbox and stops the producer reading RT-owned state.
    {
        uint16_t t2 = 0;
        TEST_ASSERT_NULL(c.requestPwmFrequency(48000, t2));
        TEST_ASSERT_NOT_NULL(c.requestPwmFrequency(60000, t2));
        TEST_ASSERT_TRUE(c.applyPendingPwmFreqRt() > 0.f);
        c.ackPendingPwmFreqRt();
        TEST_ASSERT_EQUAL_UINT16(3333, c.getPeriodTicks());
    }
    TEST_ASSERT_NULL(c.requestPwmFrequency(39000, ticks));
    TEST_ASSERT_EQUAL_UINT16(p39, ticks);
    TEST_ASSERT_TRUE(c.applyPendingPwmFreqRt() > 0.f);
    c.ackPendingPwmFreqRt();   // the RT caller owns the ack; without it the next request is refused
    TEST_ASSERT_TRUE(c.pwmFreqIdle());
    TEST_ASSERT_EQUAL_UINT16(p39, c.getPeriodTicks());
    TEST_ASSERT_FLOAT_WITHIN(2.f / (float) p39, ratio0, (float) c.getCtrlOnPwmCnt() / (float) p39);
    TEST_ASSERT_EQUAL_UINT16(rectMin0, c.getRectOnPwmMin());
}
