#include <unity.h>
#include <cmath>

#include "adc/sampling.h"   // Sensor / VirtualSensor + the real notch/median/EWM pipeline
#include "sim/vconv.h"      // VirtualConverter PV model

// Replays fry's dawn on 2026-05-31 (high-voltage string, converter idle in START).
// Measured Vin (= panel Voc while unloaded) from InfluxDB mppt/device=fry, 30s buckets, local time:
//   06:05 26.2 .. 06:10:00 27.0 -> step 06:10:30 48.4 -> smooth climb -> 06:32:30 67.2,
//   then the converter finally swept at 06:33:00 (MPP only 6.8W). Vout held ~26.6 (battery).
//
// The puzzle these tests pin down: Vin crossed Vout+1 (~27.6V) at 06:10:30 yet the converter did
// not start until 06:33 — a 22-min gap. startCondition()'s Vin clause is the obvious suspect, so
// test 1 proves (with the production EWM) that the clause clears within a second of the crossing
// and then stays clear for the whole ramp — i.e. the stall is NOT the Vin term, it's one of the
// other clauses (backoff / temp / calibration), which the on-device "START blocked:" log now names.

static constexpr float kVbat = 26.6f;         // Vout (battery) over the window
static constexpr uint32_t kSpan = 20;         // sensor.conf Vin/Vout _filt_len (hv_v_/lv_v_filt_len in a buck)
static constexpr int kSps = 170;              // Vin per-channel rate (~511sps / 3 channels)

// Measured Voc ramp after the dawn step (06:10:30 onward), volts. Monotone 48->67.
static const float kVocRamp[] = {
    48.4f, 49.2f, 50.9f, 52.0f, 53.7f, 55.2f, 56.5f, 57.8f, 58.9f, 59.8f,
    60.6f, 61.5f, 62.1f, 62.9f, 63.4f, 63.9f, 64.4f, 64.9f, 65.3f, 65.7f,
    66.1f, 66.5f, 66.7f, 67.1f, 67.2f,
};

// startCondition()'s buck Vin clause, evaluated on the live EWM averages.
static bool vinGateOpen(const Sensor &vin, const Sensor &vout) {
    return vin.ewm.avg.get() > vout.ewm.avg.get() + 1.f;
}

static void feed(Sensor &vin, Sensor &vout, float vinVal, int n) {
    for (int i = 0; i < n; ++i) {
        vin.add_sample(vinVal);
        vout.add_sample(kVbat);
    }
}

// The Vin/Voc clause clears within ~1s of the real Voc crossing and stays open across the whole
// 22-min ramp — so it cannot be what held fry in START until 06:33.
void test_dawn_vin_gate_clears_fast_and_stays_open() {
    VirtualSensor vin([] { return 0.f; }, kSpan, "vin", 'V');
    VirtualSensor vout([] { return 0.f; }, kSpan, "vout", 'V');

    // Pre-dawn: Voc sits at/below the battery (26.2V). Gate must stay CLOSED.
    feed(vin, vout, 26.2f, kSps * 3);
    TEST_ASSERT_FALSE(vinGateOpen(vin, vout));
    feed(vin, vout, 27.0f, kSps);                     // 06:10:00, still < Vout+1
    TEST_ASSERT_FALSE(vinGateOpen(vin, vout));

    // 06:10:30 step to 48.4V. Count samples until the gate opens.
    int samplesToOpen = -1;
    for (int i = 0; i < kSps * 5; ++i) {
        vin.add_sample(kVocRamp[0]);
        vout.add_sample(kVbat);
        if (vinGateOpen(vin, vout)) { samplesToOpen = i + 1; break; }
    }
    TEST_ASSERT_TRUE_MESSAGE(samplesToOpen > 0, "gate never opened after Voc stepped above Vbat");
    // EWM lag is sub-second: nowhere near the observed 22-min START delay.
    TEST_ASSERT_TRUE_MESSAGE(samplesToOpen < kSps, "Vin EWM took >1s to cross — not the EWM's doing");

    // Replay the full measured 48->67V climb; the gate must never fall closed again.
    for (float v : kVocRamp) {
        feed(vin, vout, v, 40);
        TEST_ASSERT_TRUE_MESSAGE(vinGateOpen(vin, vout), "Vin gate dipped closed mid-ramp");
    }
}

// Rules OUT sensor calibration as the dawn blocker. startSweep() triggers a calibration that must
// clear  ewm.std*|avg| <= maxStddev (vin maxStddev=1.8, sensor_setup.cpp). ewm.std is a NORMALISED
// variance (~(dx/avg)^2, statmath.h::EWM::add), so the product ~ dx^2/avg shrinks at high Voc —
// fry's high-voltage string passes more easily, not less. Driven on the real Sensor EWM.
void test_dawn_calibration_not_the_blocker() {
    VirtualSensor vin([] { return 0.f; }, kSpan, "vin", 'V');
    float v = 65.0f;                              // worst case: highest Voc (06:30), still ramping
    for (int i = 0; i < kSps * 60; ++i) { vin.add_sample(v); v += 0.031f / kSps; }
    float gate = vin.ewm.std.get() * std::fabs(vin.ewm.avg.get());
    TEST_ASSERT_TRUE_MESSAGE(gate < 1.8f, "calibration gate should pass on the dawn ramp");
}

// Why production was negligible regardless of when it started: a high-Voc string at dawn light
// sits near Voc with only a few watts available. Models today's panel (Voc~67, sweep MPP 6.8W).
void test_dawn_high_voc_panel_only_yields_a_few_watts() {
    VirtualConverter pv;
    pv.setPv(/*isc*/ 0.18f, /*voc*/ 67.0f, /*k=Vmpp/Voc*/ 0.74f);

    // Unloaded (converter idle in START) the panel rests at Voc -> zero current, zero power.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.f, pv.pvCurrent(67.0f));

    // Sweep the I-V curve for the max power point.
    float pMax = 0.f, vMpp = 0.f;
    for (float v = 1.f; v < 67.0f; v += 0.25f) {
        float p = v * pv.pvCurrent(v);
        if (p > pMax) { pMax = p; vMpp = v; }
    }
    // Only a handful of watts (vs the ~800W this string makes at full sun) and the MPP sits near
    // k*Voc ~ 49.6V — matching the 6.8W@49.6V the device logged when it finally swept at 06:33.
    TEST_ASSERT_TRUE_MESSAGE(pMax > 1.f && pMax < 15.f, "dawn MPP should be a few watts");
    TEST_ASSERT_FLOAT_WITHIN(6.f, 49.6f, vMpp);
}
