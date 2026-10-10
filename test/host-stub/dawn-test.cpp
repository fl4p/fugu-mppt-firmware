// Host-stub runner for the 2026-05-31 dawn replay (mirrors test/test_dawn.cpp, which uses the real
// on-target Sensor). Here we drive the same production filter classes (RunningMedian5 + EWM from
// math/statmath.h) and the same VirtualConverter PV plant (sim/vconv.h) natively, reproducing
// Sensor::add_sample's pipeline for a VirtualSensor (identity transform, no offset, no notch).
//
//   clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src test/host-stub/dawn-test.cpp -o /tmp/dawn-test
//
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <algorithm>
using std::isnan;
using std::min;   // statmath.h::median() calls min()/max() unqualified (Arduino macros on-target)
using std::max;
#define likely(x) (x)        // mock.h defines unlikely() only; statmath.h needs both
#include "mock.h"
#include "math/statmath.h"   // RunningMedian5, EWM — the real Sensor filter chain
#include "sim/vconv.h"       // VirtualConverter PV model

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_fail; } } while (0)

static constexpr float kVbat = 26.6f;
static constexpr uint32_t kSpan = 20;     // sensor.conf Vin _filt_len (hv_v_filt_len in a buck)
static constexpr int kSps = 170;          // Vin per-channel rate (~511sps / 3)

static const float kVocRamp[] = {
    48.4f, 49.2f, 50.9f, 52.0f, 53.7f, 55.2f, 56.5f, 57.8f, 58.9f, 59.8f,
    60.6f, 61.5f, 62.1f, 62.9f, 63.4f, 63.9f, 64.4f, 64.9f, 65.3f, 65.7f,
    66.1f, 66.5f, 66.7f, 67.1f, 67.2f,
};

// One sensor channel: exactly Sensor::add_sample's path for a VirtualSensor.
struct Chan {
    RunningMedian5<float> med3{};
    EWM<false, float> ewm{kSpan};
    void add(float x) { ewm.add(med3.next(x)); }
    float avg() const { return ewm.avg.get(); }
};

static bool vinGateOpen(const Chan &vin, const Chan &vout) {
    return vin.avg() > vout.avg() + 1.f;
}
static void feed(Chan &vin, Chan &vout, float v, int n) {
    for (int i = 0; i < n; ++i) { vin.add(v); vout.add(kVbat); }
}

static void test_dawn_vin_gate_clears_fast_and_stays_open() {
    printf("test_dawn_vin_gate_clears_fast_and_stays_open\n");
    Chan vin, vout;

    feed(vin, vout, 26.2f, kSps * 3);
    CHECK(!vinGateOpen(vin, vout), "gate should be closed at Voc=26.2");
    feed(vin, vout, 27.0f, kSps);
    CHECK(!vinGateOpen(vin, vout), "gate should be closed at Voc=27.0 (< Vbat+1)");

    int samplesToOpen = -1;
    for (int i = 0; i < kSps * 5; ++i) {
        vin.add(kVocRamp[0]); vout.add(kVbat);
        if (vinGateOpen(vin, vout)) { samplesToOpen = i + 1; break; }
    }
    CHECK(samplesToOpen > 0, "gate never opened after Voc stepped above Vbat");
    CHECK(samplesToOpen < kSps, "Vin EWM took >1s to cross — not the EWM's doing");
    printf("  samplesToOpen=%d (< %d = 1s)\n", samplesToOpen, kSps);

    for (float v : kVocRamp) {
        feed(vin, vout, v, 40);
        CHECK(vinGateOpen(vin, vout), "Vin gate dipped closed mid-ramp");
    }
}

static void test_dawn_high_voc_panel_only_yields_a_few_watts() {
    printf("test_dawn_high_voc_panel_only_yields_a_few_watts\n");
    VirtualConverter pv;
    pv.setPv(0.18f, 67.0f, 0.74f);

    CHECK(std::fabs(pv.pvCurrent(67.0f)) < 1e-3f, "I(Voc) should be ~0");

    float pMax = 0.f, vMpp = 0.f;
    for (float v = 1.f; v < 67.0f; v += 0.25f) {
        float p = v * pv.pvCurrent(v);
        if (p > pMax) { pMax = p; vMpp = v; }
    }
    CHECK(pMax > 1.f && pMax < 15.f, "dawn MPP should be a few watts");
    CHECK(std::fabs(vMpp - 49.6f) < 6.f, "MPP voltage should sit near k*Voc");
    printf("  pMax=%.2fW vMpp=%.1fV\n", pMax, vMpp);
}

// Integration check: rules OUT sensor calibration as the dawn blocker. startSweep() triggers a
// calibration that must clear  ewm.std*|avg| <= maxStddev (vin maxStddev=1.8) to start. ewm.std is
// a NORMALISED variance (~(dx/avg)^2, see statmath.h::EWM::add), so the product ~ dx^2/avg actually
// SHRINKS at high Voc — fry's high-voltage string passes more easily, not less. Drive the real EWM
// over the dawn ramp + generous 0.1V noise and confirm the gate passes with huge margin.
static void test_dawn_calibration_not_the_blocker() {
    printf("test_dawn_calibration_not_the_blocker\n");
    const float maxStddev = 1.8f;                 // sensor_setup.cpp vin calibrationConstraints
    EWM<false, float> e{kSpan};
    unsigned seed = 2026;
    auto rnd = [&] { seed = seed * 1103515245u + 12345u; return ((seed >> 16) & 0x7fff) / 32767.f * 2 - 1; };
    float v = 65.0f;                               // worst case: highest Voc (06:30), still ramping
    for (int i = 0; i < kSps * 60; ++i) { e.add(v + 0.1f * rnd()); v += 0.031f / kSps; }
    float gate = e.std.get() * std::fabs(e.avg.get());
    CHECK(gate < maxStddev, "calibration gate should pass on the dawn ramp");
    printf("  calib gate std*|avg|=%.4f < %.1f -> calibration ruled out as the blocker\n", gate, maxStddev);
}

int main() {
    test_dawn_vin_gate_clears_fast_and_stays_open();
    test_dawn_high_voc_panel_only_yields_a_few_watts();
    test_dawn_calibration_not_the_blocker();
    printf(g_fail ? "\n%d FAILURES\n" : "\nALL PASS\n", g_fail);
    return g_fail ? 1 : 0;
}
