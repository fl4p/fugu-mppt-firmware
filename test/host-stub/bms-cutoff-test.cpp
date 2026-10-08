// Host test for bmsCutoffSignature() (battery.h) against the VirtualConverter plant at fixed duty,
// i.e. before any control-loop reaction. Each channel is a point sample of the cycle-average value,
// filtered med5 -> EWM. Not modelled: notch, INA226 integrating conversion, the production mux order.
//
//   clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src test/host-stub/bms-cutoff-test.cpp \
//       src/sim/vconv.cpp -o /tmp/bms-cutoff-test && /tmp/bms-cutoff-test
//
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <algorithm>
#include <random>
using std::isnan;
using std::min;
using std::max;
#define likely(x) (x)
#include "mock.h"
#include "math/statmath.h"
#include "sim/vconv.h"
#include "battery.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_fail; } } while (0)

static constexpr uint32_t kFsw = 39000;
static constexpr uint16_t kPmax = 2047;
static constexpr int kTickHz = 3000;      // plant step / ADC tick
static constexpr int kChannels = 4;       // round-robin: each channel sampled every 4th tick
static constexpr float kVbatMax = 29.2f;  // 8S LFP
static constexpr float kIoutMax = 30.f;

struct Chan {
    RunningMedian5<float> med3{};
    EWM<false, float> ewm;
    explicit Chan(uint32_t span) : ewm{span} {}
    void add(float x) { ewm.add(med3.next(x)); }
};

struct Rig {
    VirtualConverter v;
    Chan vin, vout, iout;
    std::mt19937 rng{1};
    float sigV, sigI;
    uint16_t ctrl = 0;
    long tick = 0;

    Rig(uint32_t spanV, uint32_t spanI, float vbat, float sigV_ = 0, float sigI_ = 0)
        : vin(spanV), vout(spanV), iout(spanI), sigV(sigV_), sigI(sigI_) {
        v.setPv(13.f, 76.f, 0.85f);
        v.setBat(vbat, 0.02f);
        v.setPassives(470e-6f, 470e-6f, 50e-6f);
        v.setVin(76.f);
        v.setVout(vbat);
    }

    void setCtrl(uint16_t c) {
        ctrl = c;
        v.setPwm({kPmax, c, (uint16_t) (kPmax - c - 20), kFsw});
    }

    float noise(float s) { return s > 0 ? std::normal_distribution<float>(0, s)(rng) : 0.f; }

    // one ADC tick, round-robin over the channels
    void step() {
        v.stepSeconds(1.f / kTickHz, kFsw);
        switch (tick++ % kChannels) {
            case 0: vin.add(v.getVin() + noise(sigV)); break;
            case 1: iout.add(v.getIoutAvg() + noise(sigI)); break;
            case 3: vout.add(v.getVout() + noise(sigV)); break;
            default: break;
        }
    }

    bool detect() const {
        return bmsCutoffSignature(iout.med3.get(), iout.ewm.avg.get(), std::max(1.f, kIoutMax * 0.05f),
                                  vout.med3.get(), vout.ewm.avg.get(), kVbatMax * 0.02f,
                                  vin.med3.get(), vin.ewm.avg.get());
    }

    // ramp duty up to the target current and settle
    void charge(float iTarget) {
        for (uint16_t c = 200; c < kPmax / 2; c += 5) {
            setCtrl(c);
            for (int i = 0; i < 200; ++i) step();
            if (v.getIoutAvg() >= iTarget) break;
        }
        for (int i = 0; i < kTickHz * 2; ++i) step();
    }

    bool ovTrip = false; // run() stopped on the 1.03x Vout OV sample (checked after, as in protect())

    // run n ticks until the first trip, return its tick (or -1); also track peak Vout until then
    int run(int n, float *vPeak = nullptr) {
        float pk = 0;
        int t = -1;
        for (int i = 0; i < n && t < 0; ++i) {
            step();
            pk = std::max(pk, v.getVout());
            if (detect()) t = i;
            else if ((tick - 1) % kChannels == 3 && v.getVout() > kVbatMax * 1.03f) t = i, ovTrip = true;
        }
        if (vPeak) *vPeak = pk;
        return t;
    }
};

static void test_open_detected(uint32_t spanV, uint32_t spanI, float vbat, float iCharge) {
    printf("open: span V/I %u/%u Vbat %.1f I %.0fA\n", (unsigned) spanV, (unsigned) spanI, vbat, iCharge);
    Rig r(spanV, spanI, vbat, 0.03f, 0.05f);
    r.charge(iCharge);
    float i0 = r.v.getIoutAvg(), v0 = r.v.getVout();
    CHECK(r.run(kTickHz * 5) < 0, "false trip in steady charge");
    r.v.setBat(0.f, 1e9f);
    float pk;
    int t = r.run(kTickHz / 10, &pk);
    float ovTh = kVbatMax * 1.03f;
    printf("  pre I=%.1fA V=%.2fV -> %s after %.2f ms, Vout peak %.2fV (OV %.2fV)\n", i0, v0,
           t < 0 ? "no trip" : r.ovTrip ? "OV trip" : "BMS-cutoff", t * 1e3f / kTickHz, pk, ovTh);
    CHECK(t >= 0, "no protection within 100 ms");
}

static void test_cloud_ignored() {
    printf("cloud: Isc 13 -> 3 A step\n");
    Rig r(20, 60, 27.f, 0.03f, 0.05f);
    r.charge(10.f);
    r.v.setPv(3.f, 76.f, 0.85f);
    CHECK(r.run(kTickHz * 2) < 0, "cloud detected as cut-off");
}

// Battery stays connected. Iout drops to dIout = dv / R_bat; once the step drives it to ~0 (here 2 V),
// it is not separable from a cut-off by this signature (the reverse-current trip covers that case).
static void test_bus_load_step(float dv, bool expectTrip) {
    printf("bus load drop: Vbat +%.2fV\n", dv);
    Rig r(20, 60, 27.f, 0.03f, 0.05f);
    r.charge(10.f);
    r.v.setBat(27.f + dv, 0.02f);
    bool trip = r.run(kTickHz * 2) >= 0;
    printf("  Iout %.2fA, %s\n", r.v.getIoutAvg(), trip ? "detected" : "ignored");
    CHECK(trip == expectTrip, "bus load step");
}

static void test_noise_no_false_trip() {
    printf("noise: 60 s steady charge, high noise\n");
    Rig r(20, 60, 27.f, 0.1f, 0.5f);
    r.charge(10.f);
    CHECK(r.run(kTickHz * 60) < 0, "false trip on noise");
}

int main() {
    test_open_detected(20, 60, 27.f, 10.f);
    test_open_detected(20, 60, 28.8f, 10.f);
    test_open_detected(20, 60, 27.f, 3.f);
    test_open_detected(20, 60, 27.f, 25.f);
    test_open_detected(3, 3, 27.f, 10.f);
    test_cloud_ignored();
    for (float dv : {0.2f, 0.4f, 0.8f, 1.2f}) test_bus_load_step(dv, false);
    test_bus_load_step(2.0f, true);
    test_noise_no_false_trip();
    printf(g_fail ? "FAILED (%d)\n" : "OK\n", g_fail);
    return g_fail != 0;
}
