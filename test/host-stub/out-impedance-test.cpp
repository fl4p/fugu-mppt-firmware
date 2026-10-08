// Host test for OutputImpedance (out_impedance.h) against the VirtualConverter plant driven by
// P&O-like duty steps. Sensor filters mirror Sensor::add_sample (med5).
//
//   clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src test/host-stub/out-impedance-test.cpp \
//       src/sim/vconv.cpp -o /tmp/out-impedance-test && /tmp/out-impedance-test
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
#include "out_impedance.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_fail; } } while (0)

static constexpr uint32_t kFsw = 39000;
static constexpr uint16_t kPmax = 2047;
static constexpr int kTickHz = 3000;
static constexpr int kChannels = 4;
struct Rig {
    VirtualConverter v;
    RunningMedian5<float> vout{}, iout{};
    OutputImpedance z;
    std::mt19937 rng{1};
    float sigV, sigI, lsbV;
    float vbat, rbat;
    uint16_t ctrl = 0;
    long tick = 0;
    bool nanOnce = false;

    Rig(float vbat_, float rbat_, float sigV_, float sigI_, float lsbV_ = 0)
        : sigV(sigV_), sigI(sigI_), lsbV(lsbV_), vbat(vbat_), rbat(rbat_) {
        v.setPv(13.f, 76.f, 0.85f);
        v.setBat(vbat, rbat);
        v.setPassives(470e-6f, 470e-6f, 50e-6f);
        v.setVin(76.f);
        v.setVout(vbat);
    }

    uint32_t now() const { return (uint32_t) (tick * 1000000LL / kTickHz); }

    float est() const { return z.get(now()); }

    float noise(float s) { return s > 0 ? std::normal_distribution<float>(0, s)(rng) : 0.f; }

    float quant(float x) const { return lsbV > 0 ? std::round(x / lsbV) * lsbV : x; }

    void setCtrl(uint16_t c) {
        ctrl = c;
        v.setPwm({kPmax, c, (uint16_t) (kPmax - c - 20), kFsw});
    }

    void step(bool feed = true) {
        v.stepSeconds(1.f / kTickHz, kFsw);
        switch (tick++ % kChannels) {
            case 1: iout.next(v.getIoutAvg() + noise(sigI)); break;
            case 3:
                vout.next(quant(v.getVout() + noise(sigV)));
                if (feed) {
                    z.add(nanOnce ? NAN : vout.get(), iout.get(), ctrl, now());
                    nanOnce = false;
                }
                break;
            default: break;
        }
    }

    void charge(float iTarget) {
        for (uint16_t c = 200; c < kPmax / 2; c += 5) {
            setCtrl(c);
            for (int i = 0; i < 200; ++i) step();
            if (v.getIoutAvg() >= iTarget) break;
        }
        for (int i = 0; i < kTickHz; ++i) step();
    }

    // P&O: +-dc counts every 100 ms, random direction (dc=0: fixed duty);
    // busStep: random Vbat jumps (loads, other sources) with probability busP per 100 ms
    void perturb(float seconds, int dc, float busStep = 0, float busP = 0.5f) {
        std::uniform_int_distribution<int> coin(0, 1);
        std::uniform_real_distribution<float> u(0, 1);
        const uint16_t c0 = ctrl;
        for (int t = 0; t < seconds * 10; ++t) {
            if (dc)
                setCtrl((uint16_t) std::clamp<int>(ctrl + (coin(rng) ? dc : -dc), c0 - 10 * dc, c0 + 10 * dc));
            if (busStep > 0 && u(rng) < busP)
                v.setBat(vbat + (coin(rng) ? busStep : -busStep), rbat);
            for (int i = 0; i < kTickHz / 10; ++i) step();
        }
    }
};

static void report(const char *name, const Rig &r, float rbat) {
    printf("%-34s R %5.2f mOhm -> est %6.2f mOhm (%u steps)\n", name, rbat * 1e3f, r.est() * 1e3f, r.z.steps());
}

static void test_estimate(const char *name, float rbat, float sigV, float sigI, float lsbV, float busStep,
                          float busP, float tol) {
    Rig r(27.f, rbat, sigV, sigI, lsbV);
    r.charge(10.f);
    r.perturb(120, 6, busStep, busP);
    report(name, r, rbat);
    float e = r.est();
    CHECK(std::isfinite(e) && std::fabs(e - rbat) <= tol * rbat, name);
}

// no own duty steps: bus disturbances and current noise must not produce a value
static void test_no_excitation(const char *name, float sigI, float busStep) {
    Rig r(27.f, 0.02f, 0.003f, sigI);
    r.charge(10.f);
    r.z = OutputImpedance{};
    r.perturb(60, 0, busStep, 0.5f);
    report(name, r, 0.02f);
    CHECK(std::isnan(r.est()), name);
}

// a degrading terminal: R jumps 5 -> 15 mOhm under P&O, the estimate must follow
static void test_tracks_rise() {
    Rig r(27.f, 0.005f, 0.002f, 0.01f, 0.00125f);
    r.charge(10.f);
    r.perturb(60, 6);
    float e0 = r.est();
    r.rbat = 0.015f;
    r.v.setBat(r.vbat, r.rbat);
    int t = -1;
    for (int s = 1; s <= 120 && t < 0; ++s) {
        r.perturb(1, 6);
        if (r.est() > 0.01f) t = s;
    }
    printf("%-34s est %.2f mOhm, > 10 mOhm after %d s\n", "rise 5 -> 15 mOhm", e0 * 1e3f, t);
    CHECK(t > 0 && t <= 60, "rise not tracked within 60 s");
}

static void test_nan_sample_recovers() {
    Rig r(27.f, 0.02f, 0.003f, 0.01f);
    r.charge(10.f);
    r.perturb(30, 6);
    r.nanOnce = true;
    r.perturb(30, 6);
    report("NaN Vout sample, then P&O", r, 0.02f);
    CHECK(std::isfinite(r.est()) && std::fabs(r.est() - 0.02f) < 0.003f, "NaN sample poisoned the fit");
}

// caller stops feeding for 10 min while Vbat and current change; no bogus step on resume
static void test_gap() {
    Rig r(27.f, 0.02f, 0.003f, 0.01f);
    r.charge(10.f);
    r.perturb(60, 6);
    r.vbat += 0.4f;
    r.v.setBat(r.vbat, r.rbat);
    r.setCtrl(r.ctrl - 60);
    for (long i = 0; i < kTickHz * 600L; ++i) r.step(false);
    r.perturb(0.5f, 6);
    report("10 min gap, then P&O", r, 0.02f);
    CHECK(std::isfinite(r.est()) && std::fabs(r.est() - 0.02f) < 0.002f, "gap injected a bogus step");
}

static void test_stale() {
    Rig r(27.f, 0.02f, 0.003f, 0.01f);
    r.charge(10.f);
    r.perturb(30, 6);
    bool fresh = std::isfinite(r.est());
    for (long i = 0; i < kTickHz * 70L; ++i) r.step();
    printf("%-34s fresh %d, after 70 s idle est %.2f mOhm\n", "stale after excitation stops", fresh, r.est() * 1e3f);
    CHECK(fresh && std::isnan(r.est()), "stale value still reported");
}

int main() {
    test_estimate("clean 20 mOhm", 0.02f, 0, 0, 0, 0, 0, 0.05f);
    test_estimate("clean 5 mOhm", 0.005f, 0, 0, 0, 0, 0, 0.05f);
    test_estimate("INA226-like 5 mOhm", 0.005f, 0.002f, 0.01f, 0.00125f, 0, 0, 0.15f);
    test_estimate("noisy 20 mOhm", 0.02f, 0.01f, 0.05f, 0, 0, 0, 0.15f);
    test_estimate("Iout noise 0.6A, 20 mOhm", 0.02f, 0.003f, 0.6f, 0, 0, 0, 0.25f);
    test_estimate("bus steps 0.2V p=.05, 20 mOhm", 0.02f, 0.003f, 0.01f, 0, 0.2f, 0.05f, 0.25f);
    test_estimate("bus steps 0.5V p=.02, 20 mOhm", 0.02f, 0.003f, 0.01f, 0, 0.5f, 0.02f, 0.25f);
    test_no_excitation("no P&O, quiet", 0.01f, 0);
    test_no_excitation("no P&O, Iout noise 0.6A", 0.6f, 0);
    test_no_excitation("no P&O, bus steps 0.5V", 0.01f, 0.5f);
    test_tracks_rise();
    test_nan_sample_recovers();
    test_gap();
    test_stale();
    printf(g_fail ? "FAILED (%d)\n" : "OK\n", g_fail);
    return g_fail != 0;
}
