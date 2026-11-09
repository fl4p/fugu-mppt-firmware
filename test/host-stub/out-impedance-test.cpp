// Host test for out_impedance.h: OutputImpedance on synthetic sweeps V = Vbat + R·I + error,
// WireResistance on synthetic BMS frames.
//
//   clang++ -std=gnu++17 -O2 -fexceptions -I test/host-stub -I src test/host-stub/out-impedance-test.cpp \
//       -o /tmp/out-impedance-test && /tmp/out-impedance-test
//
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <random>
#include <algorithm>
#include "out_impedance.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_fail; } } while (0)

struct Sweep {
    float vbat = 27.f, r = 0.02f;
    float i0 = 0.f, i1 = 15.f;      // ramp start/end current
    float seconds = 10.f, hz = 1000.f;
    float sigV = 0.005f, sigI = 0.02f;
    float bend = 0;                 // duty-correlated Vout error amplitude [V], nonlinear in the ramp
    float quad = 0;                 // Vout error growing with the square of the ramp [V], biases R
    int nanEvery = 0;
    uint32_t seed = 1;
};

static uint32_t g_us = 0;

static OutputImpedance::Fit run(OutputImpedance &z, const Sweep &s, uint32_t nowS) {
    std::mt19937 rng{s.seed};
    std::normal_distribution<float> nv(0, s.sigV > 0 ? s.sigV : 1e-9f), ni(0, s.sigI > 0 ? s.sigI : 1e-9f);
    const int n = (int) (s.seconds * s.hz);
    z.reset();
    for (int k = 0; k < n; ++k) {
        const float x = (float) k / (float) (n - 1);
        const float i = s.i0 + (s.i1 - s.i0) * x;
        float v = s.vbat + s.r * i + s.bend * std::sin(3.14159f * x) + s.quad * x * x + nv(rng);
        if (s.nanEvery && k % s.nanEvery == 0) v = NAN;
        z.add(v, i + ni(rng), g_us);
        g_us += (uint32_t) (1e6f / s.hz);
    }
    return z.finish(nowS);
}

static void report(const char *name, const OutputImpedance::Fit &f) {
    printf("%-36s %-8s R %6.2f mOhm  SE %5.2f  res %5.1f mV  span %5.2f A  n %u\n", name, f.ok ? "publish" : "reject",
           f.r * 1e3f, f.se * 1e3f, f.res * 1e3f, f.span, f.n);
}

static void test_recovers(const char *name, Sweep s, float tol) {
    OutputImpedance z;
    auto f = run(z, s, 1000);
    report(name, f);
    CHECK(f.ok && std::fabs(z.get(1000) - s.r) <= tol, name);
    CHECK(std::fabs(f.r - s.r) <= 4 * f.se + 1e-4f, "R outside 4 SE");
}

static void test_rejects(const char *name, Sweep s) {
    OutputImpedance z;
    auto f = run(z, s, 1000);
    report(name, f);
    CHECK(!f.ok && std::isnan(z.get(1000)), name);
}

// a rejected sweep must keep the previous good value
static void test_keeps_previous() {
    OutputImpedance z;
    Sweep good;
    run(z, good, 1000);
    Sweep bad;
    bad.bend = 0.2f;
    bad.r = 0.05f;
    auto f = run(z, bad, 2000);
    report("bad after good", f);
    CHECK(!f.ok && std::fabs(z.get(2000) - good.r) < 1e-3f, "rejected sweep replaced the previous value");
    Sweep small;
    small.i1 = 0.1f;
    run(z, small, 2100);
    CHECK(std::fabs(z.get(2100) - good.r) < 1e-3f, "night sweep replaced the previous value");
}

static void test_stale() {
    OutputImpedance z;
    CHECK(std::isnan(z.get(0)) && std::isnan(z.get(100000)), "value before any sweep");
    run(z, Sweep{}, 1000);
    const bool fresh = std::isfinite(z.get(1000 + OutputImpedance::STALE_S));
    const bool stale = std::isnan(z.get(1001 + OutputImpedance::STALE_S));
    printf("%-36s fresh %d, stale after %u s %d\n", "staleness", fresh, OutputImpedance::STALE_S + 1, stale);
    CHECK(fresh && stale, "staleness");
}

// a sweep aborted by a trip: the caller resets instead of finishing; the next sweep starts clean
static void test_abort() {
    OutputImpedance z;
    Sweep s;
    s.r = 0.03f;
    s.i1 = 10.f;
    for (int k = 0; k < 5000; ++k) z.add(30.f, (float) k * 1e-3f, g_us += 1000);
    z.reset();
    auto f = run(z, s, 1000);
    report("abort, then clean sweep", f);
    CHECK(f.ok && std::fabs(f.r - s.r) < 1e-3f && f.res < 0.002f, "aborted ramp leaked into the next fit");
    z.reset();
    auto e = z.finish(1001);
    CHECK(!e.ok && e.n == 0 && std::fabs(z.get(1001) - s.r) < 1e-3f, "empty finish");
}

// residual and span gates must not flip back to publish as the data gets worse
static void test_monotonic() {
    bool rejected = false, flip = false;
    for (float b = 0; b <= 2.f; b += 0.01f) {
        OutputImpedance z;
        Sweep s;
        s.bend = b;
        bool ok = run(z, s, 1000).ok;
        if (!ok) rejected = true;
        else if (rejected) flip = true;
    }
    for (float sig: {0.f, 0.01f, 0.05f, 0.2f, 1.f, 5.f}) {
        OutputImpedance z;
        Sweep s;
        s.sigV = sig;
        s.hz = 10;
        bool ok = run(z, s, 1000).ok;
        if (!ok) rejected = true;
        else if (rejected && sig > 0.05f) flip = true;
    }
    printf("%-36s %s\n", "monotonic in error", flip ? "FLIP" : "ok");
    CHECK(!flip, "worse data published again");
}

// WireResistance on synthetic BMS frames (4 s): currents correlated like a shared PV bus,
// pack voltage quantised to 10 mV. offDrift: a non-constant output-voltage error (flat-like).
static void wireCase(const char *name, float sigV, float offDrift, int frames, bool expectValid) {
    WireResistance w;
    std::mt19937 rng{3};
    std::normal_distribution<float> n(0, 1);
    const float r1 = 0.012f, r2 = 0.003f, a = 0.025f;
    float sun = 5, off = 0;
    for (int k = 0; k < frames; ++k) {
        sun = std::clamp(sun + n(rng) * 0.4f, 0.3f, 14.f);
        float i1 = sun, i2 = 1.8f * sun - 2.f + n(rng) * 1.5f; // sibling charger + loads
        off = std::clamp(off + n(rng) * offDrift, -2.f, 0.f);
        float vpack = std::round((26.6f + 0.02f * i2) / 0.01f) * 0.01f;
        float vout = 26.6f + 0.02f * i2 + a + r1 * i1 + r2 * i2 + off + n(rng) * sigV;
        w.add(vout - vpack, i1, i2, (uint32_t) (k * 4));
    }
    const uint32_t t = (uint32_t) (frames * 4);
    printf("%-34s R1 %6.2f R2 %6.2f mOhm  sigma %5.1f mV\n", name, w.own(t) * 1e3f, w.shared(t) * 1e3f,
           w.residual() * 1e3f);
    if (expectValid)
        CHECK(std::fabs(w.own(t) - r1) < 0.0015f && std::fabs(w.shared(t) - r2) < 0.0015f, name);
    else
        CHECK(std::isnan(w.own(t)) && std::isnan(w.shared(t)), name);
}

static void test_wire() {
    wireCase("wire: fry-like", 0.012f, 0, 3000, true);
    wireCase("wire: too few frames", 0.012f, 0, 150, false);
    wireCase("wire: flat-like offset drift", 0.012f, 0.03f, 3000, false);
    wireCase("wire: flat-like noise 150 mV", 0.15f, 0, 3000, false);
    WireResistance w;
    for (int k = 0; k < 400; ++k) w.add(0.025f, 5.f, 7.f, (uint32_t) (k * 4)); // currents never vary
    CHECK(std::isnan(w.own(1600)), "wire: constant currents gave a value");
    printf("%-34s R1 %6.2f mOhm\n", "wire: constant currents", w.own(1600) * 1e3f);
    WireResistance v;
    std::mt19937 rng{4};
    std::uniform_real_distribution<float> u(0, 14);
    for (int k = 0; k < 2000; ++k) {
        float i1 = u(rng), i2 = 1.5f * i1 + u(rng) * 0.3f;
        v.add(0.025f + 0.012f * i1 + 0.003f * i2, i1, i2, (uint32_t) (k * 4));
    }
    bool fresh = std::isfinite(v.own(8000)), stale = std::isnan(v.own(8000 + WireResistance::STALE_S + 1));
    printf("%-34s fresh %d, stale after STALE_S %d\n", "wire: staleness", fresh, stale);
    CHECK(fresh && stale, "wire: staleness");
}

int main() {
    test_recovers("20 mOhm, 0-15 A, 1 kHz", Sweep{}, 0.0005f);
    Sweep s;
    s.r = 0.005f;
    s.sigV = 0.002f;
    test_recovers("5 mOhm, INA226-like noise", s, 0.0005f);
    s = Sweep{};
    s.hz = 2.5f;
    s.seconds = 10.f;
    s.r = 0.0235f;
    s.sigV = 0.025f;
    test_recovers("23.5 mOhm, 2.5 Hz, res 25 mV", s, 0.003f);
    s = Sweep{};
    s.i1 = 3.5f;
    test_recovers("span 3.5 A", s, 0.001f);
    s = Sweep{};
    s.nanEvery = 7;
    test_recovers("NaN every 7th sample", s, 0.0005f);

    s = Sweep{};
    s.i0 = -0.1f;
    s.i1 = 0.1f;
    test_rejects("night sweep, span 0.2 A", s);
    s.i0 = 0;
    s.i1 = 2.5f;
    test_rejects("span 2.5 A", s);
    s = Sweep{};
    s.bend = 0.2f;
    test_rejects("duty-correlated error 0.2 V", s);
    s.bend = -0.2f;
    test_rejects("duty-correlated error -0.2 V", s);
    s = Sweep{};
    s.hz = 2.5f;
    s.seconds = 6.f;
    test_rejects("n < 20", s);
    s = Sweep{};
    s.hz = 2.5f;
    s.sigV = 0.06f;
    test_rejects("residual 60 mV", s);
    s = Sweep{};
    s.r = -0.01f;
    test_rejects("negative slope", s);
    s = Sweep{};
    s.nanEvery = 1;
    test_rejects("all NaN", s);
    s = Sweep{};
    s.i1 = 0;
    test_rejects("no current change", s);

    // known-bad calibration: a 20 mOhm path read through a Vout channel whose gain error grows with
    // duty (+0.15 V at mid-ramp, 0 at the ends) must not be published as a resistance
    s = Sweep{};
    s.bend = 0.15f;
    test_rejects("known-bad Vout calibration", s);
    // same with an error that grows with duty (+0.6 V at the end), which would bias R by ~+40 mOhm
    s = Sweep{};
    s.quad = 0.6f;
    test_rejects("known-bad, error 0.6 V * x^2", s);

    test_keeps_previous();
    test_stale();
    test_abort();
    test_monotonic();
    test_wire();
    printf(g_fail ? "FAILED (%d)\n" : "OK\n", g_fail);
    return g_fail != 0;
}
