// Host test of HsShortGuard, HsClampArm (src/selftest/hs_short.h) and BfDaily (src/selftest/bflow_test.h):
//   clang++ -std=c++20 -I src test/host-stub/hs-short-test.cpp -o /tmp/hs-short-test && /tmp/hs-short-test
#include "selftest/hs_short.h"
#include "selftest/bflow_test.h"

#include <cstdio>
#include <initializer_list>

static int fails = 0;
static void check(bool c, const char *what) {
    printf("%s: %s\n", c ? "ok  " : "FAIL", what);
    fails += !c;
}

static constexpr float iMin = .35f, iBig = 3.5f; // 35 A sensor: 5 % zero bound 1.75 A

// feeds a constant sample every 2 ms from t0 for ms; returns the latch time or 0
static uint32_t feed(HsShortGuard &g, uint32_t &t, uint32_t ms, bool off, float vin, float vout, float i, bool bfOff = true,
                     float vOv = 30.f) {
    uint32_t at = 0;
    for (uint32_t end = t + ms; t < end; t += 2)
        if (g.update(t, off, vin, vout, i, iMin, iBig, vOv, bfOff) && !at) at = t;
    return at;
}

int main() {
    {
        HsShortGuard g;
        uint32_t t = 1000;
        check(g.blocks(), "boot: blocks before any verdict");
        feed(g, t, 400, true, 30, 27, 0);
        check(g.blocks(), "boot: still blocks inside the 0.5 s off time");
        feed(g, t, 200, true, 30, 27, 0);
        check(!g.blocks(), "clean after 0.5 s off");
    }
    {
        // known bad: the short, Vin pulled to Vout + diode, panel current
        HsShortGuard g;
        uint32_t t = 1000, t0 = t;
        const uint32_t at = feed(g, t, 6000, true, 27.8f, 27.f, 5.f);
        check(at && at - t0 >= 3500 && at - t0 < 3600, "short latches 0.5 s off + 3 s hold");
        feed(g, t, 60000, true, 10, 27, 0);
        check(g.latched && g.blocks(), "no auto-clear when the readings go clean");
        feed(g, t, 1000, false, 60, 27, 10);
        check(g.latched, "no auto-clear while running");
    }
    {
        // a 1 A short inside the 5 % zero bound: the guard is fed the raw current, so a zero taken during the short
        // (calibration) does not move it; only the night zero below does
        HsShortGuard g;
        uint32_t t = 1000;
        check(feed(g, t, 5000, true, 27.7f, 27.f, 1.f) != 0, "1 A short latches");
    }
    {
        // unevaluable input never counts as clean
        HsShortGuard g;
        uint32_t t = 1000;
        feed(g, t, 1000, true, NAN, 27, 0);
        check(g.blocks() && !g.clean, "NaN Vin: blocks");
        feed(g, t, 1000, true, 30, 27, NAN);
        check(g.blocks(), "NaN I: blocks");
        feed(g, t, 10, true, 30, 27, 0);
        check(!g.blocks(), "clean once evaluable");
        // a suspicion survives an unevaluable gap and keeps its age
        feed(g, t, 2000, true, 27.5f, 27, 2);
        feed(g, t, 500, true, NAN, 27, 2);
        check(feed(g, t, 600, true, 27.5f, 27, 2) != 0, "suspicion keeps its age over a NaN gap");
    }
    {
        // monotonicity in the current: from just under iMin to far beyond iBig, with Vin at Vout and far above it
        bool mono = true;
        for (float dv: {-1.f, 0.f, 1.f, 5.f, 30.f}) {
            bool was = false;
            for (float i = 0; i < 400; i += .05f) {
                HsShortGuard g;
                uint32_t t = 1000;
                const bool l = feed(g, t, 4000, true, 27 + dv, 27, i) != 0;
                if (was && !l) mono = false, printf("     dv %.1f: latched below %.2f A, not at it\n", dv, i);
                was = l;
            }
        }
        check(mono, "latch verdict monotone in current (up to 400 A)");
        HsShortGuard g;
        uint32_t t = 1000;
        check(feed(g, t, 4000, true, 60, 27, 3.6f) != 0, "beyond iBig latches whatever the voltages");
        g = {};
        t = 1000;
        check(feed(g, t, 4000, true, 60, 27, 3.4f) == 0, "below iBig with Vin well above Vout: no latch");
    }
    {
        // dawn: offset 0.5 A, noise; Voc crosses Vout. With a night zero: no latch
        HsShortGuard g;
        uint32_t t = 1000;
        feed(g, t, 20000, true, 5, 27, .5f);
        check(std::fabs(g.zero - .5f) < .01f, "night zero learnt");
        uint32_t at = 0;
        for (float v = 20; v < 40 && !at; v += .01f) at = feed(g, t, 20, true, v, 27, .5f + .1f * std::sin(v * 37));
        check(!at, "dawn with a night zero: no latch");
        // without the night: the offset alone looks like a short (known limit)
        HsShortGuard h;
        t = 1000;
        check(feed(h, t, 5000, true, 27.5f, 27, .5f) != 0, "offset 0.5 A without a night zero latches (known limit)");
        // no zero is learnt while the BF is commanded on, or from a large current
        HsShortGuard k;
        t = 1000;
        feed(k, t, 5000, true, 5, 27, .5f, false);
        check(std::isnan(k.zero), "no zero with the BF on");
        feed(k, t, 5000, true, 5, 27, 3.f);
        check(std::isnan(k.zero), "no zero from 3 A");
    }
    {
        // a running converter resets the check; a start right after a stop (< 0.5 s) is not held
        HsShortGuard g;
        uint32_t t = 1000;
        feed(g, t, 1000, true, 30, 27, 0);
        feed(g, t, 1000, false, 60, 27, 10);
        feed(g, t, 100, true, 60, 27, 10);
        check(!g.blocks(), "re-start within 0.5 s of a stop is free");
        feed(g, t, 600, true, 60, 27, 0);
        check(!g.blocks(), "clean after 0.5 s off");
        feed(g, t, 10, true, 27.5f, 27, 2);
        check(g.blocks(), "a suspect sample blocks");
    }
    {
        // voltage only: the battery is gone and the shorted HS holds Vout at Vin above the OV limit, ~0 A
        HsShortGuard g;
        uint32_t t = 1000;
        const uint32_t t0 = t, at = feed(g, t, 62000, true, 45.5f, 45.f, 0);
        check(at && g.byVoltage && at - t0 >= 60500, "Vout above OV following Vin at 0 A latches by voltage after 60 s");
        HsShortGuard p;
        t = 1000;
        feed(p, t, 50000, true, 45.5f, 45.f, 0);
        check(p.blocks() && !p.latched, "voltage suspicion holds the start before it latches");
        check(!feed(p, t, 20000, true, 52.f, 45.f, 0) && !p.blocks(), "Vin leaving the window within 60 s: no latch");
        HsShortGuard h;
        t = 1000;
        check(!feed(h, t, 70000, true, 60.f, 45.f, 0), "Vout above OV, Vin far above: no latch");
        HsShortGuard k;
        t = 1000;
        check(!feed(k, t, 70000, true, 45.5f, 45.f, 0, true, NAN), "no OV limit (PSU mode): no voltage verdict");
        // a 2 % gain error at 60 V (1.2 V) stays inside the window; at 27 V the 1.5 V floor holds
        HsShortGuard m;
        t = 1000;
        check(feed(m, t, 5000, true, 60.f - 2.3f, 60.f, 2.f, true, 100.f) != 0, "-2.3 V at 60 V is inside the 4 % window");
        HsShortGuard n;
        t = 1000;
        check(!feed(n, t, 5000, true, 60.f + 3.f, 60.f, 2.f, true, 100.f), "+3 V at 60 V with 2 A: outside");
    }
    {
        // clamp arming: Vin below 9 V for 60 s without a break
        auto low = [](HsClampArm &a, uint32_t &t, uint32_t ms, float vin) {
            bool armed = false;
            for (uint32_t end = t + ms; t < end; t += 2) armed = a.update(t, vin);
            return armed;
        };
        HsClampArm a;
        uint32_t t = 1000;
        check(!low(a, t, 59000, 8.9f), "8.9 V for 59 s: not armed");
        check(low(a, t, 1002, 8.9f), "8.9 V for 60 s: armed");
        check(a.heldMs(t) >= 60000, "held time reported");
        check(!a.update(t, 9.f) && !a.heldMs(t), "9.0 V resets");
        check(!low(a, t, 59000, 5.f) && !a.update(t, NAN), "NaN resets");
        check(!low(a, t, 59000, 5.f), "59 s after the NaN: not armed");
        check(low(a, t, 1002, 5.f), "60 s after the NaN: armed");
        HsClampArm b;
        t = 1000;
        low(b, t, 40000, 6.8f);
        b.update(t, 9.5f);
        check(!low(b, t, 59000, 6.8f), "a value above 9 V restarts the 60 s");
        check(low(b, t, 1002, 6.8f), "then 60 s arms");
        // ramp: 200 ns at 6.25 ns/tick = 32 ticks, 4103 per period
        check(HsClampArm::rampTicks(0, 32, 4103) == 32, "ramp starts at t0");
        check(HsClampArm::rampTicks(10000, 32, 4103) == 32 + (4103 - 32) / 2, "half way at 10 ms");
        check(HsClampArm::rampTicks(19999, 32, 4103) < 4103 && HsClampArm::rampTicks(20000, 32, 4103) == 4103,
              "full at 20 ms");
        uint32_t prev = 0;
        bool mono = true;
        for (uint32_t us = 0; us <= 25000; us += 7) mono &= HsClampArm::rampTicks(us, 32, 4103) >= prev, prev = HsClampArm::rampTicks(us, 32, 4103);
        check(mono, "ramp monotone");
    }
    {
        // daily run scheduling: hold, abort retry, no spin, refusal back-off
        BfDaily d;
        uint32_t t = 0, starts = 0;
        // a run that starts keeps the caller busy (no polls) for the rest of the call
        auto tick = [&](uint32_t ms, bool due, bool el) {
            bool busy = false;
            for (uint32_t end = t + ms; t < end; t += 1000)
                if (!busy && d.poll(t, due, el)) busy = true, ++starts;
        };
        tick(59000, true, true);
        check(!starts, "needs 60 s eligible");
        tick(2000, true, true);
        check(starts == 1, "starts after 60 s");
        d.result(t, true); // aborted
        starts = 0;
        tick(29 * 60000, true, true);
        check(!starts, "abort: no retry within 29 min");
        tick(3 * 60000, true, true);
        check(starts == 1, "abort: retry after 30 min + hold");
        for (int k = 0; k < 3; ++k) d.result(t, true), tick(32 * 60000, true, true);
        starts = 0;
        tick(19 * 3600000u, true, true);
        check(!starts, "4 aborts: waits 20 h");
        tick(2 * 3600000u, true, true);
        check(starts == 1, "then tries again");
        d.result(t, false);
        starts = 0;
        tick(3600000, false, true);
        check(!starts, "not due: no run");
        d.refused(t);
        tick(4 * 60000, true, true);
        check(!starts, "refused: waits 5 min");
        tick(3 * 60000, true, true);
        check(starts == 1, "then starts");
        BfDaily e;
        t = 0;
        starts = 0;
        for (int k = 0; k < 600; ++k) d.poll(t, true, k % 50 != 0), e.poll(t, true, k % 50 != 0), starts += e.poll(t += 1000, true, k % 50 != 0);
        check(!starts, "eligibility gaps restart the 60 s hold");
    }
    printf("== hs-short-test: %s (%d failures)\n", fails ? "FAILED" : "OK", fails);
    return fails != 0;
}
