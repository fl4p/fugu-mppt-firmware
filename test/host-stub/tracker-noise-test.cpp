// Host test for the tracker's noise-derived reversal threshold (src/tracker.h): after 200 periods of constant
// power (estimate 0, threshold at the floor) the threshold must rise again once noise appears, or fall back to
// the fixed rule when the noise starves the estimator (alternating dips reverse every other period).
//   test/host-stub/tracker-noise-test.sh
//   TRK_OLD=<file> test/host-stub/tracker-noise-test.sh   (another tracker.h, results printed)

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <esp_log.h>
#include "tracker.h"
#include "sim_alloc.h"

time_us loopWallClockUs_ = 0;
SimRtAlloc g_simRtAlloc{};

static float run(int pattern, uint8_t &n, float &before) {
    loopWallClockUs_ = 1'000'000;
    auto tp = std::make_unique<Tracker>();
    auto &t = *tp;
    t.revFloor = 0.05f;
    t.setSlowMode(false);
    auto settle = [] { return 150.f; };
    uint32_t lcg = 1;
    float p = 100;
    for (int i = 0; i < 400; ++i) {
        if (i == 200) before = t.minPowerStep;
        if (i >= 200) {
            lcg = lcg * 1664525u + 1013904223u;
            if (pattern == 0) p = 100.f + (i & 1 ? 1.f : -1.f);      // alternating +-1 W
            else if (pattern == 1) p = 100.f + (lcg >> 31 ? 1.f : -1.f); // random sign +-1 W
            else p += i & 1 ? 2.f : 0.f;                               // staircase: dP >= 0, d = +-2 W
        }
        loopWallClockUs_ += 40'000;
        t.update(p, 1000, 50.f, settle);
    }
    n = t._noiseN[0];
    return t.minPowerStep;
}

int main() {
    static const char *names[] = {"alternating +-1 W", "random +-1 W", "staircase d=+-2 W"};
    int fail = 0;
    for (int k = 0; k < 3; ++k) {
        uint8_t n;
        float before = NAN, after = run(k, n, before);
        printf("%-18s threshold before %.4f W, after %.4f W (%u noise samples)\n", names[k], before, after, n);
        if (!(after >= 0.05f * 1.5f)) ++fail;
    }
    printf(fail ? "FAIL: threshold stuck at the floor\n" : "PASS\n");
    return fail;
}
