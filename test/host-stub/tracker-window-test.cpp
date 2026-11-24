// Host test: a limiter that takes over again within every tracker period must not keep the averaging window
// open (it grew until MeanAccumulator overflowed and aborted), and MeanAccumulator saturates keeping its mean.
//
// Build & run (repo root):
//   clang++ -std=gnu++20 -O2 -fexceptions -DMOCK=1 -I test/host-stub -I src \
//       test/host-stub/tracker-window-test.cpp -o /tmp/tracker-window-test && /tmp/tracker-window-test

#include <cassert>
#include <cstdio>
#include <cmath>
#include <algorithm>

#include "esp_log.h"
#include "util.h"
#include "tracker.h"

time_us loopWallClockUs_ = 0;

static Tracker t;

int main() {
    // 2 ms ticks: 12 tracker samples (24 ms, under the 33 ms fast-mode period), then a limiter hand-back
    loopWallClockUs_ = 1'000'000;
    t.resetDirection(true, 40.f);
    unsigned maxNum = 0;
    for (int i = 0; i < 200000; ++i) {
        loopWallClockUs_ += 2000;
        if (i % 13 == 12) t.resetDirection(true, 40.f);
        else t.update(10.f, 1000, 40.f, [] { return 150.f; });
        if (t._powerBuf.num > maxNum) maxNum = t._powerBuf.num;
    }
    printf("limit cycle: max window %u samples\n", maxNum);
    assert(maxNum <= 13);

    // the period still ends without a hand-back
    loopWallClockUs_ += 1'000'000;
    t.resetDirection(true, 40.f);
    for (int i = 0; i < 20; ++i) loopWallClockUs_ += 2000, t.update(10.f, 1000, 40.f, [] { return 150.f; });
    assert(t._powerBuf.num < 20);

    MeanAccumulator a;
    for (unsigned i = 0; i < 200000; ++i) a.add(i & 1 ? 3.f : 5.f);
    printf("MeanAccumulator after 200000 adds: num %u mean %.4f\n", (unsigned) a.num, a.getMean());
    assert(a.num > 0 && std::fabs(a.getMean() - 4.f) < 1e-3f);

    printf("tracker-window-test: all ok\n");
    return 0;
}
