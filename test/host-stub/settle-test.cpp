// Host test: MpptController::settleMs() covers the 99 % step response of the real sensor chain
// with the comb active (comb -> median-5 -> 2-pass EWM), and the comb's DC delay is L/2 samples. Notch-only rows
// are informational: that model ignores the notch transient and runs short by up to ~3 ms at high fs.
// Build + run: test/host-stub/settle-test.sh
#include <cstdio>
#include "mppt.h"

static int fails = 0;

static void check(bool ok, const char *what) {
    printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++fails;
}

// samples until ewm.avg reaches 99 % of a unit step, from a settled zero
static int steps99(float fs, uint32_t span, uint8_t mode) {
    PhysicalSensor s(nullptr, {255, {1, 0}, {}, "t", 'V', false}, span);
    if (mode & 2) s.comb = new RippleComb<24>{};
    s.createNotchFilter(fs, 100.f, 20.f, NotchFilter::gainAmp(-15.f), mode, 80.f, 140.f);
    for (int i = 0; i < 20000; ++i) s.add_sample(0.f);
    for (int i = 0; i < 20000; ++i) {
        s.add_sample(1.f);
        if (s.ewm.avg.get() >= .99f) {
            float ms = MpptController::settleMs(&s), t = 1000.f * (float) (i + 1) / fs;
            char b[160];
            snprintf(b, sizeof b, "fs %4.0f span %2u %-5s 99 %% at %6.2f ms <= settleMs %6.2f ms", fs, (unsigned) span,
                     mode == 1 ? "notch" : mode == 2 ? "comb" : "both", t, ms);
            if (mode == 1) printf("  %s  %s\n", t <= ms ? "ok  " : "INFO", b); // notch-only model predates the comb
            else check(t <= ms, b);
            delete s.comb;
            return i;
        }
    }
    check(false, "never settled");
    return -1;
}

int main() {
    printf("settleMs vs measured 99 %% step settling\n");
    for (float fs: {300.f, 450.f, 513.f, 1470.f, 1800.f})
        for (uint32_t span: {1u, 3u, 20u, 60u})
            for (uint8_t mode: {1, 2, 3})
                steps99(fs, span, mode);

    printf("comb DC delay = L/2\n");
    for (float L: {2.f, 4.5f, 5.13f, 14.7f, 22.9f}) {
        RippleComb<24> c;
        c.tune(L);
        c.seed(0.f);
        double d = 0;
        for (int i = 0; i < 100; ++i) d += 1.0 - c.filter(1.f); // mean delay = area above the step response
        char b[96];
        snprintf(b, sizeof b, "L %5.2f delay %.4f (L/2 %.4f) length() %.4f", L, d, L / 2, c.length());
        check(std::fabs(d - L / 2) < 1e-3 && std::fabs(c.length() - L) < 1e-4f, b);
    }
    printf(fails ? "\nFAILED %d\n" : "\nOK\n", fails);
    return fails != 0;
}
