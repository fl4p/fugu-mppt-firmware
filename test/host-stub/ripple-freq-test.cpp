// Host-side test for the adaptive inverter-ripple frequency detector (src/math/ripple_freq.h), the
// one-period comb (src/math/ripple_comb.h) and the notch depth (src/math/notch.h).
// Self-contained, no ESP-IDF, no hardware. Build + run:
//
//   c++ -std=gnu++17 -O2 -I src -I test -I test/host-stub/esp \
//       -o /tmp/ripple-freq-test test/host-stub/ripple-freq-test.cpp && /tmp/ripple-freq-test
//
// Covers synthetic tones (50/60 Hz inverter -> 100/120 Hz, sub-bin, weak ripple, DC-only) and
// the REAL Vout capture from converter "fry" with a ~2 kW inverter on its DC bus, which the
// host FFT pins at ~126 Hz.

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>

#include "math/ripple_freq.h"
#include "math/ripple_comb.h"
#include "math/notch.h"
#include "data/fry_vout_inverter_646.h"

static int failures = 0;
#define CHECK(cond, fmt, ...) do { \
    if (cond) { printf("  PASS  " fmt "\n", ##__VA_ARGS__); } \
    else { printf("  FAIL  " fmt "\n", ##__VA_ARGS__); ++failures; } } while (0)

static void detectTone(float fs, float ftone, float amp, float dc, float &hz, float &snr) {
    RippleFreqDetector<61> det;
    uint16_t blockN = (uint16_t) (fs * 0.75f);
    det.configure(fs, 80.f, 140.f, blockN);
    hz = NAN; snr = 0;
    unsigned seed = 12345;
    for (int n = 0; n < blockN * 4; ++n) {
        seed = seed * 1103515245u + 12345u;
        float noise = ((int) (seed >> 16) % 1000 - 500) * 1e-4f;
        float x = dc + noise + (amp > 0 ? amp * sinf(2 * (float) M_PI * ftone * n / fs) : 0.f);
        det.push(x);
        float h, s;
        if (det.poll(h, s)) { hz = h; snr = s; }
    }
}

using Comb = RippleComb<24>;

// steady-state gain of a 1 V sine at f on 26 V through the filter: peak |y - dc| after settling
template<class F>
static float gainDb(F &&filt, float fs, float f) {
    float peak = 0;
    for (int n = 0; n < 6000; ++n) {
        float y = filt(26.f + sinf(2 * (float) M_PI * f * (float) n / fs)) - 26.f;
        if (n > 2000) peak = fmaxf(peak, fabsf(y));
    }
    return 20 * log10f(peak + 1e-9f);
}

static float combDb(float fs, float f0, float f) {
    Comb c;
    c.tune(fs / f0);
    c.seed(26.f);
    return gainDb([&](float x) { return c.filter(x); }, fs, f);
}

static float notchDb(float fs, float f0, float f, float depth = -30.f) {
    NotchFilter nf;
    nf.begin(f0 / fs, depth, 20.f);
    nf.seed(26.f);
    return gainDb([&](float x) { float y; nf.filter(&x, &y, 1); return y; }, fs, f);
}

// Goertzel amplitude of tone f in x[0..n)
static float toneAmp(const float *x, int n, float fs, float f) {
    float c = 2 * cosf(2 * (float) M_PI * f / fs), q1 = 0, q2 = 0;
    double m = 0;
    for (int i = 0; i < n; ++i) m += x[i];
    m /= n;
    for (int i = 0; i < n; ++i) {
        float q0 = c * q1 - q2 + (x[i] - (float) m);
        q2 = q1, q1 = q0;
    }
    return 2 * sqrtf(fmaxf(q1 * q1 + q2 * q2 - c * q1 * q2, 0)) / (float) n;
}

static void testComb() {
    printf("\nripple comb (one-period moving average)\n");
    // stopband per sample rate: INA226 boards ~450/513 sps, internal ADC ~1470 sps
    struct { float fs, f0, at0, at1, at2, at5, h2, h3; } rs[] = {
        {450.f, 100.f, -44, -38, -33, -26, -27, -15},
        {513.f, 100.f, -50, -38, -33, -26, -36, -27},
        {450.f, 120.f, -40, -35, -33, -26, -23, -10},
        {1470.f, 100.f, -70, -38, -33, -25, -60, -50},
    };
    for (auto &r: rs) {
        float g0 = combDb(r.fs, r.f0, r.f0), gm1 = combDb(r.fs, r.f0, r.f0 - 1), gp1 = combDb(r.fs, r.f0, r.f0 + 1);
        float gm2 = combDb(r.fs, r.f0, r.f0 - 2), gp2 = combDb(r.fs, r.f0, r.f0 + 2);
        float gm5 = combDb(r.fs, r.f0, r.f0 - 5), gp5 = combDb(r.fs, r.f0, r.f0 + 5);
        float h2 = combDb(r.fs, r.f0, 2 * r.f0), h3 = combDb(r.fs, r.f0, 3 * r.f0);
        CHECK(g0 < r.at0 && fmaxf(gm1, gp1) < r.at1 && fmaxf(gm2, gp2) < r.at2 && fmaxf(gm5, gp5) < r.at5,
              "fs=%4.0f f=%3.0f  f %.1f  -+1 %.1f/%.1f  -+2 %.1f/%.1f  -+5 %.1f/%.1f dB (notch: f %.1f, +1 %.1f, +2 %.1f)",
              r.fs, r.f0, g0, gm1, gp1, gm2, gp2, gm5, gp5,
              notchDb(r.fs, r.f0, r.f0), notchDb(r.fs, r.f0, r.f0 + 1), notchDb(r.fs, r.f0, r.f0 + 2));
        CHECK(h2 < r.h2 && h3 < r.h3, "fs=%4.0f f=%3.0f  harmonics 2f %.1f dB, 3f %.1f dB (3f aliases to %.0f Hz)",
              r.fs, r.f0, h2, h3, fabsf(3 * r.f0 - r.fs * roundf(3 * r.f0 / r.fs)));
        float lp = combDb(r.fs, r.f0, 5.f);
        CHECK(lp > -0.2f, "fs=%4.0f f=%3.0f  passband 5 Hz %.2f dB", r.fs, r.f0, lp);
    }

    // DC gain 1, also from an unseeded (zero) ring
    {
        float worst = 0;
        for (float L: {2.0f, 2.3f, 4.5f, 5.13f, 9.99f, 14.7f, 22.9f}) {
            Comb c;
            c.tune(L);
            for (int n = 0; n < 200; ++n) {
                float y = c.filter(53.217f);
                if (n > 25) worst = fmaxf(worst, fabsf(y - 53.217f) / 53.217f);
            }
        }
        CHECK(worst < 3e-7f, "DC gain 1: worst relative error %.2g over L = 2 .. 22.9", worst);
    }

    // a step settles in one period (exact after ceil(L) + 1 samples) and is monotonic
    {
        bool ok = true;
        for (float L: {4.51f, 5.13f, 14.7f}) {
            Comb c;
            c.tune(L);
            c.seed(0);
            float prev = 0;
            int settle = 0;
            for (int n = 0; n < 40; ++n) {
                float y = c.filter(1.f);
                if (y < prev - 1e-6f) ok = false;
                prev = y;
                if (fabsf(y - 1) > 1e-6f) settle = n + 1;
            }
            ok &= settle <= (int) ceilf(L) + 1;
            printf("        L=%.2f: step settled after %d samples\n", L, settle);
        }
        CHECK(ok, "step response monotonic, settled within ceil(L) + 1 samples");
    }

    // retune: on a level the output never moves; with 1 V ripple the change at a retune (also across the integer
    // part) stays at the size of the normal sample-to-sample residual
    {
        Comb c;
        c.tune(4.51f);
        c.seed(26.f);
        float dmax = 0;
        for (int n = 0; n < 500; ++n) {
            if (n == 200) c.tune(3.99f);
            if (n == 350) c.tune(4.51f);
            dmax = fmaxf(dmax, fabsf(c.filter(26.f) - 26.f));
        }
        CHECK(dmax < 1e-5f, "retune on a level: max deviation %.2g V", dmax);

        // input fixed at 112.5 Hz (L = 4.00 at 450 sps); the comb retunes 109.8 -> 113.9 Hz (L 4.10 -> 3.95)
        const float fs = 450.f, f = 112.5f;
        Comb d;
        d.tune(4.10f);
        d.seed(26.f);
        float prev = 26.f, jumpRetune = 0, jumpBefore = 0, jumpAfter = 0;
        for (int n = 0; n < 3000; ++n) {
            if (n == 2000) d.tune(3.95f);
            float y = d.filter(26.f + sinf(2 * (float) M_PI * f * (float) n / fs));
            float j = fabsf(y - prev);
            if (n == 2000) jumpRetune = j;
            else if (n > 1000 && n < 2000) jumpBefore = fmaxf(jumpBefore, j);
            else if (n > 2000) jumpAfter = fmaxf(jumpAfter, j);
            prev = y;
        }
        CHECK(jumpRetune <= 1.5f * jumpBefore, "retune across the integer part on 1 V ripple: step %.4f V, residual"
              " sample-to-sample %.4f V before, %.4f V after", jumpRetune, jumpBefore, jumpAfter);
    }

    // long run: running sum vs the exact double-precision FIR after 2e7 samples at ~80 V with ripple and noise
    {
        const double L = 513.0 / 100.0;
        const int N = (int) L;
        const double a = L - N;
        double h[32] = {}, ring[32];
        h[0] = .5, h[N] = .5 + a - a * a / 2, h[N + 1] = a * a / 2;
        for (int k = 1; k < N; ++k) h[k] = 1;
        Comb c;
        c.tune((float) L);
        c.seed(80.f);
        for (double &r: ring) r = 80.f;
        unsigned seed = 7;
        float worst = 0;
        for (int n = 0; n < 20000000; ++n) {
            seed = seed * 1103515245u + 12345u;
            float x = 80.f + 3.f * sinf(2 * (float) M_PI * (float) (n % 513) / 5.13f) +
                      (float) ((int) (seed >> 16) % 1000) * 1e-3f;
            float y = c.filter(x);
            for (int k = 31; k > 0; --k) ring[k] = ring[k - 1];
            ring[0] = x;
            if (n % 997 == 0 || n > 20000000 - 100) {
                double ref = 0;
                for (int k = 0; k <= N + 1; ++k) ref += h[k] * ring[k];
                worst = fmaxf(worst, (float) fabs(y - ref / L));
            }
        }
        CHECK(worst < 1e-4f, "2e7 samples at 80 V: max |running - exact| %.2g V", worst);
    }

    {
        Comb c;
        c.tune(2000.f / 80.f);
        CHECK(!Comb::fits(2000.f / 80.f) && c.filter(1.5f) == 1.5f && c.filter(-2.f) == -2.f,
              "L = 25 > ring: pass-through");
    }

    // REAL fry Vout capture: the ripple tone before / after (same normalised frequency as on the device)
    {
        static float x[FRY_VOUT_N], yc[FRY_VOUT_N], yn[FRY_VOUT_N];
        const float fs = FRY_VOUT_FS_HZ, f = FRY_VOUT_RIPPLE_HZ;
        Comb c;
        c.tune(fs / f);
        NotchFilter nf;
        nf.begin(f / fs, -30.f, 20.f);
        c.seed(FRY_VOUT_INVERTER[0]);
        nf.seed(FRY_VOUT_INVERTER[0]);
        for (int n = 0; n < FRY_VOUT_N; ++n) {
            x[n] = FRY_VOUT_INVERTER[n];
            yc[n] = c.filter(x[n]);
            nf.filter(&x[n], &yn[n], 1);
        }
        const int s = 1000, m = FRY_VOUT_N - s;
        float a0 = toneAmp(x + s, m, fs, f), ac = toneAmp(yc + s, m, fs, f), an = toneAmp(yn + s, m, fs, f);
        float a2 = toneAmp(x + s, m, fs, 2 * f), ac2 = toneAmp(yc + s, m, fs, 2 * f), an2 = toneAmp(yn + s, m, fs, 2 * f);
        CHECK(20 * log10f(ac / a0) < -30.f,
              "REAL fry capture: ripple %.2f counts -> comb %.3f (%.1f dB), notch %.3f (%.1f dB); 2f %.3f -> comb %.4f, notch %.4f",
              a0, ac, 20 * log10f(ac / a0), an, 20 * log10f(an / a0), a2, ac2, an2);
    }
}

static void testNotchDepth() {
    printf("\nnotch depth\n");
    for (float g: {-15.f, -30.f, -40.f}) {
        float d = notchDb(450.f, 100.f, 100.f, g);
        CHECK(fabsf(d - g) < 0.5f, "requested %.0f dB -> %.1f dB at f0", g, d);
    }
}

int main() {
    printf("ripple-freq-test (adaptive inverter-ripple detector)\n");

    float hz, snr;
    struct { float f, amp; const char *name; } tones[] = {
        {100.f, 0.5f, "50Hz inverter -> 100Hz"},
        {120.f, 0.5f, "60Hz inverter -> 120Hz"},
        {126.f, 0.4f, "odd 63Hz -> 126Hz"},
        {117.4f, 0.5f, "sub-bin 117.4Hz (parabolic)"},
        {130.f, 0.05f, "weak 0.05V ripple"},
    };
    for (auto &t : tones) {
        detectTone(640.f, t.f, t.amp, 26.f, hz, snr);
        CHECK(fabsf(hz - t.f) < 1.5f && snr > 15.f,
              "%-32s detected %.2fHz snr=%.0f (want %.1f)", t.name, hz, snr, t.f);
    }

    // DC + noise only: must NOT report a confident tone (else the notch would chase noise)
    detectTone(640.f, 0.f, 0.f, 26.f, hz, snr);
    CHECK(snr < 15.f, "DC-only no false positive       snr=%.1f (< 15)", snr);

    // real field capture
    {
        RippleFreqDetector<61> det;
        uint16_t blockN = (uint16_t) (FRY_VOUT_FS_HZ * 0.75f);
        det.configure(FRY_VOUT_FS_HZ, 80.f, 140.f, blockN);
        hz = NAN; snr = 0; float h, s;
        for (int n = 0; n < FRY_VOUT_N; ++n) {
            det.push((float) FRY_VOUT_INVERTER[n]);
            if (det.poll(h, s)) { hz = h; snr = s; }
        }
        CHECK(fabsf(hz - FRY_VOUT_RIPPLE_HZ) < 2.0f && snr > 15.f,
              "REAL fry capture detected %.2fHz snr=%.0f (host-FFT %.2fHz)",
              hz, snr, FRY_VOUT_RIPPLE_HZ);
    }

    testComb();
    testNotchDepth();

    printf(failures ? "\nFAILED (%d)\n" : "\nOK\n", failures);
    return failures ? 1 : 0;
}
