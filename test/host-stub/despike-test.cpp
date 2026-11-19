// Host test for the glitch-safe (decision-based) median in src/adc/sampling.h::Sensor::add_sample.
// Sensor itself is IDF-coupled (esp_dsp/esp_log), so this mirrors the exact 6-line decision logic
// against the same RunningMedian5 and asserts the properties validated on the real China-inverter
// capture: dense load pulses pass through (mean stays unbiased) while a lone impulse glitch is
// clipped, a quantized quiet signal is not clipped (LSB floor), and disabled mode reproduces the
// legacy unconditional median bit-for-bit.
//
// Build & run:
//   clang++ -std=gnu++17 -I . -o /tmp/despike-test test/host-stub/despike-test.cpp && /tmp/despike-test

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <vector>
#include <random>
using std::min; using std::max;
#include "../../src/math/statmath.h"

namespace {
int g_run = 0, g_fail = 0;
#define EXPECT(c) do{ ++g_run; if(!(c)){ ++g_fail; std::printf("  FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);} }while(0)

// Exact copy of the decision branch from Sensor::add_sample (despikeK=8, scale alpha 0.095),
// identity transform (raw == value, factor 1).
struct Despiker {
    RunningMedian5<float> med3{};
    float despikeScale = 0, despikeStep = 0, lastRaw = NAN;
    uint32_t despikeTrips = 0;
    const bool enabled; const float despikeK;
    Despiker(bool en, float k = 8.f) : enabled(en), despikeK(k) {}
    float step(float v) {
        const float dRaw = std::fabs(v - lastRaw);
        lastRaw = v;
        float m = med3.next(v);
        if (enabled) {
            float dev = std::fabs(v - m);
            if (dRaw > 0.f) {
                float d = dRaw * 1.f;
                if (d < despikeStep || despikeStep == 0.f) despikeStep = d;
            }
            if (despikeScale > 0.f && dev > despikeK * despikeScale && dev > 4.f * despikeStep) { v = m; ++despikeTrips; }
            despikeScale = (despikeScale > 0.f) ? (despikeScale + 0.095f * (dev - despikeScale)) : dev;
        } else {
            v = m;
        }
        return v;
    }
};

float ewmMean(const std::vector<float> &x, int span, int from) {
    float a = 2.f / (span + 1), y = NAN, last = 0;
    for (size_t i = 0; i < x.size(); ++i) { y = std::isnan(y) ? x[i] : (1 - a) * y + a * x[i]; if ((int) i >= from) last = y; }
    return last;
}
double mean(const std::vector<float> &x, int from) {
    double s = 0; int n = 0; for (size_t i = from; i < x.size(); ++i) { s += x[i]; ++n; } return s / n;
}

// Dense periodic current: baseline + a narrow pulse every 4-5 samples (China-inverter character).
std::vector<float> densePulseTrain(int N, float base, float pulse) {
    std::vector<float> v(N);
    for (int i = 0; i < N; ++i) v[i] = base + ((i % 9 == 0) ? pulse : (i % 9 == 4) ? 0.6f * pulse : 0.f);
    return v;
}
} // namespace

int main() {
    const int N = 3000, from = 300;
    // --- 1) Dense pulses: decision-median stays unbiased; plain median under-reads. ---
    {
        auto sig = densePulseTrain(N, 100.f, 80.f);
        double trueDc = mean(sig, from);
        Despiker plain(false), despike(true);
        std::vector<float> a(N), b(N);
        for (int i = 0; i < N; ++i) { a[i] = plain.step(sig[i]); b[i] = despike.step(sig[i]); }
        double biasPlain = ewmMean(a, 10, from) - trueDc;
        double biasDespk = ewmMean(b, 10, from) - trueDc;
        std::printf("  dense pulses: plain-median bias=%.2f  despike bias=%.2f  (trueDC=%.1f)\n",
                    biasPlain, biasDespk, trueDc);
        EXPECT(biasPlain < -3.0);                 // legacy median discards pulse charge -> reads low
        EXPECT(std::fabs(biasDespk) < 1.0);       // decision-median preserves the mean
    }
    // --- 2) Clean signal + lone glitch: glitch must be clipped (rejected). ---
    {
        std::vector<float> sig(N, 50.f);
        for (int i = 0; i < N; ++i) sig[i] += 0.3f * std::sin(i * 0.1f);   // mild ripple, no outliers
        sig[1500] += 90.f;                                                 // one impulse glitch
        Despiker despike(true);
        float out1500 = 0;
        for (int i = 0; i < N; ++i) { float o = despike.step(sig[i]); if (i == 1500) out1500 = o; }
        std::printf("  lone glitch: in=%.1f out=%.1f trips=%u\n", sig[1500], out1500, despike.despikeTrips);
        EXPECT(out1500 < 60.f);                   // 140 glitch clipped back toward the ~50 baseline
        EXPECT(despike.despikeTrips >= 1);        // and it tripped (WARN-worthy)
    }
    // --- 3) Quantized quiet signal (sigma 0.2 LSB): no clips; a 50-LSB glitch still clips. ---
    {
        const int Nq = 200000;
        std::mt19937 rng(1);
        std::normal_distribution<float> nd(0.3f, 0.2f);
        Despiker quiet(true), glitchy(true);
        uint32_t glitches = 0, glitchClipped = 0;
        for (int i = 0; i < Nq; ++i) {
            float q = std::round(nd(rng));
            quiet.step(q);
            bool g = i % 1000 == 500;
            uint32_t t0 = glitchy.despikeTrips;
            glitchy.step(g ? q + 50.f : q);
            if (g) { ++glitches; glitchClipped += glitchy.despikeTrips != t0; }
        }
        std::printf("  quantized sigma=0.2 LSB: step=%.2f clips=%u/%d; glitches clipped %u/%u\n",
                    quiet.despikeStep, quiet.despikeTrips, Nq, glitchClipped, glitches);
        EXPECT(quiet.despikeStep == 1.f);
        EXPECT(quiet.despikeTrips * 10000 < (uint32_t) Nq);  // < 1e-4
        EXPECT(glitchClipped == glitches);
    }
    // --- 4) Disabled == legacy unconditional median, sample-for-sample. ---
    {
        auto sig = densePulseTrain(N, 30.f, 50.f);
        Despiker off(false); RunningMedian5<float> ref{};
        bool identical = true;
        for (int i = 0; i < N; ++i) {
            float a = off.step(sig[i]), b = ref.next(sig[i]);
            if (i >= 5 && a != b) { identical = false; break; }   // skip NaN window-fill (NaN!=NaN)
        }
        EXPECT(identical);
    }

    std::printf("\ndespike-test: %d/%d passed\n", g_run - g_fail, g_run);
    return g_fail ? 1 : 0;
}
