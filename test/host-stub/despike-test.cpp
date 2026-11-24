// Host test for the glitch-safe (decision-based) median in src/adc/sampling.h::Sensor::add_sample.
// Sensor itself is IDF-coupled (esp_dsp/esp_log), so this mirrors the exact 6-line decision logic
// against the same RunningMedian5 and asserts the properties validated on the real China-inverter
// capture: dense load pulses pass through (mean stays unbiased) while a lone impulse glitch is
// clipped, a quantized quiet signal is not clipped (LSB floor), a glitch on a channel quiet since
// reset (mean deviation 0) is clipped while a real step there arrives as fast as with the plain median,
// noisy channels match the pre-floor logic, and disabled mode reproduces the legacy unconditional
// median bit-for-bit.
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
// identity transform (raw == value, factor 1). legacy = the branch before the quiet-channel floor.
struct Despiker {
    RunningMedian5<float> med3{};
    float despikeScale = 0, despikeStep = 0, lastRaw = NAN;
    uint32_t despikeTrips = 0, numSamples = 0;
    const bool enabled; const float despikeK; const bool legacy;
    Despiker(bool en, float k = 8.f, bool legacy = false) : enabled(en), despikeK(k), legacy(legacy) {}
    void reset() { med3.reset(); despikeScale = 0; numSamples = 0; }   // Sensor::reset keeps despikeStep
    float step(float v) {
        const float dRaw = std::fabs(v - lastRaw);
        lastRaw = v;
        float m = med3.next(v);
        if (enabled && legacy) {
            float dev = std::fabs(v - m);
            if (dRaw > 0.f) {
                float d = dRaw * 1.f;
                if (d < despikeStep || despikeStep == 0.f) despikeStep = d;
            }
            if (despikeScale > 0.f && dev > despikeK * despikeScale && dev > 4.f * despikeStep) { v = m; ++despikeTrips; }
            despikeScale = (despikeScale > 0.f) ? (despikeScale + 0.095f * (dev - despikeScale)) : dev;
        } else if (enabled) {
            float dev = std::fabs(v - m);
            if ((despikeScale > 0.f || numSamples >= 5) && dev > despikeK * despikeScale && dev > 4.f * despikeStep) {
                v = m;
                ++despikeTrips;
                if (despikeScale == 0.f) despikeScale = 1e-30f;
            } else if (dRaw > 0.f) {
                float d = dRaw * 1.f;
                if (d < despikeStep || despikeStep == 0.f) despikeStep = d;
            }
            despikeScale = (despikeScale > 0.f) ? (despikeScale + 0.095f * (dev - despikeScale)) : dev;
        } else {
            v = m;
        }
        ++numSamples;
        return v;
    }
};

// First output index (>= at) whose value is within tol of target.
int settleAt(const std::vector<float> &y, int at, float target, float tol) {
    for (size_t i = at; i < y.size(); ++i) if (std::fabs(y[i] - target) <= tol) return (int) i - at;
    return -1;
}

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

    // --- 5) Quiet channel (mean deviation exactly 0): single and double glitches are clipped. ---
    for (int known = 0; known < 2; ++known) {
        for (int width = 1; width <= 2; ++width) {
            Despiker d(true), old(true, 8.f, true);
            if (known) {   // step learned from 1-LSB toggles before a reset, then the ADC sits at 512 counts
                for (int i = 0; i < 200; ++i) { d.step(512.f + (i & 1)); old.step(512.f + (i & 1)); }
                d.reset(); old.reset();
            }
            float maxOut = 0, maxOld = 0;
            for (int i = 0; i < 400; ++i) {
                float x = 512.f + ((i >= 200 && i < 200 + width) ? 60.f : 0.f);
                maxOut = std::max(maxOut, d.step(x));
                maxOld = std::max(maxOld, old.step(x));
            }
            std::printf("  quiet (step %s) %d-sample glitch +60: out max %.0f (pre-floor %.0f) trips %u\n",
                        known ? "known" : "unknown", width, maxOut, maxOld, d.despikeTrips);
            EXPECT(maxOut == 512.f);
            EXPECT(d.despikeTrips == (uint32_t) width);
            EXPECT(maxOld > 512.f);   // the defect: the pre-floor branch passed it
        }
    }
    // --- 6) Quiet channel + real step: reaches the EWM input no later than the plain median. ---
    for (int known = 0; known < 2; ++known) {
        for (float h: {3.f, 60.f, -60.f}) {
            Despiker d(true), plain(false);
            if (known) { for (int i = 0; i < 200; ++i) d.step(512.f + (i & 1)); d.reset(); }
            std::vector<float> yd, yp;
            for (int i = 0; i < 300; ++i) {
                float x = 512.f + (i >= 100 ? h : 0.f);
                yd.push_back(d.step(x)); yp.push_back(plain.step(x));
            }
            int ld = settleAt(yd, 100, 512.f + h, 0.f), lp = settleAt(yp, 100, 512.f + h, 0.f);
            bool stays = true;
            for (int i = 100 + ld; i < 300; ++i) stays &= yd[i] == 512.f + h;
            std::printf("  quiet (step %s) real step %+.0f: delay despike %d, plain median %d samples\n",
                        known ? "known" : "unknown", h, ld, lp);
            EXPECT(ld >= 0 && ld <= lp);
            EXPECT(stays);
        }
    }
    // --- 6b) Dense pulses starting on a dead-constant baseline: the scale grows, pulses pass again. ---
    {
        auto sig = densePulseTrain(N, 100.f, 80.f);
        for (int i = 0; i < 200; ++i) sig[i] = 100.f;
        Despiker d(true);
        std::vector<float> y(N);
        for (int i = 0; i < N; ++i) y[i] = d.step(sig[i]);
        double bias = ewmMean(y, 10, from) - mean(sig, from);
        std::printf("  pulses after a dead-constant start: despike bias=%.2f trips=%u\n", bias, d.despikeTrips);
        EXPECT(std::fabs(bias) < 1.0);
    }
    // --- 7) Noisy channels: identical to the pre-floor logic once the window is full. ---
    {
        std::mt19937 rng(7);
        std::normal_distribution<float> nd(0.f, 1.f);
        const int Nn = 200000;
        int diff = 0, diffLate = 0;
        double maxAbs = 0;
        for (int kase = 0; kase < 3; ++kase) {
            Despiker d(true), old(true, 8.f, true);
            for (int i = 0; i < Nn; ++i) {
                float x;
                if (kase == 0) x = std::round(2048.f + 3.f * nd(rng)) + ((i % 997 == 0) ? 80.f : 0.f);   // quantized + glitches
                else if (kase == 1) x = 30.f + 0.05f * nd(rng) + ((i % 9 == 0) ? 8.f : (i % 9 == 4) ? 4.8f : 0.f);   // dense pulses
                else x = 27.f + 0.01f * nd(rng) + ((i / 5000) & 1) * 2.f;   // steps on fine noise
                float a = d.step(x), b = old.step(x);
                if (a != b) { ++diff; diffLate += i >= 5; maxAbs = std::max(maxAbs, (double) std::fabs(a - b)); }
            }
        }
        std::printf("  noisy: %d differing outputs of %d (after window fill %d), max |delta| %.3g\n",
                    diff, 3 * Nn, diffLate, maxAbs);
        EXPECT(diffLate * 100000 < 3 * Nn);   // < 1e-5
    }

    std::printf("\ndespike-test: %d/%d passed\n", g_run - g_fail, g_run);
    return g_fail ? 1 : 0;
}
