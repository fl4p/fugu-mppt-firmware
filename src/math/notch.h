#pragma once

#include "esp_dsp.h"
#include <utility>
#include <math.h>

class NotchFilter {
    float coeffs[6] = {0, 0, 0, 0, 0, 0}; // 2nd denominator+numerator
    float w_lpf[2] = {0, 0}; // filter delay line (state)
public:
    //explicit NotchFilter(float f, float gain = -30.f, float Q = 20.f) {
    //    begin(f, gain, Q);
    //}

    static float gainAmp(float gainDb) { return sqrtf(powf(10.f, gainDb / 20.f)); }

    void begin(float f, float gain = -30.f, float Q = 20.f) { tune(f, gainAmp(gain), Q); }

    // dsps_biquad_gen_notch_f32 without libm (its sin/cos/pow run in soft double, ~170 us);
    // f = f0/fs in (0, 0.5), amp from gainAmp()
    void tune(float f, float amp, float Q) {
        // sin, cos of w0 in (0, pi): fold to [0, pi/4], Taylor to z^9 / z^8 (error < 3e-8)
        float x = 2 * (float) M_PI * f;
        bool neg = x > (float) M_PI_2;
        if (neg) x = (float) M_PI - x;
        bool swap = x > (float) M_PI_4;
        float z = swap ? (float) M_PI_2 - x : x, q = z * z;
        float s = z * (1 + q * (-1 / 6.f + q * (1 / 120.f + q * (-1 / 5040.f + q * (1 / 362880.f)))));
        float c = 1 + q * (-1 / 2.f + q * (1 / 24.f + q * (-1 / 720.f + q * (1 / 40320.f))));
        if (swap) std::swap(s, c);
        if (neg) c = -c;
        float alpha = s / (2 * Q), a0 = 1 + alpha;
        coeffs[0] = (1 + alpha * amp) / a0;
        coeffs[1] = coeffs[3] = -2 * c / a0;
        coeffs[2] = (1 - alpha * amp) / a0;
        coeffs[4] = (1 - alpha) / a0;
    }

    // DC steady state for input x (direct form II), so a level step doesn't ring
    void seed(float x) {
        w_lpf[0] = w_lpf[1] = x / (1.f + coeffs[3] + coeffs[4]);
    }

    void filter(const float *in, float *out, int len) {
        ESP_ERROR_CHECK(dsps_biquad_f32(in, out, len, coeffs, w_lpf));
    }
};