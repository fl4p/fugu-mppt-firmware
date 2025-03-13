#pragma once

#include "esp_dsp.h"

class NotchFilter {
    float coeffs[6] = {0, 0, 0, 0, 0, 0}; // 2nd denominator+numerator
    float w_lpf[2] = {0, 0}; // filter delay line (state)
public:
    //explicit NotchFilter(float f, float gain = -30.f, float Q = 20.f) {
    //    begin(f, gain, Q);
    //}

    void begin(float f, float gain = -30.f, float Q = 20.f) {
        ESP_ERROR_CHECK(dsps_biquad_gen_notch_f32(coeffs, f, gain, Q));
    }

    void filter(const float *in, float *out, int len) {
        ESP_ERROR_CHECK(dsps_biquad_f32(in, out, len, coeffs, w_lpf));
    }
};