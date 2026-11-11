#pragma once
// Sim: the two esp-dsp biquad functions notch.h uses (ANSI reference implementations).
#include <cmath>
#include "esp_log.h"

inline esp_err_t dsps_biquad_gen_notch_f32(float *coeffs, float f, float gain, float qFactor) {
    if (qFactor <= 0.0001) qFactor = 0.0001;
    float A = sqrtf(pow(10, (double) gain / 20.0));
    float w0 = 2 * M_PI * f;
    float c = cosf(w0), s = sinf(w0), alpha = s / (2 * qFactor);
    float a0 = 1 + alpha;
    coeffs[0] = (1 + alpha * A) / a0;
    coeffs[1] = -2 * c / a0;
    coeffs[2] = (1 - alpha * A) / a0;
    coeffs[3] = -2 * c / a0;
    coeffs[4] = (1 - alpha) / a0;
    return ESP_OK;
}

inline esp_err_t dsps_biquad_f32(const float *input, float *output, int len, float *coef, float *w) {
    for (int i = 0; i < len; i++) {
        float d0 = input[i] - coef[3] * w[0] - coef[4] * w[1];
        output[i] = coef[0] * d0 + coef[1] * w[0] + coef[2] * w[1];
        w[1] = w[0];
        w[0] = d0;
    }
    return ESP_OK;
}
