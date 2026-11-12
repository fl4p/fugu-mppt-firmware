#pragma once
// esp-dsp notch generator + biquad, copied from the ANSI reference implementations
// (managed_components/espressif__esp-dsp/modules/iir/biquad/dsps_biquad_gen_f32.c, dsps_biquad_f32_ansi.c).
#include <cmath>
#include "esp_err.h"
inline esp_err_t dsps_biquad_gen_notch_f32(float *coeffs, float f, float gain, float qFactor) {
    if (qFactor <= 0.0001) qFactor = 0.0001;
    float Fs = 1;
    float A = sqrtf(pow(10, (double) gain / 20.0));
    float w0 = 2 * M_PI * f / Fs;
    float c = cosf(w0);
    float s = sinf(w0);
    float alpha = s / (2 * qFactor);
    float b0 = 1 + alpha * A, b1 = -2 * c, b2 = 1 - alpha * A;
    float a0 = 1 + alpha, a1 = -2 * c, a2 = 1 - alpha;
    coeffs[0] = b0 / a0; coeffs[1] = b1 / a0; coeffs[2] = b2 / a0; coeffs[3] = a1 / a0; coeffs[4] = a2 / a0;
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
