#pragma once

#include <cmath>

// PV panel I-V curve: single-diode-ish exponential parameterized by Isc, Voc, k = V_mpp/Voc.
// alpha is solved from k once at set()-time. Pure, no IDF deps. Used by the vconv simulator
// (input source) and the PV-sim output mode (inverse, V from I).
// The alpha family realizes k in ~(0.5, 1); below that the solver floors and the curve
// degenerates toward linear — callers validate k.
struct PvModel {
    float isc = 8.0f;
    float voc = 40.0f;
    float k = 0.8f;
    float alpha = 11.5f;
    float norm = 1.0f;

    PvModel() { set(isc, voc, k); }

    // isc > 0, voc > 0, 0 < k < 1
    void set(float isc_, float voc_, float k_) {
        isc = isc_; voc = voc_; k = k_;
        // Solve alpha * (1 - k) = ln(1 + k * alpha) by Newton iteration.
        // f(a)  = a*(1-k) - ln(1 + k*a)
        // f'(a) = (1-k) - k / (1 + k*a)
        float a = (k > 0.0f && k < 1.0f) ? (1.0f / (1.0f - k)) : 1.0f;
        for (int i = 0; i < 12; ++i) {
            float fa = a * (1.0f - k) - std::log(1.0f + k * a);
            float fp = (1.0f - k) - k / (1.0f + k * a);
            if (std::fabs(fp) < 1e-9f) break;
            float step = fa / fp;
            a -= step;
            if (a < 1e-3f) a = 1e-3f;
            if (std::fabs(step) < 1e-7f) break;
        }
        alpha = a;
        norm = 1.0f / (1.0f - std::exp(-a));
        vocOverAlpha_ = voc / a;
        invIscNorm_ = 1.0f / (isc * norm);
        expNegAlpha_ = std::exp(-a);
    }

    [[nodiscard]] float current(float v) const {
        if (v >= voc) return 0.0f;
        if (v <= 0.0f) return isc;
        float i = isc * (1.0f - std::exp(alpha * (v - voc) / voc)) * norm;
        if (i < 0.0f) return 0.0f;
        if (i > isc) return isc;
        return i;
    }

    // Exact inverse of current(): V(0) = Voc, V(>= Isc) = 0. No divides (RT hot path).
    [[nodiscard]] float voltage(float i) const {
        if (i <= 0.0f) return voc;
        float x = 1.0f - i * invIscNorm_;
        if (x <= expNegAlpha_) return 0.0f;
        float v = voc + vocOverAlpha_ * std::log(x);
        if (v < 0.0f) return 0.0f;
        if (v > voc) return voc;
        return v;
    }

private:
    float vocOverAlpha_ = 0.0f;
    float invIscNorm_ = 0.0f;
    float expNegAlpha_ = 0.0f;
};
