#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

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

// Series string of n (2..6) equal substrings, each with a bypass diode: g[i] (percent) scales the Isc
// of substring i. n < 2: one unshaded panel.
struct PvShade {
    static constexpr uint8_t kMax = 6;
    uint8_t n = 0;
    uint8_t g[kMax]{};

    [[nodiscard]] bool on() const { return n >= 2; }

    // String voltage at current i; a substring carrying more than its Isc is bypassed at -vd. Over the
    // last kKnee of its Isc a substring falls linearly to -vd, which bounds the slope dV/dI.
    static constexpr float kKnee = 0.05f;
    [[nodiscard]] float voltage(const PvModel &m, float i, float vd = 0.5f) const {
        if (!on()) return m.voltage(i);
        float v = 0;
        for (uint8_t k = 0; k < n; ++k) {
            const float gk = (float) g[k] * 0.01f, x = i / (gk * m.isc);
            if (x >= 1.0f) {
                v -= vd;
            } else if (x > 1.0f - kKnee) {
                const float vk = m.voltage(m.isc * (1.0f - kKnee)) / (float) n;
                v += vk - (vk + vd) * (x - (1.0f - kKnee)) / kKnee;
            } else {
                v += m.voltage(i / gk) / (float) n;
            }
        }
        return std::max(v, 0.0f);
    }

    // String short-circuit current over the panel's Isc.
    [[nodiscard]] float iscScale() const {
        if (!on()) return 1.0f;
        uint8_t mx = 0;
        for (uint8_t k = 0; k < n; ++k) mx = std::max(mx, g[k]);
        return (float) mx * 0.01f;
    }

    // "1,1,0.4" (2..6 factors in (0,1]); "off", "" and a single 1 clear. False on bad input.
    bool parse(const char *s) {
        PvShade r;
        if (s && *s && strcmp(s, "off") != 0) {
            for (;;) {
                char *e;
                const float f = strtof(s, &e);
                if (e == s || !(f > 0.0f && f <= 1.0f) || r.n == kMax) return false;
                r.g[r.n++] = (uint8_t) std::max(1L, std::lround(f * 100.0f));
                if (!*e) break;
                if (*e != ',') return false;
                s = e + 1;
            }
            if (r.n == 1 && r.g[0] == 100) r.n = 0;
            else if (r.n < 2) return false;
        }
        *this = r;
        return true;
    }
};
