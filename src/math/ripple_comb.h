#pragma once

#include <cstdint>

/**
 * Moving average over exactly one ripple period of L = fs/f samples (L fractional): the integral of the
 * linearly interpolated input over the last L samples, divided by L. Taps (x[n] first)
 * 1/2, 1, .., 1, 1/2 + a - a^2/2, a^2/2 with N = floor(L), a = L - N. Deep but finite rejection at f and at its
 * harmonics below fs/2 (exact zeros only for integer L), DC gain 1, DC delay L/2 samples, a step completes after
 * ceil(L) samples. O(1) per sample (running sum, re-summed once per buffer turn). The ring keeps its history over a
 * retune, so there is no reset transient, but the output can still step.
 */
template<uint8_t CAP>
class RippleComb {
    static_assert(CAP >= 4);
    float buf_[CAP]{};
    float sum_ = 0, c1_ = 0, c2_ = 0, invL_ = 0; // sum_: the last n_ + 1 samples
    uint8_t n_ = 0, pos_ = 0; // n_ = floor(L), 0 = pass through; pos_ = next write

    [[nodiscard]] float at(uint8_t k) const { // x[n-k], x[n] = last written
        int i = pos_ - 1 - k;
        return buf_[i < 0 ? i + CAP : i];
    }

    void resum() {
        float s = 0;
        for (uint8_t k = 0; k <= n_; ++k) s += at(k);
        sum_ = s;
    }

public:
    // window length the ring holds: N + 2 taps, 2 <= L < CAP - 1
    static bool fits(float L) { return L >= 2.f && L < CAP - 1; }

    // L [samples], 0 = pass through
    [[nodiscard]] float length() const { return n_ ? 1.f / invL_ : 0.f; }

    // L = fs / f [samples]; outside fits() the filter passes the input through
    void tune(float L) {
        if (!fits(L)) {
            n_ = 0;
            return;
        }
        const auto n = (uint8_t) L;
        const float a = L - (float) n;
        c2_ = .5f * a * a;
        c1_ = a - c2_ - .5f;
        invL_ = 1.f / L;
        if (n != n_) n_ = n, resum();
    }

    void seed(float x) {
        for (auto &b: buf_) b = x;
        sum_ = (float) (n_ + 1) * x;
    }

    __attribute__((always_inline)) inline float filter(float x) {
        float y = x;
        if (n_) {
            const float xo = at(n_); // x[n-N-1], leaves the sum
            sum_ += x - xo;
            y = (sum_ - .5f * x + c1_ * at(n_ - 1) + c2_ * xo) * invL_;
        }
        buf_[pos_] = x;
        if (++pos_ == CAP) {
            pos_ = 0;
            if (n_) resum(); // bounds the running sum's rounding drift
        }
        return y;
    }
};
