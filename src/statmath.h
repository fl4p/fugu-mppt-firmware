#pragma once

#include <cstdlib>

template<class float_t=float>
class EWMA {
/**
 * Implement exponential weighted moving average
 */

public:
    const float_t alpha;
    float_t y = std::numeric_limits<float_t>::quiet_NaN();

    explicit EWMA(uint32_t span) : alpha(2.f / (float_t)(span + 1)) {}

    inline void add(float_t x) {
        if(isnan(y))
            y = x;
        y = (1 - alpha) * y + alpha * x;
    }

    inline float_t get() const { return y; }
};

template<class float_t=float>
class EWM {
    float_t last_x = std::numeric_limits<float_t>::quiet_NaN();
public:
    EWMA<float_t> avg, std;

    explicit EWM(uint32_t span) : avg(span), std(span) {}

    inline void add(float_t x) {
        avg.add(x);
        if(!isnan(last_x)) {
            float_t pct = (x - last_x) / last_x;
            std.add(pct * pct);
        }
        last_x = x;
    }
};

