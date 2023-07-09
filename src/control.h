#pragma once

/**
 * PD controller (PID without I component)
 */
struct PD {
    const float P, D;
    const bool normalize;
    float _prevE = NAN;

    /**
     *
     * @param P proportional gain
     * @param D derivative gain
     * @param normalize normalize input values. Compute relative error.
     */
    PD(float P, float D, bool normalize) : P(P), D(D), normalize(normalize) {}

    /**
     * Clears memory. Next D component will be 0
     */
    void reset() {
        _prevE = NAN;
    }

    float update(float actual, float target) {
        if (normalize) {
            actual /= target;
            target = 1;
        }
        auto e = target - actual;
        auto de = e - _prevE;
        _prevE = e;
        if (std::isnan(de)) de = 0; // first D component is 0
        return P * e + D * de;
    }
};