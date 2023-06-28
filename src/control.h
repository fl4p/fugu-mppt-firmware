#pragma once

struct PD {
    const float P, D;
    const bool normalize;
    float _prevE = NAN;

    PD(float P, float D, bool normalize) : P(P), D(D), normalize(normalize) {}

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