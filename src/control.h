#pragma once

/**
 * PD controller (PID without I component)
 */
struct PD_Control {
    const float P, D;
    const bool normalize;
    float _prevE = NAN;

    /**
     *
     * @param P proportional gain
     * @param D derivative gain
     * @param normalize normalize input values. Compute relative error.
     */
    PD_Control(float P, float D, bool normalize) : P(P), D(D), normalize(normalize) {}

    /**
     * Clears memory. Next D component will be 0
     */
    void reset() {
        _prevE = NAN;
    }

    virtual float update(float actual, float target) {
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

struct PD_Control_SmoothTarget : PD_Control {
    EWMA<float> _ewma;
    PD_Control_SmoothTarget(float P, float D,  int smoothSpan) : PD_Control(P,D,true), _ewma(smoothSpan) {}
    float update(float actual, float target) override {
        _ewma.add(target);
        return PD_Control::update(actual, _ewma.get());
    }
};