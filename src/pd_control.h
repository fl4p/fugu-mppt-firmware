#pragma once

/**
 * PD controller (PID without I component)
 */
struct PD_Control {
    const float Kp, Kd;
    const bool normalize;//
    float _prevE = NAN;

    /**
     *
     * @param Kp proportional gain
     * @param Kd derivative gain
     * @param normalize normalize measurement to setpoint. Compute relative error.
     */
    PD_Control(float Kp, float Kd, bool normalize) : Kp(Kp), Kd(Kd), normalize(normalize) {}

    /**
     * Clears memory. Next D component will be 0
     */
    void reset() {
        _prevE = NAN;
    }

    virtual float update(float measurement, float setpoint) {
        if (normalize) {
            measurement /= setpoint;
            setpoint = 1;
        }
        auto e = setpoint - measurement;
        auto de = e - _prevE;
        _prevE = e;
        if (std::isnan(de)) de = 0; // first D component is 0
        return Kp * e + Kd * de;
    }
};

struct PD_Control_SmoothSetpoint : PD_Control {
    EWMA<float> _ewma;
    PD_Control_SmoothSetpoint(float Kp, float Kd, int smoothSpan) : PD_Control(Kp, Kd, true), _ewma(smoothSpan) {}
    float update(float actual, float target) override {
        _ewma.add(target);
        return PD_Control::update(actual, _ewma.get());
    }
};