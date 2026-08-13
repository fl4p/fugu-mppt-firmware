#pragma once

#include "conf.h"

/**
 * PD controller (PID without I component)
 * TODO https://github.com/espressif/idf-extra-components/blob/master/pid_ctrl/src/pid_ctrl.c
 */
struct PD_Control {
    float Kp, Kd;
    float Td = NAN; // derivative time [s]; when finite it replaces Kd (see update())
    const bool normalize;//
    float _prevE = NAN;

    /**
     *
     * @param Kp proportional gain
     * @param Kd derivative gain, applied to the per-sample error difference
     * @param normalize normalize measurement to setpoint. Compute relative error.
     */
    PD_Control(float Kp, float Kd, bool normalize) : Kp(Kp), Kd(Kd), normalize(normalize) {}

    /**
     * Clears memory. Next D component will be 0
     */
    void reset() {
        _prevE = NAN;
    }

    /**
     * @param dt seconds since the previous update. Only used with a finite Td; <=0 suppresses the
     *           D component (as does the first call after a reset).
     */
    virtual float update(float measurement, float setpoint, float dt) {
        if (normalize) {
            measurement /= setpoint;
            setpoint = 1;
        }
        auto e = setpoint - measurement;
        auto de = e - _prevE;
        _prevE = e;
        if (std::isnan(de)) de = 0; // first D component is 0
        // Kd multiplies the raw per-sample difference, so its contribution scales with the loop
        // period: the same Kd is a different derivative gain at a different sample rate. A finite
        // Td expresses it as a time instead, making the D component sample-rate invariant.
        auto d = std::isfinite(Td) ? (dt > 0 ? Kp * Td * de / dt : 0.f) : Kd * de;
        return Kp * e + d;
    }
};

struct PD_Control_SmoothSetpoint : PD_Control {
    EWMA<float> _ewma;
    PD_Control_SmoothSetpoint(float Kp, float Kd, int smoothSpan) : PD_Control(Kp, Kd, true), _ewma(smoothSpan) {}
    float update(float actual, float target, float dt) override {
        _ewma.add(target);
        return PD_Control::update(actual, _ewma.get(), dt);
    }
};

// strtof parses "nan"/"inf" cleanly, so a typo'd gain would arrive non-finite and latch the
// converter in shutdownDcdc("ctrl-nan") with nothing naming the key. Refuse it, keep the default.
inline float pdReadGain(const ConfFile &conf, const std::string &key, float def) {
    float v = conf.getFloat(key, def);
    if (!std::isfinite(v)) {
        ESP_LOGE("pd_ctrl", "%s=%f is not finite, keeping %f", key.c_str(), v, def);
        return def;
    }
    return v;
}

/**
 * Applies conf keys `ctrl_<name>_{kp,kd,td}` on top of whatever gains ctrl already holds. An absent
 * key leaves that gain untouched, so a conf setting none of them is a no-op.
 */
inline void pdLoadGains(const ConfFile &conf, PD_Control &ctrl, const char *name) {
    const std::string p = std::string("ctrl_") + name + "_";
    ctrl.Kp = pdReadGain(conf, p + "kp", ctrl.Kp);
    ctrl.Kd = pdReadGain(conf, p + "kd", ctrl.Kd);

    // NaN is the "use legacy Kd" sentinel here, so Td does not go through pdReadGain.
    float td = conf.getFloat(p + "td", ctrl.Td);
    if (std::isfinite(td) && td < 0) {
        // A negative Td inverts the D term. These units feed a min() across limiters, so an
        // inverted response turns "not limiting" into "limit hard" on a protection path.
        ESP_LOGE("pd_ctrl", "%std=%f is negative, ignoring", p.c_str(), td);
        td = NAN;
    }
    ctrl.Td = td;

    if (std::isfinite(ctrl.Td))
        ESP_LOGI("pd_ctrl", "ctrl %s: Kp=%.4g Td=%.4gs (Kd ignored)", name, ctrl.Kp, ctrl.Td);
    else
        ESP_LOGI("pd_ctrl", "ctrl %s: Kp=%.4g Kd=%.4g (per-sample)", name, ctrl.Kp, ctrl.Kd);
}
