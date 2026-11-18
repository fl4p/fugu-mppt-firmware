#pragma once

#include "math/statmath.h"
#include "util.h"

struct MPP {
    float power = 0;
    time_ms timestamp = 0;
    uint16_t dutyCycle = 0;
    float vin = 0;
};


/**
 * Implement local MPP tracking perturb / observe.
 * If it doesn't find a new MPP for some time, it'll switch into slow mode.
 * Slow mode uses a lower update frequency, smaller power thresholds and a slower perturbation speed
 */
struct Tracker {
    float minPowerStep = 1.0f;
    float minPowerStepRel = 1.0f;
    float frequency = 20;

    bool _direction = false; //true => increase duty cycle/decrease solar voltage
    time_ms _time = 0;
    time_ms _timeLastReverse = 0;
    time_ms _timeSettle = 0; // settle window start: reversal or limiter hand-back
    time_ms _timeTakeover = 0; // limiter hand-back

    float _curPower = 0.0;
    float _lastPower = 0.0;
    float _lastVin = 0.0;

    MeanAccumulator _powerBuf{};
    MeanAccumulator _vinBuf{};

    MPP maxPowerPoint;

    float dP = NAN;

    bool slowMode = false;

    void resetTracker(float power, bool direction) {
        _direction = direction;
        _time = wallClockMs();
        _lastPower = power;
        maxPowerPoint.power = 0;
        maxPowerPoint.vin = 0;
    }

    void setSlowMode(bool slow) {
        slowMode = slow;
        frequency = slow ? 2 : 30;
        minPowerStep = slow ? 0.5f : 1.0f;
        minPowerStepRel = slow ? 0.0015f : 0.015f;
    }

    // limiter hand-back: starts the settle window at vin, fast mode; slow mode waits for a reversal and 30 s
    void resetDirection(bool direction, float vin) {
        _direction = direction;
        _time = wallClockMs();
        _timeSettle = _timeTakeover = _time;
        if (!isnan(vin)) _lastVin = vin;
        setSlowMode(false);
    }

    // continue after a pause that returned to the same operating point, power settled at `power`
    void resume(float power) {
        _time = wallClockMs();
        _powerBuf.clear(), _vinBuf.clear();
        _lastPower = power;
    }

    // settleMs(): step settling time [ms] of the filter on powerSample, called once per tracker period.
    // Returns the control value; on slow-mode entry the distance [counts] to the MPP duty.
    template<class SettleMs>
    float update(float powerSample, uint16_t dutyCycle, float vin, SettleMs settleMs) {
        auto now = wallClockMs();

        _powerBuf.add(powerSample);
        _vinBuf.add(vin);

        if ((float) (now - _time) > (1000.f / frequency)) {
            _time = now;
            auto power = _powerBuf.pop();
            auto vin = _vinBuf.pop();

            _curPower = power;
            dP = power - _lastPower;

            if (power < 1) {
                _direction = true; // pump more power
            } else {
                auto absDp = std::abs(dP);
                bool rev = false;

                // after a reversal the filtered power still shows the old direction and the input cap's
                // charge/discharge, which lags the duty by ~C*V/(2.5*I) in low light: the reference follows the
                // power until the filter has settled and Vin is back at the reversal point. At most 1 s: the
                // excursion of a longer blind leg costs more than the lag. Not in slow mode: a window there can only
                // end at a 500 ms decision, so it skips a whole period and doubles the leg. The slew is 25x slower,
                // so the input-cap current it causes stays below the power threshold
                auto since = now - _timeSettle;
                if (!slowMode && since < 1000 &&
                    ((float) since < settleMs() || (_direction ? vin > _lastVin : vin < _lastVin))) {
                    _lastPower = power;
                } else if (absDp >= minPowerStep
                           or (abs(_lastPower) > minPowerStep and absDp / (abs(_lastPower) + 5.0f) > minPowerStepRel)) {
                    _lastPower = power;
                    if (dP < 0) rev = true;
                }

                // cloud recovery
                // when vin changed 5% since last reversal, reverse.
                // we might be tracking on a light intensity transient
                // and end up at a voltage far away from MPP
                // 5 % of the higher end, so down and up legs are equal (5 % of each start ratchets down)
                if (abs(vin - _lastVin) > 0.05f * std::max(vin, _lastVin)) {
                    rev = true;
                }

                if (rev) {
                    _direction = !_direction;
                    _timeLastReverse = _timeSettle = now;
                    _lastVin = vin;
                }
            }

            // MPP record: invalid after 5 min, or below 85 % once 30 s old; evaluated before the slow-mode
            // entry, which waits on the same 30 s
            auto &m = maxPowerPoint;
            if (m.power > 0 && (now - m.timestamp > 1000 * 60 * 5 ||
                                (power < m.power * 0.85f && now - m.timestamp > 1000 * 30))) {
                ESP_LOGI("mppt", "Reset maxPower %.2f (P=%.2f)", m.power, power);
                m.power = 0;
            }
            bool newMax = power > m.power * 1.005f;
            if (newMax) m = {power, now, dutyCycle, vin};

            // slow mode: no new MPP for 30 s, a reversal since the last takeover and 30 s after it. Ends on a new
            // MPP (also after a reset) or 45 s without a reversal
            bool slow = power > (slowMode ? 0.5f : 2.f) && !newMax && _timeLastReverse > _timeTakeover &&
                        now - _timeLastReverse < 1000 * 45;
            if (!slowMode) slow = slow && now - m.timestamp > 1000 * 30 && now - _timeTakeover > 1000 * 30;
            bool enter = slow && !slowMode;
            setSlowMode(slow);
            // the MPP duty is a reference where a poor or non-linear output current measurement misleads the slow
            // P&O: the caller applies the distance like a control value, i.e. a fraction of it
            if (enter) return (float) m.dutyCycle - (float) dutyCycle;
        }

        float speed = slowMode ? .02f : .5f;
        return _direction ? speed : -speed;
    }
};
