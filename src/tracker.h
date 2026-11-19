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
    static constexpr float VinAgainstBand = 0.01f; // of Vin, see update()
    static constexpr time_ms HandBackMs = 3000;

    float minPowerStep = 1.0f; // reversal threshold [W] of the last decision
    float frequency = 20;
    float revK = 1.4f, revFloor = 0.05f; // threshold k*sqrt(2)*sigma, at least revFloor [W]

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

    // noise of one period mean per mode, from second differences of successive means
    float _p1 = NAN, _p2 = NAN;
    EWMA<float> _noiseVar[2]{EWMA<float>(32), EWMA<float>(32)};
    uint8_t _noiseN[2]{};
    uint8_t _noiseIdle[2]{}; // periods since the last noise sample

    void resetTracker(float power, bool direction) {
        _direction = direction;
        _time = wallClockMs();
        _lastPower = power;
        maxPowerPoint.power = 0;
        maxPowerPoint.vin = 0;
    }

    void setSlowMode(bool slow) {
        if (slow != slowMode) _p1 = _p2 = NAN;
        slowMode = slow;
        frequency = slow ? 2 : 30;
    }

    // fewer than 8 noise samples or none for 32 periods: the fixed rule, else max(k*sqrt(2)*sigma, floor).
    // rho: correlation of adjacent period means (AR(1)), var(d) = 2 sigma^2 (1 - rho)(3 - rho)
    bool exceedsNoise(float absDp, float rho) {
        const int m = slowMode;
        const float var = _noiseVar[m].get() / (2 * (1 - rho) * (3 - rho));
        if (_noiseN[m] >= 8 && _noiseIdle[m] < 32 && std::isfinite(var)) {
            minPowerStep = std::max(revK * 1.41421356f * sqrtf(std::max(var, 0.f)), revFloor);
            return absDp >= minPowerStep;
        }
        const float step = slowMode ? 0.5f : 1.0f, rel = slowMode ? 0.0015f : 0.015f;
        const float lp = std::abs(_lastPower);
        minPowerStep = lp > step ? std::min(step, rel * (lp + 5.0f)) : step;
        return absDp >= step || (lp > step && absDp / (lp + 5.0f) > rel);
    }

    // EWMA of d^2, d = p - 2 p1 + p2
    void addNoiseSample(float p) {
        if (!isnan(_p2)) {
            const int m = slowMode;
            auto &e = _noiseVar[m];
            float d = p - 2 * _p1 + _p2, x = d * d;
            // variance at which theta == floor: a zero estimate can rise again
            const float vf = revFloor * revFloor / (2 * revK * revK + 1e-6f);
            if (_noiseN[m] >= 8) x = std::min(x, 9 * std::max(e.get(), vf));
            e.add(x);
            if (_noiseN[m] < 255) ++_noiseN[m];
            _noiseIdle[m] = 0;
        }
        _p2 = _p1;
        _p1 = p;
    }

    // limiter hand-back: starts the settle window at vin, fast mode; slow mode waits for a reversal and 30 s
    void resetDirection(bool direction, float vin) {
        _direction = direction;
        _time = wallClockMs();
        _timeSettle = _timeTakeover = _time;
        _p1 = _p2 = NAN;
        if (!isnan(vin)) _lastVin = vin;
        setSlowMode(false);
    }

    // continue after a pause that returned to the same operating point, power settled at `power`
    void resume(float power) {
        _time = wallClockMs();
        _powerBuf.clear(), _vinBuf.clear();
        _p1 = _p2 = NAN;
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
                _p1 = _p2 = NAN;
            } else {
                auto absDp = std::abs(dP);
                bool rev = false;
                if (_noiseIdle[slowMode] < 255) ++_noiseIdle[slowMode];

                // after a reversal the filtered power still shows the old direction and the input cap's
                // charge/discharge, which lags the duty by ~C*V/(2.5*I) in low light: the reference follows the
                // power until the filter has settled and Vin is back at the reversal point, at most 1 s (past that,
                // Vin more than 1 % against still holds a reversal, below). Not in slow mode: a window there can only
                // end at a 500 ms decision, so it skips a whole period and doubles the leg. The slew is 25x slower,
                // so the input-cap current it causes stays below the power threshold
                // Vin against the way we steer since the last reversal (> 0: not back yet). 3 s after a hand-back, a
                // power drop with Vin more than 1 % against, or a 5 % Vin move against, comes from the plant, not from
                // our step, and does not reverse: near D = M a DCM plant can drop Vin by several volts at an unchanged
                // duty (ring current at HS turn-on), and reversing on that steered further down, to a flat P(D) far
                // below the MPP. The band keeps reversals on light changes; the 3 s skip the overshoot of the fade
                // or limiter step before the hand-back, which is no reference
                // filter time constant = 99 % settling time / 6.64
                const float settle = settleMs();
                const float rho = settle > 0 ? std::min(expf(-6.64f * 1000.f / (frequency * settle)), 0.95f) : 0.f;
                const float against = _lastVin > 0 ? (_direction ? vin - _lastVin : _lastVin - vin) : 0;
                const bool plant = now - _timeTakeover > HandBackMs && against > 0;
                auto since = now - _timeSettle;
                if (!slowMode && since < 1000 && ((float) since < settle || against > 0)) {
                    _lastPower = power;
                    _p1 = _p2 = NAN;
                } else {
                    if (exceedsNoise(absDp, rho)) {
                        _lastPower = power;
                        if (dP < 0 && !(plant && against > VinAgainstBand * _lastVin)) rev = true;
                    }
                    addNoiseSample(power);
                }

                // cloud recovery
                // when vin changed 5% since last reversal, reverse.
                // we might be tracking on a light intensity transient
                // and end up at a voltage far away from MPP
                // 5 % of the higher end, so down and up legs are equal (5 % of each start ratchets down)
                if (!plant && abs(vin - _lastVin) > 0.05f * std::max(vin, _lastVin)) {
                    rev = true;
                }

                if (rev) {
                    _p1 = _p2 = NAN;
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
