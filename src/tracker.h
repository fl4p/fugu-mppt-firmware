#pragma once

#include "statmath.h"

struct Tracker {
    const float minPowerStep = 1.5f;
    float frequency = 20;

    bool _direction = false;
    unsigned long _time = 0;

    float _lastPower = NAN;
    MeanAccumulator _powerBuf;

    float _maxPower = 0;
    unsigned long _timeMaxPower = 0;

    float dP = NAN;

    bool prevSlow = false;

    void resetTracker(float power, bool direction) {
        _lastPower = power;
        _direction = direction;
        _time = millis();
        _maxPower = 0;
    }

    float update(float powerSample) {
        auto now = millis();



        if ((now - _time) > (1000.f / frequency)) {
            _time = now;
            auto power = _powerBuf.pop();
            dP = power - _lastPower;
            auto absDp = std::abs(dP);
            if (absDp >= minPowerStep or absDp / _lastPower > 0.05f) {
                _lastPower = power;
                if (dP < 0)
                    _direction = !_direction;
            }

            // invalidate maxPower every 5min
            if (now - _timeMaxPower > 1000 * 60 * 5) {
                _maxPower = 0;
                ESP_LOGI("mppt", "Reset maxPower to 0");
            }

            // capture maxPower
            if (power > _maxPower * 1.01f) {
                _maxPower = power;
                _timeMaxPower = now;
                ESP_LOGI("mppt", "New maxPower %.2f", _maxPower);
            }


            //// slow-down if we are near 5% of maxPower
            //if (_maxPower > 1.f && (std::abs(power - _maxPower) / _maxPower) < 0.05f) {
            //    speed = .25f;
            //}
        } else {
            _powerBuf.add(powerSample);
            //speed = .05f;
        }

        float speed = 1.f;

        // if we didn't find a new maxPower for 15s, slow-down
        if (_maxPower > 1 && now - _timeMaxPower > 1000 * 60) {
            speed = .25;
            if(!prevSlow)
                ESP_LOGI("mppt", "Slow-down");
            frequency = 10;
            prevSlow = true;
        } else {
            frequency = 20;
            prevSlow = false;
        }

        return _direction ? speed : -speed;
    }
};