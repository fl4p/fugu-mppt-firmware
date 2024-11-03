#pragma once

#include "math/statmath.h"

struct MPP {
    float power = 0;
    unsigned long timestamp = 0;
    uint16_t dutyCycle = 0;
    //float voltage = 0;
};


/**
 * Implement local MPP tracking perturb / observe.
 * If it doesn't find a new MPP for some time, it'll switch into slow mode.
 * Slow mode uses longer smoothing filters, a lower update frequency and a slower perturbation speed
 */
struct Tracker {
    float minPowerStep = 1.5f;
    float frequency = 20;

    bool _direction = false; //true => increase duty cycle/decrease solar voltage
    unsigned long _time = 0;
    unsigned long _timeLastReverse = 0;

    float _lastPower = 0.0;


    EWMA<float> avgPower{200};

    MeanAccumulator _powerBuf{};

    //RunningMedian3<float> med3{};
    //bool _sweeping = false;
    //unsigned long _lastSweepTime = 0;

    MPP maxPowerPoint;

    std::array<EWMA_N<80>, 2048> pwmPowerTable{};
    std::array<unsigned long, 2048> pwmTimeTable{};

    float dP = NAN;

    bool slowMode = false;

    void resetTracker(float power, bool direction) {
        resetDirection(direction);
        _lastPower = power;
        maxPowerPoint.power = 0;
        avgPower.reset();
    }

    void resetDirection(bool direction) {
        _direction = direction;
        _time = loopWallClockMs();
        //maxPowerPoint.power = 0;
        //avgPower.reset();
    }

    float update(float powerSample, uint16_t dutyCycle) {
        auto now = loopWallClockMs();

        avgPower.add(powerSample);

        _powerBuf.add(slowMode ? avgPower.get() : powerSample);


        if (now - pwmTimeTable[dutyCycle] > 1000 * 60) {
            pwmPowerTable[dutyCycle].reset();
        }
        pwmTimeTable[dutyCycle] = now;
        pwmPowerTable[dutyCycle].add(powerSample);

        if ((float) (now - _time) > (1000.f / frequency)) {
            _time = now;
            auto power = _powerBuf.pop();
            //auto power = pwmPowerTable[dutyCycle].get();
            dP = power - _lastPower;

            if (power < 1) {
                _direction = true; // pump more power
            } else {
                auto absDp = std::abs(dP);
                if ((absDp >= minPowerStep or (absDp / _lastPower > 0.1f))
                    && (!slowMode || (now - _timeLastReverse) > 6000)) {
                    _lastPower = power;
                    if (dP < 0) {
                        _direction = !_direction;
                        _timeLastReverse = now;
                    }
                }
            }

            // TODO does this help?:
            if (power > _lastPower)
                _lastPower = power;

            // invalidate MPP every 5min ..
            if (now - maxPowerPoint.timestamp > 1000 * 60 * 5 and maxPowerPoint.power > 0) {
                maxPowerPoint.power = 0;
                ESP_LOGI("mppt", "Reset maxPower to 0");
            }

            // .. or if cur power is at 85%
            if (power < maxPowerPoint.power * 0.85f && now - maxPowerPoint.timestamp > 1000 * 30 and
                maxPowerPoint.power > 0) {
                maxPowerPoint.power = 0;
                ESP_LOGI("mppt", "Reset maxPower to 0 (<%.3f * 90%%)", power);
            }

            // capture MPP
            if (power > maxPowerPoint.power * 1.005f) {
                maxPowerPoint.power = power;
                maxPowerPoint.timestamp = now;
                maxPowerPoint.dutyCycle = dutyCycle;
                //ESP_LOGI("mppt", "New maxPower %.2f", maxPowerPoint.power);
            }


            //// slow-down if we are near 5% of maxPower
            //if (maxPowerPoint.power > 1.f && (std::abs(power - maxPowerPoint.power) / maxPowerPoint.power) < 0.05f) {
            //    speed = .25f;
            //}
        } else {

            //speed = .05f;
        }

        float speed = 1.f;

        // slow-mode condition:
        if (powerSample > 1
            and now - maxPowerPoint.timestamp > 1000 * 15 // if we didn't find a new maxPower for 15s
            and now - _timeLastReverse < 1000 * 15 // if we move in one direction for more than 15s, don't slow down
                ) {
            speed = .1; // 0.02
            frequency = 1;
            minPowerStep = 1.5f; // 0.75 is too small
            if (!slowMode) {
                ESP_LOGI("mppt", "Near MPP, slow-down, set dutyCycle %hu", maxPowerPoint.dutyCycle);
                slowMode = true;
                avgPower.reset();
                // out power measurement with hall sensor is poor and non-linear, so in slow-mode we might stay in a
                // local maximum. this work-around captures MPP duty cycle during normal mode
                return (float) maxPowerPoint.dutyCycle - (float) dutyCycle;
            }
        } else {
            // normal-mode
            speed = .5;
            frequency = 10;
            minPowerStep = 1.5f;
            slowMode = false;
            //avgPower.reset();
        }

        return _direction ? speed : -speed;
    }
};

/**

template<typename T, typename I>
class AdaptiveEWMATable {
    static constexpr auto stdSpan = 200;

    std::array<float, 2048> avg{};
    std::array<unsigned long, 2048> time{};

    float alpha;

    PctChange<float> pctPower{};
    EWMA<float> stdPower{stdSpan};

    unsigned int num = 0;

    void add(T x, I index) {

        if unlikely(isnan(x)) return;

        auto now = millis();

        stdPower.add(pctPower.next(x));

        if (num % stdSpan == 0) {
            // central limit theorem uncertainty to update ewm span
            float ns = stdPower.get() / 0.005;
            auto newSpan = constrain(ns * ns, 40, 400);
            alpha = (2.f / (newSpan + 1.f));
            ESP_LOGI("track", "Power std=%.6f, newSpan=%.1f, alpha=%.3f", stdPower.get(), newSpan, alpha);
        }

        auto &y(avg[index]);

        if (now - time[index] > 1000 * 60) y = NAN; // invalidate old
        if unlikely(isnan(y)) y = x; // reset
        y = (1 - alpha) * y + alpha * x; // ewm update
        time[index] = now;


        ++num;
    }
};

*/