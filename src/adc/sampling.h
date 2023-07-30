#pragma once

//#include <utility>
#include <utility>
#include <vector>
#include <string>
#include <functional>

#include <esp_log.h>

#include <Arduino.h>

#include "adc.h"
#include "math/statmath.h"


struct LinearTransform {
    float factor;
    float midpoint;

    inline float apply(float x) const { return (x - midpoint) * factor; }

    inline float apply_inverse(float y) const { return (y / factor) + midpoint; }
};

struct CalibrationConstraints {
    float maxAbsValue;
    float maxStddev;

    bool calibrateMidpoint;
};

template<typename T>
struct VIinVout {
    T Vin, Vout, Iin;
};

/**
 *
 * Implements asynchronous interleaved sampling of multiple sensors. Each sensor uses an ADC channel.
 *
 * Implement
 * - ADC channel abstraction (for Vin, Vout, Iin)
 * - Channel cycling
 * - Zero current calibration
 * - Exponentially weighted moving average (EWMA) filtering
 */
class ADC_Sampler {
    uint8_t cycleCh = 0;

    uint8_t calibrating_ = 0;
    unsigned long timeLastCalibration = 0;

    uint16_t ewmSpan = 1;

public:

    AsyncADC<float> *adc = nullptr;

    struct SensorParams {
        const uint8_t adcCh;
        const LinearTransform transform;
        const CalibrationConstraints calibrationConstraints;
        const std::string teleName;
        const bool rawTelemetry;
    };

    struct Sensor {
        const SensorParams params;

        float last = NAN;
        float previous = NAN;
        uint32_t numSamples = 0;
        RunningMedian3<float> med3{};
        EWM<float> ewm;

        float calibrationAvg = 0;

        Sensor(SensorParams params, uint32_t ewmSpan) : params(std::move(params)), ewm{ewmSpan} {}

        Sensor(const Sensor &other) = delete; // no-copy
        Sensor &operator=(const Sensor &) = delete; // no-copy

        void reset(bool resetCalibration = false) {
            last = NAN;
            previous = NAN;
            numSamples = 0;
            med3.reset();
            ewm.reset();
            if (resetCalibration)
                calibrationAvg = 0;
        }

        void add_sample(float x) {
            auto v = params.transform.apply(x);

            if (params.calibrationConstraints.calibrateMidpoint) {
                v -= calibrationAvg;
            }

            previous = last;
            last = v;
            ewm.add(med3.next(v));
            ++numSamples;

            //ESP_LOGD("s", "Sensor %s: add sample %.5f #%d (ewm avg %.5f)", teleName.c_str(), last, numSamples, ewm.avg.get());
        }
    };

    /*struct _SensorCalibrationState {
        std::vector<float> acc{};
        std::vector<uint32_t> num {};
    };*/

    std::function<void(const ADC_Sampler &sampler, const Sensor &)> onNewSample = nullptr;
    std::vector<Sensor *> channels{};

    //_SensorCalibrationState * calibrationState = nullptr;
    //explicit ADC_Sampler()  {}

    void setADC(AsyncADC<float> *adc_) {
        adc = adc_;
    }

    /**
     * Add a sensor with a given transform, a max expected value for ADC ranging and a name used for telemetry
     *
     * @param adcChannel ADC channel to read samples from
     * @param transform Transform applied to the samples
     * @param maxY Max expected value of the transformed sample (used to program the ADC PGA)
     */
    const Sensor *addSensor(SensorParams params, float maxY) {
        assert(adc != nullptr);
        auto maxX = params.transform.apply_inverse(params.transform.factor < 0 ? -maxY : maxY);
        ESP_LOGI("sampler", "%s ADC ch %hhu maxY=%.4f, maxX=%.4f", params.teleName.c_str(), params.adcCh, maxY, maxX);
        adc->setMaxExpectedVoltage(params.adcCh, maxX);

        channels.push_back(new Sensor{std::move(params), ewmSpan});
        channels.back()->ewm.updateSpan(ewmSpan);
        return channels.back();
    }

    void _readNext() {
        adc->startReading(channels[cycleCh]->params.adcCh);
    }


    void begin(uint16_t ewmaSpan) {
        assert(adc != nullptr);
        assert(!channels.empty());
        for (auto &ch: channels)
            ch->ewm.updateSpan(ewmaSpan);
        ewmSpan = ewmaSpan;
        _readNext();
    }

    void startCalibration() {
        if (!calibrating_)
            ESP_LOGI("mppt", "Start calibration");
        // TODO use mean average, not EWM!
        // - reset mean here
        // consider: peak2peak values, emipiral uncertainty (higher stddev->need more samples)
        calibrating_ = channels.size();
        for (auto &ch: channels) {
            ch->reset(true);
        }
    }

    void cancelCalibration() {
        if (!calibrating_)
            return;
        ESP_LOGI("mppt", "Cancel calibration");
        calibrating_ = 0;
        for (auto &ch: channels) {
            ch->reset(false);
        }
    }

    bool update() {
        if (!adc->hasData())
            return false;

        auto &ch(*channels[cycleCh]);
        auto x = adc->getSample();
        cycleCh = (cycleCh + 1) % channels.size();
        _readNext(); // start async read

        ch.add_sample(x);
        if (onNewSample) {
            onNewSample(*this, ch);
        }

        /*if(calibrationState) {
            calibrationState->acc[cycleCh] += ch.last;
            ++calibrationState->num[cycleCh];
        }*/


        if (calibrating_ && ch.numSamples >= std::max(3, ewmSpan * 2)) {
            // calibZeroCurrent = ewm.s.chIin.avg.get();
            auto avg = ch.ewm.avg.get();
            auto std = ch.ewm.std.get();

            auto &constrains{ch.params.calibrationConstraints};

            if (!std::isfinite(avg) or std::fabs(avg) > constrains.maxAbsValue) {
                ESP_LOGE("sampler", "Calibration failed, %s abs value %.6f > %.6f (last=%.6f, x=%.6f)",
                         ch.params.teleName.c_str(), std::fabs(avg), constrains.maxAbsValue, ch.last, x);
                startCalibration();
                return false;
            }

            if ( /*!std::isfinite(std) or */ std > constrains.maxStddev) {
                ESP_LOGE("sampler", "Calibration failed, %s stddev %.6f > %.6f", ch.params.teleName.c_str(), std,
                         constrains.maxStddev);
                startCalibration();
                return false;
            }

            // TODO peak2peak

            ch.calibrationAvg = avg;

            ESP_LOGI("sampler", "Sensor %s calibration: avg=%.4f std=%.6f", ch.params.teleName.c_str(), avg, std);
            if (ch.params.calibrationConstraints.calibrateMidpoint)
                ESP_LOGI("sampler", "Sensor %s midpoint-calibrated: %.6f", ch.params.teleName.c_str(), avg);

            --calibrating_;

            assert(calibrating_ < channels.size());

            if (calibrating_ == 0) {
                ESP_LOGI("sampler", "Calibration done!");
                timeLastCalibration = millis();
                return true;
            }
        }

        return calibrating_ == 0;
    }

    bool isCalibrating() const { return calibrating_ > 0; }

    unsigned getTimeLastCalibration() const { return timeLastCalibration; }
};
