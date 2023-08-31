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

    //const static LinearTransform Identity;
};

struct CalibrationConstraints {
    float maxAbsValue;
    float maxStddev;

    bool calibrateMidpoint;
};

template<typename T>
struct VIinVout {
    T Vin, Vout, Iin, Iout;
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


public:

    AsyncADC<float> *adc = nullptr;

    struct SensorParams {
        const uint8_t adcCh;
        const LinearTransform transform;
        //uint16_t ewmaSpan;
        const CalibrationConstraints calibrationConstraints;
        const std::string teleName;
        /**
         * Whether to capture each ADC conversion. Can produce a lot of networking data
         */
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

        const bool isVirtual;

        Sensor(SensorParams params, uint32_t ewmSpan)
                : params(std::move(params)), ewm{ewmSpan}, isVirtual(false) {}


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

    protected:
        Sensor(SensorParams params, uint32_t ewmSpan, bool isVirtual)
                : params(std::move(params)), ewm{ewmSpan} , isVirtual{isVirtual}{}

        //virtual ~Sensor() = default;  // make class polymorphic (to enable dynamic_cast)
    };

    struct VirtualSensor : public Sensor {
        std::function<float()> func;

        explicit VirtualSensor(std::function<float()> func, uint32_t ewmSpan)
                : Sensor({0, {1, 0}, {}, "", false}, ewmSpan, true),
                  func(std::move(func)) {
        }
    };

    /*struct _SensorCalibrationState {
        std::vector<float> acc{};
        std::vector<uint32_t> num {};
    };*/

    std::function<void(const ADC_Sampler &sampler, const Sensor &)> onNewSample = nullptr;
    std::vector<Sensor *> sensors{};
    std::vector<Sensor *> realSensors{};
    std::vector<VirtualSensor *> virtualSensors{};

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
    const Sensor *addSensor(SensorParams params, float maxY, uint32_t ewmSpan) {
        assert(adc != nullptr);
        auto maxX = params.transform.apply_inverse(params.transform.factor < 0 ? -maxY : maxY);
        ESP_LOGI("sampler", "%s ADC ch %hhu maxY=%.4f, maxX=%.4f", params.teleName.c_str(), params.adcCh, maxY, maxX);
        adc->setMaxExpectedVoltage(params.adcCh, maxX);

        sensors.push_back(new Sensor{std::move(params), ewmSpan});
        realSensors.push_back(sensors.back());
        return sensors.back();
    }

    const Sensor *addVirtualSensor(std::function<float()> func, uint32_t ewmaSpan) {
        virtualSensors.push_back(new VirtualSensor{std::move(func), ewmaSpan});
        sensors.push_back(virtualSensors.back());
        return sensors.back();
    }

    void _readNext() {
        adc->startReading(realSensors[cycleCh]->params.adcCh);
    }


    void begin() {
        assert(adc != nullptr);
        assert(!realSensors.empty());
        _readNext();
    }

    void startCalibration() {
        if (!calibrating_)
            ESP_LOGI("mppt", "Start calibration");
        // TODO use mean average, not EWM!
        // - reset mean here
        // consider: peak2peak values, empirical uncertainty (higher stddev->need more samples)
        calibrating_ = realSensors.size();
        for (auto &ch: sensors) {
            ch->reset(true);
        }
    }

    void cancelCalibration() {
        if (!calibrating_)
            return;
        ESP_LOGI("mppt", "Cancel calibration");
        calibrating_ = 0;
        for (auto &ch: sensors) {
            ch->reset(false);
        }
    }

    bool update() {
        if (!adc->hasData())
            return false;

        auto &sensor(*realSensors[cycleCh]);
        auto x = adc->getSample();
        cycleCh = (cycleCh + 1) % realSensors.size();
        _readNext(); // start async read

        sensor.add_sample(x);
        if (onNewSample) {
            onNewSample(*this, sensor);
        }

        /*if(calibrationState) {
            calibrationState->acc[cycleCh] += sensor.last;
            ++calibrationState->num[cycleCh];
        }*/


        if (calibrating_ && sensor.numSamples >= 60) {
            // calibZeroCurrent = ewm.s.chIin.avg.get();
            auto avg = sensor.ewm.avg.get();
            auto std = sensor.ewm.std.get();

            auto &constrains{sensor.params.calibrationConstraints};

            if (!std::isfinite(avg) or std::fabs(avg) > constrains.maxAbsValue) {
                ESP_LOGE("sampler", "Calibration failed, %s abs value %.6f > %.6f (last=%.6f, x=%.6f, stdn=%.6f)",
                         sensor.params.teleName.c_str(), std::fabs(avg), constrains.maxAbsValue, sensor.last, x, std);
                startCalibration();
                return false;
            }

            if ( /*!std::isfinite(std) or */ std * avg > constrains.maxStddev) {
                ESP_LOGE("sampler", "Calibration failed, %s stddev %.6f > %.6f (last=%.6f, x=%.6f, avg=%.6f)", sensor.params.teleName.c_str(),
                         std * avg,
                         constrains.maxStddev, sensor.last, x, avg);
                ESP_LOGW("sampler", "%s last=%.6f med3=%.6f avg=%.6f num=%u", sensor.params.teleName.c_str(),
                         sensor.last,
                         sensor.med3.get(), sensor.ewm.avg.get(), sensor.numSamples);
                startCalibration();
                return false;
            }

            // TODO peak2peak

            sensor.calibrationAvg = avg;

            ESP_LOGI("sampler", "Sensor %s calibration: avg=%.4f std=%.6f", sensor.params.teleName.c_str(), avg, std);
            if (sensor.params.calibrationConstraints.calibrateMidpoint)
                ESP_LOGI("sampler", "Sensor %s midpoint-calibrated: %.6f", sensor.params.teleName.c_str(), avg);

            --calibrating_;

            assert(calibrating_ < realSensors.size());

            if (calibrating_ == 0) {
                ESP_LOGI("sampler", "Calibration done!");
                timeLastCalibration = millis();
                return true;
            }
        }

        if (calibrating_ == 0 && cycleCh == 0) {
            for (auto &sn: virtualSensors) {
                sn->add_sample(sn->func());
            }
        }

        return calibrating_ == 0;
    }

    bool isCalibrating() const { return calibrating_ > 0; }

    unsigned getTimeLastCalibration() const { return timeLastCalibration; }

};
