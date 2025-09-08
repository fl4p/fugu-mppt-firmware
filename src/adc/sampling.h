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
#include "util.h"
#include "etc/rt.h"

#include "tele/scope.h"
#include "math/notch.h"
#include "math/noise.h"


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

    bool calibrateOffset;
};

/*struct _SensorCalibrationState {
    std::vector<float> acc{};
    std::vector<uint32_t> num {};
};*/

struct SensorParams {
    const uint8_t adcCh;
    const LinearTransform transform;
    //uint16_t ewmaSpan;
    CalibrationConstraints calibrationConstraints;
    const std::string teleName;
    char unit;
    /**
     * Whether to capture each ADC conversion. Can produce a lot of networking data
     */
    bool rawTelemetry;
    //uint16_t filtLen;
};

struct Sensor {
    const SensorParams params;

    float last = NAN, lastRaw = NAN;
    float previous = NAN;
    uint32_t numSamples = 0;
    NotchFilter *notchFilter = nullptr; // filter 50/60 Hz inverter noise
    RunningMedian5<float> med3{}; // filter burst noise
    AdaptiveNoiseFilter anf{};
    EWM<false, float> ewm; // filter residual noise
    MeanAccumulator calibBuffer{}; // for offset calibration

    float calibrationAvg = 0; // stores the mean value from calibBuffer

    const bool isVirtual;

    /*
    Sensor(SensorParams params, uint32_t ewmSpan)
            : params(std::move(params)), ewm{ewmSpan}, isVirtual(false) {
        anf.begin(ewmSpan, 0.2f);
    } */


    Sensor(const Sensor &other) = delete; // no-copy
    Sensor &operator=(const Sensor &) = delete; // no-copy

    void reset(bool resetCalibration = false) {
        last = NAN;
        previous = NAN;
        numSamples = 0;
        med3.reset();
        ewm.reset();
        if (resetCalibration && calibrationAvg != 0) {
            ESP_LOGI("sensor", "%s reset calibration", params.teleName.c_str());
            calibrationAvg = 0;
            calibBuffer.clear();
        }
    }

    void add_sample(float x) { // IRAM_ATTR
        auto v = params.transform.apply(x);

        //if( !isfinite(v)) {
        //    ESP_LOGW("s", "not-finite sensor val %f (name=%s, x=%f) ", v, params.teleName.c_str(), x);
        //}

        if (params.calibrationConstraints.calibrateOffset) {
            //ESP_LOGI("sensor", "%s %.4f offset=%.4f std=%.4f n=%u", params.teleName.c_str(), v, calibrationAvg,
            //         ewm.std.get(), numSamples);
            v -= calibrationAvg;
        }

        previous = last;
        last = v;
        lastRaw = x;

        if (notchFilter)notchFilter->filter(&last, &v, 1);
        v = med3.next(v);
        ewm.add(v);
        anf.add(v);

        ++numSamples;

        //ESP_LOGD("s", "Sensor %s: add sample %.5f #%d (ewm avg %.5f)", teleName.c_str(), last, numSamples, ewm.avg.get());
    }

protected:
    Sensor(SensorParams params, uint32_t ewmSpan, bool isVirtual)
            : params(std::move(params)), ewm{ewmSpan}, isVirtual{isVirtual} {
        anf.begin(ewmSpan, 0.2f);
    }

    //virtual ~Sensor() = default;  // make class polymorphic (to enable dynamic_cast)
};

struct PhysicalSensor : public Sensor {
    AsyncADC<float> *adc{nullptr};

    explicit PhysicalSensor(AsyncADC<float> *adc, const SensorParams &params, uint32_t ewmSpan)
            : Sensor(params, ewmSpan, false),
              adc(adc) {
    }

    void createNotchFilter() {
        try {
            auto fs = adc->getSamplingRate(params.adcCh);
            notchFilter = new NotchFilter();
            const auto inverterFreq = 50.0f;
            const auto inverterInputFreq = 2*50.0f; // abs(sin(t))
            notchFilter->begin(inverterInputFreq / fs);
            ESP_LOGI("sampling", "%s notch filter fs=%.1fHz f0=%.1fHz", params.teleName.c_str(),fs, inverterInputFreq);
        } catch (const std::exception &ex) {
            ESP_LOGE("sampling", "error %s", ex.what());
        }
    }
};

struct VirtualSensor : public Sensor {
    std::function<float()> func;

    explicit VirtualSensor(std::function<float()> func, uint32_t ewmSpan, const char *teleName, char unit)
            : Sensor({255, {1, 0}, {}, teleName, unit, false}, ewmSpan, true),
              func(std::move(func)) {
    }
};

template<typename T>
struct VIinVout {
    T Vin{}, Vout{}, Iin{}, Iout{};

    VIinVout(const VIinVout &) = delete;

    VIinVout(const VIinVout &&) = delete;

    VIinVout(T vin, T vout, T iin, T iout) : Vin{vin}, Vout{vout}, Iin{iin}, Iout{iout} {}
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
public:
    bool ignoreCalibrationConstraints = false;// for testing

    //typedef Sensor Sensor;
    //using Sensor = Sensor;

private:
    struct AdcState {
        AsyncADC<float> *adc{nullptr};
        std::array<Sensor *, 8> sensorByCh{nullptr};
        uint8_t cycleCh = 0;
    };

    uint8_t calibrating_ = 0;
    unsigned long timeLastCalibration = 0;


public:

    volatile bool halted = false;


    std::function<void(const ADC_Sampler &sampler, const Sensor &)> onNewSample = nullptr;

    std::vector<AdcState> adcStates{};
    std::vector<Sensor *> sensors{};
    std::vector<Sensor *> realSensors{};
    std::vector<VirtualSensor *> virtualSensors{};


    AdcState &getAdcState(AsyncADC<float> *adc) {
        for (auto &s: adcStates)
            if (s.adc == adc)
                return s;
        return adcStates.emplace_back(AdcState{.adc = adc});
    }

    /**
     * Add a sensor with a given transform, a max expected value for ADC ranging and a name used for telemetry
     *
     * @param adcChannel ADC channel to read samples from
     * @param transform Transform applied to the samples
     * @param maxY Max expected value of the transformed sample (used to program the ADC PGA)
     */
    const Sensor *addSensor(AsyncADC<float> *adc, SensorParams params, float maxY, uint32_t ewmSpan) {
        assert(adc != nullptr);
        auto maxX = params.transform.apply_inverse(params.transform.factor < 0 ? -maxY : maxY);
        ESP_LOGI("sampler", "%s ADC ch %hhu maxY=%.4f, maxX=%.4f", params.teleName.c_str(), params.adcCh, maxY, maxX);
        adc->setMaxExpectedVoltage(params.adcCh, maxX);

        auto &sensorByCh(getAdcState(adc).sensorByCh);

        if (adc->scheme() == SampleReadScheme::any)
            assert_throw(sensorByCh[params.adcCh] == nullptr, "duplicate sensor adc channel");

        auto sensorPtr = new PhysicalSensor{adc, params, ewmSpan};

        // todo scope:  adc->getSamplingRate() needs to be called after adc->setMaxExpectedVoltage()
        if (scope)
            scope->addChannel(adc, sensorPtr->params.adcCh, 'u', 12, sensorPtr->params.teleName.c_str());

        sensors.push_back(sensorPtr);
        realSensors.push_back(sensorPtr);
        if (!sensorByCh[sensorPtr->params.adcCh])
            sensorByCh[sensorPtr->params.adcCh] = sensorPtr;

        return sensorPtr;
    }

    const Sensor *addVirtualSensor(std::function<float()> func, uint32_t ewmaSpan, const char *teleName, char unit) {
        virtualSensors.push_back(new VirtualSensor{std::move(func), ewmaSpan, teleName, unit});
        sensors.push_back(virtualSensors.back());
        return sensors.back();
    }

    void _readNext(AdcState &state) {
        auto r = std::find_if(state.sensorByCh.begin(), state.sensorByCh.end(), [](Sensor *s) { return !!s; });
        assert(r != state.sensorByCh.end());
        state.adc->startReading((*r)->params.adcCh);
    }


    /**
     * user must call this from the same task that perform ADC reading (calls hasData & getSample)
     */
    void begin() {
        assert_throw (!adcStates.empty(), "adc null");
        assert_throw(!realSensors.empty(), "");

        for (auto &s: adcStates) {
            s.adc->start();
            if (s.adc->scheme() != SampleReadScheme::any)
                _readNext(s);
        }

        for(auto &s:sensors) {
            if(s->isVirtual) continue;
            auto ps = (PhysicalSensor*)s;
            ps->createNotchFilter();
        }
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
            if (!ch->isVirtual)
                static_cast<PhysicalSensor *>(ch)->adc->reset(ch->params.adcCh);
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

    enum class UpdateRet : uint8_t {
        NoNewData = 0,
        NewData,
        Calibrating,
        CalibFailure,
    };

    UpdateRet handleSensorCalib(Sensor &sensor) {
        if (calibrating_ && sensor.numSamples >= 100) {
            // calibZeroCurrent = ewm.s.chIin.avg.get();
            sensor.calibBuffer.add(sensor.last);

            if (sensor.calibBuffer.num > 100) {
                auto avg = sensor.calibBuffer.pop(); //sensor.ewm.avg.get();
                auto std = sensor.ewm.std.get();

                auto &constrains{sensor.params.calibrationConstraints};

                if (!ignoreCalibrationConstraints && (!std::isfinite(avg) or std::fabs(avg) > constrains.maxAbsValue)) {
                    ESP_LOGE("sampler", "Calibration failed, %s abs value %.6f > %.6f (last=%.6f, stdn=%.6f)",
                             sensor.params.teleName.c_str(), std::fabs(avg), constrains.maxAbsValue, sensor.last, std);
                    calibrating_ = 0;
                    //startCalibration();
                    return UpdateRet::CalibFailure;
                }

                if (!ignoreCalibrationConstraints &&
                    (std::abs(avg) < 1e-9f or std::isfinite(std)) // std can be non-finite for 0 values
                    and std * std::abs(avg) > constrains.maxStddev) {
                    ESP_LOGE("sampler", "Calibration failed, %s stddev %.6f > %.6f (last=%.6f, avg=%.6f)",
                             sensor.params.teleName.c_str(),
                             std * avg,
                             constrains.maxStddev, sensor.last, avg);
                    ESP_LOGW("sampler", "%s last=%.6f med3=%.6f avg=%.6f num=%lu", sensor.params.teleName.c_str(),
                             sensor.last,
                             sensor.med3.get(), sensor.ewm.avg.get(), sensor.numSamples);
                    calibrating_ = 0;
                    //startCalibration();
                    return UpdateRet::CalibFailure;
                }

                // TODO peak2peak

                sensor.calibrationAvg = avg;
                sensor.reset(false);

                ESP_LOGI("sampler", "Sensor %s calibration: avg=%.4f std=%.6f", sensor.params.teleName.c_str(), avg,
                         std);

                if (sensor.params.calibrationConstraints.calibrateOffset)
                    ESP_LOGI("sampler", "Sensor %s offset-calibrated: %.6f", sensor.params.teleName.c_str(), avg);

                --calibrating_;

                assert(calibrating_ < realSensors.size());

                if (calibrating_ == 0) {
                    ESP_LOGI("sampler", "Calibration done!");
                    timeLastCalibration = wallClockUs();
                    return UpdateRet::Calibrating;
                }
            }
        }

        return UpdateRet::NoNewData;
    }

    UpdateRet _addSensorSample(Sensor *sensor, float v) {
        if (unlikely(isnan(v)))
            return UpdateRet::CalibFailure;

        sensor->add_sample(v);
        rtcount("adc.update.addSample");

        if (onNewSample) {
            onNewSample(*this, *sensor);
            rtcount("adc.update.onNewSample");
        }

        auto calibRes = handleSensorCalib(*sensor);
        rtcount("adc.update.handleSensorCalib");
        return calibRes;
    }


    UpdateRet _updateAdc(AdcState &state) {
        auto adc = state.adc;

        auto hd = adc->hasData();
        rtcount("adc.update.hasData");
        if (!hd)
            return UpdateRet::NoNewData;


        auto scheme = adc->scheme();
        UpdateRet calibRes;

        if (scheme == SampleReadScheme::any) {
            calibRes = UpdateRet::NoNewData;
            adc->read([&](uint8_t ch, float v) {
                auto cr = _addSensorSample(state.sensorByCh[ch], v);
                if (cr > calibRes) calibRes = cr;
            });
            rtcount("adc.update.read");
        } else if (scheme == SampleReadScheme::all) {
            calibRes = UpdateRet::NoNewData;
            for (int i = 0; i < state.sensorByCh.size(); ++i) {
                auto sensor = state.sensorByCh[i];
                if (!sensor) continue;
                adc->startReading(i);
                rtcount("adc.update.startReading");

                auto x = adc->getSample();
                rtcount("adc.update.getSample");

                auto cr = _addSensorSample(sensor, x);
                if (cr > calibRes) calibRes = cr;
            }
        } else {
            assert_throw(false, "not implemented");

            auto sensor(realSensors[state.cycleCh]);

            auto x = adc->getSample();
            rtcount("adc.update.getSample");

            state.cycleCh = (state.cycleCh + 1) % realSensors.size();
            _readNext(state); // start async read
            rtcount("adc.update.startReading");

            calibRes = _addSensorSample(sensor, x);
        }

        if (calibRes != UpdateRet::NoNewData)
            return calibRes;

        /*if(calibrationState) {
            calibrationState->acc[cycleCh] += sensor.last;
            ++calibrationState->num[cycleCh];
        }*/


        return calibrating_ == 0 ? UpdateRet::NewData : UpdateRet::Calibrating;
    }

    UpdateRet update() {
        UpdateRet res = UpdateRet::NoNewData;

        if (unlikely(halted)) {
            vTaskDelay(10);
            return res;
        }

        bool updateVirtual = false;
        for (auto &state: adcStates) {
            auto r = _updateAdc(state);
            if (r > res) res = r;
            if (state.cycleCh == 0) updateVirtual = true;
        }

        // update virtual sensors
        // TODO virtual sensor calibration?
        if (calibrating_ == 0 && updateVirtual) {
            for (auto &sn: virtualSensors) {
                sn->add_sample(sn->func());
                rtcount("adc.update.AddSampleVirtual");
            }
        }

        return res;
    }


    [[nodiscard]] bool isCalibrating() const { return calibrating_ > 0; }

    [[nodiscard]] unsigned long getTimeLastCalibrationUs() const { return timeLastCalibration; }


    void reInitADCs() {
        ConfFile pinConf{"/littlefs/conf/pins.conf"};
        for (auto &s: adcStates) {
            s.adc->deinit();
            s.adc->init(pinConf);
            s.adc->start();
        }
    }

    bool resetPeripherals() {
        bool ok = true;
        for (auto &s: adcStates) {
            ok = s.adc->resetPeripherals() and ok;
        }
        return ok;
    }

};
