#pragma once

#include "adc.h"
#include "statmath.h"

template<class T>
struct ThreeChannelStruct {
    T chVin, chVout, chIin;
};

template<class T>
union ThreeChannelUnion {
    ThreeChannelStruct<T> s;
    T arr[3];

    inline const T &operator[](int i) const { return arr[i]; }

    inline T &operator[](int i) { return arr[i]; }

    //template<void T::* MemPtr>
    //template<typename T_arg0, void (T::*fn)(const T_arg0 &)>
    //void call(const T_arg0 &arg0) {
    //    for(auto & o : arr)
    //        fn(o, arg0);
    //}
};

struct ChannelTransform {
    uint8_t num;
    float factor;
    float midpoint;
};


/**
 * Implement
 * - ADC channel abstraction (for Vin, Vout, Iin)
 * - Channel cycling
 * - Zero current calibration
 * - Exponentially weighted moving average (EWMA) filtering
 */
class DCDC_PowerSampler {


    uint8_t cycleCh = 0;

    bool calibrating_ = false;
    unsigned long timeLastCalibration = 0;
    float calibZeroCurrent = 0;
    float calibVout = 0;

    uint16_t ewmSpan = 1;


public:

    AsyncADC<float> *adc = nullptr;

    std::function<void(const DCDC_PowerSampler &dcdc, uint8_t)> onDataChange = nullptr;

    const ThreeChannelUnion<ChannelTransform> channels;
    ThreeChannelUnion<float> last{NAN, NAN, NAN};
    ThreeChannelUnion<float> previous{NAN, NAN, NAN};
    ThreeChannelUnion<uint32_t> numSamples{0, 0, 0};
    ThreeChannelUnion<EWM<float>> ewm{
            EWM<float>{1},
            EWM<float>{1},
            EWM<float>{1}};
    ThreeChannelUnion<RunningMedian3<float>> med3{{{}, {}, {}}};


    explicit DCDC_PowerSampler(const ThreeChannelUnion<ChannelTransform> &channels) :
            channels(channels) {
    }

    float getIoutSmooth(float conversionEff = 0.97f) const {
        return ewm.s.chIin.avg.get() * ewm.s.chVin.avg.get() * conversionEff / std::max(ewm.s.chVout.avg.get(), 2.f);
    }

    void _readNext() {
        adc->startReading(channels[cycleCh].num);
    }

    void begin(AsyncADC<float> *adc_, uint16_t ewmaSpan) {
        adc = adc_;
        //auto f = &EWM<float>::updateSpan;
        for (auto &o: ewm.arr)
            o.updateSpan(ewmaSpan);
        ewmSpan = ewmaSpan;
        _readNext();
    }

    void startCalibration() {
        if (!calibrating_)
            ESP_LOGI("mppt", "Start calibration");
        // TODO use mean average, not EWM!
        // - reset mean here
        // consider: peak2peak values, emipiral uncertainty (higher stddev->need more samples)
        calibrating_ = true;
        calibZeroCurrent = 0;
        for (auto i = 0; i < 3; ++i) {
            numSamples[i] = 0;
            ewm.arr[i].reset();
        }
    }

    bool update() {
        if (adc->hasData()) {
            auto v = (adc->getSample() - channels[cycleCh].midpoint) * channels[cycleCh].factor;
            //bool changed = (last[cycleCh] != v);
            if (&last[cycleCh] == &last.s.chIin) {
                v -= calibZeroCurrent;
                if (std::max(last.s.chVin, last.s.chVout) < 10.f) {
                    v = 0; // current sensor needs at 5V
                }
            }
            previous[cycleCh] = last[cycleCh];
            last[cycleCh] = v;
            ewm[cycleCh].add(med3[cycleCh].next(v));
            ++numSamples[cycleCh];
            cycleCh = (cycleCh + 1) % 3;

            _readNext(); // start async read

            if (onDataChange) { // changed &&
                onDataChange(*this, cycleCh);
            }



            /*if (std::max(last.s.chVin, last.s.chVout) < 10.f) {
                if (!calibrating_)
                    ESP_LOGW("dcdc", "Supply under-voltage!");
                startCalibration();
            } else **/ if (calibrating_ && numSamples[cycleCh] > ewmSpan * 2) {
                calibZeroCurrent = ewm.s.chIin.avg.get();
                ESP_LOGI("dcdc", "Zero Current Calibration avg=%.4f std=%.6f", calibZeroCurrent, ewm.s.chIin.std.get());

                calibVout = ewm.s.chVout.avg.get();
                ESP_LOGI("dcdc", "V_out avg=%.4f std=%.6f", calibVout, ewm.s.chVout.std.get());
                ESP_LOGI("dcdc", "V_in avg=%.4f std=%.6f", ewm.s.chVin.avg.get(), ewm.s.chVin.std.get());

                auto vOut_std = std::fabs(ewm.s.chVout.std.get() * ewm.s.chVout.avg.get());

                // TODO peak2peak

                if (std::fabs(calibZeroCurrent) > 0.5f) {
                    ESP_LOGE("dcdc", "Zero Current too high %.2f", calibZeroCurrent);
                    startCalibration();
                } else if (ewm.s.chIin.std.get() > 0.1) { // 0.001
                    ESP_LOGE("dcdc", "Zero Current stddev too high %.5f", ewm.s.chIin.std.get());
                    startCalibration();
                } else if (vOut_std > 0.05f) {
                    ESP_LOGE("dcdc", "Zero Current Vout std too high %.2f V", vOut_std);
                    startCalibration();
                } else {
                    calibrating_ = false;
                    timeLastCalibration = millis();
                }
            }

            return true;
        }
        return false;
    }

    bool isCalibrating() const {
        return calibrating_;
    }

    float getBatteryIdleVoltage() const {
        return calibVout;
    }

    unsigned getTimeLastCalibration() const { return timeLastCalibration; }
};
