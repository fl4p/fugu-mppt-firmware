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
};

struct ChannelAndFactor {
    uint8_t num;
    float factor;
    float midpoint;
};


class DCDC_PowerSampler {
    AsyncADC<float> &adc;

    uint8_t cycleCh = 0;

    bool calibrating_ = false;
    float calibZeroCurrent = 0;
    float calibVout = 0;


public:
    static constexpr uint8_t EWM_SPAN = 5; // 20
    static constexpr uint8_t EWM_SPAN_V = 5;

    std::function<void(const DCDC_PowerSampler &dcdc, uint8_t)> onDataChange = nullptr;

    const ThreeChannelUnion<ChannelAndFactor> channels;
    ThreeChannelUnion<float> last{NAN, NAN, NAN};
    ThreeChannelUnion<float> previous{NAN, NAN, NAN};
    ThreeChannelUnion<uint32_t> numSamples{0, 0, 0};
    ThreeChannelUnion<EWM<float>> ewm{
            EWM<float>{EWM_SPAN_V},
            EWM<float>{EWM_SPAN_V},
            EWM<float>{EWM_SPAN}};


    DCDC_PowerSampler(AsyncADC<float> &adc, const ThreeChannelUnion<ChannelAndFactor> &channels) :
            adc(adc),
            channels(channels) {
    }

    void begin() {
        adc.startReading(channels[cycleCh].num);
    }

    void startCalibration() {
        calibrating_ = true;
        for (auto i = 0; i < 3; ++i)
            numSamples[i] = 0;
    }

    bool update() {
        if (adc.hasData()) {
            auto v = (adc.getSample() - channels[cycleCh].midpoint) * channels[cycleCh].factor;
            //bool changed = (last[cycleCh] != v);
            if (&last[cycleCh] == &last.s.chIin) {
                v -= calibZeroCurrent;
            }
            previous[cycleCh] = last[cycleCh];
            last[cycleCh] = v;
            ewm[cycleCh].add(v);
            ++numSamples[cycleCh];
            cycleCh = (cycleCh + 1) % 3;
            begin();

            if (onDataChange) { // changed &&
                onDataChange(*this, cycleCh);
            }

            if (calibrating_ && numSamples[cycleCh] > std::max(EWM_SPAN,EWM_SPAN_V) * 2) {
                calibZeroCurrent = ewm.s.chIin.avg.get();
                ESP_LOGI("dcdc", "Zero Current Calibration avg=%.4f std=%.6f", calibZeroCurrent, ewm.s.chIin.std.get());

                calibVout = ewm.s.chVout.avg.get();
                ESP_LOGI("dcdc", "V_out avg=%.4f std=%.6f", calibVout, ewm.s.chVout.std.get());

                if (std::fabs(calibZeroCurrent) > 0.5f) {
                    ESP_LOGE("dcdc", "Zero Current too high %.2f", calibZeroCurrent);
                    startCalibration();
                } else if (std::fabs(ewm.s.chVout.std.get()) > 10e-3f) {
                    ESP_LOGE("dcdc", "Zero Current Vout std too high %.2f", ewm.s.chVout.std.get());
                    startCalibration();
                } else {
                    calibrating_ = false;
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
};
