#pragma once

#include "adc.h"
#include "statmath.h"

template<class T> struct ThreeChannelStruct {T chVin, chVout, chIin;};

template<class T> union ThreeChannelUnion {
    ThreeChannelStruct<T> s;
    T arr[3];
    inline const T &operator[](int i) const {return arr[i];}
    inline T &operator[](int i)  {return arr[i];}
};

struct ChannelAndFactor {
    uint8_t num;
    float factor;
    float midpoint;
};


class DCDC_PowerSampler {
    AsyncADC<float> &adc;

    uint8_t cycleCh = 0;

public:
    const ThreeChannelUnion<ChannelAndFactor> channels;
    ThreeChannelUnion<float> last{NAN, NAN, NAN};
    ThreeChannelUnion<uint32_t> numSamples{0,0,0};
    ThreeChannelUnion<EWM<float>> ewm {
        EWM<float>{20},
        EWM<float>{20},
        EWM<float>{20}};


    DCDC_PowerSampler(AsyncADC<float> &adc, const ThreeChannelUnion<ChannelAndFactor> &channels) :
            adc(adc),
            channels(channels) {
    }

    void begin() {
        adc.startReading(channels[cycleCh].num);
    }

    bool update() {
        if(adc.hasData()) {
            auto v = (adc.getSample() - channels[cycleCh].midpoint) * channels[cycleCh].factor;
            last[cycleCh] = v;
            ewm[cycleCh].add(v);
            ++numSamples[cycleCh];
            cycleCh = (cycleCh+1) %3;
            begin();
            return true;
        }
        return false;
    }
};

/*

 */