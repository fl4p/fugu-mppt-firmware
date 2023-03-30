#pragma once

#include "ads.h"
#include "statmath.h"

class MpptSampler {
    AsyncADC<float> &adc;

    EWM<float> Vin{20}, Vout{20}, Iin{20}, Iout{20};

    explicit MpptSampler(AsyncADC<float> &adc) : adc(adc) {
        //uint8_t ch_Vin =
    }

    void update() {
        if(adc.hasData()) {
            adc.getSample();
        }
    }
};