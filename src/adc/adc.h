#pragma once

#include <cstdlib>

#include <driver/adc.h>
#include <esp_adc_cal.h>

template<class T>
class AsyncADC {
public:
    virtual bool init() = 0;

    virtual void startReading(uint8_t channel) = 0;

    virtual bool hasData() = 0;

    virtual T getSample() = 0;
    //virtual uint8_t getReadingChannel() = 0;

    virtual void setMaxExpectedVoltage(uint8_t ch, float voltage) = 0;

    virtual float getInputImpedance(uint8_t ch) = 0;

};