#pragma once

#include <SPI.h> // not sure why this is needed
#include <Adafruit_ADS1X15.h>
#include "adc.h"
#include "pinconfig.h"

class ADC_ADS;

ADC_ADS *ads_inst = nullptr;

void ICACHE_RAM_ATTR AdsAlertISR();

/**
 * Implementation for ADS1x15 ADC. Uses ALERT pin for conversion ready notification interrupt.
 */
class ADC_ADS : public AsyncADC<float> {

    //Adafruit_ADS1115 ads; /* Use this for the 16-bit version */

    Adafruit_ADS1X15 ads;

    std::array<adsGain_t, 4> gainsByChannel{GAIN_ONE, GAIN_ONE, GAIN_ONE, GAIN_ONE};

    uint8_t readingChannel = 255;
    volatile bool newData = false;

public:

    explicit ADC_ADS(bool _16bit) {
        if(_16bit) {
            ads = Adafruit_ADS1115();
        } else {
            ads = Adafruit_ADS1015();
        }
    }

    bool init() override {

        if ((int) PinConfig::ADS_ALERT == 0) {
            return false;
        }

        if (!ads.begin(0x48)) {
            return false;
        }

        pinMode((uint8_t) PinConfig::ADS_ALERT, INPUT_PULLUP);

        if (ads_inst) {
            return false;
        }
        ads_inst = this;
        attachInterrupt(digitalPinToInterrupt((uint8_t) PinConfig::ADS_ALERT), AdsAlertISR, FALLING); // TODO rising

        //ads.setDataRate(RATE_ADS1115_860SPS); // this is for ADS1015 also! (130 sps). fake chips?
        // ads.setDataRate(RATE_ADS1015_3300SPS);
        //ads.setDataRate(RATE_ADS1015_3300SPS);
        return true;
    }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        adsGain_t g;
        assert(voltage <= 6.144f);
        if (voltage > 4.096f) g = GAIN_TWOTHIRDS;
        else if (voltage > 2.048f) g = GAIN_ONE;
        else if (voltage > 1.024f) g = GAIN_TWO;
        else if (voltage > 0.512f) g = GAIN_FOUR;
        else if (voltage > 0.256f) g = GAIN_EIGHT;
        else g = GAIN_SIXTEEN;
        setChannelGain(ch, g);
    }

    static float gainVoltageRange(adsGain_t gain) {
        float fsRange;
        switch (gain) {
            case GAIN_TWOTHIRDS:
                fsRange = 6.144f;
                break;
            case GAIN_ONE:
                fsRange = 4.096f;
                break;
            case GAIN_TWO:
                fsRange = 2.048f;
                break;
            case GAIN_FOUR:
                fsRange = 1.024f;
                break;
            case GAIN_EIGHT:
                fsRange = 0.512f;
                break;
            case GAIN_SIXTEEN:
                fsRange = 0.256f;
                break;
            default:
                fsRange = 0.0f;
        }
        return fsRange;
    }

    void setChannelGain(uint8_t channel, adsGain_t gain) {
        if (gain != gainsByChannel[channel])
            ESP_LOGI("ads", "Set channel %hhu gain %.3f -> %.3f", channel, gainVoltageRange(gainsByChannel[channel]),
                     gainVoltageRange(gain));
        gainsByChannel[channel] = gain;
    }

    void startReading(uint8_t channel) override {
        assert(readingChannel == 255);
        readingChannel = channel;
        ads.setGain(gainsByChannel[channel]);
        ads.startADCReading(MUX_BY_CHANNEL[channel], /*continuous=*/false);
    }


/*
    static float computeVolts(int16_t counts, adsGain_t gain) {
        constexpr uint8_t m_bitShift = 0;  // ads1115
        // see data sheet Table 3
        float fsRange;
        switch (gain) {
            case GAIN_TWOTHIRDS:
                fsRange = 6.144f;
                break;
            case GAIN_ONE:
                fsRange = 4.096f;
                break;
            case GAIN_TWO:
                fsRange = 2.048f;
                break;
            case GAIN_FOUR:
                fsRange = 1.024f;
                break;
            case GAIN_EIGHT:
                fsRange = 0.512f;
                break;
            case GAIN_SIXTEEN:
                fsRange = 0.256f;
                break;
            default:
                fsRange = 0.0f;
        }
        return (float) counts * (fsRange / (float) (32768 >> m_bitShift));
    }
    */

    inline void alertNewDataFromISR() { newData = true; }

    inline bool hasData() override { return newData; }

    float getSample() override {
        // TODO detect clipping
        int16_t adc = ads.getLastConversionResults();
        auto v = ads.computeVolts(adc);
        newData = false;
        readingChannel = 255;
        return v;
    }

    float getInputImpedance(uint8_t ch) override {
        return 6e6; // TODO this depends on PGA gain! also consider diff.inp.imp
    }
};



void ICACHE_RAM_ATTR AdsAlertISR() {
    if (ads_inst) ads_inst->alertNewDataFromISR();
}
