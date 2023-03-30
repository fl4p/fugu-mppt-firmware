#include <SPI.h> // not sure why this is needed
#include <Adafruit_ADS1X15.h>
#include "adc.h"

//#include "sampling.h"

volatile bool ads_alert = false;


class ADC_ADS : public AsyncADC<float> {

    //Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
    Adafruit_ADS1015 ads; /* Use this for the 12-bit version */

    std::array<adsGain_t, 4> gainsByChannel {GAIN_ONE,GAIN_ONE,GAIN_ONE,GAIN_ONE};

    uint8_t readingChannel = 255;
    bool newData = false;

public:
    bool init() override {
        // RATE_ADS1115_128SPS (default)
        // RATE_ADS1115_250SPS, RATE_ADS1115_475SPS
        //ads.setDataRate(RATE_ADS1115_250SPS);
        //ads.setDataRate(RATE_ADS1115_475SPS);
        //ads.setDataRate(RATE_ADS1115_860SPS);

        return ads.begin();
    }

    void setChannelGain(uint8_t channel, adsGain_t gain) {
        gainsByChannel[channel] = gain;
    }


    void startReading(uint8_t channel) {
        assert(readingChannel == 255);
        readingChannel = channel;
        ads.setGain(gainsByChannel[channel]);
        ads.startADCReading(MUX_BY_CHANNEL[2], /*continuous=*/false);
    }


    float computeVolts(int16_t counts, adsGain_t gain) {
        uint8_t m_bitShift = 0;  // ads1115
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
        return (float)counts * (fsRange / (float)(32768 >> m_bitShift));
    }

    void alertNewDataFromISR() {
        newData = true;
    }

    bool hasData() {
        return newData;
    }

    float getSample() {
        // TODO detect clipping
        int16_t adc = ads.getLastConversionResults();
        newData = false;
        auto ch = readingChannel;
        readingChannel = 255;
        return computeVolts(adc, gainsByChannel[ch]);
    }
};
