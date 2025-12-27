#pragma once

#include <SPI.h> // not sure why this is needed
#include <Adafruit_ADS1X15.h>
#include "adc.h"
#include "util.h"
#include "etc/pinconfig.h"
#include "etc/rt.h"
#include "tele/scope.h"

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
    bool _isADS1115_16bit = false;

    TaskNotification taskNotification{};

public:
    explicit ADC_ADS(bool _16bit) {
        if (_16bit) {
            ads = Adafruit_ADS1115();
            _isADS1115_16bit = true;
        } else {
            ads = Adafruit_ADS1015();
        }
    }

    [[nodiscard]] SampleReadScheme scheme() const override {
        return SampleReadScheme::cycle; // cycle
    }

    /*
    uint32_t read(SampleCallback &&newSampleCallback) override {

        //newSampleCallback

        newSampleCallback(readingChannel, getSample());


    } */

    bool init(const ConfFile &boardConf) override {
        auto adsAlert = boardConf.getByte("ads_alert");

        if (adsAlert == 0) {
            return false;
        }

        if (!ads.begin(0x48)) {
            return false;
        }

        pinMode((uint8_t) boardConf.getByte("ads_alert"), INPUT_PULLUP);

        if (ads_inst) {
            return false;
        }
        ads_inst = this;
        attachInterrupt(digitalPinToInterrupt(adsAlert), AdsAlertISR, FALLING); // TODO rising

        // defaults: RATE_ADS1015_1600SPS and RATE_ADS1115_128SPS

        testAlert();

        if (!_isADS1115_16bit) {
            ads.setDataRate(RATE_ADS1015_490SPS);
        }
        //ads.setDataRate(RATE_ADS1115_860SPS); // this is for ADS1015 also! (130 sps). fake chips?
        // ads.setDataRate(RATE_ADS1015_3300SPS);
        //ads.setDataRate(RATE_ADS1015_3300SPS);
        return true;
    }

    float getSamplingRate(uint8_t ch) override {
        int usedChannels = 3; // TODO
        if (_isADS1115_16bit) {
            assert(ads.getDataRate()==RATE_ADS1115_128SPS);
            return 128.f / (float) usedChannels;
        } else {
            assert(ads.getDataRate()==RATE_ADS1015_490SPS);
            return 490.f / (float) usedChannels;
        }
    }

    void deinit() {
        void();
        // TODO detach interrupt
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
        assert_throw(channel <= 3u, "");
        if (gain != gainsByChannel[channel])
            ESP_LOGI("ads", "Set channel %hhu gain %.3f -> %.3f", channel, gainVoltageRange(gainsByChannel[channel]),
                 gainVoltageRange(gain));
        gainsByChannel[channel] = gain;
    }

    void startReading(uint8_t channel) override {
        //ESP_LOGI("ads", "Start reading channel %hhu", channel);
        //assert(readingChannel == 255);
        readingChannel = channel;
        ads.setGain(gainsByChannel[channel]);
        //newData = false;
        ads.startADCReading(MUX_BY_CHANNEL[channel], /*continuous=*/false);
        taskNotification.subscribe();
    }

    bool testAlert() {
        vTaskDelay(1);
        newData = false;
        ads.setDataRate(_isADS1115_16bit ? RATE_ADS1115_128SPS : RATE_ADS1015_1600SPS);
        ads.startADCReading(MUX_BY_CHANNEL[0], /*continuous=*/false);
        vTaskDelay(pdMS_TO_TICKS(10));
        assert(newData);
        vTaskDelay(pdMS_TO_TICKS(10));
        //assert(!newData);

        newData = false;
        ads.startADCReading(MUX_BY_CHANNEL[0], /*continuous=*/false);
        auto t0 = micros();
        while (!newData) { if (micros() - t0 > 100000) return false; } // busy wait
        auto convTime = micros() - t0;

        ESP_LOGI("ads", "ADC conv time: %lu us", convTime);
        //auto expectedCT = 1e6f / ( _isADS1115_16bit ? 128 : 1600);
        //assert ((fabs(convTime - expectedCT) / expectedCT) < 0.05f);

        return true;
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

    inline void alertNewDataFromISR() {
        newData = true;
        taskNotification.notifyFromIsr();
    }

    inline bool hasData() override {
        //return taskNotification.wait(3);
        //bool hd = newData || (taskNotification.wait(30) && newData);
        if (!taskNotification.wait(30)) {
            //ESP_LOGW("ads", "ads timeout!");
            startReading(readingChannel);
            return false;
        }

        if (newData) {
            newData = false;
            return true;
        }
        return false;

        //bool hd = newData || taskNotification.wait(30);
        //newData = false;
        //return hd;
    }

    float getSample() override {
        // TODO detect clipping
        int16_t raw = ads.getLastConversionResults();
        if (scope) scope->addSample12(this, readingChannel, raw);
        auto v = ads.computeVolts(raw);
        //if (v > 4) v = 4;
        //if (v < 0) v = 0;
        //newData = false;
        //readingChannel = 255;
        return v;
    }

    float getInputImpedance(uint8_t ch) override {
        return 6e6; // TODO this depends on PGA gain! also consider diff.inp.imp
    }
};


void ICACHE_RAM_ATTR AdsAlertISR() {
    if (ads_inst) ads_inst->alertNewDataFromISR();
}
