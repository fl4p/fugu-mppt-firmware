#include "adc.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "pinconfig.h"
#include "statmath.h"


class ADC_ESP32 : public AsyncADC<float> {

    uint8_t readingChannel;
    esp_adc_cal_characteristics_t adc_chars[4];

    //static constexpr
    std::array<adc1_channel_t, 4> channel2pin = {
            ADC1_CHANNEL_0,
            (adc1_channel_t)PinConfig::ADC_Vin,
            (adc1_channel_t)PinConfig::ADC_Iin,
            (adc1_channel_t)PinConfig::ADC_Vout
    };

    std::array<RunningMedian3<float>, 4> medians;

public:
    bool init() override {
        adc1_config_width(ADC_WIDTH_BIT_12);

        //for(uint8_t i = 0; i < 4; ++i) {
        //    setMaxExpectedVoltage(i, 2);
        //}

        return true;
    }

    void startReading(uint8_t channel) override {
        readingChannel = channel;
    }

    bool hasData() override {
        return true;
    }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        adc_atten_t g;
        assert(ch < 4);
        assert(voltage <= (0.8f * 3.548134f /*11dB*/));

        if (voltage > 1.6f) g = ADC_ATTEN_DB_11;
        else if (voltage > 0.8f * 1.33f) g = ADC_ATTEN_DB_6;
        else if (voltage > 0.8f) g = ADC_ATTEN_DB_2_5;
        else g = ADC_ATTEN_DB_0;

        adc1_config_channel_atten(channel2pin[readingChannel], g);
        esp_adc_cal_characterize(ADC_UNIT_1, g, ADC_WIDTH_BIT_12, 1100, &adc_chars[ch]);
    }


    float getSample() override {
        // TODO detect clipping
        auto raw = adc1_get_raw(channel2pin[readingChannel]);
        float v = (float)esp_adc_cal_raw_to_voltage(raw, &adc_chars[readingChannel]) * 1e-3f;
        return medians[readingChannel].next(v);
    }
};
