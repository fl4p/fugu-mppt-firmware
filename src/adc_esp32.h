#include "adc.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "pinconfig.h"
#include "statmath.h"


class ADC_ESP32 : public AsyncADC<float> {

    uint8_t readingChannel;
    esp_adc_cal_characteristics_t adc_chars;

    //static constexpr
    std::array<adc1_channel_t, 4> channels = {
            ADC1_CHANNEL_0,
            (adc1_channel_t)PinConfig::ADC_Vin,
            (adc1_channel_t)PinConfig::ADC_Iin,
            (adc1_channel_t)PinConfig::ADC_Vout
    };

    std::array<RunningMedian3<float>, 4> medians;

public:
    bool init() override {
        adc1_config_width(ADC_WIDTH_BIT_12);

        adc1_config_channel_atten((adc1_channel_t)PinConfig::ADC_Vin, ADC_ATTEN_DB_6);
        adc1_config_channel_atten((adc1_channel_t)PinConfig::ADC_Vout, ADC_ATTEN_DB_6);
        adc1_config_channel_atten((adc1_channel_t)PinConfig::ADC_Iin, ADC_ATTEN_DB_6);

        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, 1100, &adc_chars);

        return true;
    }

    //void setChannelGain(uint8_t channel, adsGain_t gain) {
        //gainsByChannel[channel] = gain;
    //}

    void startReading(uint8_t channel) override {
        readingChannel = channel;
    }

    bool hasData() override {
        return true;
    }

    float raw2V(int raw) {
        //constexpr float Vmax = 1.750f; // 6db
        //return raw * Vmax / 4095.0f;
        return esp_adc_cal_raw_to_voltage(raw, &adc_chars) * 1e-3f;
    }




    float getSample() override {
        // TODO detect clipping
        auto raw = adc1_get_raw(channels[readingChannel]);
        float v = raw2V(raw);

        return medians[readingChannel].next(v);
    }
};
