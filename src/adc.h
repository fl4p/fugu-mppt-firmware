#pragma once

#include <cstdlib>

#include <driver/adc.h>
#include <esp_adc_cal.h>

template <class T> class AsyncADC {
public:
    virtual bool init() = 0;
    virtual void startReading(uint8_t channel) = 0;
    virtual bool hasData() = 0;
    virtual T getSample() = 0;
    //virtual uint8_t getReadingChannel() = 0;
};

//

class ADC_ESP32_RTC : public AsyncADC<float> {
    static const uint8_t PIN_I0 = 4;
    static const uint8_t PIN_I1 = 5;
    static const uint8_t PIN_U = 6;

    static esp_adc_cal_characteristics_t adc_chars;

    // https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#adc-attenuation
    float raw2V(int raw) {
        //constexpr float Vmax = 1.750f; // 6db
        //return raw * Vmax / 4095.0f;
        return (float)esp_adc_cal_raw_to_voltage(raw, &adc_chars) * 1e-3f;
    }

public:


    bool init(adc_atten_t atten=ADC_ATTEN_DB_6) {
        if(adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK)
            return false;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, 1100,
                                 &adc_chars);
        return true;
    }

    void initChannel(uint8_t ch) {
        adc1_config_channel_atten(static_cast<adc1_channel_t>(ch), adc_chars.atten);
    }

    void startReading() { /*nop*/}

    bool hasData() {
        // we do the actual read in getSample(), so always have data to sample
        return true;
    }

    float getSample(uint8_t channel) {
        auto raw = adc1_get_raw(static_cast<adc1_channel_t>(channel));
        return raw2V(raw);
    }
};
