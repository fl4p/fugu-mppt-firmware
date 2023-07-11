#include "adc.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "pinconfig.h"
#include "statmath.h"

/**
 * Single-shot implementation for ESP32's internal ADC1. Uses `esp_adc_cal_characterize()`.
 */
class ADC_ESP32 : public AsyncADC<float> {

    uint8_t readingChannel = 255;
    esp_adc_cal_characteristics_t adc_chars[4]{{},
                                               {},
                                               {},
                                               {}};

    //static constexpr
    std::array<adc1_channel_t, 4> channel2pin = {
            ADC1_CHANNEL_0,
            (adc1_channel_t) PinConfig::ADC_Vin,
            (adc1_channel_t) PinConfig::ADC_Iin,
            (adc1_channel_t) PinConfig::ADC_Vout
    };

    std::array<RunningMedian3<float>, 4> medians;

public:
    bool init() override {
        if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK)
            return false;
        return true;
    }

    void startReading(uint8_t channel) override { readingChannel = channel; }

    bool hasData() override { return true; }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        adc_atten_t g;
        assert(ch < 4);
        assert(voltage <= (0.8f * 3.548134f /*11dB*/));

        if (voltage > 1.6f) g = ADC_ATTEN_DB_11;
        else if (voltage > 0.8f * 1.33f) g = ADC_ATTEN_DB_6;
        else if (voltage > 0.8f) g = ADC_ATTEN_DB_2_5;
        else g = ADC_ATTEN_DB_0;

        if(adc1_config_channel_atten(channel2pin[ch], g) != ESP_OK) {
            ESP_LOGE("adc", "Failed to set ADC1 ch %i atten %i", (int)channel2pin[ch], (int)g);
        }
        esp_adc_cal_characterize(ADC_UNIT_1, g, ADC_WIDTH_BIT_12, 1100, &adc_chars[ch]);
    }


    float getSample() override {
        // TODO detect clipping
        auto raw = adc1_get_raw(channel2pin[readingChannel]);
        float v = (float) esp_adc_cal_raw_to_voltage(raw, &adc_chars[readingChannel]) * 1e-3f;
        return medians[readingChannel].next(v);
    }
};


//


/**
 * WIP
 * RTC implementation of ESP32's internal ADC1.
 * This will have a higher sampling rate than single-short reading.
 */
class ADC_ESP32_RTC : public AsyncADC<float> {
    static const uint8_t PIN_I0 = 4;
    static const uint8_t PIN_I1 = 5;
    static const uint8_t PIN_U = 6;

    static esp_adc_cal_characteristics_t adc_chars;

    // https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#adc-attenuation
    float raw2V(int raw) {
        //constexpr float Vmax = 1.750f; // 6db
        //return raw * Vmax / 4095.0f;
        return (float) esp_adc_cal_raw_to_voltage(raw, &adc_chars) * 1e-3f;
    }

public:


    bool init(adc_atten_t atten = ADC_ATTEN_DB_6) {
        if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK)
            return false;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, 1100,
                                 &adc_chars);
        return true;
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
