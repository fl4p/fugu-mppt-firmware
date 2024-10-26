#include "adc.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <stdexcept>

#include "pinconfig.h"
#include "math/statmath.h"

/**
 * Single-shot implementation for ESP32's internal ADC1. Uses `esp_adc_cal_characterize()`.
 */
class ADC_ESP32 : public AsyncADC<float> {

public:
    adc1_channel_t readingChannel = adc1_channel_t::ADC1_CHANNEL_MAX;
private:
    esp_adc_cal_characteristics_t *adc_chars[4]{nullptr, nullptr, nullptr, nullptr};
    adc_atten_t attenuation[adc1_channel_t::ADC1_CHANNEL_MAX]{};

public:
    bool init(const ConfFile &pinConf) override {
        if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK)
            return false;
        return true;
    }

    void startReading(uint8_t channel) override { readingChannel = (adc1_channel_t) channel; }

    bool hasData() override { return true; }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        adc_atten_t atten;
        assert(ch < adc1_channel_t::ADC1_CHANNEL_MAX);

        // 0.81 to fit suggested range?
        // see https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/adc.html#_CPPv425adc1_config_channel_atten14adc1_channel_t11adc_atten_t
        if (voltage > (0.81f * 3.548134f /*11dB=max */)) {
            throw std::range_error(
                    "ch" + std::to_string(ch) + ": expected voltage too high: " + std::to_string(voltage));
        }

        if (voltage > 1.6f) atten = ADC_ATTEN_DB_12;
        else if (voltage > 0.8f * 1.33f) atten = ADC_ATTEN_DB_6;
        else if (voltage > 0.8f) atten = ADC_ATTEN_DB_2_5;
        else atten = ADC_ATTEN_DB_0;

        if (adc1_config_channel_atten((adc1_channel_t) ch, atten) != ESP_OK) {
            ESP_LOGE("adc", "Failed to set ADC1 ch %i attenuation %i", (int) ch, (int) atten);
            assert(false);
        }
        attenuation[ch] = atten;

        if (adc_chars[atten] == nullptr) {
            adc_chars[atten] = new esp_adc_cal_characteristics_t{};
            esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, 1100, adc_chars[atten]);
        }
    }


    float getSample() override {
        // TODO detect clipping
        auto raw = adc1_get_raw(readingChannel);
        float v = (float) esp_adc_cal_raw_to_voltage(raw, adc_chars[attenuation[readingChannel]]) * 1e-3f;
        return v;
    }

    float getInputImpedance(uint8_t ch) override {
        return 500e3; //  500k ESP ADC impedance?
    }
};


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
