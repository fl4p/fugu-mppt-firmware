#include "adc.h"
//#include <driver/adc.h>
//#include <esp_adc_cal.h>
#include <stdexcept>

#include "pinconfig.h"
#include "math/statmath.h"
#include "rt.h"

#include <string.h>
#include <stdio.h>
#include <esp_adc_cal.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "util.h"

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN 256


class ADC_ESP32_Cont : public AsyncADC<float> {

public:
    //adc1_channel_t readingChannel = adc1_channel_t::ADC1_CHANNEL_MAX;
private:
    esp_adc_cal_characteristics_t *adc_chars[4]{nullptr, nullptr, nullptr, nullptr};
    adc_atten_t attenuation[adc1_channel_t::ADC1_CHANNEL_MAX]{};

    TaskNotification notification;
    adc_continuous_handle_t handle = nullptr;
    uint8_t result[EXAMPLE_READ_LEN] = {0};

public:
    [[nodiscard]] SampleReadScheme scheme() const override { return SampleReadScheme::any;};

    bool init(const ConfFile &pinConf) override {
        for (auto &at: attenuation)
            at = (adc_atten_t) -1;
        return true;
    }

    void start() override;

    void startReading(uint8_t channel) override { abort(); } // this should never get called
    float getSample() override { abort(); }

    bool hasData() override { return notification.wait(1); }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        adc_atten_t atten;
        assert(ch < adc1_channel_t::ADC1_CHANNEL_MAX);

        // for best linearity, we expect a voltage < 1.8V
        // 0.81 to fit suggested range?
        // see https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/adc.html#_CPPv425adc1_config_channel_atten14adc1_channel_t11adc_atten_t
        auto maxVolt = 0.81f * 3.548134f;/*12dB=max */
        if (voltage > maxVolt) {
            throw std::range_error(
                    "ch" + std::to_string(ch) + ": expected voltage too high: " + std::to_string(voltage)
                    + " > " + std::to_string(maxVolt));
        }

        if (voltage > 1.6f) atten = ADC_ATTEN_DB_12;
        else if (voltage > 0.8f * 1.33f) atten = ADC_ATTEN_DB_6;
        else if (voltage > 0.8f) atten = ADC_ATTEN_DB_2_5;
        else atten = ADC_ATTEN_DB_0;

        //if (adc1_config_channel_atten((adc1_channel_t) ch, atten) != ESP_OK) {
        //    ESP_LOGE("adc", "Failed to set ADC1 ch %i attenuation %i", (int) ch, (int) atten);
        //    assert(false);
        //}

        assert_throw(handle == nullptr, "adc already started");

        attenuation[ch] = atten;

        if (adc_chars[atten] == nullptr) {
            adc_chars[atten] = new esp_adc_cal_characteristics_t{};
            esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, 1100, adc_chars[atten]);
        }
    }


    uint32_t read(SampleCallback &&newSampleCallback) override {
        uint32_t ret_num = 0;
        esp_err_t ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);

        if (ret == ESP_OK) {
            //ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                auto *p = (adc_digi_output_data_t *) &result[i];
                uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
                    //ESP_LOGI("adc_esp32", "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
                    float v = (float) esp_adc_cal_raw_to_voltage(data, adc_chars[attenuation[chan_num]]) * 1e-3f;
                    newSampleCallback(chan_num, v);
                } else {
                    ESP_LOGW("adc_esp32", "Invalid data [%s_%"PRIu32"_%"PRIx32"]", "ADC1", chan_num, data);
                }
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
            ESP_LOGW("adc_esp32", "Read timeout.");
            //vTaskDelay(100);
        }

        return ret_num;
    }


    float getInputImpedance(uint8_t ch) override { return 500e3; } //  500k ESP ADC impedance?

    bool IRAM_ATTR convDoneCallback() { return notification.notifyFromIsr(); }

    void stop() {
        ESP_ERROR_CHECK(adc_continuous_stop(handle));
        ESP_ERROR_CHECK(adc_continuous_deinit(handle));
    }
};

