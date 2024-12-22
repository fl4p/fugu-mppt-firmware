#include "adc.h"
//#include <driver/adc.h>
//#include <esp_adc_cal.h>
#include <stdexcept>

#include "etc/pinconfig.h"
#include "math/statmath.h"
#include "etc/rt.h"

#include <cstring>
#include <cstdio>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#include "util.h"


// TODO decrease READ_LEN and do ADC reading from another dedicated loop with even higher priority
// than the "big" control loop and a fast shutdown path

// higher values inc latency, driver will always return this read len
// for abg=32, 2ch, 4bytes/sample:

#define ADC1_READ_LEN 128


class ADC_ESP32_Cont : public AsyncADC<float> {

public:
    //adc1_channel_t readingChannel = adc1_channel_t::ADC1_CHANNEL_MAX;
private:
    adc_cali_handle_t adc_chars[4]{nullptr, nullptr, nullptr, nullptr};
    adc_atten_t attenuation[adc_channel_t::ADC_CHANNEL_9 + 1]{};

    TaskNotification notification;
    adc_continuous_handle_t handle = nullptr;
    uint8_t result[ADC1_READ_LEN] = {0};


    uint32_t sr = 0; // sampling rate
    uint16_t avgNum = 0;
    uint8_t ntcCh = 255;

    struct ChAvgBuf {
        uint32_t agg: 22; // 22bit can store 1024 accumulated 12-bit values
        uint32_t num: 10; // 2**10 = 1024
    };
    static_assert(sizeof(ChAvgBuf) <= 4);

    ChAvgBuf avgBuf[adc_channel_t::ADC_CHANNEL_9 + 1]{};

public:
    [[nodiscard]] SampleReadScheme scheme() const override { return SampleReadScheme::any; };

    explicit ADC_ESP32_Cont(const ConfFile &sensConf) {
        avgNum = sensConf.getLong("esp32adc1_avg");
        sr = sensConf.getLong("esp32adc1_sr");
    }

    bool init(const ConfFile &pinConf) override {
        for (auto &at: attenuation)
            at = (adc_atten_t) -1;

        memset(avgBuf, 0, sizeof(avgBuf));

        return true;
    }

    void setNtcCh(uint8_t ch) { ntcCh = ch; }

    void start() override;

    void startReading(uint8_t channel) override { abort(); } // this should never get called
    float getSample() override { abort(); }

    bool hasData() override { return notification.wait(1); }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        adc_atten_t atten;
        assert(ch < adc_channel_t::ADC_CHANNEL_9);

        // esp32-s3 measurable voltage range:
        // https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#adc-attenuation

        // for best linearity, we expect a voltage < 1.8V
        // 0.81 to fit suggested range?
        // see https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#_CPPv425adc1_config_channel_atten14adc1_channel_t11adc_atten_t

        auto maxVolt = 3.548134f;
        auto suggestVolt = 0.81f *maxVolt; /*12dB=max */
        if (voltage > maxVolt) {
            throw std::range_error(
                    "ch" + std::to_string(ch) + ": expected voltage too high: " + std::to_string(voltage)
                    + " > " + std::to_string(maxVolt));
        }

        if (voltage > suggestVolt) {
            ESP_LOGW("esp32_adc", "%s", ("ch" + std::to_string(ch) + ": expected voltage too high: " + std::to_string(voltage)
                            + " > " + std::to_string(suggestVolt) + " (suggested)").c_str());
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
            adc_cali_curve_fitting_config_t conf{
                    .unit_id = ADC_UNIT_1,
                    .chan = ADC_CHANNEL_0,
                    .atten = atten,
                    .bitwidth = ADC_BITWIDTH_12,
            };
            ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&conf, &adc_chars[atten]));
            //esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, 1100, adc_chars[atten]);
        }
    }


    uint32_t read(SampleCallback &&newSampleCallback) override;


    float getInputImpedance(uint8_t ch) override { return 500e3; } //  500k ESP ADC impedance?

    bool IRAM_ATTR convDoneCallback() { return notification.notifyFromIsr(); }

    void stop() {
        ESP_ERROR_CHECK(adc_continuous_stop(handle));
        ESP_ERROR_CHECK(adc_continuous_deinit(handle));
    }
};

