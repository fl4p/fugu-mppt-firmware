#pragma once

#include <stdexcept>
#include <cstring>
#include <cstdio>

#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_timer.h>
#include "esp_log.h"

#include "adc.h"
#include "math/statmath.h"
#include "etc/rt.h"
#include "util.h"
#include "sdkconfig.h"


// conv_frame_size = ADC1_READ_LEN/2 sets the DMA EOF granularity. The driver keeps a fixed
// INTERNAL_BUF_NUM (5) frames of DMA descriptors, so DMA survival under an ISR stall = 5 * frame
// time. A console uxTaskGetSystemState() (tasks/rt-stats) holds taskENTER_CRITICAL ~1.2ms (8 tasks),
// during which our conv_done callback can't recycle descriptors. At 128B frames (here) that's
// 5*~0.38ms = ~1.9ms of headroom at 83kHz, so the DMA rides through instead of wedging. Cost:
// conv_done/OV-protection latency rises ~one frame (192us->~384us). A busier (>~13 task) converter's
// critical section can exceed 1.9ms; the loopRT watchdog then resets+recovers it. See
// doc/dev-notes/Real-Time Latency.
#define ADC1_READ_LEN 256

#define ADC_ATTEN_NA ((adc_atten_t)-1)

class ADC_ESP32_Cont : public AsyncADC<float> {
private:
    adc_cali_handle_t calByAtten[4]{nullptr, nullptr, nullptr, nullptr};
    adc_atten_t attenByCh[adc_channel_t::ADC_CHANNEL_9 + 1] = {ADC_ATTEN_NA};
    adc_atten_t maxAtten = ADC_ATTEN_DB_0;

    TaskNotification notification;
    adc_continuous_handle_t handle = nullptr;
    uint8_t result[ADC1_READ_LEN] = {0};
    bool good_ = true; // cleared on a continuous-read driver error, restored by start()
    int64_t lastDataUs_ = 0; // last time the DMA delivered samples (no-sample watchdog)
    int64_t lastWarnUs_ = 0; // throttles the no-sample warning
    // 250ms: well above the ~0.38ms frame interval, low enough that a wedge the DMA headroom didn't
    // absorb is reset quickly (the converter is NOT stopped for a transient — see loopRT). A genuine
    // dead ADC is caught by the sustained-error path there.
    static constexpr int64_t kNoDataTimeoutUs = 250000;


    uint32_t sr = 0; // sampling rate of driver
    uint16_t avgNum = 0;
    uint8_t ntcCh = 255;

    struct ChAvgBuf {
        uint32_t agg: 22; // 22bit can store 1024 accumulated 12-bit values
        uint32_t num: 10; // 2**10 = 1024
    };

    static_assert(sizeof(ChAvgBuf) <= 4);

    ChAvgBuf avgBuf[adc_channel_t::ADC_CHANNEL_9 + 1]{};

public:
    [[nodiscard]] AdcReadMode readMode() const override { return AdcReadMode::StreamedCallback; };

    explicit ADC_ESP32_Cont(const ConfFile &sensConf) {
        avgNum = sensConf.getLong("esp32adc1_avg");
        sr = sensConf.getLong("esp32adc1_sr");
        // ChAvgBuf::num is 10-bit: avgNum>=1024 would wrap before reaching the count and starve the
        // channel; agg is 22-bit and holds 1023*4095 with margin, so 1..1023 is safe.
        assert_throw(avgNum >= 1 && avgNum <= 1023, "esp32adc1_avg must be 1..1023");
    }

    bool init(const ConfFile &boardConf) override {
        for (auto &at: attenByCh)
            at = ADC_ATTEN_NA;

        memset(avgBuf, 0, sizeof(avgBuf));

        return true;
    }

    float getSamplingRate(uint8_t ch) override {
        int chNum = 0;
        bool hasNtc = false;
        for (auto ch = 0; ch <= adc_channel_t::ADC_CHANNEL_9; ++ch) {
            if (attenByCh[ch] != ADC_ATTEN_NA) ++chNum;
            if (ch == ntcCh) hasNtc = true;
        }
        assert_throw(chNum > 0, "");

        // pattern length:
        chNum = 2 * chNum;
        if (hasNtc) chNum -= 1;

        if (ntcCh == ch) {
            return (float) sr / (float) (avgNum * chNum);
        } else {
            // all other channels are sampled 2 times per patterns
            return (float) sr / (float) (avgNum * chNum) * 2.0f;
        }
    }

    void deinit() override {
        if (!handle) return;
        adc_continuous_stop(handle);
        adc_continuous_deinit(handle);
        handle = nullptr;
    }

    void setNtcCh(uint8_t ch) { ntcCh = ch; }

    void start() override;

    // The continuous DMA can silently stall (conv-done stops firing) — isGood()'s no-sample
    // watchdog detects it but nothing else restarts the driver. Tear down and re-start in place
    // so the sampler's recovery path can heal a dead DMA without a reboot. atten/cali config is
    // kept (set once via setMaxExpectedVoltage); start() restores good_/lastDataUs_.
    bool resetPeripherals() override {
        ESP_LOGW("adc_esp32", "resetPeripherals: restarting continuous ADC");
        deinit();
        try {
            start();
        } catch (const std::exception &e) {
            ESP_LOGE("adc_esp32", "restart failed: %s", e.what());
            good_ = false;
            return false;
        }
        return true;
    }

    void startReading(uint8_t channel) override { abort(); } // this should never get called
    float getSample() override { abort(); }

    bool hasData() override { return notification.wait(1); }

    bool isGood() override {
        if (!good_) return false;
        int64_t now = esp_timer_get_time();
        if (now - lastDataUs_ > kNoDataTimeoutUs) {
            if (now - lastWarnUs_ > 5000000) { // throttle to 5s
                lastWarnUs_ = now;
                ESP_LOGW("adc_esp32", "no ADC samples for %lu ms", (unsigned long) ((now - lastDataUs_) / 1000));
            }
            return false;
        }
        return true;
    }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        adc_atten_t atten;
        assert_throw(ch <= adc_channel_t::ADC_CHANNEL_9, "adc channel out of range");

        auto maxVolt = 3.548134f;
        auto suggestVolt = 0.81f * maxVolt;
        if (voltage > maxVolt) {
            throw std::range_error(
                "ch" + std::to_string(ch) + ": expected voltage too high: " + std::to_string(voltage)
                + " > " + std::to_string(maxVolt));
        }

        if (voltage > suggestVolt) {
            ESP_LOGW("esp32_adc", "%s",
                     ("ch" + std::to_string(ch) + ": expected voltage too high: " + std::to_string(voltage)
                         + " > " + std::to_string(suggestVolt) + " (suggested)").c_str());
        }

        if (voltage > 1.6f) atten = ADC_ATTEN_DB_12;
        else if (voltage > 0.8f * 1.33f) atten = ADC_ATTEN_DB_6;
        else if (voltage > 0.8f) atten = ADC_ATTEN_DB_2_5;
        else atten = ADC_ATTEN_DB_0;

        assert_throw(handle == nullptr, "adc already started");

        // IDF adc_continuous_config requires all ADC1 channels share the same attenuation.
        // Track the max; start() normalizes all channels to it.
        if (atten > maxAtten) maxAtten = atten;

        attenByCh[ch] = atten;

        if (calByAtten[atten] == nullptr) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
            adc_cali_curve_fitting_config_t conf{
                .unit_id = ADC_UNIT_1,
                .chan = ADC_CHANNEL_0,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_12,
            };
            ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&conf, &calByAtten[atten]));
#else
            adc_cali_line_fitting_config_t cali_config = {
                .unit_id = ADC_UNIT_1,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
                .default_vref = 0,
            };
            ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, & calByAtten[atten]));
#endif
        }
    }


    uint32_t read(SampleCallback &&newSampleCallback) override;


    float getInputImpedance(uint8_t ch) override { return 500e3; } //  500k ESP ADC impedance?

    bool IRAM_ATTR convDoneCallback() { return notification.notifyFromIsr(); }
};

