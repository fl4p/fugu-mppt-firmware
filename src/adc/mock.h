#pragma once

#include <Arduino.h> // millis()
#include "adc.h"
#include <stdexcept>
//#include "math/statmath.h"
#include "etc/rt.h"
#include "tele/scope.h"
#include "../util.h"


const time_us &wallClockUs();

static bool adc_fake_periodic_timer_callback(void *arg);

class ADC_Fake : public AsyncADC<float> {
    /**
     * ch0: const 0
     * ch1: const 1
     * ch2: 2 + sin(a*t) if t > 2s else 0
     */
public:
    uint8_t readingChannel = 0;

    [[nodiscard]] AdcReadMode readMode() const override {
        return AdcReadMode::SnapshotAllChannels;
    }

private:
    std::array<time_us, 4> resetTimes;

    TaskNotification taskNotification{};

    PeriodicTimer periodic_timer{};
#if CONFIG_FUGU_WITH_PWR_KNOBS
    uint16_t waitMs = 10;
#else
    static constexpr uint16_t waitMs = 10;
#endif

public:
    bool init(const ConfFile &boardConf) override {
        auto f = boardConf.getLong("adc_fake_freq", 1000 * 3);
        periodic_timer.begin(f, &adc_fake_periodic_timer_callback, this);

        // Test taskNotification
        {
            taskNotification.subscribe(true);

            auto t0 = millis();
            bool r = taskNotification.wait(10);
            assert(!r);
            if (!(millis() - t0 >= 9 && millis() - t0 < 12)) {
                printf("unexpected wait time %lu≠10\n", millis() - t0);
                assert(millis() - t0 == 10);
            }

            //enqueue_task( [&] {taskNotification.notify()});
            /*
            r = taskNotification.wait(100);
            assert(millis() - t0 < 5);
            assert(r);
            r = taskNotification.wait(10);
            assert(millis() - t0 >= 9);
            assert(!r);
             */

            periodic_timer.start();

            t0 = millis();
            r = taskNotification.wait(100);
            assert(millis() - t0 < 10);
            assert(r);


            t0 = millis();
            r = taskNotification.wait(100);
            assert(millis() - t0 < 10);
            assert(r);

            taskNotification.unsubscribe();
            ESP_LOGI("adc_fake", "Timer notify test passed");
        }

        return true;
    }

    void deinit() override {
        periodic_timer.destroy();
    }

    float getSamplingRate(uint8_t channel) override {
        return static_cast<float>(periodic_timer.freq());
    };

    void startReading(uint8_t channel) override {
        taskNotification.subscribe();
        readingChannel = channel;
    }

    bool hasData() override {
        return taskNotification.wait(waitMs);
    }

#if CONFIG_FUGU_WITH_PWR_KNOBS
    // `pwr rt`: hasData() waits 1.5 periods so slow rates do not add 10 ms timeout wakes
    esp_err_t setRate(uint32_t hz) {
        esp_err_t e = periodic_timer.setFreq(hz);
        if (e == ESP_OK) waitMs = std::max<uint32_t>(10, 1500 / periodic_timer.freq());
        return e;
    }
#endif

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
    }


    float getSample() override {
        float x = NAN;
        auto t = wallClockUs() - resetTimes[readingChannel];
        auto s = sinf((float) t / 10e6f);
        if (readingChannel == 0) {
            x = .1f + 0.01f * s;;
        } /*else if (readingChannel == 1) {
            x = 0.9f + 0.1f * s;
        } */else if (readingChannel == 2) {
            if (t > 2000000) {
                // ramp up a 2 + sin(t)
                x = (2.0f + s) * min((t - 2000000.f) / 2000000.f, 1.f);
            } else {
                x = 0.0f;
            }
        } else {
            x = .9f + .1f * s;
        }

        if (scope)
            scope->addSample12(this, readingChannel, x / 3 * 4000);

        return x;
    }

    float getInputImpedance(uint8_t ch) override { return 100e3; }

    void reset(const uint8_t ch) override {
        ESP_LOGI("adc_fake", "Reset channel %u at %llu", ch, wallClockUs());
        resetTimes[ch] = wallClockUs();
    }

    bool periodicTimerCallback() {
        return taskNotification.notifyFromIsr();
    }
};

static IRAM_ATTR bool adc_fake_periodic_timer_callback(void *arg) {
    auto adc = static_cast<ADC_Fake *>(arg);
    return adc->periodicTimerCallback();
}
