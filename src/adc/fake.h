#pragma once

#include "adc.h"
#include <stdexcept>
//#include "math/statmath.h"
#include "rt.h"


const unsigned long &loopWallClockUs();

static bool adc_fake_periodic_timer_callback(void *arg);

class ADC_Fake : public AsyncADC<float> {
    /**
     * ch0: const 0
     * ch1: const 1
     * ch2: 2 + sin(a*t) if t > 2s else 0
     */
public:
    uint8_t readingChannel = 0;

    [[nodiscard]] SampleReadScheme scheme() const override {
        return SampleReadScheme::cycle;
    }

private:

    std::array<unsigned long, 4> resetTimes;

    TaskNotification taskNotification{};

    PeriodicTimer periodic_timer{};

public:
    bool init(const ConfFile &pinConf) override {
        auto f = pinConf.getLong("adc_fake_freq", 4000 * 3);
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


    void startReading(uint8_t channel) override {
        taskNotification.subscribe();
        readingChannel = channel;
    }

    bool hasData() override {
        return taskNotification.wait(10);
    }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {}


    float getSample() override {
        if (readingChannel == 0) {
            return 0;
        } else if (readingChannel == 1) {
            return 1;
        } else if (readingChannel == 2) {
            auto t = loopWallClockUs() - resetTimes[readingChannel];
            if (t > 2000000) {
                // ramp up a 2 + sin(t)
                return (2.0f + sinf((float) t / 10e6f)) * min((t - 2000000.f) / 2000000.f, 1.f);
            } else {
                return 0.0f;
            }
        }
        return NAN;
    }

    float getInputImpedance(uint8_t ch) override { return 100e3; }

    void reset(const uint8_t ch) override {
        ESP_LOGI("adc_fake", "Reset channel %hhu at %lu", ch, loopWallClockUs());
        resetTimes[ch] = loopWallClockUs();
    }

    bool periodicTimerCallback() {
        return taskNotification.notifyFromIsr();
    }
};

static IRAM_ATTR bool adc_fake_periodic_timer_callback(void *arg) {
    auto adc = static_cast<ADC_Fake *>(arg);
    return adc->periodicTimerCallback();
}