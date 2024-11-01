#pragma once

#include <map>

struct rtcount_stat {
    unsigned long total{0}, num{0}, max{0};
};

std::unordered_map<const char *, rtcount_stat> rtcount_stats{};

volatile bool rtcount_en = true;

void rtcount(const char *l) {
    static unsigned long t0 = 0;

    if(!rtcount_en) return;

    if (t0) {
        auto dt = micros() - t0;
        auto f = rtcount_stats.find(l);
        if (f == rtcount_stats.end()) {
            rtcount_stats[l] = {};
            f = rtcount_stats.find(l);
        }
        auto &stat(f->second);
        ++stat.num;
        stat.total += dt;
        if (dt > stat.max) stat.max = dt;
    }

    t0 = micros();
}

void rtcount_print(bool reset) {
    if(rtcount_en) {
        rtcount_en = false;
        vTaskDelay(100);
    }

    printf("rtcount_print:\n");
    printf("%-22s %9s %9s %6s %6s (us)\n", "key", "num", "tot", "mean", "max");

    //typedef std::remove_reference<decltype(*rtcount_stats.begin())>::type P;
    typedef std::pair<const char *, rtcount_stat> P;
    std::vector<P> sorted(rtcount_stats.begin(), rtcount_stats.end());
    std::sort(sorted.begin(), sorted.end(), [](const P &a, const P&b ) {
        return a.second.max > b.second.max;
    });

    for (auto [k, stat]: sorted) {
        printf("%-22s %9lu %9lu %6lu %6lu\n", k, stat.num, stat.total, stat.total / stat.num, stat.max);
    }
    printf("\n\n");
    if(reset)
        rtcount_stats.clear();

    rtcount_en = true;
}

class TaskNotification {
    // https:// www. FreeRTOS. org/ RTOS-task-notifications. html
    // notifications are faster than semaphores!
    TaskHandle_t readingTask = nullptr;
public:
    inline void subscribe(bool updateTaskHandle = false) {
        if (updateTaskHandle or unlikely(readingTask == nullptr))
            readingTask = xTaskGetCurrentTaskHandle();
    }

    void unsubscribe() {
        readingTask = nullptr;
    }

    /***
     *
     * @param ms
     * @return false on timeout
     */
    inline bool wait(uint32_t ms) {
        void(this);
        //assert(readingTask and readingTask == xTaskGetCurrentTaskHandle());
        return ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(ms)) == 1; // pdTRUE: binary semaphore
    }

    /***
     *
     * @return true if a higher priority task has been woken
     */
    bool notifyFromIsr() {
        BaseType_t higherWokenTask;
        if (readingTask) {
            vTaskNotifyGiveFromISR(readingTask, &higherWokenTask);
            if (higherWokenTask) {
                portYIELD_FROM_ISR();
                return true;
            }
        }
        return false;
    }

    void notify() {
        xTaskNotifyGive(readingTask);
    }
};


#if 1 // legacy timer api

// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gptimer.html
// https://github.com/espressif/arduino-esp32/blob/master/docs/en/api/timer.rst

#include <driver/gptimer.h>

static bool IRAM_ATTR
periodic_timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);

class PeriodicTimer {
    gptimer_handle_t gptimer = nullptr;

public:
    typedef bool Callback(void *arg);

    Callback *callback = nullptr;
    void *arg = nullptr;

    /***
     *
     * @param hz
     * @param cb timer alarm callback. returns true if a higher prio task has been woken (e.g. when notifying other tasks)
     * @param arg
     */
    void begin(uint32_t hz, bool cb(void *), void *arg) {
        callback = cb;
        this->arg = arg;

        gptimer_config_t timer_config = {
                .clk_src = GPTIMER_CLK_SRC_DEFAULT,
                .direction = GPTIMER_COUNT_UP,
                .resolution_hz = hz, // 1MHz, 1 tick=1us
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

        gptimer_event_callbacks_t cbs = {
                .on_alarm = periodic_timer_on_alarm,
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, this));

        ESP_ERROR_CHECK(gptimer_enable(gptimer));
        gptimer_alarm_config_t alarm_config1 = {
                .alarm_count = 1, // period = 1/hz
                .reload_count = 0,
                .flags = {.auto_reload_on_alarm = 1},
        };
        ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    }

    void start() {
        ESP_ERROR_CHECK(gptimer_start(gptimer));
    }

    void stop() {
        ESP_ERROR_CHECK(gptimer_stop(gptimer));
    }
};

static bool IRAM_ATTR
periodic_timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    auto pt = (PeriodicTimer *) user_data;
    return pt->callback(pt->arg);
}

#else
class PeriodicTimer {

    void begin(int hz, void *cb(void *), void *arg) {}

    void start();
    void stop();

};
#endif
