#pragma once

#include <map>


void rtcount_test_cycle_counter() {
    auto t0 = micros();
    auto c0 = esp_cpu_get_cycle_count();
    vTaskDelay(100);
    auto c1 = esp_cpu_get_cycle_count();
    auto t1 = micros();
    int cpUs = (c1 - c0) / (t1 - t0);
    ESP_LOGI("main", "dt=%lu dc=%lu cycles/s=%i", t1 - t0, c1 - c0, cpUs);
    assert(abs(cpUs - CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) < CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 0.05f);
}

struct rtcount_stat {
    unsigned long total{0}, num{0}, max{0}, max_num{0};
};

std::unordered_map<const char *, rtcount_stat> rtcount_stats{};

volatile bool rtcount_en = true;

unsigned long rtclock_us() {
    // micros
    return esp_cpu_get_cycle_count();
}

void rtcount(const char *l) {
    static unsigned long t0 = 0;

    if (rtcount_en && t0) {
        constexpr auto maxT = std::numeric_limits<unsigned long>::max();
        auto t = rtclock_us();
        //auto dt = (t < t0) ? (maxT - t0 + t) : (t - t0);
        auto dt = (t - t0)  / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
        //if(dt > maxT/2) dt = maxT - dt;
        auto f = rtcount_stats.find(l);
        if (f == rtcount_stats.end()) {
            rtcount_stats[l] = {};
            f = rtcount_stats.find(l);
        }
        auto &stat(f->second);
        stat.total += dt;
        if (dt > stat.max) {
            stat.max = dt;
            stat.max_num = stat.num;
        }
        ++stat.num;
    }

    t0 = rtclock_us();
}

void rtcount_print(bool reset) {
    if (rtcount_en) {
        rtcount_en = false;
        vTaskDelay(100);
    }

    printf("rtcount_print :\n");
    printf("%-30s %9s %9s %6s %6s %6s\n", "key", "num", "tot", "mean", "max", "maxNum");

    //typedef std::remove_reference<decltype(*rtcount_stats.begin())>::type P;
    typedef std::pair<const char *, rtcount_stat> P;
    std::vector<P> sorted(rtcount_stats.begin(), rtcount_stats.end());
    std::sort(sorted.begin(), sorted.end(), [](const P &a, const P &b) {
        return a.second.max > b.second.max;
    });

    for (auto [k, stat]: sorted) {
        printf("%-30s %9lu %9lu %6lu %6lu %6lu\n", k, stat.num, stat.total, stat.total / stat.num, stat.max,
               stat.max_num);
    }
    printf("\n\n");
    if (reset)
        rtcount_stats.clear();

    rtcount_en = true;
}

class TaskNotification {
    // https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/03-Direct-to-task-notifications/02-As-binary-semaphore
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
