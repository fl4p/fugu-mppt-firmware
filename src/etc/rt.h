#pragma once

#include <map>
#include <stdexcept>
#include <Arduino.h>

#include <soc/interrupts.h>

#include "esp_intr_alloc.h"

#define CPU_INT_LINES_COUNT 32

typedef struct vector_desc_t vector_desc_t;

#define VECDESC_FL_RESERVED     (1<<0)
#define VECDESC_FL_INIRAM       (1<<1)
#define VECDESC_FL_SHARED       (1<<2)
#define VECDESC_FL_NONSHARED    (1<<3)

struct shared_vector_desc_t {
    int disabled: 1;
    int source: 8;
    volatile uint32_t *statusreg;
    uint32_t statusmask;
    intr_handler_t isr;
    void *arg;
    shared_vector_desc_t *next;
};

//Pack using bitfields for better memory use
struct vector_desc_t {
    int flags: 16; //OR of VECDESC_FL_* defines
    unsigned int cpu: 1;
    unsigned int intno: 5;
    int source: 8; //Interrupt mux flags, used when not shared
    shared_vector_desc_t *shared_vec_info; //used when VECDESC_FL_SHARED
    vector_desc_t *next;
};


esp_err_t esp_intr_dump(FILE *stream); // since idf5.5

#define RT_PRIO 20


void rtcount_test_cycle_counter();

struct rtcount_stat {
    unsigned long total{0}, num{0},
            max{0}, max_num{0},
            min{std::numeric_limits<unsigned long>::max()}, min_num{0};
};

static std::unordered_map<const char *, rtcount_stat> rtcount_stats{};

extern volatile bool rtcount_en;

[[maybe_unused]] static inline unsigned long rtclock_us() {
    // micros
    return esp_cpu_get_cycle_count();
}

//#define rtcount(s) do {}while(0)

void rtcount(const char *l);

void rtcount_print(bool reset);

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
    bool IRAM_ATTR notifyFromIsr() {
        BaseType_t higherWokenTask; // = must yield
        if (readingTask) {
            vTaskNotifyGiveFromISR(readingTask, &higherWokenTask);
            if (higherWokenTask) {
                portYIELD_FROM_ISR(higherWokenTask);
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

static bool
periodic_timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);

class PeriodicTimer {
    gptimer_handle_t gptimer = nullptr;

public:
    typedef bool Callback(void *arg);

    Callback *callback = nullptr;
    void *arg = nullptr;

    uint32_t hz;

    /***
     *
     * @param hz
     * @param cb timer alarm callback. returns true if a higher prio task has been woken (e.g. when notifying other tasks)
     * @param arg
     */
    void begin(uint32_t hz, bool cb(void *), void *arg) {
        this->hz = hz;
        callback = cb;
        this->arg = arg;

        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = hz, // 1MHz, 1 tick=1us
            .intr_priority = 0, // GPTIMER_ALLOW_INTR_PRIORITY_MASK
            .flags = {
                .intr_shared = 0, .allow_pd = 0, .backup_before_sleep = 0, // backup_before_sleep is deprecated
            }, // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/intr_alloc.html
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

        gptimer_event_callbacks_t cbs = {
            .on_alarm = periodic_timer_on_alarm,
        };
        if (gptimer_register_event_callbacks(gptimer, &cbs, this) != ESP_OK) {
            esp_intr_dump(NULL);
            throw std::runtime_error("register event callback failed");
        }

        ESP_ERROR_CHECK(gptimer_enable(gptimer));
        gptimer_alarm_config_t alarm_config1 = {
            .alarm_count = 1, // period = 1/hz
            .reload_count = 0,
            .flags = {.auto_reload_on_alarm = 1},
        };
        ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    }

    [[nodiscard]] const uint32_t &freq() const {
        return hz; // todo use timer field
    }

    void destroy() {
        gptimer_stop(gptimer);
        gptimer_disable(gptimer);
        gptimer_del_timer(gptimer);
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
    //BaseType_t high_task_awoken = pdFALSE;
    auto pt = (PeriodicTimer *) user_data;
    return pt->callback(pt->arg);
}


#else
class PeriodicTimer {
    void begin(int hz, void *cb(void *), void *arg) {
    }

    void start();

    void stop();
};
#endif
