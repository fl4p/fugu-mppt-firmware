#pragma once

#include <atomic>
#include <exception>
#include <stdexcept>
#include <esp_attr.h> // IRAM_ATTR
#include <esp_cpu.h>  // esp_cpu_get_cycle_count
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // TaskHandle_t, task notifications

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
    // total/max/min are in CPU cycles (converted to µs at print time for sub-µs precision)
    unsigned long long total{0};
    unsigned long num{0},
            max{0}, max_num{0},
            min{std::numeric_limits<unsigned long>::max()}, min_num{0};
};

// Fixed, pre-allocated stat table: rtcount() runs on the RT core, so it must never touch the heap.
// Keys are interned string literals (fixed call sites) -> matched by pointer, appended into this
// array via an atomic index. No allocation, ever. (The old unordered_map heap-allocated on a
// first-seen key mid-mppt.update() and tripped a TLSF heap assert.)
struct rtcount_entry {
    const char *key;
    rtcount_stat stat;
};

constexpr int RTCOUNT_MAX = 64; // distinct labels; excess is dropped (logged once in rtcount_print)

// Single definition lives in rt.cpp; declared extern so all TUs share one table.
extern rtcount_entry rtcount_table[RTCOUNT_MAX];
extern std::atomic<int> rtcount_count;

extern volatile bool rtcount_en;

[[maybe_unused]] static inline unsigned long rtclock_us() {
    // micros
    return esp_cpu_get_cycle_count();
}

//#define rtcount(s) do {}while(0)

void rtcount(const char *l);

void rtcount_print(bool reset);

// Heap operations of the RT task (CONFIG_FUGU_RT_ALLOC_COUNT, heap hooks). rtAllocBegin() selects the
// calling task; rtAllocPrint() writes the counts (and, with CONFIG_FUGU_RT_ALLOC_TRACE, the call sites).
void rtAllocBegin();
void rtAllocPrint();

// core that last ran the internal-ADC / alert-pin interrupt (0xff: none yet)
extern uint8_t g_isrCore[2];

// HW over-voltage alert (sensor.conf ina22x_ov): set by the alert ISR, consumed by mppt.protect()
extern volatile bool g_hwOvTrip;
// called from the alert ISR to force the gates off; must be IRAM-safe
extern void (*volatile g_hwOvIsrHook)();

// Runs fn on a task pinned to `core` and waits for it, so interrupts allocated inside land on that
// core. Exceptions are rethrown in the caller.
template<class F>
void runOnCore(BaseType_t core, F &&fn, uint32_t stack = 8192) {
    struct Ctx {
        F *fn;
        TaskHandle_t caller;
        std::exception_ptr ex;
    } ctx{&fn, xTaskGetCurrentTaskHandle(), nullptr};
    if (xPortGetCoreID() != core && xTaskCreatePinnedToCore([](void *a) {
        auto *c = (Ctx *) a;
        try { (*c->fn)(); } catch (...) { c->ex = std::current_exception(); }
        xTaskNotifyGive(c->caller);
        vTaskDelete(nullptr);
    }, "onCore", stack, &ctx, uxTaskPriorityGet(nullptr), nullptr, core) == pdPASS) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ctx.ex) std::rethrow_exception(ctx.ex);
    } else {
        if (xPortGetCoreID() != core) ESP_LOGW("rt", "runOnCore(%d): no task, running on core %d", (int) core, (int) xPortGetCoreID());
        fn();
    }
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
    __attribute__((always_inline)) inline bool wait(uint32_t ms) {
        return waitCount(ms) != 0;
    }

    // Like wait() but returns how many notifications had piled up (0 == timeout).
    // clear-on-exit (pdTRUE): a burst that accumulated while the reader wasn't draining reads as one
    // wakeup carrying the full count. The old (pdFALSE, ==1) decrement-by-one returned false whenever
    // >1 had accumulated, starving read().
    __attribute__((always_inline)) inline uint32_t waitCount(uint32_t ms) {
        void(this);
        //assert(readingTask and readingTask == xTaskGetCurrentTaskHandle());
        return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms));
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
#if CONFIG_FUGU_WITH_PWR_KNOBS
    [[nodiscard]] bool subscribed() const { return readingTask; }
#endif
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
    void begin(uint32_t requestedHz, bool cb(void *), void *arg) {
        callback = cb;
        this->arg = arg;

        // Fixed 1 MHz timer resolution; the period comes from alarm_count, not the resolution.
        // (resolution_hz = requestedHz with alarm_count = 1 overflowed the gptimer prescaler
        // src_clk/resolution for low rates -> timer_ll divider assert; e.g. <~2.5 kHz failed.)
        // A 1 MHz resolution keeps the prescaler a small valid divider for any source clock, so
        // any sample rate (down to a few Hz) is configurable. alarm_count = 1e6/requestedHz.
        constexpr uint32_t RES_HZ = 1000000;
        uint32_t alarmCount = requestedHz ? RES_HZ / requestedHz : RES_HZ;
        if (alarmCount < 1) alarmCount = 1;
        this->hz = RES_HZ / alarmCount; // actual achieved rate (quantized to 1e6/N)

        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT, // APB = 80 MHz
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = RES_HZ, // 1 MHz ticks
            .intr_priority = 0, // GPTIMER_ALLOW_INTR_PRIORITY_MASK
            .flags = {}, // all off; field set differs between IDF 5 and 6
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
            .alarm_count = alarmCount, // period = alarmCount / RES_HZ = 1/hz
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

#if CONFIG_FUGU_WITH_PWR_KNOBS
    // stop + disable: releases the driver's APB_FREQ_MAX pm lock
    esp_err_t suspend() {
        esp_err_t e = gptimer_stop(gptimer);
        return e == ESP_OK ? gptimer_disable(gptimer) : e;
    }

    // Running timer, or a suspended one with resume. Restarts the period from 0 (a shrunk alarm may
    // already be behind the count).
    esp_err_t setFreq(uint32_t requestedHz, bool resume = false) {
        uint32_t n = requestedHz ? 1000000 / requestedHz : 1000000;
        if (n < 2) n = 2;
        gptimer_alarm_config_t a = {};
        a.alarm_count = n;
        a.flags.auto_reload_on_alarm = 1;
        esp_err_t e = resume ? gptimer_enable(gptimer) : gptimer_stop(gptimer);
        if (e == ESP_OK) e = gptimer_set_raw_count(gptimer, 0);
        if (e == ESP_OK) e = gptimer_set_alarm_action(gptimer, &a);
        if (e == ESP_OK) hz = 1000000 / n;
        esp_err_t s = gptimer_start(gptimer);
        return e != ESP_OK ? e : s;
    }
#endif
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
