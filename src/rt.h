#pragma once

#include <map>
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
    int flags: 16;                          //OR of VECDESC_FL_* defines
    unsigned int cpu: 1;
    unsigned int intno: 5;
    int source: 8;                          //Interrupt mux flags, used when not shared
    shared_vector_desc_t *shared_vec_info;  //used when VECDESC_FL_SHARED
    vector_desc_t *next;
};


//Linked list of vector descriptions, sorted by cpu.intno value
static vector_desc_t *vector_desc_head = NULL;

//Returns a vector_desc entry for an intno/cpu, or NULL if none exists.
static vector_desc_t *find_desc_for_int(int intno, int cpu)
{
    vector_desc_t *vd = vector_desc_head;
    while(vd != NULL) {
        if (vd->cpu == cpu && vd->intno == intno) {
            break;
        }
        vd = vd->next;
    }
    return vd;
}

static esp_err_t esp_intr_dump(FILE *stream)
{
    if (stream == NULL) {
        stream = stdout;
    }
#ifdef CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    const int cpu_num = 1;
#else
    const int cpu_num = SOC_CPU_CORES_NUM;
#endif

    int general_use_ints_free = 0;
    int shared_ints = 0;

    for (int cpu = 0; cpu < cpu_num; ++cpu) {
        fprintf(stream, "CPU %d interrupt status:\n", cpu);
        fprintf(stream, " Int  Level  Type   Status\n");
        for (int i_num = 0; i_num < CPU_INT_LINES_COUNT; ++i_num) {
            fprintf(stream, " %2d  ", i_num);
            esp_cpu_intr_desc_t intr_desc;
            esp_cpu_intr_get_desc(cpu, i_num, &intr_desc);
            bool is_general_use = true;
            vector_desc_t *vd = find_desc_for_int(i_num, cpu);

#ifndef SOC_CPU_HAS_FLEXIBLE_INTC
            fprintf(stream, "   %d    %s  ",
                    intr_desc.priority,
                    intr_desc.type == ESP_CPU_INTR_TYPE_EDGE ? "Edge " : "Level");

            is_general_use = (intr_desc.type == ESP_CPU_INTR_TYPE_LEVEL) && (intr_desc.priority <= XCHAL_EXCM_LEVEL);
#else // SOC_CPU_HAS_FLEXIBLE_INTC
            if (vd == NULL) {
                fprintf(stream, "   *      *    ");
            } else {
                // # TODO: IDF-9512
                // esp_cpu_intr_get_* functions need to be extended with cpu parameter.
                // Showing info for the current cpu only, in the meantime.
                if (esp_cpu_get_core_id() == cpu) {
                    fprintf(stream, "   %d    %s  ",
                            esp_cpu_intr_get_priority(i_num),
                            esp_cpu_intr_get_type(i_num) == ESP_CPU_INTR_TYPE_EDGE ? "Edge " : "Level");
                } else {
                    fprintf(stream, "   ?      ?    ");
                }
            }
#endif // SOC_CPU_HAS_FLEXIBLE_INTC

            if (intr_desc.flags & ESP_CPU_INTR_DESC_FLAG_RESVD) {
                fprintf(stream, "Reserved");
            } else if (intr_desc.flags & ESP_CPU_INTR_DESC_FLAG_SPECIAL) {
                fprintf(stream, "CPU-internal");
            } else {
                if (vd == NULL || (vd->flags & (VECDESC_FL_RESERVED | VECDESC_FL_NONSHARED | VECDESC_FL_SHARED)) == 0) {
                    fprintf(stream, "Free");
                    if (is_general_use) {
                        ++general_use_ints_free;
                    } else {
                        fprintf(stream, " (not general-use)");
                    }
                } else if (vd->flags & VECDESC_FL_RESERVED)  {
                    fprintf(stream, "Reserved (run-time)");
                } else if (vd->flags & VECDESC_FL_NONSHARED) {
                    fprintf(stream, "Used: %s", esp_isr_names[vd->source]);
                } else if (vd->flags & VECDESC_FL_SHARED) {
                    fprintf(stream, "Shared: ");
                    for (shared_vector_desc_t *svd = vd->shared_vec_info; svd != NULL; svd = svd->next) {
                        fprintf(stream, "%s ", esp_isr_names[svd->source]);
                    }
                    ++shared_ints;
                } else {
                    fprintf(stream, "Unknown, flags = 0x%x", vd->flags);
                }
            }

            fprintf(stream, "\n");
        }
    }
    fprintf(stream, "Interrupts available for general use: %d\n", general_use_ints_free);
    fprintf(stream, "Shared interrupts: %d\n", shared_ints);
    return ESP_OK;
}

#define RT_PRIO 20


static void rtcount_test_cycle_counter() {
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

static std::unordered_map<const char *, rtcount_stat> rtcount_stats{};

static volatile bool rtcount_en = true;

static unsigned long rtclock_us() {
    // micros
    return esp_cpu_get_cycle_count();
}

//#define rtcount(s) do {}while(0)

static void rtcount(const char *l) {
    static unsigned long t0 = 0;

    if (rtcount_en && t0) {
        //constexpr auto maxT = std::numeric_limits<unsigned long>::max();
        auto t = rtclock_us();
        //auto dt = (t < t0) ? (maxT - t0 + t) : (t - t0);
        auto dt = (t - t0) / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
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

static void rtcount_print(bool reset) {
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
    bool IRAM_ATTR notifyFromIsr() {
        BaseType_t higherWokenTask; // = must yield
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

static bool
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
                .intr_priority = 0, // GPTIMER_ALLOW_INTR_PRIORITY_MASK
                .flags =  {.intr_shared = 0}, // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/intr_alloc.html
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

        gptimer_event_callbacks_t cbs = {
                .on_alarm = periodic_timer_on_alarm,
        };
        if(gptimer_register_event_callbacks(gptimer, &cbs, this) != ESP_OK) {
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

    void begin(int hz, void *cb(void *), void *arg) {}

    void start();
    void stop();

};
#endif
