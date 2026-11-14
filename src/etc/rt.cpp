#include <Arduino.h> // micros()
#include <cstring>
#include <esp_timer.h>
#include "rt.h"
#include "logging.h"
#include "util.h" // RT_CORE

volatile bool rtcount_en = true;

rtcount_entry rtcount_table[RTCOUNT_MAX]{};
std::atomic<int> rtcount_count{0};

//Linked list of vector descriptions, sorted by cpu.intno value
vector_desc_t *vector_desc_head = NULL;

//Returns a vector_desc entry for an intno/cpu, or NULL if none exists.
vector_desc_t *find_desc_for_int(int intno, int cpu) {
    vector_desc_t *vd = vector_desc_head;
    while (vd != NULL) {
        if (vd->cpu == cpu && vd->intno == intno) {
            break;
        }
        vd = vd->next;
    }
    return vd;
}


esp_err_t esp_intr_dump_(FILE *stream) {
    // TODO remove? idf5.5 already ships with `esp_intr_dump()`
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
                } else if (vd->flags & VECDESC_FL_RESERVED) {
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


DRAM_ATTR uint8_t g_isrCore[2] = {0xff, 0xff};
DRAM_ATTR volatile bool g_hwOvTrip = false;
DRAM_ATTR void (*volatile g_hwOvIsrHook)() = nullptr;

void rtcount(const char *l) {
    static unsigned long t0 = 0;

    if (rtcount_en && t0) {
        //constexpr auto maxT = std::numeric_limits<unsigned long>::max();
        auto t = rtclock_us();
        //auto dt = (t < t0) ? (maxT - t0 + t) : (t - t0);
        auto dt = (t - t0); // CPU cycles; converted to µs at print time for sub-µs precision
        //if(dt > maxT/2) dt = maxT - dt;

        // pointer-match against the interned label literals (no heap, no strcmp)
        int n = rtcount_count.load(std::memory_order_acquire);
        rtcount_stat *stat = nullptr;
        for (int i = 0; i < n; ++i) {
            if (rtcount_table[i].key == l) { stat = &rtcount_table[i].stat; break; }
        }
        if (!stat) {
            int idx = rtcount_count.fetch_add(1, std::memory_order_acq_rel);
            if (idx < RTCOUNT_MAX) {
                rtcount_table[idx].key = l;
                rtcount_table[idx].stat = {};
                stat = &rtcount_table[idx].stat;
            } else {
                rtcount_count.store(RTCOUNT_MAX, std::memory_order_release); // cap; drop this label
            }
        }
        if (stat) {
            stat->total += dt;
            if (dt > stat->max) {
                stat->max = dt;
                stat->max_num = stat->num;
            }
            if (dt < stat->min) {
                stat->min = dt;
                stat->min_num = stat->num;
            }
            ++stat->num;
        }
    }

    t0 = rtclock_us();
}

void rtcount_print(bool reset) {
    if (rtcount_en) {
        rtcount_en = false;
        vTaskDelay(100);
    }

    constexpr float mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ; // cycles -> µs

    UART_LOG("rtcount_print : (µs)");
    UART_LOG("%-30s %9s %9s %8s %8s %6s %8s %6s", "key", "num", "tot", "mean", "max", "maxNum", "min", "minNum");

    // rtcount_en is false here, so the table is quiescent — safe to sort in place by max desc.
    int n = rtcount_count.load(std::memory_order_acquire);
    if (n > RTCOUNT_MAX) n = RTCOUNT_MAX;
    std::sort(rtcount_table, rtcount_table + n, [](const rtcount_entry &a, const rtcount_entry &b) {
        return a.stat.max > b.stat.max;
    });

    for (int i = 0; i < n; ++i) {
        const auto &k = rtcount_table[i].key;
        const auto &stat = rtcount_table[i].stat;
        UART_LOG("%-30s %9lu %9.0f %8.3f %8.3f %6lu %8.3f %6lu", k, stat.num, stat.total / mhz,
                 (float) stat.total / stat.num / mhz,
                 stat.max / mhz, stat.max_num, stat.min / mhz, stat.min_num);
    }
    UART_LOG("\n");
    if (reset)
        rtcount_count.store(0, std::memory_order_release);

    rtcount_en = true;
}


#if CONFIG_FUGU_RT_ALLOC_COUNT
#include <esp_heap_caps.h>
#if CONFIG_FUGU_RT_ALLOC_TRACE
#include <esp_debug_helpers.h>
#include <esp_cpu_utils.h>
#endif

static TaskHandle_t s_rtAllocTask = nullptr;
static volatile uint32_t s_rtAllocs = 0, s_rtFrees = 0, s_rtAllocLastMs = 0;

#if CONFIG_FUGU_RT_ALLOC_TRACE
struct RtAllocSite {
    uint32_t pc[8]; // return addresses, innermost first (the hook and heap_caps frames included)
    uint32_t n, size, firstMs;
    bool free;
};
static constexpr uint8_t kRtAllocSites = 8;
static RtAllocSite s_rtAllocSite[kRtAllocSites];
static uint8_t s_rtAllocNumSites = 0;

static IRAM_ATTR void rtAllocTrace(bool isFree, size_t size, uint32_t ms) {
    uint32_t pc[8]{};
    esp_backtrace_frame_t f{};
    esp_backtrace_get_start(&f.pc, &f.sp, &f.next_pc);
    for (int i = 0; i < 8; ++i) {
        pc[i] = esp_cpu_process_stack_pc(f.pc);
        if (!f.next_pc || !esp_backtrace_get_next_frame(&f)) break;
    }
    for (uint8_t i = 0; i < s_rtAllocNumSites; ++i) {
        auto &st = s_rtAllocSite[i];
        if (st.free == isFree && !memcmp(st.pc, pc, sizeof(pc))) {
            ++st.n;
            return;
        }
    }
    if (s_rtAllocNumSites >= kRtAllocSites) return;
    auto &st = s_rtAllocSite[s_rtAllocNumSites];
    memcpy(st.pc, pc, sizeof(pc));
    st.n = 1;
    st.size = size;
    st.firstMs = ms;
    st.free = isFree;
    ++s_rtAllocNumSites;
}
#endif

static IRAM_ATTR inline bool rtAllocMine() {
    return s_rtAllocTask && xPortGetCoreID() == RT_CORE && xTaskGetCurrentTaskHandle() == s_rtAllocTask;
}

// placement (HEAP_IRAM_ATTR) comes from the declaration in esp_heap_caps.h
extern "C" void esp_heap_trace_alloc_hook(void *, size_t size, uint32_t) {
    if (!rtAllocMine()) return;
    s_rtAllocs = s_rtAllocs + 1;
    s_rtAllocLastMs = (uint32_t) (esp_timer_get_time() / 1000);
#if CONFIG_FUGU_RT_ALLOC_TRACE
    rtAllocTrace(false, size, s_rtAllocLastMs);
#else
    (void) size;
#endif
}

extern "C" void esp_heap_trace_free_hook(void *) {
    if (!rtAllocMine()) return;
    s_rtFrees = s_rtFrees + 1;
    s_rtAllocLastMs = (uint32_t) (esp_timer_get_time() / 1000);
#if CONFIG_FUGU_RT_ALLOC_TRACE
    rtAllocTrace(true, 0, s_rtAllocLastMs);
#endif
}

void rtAllocBegin() { s_rtAllocTask = xTaskGetCurrentTaskHandle(); }

void rtAllocPrint() {
    static uint32_t prevAllocs = 0, prevFrees = 0;
    const uint32_t a = s_rtAllocs, f = s_rtFrees;
    UART_LOG("rt-alloc: %lu allocs, %lu frees by loopRt (+%lu/+%lu since last print), last at %lu ms",
             (unsigned long) a, (unsigned long) f, (unsigned long) (a - prevAllocs), (unsigned long) (f - prevFrees),
             (unsigned long) s_rtAllocLastMs);
    prevAllocs = a;
    prevFrees = f;
#if CONFIG_FUGU_RT_ALLOC_TRACE
    for (uint8_t i = 0; i < s_rtAllocNumSites; ++i) {
        const auto &st = s_rtAllocSite[i];
        UART_LOG("  #%u %s size=%lu n=%lu first=%lu ms: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx", i,
                 st.free ? "free " : "alloc", (unsigned long) st.size, (unsigned long) st.n, (unsigned long) st.firstMs,
                 st.pc[0], st.pc[1], st.pc[2], st.pc[3], st.pc[4], st.pc[5], st.pc[6], st.pc[7]);
    }
    if (s_rtAllocNumSites)
        UART_LOG("  resolve: xtensa-esp32s3-elf-addr2line -pfiaC -e build/fugu-firmware.elf <addresses>");
#endif
}
#else
void rtAllocBegin() {}
void rtAllocPrint() {}
#endif

void rtcount_test_cycle_counter() {
    auto t0 = micros();
    auto c0 = esp_cpu_get_cycle_count();
    vTaskDelay(100);
    auto c1 = esp_cpu_get_cycle_count();
    auto t1 = micros();
    int cpUs = (c1 - c0) / (t1 - t0);
    ESP_LOGD("rtcount", "dt=%lu dc=%lu cycles/s=%i", t1 - t0, c1 - c0, cpUs);
    assert(abs(cpUs - CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) < CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 0.05f);
}