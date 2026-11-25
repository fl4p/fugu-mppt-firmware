#include "heap_diag.h"

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#ifdef __XTENSA__
#include <esp_debug_helpers.h>
#include <esp_cpu_utils.h>
#endif

#include "net_guard.h"

// Last failed allocation. Written by the hook on any task/core, read under s_mux by the printer.
struct AllocFail {
    uint32_t size, caps, freeB;
    const char *fn;
    char task[configMAX_TASK_NAME_LEN];
    uint32_t pc[6];
};

static AllocFail s_fail;
static volatile uint32_t s_failCount = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Runs inside the failing malloc (no heap lock held, possibly on the RT core): no allocation, no
// blocking, only a short spinlock section.
static void onAllocFailed(size_t size, uint32_t caps, const char *fn) {
    AllocFail f;
    f.size = size;
    f.caps = caps;
    f.fn = fn;
    f.freeB = heap_caps_get_free_size(MALLOC_CAP_DEFAULT); // reads counters, no lock
    const char *t = pcTaskGetName(nullptr);
    size_t i = 0;
    for (; t && t[i] && i + 1 < sizeof(f.task); ++i) f.task[i] = t[i];
    f.task[i] = 0;
    for (auto &p: f.pc) p = 0;
#ifdef __XTENSA__
    esp_backtrace_frame_t fr;
    esp_backtrace_get_start(&fr.pc, &fr.sp, &fr.next_pc);
    // skip heap_caps_alloc_failed and the heap_caps_malloc* frame
    for (int k = 0, j = 0; j < 6 && esp_backtrace_get_next_frame(&fr); ++k)
        if (k >= 2) f.pc[j++] = esp_cpu_process_stack_pc(fr.pc);
#endif
    portENTER_CRITICAL_SAFE(&s_mux);
    s_fail = f;
    s_failCount = s_failCount + 1;
    portEXIT_CRITICAL_SAFE(&s_mux);
}

void heapDiagBegin() {
    heap_caps_register_failed_alloc_callback(onAllocFailed);
}

void heapDiagLf(uint32_t nowMs) {
    static uint32_t reported = 0;
    static LogLimiter failLog{5, 10000}, lowLog{1, 60000};

    if (const uint32_t n = s_failCount; n != reported && failLog.allow(nowMs)) {
        AllocFail f;
        portENTER_CRITICAL(&s_mux);
        f = s_fail;
        portEXIT_CRITICAL(&s_mux);
        ESP_LOGE("heap", "alloc failed (%lu new, %lu total): %lu B caps 0x%lx %s task %s free %lu, now free %u largest %u"
                 " bt 0x%08lx 0x%08lx 0x%08lx 0x%08lx 0x%08lx 0x%08lx",
                 (unsigned long) (n - reported), (unsigned long) n, (unsigned long) f.size, (unsigned long) f.caps,
                 f.fn ? f.fn : "?", f.task, (unsigned long) f.freeB, (unsigned) heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                 (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
                 (unsigned long) f.pc[0], (unsigned long) f.pc[1], (unsigned long) f.pc[2], (unsigned long) f.pc[3],
                 (unsigned long) f.pc[4], (unsigned long) f.pc[5]);
        reported = n;
    }

    const size_t freeB = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    if (freeB < 8 * 1024 && lowLog.allow(nowMs))
        ESP_LOGW("heap", "low heap: free %u min %u largest %u", (unsigned) freeB,
                 (unsigned) heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT),
                 (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

#if CONFIG_FUGU_PBUF_PROBE
#include <cstring>
#include <esp_attr.h>
#include <esp_memory_utils.h>
#include <lwip/pbuf.h>

// Filled once, right before lwIP aborts on a pbuf whose ref count is already 0; the core dump
// includes it (gdb: p/x g_pbufProbe). mem holds [p - 128, p + 256): TLSF header at p - 8, the
// struct pbuf at p, then its payload.
struct PbufProbe {
    uint32_t magic; // 'PBF0' once filled
    uint32_t p, caller, mem0, freeB, largest;
    int32_t heapOk; // heap_caps_check_integrity_all(); errors also go to the UART
    char task[configMAX_TASK_NAME_LEN];
    uint8_t mem[384];
};
COREDUMP_DRAM_ATTR PbufProbe g_pbufProbe;

static void __attribute__((noinline, cold)) pbufProbeStash(const struct pbuf *p, void *caller) {
    auto &s = g_pbufProbe;
    s.p = (uint32_t) p;
    s.caller = (uint32_t) caller;
    const char *t = pcTaskGetName(nullptr);
    strlcpy(s.task, t ? t : "?", sizeof(s.task));
    const auto *m = (const uint8_t *) p - 128;
    if (esp_ptr_in_dram(m) && esp_ptr_in_dram(m + sizeof(s.mem) - 1)) {
        memcpy(s.mem, m, sizeof(s.mem));
        s.mem0 = (uint32_t) m;
    }
    s.freeB = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    s.largest = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    s.magic = 0x30464250; // set before the scan, which may itself abort on a broken heap
    s.heapOk = heap_caps_check_integrity_all(true);
}

extern "C" u8_t __real_pbuf_free(struct pbuf *p);

extern "C" u8_t __wrap_pbuf_free(struct pbuf *p) {
    if (__builtin_expect(p && !p->ref, 0)) pbufProbeStash(p, __builtin_return_address(0));
    return __real_pbuf_free(p);
}
#endif
