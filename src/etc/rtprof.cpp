#include "rtprof.h"

#include <cstring>
#include <driver/gptimer.h>
#include <esp_app_desc.h>
#include <esp_attr.h>
#include <esp_debug_helpers.h>
#include <esp_rom_sys.h>
#include <esp_cpu_utils.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <xtensa_context.h>

#include "logging.h"
#include <fugu_flash_chip.h>

static constexpr uint32_t N = CONFIG_FUGU_RTPROF_SLOTS;
static_assert(N >= 64 && (N & (N - 1)) == 0, "FUGU_RTPROF_SLOTS must be a power of two");
static DRAM_ATTR uint32_t s_pc[N], s_n[N];
static DRAM_ATTR volatile uint32_t s_samples, s_lost, s_task;
static unsigned s_hz;
static DRAM_ATTR volatile bool s_all, s_flash, s_hold;

static inline bool inFlash(uint32_t pc) { return s_all || (pc >= 0x42000000u && pc < 0x44000000u); }

static IRAM_ATTR void put(uint32_t pc) {
    uint32_t h = ((pc >> 2) * 2654435761u) >> (32 - __builtin_ctz(N));
    for (int k = 0; k < 16; ++k, h = (h + 1) & (N - 1)) {
        if (s_pc[h] == pc) { ++s_n[h]; return; }
        if (!s_pc[h]) { s_pc[h] = pc; s_n[h] = 1; return; }
    }
    ++s_lost;
}

static IRAM_ATTR bool onAlarm(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *) {
    if (s_hold || (s_flash && !(fugu_flash_seq & 1))) return false;
    ++s_samples;
    auto tcb = (void **) xTaskGetCurrentTaskHandleForCore(1);
    if (!tcb) return false;
    auto fr = (const XtExcFrame *) tcb[0]; // pxTopOfStack = interrupted frame (saved on interrupt entry)
    if (!fr) return false;
    ++s_task;
    if (inFlash((uint32_t) fr->pc)) put((uint32_t) fr->pc);
    esp_backtrace_frame_t f = {.pc = (uint32_t) fr->pc, .sp = (uint32_t) fr->a1, .next_pc = (uint32_t) fr->a0,
                               .exc_frame = nullptr};
    for (int d = 0; d < 24 && f.next_pc; ++d) {
        if (!esp_backtrace_get_next_frame(&f)) break;
        uint32_t pc = esp_cpu_process_stack_pc(f.pc);
        if (inFlash(pc)) put(pc);
    }
    return false;
}

static void startTask(void *) {
    gptimer_handle_t t = nullptr;
    gptimer_config_t c = {};
    c.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    c.direction = GPTIMER_COUNT_UP;
    c.resolution_hz = 1000000;
    gptimer_alarm_config_t a = {};
    a.alarm_count = 1000000 / s_hz;
    a.flags.auto_reload_on_alarm = 1;
    gptimer_event_callbacks_t cb = {};
    cb.on_alarm = onAlarm;
    if (gptimer_new_timer(&c, &t) != ESP_OK || gptimer_register_event_callbacks(t, &cb, nullptr) != ESP_OK
        || gptimer_enable(t) != ESP_OK || gptimer_set_alarm_action(t, &a) != ESP_OK || gptimer_start(t) != ESP_OK)
        ESP_LOGE("rtprof", "timer setup failed");
    else
        ESP_LOGI("rtprof", "sampling core 1 at %u Hz", s_hz);
    vTaskDelete(nullptr);
}

void rtprofStart(unsigned hz) {
    s_hz = hz ? hz : 2000;
    xTaskCreatePinnedToCore(startTask, "rtprof", 3072, nullptr, 1, nullptr, 1);
}

// lets a sample that passed the s_hold check finish (24-frame unwind) before the table is read or cleared
static void holdOn() {
    s_hold = true;
    esp_rom_delay_us(200);
}

void rtprofClear(bool all, bool flash) {
    holdOn();
    s_all = all || flash;
    s_flash = flash;
    memset(s_pc, 0, sizeof(s_pc));
    memset(s_n, 0, sizeof(s_n));
    s_samples = s_lost = s_task = 0;
    s_hold = false;
}

void rtprofDump() {
    holdOn(); // a table growing during the print no longer matches the header
    unsigned used = 0;
    for (auto p : s_pc) used += p != 0;
    char sha[9];
    esp_app_get_elf_sha256(sha, sizeof(sha));
    UART_LOG("rtprof samples=%lu task=%lu lost=%lu entries=%u elf=%s", (unsigned long) s_samples,
             (unsigned long) s_task, (unsigned long) s_lost, used, sha);
    char line[200];
    int l = 0;
    for (uint32_t i = 0; i < N; ++i) {
        if (!s_pc[i]) continue;
        l += snprintf(line + l, sizeof(line) - l, " %lx:%lu", (unsigned long) s_pc[i], (unsigned long) s_n[i]);
        if (l > (int) sizeof(line) - 24) {
            UART_LOG("rtp%s", line);
            l = 0;
        }
    }
    if (l) UART_LOG("rtp%s", line);
    UART_LOG("rtprof end");
    s_hold = false;
}
