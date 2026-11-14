#include "logging.h"

#include <atomic>
#include <cstring>
#include <esp_attr.h>
#include <new>
#include <ESPTelnet.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>   // pcTaskGetName (wifi-task bypass in vprintf_)
#include <esp_timer.h>

// without the queue's debug-only reentrancy guard: its out-of-line constructor would run from flash on the RT core
#pragma push_macro("NDEBUG")
#undef NDEBUG
#define NDEBUG
#include "etc/readerwriterqueue.h"
#pragma pop_macro("NDEBUG")
#include <cassert>
#include "util.h"
#include <soc/soc.h>


//struct ConcurrentQueueMinMemTraits : public moodycamel::ConcurrentQueueDefaultTraits {
//    static const size_t BLOCK_SIZE = 2;
//   static const size_t INITIAL_IMPLICIT_PRODUCER_HASH_SIZE = 0;
//};

static int vprintf_(const char *fmt, va_list argptr);

static int vprintf_mux(const char *fmt, va_list argptr);

bool g_rawLogForce = false;

#ifdef __XTENSA__
// A log call captured without formatting: the format pointer and the argument words of its va_list.
struct RawLog {
    const char *fmt;
    uint32_t us;
    int32_t ndx;
    uint32_t reg[6];  // register-passed argument area
    uint32_t stk[32]; // stack-passed arguments
    char str[80];     // copies of %s arguments
};

// layout of the Xtensa (windowed ABI) va_list, which C++ cannot access by member
struct XtVaList {
    int32_t *stk; // stack arguments, the first one at index 32
    int32_t *reg; // the six register-passed words
    int32_t ndx;  // byte index of the next argument
};
static_assert(sizeof(va_list) == sizeof(XtVaList));

enum RawLogFlags : uint8_t { RawLogSet = 1, RawLogBreak = 2, RawLogStamp = 4, RawLogStrings = 8 };

// %s arguments that do not point into flash: copy the string into the capture (copy, on the logging
// core while it is still valid) or, when that was skipped, drop it (the memory may be gone by now).
// No libc calls and no tables: this also runs on the RT core.
static IRAM_ATTR void rawLogStrings(RawLog &r, bool copy) {
    static DRAM_ATTR char gone[] = "?";
    int32_t ndx = r.ndx;
    size_t used = 0;
    auto arg = [&](int size) -> uint32_t * {
        if (size == 8) ndx = (ndx + 7) & ~7;
        ndx += size;
        if (ndx > 24 && ndx < 32 + size) ndx = 32 + size;
        const int32_t at = ndx - size;
        if (at < 24) return &r.reg[at / 4];
        const int32_t i = (at - 32) / 4;
        return i >= 0 && i + size / 4 <= (int32_t) (sizeof(r.stk) / 4) ? &r.stk[i] : nullptr;
    };
    for (const char *p = r.fmt; *p; ++p) {
        if (*p != '%') continue;
        if (*++p == '%') continue;
        int wide = 0; // ll, j, L: an 8-byte argument
        for (char c; (c = *p) && (c == '-' || c == '+' || c == ' ' || c == '#' || c == '.' || c == '*'
                                  || (c >= '0' && c <= '9') || c == 'l' || c == 'h' || c == 'z' || c == 'j' || c == 't'
                                  || c == 'L'); ++p) {
            if (c == '*') arg(4);
            else if (c == 'l') ++wide;
            else if (c == 'j' || c == 'L') wide = 2;
        }
        if (!*p) break;
        const char c = (char) (*p | 0x20);
        if (c == 'f' || c == 'e' || c == 'g' || c == 'a' || (wide >= 2 && *p != 's')) arg(8);
        else if (*p == 's') {
            uint32_t *a = arg(4);
            if (!a || !*a || (*a >= SOC_DROM_LOW && *a < SOC_DROM_HIGH)) continue;
            const char *src = (const char *) *a;
            *a = (uint32_t) gone;
            if (!copy || used + 2 > sizeof(r.str)) continue;
            *a = (uint32_t) &r.str[used];
            while (*src && used + 1 < sizeof(r.str)) r.str[used++] = *src++;
            r.str[used++] = 0;
        } else arg(4);
    }
}

static int rawLogFormat(RawLog &r, uint8_t flags, char *out, size_t n) {
    if (!(flags & RawLogStrings)) rawLogStrings(r, false);
    int len = (flags & RawLogStamp) ? snprintf(out, n, "(%lu): ", (unsigned long) r.us) : 0;
    const XtVaList x{(int32_t *) ((char *) r.stk - 32), (int32_t *) r.reg, r.ndx};
    va_list ap;
    memcpy(&ap, &x, sizeof(ap));
    const size_t cap = n - len - 2;
    int l = vsnprintf(out + len, cap, r.fmt, ap);
    if (l < 0) l = 0;
    if ((size_t) l >= cap) l = (int) cap - 1;
    len += l;
    if (flags & RawLogBreak) out[len++] = '\n';
    out[len] = 0;
    return len;
}
#endif

// RT-core log lines: a fixed ring filled on core 1 (no heap), drained in order by the core-0 flush.
// A slot is claimed with interrupts masked (an ISR on core 1 may log too) and published by `flags`.
struct RtLogSlot {
    std::atomic<uint8_t> flags; // RawLogFlags, 0 = free or being written
#ifdef __XTENSA__
    RawLog r;
#else
    uint16_t len;
    char str[230];
#endif
};
static constexpr uint32_t kRtLogSlots = 16;
static RtLogSlot s_rtLog[kRtLogSlots];
static std::atomic<uint32_t> s_rtLogHead{0}, s_rtLogTail{0};
static volatile uint32_t s_rtLogDropped = 0;

static IRAM_ATTR RtLogSlot *rtLogClaim() {
    const UBaseType_t irq = portSET_INTERRUPT_MASK_FROM_ISR();
    const uint32_t h = s_rtLogHead.load(std::memory_order_relaxed);
    RtLogSlot *slot = nullptr;
    if (h - s_rtLogTail.load(std::memory_order_acquire) < kRtLogSlots) {
        slot = &s_rtLog[h % kRtLogSlots];
        s_rtLogHead.store(h + 1, std::memory_order_release);
    } else {
        s_rtLogDropped = s_rtLogDropped + 1;
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
    return slot;
}

ESPTelnet *log_telnet = nullptr;

// Up to kMaxLogCallbacks sinks. Registered from the MQTT/BLE tasks, iterated from any core-0
// logging context (vprintf_mux) + the network loop (flush). logCbMux guards the array; readers
// snapshot it under the lock and invoke the callbacks outside (a callback may log -> re-enter).
static constexpr int kMaxLogCallbacks = 4;
static LogCallback logCallbacks[kMaxLogCallbacks] = {};
static int logCallbackCount = 0;
static portMUX_TYPE logCbMux = portMUX_INITIALIZER_UNLOCKED;

// Boot-log backlog: captures early log lines (before any remote sink exists) so the first sink to
// attach — typically MQTT, which can't connect until WiFi is up, well after setup() logs — gets the
// boot sequence replayed. Heap buffer, allocated on the first captured line; closes when full and is
// freed on first attach or after kBootLogKeepUs. Guarded by logCbMux. mqtt-tagged lines are skipped
// so the one-shot replay isn't dropped by mqttLogCallback's ") mqtt:" filter.
static constexpr size_t kBootLogSize = 8192;
static constexpr int64_t kBootLogKeepUs = 300 * 1000000LL;
static char *s_bootLog = nullptr;
static size_t s_bootLogLen = 0;
static bool s_bootLogOpen = true;

static void bootLogCapture(const char *str, int len) {
    if (!s_bootLogOpen || len <= 0) return;
    if (strnstr(str, ") mqtt:", len)) return;
    char *fresh = nullptr;
    if (!s_bootLog) {
        if (!xPortCanYield()) return;
        fresh = (char *) malloc(kBootLogSize);
    }
    portENTER_CRITICAL(&logCbMux);
    if (s_bootLogOpen && !s_bootLog) {
        s_bootLog = fresh;
        fresh = nullptr;
        if (!s_bootLog) s_bootLogOpen = false;
    }
    if (s_bootLogOpen) {
        // Hard guard: if s_bootLogLen is ever out of range (corrupted by an external wild write),
        // the room/`s_bootLog + s_bootLogLen` math would scribble a log line at a wild address.
        if (s_bootLogLen > kBootLogSize) {
            s_bootLogOpen = false;
        } else {
            size_t room = kBootLogSize - s_bootLogLen;
            if ((size_t) len > room) { len = (int) room; s_bootLogOpen = false; } // fill remainder, then close
            memcpy(s_bootLog + s_bootLogLen, str, len);
            s_bootLogLen += len;
        }
    }
    portEXIT_CRITICAL(&logCbMux);
    free(fresh);
}

// Close the backlog and detach its buffer (caller frees). Call under logCbMux.
static char *bootLogTake() {
    s_bootLogOpen = false;
    char *b = s_bootLog;
    s_bootLog = nullptr;
    return b;
}

// Retained lines: a result that must survive a sink outage (e.g. the MQTT session dropping with the
// same network fault that failed an OTA). Replayed to every sink attaching within kRetainUs.
static constexpr int64_t kRetainUs = 120 * 1000000LL;
static char s_retain[256];
static size_t s_retainLen = 0;
static int64_t s_retainUs = 0;

static size_t copyRetained(char *out) {
    portENTER_CRITICAL(&logCbMux);
    size_t n = esp_timer_get_time() - s_retainUs < kRetainUs ? s_retainLen : 0;
    memcpy(out, s_retain, n);
    portEXIT_CRITICAL(&logCbMux);
    return n;
}

// Copy the sink table into `out` (must hold kMaxLogCallbacks) under the lock; returns the count.
static int snapshotLogCallbacks(LogCallback *out) {
    portENTER_CRITICAL(&logCbMux);
    int n = logCallbackCount;
    for (int i = 0; i < n; ++i) out[i] = logCallbacks[i];
    portEXIT_CRITICAL(&logCbMux);
    return n;
}

vprintf_like_t old_vprintf = &vprintf;

bool deferLogs = false;

void loggingEnableDefer() {
    deferLogs = true;
}

void addLogCallback(LogCallback callback, bool replayBootLog) {
    if (!callback) return;
    bool full = false;
    size_t replayLen = 0;
    portENTER_CRITICAL(&logCbMux);
    bool dup = false;
    for (int i = 0; i < logCallbackCount; ++i)
        if (logCallbacks[i] == callback) { dup = true; break; }
    if (!dup) {
        if (logCallbackCount >= kMaxLogCallbacks) full = true;
        else logCallbacks[logCallbackCount++] = callback;
    }
    // Close the backlog on any first attach (a later sink must not capture a partial tail),
    // but only hand it to sinks that asked for it.
    char *boot = nullptr;
    if (!dup && !full) {
        boot = bootLogTake();
        if (replayBootLog && boot) replayLen = s_bootLogLen;
    }
    portEXIT_CRITICAL(&logCbMux);
    if (full) ESP_LOGW("log", "log callback table full, dropping"); // log outside the lock
    // Replay outside the lock; the buffer is detached, so nothing writes it. One call = one MQTT message.
    if (replayLen) callback(boot, (uint16_t) replayLen);
    free(boot);
    if (!dup && !full) {
        char r[sizeof(s_retain)];
        if (size_t n = copyRetained(r)) callback(r, (uint16_t) n);
    }
}

void logRetained(const char *fmt, ...) {
    char b[sizeof(s_retain)];
    va_list args;
    va_start(args, fmt);
    int l = vsnprintf(b, sizeof(b) - 1, fmt, args);
    va_end(args);
    if (l <= 0) return;
    if (l > (int) sizeof(b) - 2) l = sizeof(b) - 2;
    b[l++] = '\n';
    b[l] = 0;
    const auto now = esp_timer_get_time();
    portENTER_CRITICAL(&logCbMux);
    if (now - s_retainUs >= kRetainUs || s_retainLen + l > sizeof(s_retain)) s_retainLen = 0;
    memcpy(s_retain + s_retainLen, b, l);
    s_retainLen += l;
    s_retainUs = now;
    portEXIT_CRITICAL(&logCbMux);
    printf_mux("%s", b);
}

void removeLogCallback(LogCallback callback) {
    portENTER_CRITICAL(&logCbMux);
    for (int i = 0; i < logCallbackCount; ++i) {
        if (logCallbacks[i] == callback) {
            logCallbacks[i] = logCallbacks[--logCallbackCount]; // compact (order irrelevant)
            logCallbacks[logCallbackCount] = nullptr;
            break;
        }
    }
    portEXIT_CRITICAL(&logCbMux);
}


// IRAM: the RT core logs through here while a flash op runs
__attribute__((flatten)) static IRAM_ATTR int enqueue_log(const char *fmt, size_t l, const va_list &args, bool appendBreak = false, bool timestamp = false) {
    assert((xPortGetCoreID() == 1)); // ensure RT core

    RtLogSlot *slot = rtLogClaim();
    if (!slot) return -1;
#ifdef __XTENSA__
    // Never format here: float formatting costs ~0.4 ms, and runs from flash. The flush on core 0 does it.
    RawLog *r = &slot->r;
    r->fmt = fmt;
    r->us = micros();
    XtVaList x;
    memcpy(&x, &args, sizeof(x));
    r->ndx = x.ndx;
    memcpy(r->reg, x.reg, sizeof(r->reg));
    memcpy(r->stk, (const char *) x.stk + 32, sizeof(r->stk));
    // reading the format string is a flash data access: not while a flash op runs
    const bool strings = !flashOpActive() && !g_rawLogForce;
    if (strings) rawLogStrings(*r, true);
    slot->flags.store((uint8_t) (RawLogSet | (appendBreak ? RawLogBreak : 0) | (timestamp ? RawLogStamp : 0)
                                 | (strings ? RawLogStrings : 0)), std::memory_order_release);
    return 0;
#else
    (void) l;
    char *buf = slot->str;
    const size_t n = sizeof(slot->str);
    int len = 0;
    if (timestamp) {
        len = snprintf(buf, n, "(%lu): ", micros());
        if (len < 0) len = 0;
    }
    size_t cap = n - len - size_t(appendBreak);
    auto r2 = vsnprintf(buf + len, cap, fmt, args);
    if (r2 < 0) r2 = 0;
    // vsnprintf returns the untruncated length; clamp so buf[len]='\n' stays in bounds.
    if ((size_t) r2 >= cap) r2 = (int) cap - 1;
    len += r2;
    if (appendBreak) {
        buf[len] = '\n';
        buf[len + 1] = 0;
        len += 1;
    }
    slot->len = (uint16_t) len;
    slot->flags.store(1, std::memory_order_release);
    return len;
#endif
}

/*
void enqueue_telnet_log(const char *s, int len) {
    assert((xPortGetCoreID() == 1));

    if (uart_async_log_queue.size_approx() > 64) return;
    auto buf = new char[len + 1];
    strncpy(buf, s, len + 1);
    uart_async_log_queue.enqueue(AsyncLogEntry{buf, (uint16_t) len, true});
}
*/
/*
void enqueue_telnet_log(const char *fmt, size_t l, const va_list &args) {
    assert((xPortGetCoreID() == 1));

    if (uart_async_log_queue.size_approx() > 64) return;
    auto buf = new char[l + 1];
    auto len = vsnprintf(buf, l + 1, fmt, args);
    uart_async_log_queue.enqueue(AsyncLogEntry{buf, (uint16_t) len, true});
}*/


IRAM_ATTR void UART_LOG(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    if (xPortGetCoreID() == 1 && deferLogs) {
        // RT core1: defer all log to core0
        enqueue_log(fmt, 200, args, true, true);
    } else {
        vprintf_mux(fmt, args);
        vprintf_mux("\n", va_list{});
    }
    va_end(args);
}

IRAM_ATTR void printf_mux(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    if (xPortGetCoreID() == 1 && deferLogs) {
        // RT core1: defer all log to core0
        enqueue_log(fmt, 200, args, false);
    } else {
        vprintf_mux(fmt, args);
    }
    va_end(args);
}


/*
void UART_LOG_ASYNC(const char *fmt, ...) {

    assert (xPortGetCoreID() == 1);
    //UART_LOG(fmt, ...);

    if (uart_async_log_queue.size_approx() > 200)
        return;



    auto buf = new char[384];

    va_list args;
    va_start(args, fmt);
    uint16_t l = vsnprintf(buf, 380, fmt, args);
    va_end(args);
    buf[l++] = '\r';
    buf[l++] = '\n';
    buf[l] = '\0';

    uart_async_log_queue.enqueue(AsyncLogEntry{buf, l, false});

    / * if (uart_async_log_queue.size() > 200) {
         delete[] uart_async_log_queue.front().str;
         uart_async_log_queue.pop_front(); // TODO fix race, use lock-free queu
         // e.g. https://github.com/cameron314/concurrentqueue/blob/master/concurrentqueue.h
     }* /
}
*/

static int printf_old(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int l = old_vprintf(fmt, args);
    va_end(args);
    return l;
}

void flush_async_uart_log() {
    // Cap entries drained per call so the network loop can pet the TWDT.
    // printf_old() blocks in uart_tx_char waiting on FIFO room: at 115200 baud a 60-byte
    // line costs ~5 ms, and if the RT core keeps enqueueing faster than UART drains (e.g.
    // a vconv saturation that spams Vin-OV warnings every sample), the while-loop never
    // sees an empty queue and core-0 sits inside flush past TASK_WDT_TIMEOUT_S → reset.
    // 32 entries ≈ 160 ms worst-case; the remainder drains on the next loop tick after
    // vTaskDelay(1). A full ring drops on the RT side.
    constexpr int kMaxPerCall = 32;
    for (int i = 0; i < kMaxPerCall; ++i) {
        const uint32_t t = s_rtLogTail.load(std::memory_order_relaxed);
        if (t == s_rtLogHead.load(std::memory_order_acquire)) break;
        RtLogSlot &slot = s_rtLog[t % kRtLogSlots];
        const uint8_t flags = slot.flags.load(std::memory_order_acquire);
        if (!flags) break; // claimed, still being written
#ifdef __XTENSA__
        char buf[224];
        const char *str = buf;
        const uint16_t len = (uint16_t) rawLogFormat(slot.r, flags, buf, sizeof(buf));
#else
        const char *str = slot.str;
        const uint16_t len = slot.len;
#endif
        printf_old("%s", str); // str is data, not a format string (it may contain %)

        if (log_telnet)
            log_telnet->write((uint8_t *) str, len);

        LogCallback cbs[kMaxLogCallbacks];
        int n = snapshotLogCallbacks(cbs);
        for (int k = 0; k < n; ++k) cbs[k](str, len);

        bootLogCapture(str, len);

        slot.flags.store(0, std::memory_order_relaxed);
        s_rtLogTail.store(t + 1, std::memory_order_release);
    }
    if (s_bootLog && esp_timer_get_time() > kBootLogKeepUs) { // no sink attached in time
        portENTER_CRITICAL(&logCbMux);
        char *b = bootLogTake();
        portEXIT_CRITICAL(&logCbMux);
        free(b);
    }
    static uint32_t reported = 0;
    if (const uint32_t d = s_rtLogDropped; d != reported) {
        reported = d;
        ESP_LOGW("log", "%lu RT log lines dropped (ring full)", (unsigned long) d);
    }
}

/**
 * Synchronous printf to UART, JTAG USB and telnet
 * @param fmt
 * @param argptr
 * @return
 */
static int vprintf_mux(const char *fmt, va_list argptr) {
    char loc_buf[300]; // stack-local: vprintf_mux re-enters across core-0 tasks (net loop + mqtt), a static races

    // Format once, then reuse the string for UART + every mirror sink. Formatting twice (one
    // vsnprintf for UART, another for the mirror) overflowed the 3072-byte ESP-IDF wifi task during
    // auth, since log emission runs on the caller's stack. va_copy before the first vsnprintf: it
    // spends argptr, and the long-line refmt below needs the args again.
    va_list ap2;
    va_copy(ap2, argptr);
    int l = vsnprintf(loc_buf, sizeof(loc_buf), fmt, argptr);
    if (l <= 0) {
        va_end(ap2);
        return l;
    }

    char *buf = loc_buf;
    // vsnprintf returns the untruncated length; reading l from loc_buf would over-read the stack.
    if (l >= (int) sizeof(loc_buf)) {
        buf = (char *) malloc(l + 1);
        if (buf) vsnprintf(buf, l + 1, fmt, ap2);
        else { buf = loc_buf; l = sizeof(loc_buf) - 1; }
    }
    va_end(ap2);

    printf_old("%s", buf); // UART/JTAG console (always)

    if (log_telnet or logCallbackCount or s_bootLogOpen) {
        if (log_telnet) log_telnet->write((uint8_t *) buf, l);
        LogCallback cbs[kMaxLogCallbacks];
        int n = snapshotLogCallbacks(cbs);
        for (int i = 0; i < n; ++i) cbs[i](buf, l);
        bootLogCapture(buf, l);
    }

    if (buf != loc_buf) free(buf);
    return l;
}


static IRAM_ATTR int vprintf_(const char *fmt, va_list argptr) {
    // Defer to the core0 flush only FROM core1 (the RT core, or when it can't yield — critical
    // section/ISR). enqueue_log() asserts core1, so core0 must always take the synchronous
    // vprintf_mux path; this also makes it safe to route esp_log here from the very start of setup()
    // (so boot-log capture sees the setup() body). core1 behaviour is unchanged.
    if (xPortGetCoreID() == 1 && (deferLogs || !xPortCanYield())) {
        return enqueue_log(fmt, 200, argptr);
    }
    // The ESP-IDF wifi task has only a 3072B stack; vprintf_mux's 300B loc_buf + mirror-sink frames
    // overflow it on connect/reconnect once this hook is active (post-setup reconnects bricked
    // devices that couldn't immediately associate). Route the wifi task to the light default vprintf
    // (UART only) so a reconnect can't crash it. Guarded to task context (pcTaskGetName is not ISR-safe).
    if (xPortCanYield()) {
        const char *tn = pcTaskGetName(nullptr);
        if (tn && strncmp(tn, "wifi", 4) == 0)
            return old_vprintf(fmt, argptr);
    }
    return vprintf_mux(fmt, argptr);
}

void logCmdResult(bool ok, const char *cmd, int64_t startUs) {
    if (esp_timer_get_time() - startUs > 5000000) logRetained("%s: %s", ok ? "OK" : "ERR", cmd);
    else UART_LOG("%s: %s", ok ? "OK" : "ERR", cmd);
}

void set_logging_telnet(ESPTelnet *telnet) {
    if (telnet && !log_telnet) {
        char r[sizeof(s_retain)];
        if (size_t n = copyRetained(r)) telnet->write((uint8_t *) r, n);
    }
    log_telnet = telnet;
    ESP_LOGI("log", "%s telnet logging", telnet ? "Enabled" : "Disabled");
}

void enable_esp_log_to_telnet() {
    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/log.html
    old_vprintf = esp_log_set_vprintf(&vprintf_);
    if (!old_vprintf)
        old_vprintf = &vprintf;
}


class TaskQueue {
    typedef std::function<void(void)> Fn;
    moodycamel::ReaderWriterQueue<Fn> q{1};
    volatile uint32_t dropped_ = 0, reported_ = 0;

public:
    inline void add(Fn &&fn) {
        // enqueue() mallocs a new block when full: on a tight heap that fails. All callers
        // re-issue their work on later ticks — drop, never panic the RT loop over an alloc
        // (took fboost down when BLE + a log storm exhausted the heap, coredump 8-05).
        if (!q.enqueue(std::move(fn)))
            ++dropped_;
    }

    inline void work() {
        Fn fn;
        while (q.try_dequeue(fn)) {
            fn();
        }
        if (uint32_t d = dropped_; d != reported_) {
            reported_ = d;
            ESP_LOGW("log", "task queue OOM, %lu deferred tasks dropped", (unsigned long) d);
        }
    }
};

static TaskQueue tq;

void enqueue_task(std::function<void(void)> &&fn) {
    tq.add(std::move(fn));
}

// Calls handed from the RT core (or any core/ISR) to core 0 without heap: a fixed ring under a spinlock.
struct RtDeferred {
    RtDeferredFn fn;
    void *ctx;
    float a, b;
};
static constexpr uint8_t kRtDeferSlots = 8;
static RtDeferred s_rtDefer[kRtDeferSlots];
static uint8_t s_rtDeferHead = 0, s_rtDeferLen = 0;
static uint32_t s_rtDeferDropped = 0;
static portMUX_TYPE s_rtDeferMux = portMUX_INITIALIZER_UNLOCKED;

IRAM_ATTR bool rt_defer(RtDeferredFn fn, void *ctx, float a, float b) {
    portENTER_CRITICAL_SAFE(&s_rtDeferMux);
    const bool ok = s_rtDeferLen < kRtDeferSlots;
    if (ok) s_rtDefer[(s_rtDeferHead + s_rtDeferLen++) % kRtDeferSlots] = {fn, ctx, a, b};
    else ++s_rtDeferDropped;
    portEXIT_CRITICAL_SAFE(&s_rtDeferMux);
    return ok;
}

void process_queued_tasks() {
    for (;;) {
        RtDeferred d;
        portENTER_CRITICAL(&s_rtDeferMux);
        const bool any = s_rtDeferLen != 0;
        if (any) {
            d = s_rtDefer[s_rtDeferHead];
            s_rtDeferHead = (s_rtDeferHead + 1) % kRtDeferSlots;
            --s_rtDeferLen;
        }
        portEXIT_CRITICAL(&s_rtDeferMux);
        if (!any) break;
        d.fn(d.ctx, d.a, d.b);
    }
    static uint32_t reported = 0;
    if (const uint32_t n = s_rtDeferDropped; n != reported) {
        reported = n;
        ESP_LOGW("log", "%lu RT deferred calls dropped (ring full)", (unsigned long) n);
    }
    tq.work();
}
