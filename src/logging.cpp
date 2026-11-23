#include "logging.h"

#include <atomic>
#include <cstring>
#include <esp_attr.h>
#include <new>
#include <Arduino.h>
#if WITH_TELNET
#include <ESPTelnet.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>   // pcTaskGetName (wifi-task bypass in vprintf_)
#include <esp_timer.h>

#include <freertos/queue.h>
#include <memory>
#include <cassert>
#include "util.h"
#include "etc/mpsc_ring.h"
#include "etc/con_guard.h"
#include <soc/soc.h>


//struct ConcurrentQueueMinMemTraits : public moodycamel::ConcurrentQueueDefaultTraits {
//    static const size_t BLOCK_SIZE = 2;
//   static const size_t INITIAL_IMPLICIT_PRODUCER_HASH_SIZE = 0;
//};

static int vprintf_(const char *fmt, va_list argptr);

static int vprintf_mux(const char *fmt, va_list argptr);

bool g_rawLogForce = false;

// RawLogLow: an ESP_LOG I/D/V line, given up first when the ring is full
enum RawLogFlags : uint8_t { RawLogSet = 1, RawLogBreak = 2, RawLogStamp = 4, RawLogStrings = 8, RawLogLow = 16 };

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

// RT-core log lines: a fixed lock-free ring filled on any core (no heap), drained in order by the
// core-0 flush. A producer masks interrupts on its core from claim to publish.
struct RtLogLine {
#ifdef __XTENSA__
    RawLog r;
#else
    uint16_t len;
    char str[230];
#endif
};
static MpscRing<RtLogLine, 16> s_rtLog;

#if WITH_TELNET
ESPTelnet *log_telnet = nullptr;
#else
static constexpr bool log_telnet = false;
#endif

// Up to kMaxLogCallbacks sinks. Registered from the MQTT/BLE tasks, iterated from any core-0
// logging context (vprintf_mux) + the network loop (flush). logCbMux guards the array; readers
// snapshot it under the lock and invoke the callbacks outside (a callback may log -> re-enter).
static constexpr int kMaxLogCallbacks = 4;
static LogCallback logCallbacks[kMaxLogCallbacks] = {};
static int logCallbackCount = 0;
static portMUX_TYPE logCbMux = portMUX_INITIALIZER_UNLOCKED;

// Boot-log backlog: captures log lines from bootLogBegin() on, so the first sink that asks for a
// replay — typically MQTT, which can't connect until WiFi is up — gets the boot sequence.
// Heap buffer allocated there (top of setup()), so capture never allocates; closes when full, freed
// when a replay sink attaches or after kBootLogKeepUs. Sinks attaching with replayBootLog=false leave
// it alone. Guarded by logCbMux; s_bootLogOpen implies s_bootLog != nullptr. mqtt-tagged lines are
// skipped so the one-shot replay isn't dropped by mqttLogCallback's ") mqtt:" filter.
static constexpr size_t kBootLogSize = 8192;
static constexpr int64_t kBootLogKeepUs = 30 * 60 * 1000000LL;
static char *s_bootLog = nullptr;
static size_t s_bootLogLen = 0;
static bool s_bootLogOpen = false;

static void bootLogCapture(const char *str, int len) {
    if (!s_bootLogOpen || len <= 0) return;
    if (strnstr(str, ") mqtt:", len)) return;
    portENTER_CRITICAL(&logCbMux);
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
    // The first replay sink takes the backlog (a later one must not get a partial tail).
    char *boot = nullptr;
    if (!dup && !full && replayBootLog) {
        boot = bootLogTake();
        if (boot) replayLen = s_bootLogLen;
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


// IRAM: the RT core logs through here while a flash op runs. Any core: an unpinned task may have
// migrated since its core test, the ring is safe for producers on both cores.
// esplog: fmt is an ESP_LOG format ("I (%lu) %s: ..."), its first char is the level.
__attribute__((flatten)) static IRAM_ATTR int enqueue_log(const char *fmt, const va_list &args, bool appendBreak = false,
                                                         bool timestamp = false, bool esplog = false) {
    uint32_t flags = RawLogSet | (appendBreak ? RawLogBreak : 0) | (timestamp ? RawLogStamp : 0);
#ifdef __XTENSA__
    // reading the format string is a flash data access: not while a flash op runs (level unknown then)
    const bool strings = !flashOpActive() && !g_rawLogForce;
    if (strings) flags |= RawLogStrings;
#else
    const bool strings = true;
#endif
    if (strings && esplog && (fmt[0] == 'I' || fmt[0] == 'D' || fmt[0] == 'V') && fmt[1] == ' ') flags |= RawLogLow;
    const UBaseType_t irq = portSET_INTERRUPT_MASK_FROM_ISR();
    auto *slot = s_rtLog.claim((flags & RawLogLow) ? 0 : RawLogLow);
    if (slot) {
#ifdef __XTENSA__
        // Never format here: float formatting costs ~0.4 ms, and runs from flash. The flush on core 0 does it.
        RawLog *r = &slot->v.r;
        r->fmt = fmt;
        r->us = micros();
        XtVaList x;
        memcpy(&x, &args, sizeof(x));
        r->ndx = x.ndx;
        memcpy(r->reg, x.reg, sizeof(r->reg));
        memcpy(r->stk, (const char *) x.stk + 32, sizeof(r->stk));
        if (strings) rawLogStrings(*r, true);
#else
        char *buf = slot->v.str;
        const size_t n = sizeof(slot->v.str);
        int len = timestamp ? snprintf(buf, n, "(%lu): ", micros()) : 0;
        if (len < 0) len = 0;
        const size_t cap = n - len - size_t(appendBreak);
        int r2 = vsnprintf(buf + len, cap, fmt, args);
        if (r2 < 0) r2 = 0;
        if ((size_t) r2 >= cap) r2 = (int) cap - 1;
        len += r2;
        if (appendBreak) buf[len++] = '\n';
        buf[len] = 0;
        slot->v.len = (uint16_t) len;
#endif
        s_rtLog.publish(slot, flags);
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
    return slot ? 0 : -1;
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
        enqueue_log(fmt, args, true, true);
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
        enqueue_log(fmt, args);
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
    // Two passes: format every ready line into s_out and free its slot (memory speed), then run the
    // slow sinks (UART ~5 ms/line at 115200, telnet, MQTT/BLE mirrors) from the copy. kMaxPerCall
    // bounds the time in here so the network loop can pet the TWDT (32 lines ~ 160 ms); the rest
    // drains on the next tick. A full ring drops on the producer side. Single consumer: s_out is static.
    constexpr int kMaxPerCall = 32;
    constexpr size_t kLine = 224;
    static char s_out[2048];
    static uint16_t s_len[kMaxPerCall];
    int n = 0;
    for (size_t used = 0; n < kMaxPerCall && used + kLine <= sizeof(s_out); used += s_len[n++] + 1) {
        uint32_t flags;
        auto *slot = s_rtLog.front(flags);
        if (!slot) break; // empty, or the oldest line is still being written
#ifdef __XTENSA__
        s_len[n] = (uint16_t) rawLogFormat(slot->v.r, (uint8_t) flags, s_out + used, kLine);
#else
        s_len[n] = slot->v.len;
        memcpy(s_out + used, slot->v.str, slot->v.len + 1);
#endif
        s_rtLog.pop();
    }
    for (int i = 0, off = 0; i < n; off += s_len[i++] + 1) {
        const char *str = s_out + off;
        const uint16_t len = s_len[i];
        printf_old("%s", str); // str is data, not a format string (it may contain %)
#if WITH_TELNET
        if (log_telnet) log_telnet->write((uint8_t *) str, len);
#endif
        LogCallback cbs[kMaxLogCallbacks];
        const int ncb = snapshotLogCallbacks(cbs);
        for (int k = 0; k < ncb; ++k) cbs[k](str, len);
        bootLogCapture(str, len);
    }
    if (s_bootLog && esp_timer_get_time() > kBootLogKeepUs) { // no replay sink attached in time
        portENTER_CRITICAL(&logCbMux);
        char *b = bootLogTake();
        portEXIT_CRITICAL(&logCbMux);
        free(b);
    }
    static uint32_t reported = 0;
    if (const uint32_t d = s_rtLog.dropped.load(std::memory_order_relaxed); d != reported) {
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
#if WITH_TELNET
        if (log_telnet) log_telnet->write((uint8_t *) buf, l);
#endif
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
    // section/ISR). Core 0 takes the synchronous vprintf_mux path; this also makes it safe to route esp_log here from the very start of setup()
    // (so boot-log capture sees the setup() body). core1 behaviour is unchanged.
    if (xPortGetCoreID() == 1 && (deferLogs || !xPortCanYield())) {
        return enqueue_log(fmt, argptr, false, false, true);
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
    const int m = secretCut(cmd);
    if (esp_timer_get_time() - startUs > 5000000) logRetained("%s: %.*s%s", ok ? "OK" : "ERR", CG_MASKED(cmd, m));
    else UART_LOG("%s: %.*s%s", ok ? "OK" : "ERR", CG_MASKED(cmd, m));
}

#if WITH_TELNET
void set_logging_telnet(ESPTelnet *telnet) {
    if (telnet && !log_telnet) {
        char r[sizeof(s_retain)];
        if (size_t n = copyRetained(r)) telnet->write((uint8_t *) r, n);
    }
    log_telnet = telnet;
    ESP_LOGI("log", "%s telnet logging", telnet ? "Enabled" : "Disabled");
}
#endif

void enable_esp_log_to_telnet() {
    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/log.html
    old_vprintf = esp_log_set_vprintf(&vprintf_);
    if (!old_vprintf)
        old_vprintf = &vprintf;
}

void bootLogBegin() {
    if (char *b = s_bootLog ? nullptr : (char *) malloc(kBootLogSize)) {
        portENTER_CRITICAL(&logCbMux);
        s_bootLog = b;
        s_bootLogOpen = true;
        portEXIT_CRITICAL(&logCbMux);
    }
}


// Multi-producer (tasks only): heap-allocated Fn* through a FreeRTOS queue. Alloc failure or a full
// queue drops the task and logs it, never panics.
class TaskQueue {
    typedef std::function<void(void)> Fn;
    static constexpr UBaseType_t kLen = 32;
    StaticQueue_t qs;
    uint8_t buf[kLen * sizeof(Fn *)];
    QueueHandle_t q = xQueueCreateStatic(kLen, sizeof(Fn *), buf, &qs);
    std::atomic<uint32_t> dropped_{0};

public:
    void add(Fn &&fn) {
        Fn *p = new(std::nothrow) Fn(std::move(fn));
        if (!p || xQueueSend(q, &p, 0) != pdTRUE) {
            ESP_LOGW("log", "task queue %s, %lu deferred tasks dropped", p ? "full" : "OOM",
                     (unsigned long) ++dropped_);
            delete p;
        }
    }

    void work() {
        Fn *p;
        while (xQueueReceive(q, &p, 0) == pdTRUE)
            std::unique_ptr<Fn>(p)->operator()();
    }
};

static TaskQueue tq;

void enqueue_task(std::function<void(void)> &&fn) {
    tq.add(std::move(fn));
}

// Calls handed from the RT core (or any core/ISR) to core 0 without heap: a lock-free ring, so the
// RT core never waits for the consumer (which runs from flash).
struct RtDeferred {
    RtDeferredFn fn;
    void *ctx;
    float a, b;
};
static MpscRing<RtDeferred, 8> s_rtDefer;

__attribute__((flatten)) IRAM_ATTR bool rt_defer(RtDeferredFn fn, void *ctx, float a, float b) {
    const UBaseType_t irq = portSET_INTERRUPT_MASK_FROM_ISR();
    auto *slot = s_rtDefer.claim();
    if (slot) {
        slot->v = {fn, ctx, a, b};
        s_rtDefer.publish(slot, 1);
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
    return slot;
}

void process_queued_tasks() {
    uint32_t flags;
    while (auto *slot = s_rtDefer.front(flags)) {
        const RtDeferred d = slot->v;
        s_rtDefer.pop();
        d.fn(d.ctx, d.a, d.b);
    }
    static uint32_t reported = 0;
    if (const uint32_t n = s_rtDefer.dropped.load(std::memory_order_relaxed); n != reported) {
        reported = n;
        ESP_LOGW("log", "%lu RT deferred calls dropped (ring full)", (unsigned long) n);
    }
    tq.work();
}
