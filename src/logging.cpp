#include "logging.h"

#include <ESPTelnet.h>

#include "etc/readerwriterqueue.h"


//struct ConcurrentQueueMinMemTraits : public moodycamel::ConcurrentQueueDefaultTraits {
//    static const size_t BLOCK_SIZE = 2;
//   static const size_t INITIAL_IMPLICIT_PRODUCER_HASH_SIZE = 0;
//};

int vprintf_(const char *fmt, va_list argptr);

int vprintf_mux(const char *fmt, va_list argptr);

struct AsyncLogEntry {
    char *str;
    uint16_t len;
    bool telnetOnly = false;
};

//static std::deque<AsyncLogEntry> uart_async_log_queue;

//static moodycamel::ConcurrentQueue<AsyncLogEntry, ConcurrentQueueMinMemTraits> uart_async_log_queue{1};
static moodycamel::ReaderWriterQueue<AsyncLogEntry> uart_async_log_queue{};

ESPTelnet *log_telnet = nullptr;

void (*logCallback)(const char *str, uint16_t len) = nullptr;

vprintf_like_t old_vprintf = &vprintf;

bool deferLogs = false;

void loggingEnableDefer() {
    deferLogs = true;
}

void addLogCallback(void (*callback)(const char *str, uint16_t len)) {
    if (logCallback != nullptr && callback != nullptr) {
        ESP_LOGW("log", "callback already set, overwrite");
    }
    logCallback = callback;
}


void enqueue_log(const char *s, int len) {
    assert((xPortGetCoreID() == 1));

    if (uart_async_log_queue.size_approx() > 200) return;
    auto buf = new char[len + 1];
    strncpy(buf, s, len + 1);
    uart_async_log_queue.enqueue(AsyncLogEntry{buf, (uint16_t) len, false});
}


int enqueue_log(const char *fmt, size_t l, const va_list &args, bool appendBreak = false, bool timestamp = false) {
    assert((xPortGetCoreID() == 1)); // ensure RT core

    if (uart_async_log_queue.size_approx() > 200) return -1;
    auto buf = new char[l + 1];
    int len = 0;
    if (timestamp) {
        len = snprintf(buf, l + 1, "(%lu): ", micros());
        if (len <= 0) return len;
    }
    auto r2 = vsnprintf(buf + len, l + 1 - len - size_t(appendBreak), fmt, args);
    if (r2 <= 0) return len; // error or empty
    len += r2;
    if (appendBreak) {
        buf[len] = '\n';
        buf[len + 1] = 0;
        len += 1;
    }
    uart_async_log_queue.enqueue(AsyncLogEntry{buf, (uint16_t) len, false});
    return len;
}

/*
void enqueue_telnet_log(const char *s, int len) {
    assert((xPortGetCoreID() == 1));

    if (uart_async_log_queue.size_approx() > 200) return;
    auto buf = new char[len + 1];
    strncpy(buf, s, len + 1);
    uart_async_log_queue.enqueue(AsyncLogEntry{buf, (uint16_t) len, true});
}
*/
/*
void enqueue_telnet_log(const char *fmt, size_t l, const va_list &args) {
    assert((xPortGetCoreID() == 1));

    if (uart_async_log_queue.size_approx() > 200) return;
    auto buf = new char[l + 1];
    auto len = vsnprintf(buf, l + 1, fmt, args);
    uart_async_log_queue.enqueue(AsyncLogEntry{buf, (uint16_t) len, true});
}*/


void UART_LOG(const char *fmt, ...) {
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

void printf_mux(const char *fmt, ...) {
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

int printf_old(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int l = old_vprintf(fmt, args);
    va_end(args);
    return l;
}

void flush_async_uart_log() {
    AsyncLogEntry entry;
    while (uart_async_log_queue.try_dequeue(entry)) {
        if (!entry.telnetOnly) {
            // we call the old vprintf here for convenience.
            // otherwise we can write the string to UART and JTAG USB
            // see esp-idf/console, which does the multiplexing for us

            //va_list l{};
            //old_vprintf(entry.str, l);
            //old_vprintf("%s", entry.str);
            printf_old(entry.str);
            //uart_write_bytes(0, entry.str, entry.len);
        }

        if (log_telnet)
            log_telnet->write((uint8_t *) entry.str, entry.len);

        if (logCallback)
            logCallback(entry.str, entry.len);

        delete[] entry.str;
    }
}

/**
 * Synchronous printf to UART, JTAG USB and telnet
 * @param fmt
 * @param argptr
 * @return
 */
int vprintf_mux(const char *fmt, va_list argptr) {
    static char loc_buf[300];

    int r = old_vprintf(fmt, argptr);

    if (log_telnet or logCallback) {
        int l = vsnprintf(loc_buf, sizeof(loc_buf), fmt, argptr);
        if (l > 0) {
            //enqueue_telnet_log(loc_buf, l);
            if (log_telnet) log_telnet->write((uint8_t *) loc_buf, l);
            if (logCallback)logCallback(loc_buf, l);
        }
    }

    return r;
}


int vprintf_(const char *fmt, va_list argptr) {
    if (xPortGetCoreID() == 1 && deferLogs) {
        // RT core1: defer all log to core0
        return enqueue_log(fmt, 200, argptr);
    } else {
        return vprintf_mux(fmt, argptr);
    }
}

void set_logging_telnet(ESPTelnet *telnet) {
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

public:
    inline void add(Fn &&fn) {
        assert(q.enqueue(std::move(fn)));
    }

    inline void work() {
        Fn fn;
        while (q.try_dequeue(fn)) {
            fn();
        }
    }
};

static TaskQueue tq;

void enqueue_task(std::function<void(void)> &&fn) {
    tq.add(std::move(fn));
}

void process_queued_tasks() {
    tq.work();
}
