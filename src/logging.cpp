#include "logging.h"

#include <ESPTelnet.h>

//#include "concurrentqueue.h"
#include "q/readerwriterqueue.h"

//struct ConcurrentQueueMinMemTraits : public moodycamel::ConcurrentQueueDefaultTraits {
//    static const size_t BLOCK_SIZE = 2;
//   static const size_t INITIAL_IMPLICIT_PRODUCER_HASH_SIZE = 0;
//};

struct AsyncLogEntry {
    char *str;
    uint16_t len;
    bool telnetOnly = false;
};
//static std::deque<AsyncLogEntry> uart_async_log_queue;

//static moodycamel::ConcurrentQueue<AsyncLogEntry, ConcurrentQueueMinMemTraits> uart_async_log_queue{1};
static moodycamel::ReaderWriterQueue<AsyncLogEntry> uart_async_log_queue{};

ESPTelnet *log_telnet = nullptr;

vprintf_like_t old_vprintf = &vprintf;

void enqueue_telnet_log(const char *s, int len) {
    if (uart_async_log_queue.size_approx() > 200) return;
    auto buf = new char[len + 1];
    strncpy(buf, s, len + 1);
    uart_async_log_queue.enqueue(AsyncLogEntry{buf, (uint16_t) len, true});
}

void enqueue_telnet_log(const char *fmt, size_t l, const va_list &args) {
    if (uart_async_log_queue.size_approx() > 200) return;
    auto buf = new char[l + 1];
    auto len = vsnprintf(buf, l + 1, fmt, args);
    uart_async_log_queue.enqueue(AsyncLogEntry{buf, (uint16_t) len, true});
}

void UART_LOG(const char *fmt, ...) {
    //static char UART_LOG_buf[384];

    va_list args;
    va_start(args, fmt);
    auto l = old_vprintf(fmt, args);
    //auto l = vsnprintf(UART_LOG_buf, 380, fmt, args);

    if (log_telnet) {
        // log_telnet->write((uint8_t *) UART_LOG_buf, l);
        //enqueue_telnet_log(UART_LOG_buf, l);
        enqueue_telnet_log(fmt, l, args);
    }

    va_end(args);
    //if(l <= 0)
    //    return;

    //UART_LOG_buf[l] = '\n';
    // UART_LOG_buf[++l] = '\0';
    //uart_write_bytes(0, UART_LOG_buf, l);

    //old_vprintf(UART_LOG_buf, nullptr);


}


void UART_LOG_ASYNC(const char *fmt, ...) {

    if (uart_async_log_queue.size_approx() > 200)
        return;

    auto buf = new char[384];

    va_list args;
    va_start(args, fmt);
    uint16_t l = vsnprintf(buf, 380, fmt, args);
    va_end(args);
    buf[l] = '\n';
    buf[++l] = '\0';

    uart_async_log_queue.enqueue(AsyncLogEntry{buf, l, false});

    /* if (uart_async_log_queue.size() > 200) {
         delete[] uart_async_log_queue.front().str;
         uart_async_log_queue.pop_front(); // TODO fix race, use lock-free queu
         // e.g. https://github.com/cameron314/concurrentqueue/blob/master/concurrentqueue.h
     }*/
}

void flush_async_uart_log() {
    AsyncLogEntry entry;
    while (uart_async_log_queue.try_dequeue(entry)) {

        if (!entry.telnetOnly) {
            va_list l{};
            old_vprintf(entry.str, l);
            //uart_write_bytes(0, entry.str, entry.len);
        }

        if (log_telnet)
            log_telnet->write((uint8_t *) entry.str, entry.len);

        delete[] entry.str;
    }
}


int vprintf_telnet(const char *fmt, va_list argptr) {
    static char loc_buf[200];

    int r = old_vprintf(fmt, argptr);

    if (log_telnet) {
        int l = vsnprintf(loc_buf, sizeof(loc_buf), fmt, argptr);
        if (l > 0) {
            enqueue_telnet_log(loc_buf, l);
            //log_telnet->write((uint8_t *) loc_buf, l);
        }
    }

    return r;
}

void set_logging_telnet(ESPTelnet *telnet) {
    log_telnet = telnet;
    ESP_LOGI("log", "%s telnet logging", telnet ? "Enabled" : "Disabled");
}

void enable_esp_log_to_telnet() {
    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/log.html
    old_vprintf = esp_log_set_vprintf(&vprintf_telnet);
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
