#include "logging.h"

#include <ESPTelnet.h>

struct AsyncLogEntry {
    char *str;
    uint16_t len;
    bool telnetOnly = false;
};
static std::deque<AsyncLogEntry> uart_async_log_queue;

ESPTelnet *log_telnet = nullptr;

vprintf_like_t old_vprintf = &vprintf;

void enqueue_telnet_log(const char *s, int len) {
    auto buf = new char[len+1];
    strncpy(buf, s, len+1);
    uart_async_log_queue.emplace_back(AsyncLogEntry{buf, (uint16_t )len, true});
}

void enqueue_telnet_log(const char *fmt, const va_list &args) {
    auto buf = new char[200];
    auto len = vsnprintf(buf, 200, fmt, args);
    uart_async_log_queue.emplace_back(AsyncLogEntry{buf, (uint16_t )len, true});
}

void UART_LOG(const char *fmt, ...) {
    //static char UART_LOG_buf[384];

    va_list args;
    va_start(args, fmt);
    old_vprintf(fmt, args);
    //auto l = vsnprintf(UART_LOG_buf, 380, fmt, args);

    if (log_telnet) {
        // log_telnet->write((uint8_t *) UART_LOG_buf, l);
        //enqueue_telnet_log(UART_LOG_buf, l);
        enqueue_telnet_log(fmt, args);
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

    auto buf = new char[384];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, 380, fmt, args);
    va_end(args);
    auto l = (uint16_t) strlen(buf);
    buf[l] = '\n';
    buf[++l] = '\0';

    uart_async_log_queue.emplace_back(AsyncLogEntry{buf, l, false});

    if (uart_async_log_queue.size() > 200) {
        delete[] uart_async_log_queue.front().str;
        uart_async_log_queue.pop_front();
    }
}

void flush_async_uart_log() {
    while (!uart_async_log_queue.empty()) {
        auto entry = uart_async_log_queue.front();

        if(!entry.telnetOnly) {
            va_list l{};
            old_vprintf(entry.str, l);
            //uart_write_bytes(0, entry.str, entry.len);
        }

        if (log_telnet)
            log_telnet->write((uint8_t *) entry.str, entry.len);

        delete[] entry.str;
        uart_async_log_queue.pop_front();
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