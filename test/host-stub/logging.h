#pragma once
// host-test shim for src/logging.h — just enough for plot.h to compile and run.
// printf_mux / UART_LOG are routed to stdout; ESP_LOG* comes from esp_log.h.

#include <cstdarg>
#include <cstdio>
#include <functional>

#include "esp_log.h"

inline void printf_mux(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stdout, fmt, ap);
    va_end(ap);
}

inline void UART_LOG(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stdout, fmt, ap);
    std::fputc('\n', stdout);
    va_end(ap);
}

inline void loggingEnableDefer() {}
inline void enable_esp_log_to_telnet() {}
inline void flush_async_uart_log() {}
inline void process_queued_tasks() {}
inline void enqueue_task(std::function<void(void)> &&) {}
typedef void (*RtDeferredFn)(void *ctx, float a, float b);
inline bool rt_defer(RtDeferredFn, void *, float = 0, float = 0) { return true; }

typedef void (*LogCallback)(const char *, unsigned short);
inline void addLogCallback(LogCallback) {}
inline void removeLogCallback(LogCallback) {}

class ESPTelnet;
inline void set_logging_telnet(ESPTelnet *) {}
