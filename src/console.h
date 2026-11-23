#pragma once

#include <esp_log.h>          // ESP_LOGW (LOG_VALUE_* helpers below)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>   // QueueHandle_t

#include "util.h"

void uartInit(int port_num);

void consoleInit();

int console_write_usb(const char *buf, unsigned int len);

void loopUart(time_ms nowMs);

// One line buffer per input stream, so bytes from one console never land in another's command.
struct ConsoleLine {
    static constexpr uint8_t bufSiz = 128;
    char buf[bufSiz];
    uint8_t pos;
    time_ms lastMs;
};

// trusted=false: only read-only commands run (handleCommand)
void loopConsole(ConsoleLine &ln, int read(char *buf, size_t len), int write(const char *buf, size_t len),
                 time_ms nowMs, bool trusted);

// Installed by a transport whose output is buffered and drained asynchronously from its own loop
// (BLE), so a long-running blocking command (e.g. an OTA download) can push pending output mid-run
// instead of stalling until it returns. Null for synchronous transports (UART/USB). consoleFlush()
// is the no-arg entry point such commands call.
extern void (*consoleFlushHook)();
void consoleFlush();

// Queue one command line for handleCommand() on loopNetwork_task (process_queued_tasks), not inline in
// the transport's receive callback: a teardown command would destroy the transport under itself.
void consoleDispatchAsync(const char *line, size_t len, bool trusted);

extern time_us lastTimeOutUs;



static constexpr int UART_BUF_SIZE = 1024;
extern QueueHandle_t uart_queue;



inline void LOG_VALUE_IGNORED(const char * tag, const char *name, size_t len, const char *str) {
    ESP_LOGW(tag, "%s ignored, invalid value '%.*s'",  name, len, str);
}

inline void LOG_VALUE_NOT_FINITE(const char * tag, const char *name, const char *group) {
    ESP_LOGW(tag, "%s in %s is not finite",  name, group);
}

