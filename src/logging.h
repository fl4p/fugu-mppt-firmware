#pragma once

#include <esp_log.h>
#include <esp32-hal-log.h>

#include <cstring>
#include <deque>
#include <driver/uart.h>

#include<functional>

//esp_log_write(ESP_LOG_ERROR,      tag, LOG_FORMAT(E, format), esp_log_timestamp(), tag __VA_OPT__(,) __VA_ARGS__);

//void log_write()

/*
#undef ESP_LOGE
#undef ESP_LOGW
#undef ESP_LOGI

#define ESP_LOGE(tag, format, ...)   do {  ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   tag, format __VA_OPT__(,) __VA_ARGS__); } while(0) //  log_e("[%s] " format, tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...)   do { ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    tag, format __VA_OPT__(,) __VA_ARGS__); } while(0)
#define ESP_LOGI(tag, format, ...)   do { ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    tag, format __VA_OPT__(,) __VA_ARGS__); } while(0)
*/

// class ESPTelnet;
// ESPTelnet *log_telnet = nullptr;

void loggingEnableDefer();

// test hook: capture every RT-core log line raw, as during a flash op
extern bool g_rawLogForce;
void enable_esp_log_to_telnet();
// Allocate and open the boot-log backlog (replayed to the first replayBootLog sink). Call once, early.
void bootLogBegin();
// esp_log_set_vprintf

void UART_LOG(const char *format, ...)  __attribute__ ((format (printf, 1, 2)));
void printf_mux(const char *format, ...)  __attribute__ ((format (printf, 1, 2)));
#define UART_LOG_ASYNC UART_LOG
//void UART_LOG_ASYNC(const char *format, ...)  __attribute__ ((format (printf, 1, 2)));

void flush_async_uart_log();
void process_queued_tasks();
void enqueue_task(std::function<void(void)> &&fn);

// Queues fn(ctx, a, b) for process_queued_tasks() on core 0, without heap. Any core, also from an
// ISR. False when the fixed ring is full (the call is dropped and counted).
typedef void (*RtDeferredFn)(void *ctx, float a, float b);
bool rt_defer(RtDeferredFn fn, void *ctx, float a = 0, float b = 0);

class ESPTelnet;
void set_logging_telnet(ESPTelnet *telnet);


// Log sinks: multiple callbacks may be registered concurrently (e.g. MQTT + BLE console). Each
// receives every muxed log line on core 0. addLogCallback dedupes and ignores nullptr.
typedef void (*LogCallback)(const char *str, uint16_t len);

// replayBootLog=false suppresses the one-shot boot-backlog replay for this sink. Pass false for
// interactive/scripted consoles: the replay is sized for MQTT (attaches late, wants the boot
// sequence), but on a console it dumps up to 8 KB into the transport at connect -- which on BLE
// pushes the log mirror over its half-FIFO cap and silently eats the NEXT command's result line.
void addLogCallback(LogCallback callback, bool replayBootLog = true);

void removeLogCallback(LogCallback callback);

// Log one line to every sink and keep it for 120 s; it is replayed to each log callback or telnet
// client attaching in that window (a sink that was down when the line was emitted still gets it).
void logRetained(const char *format, ...) __attribute__ ((format (printf, 1, 2)));

// Console completion marker "OK: <cmd>" / "ERR: <cmd>"; retained when the command ran longer than 5 s.
void logCmdResult(bool ok, const char *cmd, int64_t startUs);
