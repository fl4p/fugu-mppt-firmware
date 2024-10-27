#pragma once

#include <esp_log.h>
#include <esp32-hal-log.h>

#include <cstring>
#include <deque>
#include <driver/uart.h>

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

void enable_esp_log_to_telnet();
// esp_log_set_vprintf

void UART_LOG(const char *fmt, ...);
void UART_LOG_ASYNC(const char *fmt, ...);
void flush_async_uart_log();

class ESPTelnet;
void set_logging_telnet(ESPTelnet *telnet);