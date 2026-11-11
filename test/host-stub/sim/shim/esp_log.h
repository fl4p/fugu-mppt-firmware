#pragma once
// Sim: ESP_LOG* -> simLog() (sim-time stamped, counted by the host main).
#include <cstdio>
#include <cstdint>

typedef enum { ESP_LOG_NONE, ESP_LOG_ERROR, ESP_LOG_WARN, ESP_LOG_INFO, ESP_LOG_DEBUG, ESP_LOG_VERBOSE } esp_log_level_t;
inline void esp_log_level_set(const char *, esp_log_level_t) {}
void simLog(char lvl, const char *tag, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

#define ESP_LOGE(tag, fmt, ...) simLog('E', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) simLog('W', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) simLog('I', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) do {} while (0)
#define ESP_LOGV(tag, fmt, ...) do {} while (0)
#define ESP_ERROR_CHECK(x) (void) (x)
#define ESP_OK 0
#define ESP_FAIL -1
typedef int esp_err_t;
