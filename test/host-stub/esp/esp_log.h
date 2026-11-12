#pragma once
// Host sim: ESP_LOGx -> sim_log() (stamped with the simulated clock, see sim_stubs.cpp).
#include <cstdio>
#include "esp_err.h"
typedef enum { ESP_LOG_NONE, ESP_LOG_ERROR, ESP_LOG_WARN, ESP_LOG_INFO, ESP_LOG_DEBUG, ESP_LOG_VERBOSE } esp_log_level_t;
inline void esp_log_level_set(const char *, esp_log_level_t) {}
void sim_log(char lvl, const char *tag, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
#define ESP_LOGE(tag, fmt, ...) sim_log('E', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) sim_log('W', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) sim_log('I', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) do {} while (0)
#define ESP_LOGV(tag, fmt, ...) do {} while (0)
