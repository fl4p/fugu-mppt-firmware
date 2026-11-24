#pragma once
// host-test shim for ESP-IDF's esp_log.h — just enough for service.h / conf.h to compile.
#include <cstdio>

typedef enum {
    ESP_LOG_NONE,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE,
} esp_log_level_t;

inline void esp_log_level_set(const char *, esp_log_level_t) {}

#ifndef ESP_LOGE
#define ESP_LOGE(tag, fmt, ...) std::fprintf(stderr, "[E %s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) std::fprintf(stderr, "[W %s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) std::fprintf(stderr, "[I %s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) std::fprintf(stderr, "[D %s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) std::fprintf(stderr, "[V %s] " fmt "\n", tag, ##__VA_ARGS__)
#endif
