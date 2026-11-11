#pragma once
// Sim: die temperature is a constant 35 C.
#include "esp_log.h"
typedef void *temperature_sensor_handle_t;
enum { TEMPERATURE_SENSOR_CLK_SRC_DEFAULT = 0 };
typedef struct { int range_min, range_max, clk_src; struct { int allow_pd; } flags; } temperature_sensor_config_t;
inline esp_err_t temperature_sensor_install(const temperature_sensor_config_t *, temperature_sensor_handle_t *h) {
    *h = (void *) 1; return ESP_OK;
}
inline esp_err_t temperature_sensor_enable(temperature_sensor_handle_t) { return ESP_OK; }
inline esp_err_t temperature_sensor_get_celsius(temperature_sensor_handle_t, float *t) { *t = 35.f; return ESP_OK; }
