#pragma once
#include "esp_err.h"
typedef struct temperature_sensor_obj_t *temperature_sensor_handle_t;
typedef enum { TEMPERATURE_SENSOR_CLK_SRC_DEFAULT } temperature_sensor_clk_src_t;
typedef struct { int range_min; int range_max; temperature_sensor_clk_src_t clk_src; struct { unsigned allow_pd: 1; } flags; } temperature_sensor_config_t;
extern float g_simChipTemp;
inline esp_err_t temperature_sensor_install(const temperature_sensor_config_t *, temperature_sensor_handle_t *h) { *h = (temperature_sensor_handle_t) 1; return ESP_OK; }
inline esp_err_t temperature_sensor_enable(temperature_sensor_handle_t) { return ESP_OK; }
inline esp_err_t temperature_sensor_get_celsius(temperature_sensor_handle_t, float *c) { *c = g_simChipTemp; return ESP_OK; }
