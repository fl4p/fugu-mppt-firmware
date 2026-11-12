#pragma once
#include "esp_err.h"
#include "hal/adc_types.h"
typedef struct adc_continuous_ctx_t *adc_continuous_handle_t;
inline esp_err_t adc_continuous_stop(adc_continuous_handle_t) { return ESP_OK; }
inline esp_err_t adc_continuous_deinit(adc_continuous_handle_t) { return ESP_OK; }
