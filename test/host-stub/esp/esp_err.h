#pragma once
#include <cstdlib>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERROR_CHECK(x) do { esp_err_t e_ = (x); if (e_ != ESP_OK) abort(); } while (0)
inline const char *esp_err_to_name(esp_err_t) { return "esp_err"; }
