#pragma once
#include <cstdint>
// Simulated monotonic clock (µs), advanced by the sim per ADC tick.
extern int64_t g_simUs;
inline int64_t esp_timer_get_time() { return g_simUs; }
typedef struct esp_timer *esp_timer_handle_t;
