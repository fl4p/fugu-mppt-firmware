#pragma once
#include <cstdint>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) (ms)
#define portYIELD_FROM_ISR(x) do {} while (0)
typedef struct { int x; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(m) do {} while (0)
#define portEXIT_CRITICAL(m) do {} while (0)
