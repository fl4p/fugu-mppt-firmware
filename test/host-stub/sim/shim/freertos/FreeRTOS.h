#pragma once
#include <cstdint>
#include "esp_log.h"
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void *QueueHandle_t;
typedef void *SemaphoreHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY 0xffffffffu
#define pdMS_TO_TICKS(ms) ((TickType_t) (ms))
#define portYIELD_FROM_ISR(x) (void) (x)
typedef struct { int dummy; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(m) (void) (m)
#define portEXIT_CRITICAL(m) (void) (m)
#define xPortGetCoreID() 1
inline void vTaskDelay(TickType_t) {}
inline void vTaskDelete(TaskHandle_t) {}
inline UBaseType_t uxTaskPriorityGet(TaskHandle_t) { return 0; }
inline TaskHandle_t xTaskGetCurrentTaskHandle() { return nullptr; }
inline uint32_t ulTaskNotifyTake(BaseType_t, TickType_t) { return 1; }
inline void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t *w) { *w = 0; }
inline void xTaskNotifyGive(TaskHandle_t) {}
