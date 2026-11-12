#pragma once
#include "FreeRTOS.h"
// Host sim: a task notification == one simulated ADC timer tick (sim_stubs.cpp).
uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, TickType_t ticks);
inline TaskHandle_t xTaskGetCurrentTaskHandle() { return (TaskHandle_t) 1; }
inline void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t *w) { *w = 0; }
inline void xTaskNotifyGive(TaskHandle_t) {}
inline void vTaskDelay(TickType_t) {}
inline BaseType_t xPortGetCoreID() { return 1; }
