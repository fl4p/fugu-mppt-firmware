#pragma once
#include "FreeRTOS.h"
// Host sim: a task notification == one simulated ADC timer tick (sim_stubs.cpp).
uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, TickType_t ticks);
inline TaskHandle_t xTaskGetCurrentTaskHandle() { return (TaskHandle_t) 1; }
inline void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t *w) { *w = 0; }
inline void xTaskNotifyGive(TaskHandle_t) {}
inline void vTaskDelay(TickType_t) {}
inline BaseType_t xPortGetCoreID() { return 1; }
inline void vTaskDelete(TaskHandle_t) {}
inline UBaseType_t uxTaskPriorityGet(TaskHandle_t) { return 1; }
// host: no second core, the caller runs the function itself
inline BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *, uint32_t, void *, UBaseType_t,
                                          TaskHandle_t *, BaseType_t) { return 0; }
