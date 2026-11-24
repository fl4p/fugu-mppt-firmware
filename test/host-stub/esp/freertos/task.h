#pragma once
#include "FreeRTOS.h"
// Host sim: a task notification == one simulated ADC timer tick (sim_stubs.cpp).
uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, TickType_t ticks);
inline TaskHandle_t xTaskGetCurrentTaskHandle() { return (TaskHandle_t) 1; }
inline void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t *w) { *w = 0; }
inline void xTaskNotifyGive(TaskHandle_t) {}
// sim: a console task's wait advances the simulation (null: no-op)
extern void (*g_simDelayHook)(uint32_t ms);
inline void vTaskDelay(TickType_t t) { if (g_simDelayHook) g_simDelayHook(t); }
inline BaseType_t xPortGetCoreID() { return 1; }
inline void vTaskDelete(TaskHandle_t) {}
inline UBaseType_t uxTaskPriorityGet(TaskHandle_t) { return 1; }
// host: no second core, the caller runs the function itself; g_simRunTasks: run it inline to its end
extern bool g_simRunTasks;
inline BaseType_t xTaskCreatePinnedToCore(void (*f)(void *), const char *, uint32_t, void *a, UBaseType_t,
                                          TaskHandle_t *, BaseType_t) {
    if (!g_simRunTasks) return 0;
    f(a);
    return pdPASS;
}
