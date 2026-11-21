#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

constexpr char kRtTaskName[] = "loopRt";

// The RT loop task exists. It is created once, only after an error-free setup, and deletes itself only when its
// sampler cannot start; a busy or stalled loop still exists.
inline bool rtLoopRunning() { return xTaskGetHandle(kRtTaskName) != nullptr; }
