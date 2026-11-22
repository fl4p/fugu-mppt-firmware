#pragma once
#include <cstdint>
#include <ctime>
inline int64_t esp_timer_get_time() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t) t.tv_sec * 1000000 + t.tv_nsec / 1000;
}
