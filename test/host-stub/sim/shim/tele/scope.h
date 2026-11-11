#pragma once
// Sim: no scope streaming.
#include <cstdint>
#include "etc/rt.h"
static uint64_t bytesSent = 0;
struct SimScope {
    void addChannel(const void *, uint8_t, char, int, const char *) {}
    void addSample12(const void *, uint8_t, float) {}
};
inline SimScope *scope = nullptr;
