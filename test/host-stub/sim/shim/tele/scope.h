#pragma once
// Sim: no scope streaming.
#include <atomic>
#include <cstdint>
#include "etc/rt.h"
inline std::atomic<uint32_t> bytesSent{0}; // mirrors src/tele/scope.h; unused in the sim
struct SimScope {
    void addChannel(const void *, uint8_t, char, int, const char *) {}
    void addSample12(const void *, uint8_t, float) {}
};
inline SimScope *scope = nullptr;
