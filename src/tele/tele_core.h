#pragma once

// Internal header for the WiFi-free telemetry core (tele_core.cpp) and its
// UDP consumer (telemetry.cpp). Producers use the public telemetry.h API.

#include <string>
#include <cstdint>
#include "../etc/readerwriterqueue.h"

// SPSC: producer = net-loop (telemetryAddPoint), consumer = UDP flushTask only.
struct QueuedTelePoint {
    std::string wire;
    uint64_t producedMs = 0;
};
extern moodycamel::ReaderWriterQueue<QueuedTelePoint> pointsQ;

// True while the UDP flush task runs (TelemetryService onStart/onStop); without a
// consumer, points are not queued.
extern bool g_teleUdpActive;

// Read at sink start; each sink retains its own mode for its lifetime.
bool telePminiConfigured();
