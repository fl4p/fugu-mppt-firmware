#pragma once

// MQTT telemetry sink (WITH_TELE_MQTT): while a lease runs, binary sym_line_protocol frames are
// published uncompressed on pv/tele/<hostname>. Each message is <cid=0><payload>, the same bytes
// as one UDP telemetry datagram. All state runs on the network loop.

#include <cstdint>
#include <string>
#include "../util.h"   // time_ms

#ifdef WITH_TELE_MQTT

// seconds=0 stops; else starts or extends the lease (capped at 3600 s). nullptr on success,
// else a static error message.
const char *teleMqttLease(uint32_t seconds);
void teleMqttEnqueue(const std::string &frame);   // producer fan-out
void teleMqttTick(time_ms nowMs);                 // produce, batch and publish
bool teleMqttActive();
uint32_t teleMqttLeaseLeftS(time_ms nowMs);
size_t teleMqttDropped();
void teleProducePoint();                          // mppt.telemetry(), defined in main.cpp

#else

inline bool teleMqttActive() { return false; }
inline void teleMqttEnqueue(const std::string &) {}
inline void teleMqttTick(time_ms) {}

#endif
