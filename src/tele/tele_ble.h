#pragma once

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif


// BLE telemetry stream (CONFIG_FUGU_WITH_BLE_TELE): binary sym_line_protocol frames, tamp-compressed,
// notified over the NUS TELE characteristic. Record framing: <0x7E><varint len><cid><payload>,
// where <cid><payload> matches one UDP telemetry datagram. All state runs on the network loop;
// enable with the `set-time` + `tele-ble 1` console commands.

#include <string>
#include "../util.h"   // time_ms

class BLEService;

#ifdef CONFIG_FUGU_WITH_BLE_TELE

bool teleBleStreaming();
// nullptr on success, else a static error message for the console reply.
const char *teleBleSetStreaming(bool on);
void teleBleEnqueue(const std::string &frame);   // producer fan-out (network loop only)
void teleBleTick(time_ms nowMs);                 // batch + compress + notify (from bleConsoleLoop)
void teleBleCreateChar(BLEService *svc);         // from bleConsoleBegin, before svc->start()
void teleBleRequestStop();                       // NimBLE-host-task safe (disconnect callback)
size_t teleBleDropped();                         // bytes dropped on overflow (status)

#else

inline bool teleBleStreaming() { return false; }
inline const char *teleBleSetStreaming(bool) { return "not compiled (FUGU_WITH_BLE_TELE)"; }
inline void teleBleEnqueue(const std::string &) {}
inline void teleBleTick(time_ms) {}
inline void teleBleCreateChar(BLEService *) {}
inline void teleBleRequestStop() {}
inline size_t teleBleDropped() { return 0; }

#endif
