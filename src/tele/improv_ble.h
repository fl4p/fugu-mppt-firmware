#pragma once

// Improv Wi-Fi over BLE (improv-wifi.com/ble), a GATT service next to the NUS console.
// Credentials are accepted only while the device is offline: no network configured, or
// Wi-Fi down for wifi.conf::improv_offline_s. Online, the state reads "provisioned" and
// credentials are refused.

#include <cstddef>
#include <cstdint>

#include "../util.h"   // time_ms

class BLEServer;

#ifdef WITH_IMPROV

// writeProps: the console RX write flags; the RPC characteristic takes the same security.
void improvCreateService(BLEServer *srv, uint32_t writeProps);
void improvTick(time_ms nowMs);          // network loop
size_t improvAdvPayload(uint8_t *ad);    // 31-byte Improv advert while offered, else 0

#else

inline void improvCreateService(BLEServer *, uint32_t) {}
inline void improvTick(time_ms) {}
inline size_t improvAdvPayload(uint8_t *) { return 0; }

#endif
