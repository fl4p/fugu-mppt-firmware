#pragma once

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

// Thin facade so main.cpp body code (status line, restart, high-temp protection) doesn't need
// inline #ifdef CONFIG_FUGU_WITH_NETW around small Wi-Fi state queries. CONFIG_FUGU_WITH_NETW=1 forwards to arduino-esp32;
// CONFIG_FUGU_WITH_NETW=0 reports "no network" and stubs the actions.

#include <string>

#ifdef CONFIG_FUGU_WITH_NETW
#include <WiFi.h>
#include "../tele/telemetry.h"  // connect_wifi_async, wait_for_wifi, wifiLoop, disconnect_wifi

inline bool wifiIsConnected() { return WiFi.isConnected(); }
inline int wifiRssi() { return WiFi.RSSI(); }
inline void wifiHardOff() { WiFi.disconnect(true); }
inline void wifiSetPowerSave(bool deep) { WiFi.setSleep(deep ? WIFI_PS_MAX_MODEM : WIFI_PS_MIN_MODEM); }
inline std::string wifiLocalIp() { return std::string(WiFi.localIP().toString().c_str()); }
#else
// telemetry.h's wifiLoop/connect_wifi_async/wait_for_wifi/disconnect_wifi are declared regardless
// of CONFIG_FUGU_WITH_NETW (included transitively via mppt.h), so we do NOT redeclare them here. Under
// CONFIG_FUGU_WITH_NETW=0 those functions are unused (gated by #ifdef CONFIG_FUGU_WITH_NETW at every call site).
inline bool wifiIsConnected() { return false; }
inline int wifiRssi() { return 0; }
inline void wifiHardOff() {}
inline std::string wifiLocalIp() { return {}; }
#endif
