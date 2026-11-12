#pragma once
// Host sim: inert WiFi for tele/scope.h (never started).
#include <cstddef>
#include <cstdint>
#include "Arduino.h"

#define WL_CONNECTED 3
#define WIFI_POWER_21dBm 84

struct WiFiClient {
    bool connected() { return false; }
    int write(const uint8_t *, size_t len) { return (int) len; }
    int write(const char *s) { return (int) strlen(s); }
    IPAddress remoteIP() { return IPAddress(); }
};

struct WiFiServer {
    WiFiServer(int = 0, int = 1) {}
    void begin() {}
    void end() {}
    void setNoDelay(bool) {}
    bool hasClient() { return false; }
    WiFiClient accept() { return {}; }
};

struct WiFiStub {
    int status() { return 0; }
    void setTxPower(int) {}
    bool isConnected() { return false; }
};
inline WiFiStub WiFi;
