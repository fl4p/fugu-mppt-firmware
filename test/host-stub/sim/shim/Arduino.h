#pragma once
// Sim: minimal Arduino surface; time comes from the sim clock (sim.cpp).
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "esp_log.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"

using std::min;
using std::max;
using std::isnan;
using std::isinf;

#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#define unlikely(x) (x)
#define likely(x) (x)
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

uint32_t millis();
uint32_t micros();
inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t) {}
inline void yield() {}
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t, int) {}
inline int digitalRead(uint8_t) { return 0; }
inline int analogRead(uint8_t) { return 0; }

class String : public std::string {
public:
    using std::string::string;
    String(const std::string &s) : std::string(s) {}
};

struct IPAddress {
    uint32_t v = 0;
    IPAddress() = default;
    explicit IPAddress(unsigned long x) : v((uint32_t) x) {}
    explicit IPAddress(const char *) : v(1) {}
    explicit operator bool() const { return v != 0; }
    String toString() const { return "0.0.0.0"; }
};
