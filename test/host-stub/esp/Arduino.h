#pragma once
// Host sim Arduino.h: time from the simulated clock, GPIO no-ops.
#include <cstdint>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <string>
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_log.h"

using std::min;
using std::max;
using std::isnan;

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

inline unsigned long millis() { return (unsigned long) (esp_timer_get_time() / 1000); }
inline unsigned long micros() { return (unsigned long) esp_timer_get_time(); }
inline void delay(uint32_t) {}
inline void usleep_(uint32_t) {}
inline void pinMode(uint8_t, uint8_t) {}
inline uint8_t simPinState[64] = {}; // host tests: last digitalWrite level per pin
inline void digitalWrite(uint8_t pin, uint8_t val) { if (pin < 64) simPinState[pin] = val; }
inline int digitalRead(uint8_t) { return 0; }

class String {
    std::string s_;
public:
    String() = default;
    String(const char *s) : s_(s ? s : "") {}
    const char *c_str() const { return s_.c_str(); }
};

class IPAddress {
    uint32_t v_ = 0;
public:
    IPAddress() = default;
    explicit IPAddress(unsigned long v) : v_((uint32_t) v) {}
    explicit IPAddress(const char *) : v_(0) {}
    explicit operator bool() const { return v_ != 0; }
    String toString() const { return String("0.0.0.0"); }
};
