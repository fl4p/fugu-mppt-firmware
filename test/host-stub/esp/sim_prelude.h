// Force-included (-include) into every TU of the host closed-loop sim (mppt-sim.cpp).
// Kconfig/sdkconfig of a CONFIG_FUGU_WITH_VCONV esp32s3 build, plus /littlefs path redirection.
#pragma once
#define CONFIG_IDF_TARGET "esp32s3"
#define CONFIG_IDF_TARGET_ESP32S3 1
#define ESP_IDF_VERSION_MAJOR 6
#define CONFIG_FUGU_WITH_VCONV 1
#define WITH_VCONV 1
#define WITH_PSU 1
#define ARDUINO 10000
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#include <cstdio>

FILE *sim_fopen(const char *path, const char *mode);
#define fopen(p, m) sim_fopen(p, m)
