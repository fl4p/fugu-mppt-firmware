// Force-included (-include) into every TU of the host closed-loop sim (mppt-sim.cpp).
// Kconfig/sdkconfig of a CONFIG_FUGU_WITH_VCONV esp32s3 build, plus /littlefs path redirection.
#pragma once
#define CONFIG_IDF_TARGET "esp32s3"
#define CONFIG_IDF_TARGET_ESP32S3 1
#define ESP_IDF_VERSION_MAJOR 6
#define CONFIG_FUGU_WITH_VCONV 1
#define WITH_VCONV 1
#define WITH_PSU 1
#define FUGU_HOST_SIM 1
#define ARDUINO 10000
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#include <cstdio>
#include <unistd.h>

FILE *sim_fopen(const char *path, const char *mode);
int sim_rename(const char *from, const char *to);
int sim_unlink(const char *path);
#define fopen(p, m) sim_fopen(p, m)
#define rename(...) sim_rename(__VA_ARGS__)
#define unlink(...) sim_unlink(__VA_ARGS__)
