#pragma once

#include <unistd.h> // fsync
#include <stdint.h>

#define ESP_LOGE(tag, format, ...)       printf("[%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...)       printf("[%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...)       printf("[%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...)       printf("[%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGV(tag, format, ...)       printf("[%s] " format "\n", tag, ##__VA_ARGS__)
#define UART_LOG(format, ...)   printf("" format "\n", ##__VA_ARGS__)

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define OUTPUT 1

#define unlikely(x) (x)

void pinMode(u_int8_t, int) { }
void digitalWrite(u_int8_t, int) { }

void assertPinState(uint8_t pin, bool digitalVal, const char *pinName, bool weakBackPull) {}