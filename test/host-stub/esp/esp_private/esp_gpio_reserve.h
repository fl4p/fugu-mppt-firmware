#pragma once
#include <cstdint>
#define BIT64(n) (1ULL << (n))
#define GPIO_IS_VALID_OUTPUT_GPIO(p) ((p) < 49)
inline bool esp_gpio_is_reserved(uint64_t) { return false; }
