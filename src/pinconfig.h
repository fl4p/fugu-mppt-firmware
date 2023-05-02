#pragma once

#include <cstdint>

enum class PinConfigESP32S3  : uint8_t {
    I2C_SDA = 42,
    I2C_SCL = 2,
    ADC_ALERT = 6,

    Bridge_IN = 16,
    Bridge_EN = 15,
    Backflow_EN = 8,

};

enum class PinConfigESP32  : uint8_t {
    I2C_SDA = 21,
    I2C_SCL = 22,
    ADC_ALERT = 34,

    Bridge_IN = 33,
    Bridge_EN = 32,
    Backflow_EN = 27,
};

#if CONFIG_IDF_TARGET_ESP32S3
typedef PinConfigESP32S3 PinConfig;
#else
typedef PinConfigESP32 PinConfig;
#endif
