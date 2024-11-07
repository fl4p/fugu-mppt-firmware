#pragma once

#include <cstdint>
#include <Esp.h>
//#include <driver/adc.h>
//#include <esp_adc_cal.h>

enum class PinConfigESP32S3 : uint8_t {
    // https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf#page=10
    I2C_SDA = 42,
    I2C_SCL = 2,
    ADS_ALERT = 6,

    Bridge_IN = 16,
    Bridge_EN = 15,
    Backflow_EN = 8,
    Backflow_SD = 0,

    LED = 48,
    //NTC = 7, // ch6
    //ADC_NTC = ADC1_CHANNEL_6,
    //Fan = 36,


    buttonLeft = 39,
    buttonRight = 37,
    buttonBack = 40,
    buttonSelect = 1,

    //ADC_Vin = ADC1_CHANNEL_4, // GPIO5
   // ADC_Vout = ADC1_CHANNEL_5, // GPIO6
   // ADC_Iin = ADC1_CHANNEL_3,

    INA22x_ALERT = 0,
};

enum class PinConfigESP32 : uint8_t {
    I2C_SDA = 21,
    I2C_SCL = 22,
    ADS_ALERT = 34,

    Bridge_IN = 33,
    Bridge_EN = 32,
    Backflow_EN = 27,

    LED = 2,
    //NTC = 35,
    //ADC_NTC = ADC1_CHANNEL_7, //io35
    //Fan = 16,


    buttonLeft = 18,
    buttonRight = 17,
    buttonBack = 19,
    buttonSelect = 23,

    //ADC_Vin = ADC1_CHANNEL_3,
   // ADC_Vout = ADC1_CHANNEL_6,
   // ADC_Iin = ADC1_CHANNEL_0,
};

enum class PinConfigESP32S3_v2 : uint8_t {
    // https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf#page=10
    //I2C_SDA = 42,
    //I2C_SCL = 2,
    // ADS_ALERT = 0,
    //INA22x_ALERT = 41,

    Bridge_IN = 21, // f2:14
    Bridge_EN = 14, // f2:13

    Backflow_EN = 0,
    Backflow_SD = 40,

    //LED = 1,
    //ADC_NTC = ADC1_CHANNEL_7, // f2:6
    //Fan = 36, // TODO


    buttonLeft = 39,
    buttonRight = 37,
    buttonBack = 40,
    buttonSelect = 1,

    //ADC_Vin = ADC1_CHANNEL_3, // gpio4
    //ADC_Vout = ADC1_CHANNEL_MAX, // n/a
    //ADC_Iin = ADC1_CHANNEL_MAX, // n/a
};



const char *getChipId();


#if CONFIG_IDF_TARGET_ESP32S3
//typedef PinConfigESP32S3 PinConfig;
typedef PinConfigESP32S3_v2 PinConfig;
#else
typedef PinConfigESP32 PinConfig;
#endif

static uint8_t getBuckIN_PIN() {
    if (ESP.getEfuseMac() == 0x704082188534 /* white dot on cover near pin 16*/) {
        ESP_LOGI("pins", "pin %i output broken, using pin 17 for Bridge_IN", (int) PinConfig::Bridge_IN);
        return 17;
    }
    return (uint8_t) PinConfig::Bridge_IN;
}

