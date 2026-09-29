#include "util.h"
#include <Wire.h>
#include <cmath>
#include <cctype>
#include <cstring>
#include <stdexcept>
#include <string>

void assertPinState(uint8_t pin, bool digitalVal, const char *pinName, bool weakBackPull) {
    // esp32s3 has 45k up/down pull resistors
    pinMode(pin, weakBackPull ? (digitalVal ? INPUT_PULLDOWN : INPUT_PULLUP) : INPUT);
    vTaskDelay(pdMS_TO_TICKS(5));
    auto read = digitalRead(pin);
    if (weakBackPull) pinMode(pin, INPUT);

    if (read != digitalVal) {
        auto msg = "pin " + std::to_string(pin) + (pinName ? ("(" + std::string(pinName) + ")") : "")
                   + " is not " + (digitalVal ? "HIGH" : "LOW") +
                   (weakBackPull ? (std::string(" with weak pull") + (digitalVal ? "down" : "up")) : "");
        throw std::runtime_error(msg);
    }
}

void scan_i2c() {
    const char *TAG = "scan_i2c";
    uint8_t error, address;
    int nDevices;

    ESP_LOGI(TAG, "Scanning I2C...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        /*
            0	success
            1	data too long to fit in transmit buffer
            2	received NACK on transmit of address
            3	received NACK on transmit of data
            4	other error
            5   timeout
         */

        if (error == 0) {
            ESP_LOGI(TAG, "Device found at address 0x%02X", address);
            nDevices++;
        } else if (error != 2) {
            ESP_LOGW(TAG, "Unknown error %u at address 0x%02X", error, address);
        }
    }
    if (nDevices == 0)
        ESP_LOGI(TAG, "No I2C devices found");
    else
        ESP_LOGI(TAG, "I2C scan done, %d devices found", nDevices);
}

float strntof(const char *dat, int len) {
    char buf[32];
    if (len <= 0 || len >= (int) sizeof(buf))
        return NAN;
    memcpy(buf, dat, len);
    buf[len] = '\0';
    char *end;
    float f = strtof(buf, &end);
    if (end == buf)
        return NAN;
    while (isspace((unsigned char) *end)) ++end;
    return *end ? NAN : f; // reject partial parses like "3.4V" or "unavailable"
}
