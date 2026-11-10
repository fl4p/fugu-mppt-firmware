#include "util.h"
#include <Wire.h>
#include <cmath>
#include <cctype>
#include <cstring>
#include <stdexcept>
#include <string>
#include <esp_flash.h>
#include <esp_flash_partitions.h>
#include <esp_image_format.h>
#include <esp_bootloader_desc.h>

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

volatile bool g_restartBlocked = false;

bool bootloaderInfo(uint32_t offset, char *buf, unsigned n, uint32_t *len) {
    struct __attribute__((packed)) {
        esp_image_header_t h;
        esp_image_segment_header_t s;
        esp_bootloader_desc_t d;
    } r;
    esp_image_metadata_t md{};
    const esp_partition_pos_t pos{.offset = offset, .size = ESP_BOOTLOADER_SIZE};
    if (offset != ESP_PRIMARY_BOOTLOADER_OFFSET) esp_image_bootloader_offset_set(offset);
    bool ok = esp_image_verify(ESP_IMAGE_VERIFY_SILENT, &pos, &md) == ESP_OK
              && esp_flash_read(nullptr, &r, offset, sizeof(r)) == ESP_OK;
    if (offset != ESP_PRIMARY_BOOTLOADER_OFFSET) esp_image_bootloader_offset_set(ESP_PRIMARY_BOOTLOADER_OFFSET);
    if (!ok) {
        snprintf(buf, n, "invalid image @0x%x", (unsigned) offset);
        return false;
    }
    if (len) *len = md.image_len;
    int l = r.d.magic_byte == ESP_BOOTLOADER_DESC_MAGIC_BYTE
                ? snprintf(buf, n, "idf %.32s v%u, %.24s", r.d.idf_ver, (unsigned) r.d.version, r.d.date_time)
                : snprintf(buf, n, "no descriptor (old IDF)");
    if (l < 0 || (unsigned) l >= n) return true;
    l += snprintf(buf + l, n - l, ", len %u", (unsigned) md.image_len);
    if (md.image.hash_appended && (unsigned) l < n)
        snprintf(buf + l, n - l, ", sha %02x%02x%02x%02x%02x%02x%02x%02x", md.image_digest[0], md.image_digest[1],
                 md.image_digest[2], md.image_digest[3], md.image_digest[4], md.image_digest[5],
                 md.image_digest[6], md.image_digest[7]);
    return true;
}
