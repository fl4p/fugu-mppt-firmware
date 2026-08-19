#pragma once

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string>

class KeyValueStorage {

    nvs_handle_t my_handle = 0;

public:
    void init() {
        // Initialize NVS
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);
    }

    void open() {
        auto err = nvs_open("storage", NVS_READWRITE, &my_handle);
        ESP_ERROR_CHECK(err);
    }

    std::string readString(const std::string &key, const std::string &defaultValue) {
        // Query the stored length first, then size exactly. A fixed buffer aborts the device
        // (ESP_ERR_NVS_INVALID_LENGTH -> ESP_ERROR_CHECK) on any value >= the buffer, which a
        // user-settable key (e.g. an over-long hostname) would turn into a permanent boot loop.
        size_t len = 0;
        auto err = nvs_get_str(my_handle, key.c_str(), nullptr, &len);
        if (err == ESP_ERR_NVS_NOT_FOUND)
            return defaultValue;
        ESP_ERROR_CHECK(err);
        if (len <= 1)
            return {};
        std::string out(len - 1, '\0'); // len counts the NUL; data()[len-1] is the writable terminator
        err = nvs_get_str(my_handle, key.c_str(), out.data(), &len);
        ESP_ERROR_CHECK(err);
        return out;
    }

    void close() {
        nvs_close(my_handle);
        my_handle = {};
    }

    void writeString(const std::string &key, const std::string &value) {
        auto err = nvs_set_str(my_handle, key.c_str(), value.c_str());
        ESP_ERROR_CHECK(err);
    }

    // nvs_set_* only stages the write; without this the value is lost on esp_restart().
    void commit() {
        ESP_ERROR_CHECK(nvs_commit(my_handle));
    }
};