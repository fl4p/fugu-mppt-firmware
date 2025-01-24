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

    nvs_handle_t my_handle;

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
        auto constexpr maxLen = 64;
        char buf[64];
        size_t len = maxLen;
        auto err = nvs_get_str(my_handle, key.c_str(), buf, &len);
        if(err == ESP_ERR_NVS_NOT_FOUND)
            return defaultValue;
        ESP_ERROR_CHECK(err);
        return buf;
    }

    void close() {
        nvs_close(my_handle);
        my_handle = {};
    }

    void writeString(const std::string& key, const std::string& value) {
        auto err =nvs_set_str(my_handle, key.c_str(), value.c_str());
        ESP_ERROR_CHECK(err);
    }
};