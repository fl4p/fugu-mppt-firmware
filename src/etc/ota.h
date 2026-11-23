#pragma once

#include <esp_ota_ops.h>

// The running image still waits for its confirm; esp_ota_begin refuses until then.
inline bool otaPendingVerify() {
    esp_ota_img_states_t st;
    return esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK
           && st == ESP_OTA_IMG_PENDING_VERIFY;
}

// Pull-and-flash a firmware image over HTTP(S). Returns false on failure; on success the device
// reboots and never returns.
bool doOta(const char *url);

// Pull a bootloader image over HTTP(S) into free flash, verify it and copy it to the bootloader
// region. convOff() must hold before the download and again before the copy. Returns nullptr on
// success (written or already identical), else the reason. Takes effect on the next reset.
// url == nullptr skips the download and installs what is already staged.
const char *doOtaBootloader(const char *url, bool (*convOff)());
