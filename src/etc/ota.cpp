

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <esp_http_client.h>
#include <esp_https_ota.h>
#if CONFIG_FUGU_WITH_HTTPS
#include <esp_crt_bundle.h>
#endif

#include <esp_flash.h>
#include <esp_flash_partitions.h>
#include <esp_flash_encrypt.h>
#include <esp_efuse.h>
#include <spi_flash_mmap.h>
#include <esp_secure_boot.h>
#include <esp_image_format.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <algorithm>
#include <cstring>
#if __has_include(<esp_private/esp_flash_internal.h>)
#include <esp_private/esp_flash_internal.h>
#else
#include <esp_flash_internal.h>
#endif

#include "ota.h"
#include "sha256.h"
#include "util.h"
#include "logging.h"
#include "tele/console_ble.h"

constexpr auto OTA_TIMEOUT_SECONDS = 5 * 60;

void systemRestart();
void consoleFlush(); // push progress over a buffered console (BLE) during the blocking download


#define TAG "ota"

static int contentLen = 0;
static int contentReceived = 0;
static uint8_t lastPct = 0;

// esp_timer, not wallClockUs(): the blocking download stalls whichever loop advances the wall clock
esp_err_t _ota_http_event_handler(esp_http_client_event_t *evt) {
    static int64_t tStart = 0;

    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            //systemRestart();
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            if (strcmp(evt->header_key, "Content-Length") == 0) {
                contentLen = atoi(evt->header_value);
                contentReceived = 0;
                tStart = esp_timer_get_time();
            }
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            contentReceived += evt->data_len;
            {
                uint8_t pct = contentLen > 0 ? contentReceived * 100 / contentLen : 0;
                if (lastPct != pct) {
                    ESP_LOGI("ota", "Download Progress: %4d %%", pct);
                    lastPct = pct;
                    if (pct == 100) {
                        const int64_t dt = std::max<int64_t>(esp_timer_get_time() - tStart, 1);
                        ESP_LOGI(TAG, "Download took %llu ms (%llu KB/s)", (unsigned long long) (dt / 1000),
                                 (unsigned long long) (contentReceived * 1000LL / dt));
                    }
                    consoleFlush(); // download blocks the net loop; push this line to a buffered (BLE) console now
                }
            }
            break;
        case HTTP_EVENT_ON_FINISH:

            break;
        case HTTP_EVENT_DISCONNECTED:
            if (lastPct != 100) {
                ESP_LOGW(TAG, "HTTP_EVENT_DISCONNECTED @ %d %%", lastPct);
            }
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGW(TAG, "HTTP_EVENT_REDIRECT");
            break;
        default:
            break;
    }
    return ESP_OK;
}

#undef TAG


void otaWatchdog(void *arg) {
    ESP_LOGI("ota", "Watchdog task started");
    const int64_t tStart = esp_timer_get_time();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (esp_timer_get_time() - tStart > OTA_TIMEOUT_SECONDS * int64_t(1e6)) {
            ESP_LOGE("ota", "Ota timeout after %.1f seconds", (esp_timer_get_time() - tStart) * 1e-6f);
            systemRestart();
        }
    }
    //ESP_LOGI("ota", "Watchdog task ends");
}


static esp_err_t httpOta(const char *url, const esp_partition_t *staging, const esp_partition_t *final) {
    esp_http_client_config_t config{};
    config.url = url;
    config.timeout_ms = 10000;
#if CONFIG_FUGU_WITH_HTTPS
    config.crt_bundle_attach = esp_crt_bundle_attach;
#endif
    config.event_handler = _ota_http_event_handler;
    config.keep_alive_enable = true;
    config.buffer_size = 1024; // DEFAULT_HTTP_BUF_SIZE=512

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .http_client_init_cb = nullptr,
        .bulk_flash_erase = false,
#if CONFIG_ESP_HTTPS_OTA_ENABLE_PARTIAL_DOWNLOAD
        .partial_http_download = false,
        .max_http_request_size = 0,
#endif
        .buffer_caps = MALLOC_CAP_DEFAULT,
        .ota_resumption = false,
        .ota_image_bytes_written = 0,
        .partition{
            .staging = staging, // nullptr: passive app slot
            .final = final,
            .finalize_with_copy = false,
        },
    };
    ESP_LOGI("ota", "Attempting to download update from %s", config.url);
    contentLen = contentReceived = lastPct = 0;

    TaskHandle_t wdTask = nullptr;
    xTaskCreatePinnedToCore(otaWatchdog, "otaWD", 4096, NULL, 20, &wdTask, !RT_CORE); // systemRestart saves off RT
    if (!wdTask)
        return ESP_ERR_NO_MEM;
    esp_err_t ret = esp_https_ota(&ota_config);
    vTaskDelete(wdTask);
    return ret;
}

bool doOta(const char *url) {
    const int64_t t0 = esp_timer_get_time();
    // Every OTA ends in a reboot, so BLE need not come back: free it for the download.
    const unsigned h = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const bool bleGone = !g_restartBlocked && bleConsoleRelease(); // no reboot to bring it back
    if (bleGone) ESP_LOGI("ota", "BLE released, free heap %u -> %u", h, (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    const esp_err_t e = httpOta(url, nullptr, nullptr);
    if (e == ESP_OK) {
        ESP_LOGI("ota", "OTA Succeed, Rebooting...");
        consoleFlush(); // get the result out over a buffered console before the reboot tears the link down
        systemRestart();
        return true;
    }
    logRetained("E (%lu) ota: update failed: %s at %d %% (%d/%d B) after %.1f s, old image kept",
                (unsigned long) esp_log_timestamp(), esp_err_to_name(e), lastPct, contentReceived, contentLen,
                (esp_timer_get_time() - t0) * 1e-6f);
    consoleFlush();
    if (bleGone) systemRestart(); // BLE cannot be restarted in this boot
    return false;
}

static bool flashSha(uint32_t off, uint32_t len, uint8_t out[32]) {
    Sha256 h;
    uint8_t b[256];
    for (uint32_t i = 0; i < len; i += sizeof(b)) {
        const uint32_t n = std::min<uint32_t>(sizeof(b), len - i);
        if (esp_flash_read(nullptr, b, off + i, n) != ESP_OK) return false;
        h.update(b, n);
    }
    return h.finish(out);
}

static void dangerousWriteProtection(bool on) {
    esp_flash_set_dangerous_write_protection(esp_flash_default_chip, on);
}

static const char *blStageAndCopy(const char *url, const esp_partition_t *prim, const esp_partition_t *stage,
                                  bool (*convOff)()) {
    if (url) {
        const esp_err_t e = httpOta(url, stage, prim);
        dangerousWriteProtection(true); // esp_ota_set_final_partition() lifts it for the rest of the uptime
        if (e != ESP_OK) return "download or image check failed";
    }

    char info[112];
    uint32_t len = 0, phys = 0;
    if (!bootloaderInfo(stage->address, info, sizeof(info), &len)) return "staged image does not verify";
    ESP_LOGI("ota", "new bootloader: %s", info);

    esp_image_header_t nh{}, ch{};
    uint8_t a[32], b[32];
    if (esp_flash_read(nullptr, &nh, stage->address, sizeof(nh)) != ESP_OK
        || esp_flash_read(nullptr, &ch, ESP_PRIMARY_BOOTLOADER_OFFSET, sizeof(ch)) != ESP_OK
        || esp_flash_get_physical_size(esp_flash_default_chip, &phys) != ESP_OK)
        return "flash read failed";
    if (nh.magic != ESP_IMAGE_HEADER_MAGIC || nh.chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID)
        return "not a bootloader for this chip";
    // IDF checks only the 8-bit checksum of a bootloader (esptool may patch its header), so check the hash here
    if (!nh.hash_appended || len <= sizeof(nh) + 32 || len > ESP_BOOTLOADER_SIZE) return "no SHA-256 or bad length";
    if (!flashSha(stage->address, len - 32, a) || esp_flash_read(nullptr, b, stage->address + len - 32, 32) != ESP_OK
        || memcmp(a, b, 32))
        return "SHA-256 mismatch";
    if (nh.spi_mode != ch.spi_mode || nh.spi_size >= ESP_IMAGE_FLASH_SIZE_MAX || (0x100000u << nh.spi_size) > phys)
        return "flash mode/size in the image header does not match this device";

    if (!flashSha(ESP_PRIMARY_BOOTLOADER_OFFSET, len, a) || !flashSha(stage->address, len, b))
        return "flash read failed";
    if (!memcmp(a, b, 32)) {
        g_restartBlocked = false;
        ESP_LOGI("ota", "bootloader identical, not written");
        return nullptr;
    }
    if (!convOff()) return "conversion running again, not written";

    ESP_LOGW("ota", "writing bootloader (%u bytes), do not reset or power off", (unsigned) len);
    consoleFlush();
    bool ok = false;
    g_restartBlocked = true;
    dangerousWriteProtection(false);
    for (int i = 0; i < 3 && !ok; ++i)
        ok = esp_partition_copy(prim, 0, stage, 0, len) == ESP_OK && flashSha(ESP_PRIMARY_BOOTLOADER_OFFSET, len, a)
             && !memcmp(a, b, 32);
    dangerousWriteProtection(true);
    if (!ok) return "BOOTLOADER WRITE FAILED, do not reset or power off, retry the update";
    g_restartBlocked = false;
    ESP_LOGI("ota", "bootloader written and verified, active after the next reset");
    return nullptr;
}

const char *doOtaBootloader(const char *url, bool (*convOff)()) {
    if (esp_efuse_is_flash_encryption_enabled() || esp_secure_boot_enabled())
        return "flash encryption or secure boot enabled";
    if (otaPendingVerify())
        return "running app not yet confirmed (rollback pending)";
    if (!convOff()) return "conversion running, stop it first (dc 0)";

    // staging: unallocated flash after the last partition, keeps the passive slot's rollback image
    uint32_t stageOff = 0, flashSize = 0;
    for (auto it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, nullptr); it;
         it = esp_partition_next(it)) {
        const esp_partition_t *p = esp_partition_get(it);
        if (p->flash_chip == esp_flash_default_chip) stageOff = std::max<uint32_t>(stageOff, p->address + p->size);
    }
    stageOff = (stageOff + SPI_FLASH_SEC_SIZE - 1) & ~(SPI_FLASH_SEC_SIZE - 1);
    if (esp_flash_get_size(esp_flash_default_chip, &flashSize) != ESP_OK
        || stageOff + ESP_BOOTLOADER_SIZE > flashSize)
        return "no free flash after the last partition for staging";

    const esp_partition_t *prim = esp_partition_find_first(ESP_PARTITION_TYPE_BOOTLOADER,
                                                           ESP_PARTITION_SUBTYPE_BOOTLOADER_PRIMARY, nullptr);
    const esp_partition_t *stage = nullptr;
    const bool ownPrim = !prim;
    if (ownPrim && esp_partition_register_external(nullptr, ESP_PRIMARY_BOOTLOADER_OFFSET, ESP_BOOTLOADER_SIZE,
                                                   "blprim", ESP_PARTITION_TYPE_BOOTLOADER,
                                                   ESP_PARTITION_SUBTYPE_BOOTLOADER_PRIMARY, &prim) != ESP_OK)
        return "cannot register the bootloader region";
    const char *err;
    if (prim->address != ESP_PRIMARY_BOOTLOADER_OFFSET)
        err = "unexpected bootloader partition";
    else if (esp_partition_register_external(nullptr, stageOff, ESP_BOOTLOADER_SIZE, "blstage",
                                             ESP_PARTITION_TYPE_BOOTLOADER, ESP_PARTITION_SUBTYPE_BOOTLOADER_OTA,
                                             &stage) != ESP_OK)
        err = "cannot register the staging region";
    else {
        ESP_LOGI("ota", "staging bootloader at 0x%x", (unsigned) stageOff);
        err = blStageAndCopy(url, prim, stage, convOff);
        esp_partition_deregister_external(stage);
    }
    if (ownPrim) esp_partition_deregister_external(prim);
    consoleFlush();
    return err;
}


