

#include <esp_log.h>
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

constexpr auto OTA_TIMEOUT_SECONDS = 2 * 60;

void systemRestart();
void consoleFlush(); // push progress over a buffered console (BLE) during the blocking download

#if CONFIG_MBEDTLS_PSK_MODES

#include <HTTPUpdate.h>

/**
 * This is lets-encrypt-x3-cross-signed.pem
 */
const char *rootCACertificate = "-----BEGIN CERTIFICATE-----\n"
        "MIIEkjCCA3qgAwIBAgIQCgFBQgAAAVOFc2oLheynCDANBgkqhkiG9w0BAQsFADA/\n"
        "MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n"
        "DkRTVCBSb290IENBIFgzMB4XDTE2MDMxNzE2NDA0NloXDTIxMDMxNzE2NDA0Nlow\n"
        "SjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxIzAhBgNVBAMT\n"
        "GkxldCdzIEVuY3J5cHQgQXV0aG9yaXR5IFgzMIIBIjANBgkqhkiG9w0BAQEFAAOC\n"
        "AQ8AMIIBCgKCAQEAnNMM8FrlLke3cl03g7NoYzDq1zUmGSXhvb418XCSL7e4S0EF\n"
        "q6meNQhY7LEqxGiHC6PjdeTm86dicbp5gWAf15Gan/PQeGdxyGkOlZHP/uaZ6WA8\n"
        "SMx+yk13EiSdRxta67nsHjcAHJyse6cF6s5K671B5TaYucv9bTyWaN8jKkKQDIZ0\n"
        "Z8h/pZq4UmEUEz9l6YKHy9v6Dlb2honzhT+Xhq+w3Brvaw2VFn3EK6BlspkENnWA\n"
        "a6xK8xuQSXgvopZPKiAlKQTGdMDQMc2PMTiVFrqoM7hD8bEfwzB/onkxEz0tNvjj\n"
        "/PIzark5McWvxI0NHWQWM6r6hCm21AvA2H3DkwIDAQABo4IBfTCCAXkwEgYDVR0T\n"
        "AQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwfwYIKwYBBQUHAQEEczBxMDIG\n"
        "CCsGAQUFBzABhiZodHRwOi8vaXNyZy50cnVzdGlkLm9jc3AuaWRlbnRydXN0LmNv\n"
        "bTA7BggrBgEFBQcwAoYvaHR0cDovL2FwcHMuaWRlbnRydXN0LmNvbS9yb290cy9k\n"
        "c3Ryb290Y2F4My5wN2MwHwYDVR0jBBgwFoAUxKexpHsscfrb4UuQdf/EFWCFiRAw\n"
        "VAYDVR0gBE0wSzAIBgZngQwBAgEwPwYLKwYBBAGC3xMBAQEwMDAuBggrBgEFBQcC\n"
        "ARYiaHR0cDovL2Nwcy5yb290LXgxLmxldHNlbmNyeXB0Lm9yZzA8BgNVHR8ENTAz\n"
        "MDGgL6AthitodHRwOi8vY3JsLmlkZW50cnVzdC5jb20vRFNUUk9PVENBWDNDUkwu\n"
        "Y3JsMB0GA1UdDgQWBBSoSmpjBH3duubRObemRWXv86jsoTANBgkqhkiG9w0BAQsF\n"
        "AAOCAQEA3TPXEfNjWDjdGBX7CVW+dla5cEilaUcne8IkCJLxWh9KEik3JHRRHGJo\n"
        "uM2VcGfl96S8TihRzZvoroed6ti6WqEBmtzw3Wodatg+VyOeph4EYpr/1wXKtx8/\n"
        "wApIvJSwtmVi4MFU5aMqrSDE6ea73Mj2tcMyo5jMd6jmeWUHK8so/joWUoHOUgwu\n"
        "X4Po1QYz+3dszkDqMp4fklxBwXRsW10KXzPMTZ+sOPAveyxindmjkW8lGy+QsRlG\n"
        "PfZ+G6Z6h7mjem0Y+iWlkYcV4PIWL1iwBi8saCbGS5jN2p8M+X+Q7UNKEkROb3N6\n"
        "KOqkqm57TH2H3eDJAkSnh6/DNFu0Qg==\n"
        "-----END CERTIFICATE-----\n";


void doOta(String url) {
    NetworkClientSecure client;
    client.setCACert(rootCACertificate);

    // Reading data over SSL may be slow, use an adequate timeout
    client.setTimeout(12000); // timeout argument is defined in milliseconds for setTimeout

    // The line below is optional. It can be used to blink the LED on the board during flashing
    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
    // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
    // httpUpdate.setLedPin(LED_BUILTIN, HIGH);
    //LED_BUILTIN

    ESP_LOGI("ota", "Trying to update from %s", url.c_str());

    t_httpUpdate_return ret = httpUpdate.update(client, url, "", [](HTTPClient *client) {
        //client->setAuthorization("test", "password");
    });
    // Or:
    //t_httpUpdate_return ret = httpUpdate.update(client, "server", 443, "/file.bin");

    switch (ret) {
        case HTTP_UPDATE_FAILED:
            ESP_LOGE("ota", "HTTP_UPDATE_FAILED Error (%d): %s", httpUpdate.getLastError(),
                     httpUpdate.getLastErrorString().c_str());
            break;

        case HTTP_UPDATE_NO_UPDATES:
            ESP_LOGI("ota", "HTTP_UPDATE_NO_UPDATES");
            break;

        case HTTP_UPDATE_OK:
            ESP_LOGI("ota", "HTTP_UPDATE_OK");
            break;
    }
}

const char *doOtaBootloader(const char *, bool (*)()) { return "not supported in this build"; }

#else

#define TAG "ota"

esp_err_t _ota_http_event_handler(esp_http_client_event_t *evt) {
    static int contentLen = 0;
    static int contentReceived = 0;
    static time_us tStart = 0;
    static uint8_t lastPct = 0;

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
                tStart = wallClockUs();
            }
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            contentReceived += evt->data_len;
            {
                uint8_t pct = contentReceived * 100 / contentLen;
                if (lastPct != pct) {
                    ESP_LOGI("ota", "Download Progress: %4d %%", pct);
                    lastPct = pct;
                    if (pct == 100) {
                        ESP_LOGI(TAG, "Download took %llu ms (%llu KB/s)", (wallClockUs() - tStart) / 1000,
                                 contentReceived * 1000 / (wallClockUs() - tStart)
                        );
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
    auto tStart = wallClockUs();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (wallClockUs() - tStart > OTA_TIMEOUT_SECONDS * int(1e6)) {
            ESP_LOGE("ota", "Ota timeout after %.1f seconds", (wallClockUs() - tStart) * 1e-6f);
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
    config.skip_cert_common_name_check = true;

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .http_client_init_cb = nullptr,
        .bulk_flash_erase = false,
#if ESP_IDF_VERSION_MAJOR < 6 || CONFIG_ESP_HTTPS_OTA_ENABLE_PARTIAL_DOWNLOAD
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

    TaskHandle_t wdTask = nullptr;
    xTaskCreate(otaWatchdog, "otaWD", 4096, NULL, 20, &wdTask);
    if (!wdTask)
        return ESP_ERR_NO_MEM;
    esp_err_t ret = esp_https_ota(&ota_config);
    vTaskDelete(wdTask);
    return ret;
}

bool doOta(const char *url) {
    if (httpOta(url, nullptr, nullptr) == ESP_OK) {
        ESP_LOGI("ota", "OTA Succeed, Rebooting...");
        consoleFlush(); // get the result out over a buffered console before the reboot tears the link down
        systemRestart();
        return true;
    }
    ESP_LOGE("ota", "Firmware upgrade failed");
    consoleFlush();
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
#if ESP_IDF_VERSION_MAJOR >= 6
    if (esp_efuse_is_flash_encryption_enabled() || esp_secure_boot_enabled())
#else
    if (esp_flash_encryption_enabled() || esp_secure_boot_enabled())
#endif
        return "flash encryption or secure boot enabled";
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK
        && st == ESP_OTA_IMG_PENDING_VERIFY)
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


#endif
