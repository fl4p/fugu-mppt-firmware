#pragma once

#include <esp_http_client.h>
#include <esp_https_ota.h>

void systemRestart();

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
    client.setTimeout(12000);  // timeout argument is defined in milliseconds for setTimeout

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

#else

#define TAG "ota"

esp_err_t _ota_http_event_handler(esp_http_client_event_t *evt) {
    static int contentLen = 0;
    static int contentReceived = 0;
    static unsigned long tStart = 0;
    static uint8_t lastPct = 0;

    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            systemRestart();
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
                    if(pct == 100) {
                        ESP_LOGI(TAG, "Download took %lu ms (%lu KB/s)",  (wallClockUs()-tStart)/1000,
                                 contentReceived*1000/(wallClockUs()-tStart)
                                 );
                    }
                }
            }
            break;
        case HTTP_EVENT_ON_FINISH:

            break;
        case HTTP_EVENT_DISCONNECTED:
            if (lastPct != 100)
                ESP_LOGW(TAG, "HTTP_EVENT_DISCONNECTED @ %d %%", lastPct);
            systemRestart();
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGW(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}

#undef TAG



void doOta(const char *url) {
    //ESP_LOGE("ota", "Not available. Enable CONFIG_MBEDTLS_PSK_MODES");

    esp_http_client_config_t config{};
    config.url = url,
    config.timeout_ms = 10000,
    #ifdef CONFIG_EXAMPLE_USE_CERT_BUNDLE
            config.crt_bundle_attach = esp_crt_bundle_attach;
#else
                    config.cert_pem = 0,// (char *)server_cert_pem_start,
    #endif /* CONFIG_EXAMPLE_USE_CERT_BUNDLE */
            config.event_handler = _ota_http_event_handler;
    config.keep_alive_enable = true;
    config.buffer_size = 1024; // DEFAULT_HTTP_BUF_SIZE=512
#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_BIND_IF
    config.if_name = &ifr;
#endif


    config.skip_cert_common_name_check = true;

    esp_https_ota_config_t ota_config = {
            .http_config = &config,
            .http_client_init_cb = nullptr,
            .bulk_flash_erase = false,
            .partial_http_download = false,
            .max_http_request_size = 0,
        .buffer_caps=MALLOC_CAP_DEFAULT,
        .ota_resumption=false,
        .ota_image_bytes_written=0,
        .partition {                                        /*!< Details of staging and final partitions for OTA update */
                .staging = nullptr,             /*!< New image will be downloaded in this staging partition. If NULL then a free app partition (passive app partition) is selected as the staging partition. */
                .final=nullptr,               /*!< Final destination partition. Its type/subtype will be used for verification. If set to NULL, staging partition shall be set as the final partition. */
                .finalize_with_copy=false,                    /*!< Flag to copy the staging image to the final partition at the end of OTA update */
            },
    };
    ESP_LOGI("ota", "Attempting to download update from %s", config.url);

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI("ota", "OTA Succeed, Rebooting...");
        systemRestart();
    } else {
        ESP_LOGE("ota", "Firmware upgrade failed");
    }
}


#endif