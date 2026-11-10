// Bootloader-install guards and the copy to 0x0 without a network: run under QEMU (or a bench board)
// with MAIN_SRC=../test/main_bl_ota.cpp. A candidate image is expected at BL_SRC (put there by
// test/bl_ota_qemu.py); without one only the refusal cases run.
#include <Arduino.h>
#include <esp_flash.h>
#include <esp_flash_partitions.h>
#include <esp_image_format.h>
#include <esp_partition.h>
#include <spi_flash_mmap.h>
#include <cstring>
#include <algorithm>

#include "etc/ota.h"
#include "etc/sha256.h"
#include "util.h"

time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;
void systemRestart() { esp_restart(); }
void consoleFlush() {}

static constexpr uint32_t BL_SRC = 0x3F8000;
static bool convOff = true;
static int fails = 0;

static bool isOff() { return convOff; }

static uint32_t stageOffset() {
    uint32_t end = 0;
    for (auto it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, nullptr); it;
         it = esp_partition_next(it)) {
        auto p = esp_partition_get(it);
        end = std::max<uint32_t>(end, p->address + p->size);
    }
    return (end + SPI_FLASH_SEC_SIZE - 1) & ~(SPI_FLASH_SEC_SIZE - 1);
}

static bool stage(uint32_t from, uint32_t len, uint32_t flipAt = 0, uint8_t flip = 0, bool fixSha = false,
                  uint32_t flipAt2 = 0) {
    static uint8_t img[ESP_BOOTLOADER_SIZE];
    if (len > sizeof(img) || esp_flash_read(nullptr, img, from, len) != ESP_OK) return false;
    img[flipAt] ^= flip;
    if (flipAt2) img[flipAt2] ^= flip;
    if (fixSha) {
        Sha256 h;
        h.update(img, len - 32);
        h.finish(img + len - 32);
    }
    const uint32_t so = stageOffset();
    return esp_flash_erase_region(nullptr, so, ESP_BOOTLOADER_SIZE) == ESP_OK
           && esp_flash_write(nullptr, img, so, len) == ESP_OK;
}

static void expect(const char *name, const char *err, bool wantOk) {
    const bool pass = wantOk ? !err : !!err;
    if (!pass) ++fails;
    printf("BLTEST %s %s -> %s\n", pass ? "PASS" : "FAIL", name, err ? err : "ok");
}

void setup() {
    char info[112];
    uint32_t curLen = 0, srcLen = 0;
    bootloaderInfo(ESP_PRIMARY_BOOTLOADER_OFFSET, info, sizeof(info), &curLen);
    printf("BLTEST bootloader: %s\n", info);
    printf("BLTEST staging at 0x%x\n", (unsigned) stageOffset());

    convOff = false;
    expect("refuse while converting", doOtaBootloader(nullptr, isOff), false);
    convOff = true;

    stage(ESP_PRIMARY_BOOTLOADER_OFFSET, curLen);
    expect("identical image", doOtaBootloader(nullptr, isOff), true);

    stage(ESP_PRIMARY_BOOTLOADER_OFFSET, curLen, curLen / 2, 0x01);
    expect("corrupt payload byte", doOtaBootloader(nullptr, isOff), false);

    // same bit in two bytes 4 apart leaves the 8-bit XOR checksum intact: only the SHA-256 catches it
    stage(ESP_PRIMARY_BOOTLOADER_OFFSET, curLen, curLen / 2, 0x01, false, curLen / 2 + 4);
    expect("checksum-preserving corruption", doOtaBootloader(nullptr, isOff), false);

    stage(ESP_PRIMARY_BOOTLOADER_OFFSET, curLen, curLen - 5, 0x01);
    expect("corrupt appended SHA", doOtaBootloader(nullptr, isOff), false);

    stage(ESP_PRIMARY_BOOTLOADER_OFFSET, curLen / 2);
    expect("truncated image", doOtaBootloader(nullptr, isOff), false);

    stage(ESP_PRIMARY_BOOTLOADER_OFFSET, curLen, 12, 0x0f, true);
    expect("wrong chip id", doOtaBootloader(nullptr, isOff), false);

    stage(ESP_PRIMARY_BOOTLOADER_OFFSET, curLen, 2, 0x01, true);
    expect("different flash mode", doOtaBootloader(nullptr, isOff), false);

    stage(ESP_PRIMARY_BOOTLOADER_OFFSET, curLen, 3, 0x70, true);
    expect("flash size > chip", doOtaBootloader(nullptr, isOff), false);

    esp_flash_erase_region(nullptr, stageOffset(), ESP_BOOTLOADER_SIZE);
    expect("erased staging", doOtaBootloader(nullptr, isOff), false);

    if (bootloaderInfo(BL_SRC, info, sizeof(info), &srcLen)) {
        printf("BLTEST candidate: %s\n", info);
        stage(BL_SRC, srcLen);
        expect("install candidate", doOtaBootloader(nullptr, isOff), true);
        bootloaderInfo(ESP_PRIMARY_BOOTLOADER_OFFSET, info, sizeof(info));
        printf("BLTEST bootloader now: %s\n", info);
    }
    printf("BLTEST DONE fails=%d\n", fails);
}

void loop() { delay(1000); }
