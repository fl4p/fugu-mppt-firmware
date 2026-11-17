#pragma once

#include <unity.h>
#include <esp_efuse.h>
#include <esp_efuse_table.h>

// QEMU boots with a blank efuse image (test/unity_qemu.py writes IDF's default one), so the factory
// MAC reads all-zero. Every real chip has it programmed by Espressif, so this cannot trigger on silicon.
inline bool runningInQemu() {
    static int8_t q = -1;
    if (q < 0) {
        uint8_t mac[6] = {};
        esp_efuse_read_field_blob(ESP_EFUSE_MAC_FACTORY, mac, 48);
        q = !(mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5]);
    }
    return q;
}

// Call first in the test, before it allocates a peripheral: TEST_IGNORE longjmps past destructors.
#define SKIP_IN_QEMU(why) do { if (runningInQemu()) TEST_IGNORE_MESSAGE("QEMU: " why); } while (0)
