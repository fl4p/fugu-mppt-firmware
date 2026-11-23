// Host tests for src/etc/con_guard.h: secret masking of console/conf lines.
// The same vectors are checked against the host-side ports in test/host_py/test_mask_secrets.py.
//
// Build & run:
//   clang++ -std=gnu++20 -fexceptions -I test/host-stub -I src \
//       -o /tmp/con-guard-test test/host-stub/con-guard-test.cpp && /tmp/con-guard-test

#include <cstdio>
#include <string>

#include "../../src/etc/con_guard.h"

static int fails = 0;

static std::string masked(const char *s) {
    char b[256];
    const int m = secretCut(s);
    snprintf(b, sizeof b, "%.*s%s", CG_MASKED(s, m));
    return b;
}

static void expectMask(const char *in, const char *want) {
    const std::string got = masked(in);
    if (got != want) {
        printf("FAIL mask '%s': got '%s', want '%s'\n", in, got.c_str(), want);
        ++fails;
    }
}

int main() {
    // vectors shared with test/host_py/test_mask_secrets.py
    expectMask("set-config mqtt.conf password hunter2", "set-config mqtt.conf password ***");
    expectMask("setc tlscon.conf psk 00112233445566778899aabbccddeeff", "setc tlscon.conf psk ***");
    expectMask("set-config wifi.conf ssid_home_psk my secret pw", "set-config wifi.conf ssid_home_psk ***");
    expectMask("set-config ftp.conf ftp_pass x", "set-config ftp.conf ftp_pass ***");
    expectMask("set-config ble.conf ble_passkey 123456", "set-config ble.conf ble_passkey ***");
    expectMask("SET-CONFIG mqtt.conf PASSWORD x", "SET-CONFIG mqtt.conf PASSWORD ***");
    expectMask("wifi-add home:hunter2", "wifi-add home:***");
    expectMask("status;;wifi-add home:pw;;mem", "status;;wifi-add home:***");
    expectMask("status;;set-config mqtt.conf password pw", "status;;set-config mqtt.conf password ***");
    expectMask("script-set s set-config mqtt.conf password pw; restart", "script-set s set-config mqtt.conf password ***");
    expectMask("curl -d token=abc http://x", "curl -d token=***");
    expectMask("password=hunter2", "password=***");
    expectMask("# ble_passkey=123456", "# ble_passkey=***");
    expectMask("ssid_home_psk = pw", "ssid_home_psk =***");
    expectMask("broker_uri=mqtt://u:pw@host:1883", "broker_uri=mqtt://***");
    expectMask("set-config mqtt.conf broker_uri mqtts://u:pw@host", "set-config mqtt.conf broker_uri mqtts://***");
    expectMask("ota http://10.0.0.2:9000/fw.bin", "ota http://10.0.0.2:9000/fw.bin");

    // no secret: unchanged
    expectMask("set-config coil.conf L0 50", "set-config coil.conf L0 50");
    expectMask("get-config mqtt.conf password", "get-config mqtt.conf password");
    expectMask("del-config tlscon.conf psk", "del-config tlscon.conf psk");
    expectMask("broker_uri=mqtt://1.2.3.4", "broker_uri=mqtt://1.2.3.4");
    expectMask("ssid_home=home", "ssid_home=home");
    expectMask("status", "status");
    expectMask("wifi-add nocolon", "wifi-add nocolon");

    if (!isSecretKey("password") || !isSecretKey("ssid_x_psk") || isSecretKey("broker_uri") || isSecretKey("cv_eoc")) {
        printf("FAIL isSecretKey\n");
        ++fails;
    }

    printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
    return fails != 0;
}
