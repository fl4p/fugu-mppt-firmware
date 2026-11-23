// Host tests for src/etc/con_guard.h: secret masking of console/conf lines, read-only command allowlist.
// The same vectors are checked against the host-side ports in test/host_py/test_mask_secrets.py.
//
// Build & run:
//   clang++ -std=gnu++20 -fexceptions -I test/host-stub -I src \
//       -o /tmp/con-guard-test test/host-stub/con-guard-test.cpp && /tmp/con-guard-test

#include <cstdio>
#include <iterator>
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

    // untrusted allowlist: registered SimpleCLI names (aliases included), arg count, first arg
    struct { const char *name; int n; const char *a0; bool ok; } ro[] = {
        {"status", 0, "", true}, {"sensor", 1, "avg", true}, {"hostname,hn", 0, "", true},
        {"hostname,hn", 1, "evil", false}, {"service,svc", 0, "", true}, {"service,svc", 1, "list", true},
        {"service,svc", 1, "off", false}, {"service,svc", 2, "list", false}, {"coredump", 1, "info", true},
        {"coredump", 1, "get", false}, {"coredump", 1, "erase", false}, {"heap", 1, "check", true},
        {"heap", 1, "chec", false}, {"get-config,getc", 2, "mqtt.conf", true}, {"cat", 1, "conf/wifi.conf", true},
        {"rect-cal", 1, "status", true}, {"rect-cal", 1, "force", false}, {"rect-cal", 0, "", false},
        {"heap", 0, "", true}, {"coredump", 0, "", true}, {"sensor", 0, "", true}, {"service,svc", 1, "", true},
        {"status", 1, "x", false}, {"heap", 1, "check,tasks", false}, {"heap", 1, "check,", false}, {"dt,deadtime", 1, "100", false},
        {"dt,deadtime", 0, "", true}, {"set-config,setc", 3, "mqtt.conf", false}, {"dc", 1, "0", false},
        {"restart,reset,reboot", 0, "", false}, {"ota", 1, "http://x", false}, {"ota-ble", 0, "", false},
        {"wifi-add", 1, "a:b", false}, {"wifi", 1, "off", false}, {"mppt", 0, "", false}, {"sweep", 0, "", false},
        {"bf,panel", 1, "0", false}, {"fan", 1, "1", false}, {"psu", 1, "off", false}, {"peek", 1, "0x0", false},
        {"run", 1, "s", false}, {"log", 2, "*", false}, {"ble", 1, "pair", false}, {"ble", 2, "unpair", false}, {"ble", 1, "bonds", true}, {"status2", 0, "", false},
        {"statu", 0, "", false}, {"hostname", 0, "", false},
    };
    for (auto &t: ro)
        if (roCmdAllowed(kRoCmds, std::size(kRoCmds), t.name, t.n, t.a0) != t.ok) {
            printf("FAIL roCmdAllowed('%s', %d, '%s') != %d\n", t.name, t.n, t.a0, t.ok);
            ++fails;
        }

    printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
    return fails != 0;
}
