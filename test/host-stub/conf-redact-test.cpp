// Host-side unit test for src/conf_redact.h, the secret redaction behind `conf-hash`. The cases
// mirror test/host_py/test_conf_hash.py::RedactionTest so the firmware and etc/conf_hash.py agree.
//
// Build & run:
//   clang++ -std=gnu++17 -I src -o /tmp/conf-redact-test test/host-stub/conf-redact-test.cpp \
//       && /tmp/conf-redact-test

#include <cstdio>
#include <string>

#include "conf_redact.h"

static int failures = 0;

static void check(bool ok, const char *what) {
    if (!ok) { std::printf("FAIL: %s\n", what); ++failures; }
}

static std::string redactAll(const std::string &data, int &n) {
    std::string out, line, red;
    n = 0;
    for (char c: data) {
        line.push_back(c);
        if (c == '\n') { n += confRedactLine(line, red); out += red; line.clear(); }
    }
    if (!line.empty()) { n += confRedactLine(line, red); out += red; }
    return out;
}

int main() {
    for (const char *k: {"ble_passkey", "password", "ftp_pass", "wifi_psk", "API_TOKEN", "my_secret",
                         "cloud_key", "Password"})
        check(confHashIsSecretKey(k), k);
    for (const char *k: {"ftp_user", "username", "ble_security", "hostname", "broker_uri", "keys", "key_x",
                         "vout_max"})
        check(!confHashIsSecretKey(k), k);

    const std::string a = "# ble_passkey=111111\n"
                          "ble_security=passkey\n"
                          "ble_passkey = 123456 \r\n"
                          "password=hunter2 # inline comment kept\n"
                          "ftp_pass=\n"
                          "API_TOKEN=abc\n"
                          "cloud_key=k1";
    const std::string b = "# ble_passkey=111111\n"
                          "ble_security=passkey\n"
                          "ble_passkey = 999999 \r\n"
                          "password=correct horse # inline comment kept\n"
                          "ftp_pass=secret2\n"
                          "API_TOKEN=zz\n"
                          "cloud_key=k2";
    const std::string expected = "# ble_passkey=111111\n"
                                 "ble_security=passkey\n"
                                 "ble_passkey = <redacted> \r\n"
                                 "password=<redacted> # inline comment kept\n"
                                 "ftp_pass=<redacted>\n"
                                 "API_TOKEN=<redacted>\n"
                                 "cloud_key=<redacted>";
    int n = 0;
    check(redactAll(a, n) == expected && n == 5, "fixture A redacts to EXPECTED");
    check(redactAll(b, n) == expected && n == 5, "fixture B redacts to EXPECTED");
    check(redactAll(std::string("x=1\0password=2\n", 15), n) == std::string("x=1\0password=2\n", 15) && n == 0,
          "NUL inside a non-secret line is kept");

    // ConfFile's fgets(buf, 256) parses this 272-byte line as a 255-byte chunk plus the rest, so a
    // secret key starting at byte 255 is live and must be redacted.
    {
        std::string longLine = "x=" + std::string(253, 'a') + "password=hunter2\n";
        std::string out, line, red;
        int hits = 0;
        for (char c: longLine) {
            line.push_back(c);
            if (c == '\n' || line.size() == CONF_LINE_CHUNK) { hits += confRedactLine(line, red); out += red; line.clear(); }
        }
        check(hits == 1 && out == "x=" + std::string(253, 'a') + "password=<redacted>\n", "255-byte chunking");
    }

    if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
    std::printf("conf-redact-test: all passed\n");
    return 0;
}
