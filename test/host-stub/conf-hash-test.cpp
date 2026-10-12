// Host-side test of the firmware's conf-hash code: compiles src/conf_hash.cpp itself (everything but
// the littlefs mount check, NVS iteration and mbedTLS, which are ESP_PLATFORM-only) and checks it
// against the SAME frozen digests and vectors as test/host_py/test_conf_hash.py.
//
// Build & run:
//   clang++ -std=gnu++17 -Wall -I test/host-stub -I src -o /tmp/conf-hash-test \
//       test/host-stub/conf-hash-test.cpp && /tmp/conf-hash-test

#include "../../src/conf_hash.cpp"

#include <cstdlib>
#include <string>
#include <unistd.h>

static int failures = 0;

static void check(bool ok, const std::string &what) {
    if (!ok) { std::printf("FAIL: %s\n", what.c_str()); ++failures; }
}

static void put(const std::string &path, const std::string &data) {
    FILE *f = fopen(path.c_str(), "wb");
    if (!f) { std::printf("cannot write %s\n", path.c_str()); std::exit(2); }
    fwrite(data.data(), 1, data.size(), f);
    fclose(f);
}

static std::string mktmp() {
    char t[] = "/tmp/conf-hash-test.XXXXXX";
    if (!mkdtemp(t)) std::exit(2);
    return t;
}

static std::string redactAll(const std::string &in, uint32_t &n) {
    ConfHashRedactor r;
    std::string out;
    auto emit = [&](const std::string &s) { out += s; };
    r.feed(in.data(), in.size(), emit);
    r.finish(emit);
    n = r.redacted;
    return r.tooLong ? "<TOO LONG>" : out;
}

#define S(lit) std::string(lit, sizeof(lit) - 1) // keeps embedded NULs

int main() {
    // The stand-in SHA-256 itself (FIPS 180-4 test vector "abc").
    {
        Sha256 h;
        h.update("abc", 3);
        uint8_t d[32];
        h.finish(d);
        check(confHashHex(d, 32) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "sha256(abc)");
    }

    // Basic fixture: order, wifi.conf exclusion, subdirectory, framing. FROZEN_BASIC in the Python test.
    {
        const std::string root = mktmp(), c = root + "/conf";
        mkdir(c.c_str(), 0755);
        mkdir((c + "/sub").c_str(), 0755);
        put(c + "/board.conf", "mcu=esp32s3\n");
        put(c + "/a_b.conf", "k=1\n");
        put(c + "/a.conf", "");
        put(c + "/Z.conf", S("\x00\xff raw\r\n"));
        put(c + "/sub.conf", "s=1\n");
        put(c + "/sub/x.conf", "x=2\n");
        put(c + "/wifi.conf", "ssid=lab\npsk=secret\n");
        ConfHashResult r;
        check(confHashComputeAt(r, true, c), "basic: ok (" + r.err + ")");
        check(confHashHex(r.sha, 32) == "280be720036dbc75cd7420b7c96ea303e097c576eca66ba6f27d439b411417df",
              "basic: frozen digest, got " + confHashHex(r.sha, 32));
        check(r.files == 6 && r.bytes == 32 && r.redacted == 0, "basic: counts");
        const char *order[] = {"Z.conf", "a.conf", "a_b.conf", "board.conf", "sub.conf", "sub/x.conf"};
        for (size_t i = 0; i < 6 && i < r.perFile.size(); ++i) check(r.perFile[i].relpath == order[i], order[i]);

        put(c + "/board.conf.new", "x=1\n");
        check(!confHashComputeAt(r, false, c) && r.err == "staging file present: board.conf.new",
              "staging file fails: " + r.err);
        unlink((c + "/board.conf.new").c_str());
        symlink(c.c_str(), (c + "/sub/loop").c_str());
        check(!confHashComputeAt(r, false, c) && r.err.find("not a regular file") == 0, "symlink fails");
        unlink((c + "/sub/loop").c_str());
        check(!confHashComputeAt(r, false, root + "/nope") && r.err == "missing " + root + "/nope", "missing dir");
        std::string cmd = "rm -rf '" + root + "'";
        (void) !system(cmd.c_str());
    }

    // Depth limit: 8 nested directories are fine, a 9th is an error.
    {
        const std::string root = mktmp();
        std::string p = root;
        for (int i = 0; i < 8; ++i) { p += "/d" + std::to_string(i); mkdir(p.c_str(), 0755); }
        put(p + "/x.conf", "x=1\n");
        ConfHashResult r;
        check(confHashComputeAt(r, true, root) && r.files == 1, "depth 8 ok");
        mkdir((p + "/d8").c_str(), 0755);
        check(!confHashComputeAt(r, false, root) && r.err.find("too deep") == 0, "depth 9 fails");
        std::string cmd = "rm -rf '" + root + "'";
        (void) !system(cmd.c_str());
    }

    // Redaction vectors: REDACT_VECTORS in the Python test.
    struct V { std::string in, out; uint32_t n; };
    const V vectors[] = {
        {"ble_passkey=123456\n", "ble_passkey=<redacted>\n", 1},
        {std::string(243, ' ') + "ble_passkey=123456\n", std::string(243, ' ') + "ble_passkey=<redacted>\n", 1},
        {"password=" + std::string(246, 'a') + "XYZ", "password=<redacted>", 1},
        {"# ble_passkey=old\n", "# ble_passkey=<redacted>\n", 1},
        {"  #\t# password = x # c\r\n", "  #\t# password =<redacted>\n", 1},
        {"ftp_pass=\n", "ftp_pass=<redacted>\n", 1},
        {"Wifi_PSK=x\n", "Wifi_PSK=<redacted>\n", 1},
        {"cloud_key=k\n", "cloud_key=<redacted>\n", 1},
        {"ble_security=passkey\n", "ble_security=passkey\n", 0},
        {"username=bob\n", "username=bob\n", 0},
        {"broker_uri=mqtt://u:p@host:1883\n", "broker_uri=mqtt://<redacted>@host:1883\n", 1},
        {"broker_uri=mqtt://host:1883\n", "broker_uri=mqtt://host:1883\n", 0},
        {"u=http://host/a@b?c@d\n", "u=http://<redacted>@d\n", 1},
        {"u=mqtt://a@b@host/x@y\n", "u=mqtt://<redacted>@y\n", 1},
        {"# old=mqtt://u:p@h#x\n", "# old=mqtt://<redacted>@h#x\n", 1},
        {"u=a://x@h b://y@g\n", "u=a://<redacted>@g\n", 1},
        {"u=file:///p@q\n", "u=file://<redacted>@q\n", 1},
        {"u=mqtt://h#frag@x\n", "u=mqtt://<redacted>@x\n", 1},
        {"u=a://longuserinfo1234@h b://y@g\n", "u=a://<redacted>@g\n", 1},
        {"broker_uri=mqtt://u:pa/ss@host\n", "broker_uri=mqtt://<redacted>@host\n", 1},
        {"broker_uri=mqtt://u:pa?ss@host\n", "broker_uri=mqtt://<redacted>@host\n", 1},
        {"broker_uri=mqtt://u:pa#ss@host\n", "broker_uri=mqtt://<redacted>@host\n", 1},
        {"u=a://h/x b://y@g\n", "u=a://<redacted>@g\n", 1},
        {"u=a://x@h\nv=b://y@g", "u=a://<redacted>@h\nv=b://<redacted>@g", 2},
        {"u=a://h b://g\n", "u=a://h b://g\n", 0},
        {"no equals password here\n", "no equals password here\n", 0},
        {"x=1\npsk=2", "x=1\npsk=<redacted>", 1},
        {"a=" + std::string(253, 'x') + "password=hunter2\n", "a=" + std::string(253, 'x') + "password=<redacted>\n", 1},
        {"# a=" + std::string(251, 'x') + "password=hunter2\n", "# a=" + std::string(251, 'x') + "password=<redacted>\n", 1},
        {"a=" + std::string(253, 'x') + "cloud_key = k1\n", "a=" + std::string(253, 'x') + "cloud_key =<redacted>\n", 1},
        {"note=about passwords=here\n", "note=about passwords=<redacted>\n", 1},
        {"x=api_KEY\t=v\n", "x=api_KEY\t=<redacted>\n", 1},
    };
    for (const auto &v: vectors) {
        uint32_t n = 0;
        const std::string got = redactAll(v.in, n);
        check(got == v.out && n == v.n, "redact: " + v.in.substr(0, 60) + " -> " + got.substr(0, 60));
    }
    {
        std::string v = "a://x@h\nb://y@g", w = "a://x\n@g";
        check(confHashRedactUris(v) == 2 && v == "a://<redacted>@h\nb://<redacted>@g", "uri: value with newline");
        check(confHashRedactUris(w) == 0 && w == "a://x\n@g", "uri: '@' after newline not used");
    }
    for (const char *k: {"ble_passkey", "password", "ftp_pass", "wifi_psk", "API_TOKEN", "my_secret",
                         "cloud_key", "Password", "api_Key"})
        check(confHashIsSecretKey(k), std::string("secret: ") + k);
    for (const char *k: {"ftp_user", "username", "ble_security", "hostname", "broker_uri", "keys", "key_x",
                         "vout_max"})
        check(!confHashIsSecretKey(k), std::string("not secret: ") + k);
    {
        uint32_t n;
        check(redactAll(std::string(4096, 'a') + "\n", n) != "<TOO LONG>", "4096-byte line ok");
        check(redactAll(std::string(4097, 'a') + "\n", n) == "<TOO LONG>", "4097-byte line too long");
    }

    // Secret fixture directory: FROZEN_SECRET in the Python test.
    {
        const std::string root = mktmp();
        put(root + "/ble.conf", "# ble_passkey=111111\nble_security=passkey\nble_passkey = 123456 \r\n");
        put(root + "/mqtt.conf", "broker_uri=mqtt://user:pw@10.0.0.1:1883\nusername=u\npassword=hunter2 # c\n");
        put(root + "/ftp.conf", "enabled=1\nftp_pass=\n");
        ConfHashResult r;
        check(confHashComputeAt(r, true, root), "secret: ok (" + r.err + ")");
        check(confHashHex(r.sha, 32) == "4af599a6bf072297df11aea5ab7b65e578982e70c75c607db3cb408318ef0c75",
              "secret: frozen digest, got " + confHashHex(r.sha, 32));
        check(r.redacted == 5, "secret: redacted=5");
        std::string cmd = "rm -rf '" + root + "'";
        (void) !system(cmd.c_str());
    }

    // NVS encoding: NVS_VECTOR / FROZEN_NVS in the Python test.
    {
        std::vector<ConfHashNvsEntry> e = {
            {"hostname", "str", "dev1"}, {"wifi_psk", "", ""}, {"n", "u16", S("\x34\x12")},
            {"uri", "str", "mqtt://a:b@h/x"}, {"ftp_pass", "", ""}};
        Sha256 h;
        const uint32_t red = confHashNvsEntries(h, e);
        uint8_t d[32];
        h.finish(d);
        check(confHashHex(d, 32) == "20ea769cd207a86263d5bc5cbc4fbb1be9890794b82765ece790fb039b95d9e5",
              "nvs: frozen digest, got " + confHashHex(d, 32));
        check(red == 3, "nvs: redacted=3");
    }

    if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
    std::printf("conf-hash-test: all passed\n");
    return 0;
}
