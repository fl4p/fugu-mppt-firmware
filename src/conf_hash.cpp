#include "conf_hash.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <vector>

#ifdef ESP_PLATFORM
#include <esp_log.h>
#include "etc/sha256.h"
#else
#include "sha256_host.h"
#endif

static constexpr const char *MARKER = "<redacted>";

static void redactLine(std::string &l) {
    std::string lo(l);
    for (auto &c: lo)
        if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    size_t end = std::string::npos;
    for (const char *w: {"pass", "psk", "secret", "token", "_key"}) {
        const size_t p = lo.find(w);
        if (p != std::string::npos) end = std::min(end, p + strlen(w));
    }
    size_t a = end == std::string::npos ? end : l.find('=', end);
    if (a != std::string::npos) {
        l.replace(a + 1, std::string::npos, MARKER);
        return;
    }
    a = l.find("://");
    if (a == std::string::npos) return;
    const size_t at = l.rfind('@');
    if (at != std::string::npos && at >= a + 3) l.replace(a + 3, at - a - 3, MARKER);
}

std::string confHashRedact(const std::string &in) {
    std::string out;
    for (size_t b = 0;;) {
        const size_t e = in.find('\n', b);
        std::string l = in.substr(b, e == std::string::npos ? e : e - b);
        redactLine(l);
        out += l;
        if (e == std::string::npos) return out;
        out += '\n';
        b = e + 1;
    }
}

std::string confHashHex(const uint8_t *b, size_t n) {
    std::string s(n * 2, '0');
    for (size_t i = 0; i < n; ++i) {
        s[2 * i] = "0123456789abcdef"[b[i] >> 4];
        s[2 * i + 1] = "0123456789abcdef"[b[i] & 0xf];
    }
    return s;
}

int confHash(const char *dir, uint8_t sha[32], std::string &err,
             void (*perFile)(const char *name, const uint8_t *sha)) {
    DIR *d = opendir(dir);
    if (!d) {
        err = std::string("cannot open ") + dir;
        return -1;
    }
    std::vector<std::string> names;
    while (const dirent *e = readdir(d)) {
        const size_t n = strlen(e->d_name);
        if (n >= 5 && !strcmp(e->d_name + n - 5, ".conf") && strcmp(e->d_name, "wifi.conf"))
            names.emplace_back(e->d_name);
    }
    closedir(d);
    std::sort(names.begin(), names.end());

    Sha256 all;
    for (const auto &name: names) {
        const std::string path = std::string(dir) + "/" + name;
        FILE *f = fopen(path.c_str(), "rb");
        if (!f) {
            err = "cannot read " + path;
            return -1;
        }
        std::string s;
        char buf[256];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), f)) > 0) s.append(buf, n);
        const bool bad = ferror(f);
        fclose(f);
        if (bad) {
            err = "cannot read " + path;
            return -1;
        }
        s = confHashRedact(s);
        all.update(name.c_str(), name.size() + 1);
        all.update(s.c_str(), s.size() + 1);
        if (perFile) {
            Sha256 one;
            uint8_t fs[32];
            one.update(s.data(), s.size());
            if (!one.finish(fs)) {
                err = "sha256 failed";
                return -1;
            }
            perFile(name.c_str(), fs);
        }
    }
    if (!all.finish(sha)) {
        err = "sha256 failed";
        return -1;
    }
    return (int) names.size();
}

#ifdef ESP_PLATFORM
static uint8_t s_boot[32];
static bool s_bootOk;

void confHashCaptureBoot() {
    std::string err;
    s_bootOk = confHash(CONF_HASH_ROOT, s_boot, err) >= 0;
    if (s_bootOk)
        ESP_LOGI("confhash", "conf sha256=%s (boot)", confHashHex(s_boot, 32).c_str());
    else
        ESP_LOGW("confhash", "boot conf hash: %s", err.c_str());
}

const uint8_t *confHashBoot() { return s_bootOk ? s_boot : nullptr; }
#endif
