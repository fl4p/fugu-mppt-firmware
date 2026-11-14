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

static constexpr char MARKER[] = "<redacted>";
static constexpr size_t ML = sizeof(MARKER) - 1;
static constexpr size_t LINE_MAX_LEN = 255; // longer lines are redacted whole

static bool hasWordBefore(const char *s, size_t n, size_t &end) {
    end = n;
    for (const char *w: {"pass", "psk", "secret", "token", "_key", "user"}) {
        const size_t wl = strlen(w);
        for (size_t i = 0; i + wl <= n && i + wl < end; ++i) {
            size_t k = 0;
            while (k < wl && (s[i + k] >= 'A' && s[i + k] <= 'Z' ? s[i + k] + 32 : s[i + k]) == w[k]) ++k;
            if (k == wl) { end = i + wl; break; }
        }
    }
    return end < n;
}

// Redacts one line (without '\n') into out (>= LINE_MAX_LEN + 2 * ML bytes); returns its length.
static size_t redactLine(const char *l, size_t n, char *out) {
    if (n > LINE_MAX_LEN) {
        memcpy(out, MARKER, ML);
        return ML;
    }
    size_t m = n, at = n;
    static constexpr char SEP[] = "://";
    const size_t a = std::search(l, l + n, SEP, SEP + 3) - l;
    for (size_t i = n; a < n && i > a + 3; --i)
        if (l[i - 1] == '@') { at = i - 1; break; }
    if (at < n) {
        memcpy(out, l, a + 3);
        memcpy(out + a + 3, MARKER, ML);
        memcpy(out + a + 3 + ML, l + at, n - at);
        m = a + 3 + ML + n - at;
    } else {
        memcpy(out, l, n);
    }
    size_t end;
    if (hasWordBefore(out, m, end))
        for (size_t i = end; i < m; ++i)
            if (out[i] == '=') {
                memcpy(out + i + 1, MARKER, ML);
                return i + 1 + ML;
            }
    return m;
}

// Streams the redacted content of f, line by line, to emit(const char *, size_t). False on read error.
template<class Emit>
static bool redactStream(FILE *f, Emit &&emit) {
    char l[LINE_MAX_LEN + 1], out[LINE_MAX_LEN + 2 * ML];
    size_t n = 0;
    for (int c;;) {
        c = getc(f);
        if (c == EOF || c == '\n') {
            if (n) emit(out, redactLine(l, n, out));
            if (c == EOF) return !ferror(f);
            emit("\n", 1);
            n = 0;
        } else if (n <= LINE_MAX_LEN) {
            l[n++] = (char) c;
        }
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
        all.update(name.c_str(), name.size() + 1);
        Sha256 one;
        const bool ok = redactStream(f, [&](const char *p, size_t k) {
            all.update(p, k);
            one.update(p, k);
        });
        fclose(f);
        uint8_t fs[32];
        if (!ok || !one.finish(fs)) {
            err = "cannot read " + path;
            return -1;
        }
        all.update("", 1);
        if (perFile) perFile(name.c_str(), fs);
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
