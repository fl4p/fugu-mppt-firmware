#pragma once

// Pure, platform-independent part of the `conf-hash` fingerprint: exclusion rules, secret and URI
// redaction, framing, NVS entry encoding, hex/decimal formatting. Templated on the hasher (any type
// with update(const void *, size_t)) so test/host-stub/conf-hash-test.cpp runs the same code on the
// host. The full definition is in conf_hash.h; etc/conf_hash.py is the host reference.

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

constexpr const char *CONF_HASH_ROOT = "/littlefs/conf";
constexpr const char *CONF_HASH_EXCLUDED = "wifi.conf";      // top-level only; it is all secrets
constexpr const char *CONF_HASH_STAGING_SUFFIX = ".new";     // USB-drive commit staging (usb_dev.cpp)
constexpr const char *CONF_HASH_REDACTED_MARKER = "<redacted>";
constexpr int CONF_HASH_MAX_DEPTH = 8;                       // max nested directory levels
constexpr size_t CONF_HASH_MAX_LINE = 4096;                  // longer line (excluding '\n') = error

// Secret-key rule, shared by the conf and NVS hashes and mirrored by etc/conf_hash.py: a key is
// secret if, ASCII-lowercased, it contains any of these substrings or ends with the suffix.
// Matches today: ble_passkey (ble.conf), password (mqtt.conf), ftp_pass (ftp.conf + NVS),
// wifi_psk (NVS).
constexpr const char *CONF_HASH_SECRET_SUBSTRINGS[] = {"pass", "psk", "passkey", "password", "secret", "token"};
constexpr const char *CONF_HASH_SECRET_SUFFIX = "_key";

inline bool confHashIsSecretKey(const std::string &key) {
    std::string k(key);
    for (auto &c: k)
        if (c >= 'A' && c <= 'Z') c = (char) (c - 'A' + 'a'); // ASCII only, locale-independent
    for (const char *s: CONF_HASH_SECRET_SUBSTRINGS)
        if (k.find(s) != std::string::npos) return true;
    const size_t sl = strlen(CONF_HASH_SECRET_SUFFIX);
    return k.size() >= sl && k.compare(k.size() - sl, sl, CONF_HASH_SECRET_SUFFIX) == 0;
}

// Whitespace for redaction purposes: the bytes conf.h trim() strips.
inline bool confHashIsSpace(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r' || c == '\0';
}

// URI userinfo: scanning left to right, for every "://" the authority runs to the next '/', '?',
// '#', '\n' or the end; if it contains '@', everything from after "://" up to the LAST '@' in it is
// replaced by the marker, and scanning resumes right after the marker (else right after "://").
// Returns the number of replacements.
inline uint32_t confHashRedactUris(std::string &s) {
    uint32_t n = 0;
    const size_t ml = strlen(CONF_HASH_REDACTED_MARKER);
    size_t pos = 0, p;
    while ((p = s.find("://", pos)) != std::string::npos) {
        const size_t a = p + 3;
        size_t e = s.find_first_of("/?#\n", a);
        if (e == std::string::npos) e = s.size();
        size_t at = std::string::npos;
        for (size_t i = e; i > a; --i)
            if (s[i - 1] == '@') { at = i - 1; break; }
        pos = a; // keep scanning inside this authority too: "a://x@h b://y@g" has two userinfos
        if (at != std::string::npos) {
            s.replace(a, at - a, CONF_HASH_REDACTED_MARKER);
            pos = a + ml;
            ++n;
        }
    }
    return n;
}

// Redact one line, given WITHOUT its '\n'. The candidate is the line after a leading run of
// whitespace and '#' (so commented-out lines are treated like live ones). If the candidate contains
// '=' and trim(text before the first '=') is a secret key, everything after that '=' to the end of
// the line (a trailing '\r' included) becomes the marker. Otherwise URI userinfo is redacted
// anywhere in the line. Returns the number of replacements.
inline uint32_t confHashRedactLine(std::string &line) {
    size_t c = 0;
    while (c < line.size() && (confHashIsSpace(line[c]) || line[c] == '#')) ++c;
    const size_t eq = line.find('=', c);
    if (eq != std::string::npos) {
        size_t ka = c, kb = eq;
        while (ka < kb && confHashIsSpace(line[ka])) ++ka;
        while (kb > ka && confHashIsSpace(line[kb - 1])) --kb;
        if (confHashIsSecretKey(line.substr(ka, kb - ka))) {
            line.replace(eq + 1, std::string::npos, CONF_HASH_REDACTED_MARKER);
            return 1;
        }
    }
    return confHashRedactUris(line);
}

// Streaming line splitter + redactor: feed raw bytes, receive redacted bytes via emit(std::string).
// Lines end at '\n' only (kept). A line longer than CONF_HASH_MAX_LINE sets tooLong.
class ConfHashRedactor {
    std::string line;
public:
    uint32_t redacted = 0;
    bool tooLong = false;

    template<class Emit>
    void feed(const void *data, size_t n, Emit &&emit) {
        const char *p = (const char *) data;
        for (size_t i = 0; i < n && !tooLong; ++i) {
            if (p[i] == '\n') {
                redacted += confHashRedactLine(line);
                line.push_back('\n');
                emit(line);
                line.clear();
            } else {
                line.push_back(p[i]);
                if (line.size() > CONF_HASH_MAX_LINE) tooLong = true;
            }
        }
    }

    template<class Emit>
    void finish(Emit &&emit) {
        if (!tooLong && !line.empty()) {
            redacted += confHashRedactLine(line);
            emit(line);
        }
        line.clear();
    }
};

inline std::string confHashDec(uint64_t v) {
    char b[24];
    snprintf(b, sizeof(b), "%" PRIu64, v);
    return b;
}

inline std::string confHashHex(const uint8_t *b, size_t n) {
    static const char *hx = "0123456789abcdef";
    std::string s(n * 2, '0');
    for (size_t i = 0; i < n; ++i) {
        s[2 * i] = hx[b[i] >> 4];
        s[2 * i + 1] = hx[b[i] & 0xf];
    }
    return s;
}

// std::string's operator< compares via char_traits<char>, i.e. as unsigned char: bytewise.
inline void confHashSortPaths(std::vector<std::string> &paths) { std::sort(paths.begin(), paths.end()); }

inline bool confHashIsStaging(const std::string &rel) {
    const size_t sl = strlen(CONF_HASH_STAGING_SUFFIX);
    return rel.size() >= sl && rel.compare(rel.size() - sl, sl, CONF_HASH_STAGING_SUFFIX) == 0;
}

// Per-file framing header, fed before the redacted content:  relpath '\0' decimal(len) '\0'
template<class H>
void confHashFrame(H &h, const std::string &rel, uint64_t redactedLen) {
    h.update(rel.data(), rel.size());
    h.update("", 1);
    const std::string d = confHashDec(redactedLen);
    h.update(d.data(), d.size());
    h.update("", 1);
}

struct ConfHashNvsEntry {
    std::string key;
    std::string type;  // u8 i8 u16 i16 u32 i32 u64 i64 str blob
    std::string value; // little-endian ints, string without NUL, raw blob; unused for secret keys
};

// Hash NVS entries: sorted by key; a secret key contributes  key '\0' "redacted";  any other entry
// key '\0' type '\0' decimal(len) '\0' value,  with URI userinfo redacted in str values (len is
// the redacted length). Returns the number of redactions (secret keys + URI userinfos).
template<class H>
uint32_t confHashNvsEntries(H &h, std::vector<ConfHashNvsEntry> entries) {
    std::sort(entries.begin(), entries.end(),
              [](const ConfHashNvsEntry &a, const ConfHashNvsEntry &b) { return a.key < b.key; });
    uint32_t red = 0;
    for (auto &e: entries) {
        h.update(e.key.data(), e.key.size());
        h.update("", 1);
        if (confHashIsSecretKey(e.key)) {
            h.update("redacted", 8);
            ++red;
            continue;
        }
        if (e.type == "str") red += confHashRedactUris(e.value);
        h.update(e.type.data(), e.type.size());
        h.update("", 1);
        const std::string d = confHashDec(e.value.size());
        h.update(d.data(), d.size());
        h.update("", 1);
        h.update(e.value.data(), e.value.size());
    }
    return red;
}
