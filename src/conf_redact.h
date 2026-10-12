#pragma once

// Secret redaction for the conf-hash fingerprint (definition: conf_hash.h). Pure C++ so the host
// test test/host-stub/conf-redact-test.cpp can pin it against etc/conf_hash.py.

#include <algorithm>
#include <cctype>
#include <string>

constexpr const char *CONF_HASH_REDACTED_MARKER = "<redacted>";
// Secret-key rule, shared by the conf and NVS hashes and mirrored by etc/conf_hash.py: a key is
// secret if, lowercased, it contains any of these substrings or ends with CONF_HASH_SECRET_SUFFIX.
// Matches today: ble_passkey (ble.conf), password (mqtt.conf), ftp_pass (ftp.conf + NVS),
// wifi_psk (NVS).
constexpr const char *CONF_HASH_SECRET_SUBSTRINGS[] = {"pass", "psk", "passkey", "password", "secret", "token"};
constexpr const char *CONF_HASH_SECRET_SUFFIX = "_key";


inline bool confHashIsSecretKey(const std::string &key) {
    std::string k(key);
    for (auto &c: k) c = (char) tolower((unsigned char) c);
    for (const char *s: CONF_HASH_SECRET_SUBSTRINGS)
        if (k.find(s) != std::string::npos) return true;
    const std::string suf = CONF_HASH_SECRET_SUFFIX;
    return k.size() >= suf.size() && k.compare(k.size() - suf.size(), suf.size(), suf) == 0;
}

// ConfFile reads lines with fgets(buf, 256): a line longer than 255 bytes is parsed as several.
constexpr size_t CONF_LINE_CHUNK = 255;

// Mirrors trim() in conf.h exactly.
inline bool confIsTrimChar(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r' || c == '\0';
}

// Redact one line (one fgets() chunk: up to and including '\n', at most CONF_LINE_CHUNK bytes) the
// way ConfFile parses it: the part before the first '#' or NUL (std::string(buf) stops at a NUL) is
// the live text; if it contains '=', the key is the trimmed text before the
// first '=', the value the trimmed text after it. For a secret key, exactly the value bytes are
// replaced by CONF_HASH_REDACTED_MARKER (inserted right after the '=' if the value is empty);
// everything else, including an inline comment, stays verbatim.
inline bool confRedactLine(const std::string &line, std::string &out) {
    size_t live = std::min(line.find('#'), line.find('\0'));
    if (live == std::string::npos) live = line.size();
    size_t eq = line.find('=');
    if (eq == std::string::npos || eq >= live) { out = line; return false; }
    size_t ka = 0, kb = eq;
    while (ka < kb && confIsTrimChar(line[ka])) ka++;
    while (kb > ka && confIsTrimChar(line[kb - 1])) kb--;
    if (!confHashIsSecretKey(line.substr(ka, kb - ka))) { out = line; return false; }
    size_t va = eq + 1, vb = live;
    while (va < vb && confIsTrimChar(line[va])) va++;
    while (vb > va && confIsTrimChar(line[vb - 1])) vb--;
    if (va == vb) va = vb = eq + 1; // empty value: the marker goes right after the '='
    out.assign(line, 0, va);
    out += CONF_HASH_REDACTED_MARKER;
    out.append(line, vb, std::string::npos);
    return true;
}

