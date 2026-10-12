#pragma once

// Configuration fingerprint for the `conf-hash` console command.
//
// The conf hash is computed over the file bytes under /littlefs/conf (not over parsed keys), so a
// host can recompute it from a backup directory: etc/conf_hash.py is the reference implementation;
// test/host_py/test_conf_hash.py and test/host-stub/conf-redact-test.cpp pin it. Definition:
//
//   files  = every regular file below the conf root, recursively, as a '/'-separated path relative
//            to the root, EXCEPT the top-level "wifi.conf" (it is all secrets: excluded entirely)
//   order  = relative path, bytewise ascending
//   content= the file's bytes with every SECRET value redacted (see below)
//   sha256 = SHA-256 over, for each file in order:  relpath '\0' decimal(len) '\0' content
//            where len is the length of the REDACTED content, not the file size
//
// Redaction (published hashes of short secrets such as a 6-digit BLE passkey are brute-forceable
// offline): the file is split into lines exactly as ConfFile's fgets(buf, 256) reads it: after each
// '\n' (kept) or after 255 bytes. A line is parsed like ConfFile does: the text before the first
// '#' or NUL is live; if it contains '=', key = trim(text before the first
// '='), value = trim(text after it), trim() stripping " \t\n\v\f\r\0" as in conf.h. If the key is
// a secret key (confHashIsSecretKey, conf_redact.h), exactly the value bytes are replaced by
// CONF_HASH_REDACTED_MARKER (an empty value gets the marker right after the '='); all other bytes
// (whitespace, '\r', inline comment) stay. Comment lines such as "# ble_passkey=123456" are not
// live and are hashed verbatim.
//
// The per-file hash is SHA-256 of that same redacted content, so it equals `sha256sum` of the file
// only when the file has no redacted line (redacted=0).
//
// The NVS hash covers only the app namespace "storage" (key-value.h). Entries are sorted by key and
// each contributes:  key '\0' typename '\0' decimal(len) '\0' value-bytes
// with typename one of u8 i8 u16 i16 u32 i32 u64 i64 str blob; integers are their little-endian
// bytes at native width, strings exclude the NUL terminator, blobs are raw. An entry with a secret
// key is never read and contributes only  key '\0' "redacted"  (no length, no value).

#include "conf_redact.h"

#include <cstdint>
#include <string>
#include <vector>

constexpr const char *CONF_HASH_ROOT = "/littlefs/conf";
constexpr const char *CONF_HASH_EXCLUDED = "wifi.conf";

struct ConfHashFile {
    std::string relpath;
    uint32_t bytes = 0;         // redacted length
    uint32_t redacted = 0;      // secret values replaced in this file
    uint8_t sha[32]{};
};

struct ConfHashResult {
    bool ok = false;
    std::string err;            // set iff !ok; no hash is valid then
    uint8_t sha[32]{};
    uint32_t files = 0;
    uint64_t bytes = 0;         // sum of redacted lengths
    uint32_t redacted = 0;
    std::vector<ConfHashFile> perFile; // only filled when requested
};

struct NvsHashResult {
    bool ok = false;            // false on any error or any unreadable entry; sha is then invalid
    std::string err;
    uint8_t sha[32]{};
    uint32_t keys = 0;          // entries found in the namespace (readable + unreadable)
    uint32_t unreadable = 0;
    uint32_t redacted = 0;      // secret-key entries, hashed by key only
    std::vector<std::string> unreadableKeys; // "key:type:err" for the report
};

// Hashes CONF_HASH_ROOT. Strictly read-only. Returns out.ok; on failure out.err says why and no
// hash or per-file entry is valid (a partial read is never reported as a hash).
bool confHashCompute(ConfHashResult &out, bool perFile);

// Opens the namespace NVS_READONLY; never writes. Returns out.ok.
bool nvsHashCompute(NvsHashResult &out, const char *ns = "storage");

// Called once from setup() right after the littlefs mount, before any conf file is read.
void confHashCaptureBoot();

// Boot-time result; ok=false (with err) if it was not computed or failed.
const ConfHashResult &confHashBoot();

std::string confHashHex(const uint8_t *b, size_t n);
