#pragma once

// Configuration fingerprint for the `conf-hash` console command. Strictly read-only.
//
// Computed over the file bytes under /littlefs/conf (not over parsed keys), with secrets redacted,
// so a host can recompute it from a backup directory. etc/conf_hash.py is the host reference;
// test/host-stub/conf-hash-test.cpp (this code) and test/host_py/test_conf_hash.py pin the same
// frozen digests. Definition:
//
//   files   every regular file below the conf root, recursively (more than 8 nested directory
//           levels is an error), as a '/'-separated path relative to the root, except the top-level "wifi.conf"
//           (all secrets). A file whose name ends in ".new" (USB-drive commit staging) is an error:
//           a hash taken mid-commit is not a stable state. Anything else (e.g. a symlink on a host
//           copy; littlefs has none) is an error.
//   order   relative path, bytewise ascending
//   content each file split into lines at '\n' only (the '\n' is kept); each line redacted by
//           confHashRedactLine (conf_hash_core.h):
//             - candidate = the line after its leading run of whitespace (" \t\n\v\f\r\0") and '#',
//               so a commented-out line is treated like a live one;
//             - if the candidate contains '=' and trim(text before the first '=') is a secret key
//               (confHashIsSecretKey), everything after that '=' up to the '\n' (a trailing '\r'
//               included) is replaced by "<redacted>";
//             - otherwise URI userinfo is redacted anywhere in the line: for each "://", the
//               authority runs to the next '/', '?', '#' or end of line; if it contains '@', the
//               text from after "://" to the last '@' in it is replaced by "<redacted>".
//           A line longer than 4096 bytes (excluding '\n') is an error.
//   sha256  SHA-256 over, per file in order:  relpath '\0' decimal(len) '\0' content
//           where len is the length of the REDACTED content.
//
// The per-file hash is SHA-256 of the same redacted content: it equals `sha256sum` of the file only
// when the file had nothing redacted (redacted=0).
//
// The NVS hash covers only the app namespace "storage" (key-value.h): entries sorted by key, each
// contributing  key '\0' typename '\0' decimal(len) '\0' value  (typename u8 i8 u16 i16 u32 i32 u64
// i64 str blob; integers little-endian at native width, strings without the NUL, blobs raw; URI
// userinfo redacted in str values, len being the redacted length). An entry with a secret key is
// never read and contributes only  key '\0' "redacted".

#include "conf_hash_core.h"

#include <cstdint>
#include <string>
#include <vector>

struct ConfHashFile {
    std::string relpath;
    uint32_t bytes = 0;         // redacted length
    uint32_t redacted = 0;      // redactions in this file
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
    uint32_t redacted = 0;      // secret-key entries + URI userinfos
    std::vector<std::string> unreadableKeys; // "key:type:err" for the report
};

// Hash the directory tree at root (no mount check). Returns out.ok; on failure out.err says why and
// no hash or per-file entry is valid (a partial read is never reported as a hash).
bool confHashComputeAt(ConfHashResult &out, bool perFile, const std::string &root);

#ifdef ESP_PLATFORM
// Hashes CONF_HASH_ROOT after checking that littlefs is mounted.
bool confHashCompute(ConfHashResult &out, bool perFile);

// Opens the namespace NVS_READONLY; never writes. Returns out.ok.
bool nvsHashCompute(NvsHashResult &out, const char *ns = "storage");

// Called once from setup() right after the littlefs mount, before any conf file is read.
void confHashCaptureBoot();

// Boot-time result; ok=false (with err) if it was not computed or failed.
const ConfHashResult &confHashBoot();
#endif
