#pragma once

// Configuration fingerprint for the `conf-hash` console command.
//
// The conf hash is byte-exact over the files under /littlefs/conf (not over parsed keys), so a host
// can recompute it from a backup directory: etc/conf_hash.py is the reference implementation and
// test/host_py/test_conf_hash.py pins the definition. Definition:
//
//   files  = every regular file below the conf root, recursively, as a '/'-separated path relative
//            to the root, EXCEPT the top-level "wifi.conf" (it holds the Wi-Fi PSK, and a published
//            hash of a short PSK can be brute-forced offline)
//   order  = relative path, bytewise ascending
//   sha256 = SHA-256 over, for each file in order:  relpath '\0' decimal(len) '\0' raw-bytes
//
// The per-file hash is plain SHA-256 of the file's raw bytes (what `sha256sum` prints).
//
// The NVS hash covers only the app namespace "storage" (key-value.h). Entries are sorted by key and
// each contributes:  key '\0' typename '\0' decimal(len) '\0' value-bytes
// with typename one of u8 i8 u16 i16 u32 i32 u64 i64 str blob; integers are their little-endian
// bytes at native width, strings exclude the NUL terminator, blobs are raw.

#include <cstdint>
#include <string>
#include <vector>

constexpr const char *CONF_HASH_ROOT = "/littlefs/conf";
constexpr const char *CONF_HASH_EXCLUDED = "wifi.conf";

struct ConfHashFile {
    std::string relpath;
    uint32_t bytes = 0;
    uint8_t sha[32]{};
};

struct ConfHashResult {
    bool ok = false;
    std::string err;            // set iff !ok; no hash is valid then
    uint8_t sha[32]{};
    uint32_t files = 0;
    uint64_t bytes = 0;
    std::vector<ConfHashFile> perFile; // only filled when requested
};

struct NvsHashResult {
    bool ok = false;            // false on any error or any unreadable entry; sha is then invalid
    std::string err;
    uint8_t sha[32]{};
    uint32_t keys = 0;          // entries found in the namespace (readable + unreadable)
    uint32_t unreadable = 0;
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
