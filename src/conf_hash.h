#pragma once

// `conf-hash`: SHA-256 over the top-level *.conf files of a dir except wifi.conf, sorted bytewise,
// each fed as  name '\0' redacted-content '\0'.  Redaction per line (split at '\n'): if the
// ASCII-lowercased line contains pass, psk, secret, token or _key, everything after the first '='
// following that word becomes "<redacted>"; otherwise the bytes between the first "://" and the
// last '@' after it do. Host twin: etc/conf_hash.py.

#include <cstddef>
#include <cstdint>
#include <string>

constexpr const char *CONF_HASH_ROOT = "/littlefs/conf";

std::string confHashRedact(const std::string &in);

std::string confHashHex(const uint8_t *b, size_t n);

// Returns the number of hashed files, or -1 with err set. perFile gets each file's redacted-content hash.
int confHash(const char *dir, uint8_t sha[32], std::string &err,
             void (*perFile)(const char *name, const uint8_t *sha) = nullptr);

#ifdef ESP_PLATFORM
void confHashCaptureBoot();

// Hash captured at boot, nullptr if it failed.
const uint8_t *confHashBoot();
#endif
