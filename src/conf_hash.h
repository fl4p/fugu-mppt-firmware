#pragma once

// `conf-hash`: SHA-256 over the top-level *.conf files of a dir except wifi.conf, sorted bytewise,
// each fed as  name '\0' redacted-content '\0'.  Redaction per line (split at '\n'): a line over
// 255 bytes becomes "<redacted>". Otherwise the bytes between the first "://" and the last '@'
// after it become "<redacted>"; then, if the ASCII-lowercased result contains pass, psk, secret,
// token, _key or user, everything after the first '=' following that word does too.
// Host twin: etc/conf_hash.py.

#include <cstddef>
#include <cstdint>
#include <string>

constexpr const char *CONF_HASH_ROOT = "/littlefs/conf";

std::string confHashHex(const uint8_t *b, size_t n);

// Returns the number of hashed files, or -1 with err set. perFile gets each file's redacted-content hash.
int confHash(const char *dir, uint8_t sha[32], std::string &err,
             void (*perFile)(const char *name, const uint8_t *sha) = nullptr);

#ifdef ESP_PLATFORM
void confHashCaptureBoot();

// Hash captured at boot, nullptr if it failed.
const uint8_t *confHashBoot();
#endif
