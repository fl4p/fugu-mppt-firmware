#pragma once

#include <cstddef>
#include <cstdint>
#include <esp_idf_version.h>
// IDF 5.x ships mbedTLS 3 with the public mbedtls/sha256.h. IDF 6 (mbedTLS 4 / TF-PSA-Crypto) made
// that header private; the PSA hash API is the public interface there.
#if ESP_IDF_VERSION_MAJOR < 6
#include <mbedtls/sha256.h>
#else
#include <psa/crypto.h>
#endif

class Sha256 {
#if ESP_IDF_VERSION_MAJOR < 6
    mbedtls_sha256_context ctx;
    bool ok;
public:
    Sha256() {
        mbedtls_sha256_init(&ctx);
        ok = mbedtls_sha256_starts(&ctx, 0) == 0;
    }
    ~Sha256() { mbedtls_sha256_free(&ctx); }
    void update(const void *p, size_t n) {
        if (ok && n) ok = mbedtls_sha256_update(&ctx, (const unsigned char *) p, n) == 0;
    }
    bool finish(uint8_t out[32]) {
        if (ok) ok = mbedtls_sha256_finish(&ctx, out) == 0;
        return ok;
    }
#else
    psa_hash_operation_t op = PSA_HASH_OPERATION_INIT;
    bool ok;
public:
    Sha256() {
        ok = psa_crypto_init() == PSA_SUCCESS && psa_hash_setup(&op, PSA_ALG_SHA_256) == PSA_SUCCESS;
    }
    ~Sha256() { psa_hash_abort(&op); }
    void update(const void *p, size_t n) {
        if (ok && n) ok = psa_hash_update(&op, (const uint8_t *) p, n) == PSA_SUCCESS;
    }
    bool finish(uint8_t out[32]) {
        size_t len = 0;
        if (ok) ok = psa_hash_finish(&op, out, 32, &len) == PSA_SUCCESS && len == 32;
        return ok;
    }
#endif
    Sha256(const Sha256 &) = delete;
    Sha256 &operator=(const Sha256 &) = delete;
};
