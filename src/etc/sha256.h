#pragma once

#include <cstddef>
#include <cstdint>
// PSA hash API: IDF 6 (mbedTLS 4) made the mbedTLS sha256 header private
#include <psa/crypto.h>

class Sha256 {
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
    Sha256(const Sha256 &) = delete;
    Sha256 &operator=(const Sha256 &) = delete;
};
