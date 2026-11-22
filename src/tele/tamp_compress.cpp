#include "compress.h"
#include <cstring>
#include <memory>
#include <new>
#include "compressor.h"   // tamp C lib (component tamp); flat header, isolated to this TU

TampCompress::TampCompress(uint8_t windowBits) : windowBits_(windowBits) {}

bool TampCompress::packet(const uint8_t *in, size_t n, std::string &out) {
    TampConf conf = {};                 // value-init all bitfields (avoids missing-field-initializers)
    conf.window = windowBits_;
    conf.literal = 8;
    // per call: held, the window costs 1 KB of internal heap for good
    std::unique_ptr<unsigned char[]> window(new (std::nothrow) unsigned char[size_t(1) << windowBits_]);
    if (!window) return false;
    ::TampCompressor c;                 // tamp's C struct (re-init per packet -> independent output)
    if (tamp_compressor_init(&c, &conf, window.get()) != TAMP_OK) return false;

    const size_t at = out.size();
    out.resize(at + n + n / 8 + 16);    // worst case: ~1 flag bit/literal + header + flush
    size_t written = 0, consumed = 0;
    tamp_res r = tamp_compressor_compress_and_flush(
        &c, reinterpret_cast<unsigned char *>(&out[at]), out.size() - at, &written,
        in, n, &consumed, /*write_token=*/false);
    if (r != TAMP_OK || consumed != n) {
        out.resize(at);
        return false;
    }
    out.resize(at + written);
    return true;
}

Compressor &compressorByName(const char *name) {
    static NoCompress none;
    static TampCompress tamp;
    if (name && std::strcmp(name, "tamp") == 0) return tamp;
    return none;
}
