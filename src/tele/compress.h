#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>

// Pluggable byte-stream compressor for the binary line protocol. Each packet()
// call is self-contained (fresh dictionary) so its output decompresses on its
// own — for a lossy transport (UDP) batch many length-prefixed wire frames into
// one buffer and call packet() once; losing a datagram never poisons the next.
// id() is a 1-byte tag the transport prepends so the receiver picks the matching
// decompressor. Swap algorithms by handing the transport a different instance
// (see compressorByName), without touching the encoder or the send path.
class Compressor {
public:
    virtual ~Compressor() = default;
    virtual uint8_t id() const = 0;          // wire tag (0=none, 1=tamp, ...)
    virtual const char *name() const = 0;
    // appends the packet to `out`; false leaves `out` as it was
    virtual bool packet(const uint8_t *in, size_t n, std::string &out) = 0;
    // Raw input size that keeps one compressed packet within `mtu` (minus the id
    // byte). The transport batches up to this many raw bytes per datagram.
    virtual size_t maxBatchRaw(size_t mtu) const = 0;
};

// Identity passthrough: the wire stays raw binary. Right choice on lossy links
// where per-packet compression doesn't pay — the symbol-table protocol already
// strips the cross-frame redundancy an LZ window would otherwise find.
class NoCompress : public Compressor {
public:
    uint8_t id() const override { return 0; }
    const char *name() const override { return "none"; }
    bool packet(const uint8_t *in, size_t n, std::string &out) override {
        out.append(reinterpret_cast<const char *>(in), n);
        return true;
    }
    size_t maxBatchRaw(size_t mtu) const override { return mtu > 1 ? mtu - 1 : 0; }  // raw == datagram
};

// Tamp (BrianPugh/tamp): DEFLATE-inspired, ~1 KB window, tiny code/RAM. Best on
// batched packets (warm window) or a reliable stream; near break-even on single
// tiny frames. Defined in tamp_compress.cpp so the tamp C headers stay isolated.
class TampCompress : public Compressor {
    uint8_t windowBits_;                     // dictionary of 1<<windowBits_ B, allocated per packet
public:
    explicit TampCompress(uint8_t windowBits = 10);
    uint8_t id() const override { return 1; }
    const char *name() const override { return "tamp"; }
    bool packet(const uint8_t *in, size_t n, std::string &out) override;
    size_t maxBatchRaw(size_t mtu) const override { return mtu * 3 / 2; }  // ~1.5x; telemetry tamps ~2x, margin for variance
};

// Shared instance by name ("none"/"tamp"); unknown -> NoCompress. The binary
// telemetry wire always selects "tamp"; NoCompress (id=0) stays for the receiver
// and any other transport.
Compressor &compressorByName(const char *name);
