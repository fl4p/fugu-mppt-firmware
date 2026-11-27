#pragma once
#include "compress.h"
#include "pmini_packet.h"
#include <cstdint>
#include <string>
#include <vector>

// Per-transport, single-owner state. Never reads the producer's SymbolTable.
class TelePacketEncoder {
    PminiPacket pmini_;
    TampCompress tamp_;
    bool split(const std::string &raw, size_t limit, std::vector<std::string> &out, PminiPacket &candidate);
public:
    // Codec IDs: 0 raw, 1 tamp, 2 PMA2. Chooses the smallest supported form;
    // splits only between complete wire frames to enforce the payload limit.
    // Failure leaves out unchanged. Reset by replacing this object on restart.
    bool pack(const std::string &raw, size_t limit, std::vector<std::string> &out);
};

class TelePacketBatch {
    uint64_t firstMs_ = 0;
public:
    static constexpr size_t CAP = 8192;
    static constexpr uint64_t WAIT_MS = 1800;
    std::string raw;
    bool fits(size_t n) const { return n <= CAP && raw.size() <= CAP - n; }
    bool append(const std::string &point, uint64_t now) {
        if (!fits(point.size())) return false;
        const bool first = raw.empty();
        raw += point;
        if (first) firstMs_ = now;
        return true;
    }
    bool due(uint64_t now) const { return !raw.empty() && ((now >= firstMs_ && now - firstMs_ >= WAIT_MS) || raw.size() == CAP); }
    void clear() { raw.clear(); firstMs_ = 0; }
};
