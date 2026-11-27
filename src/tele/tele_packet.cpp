#include "tele_packet.h"
#include <new>

namespace {
// Strict canonical length prefix; no out-of-range pointer arithmetic.
bool frameEnds(const std::string &raw, std::vector<size_t> &ends) {
    size_t at = 0;
    while (at < raw.size()) {
        uint64_t n = 0; unsigned shift = 0, count = 0; uint8_t byte;
        do {
            if (at == raw.size() || count == 10) return false;
            byte = uint8_t(raw[at++]);
            if (shift == 63 && (byte & 0x7e)) return false;
            n |= uint64_t(byte & 127) << shift;
            shift += 7; ++count;
        } while (byte & 128);
        if ((count > 1 && byte == 0) || !n || n > raw.size() - at) return false;
        at += size_t(n); ends.push_back(at);
        if (ends.size() > 512) return false;
    }
    return !ends.empty();
}
}

bool TelePacketEncoder::split(const std::string &raw, size_t limit, std::vector<std::string> &out, PminiPacket &candidate) {
    std::vector<size_t> ends;
    if (!frameEnds(raw, ends)) return false;
    std::string best(1, '\0'); best += raw;
    std::string compressed(1, '\1');
    if (tamp_.packet(reinterpret_cast<const uint8_t *>(raw.data()), raw.size(), compressed) && compressed.size() < best.size())
        best.swap(compressed);
    compressed.assign(1, '\2');
    if (candidate.packet(reinterpret_cast<const uint8_t *>(raw.data()), raw.size(), compressed) && compressed.size() < best.size())
        best.swap(compressed);
    if (best.size() <= limit) { out.push_back(std::move(best)); return true; }
    if (ends.size() < 2) return false;
    const size_t cut = ends[ends.size() / 2 - 1];
    return split(raw.substr(0, cut), limit, out, candidate) && split(raw.substr(cut), limit, out, candidate);
}

bool TelePacketEncoder::pack(const std::string &raw, size_t limit, std::vector<std::string> &out) {
    if (raw.empty() || raw.size() > TelePacketBatch::CAP || limit < 2) return false;
    try {
        std::vector<std::string> packets;
        PminiPacket candidate = pmini_;
        if (!split(raw, limit, packets, candidate)) return false;
        out.insert(out.end(), std::make_move_iterator(packets.begin()), std::make_move_iterator(packets.end()));
        pmini_ = std::move(candidate);
        return true;
    } catch (const std::bad_alloc &) { return false; }
}
