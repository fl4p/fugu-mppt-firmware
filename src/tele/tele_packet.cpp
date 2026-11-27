#include "tele_packet.h"
#include <new>

namespace {
// Strict canonical length prefix; no out-of-range pointer arithmetic.
bool frameEnds(std::string_view raw, std::vector<size_t> &ends) {
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

bool TelePacketEncoder::split(std::string_view raw, size_t limit, std::vector<std::string> &out, PminiPacket &candidate) {
    std::vector<size_t> ends;
    if (!frameEnds(raw, ends)) return false;
    // Try pmini before allocating tamp's worst-case output buffer. Never keep
    // a full raw candidate alive alongside the adapter's scratch space.
    PminiPacket trial = candidate;
    std::string best(1, '\2');
    const bool encoded = trial.packet(reinterpret_cast<const uint8_t *>(raw.data()), raw.size(), best);
    if (!encoded) best.clear();
    {
        std::string compressed(1, '\1');
        if (tamp_.packet(reinterpret_cast<const uint8_t *>(raw.data()), raw.size(), compressed) &&
            (best.empty() || compressed.size() < best.size())) best.swap(compressed);
    }
    if (best.empty() || raw.size() + 1 < best.size()) {
        best.assign(1, '\0'); best.append(raw.data(), raw.size());
    }
    if (best.size() <= limit) {
        // A fallback may contain the heartbeat table needed by the NEXT batch,
        // even when earlier data has unknown SIDs. Do not apply it retroactively.
        if (!encoded && !trial.observeTables(reinterpret_cast<const uint8_t *>(raw.data()), raw.size())) return false;
        out.push_back(std::move(best));
        candidate = std::move(trial);
        return true;
    }
    // Recursive children only see state from packets actually retained.
    std::string().swap(best);
    if (ends.size() < 2) return false;
    const size_t cut = ends[ends.size() / 2 - 1];
    return split(raw.substr(0, cut), limit, out, candidate) && split(raw.substr(cut), limit, out, candidate);
}

bool TelePacketEncoder::pack(std::string_view raw, size_t limit, std::vector<std::string> &out) {
    if (raw.empty() || raw.size() > TelePacketBatch::MAX_RAW_INPUT || limit < 2) return false;
    try {
        std::vector<std::string> packets;
        PminiPacket candidate = pmini_;
        if (!split(raw, limit, packets, candidate)) return false;
        out.insert(out.end(), std::make_move_iterator(packets.begin()), std::make_move_iterator(packets.end()));
        pmini_ = std::move(candidate);
        return true;
    } catch (const std::bad_alloc &) { return false; }
}
