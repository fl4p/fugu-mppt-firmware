#include "pmini_packet.h"

#include "pmini_codec.h"
#include "sym_line_protocol.h"

#include <algorithm>
#include <cstring>
#include <new>
#include <vector>

namespace {
constexpr std::size_t kMaxInput = 8192;
constexpr std::size_t kMaxRows = 128;
constexpr std::size_t kMaxFields = 32;
constexpr std::size_t kMaxSids = 4096;
constexpr std::size_t kMaxEvents = 512;
constexpr std::size_t kMaxPrefix = 4096;
constexpr std::uint8_t kEnvelopeVersion = 1;
constexpr std::uint8_t kTableEvent = 0;
constexpr std::uint8_t kDataEvent = 1;
constexpr char kMagic[] = "PMA2";

struct Event { bool table; std::uint16_t at; std::uint16_t len; };
struct FieldRef { std::uint16_t sid; std::uint8_t dt; std::uint16_t valueAt; };
struct SchemaField { std::uint16_t sid; std::uint8_t dt; };
struct Row {
    std::uint64_t timestamp = 0;
    std::uint32_t presence = 0;
    std::uint16_t fieldAt = 0;
    std::uint8_t fieldCount = 0;
};

bool readVarint(const std::uint8_t *p, const std::uint8_t *end,
                std::uint64_t &value, std::size_t &used) {
    value = 0; used = 0;
    for (unsigned shift = 0; shift < 64 && p + used < end && used < 10; ++used) {
        const std::uint8_t b = p[used];
        if (used == 9 && b > 1) return false;
        value |= std::uint64_t(b & 0x7f) << shift;
        if ((b & 0x80) == 0) {
            ++used;
            if (used > 1 && value < (std::uint64_t(1) << (7 * (used - 1)))) return false;
            return true;
        }
        shift += 7;
    }
    return false;
}

void putPVarint(std::string &out, std::uint64_t value) {
    while (value >= 0x80) { out.push_back(char((value & 0x7f) | 0x80)); value >>= 7; }
    out.push_back(char(value));
}

bool knownString(const std::vector<std::uint8_t> &types, std::uint64_t sid) {
    return sid != 0 && sid < types.size() && types[std::size_t(sid)] == std::uint8_t(WireDT::Str);
}

bool knownNumeric(const std::vector<std::uint8_t> &types, std::uint64_t sid, std::uint8_t &dt) {
    if (sid == 0 || sid >= types.size()) return false;
    dt = types[std::size_t(sid)];
    return dt != std::uint8_t(WireDT::Str) && dt <= std::uint8_t(WireDT::F64);
}

bool applyTable(const std::uint8_t *frame, std::size_t n, std::vector<std::uint8_t> &types) {
    if (n < 2 || frame[0] != std::uint8_t(FrameT::Table)) return false;
    const std::uint8_t *p = frame + 1, *end = frame + n;
    std::uint64_t maxSid = 0;
    // Validate the complete frame before changing the private map.
    for (;;) {
        std::uint64_t sid = 0; std::size_t su = 0;
        if (!readVarint(p, end, sid, su)) return false;
        p += su;
        if (sid == 0) break;
        if (sid > kMaxSids) return false;
        const auto *z = static_cast<const std::uint8_t *>(std::memchr(p, 0, std::size_t(end - p)));
        if (z == nullptr || z + 1 >= end || z[1] > std::uint8_t(WireDT::F64)) return false;
        maxSid = std::max(maxSid, sid); p = z + 2;
    }
    if (p != end) return false;
    if (maxSid >= types.size()) types.resize(std::size_t(maxSid) + 1, 0xff);
    p = frame + 1;
    for (;;) {
        std::uint64_t sid = 0; std::size_t su = 0;
        if (!readVarint(p, end, sid, su)) return false;
        p += su;
        if (sid == 0) return true;
        const auto *z = static_cast<const std::uint8_t *>(std::memchr(p, 0, std::size_t(end - p)));
        types[std::size_t(sid)] = z[1]; p = z + 2;
    }
}

std::uint32_t crc32(const std::uint8_t *p, std::size_t n) {
    std::uint32_t crc = 0xffffffffu;
    while (n--) {
        crc ^= *p++;
        for (unsigned bit = 0; bit != 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return crc ^ 0xffffffffu;
}

void putU32LE(std::string &out, std::uint32_t v) {
    out.push_back(char(v)); out.push_back(char(v >> 8));
    out.push_back(char(v >> 16)); out.push_back(char(v >> 24));
}
} // namespace

void PminiPacket::reset() {
    sidTypes_.clear();
    sidTypes_.shrink_to_fit();
}

bool PminiPacket::packet(const std::uint8_t *in, std::size_t n, std::string &out) {
    try {
    if (in == nullptr || n == 0 || n > kMaxInput) return false;
    std::vector<std::uint8_t> types = sidTypes_;
    if (types.empty()) types.resize(1, 0xff); // SID zero is reserved.

    std::vector<Event> events;
    std::vector<Row> rows;
    std::vector<FieldRef> fieldRefs;
    std::vector<std::uint32_t> words;
    std::vector<SchemaField> schema;
    events.reserve(32); rows.reserve(32); fieldRefs.reserve(256);
    std::string prefix;
    std::size_t at = 0;

    while (at < n) {
        const std::size_t recordAt = at;
        std::uint64_t flen = 0; std::size_t lu = 0;
        if (!readVarint(in + at, in + n, flen, lu) || flen == 0 || flen > n - at - lu)
            return false;
        at += lu;
        const std::uint8_t *frame = in + at;
        const std::size_t frameLen = std::size_t(flen);
        at += frameLen;
        if (events.size() >= kMaxEvents) return false;

        if (frame[0] == std::uint8_t(FrameT::Table)) {
            if (!applyTable(frame, frameLen, types)) return false;
            // Store length prefix and frame together: table bytes are exactly
            // replayable even if a future writer changes its varint helper.
            events.push_back({true, static_cast<std::uint16_t>(recordAt),
                              static_cast<std::uint16_t>(lu + frameLen)});
            continue;
        }
        if (frame[0] != std::uint8_t(FrameT::Data) || rows.size() >= kMaxRows)
            return false;

        const std::uint8_t *p = frame + 1, *end = frame + frameLen;
        std::uint64_t sid = 0; std::size_t su = 0;
        if (!readVarint(p, end, sid, su)) return false;
        p += su;
        if (!knownString(types, sid)) return false;

        const std::uint8_t *tagEnd = nullptr;
        for (;;) {
            if (p >= end) return false;
            if (*p == 0) { tagEnd = p + 1; ++p; break; }
            std::uint64_t key = 0, value = 0; std::size_t ku = 0, vu = 0;
            if (!readVarint(p, end, key, ku)) return false;
            p += ku;
            if (!readVarint(p, end, value, vu)) return false;
            p += vu;
            if (!knownString(types, key) || !knownString(types, value)) return false;
        }
        if (prefix.empty()) {
            prefix.assign(reinterpret_cast<const char *>(frame), std::size_t(tagEnd - frame));
            if (prefix.size() > kMaxPrefix) return false;
        } else if (prefix.size() != std::size_t(tagEnd - frame) ||
                   std::memcmp(prefix.data(), frame, prefix.size()) != 0) {
            return false; // mixed measurement or tags
        }

        Row row;
        row.fieldAt = static_cast<std::uint16_t>(fieldRefs.size());
        while (p < end && *p != 0) {
            std::uint64_t fsid = 0; std::size_t fu = 0;
            if (!readVarint(p, end, fsid, fu)) return false;
            p += fu;
            std::uint8_t dt = 0;
            if (!knownNumeric(types, fsid, dt) || dt == std::uint8_t(WireDT::F64)) return false;
            const std::size_t width = dtSize(WireDT(dt));
            if (width == 0 || width > 4 || std::size_t(end - p) < width) return false;
            for (std::size_t i = row.fieldAt; i < fieldRefs.size(); ++i)
                if (fieldRefs[i].sid == fsid) return false;
            if (fieldRefs.size() >= 0xffffu || row.fieldCount == 0xffu) return false;
            fieldRefs.push_back({static_cast<std::uint16_t>(fsid), dt,
                                 static_cast<std::uint16_t>(p - in)});
            ++row.fieldCount;
            p += width;
        }
        if (p >= end || *p++ != 0) return false;
        std::uint64_t timestamp = 0; std::size_t tu = 0;
        if (!readVarint(p, end, timestamp, tu) || p + tu != end) return false;
        row.timestamp = timestamp;

        rows.push_back(row);
        events.push_back({false, 0, 0});
    }
    if (rows.empty()) {
        // A complete table-only batch is a normal tamp fallback, but it is
        // authoritative stream state and can seed the next split batch.
        sidTypes_.swap(types);
        return false;
    }

    // Build the union and a stable topological order from every row's local
    // field order. Sparse telemetry may introduce a field late, before one
    // seen in the first row; first-seen order would reject that valid stream.
    for (const Row &row : rows) {
        for (std::size_t ri = row.fieldAt; ri < row.fieldAt + row.fieldCount; ++ri) {
            const FieldRef &f = fieldRefs[ri];
            std::size_t pos = schema.size();
            for (std::size_t i = 0; i < schema.size(); ++i)
                if (schema[i].sid == f.sid) { pos = i; break; }
            if (pos == schema.size()) {
                if (schema.size() >= kMaxFields) return false;
                schema.push_back({f.sid, f.dt});
            } else if (schema[pos].dt != f.dt) {
                return false;
            }
        }
    }
    if (schema.empty()) return false;
    const std::size_t fieldCount = schema.size();
    std::uint32_t edges[ kMaxFields ] = {};
    std::uint32_t indegree = 0;
    for (const Row &row : rows) {
        for (std::size_t i = 1; i < row.fieldCount; ++i) {
            std::size_t a = fieldCount, b = fieldCount;
            const FieldRef &before = fieldRefs[row.fieldAt + i - 1];
            const FieldRef &after = fieldRefs[row.fieldAt + i];
            for (std::size_t j = 0; j < fieldCount; ++j) {
                if (schema[j].sid == before.sid) a = j;
                if (schema[j].sid == after.sid) b = j;
            }
            if (a == fieldCount || b == fieldCount || a == b) return false;
            const std::uint32_t bit = std::uint32_t(1u << b);
            if ((edges[a] & bit) == 0) { edges[a] |= bit; ++indegree; }
        }
    }
    (void)indegree;
    std::vector<SchemaField> ordered;
    ordered.reserve(fieldCount);
    std::uint32_t placed = 0;
    while (ordered.size() < fieldCount) {
        bool chose = false;
        for (std::size_t i = 0; i < fieldCount; ++i) {
            if (placed & (std::uint32_t(1u) << i)) continue;
            bool hasIncoming = false;
            for (std::size_t a = 0; a < fieldCount; ++a)
                if (!(placed & (std::uint32_t(1u) << a)) && (edges[a] & (std::uint32_t(1u) << i)))
                    { hasIncoming = true; break; }
            if (!hasIncoming) {
                ordered.push_back(schema[i]);
                placed |= std::uint32_t(1u) << i;
                chose = true;
                break;
            }
        }
        if (!chose) return false; // contradictory per-row order
    }
    schema.swap(ordered);
    for (Row &row : rows) {
        row.presence = 0;
        for (std::size_t ri = row.fieldAt; ri < row.fieldAt + row.fieldCount; ++ri) {
            const FieldRef &f = fieldRefs[ri];
            std::size_t pos = fieldCount;
            for (std::size_t i = 0; i < fieldCount; ++i)
                if (schema[i].sid == f.sid) { pos = i; break; }
            if (pos == fieldCount) return false;
            row.presence |= std::uint32_t(1u << pos);
        }
    }

    const std::size_t cols = 3 + schema.size();
    words.reserve(rows.size() * cols);
    for (const Row &row : rows) {
        words.push_back(std::uint32_t(row.timestamp));
        words.push_back(std::uint32_t(row.timestamp >> 32));
        words.push_back(row.presence);
        for (const SchemaField &s : schema) {
            std::uint32_t value = 0;
            for (std::size_t ri = row.fieldAt; ri < row.fieldAt + row.fieldCount; ++ri) {
                const FieldRef &f = fieldRefs[ri];
                if (f.sid == s.sid) {
                    const std::size_t width = dtSize(WireDT(f.dt));
                    for (std::size_t i = 0; i < width; ++i)
                        value |= std::uint32_t(in[f.valueAt + i]) << (8 * i);
                    break;
                }
            }
            words.push_back(value);
        }
    }
    const std::size_t rowCount = rows.size();
    std::vector<FieldRef>().swap(fieldRefs);
    std::vector<Row>().swap(rows);

    std::string shard;
    if (!pminiEncode(words.data(), rowCount, cols, shard) || shard.size() < 4)
        return false;

    std::string encoded;
    encoded.reserve(32 + prefix.size() + shard.size());
    encoded.append(kMagic, sizeof(kMagic) - 1);
    encoded.push_back(char(kEnvelopeVersion));
    putPVarint(encoded, events.size());
    putPVarint(encoded, prefix.size());
    encoded.append(prefix);
    putPVarint(encoded, schema.size());
    for (const SchemaField &f : schema) { putPVarint(encoded, f.sid); encoded.push_back(char(f.dt)); }
    for (const Event &event : events) {
        if (!event.table) { encoded.push_back(char(kDataEvent)); continue; }
        encoded.push_back(char(kTableEvent));
        putPVarint(encoded, event.len);
        encoded.append(reinterpret_cast<const char *>(in + event.at), event.len);
    }
    encoded.append(shard);
    putU32LE(encoded, crc32(reinterpret_cast<const std::uint8_t *>(encoded.data()), encoded.size()));
    out.append(encoded);
    sidTypes_.swap(types);
    return true;
    } catch (const std::bad_alloc &) {
        return false;
    }
}
