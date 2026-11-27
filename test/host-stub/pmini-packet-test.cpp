// Focused host tests for the codec-2 wire adapter. Build this file with
// src/tele/pmini_packet.cpp; the stub below records the exact row words that
// would be handed to the native MTP2 encoder.
#include "src/tele/pmini_packet.h"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

static std::vector<std::uint32_t> encodedWords;
static std::size_t encodedRows = 0, encodedCols = 0;

bool pminiEncode(const std::uint32_t *words, std::size_t rows, std::size_t cols, std::string &out) {
    encodedWords.assign(words, words + rows * cols);
    encodedRows = rows; encodedCols = cols;
    out = "MTP2stub";
    return true;
}

static void uv(std::string &s, std::uint64_t v) {
    while (v >= 128) { s.push_back(char((v & 127) | 128)); v >>= 7; }
    s.push_back(char(v));
}

static std::string table() {
    std::string f(1, char(2));
    auto entry = [&](unsigned sid, const char *name, unsigned dt) {
        uv(f, sid); f += name; f.push_back(0); f.push_back(char(dt));
    };
    entry(1, "mppt", 0); entry(2, "device", 0); entry(3, "fry", 0);
    entry(4, "I", 9); entry(5, "count", 4); entry(6, "wide", 10);
    f.push_back(0);
    return f;
}

static std::string data(std::uint32_t bits, std::uint16_t count, std::uint64_t ts,
                        bool includeI = true, bool includeCount = true, bool reverse = false,
                        bool = false) {
    std::string f(1, char(1)); uv(f, 1); uv(f, 2); uv(f, 3); f.push_back(0);
    auto i = [&] { uv(f, 4); f.append(reinterpret_cast<const char *>(&bits), 4); };
    auto c = [&] { uv(f, 5); f.push_back(char(count)); f.push_back(char(count >> 8)); };
    if (reverse) { if (includeCount) c(); if (includeI) i(); }
    else { if (includeI) i(); if (includeCount) c(); }
    f.push_back(0); uv(f, ts); return f;
}

static std::string wire(const std::string &frame) {
    std::string out; uv(out, frame.size()); out += frame; return out;
}

static std::string f64data() {
    std::string f(1, char(1)); uv(f, 1); uv(f, 2); uv(f, 3); f.push_back(0);
    const double value = 1.0;
    uv(f, 6); f.append(reinterpret_cast<const char *>(&value), sizeof value);
    f.push_back(0); uv(f, 3); return f;
}

int main() {
    const std::string tbl = wire(table());
    const std::string d1 = wire(data(0x3f800000u, 0x1234, 0x100000001ull));
    const std::string d2 = wire(data(0x40000000u, 0, 0x100000002ull, true, false));

    // Tables in one batch are copied as literals; omitted fields survive in
    // the presence column and the private SID map is not shared with a source.
    PminiPacket p;
    std::string in = tbl + d1 + d2, out = "caller-prefix";
    assert(p.packet(reinterpret_cast<const std::uint8_t *>(in.data()), in.size(), out));
    assert(out.compare(0, 13, "caller-prefix") == 0);
    assert(out.find("PMA2") == 13);
    assert(out.find(tbl) != std::string::npos);
    assert(encodedRows == 2 && encodedCols == 5);
    assert(encodedWords[0] == 1 && encodedWords[1] == 1 && encodedWords[2] == 3);
    assert(encodedWords[5] == 2 && encodedWords[6] == 1 && encodedWords[7] == 1);
    assert(encodedWords[8] == 0x40000000u && encodedWords[9] == 0);

    // A field introduced in a later sparse row may precede an existing field;
    // the adapter derives a valid union order instead of assuming first-seen.
    PminiPacket late;
    std::string unchanged = "keep";
    const std::string lateInput = tbl + d2 + wire(data(3, 4, 5, true, true, true));
    assert(late.packet(reinterpret_cast<const std::uint8_t *>(lateInput.data()), lateInput.size(), unchanged));

    // A valid table-only packet is a fallback, but its table is still learned.
    PminiPacket split;
    unchanged = "keep";
    assert(!split.packet(reinterpret_cast<const std::uint8_t *>(tbl.data()), tbl.size(), unchanged));
    assert(unchanged == "keep");
    assert(split.packet(reinterpret_cast<const std::uint8_t *>(d1.data()), d1.size(), unchanged));
    split.reset();
    unchanged = "keep";
    assert(!split.packet(reinterpret_cast<const std::uint8_t *>(d1.data()), d1.size(), unchanged));
    assert(unchanged == "keep");

    // Reordered fields and f64 require tamp fallback.
    PminiPacket bad;
    std::string reorder = tbl + d1 + wire(data(1, 2, 3, true, true, true));
    unchanged = "keep";
    assert(!bad.packet(reinterpret_cast<const std::uint8_t *>(reorder.data()), reorder.size(), unchanged));
    PminiPacket f64;
    unchanged = "keep";
    assert(!f64.packet(reinterpret_cast<const std::uint8_t *>(tbl.data() + 0), tbl.size(), unchanged));
    const std::string wide = tbl + wire(f64data());
    assert(!f64.packet(reinterpret_cast<const std::uint8_t *>(wide.data()), wide.size(), unchanged));

    // Truncated length-prefixed frames and unknown field SIDs are never
    // interpreted as an empty/verified packet.
    PminiPacket malformed;
    const std::string trunc = std::string(1, char(5)) + "\x01";
    unchanged = "keep";
    assert(!malformed.packet(reinterpret_cast<const std::uint8_t *>(trunc.data()), trunc.size(), unchanged));
    assert(unchanged == "keep");

    // A table preceding a malformed data frame is not published into the
    // private cache; otherwise an attacker could poison the next batch.
    PminiPacket polluted;
    const std::string badBatch = tbl + trunc;
    assert(!polluted.packet(reinterpret_cast<const std::uint8_t *>(badBatch.data()), badBatch.size(), unchanged));
    assert(!polluted.packet(reinterpret_cast<const std::uint8_t *>(d1.data()), d1.size(), unchanged));
    return 0;
}
