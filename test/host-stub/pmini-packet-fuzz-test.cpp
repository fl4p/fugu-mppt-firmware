// Deterministic malformed-input sweep. This is intentionally small enough for
// ASAN/UBSAN runs while exercising every varint/frame boundary repeatedly.
#include "src/tele/pmini_packet.h"

#include <cassert>
#include <cstdint>
#include <random>
#include <string>

bool pminiEncode(const std::uint32_t *, std::size_t, std::size_t, std::string &out) {
    out += "MTP2";
    return true;
}

int main() {
    std::mt19937 gen(0x504d494eu);
    std::uniform_int_distribution<unsigned> length(0, 8192), byte(0, 255);
    PminiPacket packet;
    for (unsigned trial = 0; trial != 100000; ++trial) {
        std::string input(length(gen), '\0');
        for (char &c : input) c = char(byte(gen));
        std::string output = "unchanged";
        const bool ok = packet.packet(reinterpret_cast<const std::uint8_t *>(input.data()), input.size(), output);
        if (!ok) assert(output == "unchanged");
    }
    return 0;
}
