#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Adapter for codec id 2. packet() appends only the PMA2 payload; the caller
// supplies the codec id and transport framing. It returns false, leaving out
// unchanged, for malformed/mixed/unsupported input so the caller can use tamp.
class PminiPacket {
public:
    bool packet(const std::uint8_t *in, std::size_t n, std::string &out);
    void reset();

private:
    // Private SID -> WireDT map. It is deliberately not an alias of the
    // producer's SymbolTable, which may be mutated by another task.
    std::vector<std::uint8_t> sidTypes_;
};
