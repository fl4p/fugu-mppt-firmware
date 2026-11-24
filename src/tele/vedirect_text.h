#pragma once

// VE.Direct text-mode block (Victron VE.Direct Protocol 3.34): "\r\n<label>\t<value>" fields, ending in a
// Checksum field whose byte makes the block sum to 0 mod 256.

#include <cstddef>
#include <cstdint>
#include <cstdio>

struct VeText {
    uint16_t pid, fw;
    const char *ser;
    int32_t v_mV, i_mA, vpv_mV;
    uint32_t ppv_W;
    uint8_t cs, mppt, err;
    uint32_t off;
    uint32_t h19, h20, h21, h22, h23; // 0.01 kWh, 0.01 kWh, W, 0.01 kWh, W
    uint16_t hsds;
};

// CS (state of operation) and MPPT (tracker mode) values for MPPT chargers.
enum : uint8_t {
    VE_CS_OFF = 0, VE_CS_FAULT = 2, VE_CS_BULK = 3, VE_CS_ABSORPTION = 4, VE_CS_FLOAT = 5, VE_CS_EXT = 252
};
enum : uint8_t { VE_MPPT_OFF = 0, VE_MPPT_LIMITED = 1, VE_MPPT_ACTIVE = 2 };
enum : uint32_t { VE_OR_NO_INPUT = 0x1, VE_OR_PROTECTION = 0x10 };

// Writes one block into buf (binary: the checksum byte can be any value). Returns its length, or 0 if it
// does not fit.
inline size_t veTextBlock(char *buf, size_t n, const VeText &t) {
    int l = snprintf(buf, n,
                     "\r\nPID\t0x%04X\r\nFW\t%u\r\nSER#\t%s\r\nV\t%ld\r\nI\t%ld\r\nVPV\t%ld\r\nPPV\t%lu"
                     "\r\nCS\t%u\r\nMPPT\t%u\r\nOR\t0x%08lX\r\nERR\t%u"
                     "\r\nH19\t%lu\r\nH20\t%lu\r\nH21\t%lu\r\nH22\t%lu\r\nH23\t%lu\r\nHSDS\t%u\r\nChecksum\t",
                     (unsigned) t.pid, (unsigned) t.fw, t.ser, (long) t.v_mV, (long) t.i_mA, (long) t.vpv_mV,
                     (unsigned long) t.ppv_W, (unsigned) t.cs, (unsigned) t.mppt, (unsigned long) t.off,
                     (unsigned) t.err, (unsigned long) t.h19, (unsigned long) t.h20, (unsigned long) t.h21,
                     (unsigned long) t.h22, (unsigned long) t.h23, (unsigned) t.hsds);
    if (l <= 0 || (size_t) l + 1 >= n) return 0;
    uint8_t sum = 0;
    for (int i = 0; i < l; ++i) sum += (uint8_t) buf[i];
    buf[l] = (char) (uint8_t) (0u - sum);
    return (size_t) l + 1;
}
