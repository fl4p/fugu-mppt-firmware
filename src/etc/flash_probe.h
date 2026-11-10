#pragma once

#include <cstdint>
#include <cstddef>

// Program/erase-suspend probe of the SPI flash chip: erases one scratch sector, suspends it with 75h,
// reads elsewhere while suspended, resumes with 7Ah. Only FsSupported means the chip passed.
enum FlashSusOutcome : uint8_t {
    FsUnprobed = 0,
    FsSupported,
    FsNoSuspend,    // WIP stayed 1 after 75h
    FsNoSusBit,     // WIP dropped but no SUS bit in SR2
    FsNoSlot,
    FsImage,        // passive slot image reaches the scratch sector
    FsWel,          // WREN not latched
    FsNoBusy,       // erase not running after start
    FsRefRead,
    FsTimeout,
    FsData,         // reads while suspended returned wrong data
    FsSusStuck,     // SUS bit still set after resume
    FsShort,        // erase finished implausibly fast
    FsCrashed,      // reset during the probe
    FsNA,
    FsOutcomeCount,
};

struct FlashSusProbe {
    uint32_t magic;
    uint32_t eraseUs;   // sector erase busy time, suspend window excluded
    uint16_t susUs;     // 75h sent -> WIP=0
    uint8_t outcome;
    uint8_t sr1Busy, sr1Sus, sr2Sus, sr2End;
    uint8_t cached;     // result kept from before a soft reset
};

void flashSuspendProbe();
int flashSusProbeFmt(char *buf, size_t n);
