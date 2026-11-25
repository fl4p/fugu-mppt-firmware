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
    FsPrep,         // programming the scratch sector failed
    FsOutcomeCount,
};

// Hardware auto-suspend test: a sector erase with caches on while the other core reads flash.
enum FlashAutoOutcome : uint8_t {
    AsUntested = 0,
    AsPass,
    AsNotBuilt,     // build without auto-suspend, or the flash guard is not installed
    AsProbeFailed,  // the 75h probe above did not pass
    AsErase,        // erase returned an error
    AsSlow,         // erase took > 2 s
    AsStall,        // a burst of 8 flash reads stalled > 1 ms
    AsData,         // flash read wrong data during the erase
    AsNoOverlap,    // the reader made no progress during the erase
    AsNotErased,    // sector not blank afterwards
    AsCrashed,      // reset during the test
    AsTask,
    AsAutoCount,
};

struct FlashSusProbe {
    uint32_t magic;
    uint32_t eraseUs;   // sector erase busy time, suspend window excluded
    uint16_t susUs;     // 75h sent -> WIP=0
    uint8_t outcome;
    uint8_t sr1Busy, sr1Sus, sr2Sus, sr2End;
    uint8_t cached;     // result kept from before a soft reset
    uint8_t autoOut;
    uint16_t autoEraseMs, autoStallUs, autoReads; // read bursts completed during the erase (saturating)
    uint32_t fwTag;     // firmware the result belongs to
    uint32_t testedS;   // RTC time of the test
};

// worst RT loop gap [us]: [0] overlapping a flash op, [1] otherwise; cleared by reset-lag
extern uint32_t g_flashLagUs[2];
// the op behind g_flashLagUs[0]: uptime, starting task, still running when the loop resumed
struct FlashLagPeak {
    uint32_t atS;
    char task[16];
    uint8_t running;
};
extern FlashLagPeak g_flashLagPeak;

void flashSuspendProbe();
int flashSusProbeFmt(char *buf, size_t n);
