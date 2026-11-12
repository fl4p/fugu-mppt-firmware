#include "flash_probe.h"

#include <cstdio>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_log.h>
#include <sdkconfig.h>

#define FS_MAGIC 0xF5A50001u

static FlashSusProbe s_fsp;

#if CONFIG_IDF_TARGET_ESP32S3

#include <esp_flash.h>
#include <esp_ota_ops.h>
#include <esp_image_format.h>
#include <esp_cpu.h>
#include <esp_rom_sys.h>
#include <esp_private/cache_utils.h>
#if __has_include(<esp_flash_chips/spi_flash_chip_driver.h>)
#include <esp_flash_chips/spi_flash_chip_driver.h>
#include <esp_flash_chips/spi_flash_defs.h>
#else
#include <spi_flash_chip_driver.h>
#include <spi_flash/spi_flash_defs.h>
#endif

#define FS_REF_LEN 64
#define FS_CHUNK 32
#define FS_SUS_MASK 0x84 // SR2 bit7 SUS (generic), bit2 SUS1/SUS2 (GD)
#define FS_WEL 0x02

static DRAM_ATTR uint8_t s_ref[FS_REF_LEN];
static DRAM_ATTR uint8_t s_got[FS_REF_LEN];
static_assert(sizeof(FlashSusProbe) % 4 == 0);
static RTC_NOINIT_ATTR uint32_t s_rtc[sizeof(FlashSusProbe) / 4]; // word access only

static void fsSave() {
    for (size_t i = 0; i < sizeof(s_rtc) / 4; ++i) s_rtc[i] = ((const uint32_t *) &s_fsp)[i];
}

static IRAM_ATTR __attribute__((noinline)) void fsCmd(spi_flash_host_inst_t *h, uint8_t cmd, uint32_t addr, uint8_t abits, uint8_t dummy,
                            uint8_t *rx, uint8_t n) {
    spi_flash_trans_t t = {};
    t.command = cmd;
    t.address = addr;
    t.address_bitlen = abits;
    t.dummy_bitlen = dummy;
    t.miso_data = rx;
    t.miso_len = n;
    t.flags = SPI_FLASH_TRANS_FLAG_IGNORE_BASEIO;
    t.io_mode = dummy ? SPI_FLASH_FASTRD : SPI_FLASH_SLOWRD;
    h->driver->common_command(h, &t);
}

static IRAM_ATTR __attribute__((noinline)) uint8_t fsSr(spi_flash_host_inst_t *h, uint8_t cmd) {
    uint8_t v = 0;
    fsCmd(h, cmd, 0, 0, 0, &v, 1);
    return v;
}

// Runs with caches + interrupts off: no flash-resident code or rodata from here.
static IRAM_ATTR __attribute__((noinline)) uint8_t fsProbeLocked(spi_flash_host_inst_t *h, uint32_t sector, uint32_t ref, uint32_t tpu,
                                       FlashSusProbe &r) {
    fsCmd(h, CMD_WREN, 0, 0, 0, nullptr, 0);
    if (!(fsSr(h, CMD_RDSR) & FS_WEL)) return FsWel;

    uint32_t t0 = esp_cpu_get_cycle_count();
    fsCmd(h, CMD_SECTOR_ERASE, sector, 24, 0, nullptr, 0);
    esp_rom_delay_us(1500);
    uint8_t s1 = r.sr1Busy = fsSr(h, CMD_RDSR);
    if (!(s1 & SR_WIP)) {
        fsCmd(h, CMD_WRDI, 0, 0, 0, nullptr, 0);
        return FsNoBusy;
    }

    uint32_t ts = esp_cpu_get_cycle_count();
    fsCmd(h, CMD_SUSPEND, 0, 0, 0, nullptr, 0);
    do s1 = fsSr(h, CMD_RDSR);
    while ((s1 & SR_WIP) && esp_cpu_get_cycle_count() - ts < 500 * tpu);
    uint32_t tsus = esp_cpu_get_cycle_count() - ts;
    r.sr1Sus = s1;
    r.sr2Sus = fsSr(h, CMD_RDSR2);
    bool dataOk = false;
    if (!(s1 & SR_WIP)) {
        r.susUs = (uint16_t) (tsus / tpu);
        for (uint32_t o = 0; o < FS_REF_LEN; o += FS_CHUNK)
            fsCmd(h, CMD_FASTRD, ref + o, 24, 8, s_got + o, FS_CHUNK);
        dataOk = true;
        for (uint32_t i = 0; i < FS_REF_LEN; ++i)
            if (s_got[i] != s_ref[i]) dataOk = false;
    }

    uint32_t tr = esp_cpu_get_cycle_count();
    fsCmd(h, CMD_RESUME, 0, 0, 0, nullptr, 0);
    uint32_t tlast = tr;
    for (;;) {
        uint32_t now = esp_cpu_get_cycle_count();
        if (!(fsSr(h, CMD_RDSR) & SR_WIP)) {
            r.eraseUs = (now - t0 - (tr - ts)) / tpu;
            break;
        }
        if (now - tr > 2000000u * tpu) return FsTimeout; // datasheet max ~400 ms
        if (now - tlast > 100000u * tpu) { // chip may have missed the resume
            fsCmd(h, CMD_RESUME, 0, 0, 0, nullptr, 0);
            tlast = now;
        }
    }
    r.sr2End = fsSr(h, CMD_RDSR2);

    if (r.sr1Sus & SR_WIP) return FsNoSuspend;
    if (!(r.sr2Sus & FS_SUS_MASK)) return FsNoSusBit;
    if (!dataOk) return FsData;
    if (r.sr2End & FS_SUS_MASK) return FsSusStuck;
    if (r.eraseUs < 5000) return FsShort; // 1.5 ms of it ran before 75h
    return FsSupported;
}

// The caller runs from flash, so cache off/on must happen in IRAM.
static IRAM_ATTR __attribute__((noinline)) uint8_t fsProbeIram(spi_flash_host_inst_t *h, uint32_t sector, uint32_t ref,
                                                             uint32_t tpu, FlashSusProbe &r) {
    spi_flash_disable_interrupts_caches_and_other_cpu();
    uint8_t o = fsProbeLocked(h, sector, ref, tpu, r);
    spi_flash_enable_interrupts_caches_and_other_cpu();
    return o;
}

static uint8_t fsRun(FlashSusProbe &r) {
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *p = esp_ota_get_next_update_partition(nullptr);
    if (!run || !p || p == run || p->size < 0x10000 || (p->address & 0xFFF) || (p->size & 0xFFF))
        return FsNoSlot;
    uint32_t sector = p->address + p->size - 0x1000;

    uint32_t magic = 0;
    if (esp_flash_read(nullptr, &magic, p->address, sizeof(magic)) != ESP_OK) return FsRefRead;
    if (magic != 0xFFFFFFFFu) {
        esp_partition_pos_t pos = {.offset = p->address, .size = p->size};
        esp_image_metadata_t md = {};
        if (esp_image_get_metadata(&pos, &md) != ESP_OK) return FsImage;
        uint32_t end = (md.start_addr + md.image_len + 0xFFF) & ~0xFFFu;
        if (end + 0x1000 > sector) return FsImage; // + room for a signature block
    }

    // a blank sector erases in a few ms on some chips, too fast to tell from an aborted erase
    uint8_t z[256] = {};
    for (uint32_t o = 0; o < 0x1000; o += sizeof(z))
        if (esp_flash_write(nullptr, z, sector + o, sizeof(z)) != ESP_OK) return FsPrep;

    if (esp_flash_read(nullptr, s_ref, run->address, FS_REF_LEN) != ESP_OK) return FsRefRead;

    spi_flash_host_inst_t *h = esp_flash_default_chip->host;
    uint32_t tpu = esp_rom_get_cpu_ticks_per_us();
    if (tpu < 10 || tpu > 400) return FsRefRead;
    return fsProbeIram(h, sector, run->address, tpu, r);
}

void flashSuspendProbe() {
    auto rr = esp_reset_reason();
    bool soft = rr == ESP_RST_SW || rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT ||
                rr == ESP_RST_WDT || rr == ESP_RST_BROWNOUT;
    // soft resets reuse the last result: a reboot loop must not wear the scratch sector
    for (size_t i = 0; i < sizeof(s_rtc) / 4; ++i) ((uint32_t *) &s_fsp)[i] = s_rtc[i];
    if (soft && s_fsp.magic == FS_MAGIC && s_fsp.outcome < FsOutcomeCount && s_fsp.outcome != FsUnprobed) {
        s_fsp.cached = 1;
    } else {
        s_fsp = {};
        s_fsp.magic = FS_MAGIC;
        s_fsp.outcome = FsCrashed; // kept if the probe itself resets the chip
        fsSave();
        s_fsp.outcome = fsRun(s_fsp);
        fsSave();
    }
    char buf[128];
    flashSusProbeFmt(buf, sizeof(buf));
    ESP_LOGI("flash", "%s", buf);
}

#else

void flashSuspendProbe() {
    s_fsp = {};
    s_fsp.outcome = FsNA;
}

#endif

int flashSusProbeFmt(char *buf, size_t n) {
    static const char *const names[FsOutcomeCount] = {
        "unprobed", "supported", "unsupported(busy)", "unsupported(no-sus-bit)", "untested(no-slot)",
        "untested(image)", "untested(wel)", "untested(no-busy)", "untested(ref-read)", "error(timeout)",
        "error(data)", "error(sus-stuck)", "error(short)", "error(crash)", "n/a", "untested(prep)",
    };
    const auto &r = s_fsp;
    const char *nm = r.outcome < FsOutcomeCount ? names[r.outcome] : "unprobed";
    if (r.outcome == FsNA || r.outcome == FsUnprobed)
        return snprintf(buf, n, "flash-suspend: %s", nm);
    uint32_t caps = 0;
#if CONFIG_IDF_TARGET_ESP32S3
    if (esp_flash_default_chip->chip_drv->get_chip_caps)
        caps = esp_flash_default_chip->chip_drv->get_chip_caps(esp_flash_default_chip);
#endif
    return snprintf(buf, n, "flash-suspend: %s tsus=%uus erase=%luus sr1=%02x/%02x sr2=%02x/%02x idf-cap=%u%s", nm,
                    (unsigned) r.susUs, (unsigned long) r.eraseUs, (unsigned) r.sr1Busy, (unsigned) r.sr1Sus,
                    (unsigned) r.sr2Sus, (unsigned) r.sr2End, (unsigned) (caps & 1), r.cached ? " (cached)" : "");
}
