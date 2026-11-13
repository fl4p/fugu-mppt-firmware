#include "flash_probe.h"
extern uint8_t g_isrCore[2]; // etc/rt.h

#include <cstdio>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_log.h>
#include <sdkconfig.h>
#include <fugu_flash_chip.h>

#define FS_MAGIC 0xF5A50002u

static FlashSusProbe s_fsp;
uint32_t g_flashLagUs[2];

#if CONFIG_IDF_TARGET_ESP32S3

#include <esp_flash.h>
#include <esp_ota_ops.h>
#include <esp_image_format.h>
#include <esp_cpu.h>
#include <esp_rom_sys.h>
#include <esp_private/cache_utils.h>
#include <cstring>
#include <esp_partition.h>
#include <esp_timer.h>
#include <esp_app_desc.h>
#include <esp_rtc_time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
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

static bool fsFill(uint32_t sector) {
    uint8_t z[256] = {};
    for (uint32_t o = 0; o < 0x1000; o += sizeof(z))
        if (esp_flash_write(nullptr, z, sector + o, sizeof(z)) != ESP_OK) return false;
    return true;
}

// Reads bursts of AS_BURST new cache lines from a 256 KB flash window every ~100 us and times each
// burst. While `erasing`, it sums the words it reads, for a check against an uncached re-read.
struct AsReader {
    const volatile uint32_t *p;
    volatile bool run, done, erasing;
    volatile uint32_t n;      // bursts done
    uint32_t maxCyc;
    uint32_t maxRdCyc;        // slowest single read
    uint32_t i0, nE, sumE;    // first word index, words read and their sum while erasing
};

#define AS_WIN (256 * 1024)
#define AS_STRIDE 8 // words: one 32-byte line per read
#define AS_BURST 8

static void asReaderTask(void *a) {
    auto &r = *(AsReader *) a;
    uint32_t i = 0;
    bool was = false;
    while (r.run) {
        const bool e = r.erasing;
        if (e && !was) r.i0 = i;
        was |= e;
        uint32_t t0 = esp_cpu_get_cycle_count(), s = 0;
        for (int k = 0; k < AS_BURST; ++k) {
            const uint32_t t1 = esp_cpu_get_cycle_count();
            s += r.p[i];
            const uint32_t d1 = esp_cpu_get_cycle_count() - t1;
            if (d1 > r.maxRdCyc) r.maxRdCyc = d1;
            if ((i += AS_STRIDE) >= AS_WIN / 4) i = 0;
        }
        uint32_t dt = esp_cpu_get_cycle_count() - t0;
        if (dt > r.maxCyc) r.maxCyc = dt;
        if (e) {
            r.sumE += s;
            r.nE += AS_BURST;
        }
        r.n = r.n + 1;
        esp_rom_delay_us(100);
    }
    r.done = true;
    vTaskDelete(nullptr);
}

// sum of the words the reader saw while erasing, read again over SPI1 (no cache)
static bool asVerify(const AsReader &r, uint32_t base) {
    uint32_t sum = 0, i = r.i0;
    for (uint32_t k = 0; k < r.nE; ++k) {
        uint32_t w;
        if (esp_flash_read(nullptr, &w, base + i * 4, 4) != ESP_OK) return false;
        sum += w;
        if ((i += AS_STRIDE) >= AS_WIN / 4) i = 0;
    }
    return sum == r.sumE;
}

static uint8_t asRun(FlashSusProbe &r, uint32_t sector) {
    if (!fugu_flash_sus_built()) return AsNotBuilt;
    if (r.outcome != FsSupported) return AsProbeFailed;
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (!run || run->size < AS_WIN) return AsTask;
    const void *map = nullptr;
    esp_partition_mmap_handle_t mh;
    if (esp_partition_mmap(run, 0, AS_WIN, ESP_PARTITION_MMAP_DATA, &map, &mh) != ESP_OK) return AsTask;
    if (!fsFill(sector)) {
        esp_partition_munmap(mh);
        return AsTask;
    }

    static AsReader rd;
    rd = {};
    rd.p = (const volatile uint32_t *) map;
    rd.run = true;
    TaskHandle_t th = nullptr;
    xTaskCreatePinnedToCore(asReaderTask, "fsAuto", 2048, &rd, configMAX_PRIORITIES - 1, &th, !xPortGetCoreID());
    if (!th) {
        esp_partition_munmap(mh);
        return AsTask;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    rd.maxCyc = 0;
    rd.maxRdCyc = 0;

    fugu_flash_sus_set(true);
    rd.erasing = true;
    const uint32_t n0 = rd.n;
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_flash_erase_region(nullptr, sector, 0x1000);
    const int64_t t1 = esp_timer_get_time();
    const uint32_t n1 = rd.n;
    rd.erasing = false;
    fugu_flash_sus_set(false);

    rd.run = false;
    for (int i = 0; i < 50 && !rd.done; ++i) vTaskDelay(1);
    if (!rd.done) return AsTask; // keep the mapping: the reader may still use it
    esp_partition_munmap(mh);

    const uint32_t tpu = esp_rom_get_cpu_ticks_per_us();
    const uint32_t ms = (uint32_t) ((t1 - t0) / 1000), stall = rd.maxCyc / (tpu ? tpu : 1), n = n1 - n0;
    r.autoEraseMs = ms > 0xffff ? 0xffff : ms;
    r.autoStallUs = stall > 0xffff ? 0xffff : stall;
    r.autoReads = n > 0xffff ? 0xffff : n;

    if (err != ESP_OK) return AsErase;
    if (ms > 2000) return AsSlow;
    if (n < 10 || rd.nE < 10 * AS_BURST) return AsNoOverlap;
    if (!asVerify(rd, run->address)) return AsData;
    // the reader forces AS_BURST misses back to back; the RT path is in IRAM (rt_iram.lf) and takes single
    // ones, so each one counts: 600 us leaves the ADC DMA (1.9 ms) three of them
    if (stall > AS_BURST * 400 || rd.maxRdCyc / (tpu ? tpu : 1) > 600) return AsStall;
    uint32_t b[64];
    for (uint32_t o = 0; o < 0x1000; o += sizeof(b)) {
        if (esp_flash_read(nullptr, b, sector + o, sizeof(b)) != ESP_OK) return AsNotErased;
        for (uint32_t w : b)
            if (w != 0xFFFFFFFFu) return AsNotErased;
    }
    return AsPass;
}

static uint8_t fsRun(FlashSusProbe &r, uint32_t &sector) {
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *p = esp_ota_get_next_update_partition(nullptr);
    if (!run || !p || p == run || p->size < 0x10000 || (p->address & 0xFFF) || (p->size & 0xFFF))
        return FsNoSlot;
    sector = p->address + p->size - 0x1000;

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
    if (!fsFill(sector)) return FsPrep;

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
    uint32_t fw;
    memcpy(&fw, esp_app_get_description()->app_elf_sha256, sizeof(fw));
    const uint32_t nowS = (uint32_t) (esp_rtc_get_time_us() / 1000000);
    // a soft reset of the same firmware within the hour reuses the last result: a reboot loop must not
    // wear the scratch sector, and a crash during the test must not repeat
    for (size_t i = 0; i < sizeof(s_rtc) / 4; ++i) ((uint32_t *) &s_fsp)[i] = s_rtc[i];
    // a crash while suspend was active: stay off until power-on or new firmware
    if (soft && s_fsp.magic == FS_MAGIC && s_fsp.fwTag == fw && s_fsp.autoOut == AsPass && rr != ESP_RST_SW &&
        rr != ESP_RST_BROWNOUT)
        s_fsp.autoOut = AsCrashed;
    const bool crashed = s_fsp.outcome == FsCrashed || s_fsp.autoOut == AsCrashed;
    if (soft && s_fsp.magic == FS_MAGIC && s_fsp.outcome < FsOutcomeCount && s_fsp.outcome != FsUnprobed &&
        s_fsp.autoOut < AsAutoCount && s_fsp.fwTag == fw && (crashed || nowS - s_fsp.testedS < 3600)) {
        s_fsp.cached = 1;
        fsSave();
    } else {
        s_fsp = {};
        s_fsp.magic = FS_MAGIC;
        s_fsp.fwTag = fw;
        s_fsp.testedS = nowS;
        s_fsp.outcome = FsCrashed; // kept if the probe itself resets the chip
        s_fsp.autoOut = AsCrashed;
        fsSave();
        uint32_t sector = 0;
        s_fsp.outcome = fsRun(s_fsp, sector);
        fsSave();
        s_fsp.autoOut = asRun(s_fsp, sector);
        fsSave();
    }
    fugu_flash_sus_set(s_fsp.autoOut == AsPass);
    char buf[200];
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
    static const char *const autoNames[AsAutoCount] = {
        "untested", "pass", "not-built", "probe-failed", "error(erase)", "fail(slow)", "fail(stall)",
        "fail(data)", "fail(no-overlap)", "fail(not-erased)", "error(crash)", "error(task)",
    };
    const auto &r = s_fsp;
    const char *nm = r.outcome < FsOutcomeCount ? names[r.outcome] : "unprobed";
    if (r.outcome == FsNA || r.outcome == FsUnprobed)
        return snprintf(buf, n, "flash-suspend: %s", nm);
    return snprintf(buf, n, "flash-suspend: %s tsus=%uus erase=%luus sr1=%02x/%02x sr2=%02x/%02x; auto: %s "
                            "erase=%ums stall=%uus reads=%u -> %s%s; lag flash/other %lu/%luus; isr core adc/alert %u/%u", nm,
                    (unsigned) r.susUs, (unsigned long) r.eraseUs, (unsigned) r.sr1Busy, (unsigned) r.sr1Sus,
                    (unsigned) r.sr2Sus, (unsigned) r.sr2End,
                    r.autoOut < AsAutoCount ? autoNames[r.autoOut] : "?", (unsigned) r.autoEraseMs,
                    (unsigned) r.autoStallUs, (unsigned) r.autoReads,
                    fugu_flash_sus_on() ? "active" : "caches-off", r.cached ? " (cached)" : "",
                    (unsigned long) g_flashLagUs[0], (unsigned long) g_flashLagUs[1], (unsigned) g_isrCore[0],
                    (unsigned) g_isrCore[1]);
}
