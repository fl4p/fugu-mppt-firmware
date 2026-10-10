#include "usb_dev.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <dirent.h>
#include <memory>
#include <stdexcept>
#include <unistd.h>

#include <esp_log.h>
#include <esp_mac.h>
#include <esp_random.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/rtc_cntl_reg.h>
#include "esp_private/usb_phy.h"
#include <tusb.h>

#include "vfat.h"
#include "cli.h"
#include "logging.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
#error "the OTG controller owns the USB PHY: disable the USB-Serial-JTAG console (sdkconfig.usb_msc)"
#endif

static const char *TAG = "usb";
static constexpr char CONF_DIR[] = "/littlefs/conf/";
static constexpr size_t SNAPSHOT_MAX = 32 * 1024;
static constexpr uint16_t OVERLAY_MAX = 96;

// files the firmware needs at boot; the drive may edit but not delete them
static const char *const REQUIRED[] = {"board.conf", "sensor.conf", "limits.conf", "coil.conf",
                                       "converter.conf", "charger.conf", "tracker.conf"};

enum ITF { ITF_CDC = 0, ITF_CDC_DATA, ITF_MSC, ITF_COUNT };

// all volume state below is only touched from the TinyUSB task
static vfat::Volume *s_vol;
static bool s_ejected, s_restartAfterStatus;
static std::atomic<UsbEvent> s_event{UsbEvent::None};
static char s_serial[13];

// --- descriptors ---

static const tusb_desc_device_t s_devDesc = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = TUSB_CLASS_MISC,
        .bDeviceSubClass = MISC_SUBCLASS_COMMON,
        .bDeviceProtocol = MISC_PROTOCOL_IAD,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = 0x303A,
        .idProduct = 0x4003, // Espressif TinyUSB default PID for CDC + MSC
        .bcdDevice = 0x0100,
        .iManufacturer = 1,
        .iProduct = 2,
        .iSerialNumber = 3,
        .bNumConfigurations = 1,
};

static constexpr uint16_t CONFIG_LEN = TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN;
static const uint8_t s_cfgDesc[] = {
        TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_LEN, 0, 100),
        TUD_CDC_DESCRIPTOR(ITF_CDC, 4, 0x81, 8, 0x02, 0x82, CFG_TUD_CDC_EP_BUFSIZE),
        TUD_MSC_DESCRIPTOR(ITF_MSC, 5, 0x03, 0x83, 64),
};

static const char *const s_strings[] = {"Fugu", "Fugu MPPT", s_serial, "Fugu Console", "Fugu Config"};

extern "C" uint8_t const *tud_descriptor_device_cb() { return (uint8_t const *) &s_devDesc; }

extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t) { return s_cfgDesc; }

extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t) {
    static uint16_t desc[33];
    uint8_t n;
    if (index == 0) {
        desc[1] = 0x0409;
        n = 1;
    } else if (index <= sizeof(s_strings) / sizeof(s_strings[0])) {
        const char *s = s_strings[index - 1];
        for (n = 0; s[n] && n < 32; ++n) desc[1 + n] = s[n];
    } else {
        return nullptr;
    }
    desc[0] = (TUSB_DESC_STRING << 8) | (2 * n + 2);
    return desc;
}

// --- drive ---

struct FileCloser {
    void operator()(FILE *f) const { fclose(f); }
};

struct DirCloser {
    void operator()(DIR *d) const { closedir(d); }
};

// appends at most `budget` bytes; false on error or if the file is larger
static bool readFile(const std::string &path, std::string &out, size_t budget) {
    std::unique_ptr<FILE, FileCloser> f(fopen(path.c_str(), "rb"));
    if (!f) return false;
    char buf[256];
    size_t n;
    while ((n = fread(buf, 1, sizeof buf, f.get())) > 0) {
        if (n > budget) return false;
        budget -= n;
        out.append(buf, n);
    }
    return !ferror(f.get());
}

static bool writeFile(const std::string &path, const std::string &data) {
    FILE *f = fopen(path.c_str(), "wb");
    if (!f) return false;
    bool ok = fwrite(data.data(), 1, data.size(), f) == data.size() && fflush(f) == 0 && fsync(fileno(f)) == 0;
    return fclose(f) == 0 && ok;
}

static void dropVolume() {
    delete s_vol;
    s_vol = nullptr;
}

static void buildVolume() {
    dropVolume();
    s_ejected = false;
    try {
        std::vector<vfat::File> files;
        size_t budget = SNAPSHOT_MAX;
        std::unique_ptr<DIR, DirCloser> d(opendir(CONF_DIR));
        if (!d) {
            ESP_LOGE(TAG, "drive: cannot open %s", CONF_DIR);
            return;
        }
        while (auto *de = readdir(d.get())) {
            if (de->d_type != DT_REG) continue;
            if (!vfat::validName(de->d_name)) {
                ESP_LOGW(TAG, "drive: hiding %s", de->d_name);
                continue;
            }
            vfat::File f{de->d_name, {}};
            if (!readFile(CONF_DIR + f.name, f.data, budget)) {
                ESP_LOGE(TAG, "drive: cannot snapshot %s", f.name.c_str());
                return;
            }
            budget -= f.data.size();
            files.push_back(std::move(f));
        }
        std::unique_ptr<vfat::Volume> vol(new vfat::Volume(OVERLAY_MAX));
        if (!vol->build(std::move(files), (uint32_t) esp_random())) {
            ESP_LOGE(TAG, "drive: conf files don't fit");
            return;
        }
        s_vol = vol.release();
    } catch (const std::exception &e) {
        ESP_LOGE(TAG, "drive: %s", e.what());
    }
}

static bool isRequired(const std::string &name) {
    for (auto *r: REQUIRED) if (name == r) return true;
    return false;
}

enum class Commit { Rejected, Unchanged, Applied, Partial };

// Validates the host's edits, stages them as <name>.new and only then renames/unlinks. The apply
// phase stops at the first failure and does not allocate.
static Commit commit() {
    std::vector<std::string> staged, targets, removed;
    try {
        std::vector<vfat::File> host;
        std::string err;
        if (!s_vol->collect(host, err)) {
            UART_LOG("usb drive: %s, nothing applied", err.c_str());
            return Commit::Rejected;
        }
        vfat::Diff d = vfat::diff(s_vol->baseline(), host);
        if (d.empty()) return Commit::Unchanged;
        for (auto &u: d.upsert) {
            if (!u.data.compare(0, 3, "\xEF\xBB\xBF")) u.data.erase(0, 3);
            if (!vfat::validConfText(u.data, err)) {
                UART_LOG("usb drive: %s %s, nothing applied", u.name.c_str(), err.c_str());
                return Commit::Rejected;
            }
        }
        for (auto &r: d.removed) {
            if (isRequired(r)) {
                UART_LOG("usb drive: %s is required and can't be deleted, nothing applied", r.c_str());
                return Commit::Rejected;
            }
            removed.push_back(CONF_DIR + r);
        }
        staged.reserve(d.upsert.size());
        const char *charger = nullptr, *limits = nullptr;
        for (auto &u: d.upsert) {
            targets.push_back(CONF_DIR + u.name);
            staged.push_back(targets.back() + ".new");
            if (!writeFile(staged.back(), u.data)) throw std::runtime_error("write " + staged.back() + " failed");
            if (u.name == "charger.conf") charger = staged.back().c_str();
            if (u.name == "limits.conf") limits = staged.back().c_str();
        }
        if (!confCheck(charger, limits)) throw std::runtime_error("conf rejected");
    } catch (const std::exception &e) {
        for (auto &p: staged) unlink(p.c_str());
        UART_LOG("usb drive: %s, nothing applied", e.what());
        return Commit::Rejected;
    }

    size_t done = 0, total = staged.size() + removed.size();
    for (size_t i = 0; i < staged.size() && done == i; ++i) {
        if (rename(staged[i].c_str(), targets[i].c_str()) == 0) ++done;
        UART_LOG("usb drive: %s %s", done > i ? "updated" : "FAILED to update", targets[i].c_str());
    }
    for (size_t i = done; i < staged.size(); ++i) unlink(staged[i].c_str());
    for (size_t i = 0; i < removed.size() && done == staged.size() + i; ++i) {
        if (unlink(removed[i].c_str()) == 0 || errno == ENOENT) ++done; // ENOENT: retry after a partial commit
        UART_LOG("usb drive: %s %s", done > staged.size() + i ? "deleted" : "FAILED to delete", removed[i].c_str());
    }
    if (done == total) return Commit::Applied;
    UART_LOG("usb drive: PARTIALLY applied (%u of %u), check the conf files", (unsigned) done, (unsigned) total);
    return Commit::Partial;
}

extern "C" void tud_mount_cb() { buildVolume(); }

extern "C" void tud_umount_cb() { dropVolume(); }

static bool ready(uint8_t lun) {
    if (s_vol && !s_ejected) return true;
    tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
    return false;
}

extern "C" void tud_msc_inquiry_cb(uint8_t, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    memcpy(vendor_id, "Fugu    ", 8);
    memcpy(product_id, "Config Drive    ", 16);
    memcpy(product_rev, "1.0 ", 4);
}

extern "C" bool tud_msc_test_unit_ready_cb(uint8_t lun) { return ready(lun); }

extern "C" void tud_msc_capacity_cb(uint8_t, uint32_t *block_count, uint16_t *block_size) {
    *block_count = vfat::Volume::TOTAL_SECTORS;
    *block_size = vfat::SECTOR;
}

extern "C" bool tud_msc_is_writable_cb(uint8_t) { return true; }

// Eject commits before its status goes back to the host; a rejected or partial commit fails the
// eject and keeps the drive (with the host's edits) so the user can fix and eject again.
extern "C" bool tud_msc_start_stop_cb(uint8_t lun, uint8_t, bool start, bool load_eject) {
    if (!load_eject) return true;
    if (start) {
        if (s_ejected) buildVolume();
        return true;
    }
    if (!s_vol || s_ejected) return true;
    switch (commit()) {
        case Commit::Applied:
            s_restartAfterStatus = true;
            break;
        case Commit::Unchanged:
            UART_LOG("usb drive: no conf changes");
            break;
        default:
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x26, 0x00); // invalid field in parameter list
            return false;
    }
    s_ejected = true;
    return true;
}

extern "C" void tud_msc_scsi_complete_cb(uint8_t, uint8_t const scsi_cmd[16]) {
    if (scsi_cmd[0] == 0x1B && s_restartAfterStatus) { // START STOP UNIT status sent
        s_restartAfterStatus = false;
        s_event = UsbEvent::ConfCommitted;
    }
}

extern "C" int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    if (!ready(lun) || offset % vfat::SECTOR || bufsize % vfat::SECTOR) return -1;
    auto *p = (uint8_t *) buffer;
    for (uint32_t i = 0; i < bufsize / vfat::SECTOR; ++i)
        if (!s_vol->read(lba + offset / vfat::SECTOR + i, p + i * vfat::SECTOR)) return -1;
    return (int32_t) bufsize;
}

extern "C" int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    if (!ready(lun) || offset % vfat::SECTOR || bufsize % vfat::SECTOR) return -1;
    for (uint32_t i = 0; i < bufsize / vfat::SECTOR; ++i)
        if (!s_vol->write(lba + offset / vfat::SECTOR + i, buffer + i * vfat::SECTOR)) {
            ESP_LOGE(TAG, "drive: write buffer full"); // TinyUSB reports this as "medium not present"
            return -1;
        }
    return (int32_t) bufsize;
}

extern "C" int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *, uint16_t) {
    if (scsi_cmd[0] == 0x35) return 0; // SYNCHRONIZE CACHE: writes are already in RAM
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

// --- CDC ---

// esptool's reset sequences request the ROM download mode, as (DTR,RTS):
// ClassicReset 01 11 10 00, UnixTightReset 01 10 00 (after 00 11)
extern "C" void tud_cdc_line_state_cb(uint8_t, bool dtr, bool rts) {
    static uint8_t step;
    static TickType_t t0;
    TickType_t now = xTaskGetTickCount();
    if (!dtr && rts) {
        step = 1;
        t0 = now;
    } else if (step && now - t0 > pdMS_TO_TICKS(2000)) step = 0;
    else if (dtr && rts) step = step == 1 ? 2 : 0;
    else if (dtr) step = step ? 3 : 0;
    else {
        if (step == 3) s_event = UsbEvent::EnterDownload;
        step = 0;
    }
}

bool usbCdcConnected() { return tud_cdc_connected(); }

int usbCdcRead(char *buf, size_t len) { return tud_cdc_available() ? (int) tud_cdc_read(buf, len) : 0; }

int usbCdcWrite(const char *buf, size_t len) {
    if (!tud_cdc_connected()) return 0;
    uint32_t n = tud_cdc_write(buf, std::min<uint32_t>(len, tud_cdc_write_available()));
    tud_cdc_write_flush();
    return (int) n;
}

static void cdcLogSink(const char *s, uint16_t len) { usbCdcWrite(s, len); }

// --- device ---

static void IRAM_ATTR phyToUsj() {
    CLEAR_PERI_REG_MASK(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_HW_USB_PHY_SEL | RTC_CNTL_SW_USB_PHY_SEL | RTC_CNTL_USB_PAD_ENABLE);
}

static void usbTask(void *) {
    tusb_rhport_init_t init = {};
    init.role = TUSB_ROLE_DEVICE;
    init.speed = TUSB_SPEED_AUTO;
    if (!tusb_init(0, &init)) {
        ESP_LOGE(TAG, "tud_init failed");
        vTaskDelete(nullptr);
    }
    for (;;) tud_task();
}

void usbDevBegin() {
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(s_serial, sizeof s_serial, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    usb_phy_config_t pc = {};
    pc.controller = USB_PHY_CTRL_OTG;
    pc.target = USB_PHY_TARGET_INT;
    pc.otg_mode = USB_OTG_MODE_DEVICE;
    usb_phy_handle_t phy;
    if (usb_new_phy(&pc, &phy) != ESP_OK) {
        ESP_LOGE(TAG, "usb phy init failed");
        return;
    }
    // a software reset keeps the RTC PHY routing; hand it back to USB-Serial-JTAG for whatever boots next
    if (esp_register_shutdown_handler(phyToUsj) != ESP_OK)
        ESP_LOGW(TAG, "no shutdown handler: a restart leaves the PHY on OTG");
    if (xTaskCreatePinnedToCore(usbTask, "usb", 6144, nullptr, 5, nullptr, 0) != pdPASS) {
        ESP_LOGE(TAG, "usb task not created");
        return;
    }
    addLogCallback(cdcLogSink, false);
}

UsbEvent usbDevPoll() { return s_event.exchange(UsbEvent::None); }

void usbEnterDownload() {
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(20));
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
}
