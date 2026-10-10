*this document is an LLM generated placeholder*

# Plan: USB config drive (issue #40), emulated FAT over LittleFS

Status 2026-10-10: implemented, host-tested, builds on IDF 5.5 and 6; bench-tested on flu (macOS) — see Verification.
Plan review: `~/codex-reviews/usb-msc-plan/review-final.md` (Codex). The design below is the revised one.

## Context

Issue #40: the ESP32 should show up as a USB drive for editing config. Host operating systems can't mount LittleFS,
so the firmware emulates a small FAT12 volume generated from `/littlefs/conf/*.conf` and writes the host's
edits back to LittleFS. LittleFS stays the only storage, so set-config, FTP, provision.py and the conf editor
keep working.

User decisions: composite MSC + CDC console; always on in `CONFIG_FUGU_WITH_USB_MSC=y` builds (flag default
off); restart after a successful commit if `!mppt.active()`; root of `conf/` with create/edit/delete.

## Design (as built)

- **TinyUSB**: `espressif/tinyusb` directly, with our own `src/usb/tusb_config.h`, descriptors, PHY init and
  task. Not `esp_tinyusb`: its `CONFIG_TINYUSB_{CDC,MSC}_ENABLED` switch on arduino-esp32's `USB*.cpp`, and its
  class callbacks collide with ours. The dependency is gated in `main/idf_component.yml` with
  `$CONFIG{FUGU_WITH_USB_MSC}`. That symbol must exist in every build, so it has no `depends on`; the
  S3 and `!WSYNC` checks live in `main/CMakeLists.txt`. The default WSYNC=y otherwise hid the symbol and broke
  every build.
- **Volume** (`src/usb/vfat.{h,cpp}`, pure C++): FAT12 with 4096 × 512 B sectors and 1 sector per cluster
  (4067 clusters), 2 FATs, a 64-entry root and LFN entries.
  - FAT2 reads mirror FAT1 and writes to FAT2 are ignored, so a late stale FAT2 write can't clobber FAT1.
  - Data sectors come from an overlay of written sectors (capped at 96 here, about 48 KiB), else from the
    immutable snapshot. A write equal to the current content takes no overlay slot.
- **collect()**: rejects the whole state on a bad or cross-linked or looping chain, a size/chain mismatch,
  duplicate names, or an unreadable `.conf` entry (a broken LFN whose 8.3 name has the extension `CON`). An
  unreadable entry can therefore never become a deletion.
- **Commit only on eject** (`START STOP UNIT` with eject), synchronously in the TinyUSB task, before the command
  status goes back to the host. Unplugging without ejecting discards the edits.
  - Strip a UTF-8 BOM, then check every changed file is strict `key=value` ASCII. Required boot confs may not be
    deleted.
  - Stage the files as `<name>.new` with checked write, fflush and fsync.
  - Run the charger/limits loaders on the staged files: `confCheck()`, factored out of `conf-check` in
    `cli.cpp`.
  - Then rename (atomic per file on LittleFS) and unlink.
  - Any failure: nothing is replaced and the console logs why.
- **Restart**: the USB task raises `UsbEvent::ConfCommitted`; the network loop restarts if `!mppt.active()`,
  else logs `conf changed, restart to apply`.
- **CDC console**: `console_{read,write}_usb` map to TinyUSB CDC. Logs reach it through `addLogCallback`, and
  the esp_log hook is also installed in USB builds. `g_app.usbConnected` comes from `tud_cdc_connected()`.
- **Download mode**: esptool's DTR/RTS sequence raises `UsbEvent::EnterDownload`. The network loop disables the
  converter, then calls `tud_disconnect()`, sets `FORCE_DOWNLOAD_BOOT` and restarts.
  - A shutdown handler hands the RTC PHY routing back to USB-Serial-JTAG on every software restart, so the ROM or
    a non-MSC image boots with USJ.

## Accepted / open (from the plan review)

- Flash writes during conversion: commit runs while converting, like `set-config` already does. Only the
  restart is gated on `mppt.active()`. No RT-owned maintenance latch.
- Multi-file updates are atomic per file only. Documented.
- Advertised capacity (2 MiB) exceeds the RAM overlay. Writes past it fail with a SCSI write error. Documented.
- Windows quick-removal users may unplug without ejecting, which loses their edits. Documented as "eject to
  apply".

## Verification

- Done:
  - `test/host-stub/vfat-test.cpp`: unit cases including broken LFN, cross-link, size mismatch, overlay cap and
    strict conf text.
  - `test/host-stub/vfat-hdiutil.sh`: macOS FAT driver round trip with fsck, edit, atomic save, create, delete
    and junk.
  - Flag-on builds on IDF 5.5 (+32.7 KB vs. the same config without the flag) and IDF 6.
  - Flag-off default build is unchanged and skips TinyUSB.
- Bench, flu (Fugu2 S3, input unpowered), macOS, 2026-10-10:
  - Enumerates as CDC + `FUGU_CONF` (PID 0x4003), auto-mounts, files match LittleFS; the console works over CDC.
  - `diskutil eject`: create, delete and reject paths all behave; the device logs the result. A rejected commit
    still loses the volume on macOS (it unmounts before START STOP UNIT), so "keep the drive" doesn't hold there.
  - `esptool chip_id` on the CDC port resets into ROM download (USJ, port renamed); `app-flash` on the new port
    works and the app comes back with the drive.
  - Free heap about 119 KB (vs 129 KB on the previous image), 451 sps, loop lag about 3 ms, no ADC errors.
- Still to do:
  - Windows and Linux.
  - The idle-converter auto-restart after a commit (flu was in manual mode, so it only logged).
  - Overlay exhaustion; editors (TextEdit, VS Code) instead of shell writes.
