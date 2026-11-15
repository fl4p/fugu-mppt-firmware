---
name: project-usb-msc-config-drive
description: "Issue #40 USB config drive (CONFIG_FUGU_WITH_USB_MSC) implemented 2026-10-08, host-tested + builds, NOT run on hardware yet"
metadata:
  node_type: memory
  type: project
  originSessionId: 0e2a5cd8-0547-4cf7-bda0-b243d91accc5
  modified: 2026-10-08T21:21:11.998Z
---

USB MSC config drive (issue #40): emulated FAT12 over /littlefs/conf, composite with a TinyUSB CDC console.
It is enabled by CONFIG_FUGU_WITH_USB_MSC (default off, S3 only, excludes WSYNC). Implemented 2026-10-08.
Host-tested, builds on IDF 5.5 and 6, bench-tested on flu with macOS 2026-10-10 (enumerate, eject
commit/reject/delete, CDC console, flash via CDC reset = two steps because the port renames). macOS drops the
volume even when the eject is rejected. flu = MAC 34:85:18:82:40:28 (lock alias esp32s3-4028). Design and status: plans/usb-msc-virtual-fat.md. Reviews:
~/codex-reviews/usb-msc-{plan,code}/review-final.md.

**Why:** host OSes can't mount LittleFS. The firmware emulates FAT and commits the host's edits only on
eject (START STOP UNIT).

**How to apply:**
- Use plain espressif/tinyusb, never esp_tinyusb (it collides with arduino-esp32's USB*.cpp).
- The Kconfig symbol must keep no `depends on`. See ~/dev/kb/tooling/idf-component-manager-config-rule-needs-the-symbol-in-every-build.md.
- Bench checks still open:
  - Windows and Linux;
  - the idle-converter auto-restart after commit.
- Never enable the flag for fry/flat.

Related: [[project-idf6-dual-build]]
