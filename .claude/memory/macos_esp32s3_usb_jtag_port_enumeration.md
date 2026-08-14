---
name: macOS ESP32-S3 USB-JTAG serial port enumeration instability
description: ESP32-S3 USB-JTAG/serial (VID 0x303a) /dev/cu.usbmodem* port can disappear and re-enumerate with a different suffix on macOS; idf-export.sh auto-detection fails; workaround
created: 2026-08-11T14:08:32.634Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: reference
  originSessionId: ses_00ed9f077ffergOIVA654jybbL
---

On macOS, the ESP32-S3's built-in USB-JTAG/serial debug unit (VID 0x303a, PID 0x1001) can have its `/dev/cu.usbmodem*` port disappear and re-enumerate with a **different suffix** (e.g. usbmodem11301 → usbmodem1101) within seconds of plugging in. `idf-export.sh` uses a zsh glob (`/dev/cu.usbmodem*`) that fails with "no matches found" when the port hasn't settled yet, leaving `ESPPORT` empty.

**How to find the port when auto-detection fails:**
1. Just wait 2–3 seconds and retry `ls /dev/cu.usbmodem*` — the port often appears after settling.
2. If `ls` shows nothing but `system_profiler SPUSBDataType` shows an Espressif device, the ACM driver may not have created the node yet. Check `ioreg -l -r -c AppleUSBACMData` and look for `IOCalloutDevice` — that's the `/dev/cu.*` path.
3. If esptool reports "port is busy or doesn't exist" on a port you just confirmed exists, the port disappeared again — retry `ls` and use whatever name it re-enumerated as.

**How to apply:** When `idf-export.sh` leaves `ESPPORT` empty on a device you know is plugged in, don't give up — manually detect the port and export `ESPPORT=/dev/cu.usbmodemXXXX` before running `idf.py`.
