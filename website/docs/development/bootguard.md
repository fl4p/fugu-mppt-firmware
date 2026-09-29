---
title: esp-bootguard (crash-loop guard)
sidebar_position: 4.5
---

*this document is an LLM generated placeholder*

# esp-bootguard

[esp-bootguard](https://github.com/fl4p/esp-bootguard) is an optional second-stage bootloader for the ESP32-S3 that
stops a crash loop: after 3 consecutive crash resets it parks the chip in ROM download mode, so the USB port stays
enumerated and `idf.py flash` works without holding BOOT and replugging. It is a development aid; the firmware runs
the same with the stock ESP-IDF bootloader.

## Enable

The top-level `CMakeLists.txt` picks it up on `esp32s3` builds when it is present:

| Source | Behaviour |
|---|---|
| `ESP_BOOTGUARD_DIR` (environment or `idf.py -DESP_BOOTGUARD_DIR=…`) | Used; the build stops if the path has no `bootloader_components/` |
| `../../esp/esp-bootguard` relative to the firmware repo | Used if it exists |
| neither | Stock bootloader, logged as `esp-bootguard: not found, using the stock bootloader` |

```bash
git clone https://github.com/fl4p/esp-bootguard
export ESP_BOOTGUARD_DIR=$PWD/esp-bootguard
cd fugu-mppt-firmware
idf.py reconfigure                    # picks up the new component directories
idf.py -p $ESPPORT bootloader-flash app-flash
```

The retained-RTC options the guard needs (`CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC*`) are already in
`sdkconfig.defaults`. Its own options (crash limit, action on trip) are under **Boot guard** in
`idf.py menuconfig`.

## Behaviour in the firmware

- Crash resets (panic, watchdogs, failed image load) increment the count; power-on and a boot the app marks healthy
  clear it. `esp_restart()` leaves it unchanged.
- The firmware marks the boot healthy together with the OTA rollback confirm: after the uptime threshold and once the
  ADC sampler is producing samples.
- `bootinfo` on the console prints the guard state: `on`/`off`, crash count against the limit, trips, and the last
  ROM reset reason. `no record (stock bootloader?)` means the board runs a bootloader without the guard.

## Caveats

- The bootloader only reaches a board by serial flash. An OTA update replaces the app, not the bootloader.
- A board parked in download mode after a trip waits for esptool. Flash a fixed image, or leave download mode with
  `esptool.py -p $ESPPORT --after hard_reset chip_id`.
