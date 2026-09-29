---
title: ESP32-S3 vs classic ESP32
sidebar_position: 2
---

# ESP32-S3 vs classic ESP32

The firmware builds for the ESP32-S3 (default) and the classic ESP32 from the same source. The S3 is the primary
target; the classic ESP32 is kept for the original Fugu hardware.

## Quick start

Building only for classic ESP32 works in the default `build/` too: `idf.py set-target esp32 && idf.py build`. If your
shell exports `IDF_TARGET` (for example `esp32s3`), `set-target esp32` fails with "not consistent with target … in
the environment"; unset it first.

To keep both targets side by side, give the classic target its own build directory **and** its own `sdkconfig`. The
generated `sdkconfig` lives in the project root by default and records the target, so two build directories
sharing it would fight over it:

```bash
idf.py set-target esp32s3 && idf.py build        # ESP32-S3: build/ + ./sdkconfig

# classic ESP32: build-esp32/ + build-esp32/sdkconfig
idf.py -B build-esp32 -D SDKCONFIG=build-esp32/sdkconfig set-target esp32
idf.py -B build-esp32 -D SDKCONFIG=build-esp32/sdkconfig build
idf.py -B build-esp32 -D SDKCONFIG=build-esp32/sdkconfig -p $ESPPORT flash
```

On a blank chip use `flash`: `app-flash` writes no bootloader or partition table. `flash` also writes
`config/fugu1/fugu1_esp32`, a real ADS1015 board configuration, not a mock.

:::warning
Keep panel and battery disconnected until the board is provisioned with the configuration of your hardware. Use
`app-flash` for later updates, which keeps the configuration.
:::

Then provision a configuration whose `board.conf` says `mcu=esp32`, e.g. `./provision.py fugu1/fugu1_esp32`.

## Differences

| Area | ESP32-S3 | classic ESP32 |
|---|---|---|
| sdkconfig | `sdkconfig.defaults` | `sdkconfig.defaults` + `sdkconfig.defaults.esp32` (ESP-IDF layers the target file automatically) |
| IRAM | fits with the defaults | `sdkconfig.defaults.esp32` disables `CONFIG_ESP_WIFI_RX_IRAM_OPT` so Wi-Fi + BLE + the IRAM-safe ADC ISR fit into IRAM |
| Console | UART0 on GPIO43/44 plus the built-in USB-Serial-JTAG port while USB is connected | UART0 on GPIO1/3, usually through the board's USB-UART bridge |
| Bootloader | stock ESP-IDF bootloader ([esp-bootguard](../../development/bootguard.md) if present) | stock ESP-IDF bootloader |
| `idf.py flash` littlefs image | `config/lab/dry_mock` | `config/fugu1/fugu1_esp32` |
| `board.conf::mcu` | `esp32s3` | `esp32` |
| ADC1 channel → GPIO | S3 mapping | classic mapping (different pins for the same channel number) |
| Simulator configs | `config/lab/wokwi_mock`, `config/lab/vconv_mock` | `config/lab/wokwi_mock_esp32`, `config/lab/vconv_mock_esp32` |

Feature flags in `main/Kconfig.projbuild` are not target-specific: BLE, networking, LEDC and MCPWM build for both.
The rollback-enabled OTA scheme and the partition table are shared.

## Details

### Target check at boot

The firmware compares `board.conf::mcu` with the target it was built for. On a mismatch it logs

```
board.conf expects MCU 'esp32s3', but target is 'esp32'
```

and does not start the control loop. The console and network services stay up, so you can fix the file with
`set-config board.conf mcu esp32` and `restart`.

`provision.py` catches the same mistake earlier: with `IDF_TARGET` set, it refuses a configuration whose `mcu`
differs.

### sdkconfig overlay

`sdkconfig.defaults.esp32` is applied on top of `sdkconfig.defaults` only when `IDF_TARGET=esp32`. It disables the
Wi-Fi RX IRAM optimization to free IRAM. Wi-Fi is pinned to core 0 and the control loop runs on core 1, so the
Wi-Fi receive path running from flash does not affect the control loop.

The top-level `CMakeLists.txt` then layers `sdkconfig.ble` and `sdkconfig.no_netw` for both targets, see
[Build Options](../getting-started/build-options.md#sdkconfig-layering).

### Bootloader (S3 only)

The build uses the stock ESP-IDF bootloader unless the optional [esp-bootguard](../../development/bootguard.md)
crash-loop guard is found, a development aid that is not needed to run the firmware.

A bootloader only reaches a device by serial flash (`idf.py bootloader-flash` or `idf.py flash`); an OTA leaves the
old bootloader in place.

### Console

Both targets run the same console on UART0 at 115200 baud. On the S3 the firmware also serves the console on the
USB-Serial-JTAG port whenever a USB host is connected, so a bare S3 module needs no USB-UART bridge.

:::note Known issue
On classic ESP32 builds, serial console *input* has been observed not to work (logs appear, commands are not
echoed). If that happens, configure Wi-Fi by adding it to `wifi.conf` in the provisioned image and use
[telnet](../connecting.md), or use BLE.
:::

### ADC pins

`sensor.conf::<chn>_ch` for the `esp32adc1` backend is an ADC1 **channel**, not a GPIO. The same channel maps to
different pins on the two chips, which is why the original Fugu has separate `fugu1_esp32` and `fugu1_esp32s3`
configurations (for example, the NTC is `ntc_ch=7` on classic and `ntc_ch=6` on the S3 module). Pin tables are in
[Internal ADC](internal-adc.md).

## Common scenarios

### Build both targets side by side

Keep passing the same `-B` and `-D SDKCONFIG=` pair for every classic-ESP32 command. Running `set-target` on a
directory to switch it back and forth regenerates `sdkconfig` and forces a full rebuild.

### Firmware flashed, nothing starts

Check the boot log for the `board.conf expects MCU` line. Provisioning an S3 configuration on a classic ESP32 (or the
reverse) is the usual cause.
