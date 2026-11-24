---
title: ESP32-S3 vs classic ESP32
sidebar_position: 2
---

# ESP32-S3 vs classic ESP32

The firmware builds for the ESP32-S3 (default) and the classic ESP32 from the same source. The S3 is the primary
target. The classic ESP32 is kept for the original Fugu hardware.

## Quick start

To build only for the classic ESP32, you can use the default `build/` directory: `idf.py set-target esp32 && idf.py
build`. If your shell exports `IDF_TARGET` (for example `esp32s3`), unset it first; otherwise `set-target esp32` fails
with "not consistent with target … in the environment". `. ./idf-export.sh esp32` sources ESP-IDF with
`IDF_TARGET=esp32` exported; without the argument it exports `esp32s3`.

To keep both targets side by side, give the classic target its own build directory and its own `sdkconfig`. By
default, the generated `sdkconfig` lives in the project root and records the target, so two build directories that
share it would overwrite each other's target. The following commands build the S3 in `build/` and the classic ESP32
in `build/esp32/`:

```bash
idf.py set-target esp32s3 && idf.py build        # ESP32-S3: build/ + ./sdkconfig

# classic ESP32: build/esp32/ + build/esp32/sdkconfig
IDF_TARGET=esp32 idf.py -B build/esp32 -D SDKCONFIG=build/esp32/sdkconfig build
IDF_TARGET=esp32 idf.py -B build/esp32 -D SDKCONFIG=build/esp32/sdkconfig -p $ESPPORT flash
```

Pass the same `IDF_TARGET=esp32` prefix and `-B`/`-D SDKCONFIG=` pair to every classic-ESP32 command. The prefix
selects the target of a new build directory, so no `set-target` is needed, and it overrides an `IDF_TARGET` your shell
may export: ESP-IDF refuses both `-D IDF_TARGET=esp32` and an `esp32` build directory while the environment says
`esp32s3`.

Running `set-target` on a directory to switch it back and forth regenerates `sdkconfig` and forces a full rebuild.

On a blank chip, use `flash`, because `app-flash` writes no bootloader or partition table. `flash` also writes
`config/fugu1/fugu1_esp32`, which is a real ADS1015 board configuration rather than a mock.

:::warning
Keep panel and battery disconnected until the board is provisioned with the configuration of your hardware. Use
`app-flash` for later updates, which keeps the configuration.
:::

After flashing, provision a configuration whose `board.conf` says `mcu=esp32`, for example
`./provision.py fugu1/fugu1_esp32`.

## Differences

The following table lists where the two targets differ:

| Area | ESP32-S3 | classic ESP32 |
|---|---|---|
| sdkconfig | `sdkconfig.defaults` + `sdkconfig.defaults.esp32s3` | `sdkconfig.defaults` + `sdkconfig.defaults.esp32` (ESP-IDF layers the target file automatically) |
| IRAM | control loop placed in IRAM (`main/rt_iram.lf`); Wi-Fi IRAM optimizations off to pay for it | `sdkconfig.defaults.esp32` disables `CONFIG_ESP_WIFI_RX_IRAM_OPT` so Wi-Fi + BLE + the IRAM-safe ADC ISR fit into IRAM |
| Flash writes | flash auto-suspend, after a boot self-test passes | caches off during each erase/write, stalling the control loop |
| Gate driver default | MCPWM | LEDC |
| USB | USB-Serial-JTAG; optional USB config drive (`CONFIG_FUGU_WITH_USB_MSC`) | none on chip |
| Console | UART0 on GPIO43/44 plus the built-in USB-Serial-JTAG port while USB is connected | UART0 on GPIO1/3, usually through the board's USB-UART bridge |
| Bootloader | stock ESP-IDF bootloader ([esp-bootguard](../../development/bootguard.md) if present) | stock ESP-IDF bootloader |
| `idf.py flash` littlefs image | `config/lab/dry_mock` | `config/fugu1/fugu1_esp32` |
| `board.conf::mcu` | `esp32s3` | `esp32` |
| ADC1 channel → GPIO | S3 mapping | classic mapping (different pins for the same channel number) |
| Simulator configs | `config/lab/wokwi_mock`, `config/lab/vconv_mock` | `config/lab/wokwi_mock_esp32`, `config/lab/vconv_mock_esp32` |

Both targets share the feature flags in `main/Kconfig.projbuild`: BLE, networking, LEDC, and MCPWM build for both.
The exceptions are S3 only: the USB config drive (the build stops with an error on a classic ESP32),
`CONFIG_FUGU_GPIO_ISR_IRAM`, and RT-task heap-allocation counting (`CONFIG_FUGU_RT_ALLOC_COUNT`, on by default only on
the S3).

Both targets also share the rollback-enabled OTA scheme and the partition table for 4 MB flash: two 1.87 MB
app slots, a 128 KB littlefs configuration partition and a 64 KB coredump partition.

## Target details

### Target check at boot

The firmware compares `board.conf::mcu` with the target it was built for. On a mismatch, it logs this line and doesn't start the control loop:

```
board.conf expects MCU 'esp32s3', but target is 'esp32'
```

The console and network services stay up, so you can fix the file with
`set-config board.conf mcu esp32` and `restart`.

`provision.py` catches the same mistake earlier. With `IDF_TARGET` set, it refuses a configuration whose `mcu`
differs.

### sdkconfig overlay

ESP-IDF applies `sdkconfig.defaults.<target>` on top of `sdkconfig.defaults` for the selected target:

- `sdkconfig.defaults.esp32` disables the Wi-Fi RX IRAM optimization to free IRAM. Wi-Fi is pinned to core 0 and the
  control loop runs on core 1, so the Wi-Fi receive path running from flash doesn't affect the control loop.
- `sdkconfig.defaults.esp32s3` enables flash auto-suspend, so a flash erase no longer freezes the control loop, and
  moves the Wi-Fi IRAM optimizations to flash to pay for the control loop running from IRAM. The firmware uses
  auto-suspend only after a boot self-test of the flash chip passes; until then flash writes disable the caches as on
  the classic ESP32.

The top-level `CMakeLists.txt` then layers `sdkconfig.d/ble` for both targets. See
[Build Options](../getting-started/build-options.md#sdkconfig-layering).

### Bootloader (S3 only)

The build uses the stock ESP-IDF bootloader unless it finds the optional
[esp-bootguard](../../development/bootguard.md) crash-loop guard. Bootguard is a development aid, and the firmware
runs without it.

An app OTA leaves the old bootloader in place. A new bootloader reaches a device by serial flash
(`idf.py bootloader-flash` or `idf.py flash`) or by [bootloader update](../updating/ota-wifi.md#bootloader-update)
(`ota bootloader <url>`).

### Console

Both targets run the same console on UART0 at 115200 baud. On the S3, the firmware also serves the console on the
USB-Serial-JTAG port whenever a USB host is connected, so a bare S3 module needs no USB-UART bridge. With
`CONFIG_FUGU_WITH_USB_MSC` the USB port is a CDC console instead; see [USB config drive](../usb-drive.md).

:::note Known issue
On classic ESP32 builds, serial console *input* has been observed to fail: logs appear, but commands are not
echoed. If that happens, configure Wi-Fi by adding it to `wifi.conf` in the provisioned image and use
[telnet](../connecting.md), or use BLE.
:::

### ADC pins

For the `esp32adc1` backend, `sensor.conf::<chn>_ch` is an ADC1 channel number, not a GPIO. The same channel maps to
different pins on the two chips. For this reason, the original Fugu has separate `fugu1_esp32` and `fugu1_esp32s3`
configurations. For example, the NTC is `ntc_ch=7` on classic and `ntc_ch=6` on the S3 module. For the pin tables,
see [Internal ADC](internal-adc.md).

## Common scenarios

### No start after flashing

The usual cause is an S3 configuration provisioned on a classic ESP32, or the reverse. Check the boot log for the
`board.conf expects MCU` line.
