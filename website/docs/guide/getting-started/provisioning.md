---
title: Provisioning
sidebar_position: 3
---

*this document is an LLM generated placeholder*

# Provisioning

Board-specific settings live as `.conf` files on the `littlefs` partition under `/littlefs/conf/`.
Provisioning writes a board's configuration image to that partition. The firmware image stays untouched.

## Quick start

```bash
export ESPPORT=/dev/cu.usbmodem1101   # Windows: set ESPPORT=COM3
./provision.py fmetal                 # board name under config/
./provision.py config/lab/dry_mock    # or a directory containing conf/
```

`provision.py` builds a littlefs image with `littlefs-python` (`pip install littlefs-python`) and writes it with
ESP-IDF's `parttool.py`, so the IDF environment must be exported.

## Board configurations

| Folder | Hardware |
|---|---|
| `config/fmetal` | [Fugu2](https://github.com/fl4p/Fugu2), the standard configuration |
| `config/fugu1/fugu1_esp32` | Original Fugu with ADS1015 ADC |
| `config/fugu1/fugu_int_adc` | Original Fugu using the [internal ADC](../hardware/internal-adc.md) |
| `config/psu_12v` | 12 V power supply using forced PWM |
| `config/solar-boost` | Boost topology |
| `config/lab/dry_mock` | Mock ADC with sinusoidal readings, for dev boards without power stage |
| `config/lab/dry_int` | Internal ADC, dry testing |
| `config/lab/vconv_mock` | Pairs with `CONFIG_FUGU_WITH_VCONV=y` (simulated converter) |
| `config/lab/wokwi_mock` | [Wokwi](https://wokwi.com) ESP32-S3 simulator (`wokwi_mock_esp32` for classic ESP32) |

## Other ways to change the configuration

| Method | Use when |
|---|---|
| `set-config <file>.conf <key> <value>` on the [console](../../reference/console.md) | Changing single values on a running device |
| `etc/config-tool/conf-editor.html` | Editing all files in a browser over serial or BLE |
| FTP (Wi-Fi up; 1 connection, passive mode off) | Copying whole files |
| `idf.py flash` | Flashing firmware and config together, using the board set in `CMakeLists.txt` |

The top-level `CMakeLists.txt` selects the image that `idf.py flash` writes through `FUGU_LITTLEFS_SRC`:
`config/lab/dry_mock` on ESP32-S3 and `config/fugu1/fugu1_esp32` on classic ESP32.

```cmake
if (IDF_TARGET STREQUAL "esp32")
    set(FUGU_LITTLEFS_SRC config/fugu1/fugu1_esp32)
else ()
    set(FUGU_LITTLEFS_SRC config/lab/dry_mock)
endif ()
```

:::warning
`idf.py flash` replaces the device's configuration with that image. Use `idf.py app-flash` to update firmware
only.
:::

## Adding your own board

1. Copy the closest folder under `config/`, e.g. `cp -r config/fmetal config/myboard`.
2. Adjust pins in `board.conf`, divider ratios and channels in `sensor.conf`, and limits in `limits.conf`.
   Every key is described in the [configuration reference](../../reference/config/index.md).
3. Start with a mock or dry configuration to check pins and sensor readings before enabling the power stage.
4. `./provision.py myboard`.
