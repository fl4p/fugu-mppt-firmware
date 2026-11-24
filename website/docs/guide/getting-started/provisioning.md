---
title: Provisioning
sidebar_position: 3
---

# Provisioning

Provisioning writes a board's configuration image to the `littlefs` partition and leaves the firmware image untouched.
Board-specific settings live as `.conf` files on that partition under `/littlefs/conf/`.

## Quick start

`provision.py` needs `littlefs-python`. Install it once with `python -m pip install littlefs-python` in the exported
ESP-IDF shell.

To provision a device, set the serial port and pass a board name or a directory to `provision.py`:

```bash
. ./idf-export.sh                     # ESP-IDF environment; sets IDF_TARGET and picks the first USB port
export ESPPORT=/dev/cu.usbmodem1101   # only to choose another port; Windows: set ESPPORT=COM3
./provision.py fmetal                 # folder under config/, e.g. fugu1/fugu1_esp32s3
./provision.py path/to/myboard        # or a directory containing conf/
```

`provision.py` builds a 128 KB littlefs image of the folder with `littlefs-python` and writes it to the `littlefs`
partition with ESP-IDF's `parttool.py`, so the IDF environment must be exported. It stops if `ESPPORT` is unset.

When `IDF_TARGET` is set, `provision.py` refuses a configuration whose `board.conf` `mcu` differs from it.
`idf-export.sh` exports `IDF_TARGET=esp32s3` unless told otherwise, so provision a classic-ESP32 configuration from a
shell sourced with `. ./idf-export.sh esp32`, or prefix the command: `IDF_TARGET=esp32 ./provision.py fugu1/fugu1_esp32`.

## Board configurations

Each folder under `config/` holds the configuration for one kind of hardware:

| Folder | Hardware |
|---|---|
| `config/fmetal` | [Fugu2](https://github.com/fl4p/Fugu2), the standard configuration |
| `config/fugu1/fugu1_esp32` | Original Fugu with ADS1015 ADC, classic ESP32 |
| `config/fugu1/fugu1_esp32s3` | Original Fugu with ADS1015 ADC, ESP32-S3 |
| `config/fugu1/fugu_int_adc` | Original Fugu using the [internal ADC](../hardware/internal-adc.md) |
| `config/psu_12v` | 12 V power supply using forced PWM |
| `config/solar-boost` | Boost topology |
| `config/psu/boost80V` | Boost in PSU mode (constant output voltage) |
| `config/lab/dry_mock` | Mock ADC with sinusoidal readings, for dev boards without power stage |
| `config/lab/dry_int` | Internal ADC, dry testing |
| `config/lab/vconv_mock` | Pairs with `CONFIG_FUGU_WITH_VCONV=y` (simulated converter) |
| `config/lab/wokwi_mock` | [Wokwi](https://wokwi.com) ESP32-S3 simulator (`wokwi_mock_esp32` for classic ESP32) |

`config/lab/` holds further bench profiles; see [Lab config profiles](../../lab/config-profiles.md).

## Other configuration methods

Besides provisioning, you can change the configuration in these ways:

| Method | Use when |
|---|---|
| `set-config <file>.conf <key> <value>` on the [console](../../reference/console.md) | Changing single values on a running device; most files are read at boot, so `restart` afterwards ([when changes take effect](../../reference/config/index.md#when-changes-take-effect)) |
| [Config editor](pathname:///tools/conf-editor.html) ([docs](../../reference/config-editor.md)) | Editing all files in a browser over USB serial, BLE or MQTT |
| FTP (Wi-Fi up; 2 sessions, passive or active mode) | Copying whole files |
| [USB config drive](../usb-drive.md) (`CONFIG_FUGU_WITH_USB_MSC` builds, ESP32-S3) | Editing the files from a computer over USB |
| HTTP (`CONFIG_FUGU_WITH_HTTP_CONF` builds, see [`http.conf`](../../reference/config/http.md)) | Reading or replacing single files over Wi-Fi |
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

## Custom boards

1. Copy the closest folder under `config/`, for example `cp -r config/fmetal config/myboard`.
2. Adjust pins in `board.conf`, divider ratios and channels in `sensor.conf`, limits in `limits.conf`, the
   topology in `converter.conf` and the inductance in `coil.conf`.
   Every key is described in the [configuration reference](../../reference/config/index.md).
3. Start with a mock or dry configuration to check pins and sensor readings before enabling the power stage.
4. Run `./provision.py myboard`.
