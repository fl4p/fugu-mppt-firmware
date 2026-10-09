---
title: Libraries
sidebar_position: 2
---

# Libraries

This page lists the third-party code the firmware builds on and where the firmware uses it. Managed components are
declared in `main/idf_component.yml`. Vendored components are git submodules under `components/`.

## Framework and managed components

The firmware builds on ESP-IDF and the following managed components:

| Library | Version | Used for |
|---|---|---|
| [ESP-IDF](https://github.com/espressif/esp-idf) | ≥ 5.5 | FreeRTOS, drivers (MCPWM, LEDC, PCNT, UART, continuous ADC), lwIP, NimBLE, HTTP client, OTA, NVS, MQTT client (IDF 5.5) |
| [espressif/arduino-esp32](https://components.espressif.com/components/espressif/arduino-esp32) | ≥ 3.3.2, < 3.3.8 on IDF 5.5; ≥ 3.3.12 on IDF 6 | `Arduino.h`, `WiFi`, `Wire` (I²C), `ESPmDNS`, BLE, `USB` |
| [espressif/esp-dsp](https://components.espressif.com/components/espressif/esp-dsp) | * | Notch filter (`src/math/notch.h`) |
| [espressif/led_strip](https://components.espressif.com/components/espressif/led_strip) | ^3.0 | WS2812 status LED (`src/viz/led.h`) |
| [espressif/mdns](https://components.espressif.com/components/espressif/mdns) | * | mDNS hostname |
| [joltwallet/littlefs](https://components.espressif.com/components/joltwallet/littlefs) | ^1.20.2 | Configuration file system |
| [brianpugh/tamp](https://components.espressif.com/components/brianpugh/tamp) | ^1.6.0 | Compression for logs and telemetry |
| [fl4p/esp-ota-ble](https://github.com/fl4p/esp-ota-ble) (git) | pinned commit | OTA-over-BLE receiver (`ota-ble` command) |
| [espressif/mqtt](https://components.espressif.com/components/espressif/mqtt) | ^1.0.0 | MQTT client on IDF 6, which no longer bundles it |
| [espressif/tinyusb](https://components.espressif.com/components/espressif/tinyusb) | ≥ 0.19.0~3, < 0.22 | USB configuration drive, only with `CONFIG_FUGU_WITH_USB_MSC` |

:::note arduino-esp32 version cap
arduino-esp32 3.3.8 calls `ble_gap_read_local_irk()`, which the NimBLE bundled with ESP-IDF 5.5.1 does not have.
On IDF 5.5 the cap stays until IDF's NimBLE catches up. The IDF 6 build has no cap and requires 3.3.12 or later.
:::

arduino-esp32 declares further dependencies that the firmware doesn't use, such as RainMaker, Insights, Zigbee, Modbus,
esp-sr, and libsodium. An empty stub in `components/_idf_stubs/` replaces each of them, so the build skips it. Keep
the stub list in sync with `CONFIG_ARDUINO_SELECTIVE_*` in `sdkconfig.defaults`.

## Vendored components

The following libraries are vendored as git submodules under `components/`:

| Library | Upstream | Used for |
|---|---|---|
| Adafruit_ADS1X15 + Adafruit_BusIO | [adafruit](https://github.com/adafruit/Adafruit_ADS1X15) | ADS1015/1115 backend (`src/adc/ads.h`) |
| INA226_WE | [fl4p fork](https://github.com/fl4p/INA226_WE) | INA226 backend |
| Arduino-LiquidCrystal-I2C | [fdebrabander](https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library) | HD44780 LCD over I²C |
| ESPTelnet | [LennartHennigs](https://github.com/LennartHennigs/ESPTelnet) | Telnet console |
| SimpleFTPServer | [fl4p fork](https://github.com/fl4p/SimpleFTPServer) | FTP access to the configuration |
| SimpleCLI | [SpacehuhnTech](https://github.com/SpacehuhnTech/SimpleCLI) | Console command parser (`src/cli.cpp`) |
| esp32-semihosting-profiler | [fl4p](https://github.com/fl4p/esp32-semihosting-profiler) | Sampling profiler (`CONFIG_FUGU_WITH_SPROFILER`) |

If you cloned the repository without `--recursive`, fetch the submodules:

```bash
git submodule update --init --recursive
```

## Host-side tools

The host-side tools live in their own submodules:

| Submodule | Repository | Contents |
|---|---|---|
| `etc/fugu` | [fl4p/fugu-py](https://github.com/fl4p/fugu-py) | Console transports (serial, TCP, BLE, MQTT) used by `etc/fugu_console.py` and `etc/ota.py` |
| `etc/idf-devtools` | [fl4p/idf-devtools](https://github.com/fl4p/idf-devtools) | ELF archive, incremental flashing, provisioning, NVS dump, symbol lookup |
| `etc/adcscope` | [fl4p/adcscope](https://github.com/fl4p/adcscope) | Soft oscilloscope for raw ADC streams |
| `test/host-stub/arduino-shim` | [fl4p/arduino-host-shim](https://github.com/fl4p/arduino-host-shim) | Arduino API stubs for host-side tests |
