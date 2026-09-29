---
title: Repo layout
sidebar_position: 1
---

*this document is an LLM generated placeholder*

# Repo layout

Where things live in the repository, which parts are git submodules, and which sibling repositories the build
expects.

## Quick start

```bash
git clone --recursive https://github.com/fl4p/fugu-mppt-firmware
cd fugu-mppt-firmware
git submodule update --init --recursive   # after a pull, or if cloned without --recursive
```

No sibling checkouts are required; local checkouts of esp-ota-ble / esp-bootguard are optional overrides, see
[External repositories](#external-repositories).

## Top-level folders

| Path | Contents |
|---|---|
| `src/` | Firmware sources: control loop (`mppt.*`, `tracker.h`, `pd_control.h`, `buck.h`, `charger.h`), `adc/` sensor backends, `pwm/` gate drivers, `math/` filters, `tele/` services and telemetry, `sync/` clock sync, `sim/` virtual converter, `selftest/`, `cli.cpp` console commands |
| `main/` | ESP-IDF component glue: `CMakeLists.txt` (source selection, compile options), `Kconfig.projbuild` (feature flags), `idf_component.yml` (managed components), `linker.lf` |
| `components/` | Vendored libraries (mostly submodules) and `_idf_stubs/`, empty stand-ins for unused arduino-esp32 dependencies |
| `config/` | littlefs configuration images per board (`fmetal/`, `fugu1/`, `psu_12v/`, `solar-boost/`, …) and [lab profiles](../lab/config-profiles.md) under `lab/` |
| `etc/` | Host tools: console client, OTA, provisioning helpers, e2e tests, config editor, measurement scripts |
| `test/` | Unity on-target tests (`test_*.cpp`, `main.cpp`), host C++ tests (`host-stub/`), host Python tests (`host_py/`) |
| `doc/` | Working notes, specs and reviews that are not part of this site |
| `plans/` | Implementation plans |
| `website/` | This documentation site (Docusaurus); pages are in `website/docs/` |

Root files: `CMakeLists.txt` (project, sdkconfig layering, littlefs image), `sdkconfig.defaults` and its fragments,
`partitions.csv`, `provision.py`, `ota.sh`, `flash.sh`, `idf_ext.py` (ELF archiving on flash), `wokwi.toml` and
`diagram.json` (Wokwi simulator).

## Tools in `etc/`

| Path | Purpose |
|---|---|
| `etc/fugu_console.py` | Console client over serial, telnet, BLE, BLE proxy or MQTT |
| `etc/ota.py`, `etc/ota_ble.py` | Firmware updates over Wi-Fi and BLE |
| `etc/e2e-test/` | End-to-end test suite, see [Testing](testing.md) |
| `etc/config-tool/` | `conf-editor.html` single-page config editor, `conf-tool.py` |
| `etc/scope.py` | Launcher for the adcscope client, see [Measurements](../lab/measurements.md#adc-noise) |
| `etc/measure_coil.py`, `etc/pico_*.py`, `etc/mcpwm_gate_verify.py` | Bench measurements |
| `etc/matrix_build.sh` | Build the common feature/target variants, see [Build](build.md) |
| `etc/filter-studies/` | Filter-design studies |
| `etc/patches/` | Patches for upstream code |

## Submodules

| Path | Repository | Purpose |
|---|---|---|
| `etc/fugu` | [fl4p/fugu-py](https://github.com/fl4p/fugu-py) | Python console transports (serial, socket, BLE, MQTT) and the `Console` line protocol used by the host tools |
| `etc/idf-devtools` | [fl4p/idf-devtools](https://github.com/fl4p/idf-devtools) | Generic ESP-IDF tools: ELF archive for coredump decoding, `provision.py`, incremental flashing, NVS dump, symbol lookup |
| `etc/adcscope` | [fl4p/adcscope](https://github.com/fl4p/adcscope) | Soft oscilloscope client for the `scope` stream |
| `test/host-stub/arduino-shim` | [fl4p/arduino-host-shim](https://github.com/fl4p/arduino-host-shim) | Arduino API shim for host tests |
| `components/*` | various | ADS1x15, BusIO, INA226, LiquidCrystal I²C, SimpleCLI, SimpleFTPServer, ESPTelnet, ESPAsyncWebServer, semihosting profiler |

The root `provision.py` is a thin wrapper that execs `etc/idf-devtools/provision.py`, so provisioning needs that
submodule. The component libraries and their use are listed in [Libraries](libraries.md).

```bash
git submodule update --init --recursive            # the commits this repo pins
git submodule update --init etc/idf-devtools etc/adcscope   # only the host tools
```

## External repositories

| Repository | Default location | Override | Provides |
|---|---|---|---|
| [fl4p/esp-ota-ble](https://github.com/fl4p/esp-ota-ble) | fetched into `managed_components/` (pinned in `main/idf_component.yml`); `../esp-ota-ble` overrides it if present | `ESP_OTA_BLE_DIR` | OTA-over-BLE receiver component and host module |
| [fl4p/esp-bootguard](https://github.com/fl4p/esp-bootguard) | `../../esp/esp-bootguard` | `ESP_BOOTGUARD_DIR` | Optional crash-loop guard bootloader (ESP32-S3), see [esp-bootguard](bootguard.md) |

Set the override as an environment variable or pass it as `-D<NAME>=<path>` to `idf.py`.
