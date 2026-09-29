---
title: Build Options
sidebar_position: 2
---

*this document is an LLM generated placeholder*

# Build Options

Compile-time features are Kconfig options under **"Fugu MPPT firmware"** in `idf.py menuconfig`. Runtime behaviour
(pins, sensors, limits, charger) is set in [configuration files](../../reference/config/index.md) instead and needs no rebuild.

## Quick start

```bash
idf.py menuconfig          # Fugu MPPT firmware -> toggle features
idf.py build
```

Non-interactively, for CI or variant builds, layer an sdkconfig fragment:

```bash
echo 'CONFIG_FUGU_WITH_BLE=n' > my.frag
SDKCONFIG_DEFAULTS="sdkconfig.defaults;my.frag" idf.py -B build-noble build
```

`etc/matrix_build.sh` builds the common variants this way.

## Feature flags

| Kconfig option | Default | What it does |
|---|:---:|---|
| `CONFIG_FUGU_WITH_NETW` | on | Wi-Fi, mDNS, MQTT, InfluxDB telemetry, HTTPS OTA, FTP, telnet. Off strips all of them (~700 KB); the UART/USB/BLE consoles remain. Layers `sdkconfig.no_netw` when off. |
| `CONFIG_FUGU_WITH_NETTOOLS` | off | `curl`, `ping`, `nslookup`, `tcpconnect`, `netstat` console commands. Needs `NETW`. |
| `CONFIG_FUGU_WITH_BLE` | on | NimBLE console (NUS) and BLE OTA push, ~250 KB. Layers `sdkconfig.ble`. |
| `CONFIG_FUGU_WITH_BLE_TELE` | off | Binary telemetry stream over a NUS notify characteristic. Needs `BLE`. |
| `CONFIG_FUGU_WITH_BLE_ADV` | off | Connectionless telemetry in BLE advertising data. Needs `BLE`. |
| `CONFIG_FUGU_WITH_LEDC` | on | LEDC gate driver, fan output and the `anaw` command. |
| `CONFIG_FUGU_WITH_MCPWM` | on\* | [MCPWM gate driver](../../internals/pwm-drivers.md): hardware dead-time, fault brake, glitch-free updates. With LEDC also on, `converter.conf::pwm_driver` picks the driver at runtime. |
| `CONFIG_FUGU_WITH_WSYNC` | on\* | Wired MCPWM clock sync between converters. Needs `MCPWM`. |
| `CONFIG_FUGU_WITH_BSYNC` | on | Beacon-sniffing MCPWM clock sync (`bsync` service). Needs `NETW` and `MCPWM`. |
| `CONFIG_FUGU_WITH_VCONV` | off | Replaces gate driver and ADC with a simulated converter (`src/sim/vconv.*`). Excludes `MCPWM`. |
| `CONFIG_FUGU_WITH_SPROFILER` | off | Semihosting sampling profiler; only useful with OpenOCD attached. |
| `CONFIG_FUGU_WITH_MEASURE_COIL` | off | On-device [coil inductance measurement](../../lab/coil-inductance.md). |
| `CONFIG_FUGU_INA226_MEASURED_RATE` | on | Report the INA226 sample rate measured at init instead of the datasheet value (some parts convert faster). |

\* The Kconfig default is off, but `sdkconfig.defaults` enables `MCPWM` and `WSYNC`, so a fresh build of this
repository has them on.

At least one of `LEDC` and `MCPWM` must be on, unless `VCONV` is.

:::note
The old `WITH_*` environment variables are rejected; the build stops with an error pointing to Kconfig.
Binary telemetry is not a build flag: it is `tele.conf::binary`.
:::

## Environment variables

| Variable | Effect |
|---|---|
| `RUN_TESTS=1` | Builds the Unity test runner (`test/main.cpp`) instead of the firmware, see [Testing](../../development/testing.md). |
| `MAIN_SRC=<file>` | Builds a single file with `setup()`/`loop()` instead of the firmware, e.g. `MAIN_SRC=../test/main_ads_rate.cpp`. |
| `FUGU_BAT_V=14.25\|28.5\|57` | Hardcodes the battery max voltage. Leave unset to read it from `charger.conf`. |
| `FUGU_DEVICE=<name>` | Device name recorded in the ELF archive on flash. |

## sdkconfig layering

`sdkconfig` is generated and not tracked. The top-level `CMakeLists.txt` layers:

1. `sdkconfig.defaults`
2. `sdkconfig.defaults.esp32` (classic ESP32 target only)
3. `sdkconfig.ble` when `CONFIG_FUGU_WITH_BLE=y`
4. `sdkconfig.no_netw` when `CONFIG_FUGU_WITH_NETW=n`

Delete `sdkconfig` to regenerate it if it looks wrong.

## Common scenarios

### Minimal BLE-only image

```ini title="noble.frag"
CONFIG_FUGU_WITH_NETW=n
```

### MCPWM converter without LEDC

```ini title="mcpwm.frag"
CONFIG_FUGU_WITH_MCPWM=y
CONFIG_FUGU_WITH_LEDC=n
```

The fan and `anaw` stop working without LEDC.

### Control-loop work without hardware

```ini title="vconv.frag"
CONFIG_FUGU_WITH_VCONV=y
CONFIG_FUGU_WITH_MCPWM=n
```

Provision `config/lab/vconv_mock`. On ESP32-S3 the MCPWM default cannot be overridden by a fragment alone;
pass a dedicated sdkconfig with `-DSDKCONFIG=sdkconfig.vconv_s3`.

:::danger
Never OTA a `VCONV` build to a real converter: it drives a simulation instead of the half-bridge and
the converter outputs 0 W.
:::
