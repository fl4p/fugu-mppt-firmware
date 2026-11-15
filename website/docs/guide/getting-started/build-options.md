---
title: Build Options
sidebar_position: 2
---

# Build Options

Compile-time features are Kconfig options under **"Fugu MPPT firmware"** in `idf.py menuconfig`. Runtime behaviour
(pins, sensors, limits, charger) is set in [configuration files](../../reference/config/index.md) instead and needs no rebuild.

## Quick start

To toggle features interactively, open menuconfig and then build:

```bash
idf.py menuconfig          # Fugu MPPT firmware -> toggle features
idf.py build
```

For CI or variant builds, layer an sdkconfig fragment instead of using menuconfig:

```bash
echo 'CONFIG_FUGU_WITH_BLE=n' > my.frag
SDKCONFIG_DEFAULTS="sdkconfig.defaults;my.frag" idf.py -B build-noble -D SDKCONFIG=build-noble/sdkconfig build
```

The top-level `CMakeLists.txt` also reads the live sdkconfig to decide fragment layering. That's the one given with
`-D SDKCONFIG=`, or else the root `sdkconfig`. If you pass `-D SDKCONFIG=build-noble/sdkconfig` on every command, a root
`sdkconfig` from another build doesn't leak into the variant.

`etc/matrix_build.sh` builds each variant in its own project root.

## Feature flags

The following table lists each feature option, its default, and what it does:

| Kconfig option | Default | What it does |
|---|:---:|---|
| `CONFIG_FUGU_WITH_NETW` | on | Wi-Fi, mDNS, MQTT, InfluxDB telemetry, OTA, FTP, telnet. Off strips all of them (~700 KB); the UART/USB/BLE consoles remain. Layers `sdkconfig.no_netw` when off. |
| `CONFIG_FUGU_WITH_NETTOOLS` | off | `curl`, `ping`, `nslookup`, `tcpconnect`, `netstat` console commands. Needs `NETW`; `https://` URLs also need `HTTPS`. |
| `CONFIG_FUGU_WITH_HTTPS` | off | TLS client: `https://` OTA and `curl`, `mqtts://` broker, verified against the mbedTLS CA bundle. Needs `NETW`; on adds ~72 KB flash (incl. the CA bundle). |
| `CONFIG_FUGU_WITH_SCOPE` | on | Raw-ADC [scope](../../reference/host-tools.md#scopepy) streamer on TCP port 24. Needs `NETW`; off saves ~7 KB flash. |
| `CONFIG_FUGU_WITH_FTP` | on | [FTP server](../../reference/config/ftp.md) for the config partition. Needs `NETW`; off saves ~23 KB flash. |
| `CONFIG_FUGU_WITH_HA` | on | Home Assistant MQTT discovery and power state. Needs `NETW`; off saves ~6.5 KB flash. |
| `CONFIG_FUGU_WITH_BLE` | on | NimBLE console (NUS) and BLE OTA push, ~250 KB. Layers `sdkconfig.ble`. |
| `CONFIG_FUGU_WITH_BLE_TELE` | off | Binary telemetry stream over a NUS notify characteristic. Needs `BLE`; turns on `TELE_BINARY`. |
| `CONFIG_FUGU_WITH_TELE_BINARY` | off | Binary telemetry wire and tamp compressor, used when [`tele.conf`](../../reference/config/tele.md) `binary=1`. Needs `NETW` or `BLE_TELE`; off saves ~8.5 KB flash. |
| `CONFIG_FUGU_WITH_BLE_ADV` | off | Connectionless telemetry in BLE advertising data. Needs `BLE`. |
| `FUGU_GATE_DRIVER` | MCPWM\*\* | Gate driver, one per build: `CONFIG_FUGU_GATE_LEDC`, `CONFIG_FUGU_WITH_MCPWM` ([MCPWM](../../internals/pwm-drivers.md): hardware dead-time, fault brake, glitch-free updates, live `pwm-freq`/`dt`) or `CONFIG_FUGU_WITH_VCONV` (simulated converter in place of gate driver and ADC, `src/sim/vconv.*`). |
| `CONFIG_FUGU_WITH_LEDC` | on | LEDC peripheral for the fan PWM and the `anaw` command. The LEDC gate driver turns it on. |
| `CONFIG_FUGU_WITH_WSYNC` | on\* | Wired MCPWM clock sync between converters. Needs `MCPWM`. |
| `CONFIG_FUGU_WITH_BSYNC` | off | Beacon-sniffing MCPWM clock sync (`bsync` service). Needs `NETW` and `MCPWM`. |
| `CONFIG_FUGU_WITH_SPROFILER` | off | Semihosting sampling profiler; only useful with OpenOCD attached. |
| `CONFIG_FUGU_WITH_MEASURE_COIL` | off | On-device [coil inductance measurement](../../lab/coil-inductance.md). |
| `CONFIG_FUGU_WITH_PSU` | on | PSU (constant-voltage) and PV-simulator output modes: `psu`/`pv` commands, `converter.conf` `mode=psu`/`pv`. A board config using either mode fails setup when off; off saves ~10 KB flash. |
| `CONFIG_FUGU_WITH_USB_MSC` | off | ESP32-S3 only: [USB config drive](../usb-drive.md) plus a CDC console on the USB port, instead of USB-Serial-JTAG. Excludes `WSYNC`. Layers `sdkconfig.usb_msc`. |
| `CONFIG_FUGU_WITH_INA226` | on | INA226 ADC backend (`sensor.conf` `*_adc=ina226`). A board config that selects it fails setup when off. |
| `CONFIG_FUGU_WITH_ADS` | on | ADS1015/ADS1115 ADC backend (`*_adc=ads1015`/`ads1115`). A board config that selects it fails setup when off. |
| `CONFIG_FUGU_INA226_MEASURED_RATE` | on | Report the INA226 sample rate measured at init instead of the datasheet value (some parts convert faster). Needs `INA226`. |
| `CONFIG_FUGU_RT_ALLOC_COUNT` | on (S3) | Count heap allocations and frees made by the control-loop task; `rt-stats` and `bootinfo` print them. Selects `CONFIG_HEAP_USE_HOOKS`. |
| `CONFIG_FUGU_RT_ALLOC_TRACE` | off | Also record the call stacks of the first 8 distinct control-loop heap operations. Layer `sdkconfig.rt_alloc_trace` to enable. |

\* The Kconfig default is off, but `sdkconfig.defaults` enables `WSYNC`, so a fresh MCPWM build of this
repository has it on.

\*\* LEDC on the classic ESP32 target.

Unless `VCONV` is on, at least one of `LEDC` and `MCPWM` must be on.

:::note
The build rejects the old `WITH_*` environment variables and stops with an error pointing to Kconfig.
Binary telemetry is a configuration setting, `tele.conf::binary`, rather than a build flag.
:::

## Environment variables

The build also reads these environment variables:

| Variable | Effect |
|---|---|
| `RUN_TESTS=1` | Builds the Unity test runner (`test/main.cpp`) instead of the firmware, see [Testing](../../development/testing.md). |
| `MAIN_SRC=<file>` | Builds a single file with `setup()`/`loop()` instead of the firmware, e.g. `MAIN_SRC=../test/main_ads_rate.cpp`. |
| `FUGU_DEVICE=<name>` | Device name recorded in the ELF archive on flash. |

## sdkconfig layering

`sdkconfig` is a generated file and isn't tracked. The top-level `CMakeLists.txt` layers these fragments in order:

1. `sdkconfig.defaults`
2. `sdkconfig.defaults.esp32` (classic ESP32 target only)
3. `sdkconfig.ble` when `CONFIG_FUGU_WITH_BLE=y`
4. `sdkconfig.no_netw` when `CONFIG_FUGU_WITH_NETW=n`
5. `sdkconfig.usb_msc` when `CONFIG_FUGU_WITH_USB_MSC=y`

If `sdkconfig` looks wrong, delete it to regenerate it.

## Common scenarios

### Minimal BLE-only image

To build an image without network features, turn off `NETW`:

```ini title="noble.frag"
CONFIG_FUGU_WITH_NETW=n
```

### MCPWM converter without LEDC

MCPWM is the default gate driver. To drop the LEDC peripheral as well, turn off `LEDC`:

```ini title="mcpwm.frag"
CONFIG_FUGU_WITH_MCPWM=y
CONFIG_FUGU_WITH_LEDC=n
```

Without LEDC, the `anaw` command isn't available. The fan output is a plain on/off GPIO and keeps working.

### Control-loop work without hardware

To run the control loop against a simulated converter, pick `VCONV` as the gate driver:

```ini title="vconv.frag"
CONFIG_FUGU_WITH_VCONV=y
```

On ESP32-S3 the repository ships this fragment as `sdkconfig.vconv_s3`. Build it in a separate build directory with
its own `sdkconfig`:

```bash
SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.vconv_s3" idf.py -B build-vconv -DSDKCONFIG=build-vconv/sdkconfig build
```

Provision `config/lab/vconv_mock`.

:::warning Never OTA a VCONV build to a real converter
A `VCONV` build replaces the gate driver with a simulation, so the half-bridge is never switched and the converter
stops charging. The sensors stay real, so the device looks healthy: it keeps sweeping, Vin sits at the panel's
open-circuit voltage, and the log repeats `Vr-sensor-fail`. Meanwhile the loads drain the battery. `etc/ota.py` warns
about VCONV images and refuses them in non-interactive runs.
:::
