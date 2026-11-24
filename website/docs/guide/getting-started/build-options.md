---
title: Build Options
sidebar_position: 2
---

# Build Options

Compile-time features are Kconfig options under **"Fugu MPPT firmware"** in `idf.py menuconfig`. Runtime behaviour
(pins, sensors, limits, charger) is set in [configuration files](../../reference/config/index.md) instead and needs no rebuild.

## Quick start

Source ESP-IDF 6 with `. ./idf-export.sh` first (it also sets `IDF_TARGET`, default `esp32s3`). To toggle features
interactively, open menuconfig and then build:

```bash
idf.py menuconfig          # Fugu MPPT firmware -> toggle features
idf.py build
```

For CI or variant builds, layer an sdkconfig fragment instead of using menuconfig:

```bash
echo 'CONFIG_FUGU_WITH_BLE=n' > my.frag
SDKCONFIG_DEFAULTS="sdkconfig.defaults;my.frag" idf.py -B build/noble -D SDKCONFIG=build/noble/sdkconfig build
```

The top-level `CMakeLists.txt` also reads the live sdkconfig to decide fragment layering. That's the one given with
`-D SDKCONFIG=`, or else the root `sdkconfig`. If you pass `-D SDKCONFIG=build/noble/sdkconfig` on every command, a root
`sdkconfig` from another build doesn't leak into the variant.

`etc/matrix_build.sh` builds each variant in its own project root.

## Feature flags

The following table lists each feature option, its default, and what it does:

| Kconfig option | Default | What it does |
|---|:---:|---|
| `CONFIG_FUGU_WITH_NETW` | on | Wi-Fi, mDNS, MQTT, InfluxDB telemetry, OTA, FTP, telnet. Off strips all of them (~700 KB); the UART/USB/BLE consoles remain. |
| `CONFIG_FUGU_WITH_NETTOOLS` | off | `curl`, `ping`, `nslookup`, `tcpconnect`, `netstat` console commands. Needs `NETW`; `https://` URLs also need `HTTPS`. |
| `CONFIG_FUGU_WITH_HTTPS` | off | TLS client: `https://` OTA and `curl`, `mqtts://` broker, verified against the mbedTLS CA bundle (MQTT: or a private CA in [`mqtt.conf`](../../reference/config/mqtt.md#tls) `ca_file`). Needs `NETW`; on adds ~77 KB flash (incl. the CA bundle). |
| `CONFIG_FUGU_WITH_TELNET` | on | [Telnet console](../../reference/config/telnet.md) on TCP port 23, without authentication; runs only with `telnet.conf` `enabled=1`. Needs `NETW`. |
| `CONFIG_FUGU_WITH_TLS_CONSOLE` | off | [TLS-PSK console](../../reference/config/tlscon.md) (`tlscon` service). Needs `NETW`; selects the mbedTLS PSK key exchange and layers `sdkconfig.d/tls_console` (TLS server role). On adds ~14 KB flash, ~13 KB on top of `HTTPS`. |
| `CONFIG_FUGU_WITH_SCOPE` | on | Raw-ADC [scope](../../reference/host-tools.md#scopepy) streamer on TCP port 24. Needs `NETW`; off saves ~7 KB flash. |
| `CONFIG_FUGU_WITH_FTP` | on | [FTP server](../../reference/config/ftp.md) for the config partition. Needs `NETW`; off saves ~15 KB flash. |
| `CONFIG_FUGU_WITH_HTTP_CONF` | off | [HTTP file endpoint](../../reference/config/http.md) (`GET`/`PUT`/`DELETE /conf/<name>`) for the config partition, no extra task. Needs `NETW`; on adds ~9 KB flash. |
| `CONFIG_FUGU_WITH_HA` | on | Home Assistant MQTT discovery and power state. Needs `NETW`; off saves ~6.5 KB flash. |
| `CONFIG_FUGU_WITH_TELE_MQTT` | off | MQTT telemetry sink: `tele-mqtt <s>` leases a binary telemetry stream on the broker. Needs `NETW`; turns on `TELE_BINARY`. |
| `CONFIG_FUGU_WITH_BLE` | on | NimBLE console (NUS) and BLE OTA push, ~250 KB. Layers `sdkconfig.d/ble`. |
| `CONFIG_FUGU_WITH_BLE_TELE` | off | Binary telemetry stream over a NUS notify characteristic. Needs `BLE`; turns on `TELE_BINARY`. |
| `CONFIG_FUGU_WITH_TELE_BINARY` | off | Binary telemetry wire and tamp compressor, used when [`tele.conf`](../../reference/config/tele.md) `binary=1`. Needs `NETW` or `BLE_TELE`; `BLE_TELE` and `TELE_MQTT` turn it on. Off saves ~8.5 KB flash. |
| `CONFIG_FUGU_WITH_BLE_ADV` | off | Connectionless telemetry in BLE advertising data. Needs `BLE`. |
| `CONFIG_FUGU_WITH_IMPROV` | on | [Improv Wi-Fi](../connecting.md#improv-over-ble) provisioning over BLE. Needs `BLE` and `NETW`; off saves ~5 KB flash. |
| `FUGU_GATE_DRIVER` | MCPWM\*\* | Gate driver, one per build: `CONFIG_FUGU_GATE_LEDC`, `CONFIG_FUGU_WITH_MCPWM` ([MCPWM](../../internals/pwm-drivers.md): hardware dead-time, fault brake, glitch-free updates, live `pwm-freq`/`dt`) or `CONFIG_FUGU_WITH_VCONV` (simulated converter in place of gate driver and ADC, `src/sim/vconv.*`). |
| `CONFIG_FUGU_WITH_LEDC` | on | LEDC peripheral for the `anaw` command (the fan output is an on/off GPIO either way). The LEDC gate driver turns it on. |
| `CONFIG_FUGU_WITH_WSYNC` | on\* | Wired MCPWM clock sync between converters. Needs `MCPWM`. |
| `CONFIG_FUGU_WITH_BSYNC` | off | Beacon-sniffing MCPWM clock sync (`bsync` service). Needs `NETW` and `MCPWM`. |
| `CONFIG_FUGU_WITH_SPROFILER` | off | Semihosting sampling profiler; only useful with OpenOCD attached. Off excludes its component. |
| `CONFIG_FUGU_WITH_RTPROF` | off | Core-1 call-stack sampler (`rtprof` command), a 2 kHz timer ISR counting return addresses. `CONFIG_FUGU_RTPROF_SLOTS` (default 4096) sizes its table. |
| `CONFIG_FUGU_WITH_EXTPORT` | off | `extport` service: [VE.Direct text output](../../reference/config/extport.md) on a UART pin pair (about 10 KB of flash, mostly the UART driver). |
| `CONFIG_FUGU_WITH_MEASURE_COIL` | off | On-device [coil inductance measurement](../../lab/coil-inductance.md) (`measure-coil`). It starts only from a converter stopped with `dc 0`; on in the `sdkconfig.d/fleet` build. |
| `CONFIG_FUGU_WITH_SCRIPTS` | off | On-device [console scripts](../../reference/console.md#scripts) (`run`, `sleep`, `scripts`, `script-set`). |
| `CONFIG_FUGU_WITH_PSU` | on | PSU (constant-voltage) and PV emulator output modes: `psu`/`pv` commands, `converter.conf` `mode=psu`/`pv`. A board config using either mode fails setup when off; off saves ~10 KB flash. |
| `CONFIG_FUGU_WITH_PWR_KNOBS` | off | `pwr` console command for power experiments (CPU frequency, power management, Wi-Fi and BLE radio settings); changes are not persisted. `sdkconfig.d/pwr` turns it on, `sdkconfig.d/pm` adds power-management and light-sleep support. |
| `CONFIG_FUGU_WITH_USB_MSC` | off | ESP32-S3 only: [USB config drive](../usb-drive.md) plus a CDC console on the USB port, instead of USB-Serial-JTAG. Excludes `WSYNC`, which `sdkconfig.defaults` turns on, so also set `CONFIG_FUGU_WITH_WSYNC=n`; the build stops otherwise. Layers `sdkconfig.d/usb_msc`. |
| `CONFIG_FUGU_WITH_INA226` | on | INA226 ADC backend (`sensor.conf` `*_adc=ina226`). A board config that selects it fails setup when off. |
| `CONFIG_FUGU_WITH_ADS` | on | ADS1015/ADS1115 ADC backend (`*_adc=ads1015`/`ads1115`). A board config that selects it fails setup when off. |
| `CONFIG_FUGU_GPIO_ISR_IRAM` | off | ESP32-S3 only: GPIO ISR service in IRAM, so the INA226/ADS alert interrupts keep running during flash writes. Not yet tested on that path; no effect with `FUGU_RT_FLASH=1`. |
| `CONFIG_FUGU_INA226_MEASURED_RATE` | on | Report the INA226 sample rate measured at init instead of the datasheet value (some parts convert faster). Needs `INA226`. |
| `CONFIG_FUGU_RT_ALLOC_COUNT` | on (S3) | Count heap allocations and frees made by the control-loop task; `rt-stats` and `bootinfo` print them. Selects `CONFIG_HEAP_USE_HOOKS`. |
| `CONFIG_FUGU_RT_ALLOC_TRACE` | off | Also record the call stacks of the first 8 distinct control-loop heap operations. Layer `sdkconfig.d/rt_alloc_trace` to enable. |

\* The Kconfig default is off, but `sdkconfig.defaults` enables `WSYNC`, so a fresh MCPWM build of this
repository has it on.

\*\* LEDC on the classic ESP32 target.

The gate-driver choice compiles exactly one driver. `LEDC` is independent of it, except that the LEDC gate driver
turns it on.

:::note
The build rejects the old `WITH_*` environment variables and stops with an error pointing to Kconfig.
Binary telemetry is a configuration setting, `tele.conf::binary`, rather than a build flag.
:::

## TLS heap

Each TLS connection (an `mqtts://` broker, an `https://` OTA download, a `tlscon` session) allocates its record
buffers from the internal heap. With the default static buffers (16 KB in, 4 KB out), an `mqtts://` connection costs
far more heap than plain MQTT. The figures below were measured on an ESP32-S3 at steady state, as internal heap
free / minimum ever free / largest free block, in bytes:

| mbedTLS buffers                                   | plain MQTT             | `mqtts://`             |
|---------------------------------------------------|------------------------|------------------------|
| static, 16 KB in / 4 KB out (default)             | 40 300 / 37 500 / 29 700 | 29 200 / 19 700 / 8 200  |
| static, 4 KB in / 2 KB out                        | 40 400 / 36 100 / 29 700 | 28 600 / 20 600 / 13 800 |
| `CONFIG_MBEDTLS_DYNAMIC_BUFFER=y`                 | 40 500 / 33 100 / 29 700 | 34 600 / 24 700 / 22 500 |

Dynamic buffers give the smallest steady-state cost, about 6 KB free and 8 KB minimum. Smaller static buffers help
little, because the handshake still needs room for the server's full certificate chain. A broker that fails
verification (wrong `ca_file`) makes the client retry the handshake every few seconds, and each attempt drives the
minimum free heap down to about 9 KB.

An open `tlscon` session costs about 2 KB with dynamic buffers and about 24.6 KB with static buffers.

### Static or dynamic buffers

Static buffers are allocated once per connection and held until it closes. Dynamic buffers are allocated per record
and released after it, so an idle connection holds almost nothing. The cost is fragmentation: every incoming record
needs one contiguous block of about 16.4 KB (`Dynamic Impl: alloc(16749 bytes) failed` when the largest free block is
smaller).

`CONFIG_MBEDTLS_DYNAMIC_BUFFER=y` is the recommended setting for `HTTPS` builds that keep a TLS connection open (an
`mqtts://` broker, `tlscon`). With BLE released for the download, an `https://` OTA works with either setting.

### `https://` OTA

The `ota <url>` command shuts BLE down before the download, which returns about 65 KB
(see [OTA over Wi-Fi](../updating/ota-wifi.md#manual-update)). Measured on an ESP32-S3 (`HTTPS` and `tlscon`
compiled in, plain MQTT connected, a 1.8 MB image at about 63 KB/s), internal heap in bytes:

| mbedTLS buffers | before the command, free / largest | lowest free, internal / DMA | smallest largest block | result |
|---|---|---|---|---|
| static | 51 700 / 31 700 | 41 100–44 200 / 33 400–36 500 | 31 700 | 2 of 2 |
| dynamic | 51 600 / 31 700 | 41 100–45 000 / 33 400–37 300 | 31 700 | 2 of 2 |

"Lowest free" is the minimum from the `ota` command to the end of the download, including the TLS handshake.

Firmware that kept BLE running during the download had only about 40 KB free and a 29 KB largest block. There, an
`https://` OTA with dynamic buffers failed every time on the 16.4 KB record buffer, and with static buffers it
succeeded only with the MQTT service stopped; with MQTT running, certificate verification ran out of memory. The same
limits still apply when BLE is kept: `ota bootloader <url>` does not release it, and `ota <url>` keeps it when a
connected BLE client does not disconnect within 5 s.

The download verifies the server against the built-in CA bundle and checks its host name. `mqtt.conf` `ca_file`
applies to the broker only, so a server with a certificate from a private CA cannot serve OTA images. The build
disables the P-384 curve (`CONFIG_MBEDTLS_ECP_DP_SECP384R1_ENABLED=n`), which the certificate chain of `github.com`
(and so of release downloads) needs: such URLs fail with `-0x3A00`. `raw.githubusercontent.com` works.

## Environment variables

The build also reads these environment variables:

| Variable | Effect |
|---|---|
| `RUN_TESTS=1` | Builds the Unity test runner (`test/main.cpp`) instead of the firmware, see [Testing](../../development/testing.md). |
| `MAIN_SRC=<file>` | Builds a single file with `setup()`/`loop()` instead of the firmware, e.g. `MAIN_SRC=../test/main_ads_rate.cpp`. |
| `FUGU_DEVICE=<name>` | Device name recorded in the ELF archive on flash. |
| `TEST_ADC_HW=1` | With `RUN_TESTS=1`, runs only the internal-ADC and INA226 tests and skips the tests that drive gate pins. |
| `FUGU_RT_FLASH=1` | ESP32-S3: keeps the control-loop code in flash instead of IRAM, for profiling with `rtprof`. |
| `FUGU_FORCE_BFLOW_OFF=1` | Keeps the backflow switch open, so no battery current can enter the converter (bench safety). |
| `ESP_OTA_BLE_DIR=<dir>` | Builds a local checkout of the OTA-over-BLE component instead of the pinned git version. |
| `ESP_BOOTGUARD_DIR=<dir>` | ESP32-S3: builds the optional esp-bootguard bootloader from this checkout. |

## sdkconfig layering

`sdkconfig` is a generated file and isn't tracked. The top-level `CMakeLists.txt` layers these fragments in order:

1. `sdkconfig.defaults`
2. `sdkconfig.defaults.<target>` (`.esp32` or `.esp32s3`, added by ESP-IDF)
3. `sdkconfig.d/ble` when `CONFIG_FUGU_WITH_BLE=y`
4. `sdkconfig.d/tls_console` when `CONFIG_FUGU_WITH_TLS_CONSOLE=y`
5. `sdkconfig.d/usb_msc` when `CONFIG_FUGU_WITH_USB_MSC=y`

Defaults only seed options the existing `sdkconfig` does not set yet, so an existing `sdkconfig` wins over a changed
default. Configure prints a warning listing such keys. If `sdkconfig` looks wrong, delete it to regenerate it.

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

On ESP32-S3 the repository ships this fragment as `sdkconfig.d/vconv_s3`. Build it in a separate build directory with
its own `sdkconfig`:

```bash
SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.d/vconv_s3" idf.py -B build/vconv -DSDKCONFIG=build/vconv/sdkconfig build
```

Provision `config/lab/vconv_mock`.

:::warning Never OTA a VCONV build to a real converter
A `VCONV` build replaces the gate driver with a simulation, so the half-bridge is never switched and the converter
stops charging. The sensors stay real, so the device looks healthy: it keeps sweeping, Vin sits at the panel's
open-circuit voltage, and the log repeats `Vr-sensor-fail`. Meanwhile the loads drain the battery. `etc/ota.py` warns
about VCONV images and refuses them in non-interactive runs.
:::
