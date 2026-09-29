---
title: Host tools
sidebar_position: 8
---

*this document is an LLM generated placeholder*

# Host tools

Python and shell tools under `etc/` and the repo root for talking to, updating, provisioning and testing devices
from a PC.

Run them from the repo root with a Python 3 environment that has their dependencies, for example a virtualenv:

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install pyserial bleak paho-mqtt zeroconf requests rich tqdm tamp   # + aioesphomeapi for --ble-proxy
```

Keep `aioesphomeapi` out of the ESP-IDF Python environment; its dependencies break `idf.py`. Tools that call `idf.py`,
`parttool.py` or `esptool` need a sourced ESP-IDF environment. Submodules: `git submodule update --init
etc/idf-devtools etc/adcscope`.

| Tool                                                       | Purpose                                          |
|------------------------------------------------------------|--------------------------------------------------|
| [`etc/fugu_console.py`](#fugu_consolepy)                   | Console client over serial, telnet, BLE, MQTT    |
| [`etc/fugu_health.py`](#fugu_healthpy)                     | Read-only health check table                     |
| [`etc/ota.py`](#otapy)                                     | OTA update over Wi-Fi                            |
| [`etc/ota_ble.py`](#ota_blepy)                             | OTA update over BLE                              |
| [`provision.py`](#provisionpy)                             | Write a board config to the `littlefs` partition |
| [`etc/dump_littlefs.py`](#dump_littlefspy)                 | Read the `littlefs` partition back               |
| [`etc/idf-devtools/flash-diff.sh`](#flash-diffsh)          | Incremental serial flashing                      |
| [`etc/idf-devtools/elf_archive.py`](#elf_archivepy)        | Archive build ELFs, decode coredumps             |
| [`etc/scope.py`](#scopepy)                                 | ADC soft oscilloscope (adcscope)                 |
| [`etc/measure_coil.py`](#measure_coilpy)                   | Measure coil inductance and rectifier timing     |
| [`etc/influx_binary_proxy.py`](#influx_binary_proxypy)     | Decode binary/BLE telemetry to InfluxDB          |
| [`etc/e2e-test/run_e2e.py`](#run_e2epy)                    | Host-side end-to-end test runner                 |

## fugu_console.py

Console client for the firmware's text [command protocol](console.md). One command (`-c`), a batch from stdin
(`--stdin`), or an interactive REPL. With no arguments at all it scans all transports (serial ports, mDNS, local
BLE, and the MQTT broker in `$MQTT_HOST`) and prints how to connect, without connecting.

```bash
python3 etc/fugu_console.py                                  # discover devices
python3 etc/fugu_console.py -p $ESPPORT                      # REPL over serial
python3 etc/fugu_console.py --ip <host> -c status -c sensor  # two commands, one connection
python3 etc/fugu_console.py --ble --name <name> --stdin < cmds.txt  # batch over BLE
python3 etc/fugu_console.py -p $ESPPORT --coredump get       # pull + decode a coredump
```

| Flag                                  | Meaning                                                                        |
|---------------------------------------|--------------------------------------------------------------------------------|
| `-p`, `--port`                        | Serial port (default `$ESPPORT` or autodetect)                                 |
| `-b`, `--baud`                        | Baud rate, default 115200                                                      |
| `--ip HOST[:PORT]`                    | TCP/telnet                                                                     |
| `--ble`                               | BLE NUS; filter by advertised name with `--name`                               |
| `--name`, `--address`                 | Name substring filter (BLE name or MQTT hostname), BLE address                 |
| `--adapter hciN`                      | BlueZ adapter (Linux)                                                          |
| `--ble-proxy HOST[:PORT]`             | BLE via an ESPHome `bluetooth_proxy` (plaintext API); `--proxy-password`       |
| `--mqtt BROKER`                       | MQTT, with `--mqtt-port`, `--mqtt-user`, `--mqtt-pass`; see [MQTT topics](mqtt.md) |
| `--mqtt-readonly`                     | Stream the log, never publish commands                                         |
| `-c`, `--command CMD`                 | Run a command and exit; repeatable                                             |
| `--stdin`                             | Run newline-separated commands from stdin (auto when stdin is piped)           |
| `--elf PATH`                          | ELF for `peek <symbol>` / `sym` resolution (default `$FUGU_ELF` or newest build) |
| `--tele`                              | With `--ble`: subscribe the telemetry stream and print decoded lines           |
| `--coredump [info\|get\|erase]`       | Inspect, fetch and decode, or erase the stored coredump                        |

Transports and line handling live in the `etc/fugu` package ([fl4p/fugu-py](https://github.com/fl4p/fugu-py)).

## fugu_health.py

Runs read-only commands (`ip`, `status`, `mem`, `svc list`, `sensor`) and prints a verdict table. Never drives
the converter. Takes the same transport flags as `fugu_console.py`, plus `--plain` (borderless table) and
`--timeout` (per command, default 6 s). Needs `rich`.

## ota.py

Updates devices over Wi-Fi. It discovers devices via mDNS, reads their running version over telnet, serves
`build/fugu-firmware.bin` over HTTP on port 9000 (starting `python3 -m http.server` from the repo root if the port
is free), sends `ota <url>` to each device that needs it, and prints a before/after version table. Successful
pushes are recorded in the [ELF archive](#elf_archivepy).

```bash
PYTHONPATH=./ python3 etc/ota.py -n -m <hostname>   # dry run first
PYTHONPATH=./ python3 etc/ota.py -m <hostname>
./ota.sh -m <hostname>                                        # idf.py build, then ota.py with these args
```

| Flag                  | Meaning                                                    |
|-----------------------|------------------------------------------------------------|
| `-m`, `--match REGEX` | Only devices whose hostname matches                         |
| `-n`, `--dry-run`     | Show what would be updated, send nothing                   |
| `-f`, `--force`       | Update even if the device already runs the local version   |

:::warning
Without `-m` every discovered device is updated. `./ota.sh` forwards its arguments to `ota.py` and refuses
(exit 2) unless they contain `-n` or `-m`. `ota.py` asks for confirmation (and refuses when not
interactive) for a build without networking, a simulator (`CONFIG_FUGU_WITH_VCONV`) build, or an uncommitted
(`-dirty`) build. See [OTA over Wi-Fi](../guide/updating/ota-wifi.md).
:::

## ota_ble.py

Pushes an image over the BLE NUS link, for devices without Wi-Fi. Needs a `CONFIG_FUGU_WITH_BLE` build with the
`ble` service on, `bleak` on the host and the `esp-ota-ble` host module (from `managed_components/` after the first
build, a local `../esp-ota-ble`, or `$ESP_OTA_BLE_HOST`). Skips the push if the device already runs the image's version.

```bash
python -m etc.ota_ble build/fugu-firmware.bin <name>
```

| Flag / argument                 | Meaning                                                                  |
|---------------------------------|--------------------------------------------------------------------------|
| `bin`                           | Image, default `build/fugu-firmware.bin`                                 |
| `name`, `-n`, `--name`          | Device name or BLE address filter (default `$BLE_NAME`)                  |
| `-f`, `--force`                 | Push even if the version matches                                         |
| `-y`, `--yes`                   | Skip the confirmation for an image built without BLE                     |
| `--address`                     | Target BLE address                                                       |
| `--ble-proxy HOST[:PORT]`       | Via an ESPHome `bluetooth_proxy`; `--proxy-password`                     |
| `--xform auto\|raw\|tamp\|delta` | Payload: delta against the running image, compressed, or raw. `auto` picks the smallest supported |
| `--base-dir DIR`                | Extra directory to search for the running image (delta base); repeatable |

Additional transport flags come from the `esp-ota-ble` module. See [OTA over BLE](../guide/updating/ota-ble.md)
and [BLE OTA transports](../guide/updating/ble-ota-transports.md).

## provision.py

Builds a littlefs image from `config/<board>` (or any directory containing `conf/`) with `littlefs-python` and
writes it to the `littlefs` partition with `parttool.py`. The firmware is not touched. Wrapper for
`etc/idf-devtools/provision.py`.

```bash
export ESPPORT=/dev/ttyUSB0
./provision.py fmetal
```

| Env                   | Meaning                                                  |
|-----------------------|----------------------------------------------------------|
| `ESPPORT`             | Serial port (required)                                   |
| `IDF_TARGET`          | If set, must match `board.conf` `mcu`                    |
| `LITTLEFS_PARTITION`  | Target partition, default `littlefs`                     |
| `LITTLEFS_SIZE`       | Image size, default `0x20000`                            |
| `LITTLEFS_BLOCK_SIZE` | Block size, default `4096`                               |

See [Provisioning](../guide/getting-started/provisioning.md) and [Partition layout](partitions.md).

## dump_littlefs.py

Reads a littlefs partition over serial and unpacks it to a directory (default `littlefs-dump`), or unpacks an
existing image with `--input`. Flags: `-p`/`--port` (default `$ESPPORT`), `--partition` (default `littlefs`),
`--input`, `--keep-image PATH`, `--block-size`.

## flash-diff.sh

Wrapper around `esptool write-flash --diff-with` that re-writes only the flash sectors that changed since the last
flash through the same port. Offsets and files come from `build/flasher_args.json`; with no file argument it
flashes the app. Needs esptool ≥ 5.2 (ESP-IDF 5.5 bundles v4; install v5 separately).

```bash
etc/idf-devtools/flash-diff.sh -p $ESPPORT
```

Env: `ESPTOOL_DIFF_CACHE` (default `build/.flash-diff`), `BUILD_DIR` (default `build`), `ESPTOOL`.

## elf_archive.py

Keeps one zstd-compressed ELF per unique build, deduplicated by the app ELF SHA-256, plus an `index.jsonl` flash
log, so a later coredump can be decoded against the exact build. `idf.py flash`/`app-flash` and `ota.py`
archive automatically; the archive lives in `$ELF_ARCHIVE_DIR` or `./elf-archive`.

```bash
python3 etc/idf-devtools/elf_archive.py decode coredump.bin             # match by SHA in the dump
python3 etc/idf-devtools/elf_archive.py decode --device <name> core.bin # fallback: latest build for device
python3 etc/idf-devtools/elf_archive.py list
python3 etc/idf-devtools/elf_archive.py find --device <name> -o fw.elf
```

| Subcommand | Flags                                                                               |
|------------|-------------------------------------------------------------------------------------|
| `archive`  | `<device>`, `--method ota\|serial`, `--at ISO`, `--build-dir`, `--elf`, `--bin`, `--version`, `--foreground` |
| `list`     | `--device`                                                                          |
| `find`     | `--device`, `--at`, `--sha`, `-o`/`--out`                                           |
| `decode`   | `<core>`, `--device`, `--at`, `--sha`, `--core-format` (default `raw`)              |

Set the device name for serial flashes with `FUGU_DEVICE=<name> idf.py flash`. See
[Debugging](../development/debugging/index.md).

## scope.py

Launches the [adcscope](https://github.com/fl4p/adcscope) soft oscilloscope (`etc/adcscope` submodule) against
the firmware's `scope` service, which streams raw ADC samples over TCP. Arguments are passed through to
adcscope.

| Flag                      | Meaning                                                    |
|---------------------------|------------------------------------------------------------|
| `--ip`                    | Device IP, skips discovery                                 |
| `--port`                  | TCP port, default 24                                       |
| `-m`, `--match`           | Connect to the device whose hostname contains this         |
| `--rate`                  | Fallback sample rate, default 2000 Hz                      |
| `--median`                | 5-tap median spike filter                                  |
| `--load DIR`              | Open a saved capture instead of a device                   |
| `--discover-interval`     | Seconds between discovery sweeps, default 3                |

## measure_coil.py

Measures the coil inductance without a current probe from Vin, Vout and Iout at a series of low duty cycles in
DCM, with the output clamped by a battery, and reports the median `L0`. `--ls-sweep` instead holds the high side
and sweeps the low-side count to find the rectifier timing (`rect_offset_ns`). The on-device `measure-coil`
command is a port of this script.

:::danger
This drives the half-bridge (`dc`). Keep `--i-max` low and have a current-limited source.
:::

```bash
python etc/measure_coil.py -p $ESPPORT --steps 12 --i-max 1.5
```

| Flag                              | Default | Meaning                                                       |
|-----------------------------------|---------|---------------------------------------------------------------|
| `-p` / `--ip` / `--ble [NAME]`    |         | Transport (one of)                                            |
| `--fsw`                           | from `board.conf` | Switching frequency, Hz                             |
| `--pwm-max`                       | derived | PWM period in counts                                          |
| `--steps`                         | 10      | Duty steps across the DCM band                                |
| `--lo`, `--hi`                    | 0.25, 0.9 | Start/end duty as a fraction of the DCM boundary             |
| `--i-max`                         | 2.0     | Abort a step above this Iout, A                               |
| `--dwell`                         | 5.0     | Settle time per step, s                                       |
| `--bidir`                         |         | Sweep up and down, compare                                    |
| `--plot-every N`                  | 0       | ASCII L plot every N points                                   |
| `--ls-sweep`                      |         | Low-side timing sweep; with `--hs`, `--ls-steps` (24), `--ls-lo` (0.5), `--ls-hi` (1.4) |
| `--apply`, `--apply-margin`       | 12      | With `--ls-sweep`: write `coil.conf` `rect_offset_ns`, keeping a margin in counts |
| `--restore mppt\|off`             | `mppt`  | State when done                                               |
| `--yes`                           |         | Skip the confirmation                                         |

See [Coil Inductance Measurement](../lab/coil-inductance.md).

## influx_binary_proxy.py

Decodes the binary telemetry wire (`tele.conf` `binary=1`, the BLE stream, or the BLE advertising record) back
into InfluxDB line protocol, then prints or forwards it. See [Telemetry fields](telemetry-fields.md).

```bash
python3 etc/influx_binary_proxy.py --listen 0.0.0.0:8086                     # UDP, decode + print
python3 etc/influx_binary_proxy.py --adv --forward-udp 127.0.0.1:8089         # BLE advertisements
python3 etc/influx_binary_proxy.py --ble <name> --influx http://influxdb:8086 --db <db> --user <u> --password <p>
```

| Flag                       | Meaning                                                               |
|----------------------------|-----------------------------------------------------------------------|
| `--listen HOST:PORT`       | UDP listen address, default `0.0.0.0:8086`                            |
| `--ble NAME`               | Pull the BLE stream from one device (`--address`: argument is a MAC)  |
| `--ble-all`                | Pull from every device in range; `--scan-interval` (default 30 s)     |
| `--adv`                    | Observe connectionless advertisements; `--verbose` prints each record |
| `--forward-udp HOST:PORT`  | Forward decoded lines as UDP line protocol                            |
| `--influx URL`             | Forward over HTTP; `--db`, `--user`, `--password`, `--precision` (default `ms`) |
| `--test BLOB`              | Offline decode self-test                                              |

## run_e2e.py

Runs the host-side end-to-end tests in `etc/e2e-test/`, grouped into clusters by rig requirement, and reports
PASS/FAIL/SKIP. Tests whose prerequisites are missing are skipped with the reason. Exit code: 1 on any FAIL,
2 when nothing ran (all skipped, not with `--dry-run`), else 0.

| Cluster       | Needs                                                                          |
|---------------|--------------------------------------------------------------------------------|
| `console`     | Any device, console only, non-destructive                                      |
| `mock`        | A mock-ADC build over serial                                                   |
| `destructive` | A bench unit only: deliberately panics, reboots, fuzzes                        |
| `power`       | A real converter with a coil; drives the half-bridge                           |
| `wifi`        | A controllable AP/router                                                       |

```bash
python etc/e2e-test/run_e2e.py --list
python etc/e2e-test/run_e2e.py --cluster console --serial $ESPPORT
```

| Flag                         | Meaning                                                              |
|------------------------------|----------------------------------------------------------------------|
| `--cluster`                  | `console` (default), `mock`, `destructive`, `power`, `wifi`, `all`   |
| `--serial DEV`, `--telnet HOST[:PORT]` | Device connection                                          |
| `--mock`                     | The connected build is a mock                                        |
| `--include-network`          | Also run NVS/Wi-Fi-mutating console commands (`wifi on`, `hostname`) |
| `--with-fuzz`                | Long fuzzers (destructive cluster)                                   |
| `--list`, `--dry-run`        | List tests / print each test's argv without running                  |
| `--mqtt-host`, `--restart-url`, `--ssid`, `--other-ssid`, `--psk`, `--router`, `--router-wan-ip` | Rig parameters (env: `MQTT_HOST`, `RESTART_URL`, `E2E_*`) |

See [Testing](../development/testing.md).
