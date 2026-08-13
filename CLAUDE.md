# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build / Flash / Monitor

This is an ESP-IDF project (v5.5+) using `espressif/arduino-esp32` as a component. Target is `esp32s3` (also supports
`esp32`).

Before any `idf.py` invocation, source ESP-IDF into the shell. The repo ships a helper:

```bash
. ./idf-export.sh          # sources ../../esp/idf5.5/export.sh, sets IDF_TARGET=esp32s3, autodetects $ESPPORT
idf.py set-target esp32s3  # only needed once per build dir
idf.py build               # feature flags live in Kconfig now (CONFIG_FUGU_WITH_*, idf.py menuconfig -> "Fugu MPPT firmware"): BLE/NETW default on, NETTOOLS/MCPWM/VCONV/SPROFILER/MEASURE_COIL off (NETTOOLS depends on NETW). The old WITH_* env vars are rejected. Binary telemetry is a tele.conf setting, not a build flag
idf.py -p $ESPPORT app-flash
./etc/fugu_console.py -p ESPPORT # see cli docs, it supports --stdin and -c
```

`idf.py flash`/`app-flash` automatically archive the build ELF for later coredump symbolication (via the project's
`idf_ext.py`). Name the device with `FUGU_DEVICE=<name> idf.py flash` or `./flash.sh <name>` (see "ELF archive" below).

Default build target is `esp32s3`, for `esp32` target use another build dir: `idf.py -B build-esp32`.

The top-level `CMakeLists.txt` invokes `littlefs_create_partition_image(... FLASH_IN_PROJECT)` so the **board config (
currently `config/lab/wokwi_mock`) is flashed together with the firmware**. To target a different board, either edit
that path in `CMakeLists.txt` before `idf.py flash`, or skip it and provision the littlefs partition separately (see
below).

**OTA update over Wi-Fi:** `./ota.sh` builds, serves `build/` over HTTP on port 9000, then runs `etc/ota.py` which tells
the device (via MQTT/serial) to pull the new image. The device must already have network and an `ota.conf`/MQTT broker
configured.

`etc/ota.py` discovers devices (scope-server broadcast + NAT-router scan, falls back to `fallback_hosts`), serves
`build/fugu-firmware.bin` on :9000 with a URL built from *this* host's IP as the device sees it (so it works through the
NAT router), sends `ota <url>` to each, then prints a before/after version table. Run it from the repo root
(`PYTHONPATH=./ .venv/bin/python3 etc/ota.py`, or just `./ota.sh` to build first). Flags:

- `-m REGEX` / `--match` — target only devices whose hostname matches (e.g. `-m flat` for a single board). **Always
  scope with `-m` when you don't intend to update every discovered device.**
- `-n` / `--dry-run` — discover + show the version delta and which hosts *would* update, without sending anything.
- `-f` / `--force` — push even if the device already reports the local build's version.

**Always `-n` first, and `-m <name>` to target, before OTAing `fry`/`flat`** — they're live converters and an OTA
reboots them into the new image. Confirm versions in the after-table.

**OTA safety net (rollback + boot watchdog).** `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` makes a freshly-OTA'd image
boot as `ESP_OTA_IMG_PENDING_VERIFY`. The app must confirm itself or the bootloader reverts to the previous slot on the
next reset:

- `lfMarkOtaValid()` (`main.cpp`, from `loopLF`) calls `esp_ota_mark_app_valid_cancel_rollback()` **only once the RT
  loop
  is proven healthy** (>20 s uptime + sampler producing samples). A directly-flashed image (state `UNDEFINED`, not
  `PENDING_VERIFY`) is left alone — the confirm is a no-op there.
- A one-shot `esp_timer` **boot watchdog** armed at the top of `setup()` (30 s) `esp_restart()`s if `setup()` never
  finishes. With rollback, that reboot then lands on the previous good slot. (The Task-WDT can't cover `setup()`: it
  watches idle, which a yielding hang keeps feeding, and `loopRT` — the only WDT-coupled task — doesn't exist yet.)

**Caveat that bit us:** rollback is a *bootloader* feature, and an OTA only writes the app — so a device that last got
the rollback bootloader via **serial** is protected, but one that's only ever been app-OTA'd keeps its old (
pre-rollback)
bootloader and will **brick, not revert**, on a bad image. To put rollback on a converter, do a full **serial** flash
(bootloader + app, omitting littlefs to keep its config) once; OTAs after that self-revert. A bricked device (hung in
`setup()` before WiFi/services) can only be recovered by serial — see [doc/dev-notes/Real-Time Latency.md] for the
boot-time stack/cache traps that cause such hangs.

## ELF archive (for coredump symbolication)

`esp-coredump` is SHA-gated: a dump only decodes against the *exact* build ELF (its `app_elf_sha256`, stored in the
`.bin`/coredump but zeroed in the `.elf`). The archiver lives in the vendored **`etc/idf-devtools`** submodule
(github.com/fl4p/idf-devtools — generic ESP-IDF tooling); it keeps that ELF around per flash so a later coredump can
still be symbolicated. `idf_ext.py` at the repo root is a thin shim delegating to it.

- **OTA** (`etc/ota.py`) archives automatically after each verified-successful push.
- **Serial:** `idf.py flash`/`app-flash` archive automatically — the `idf_ext.py` shim wraps the stock flash callback
  (so `idf.py flash monitor` still works; archiving happens before monitor). The device name comes from `$FUGU_DEVICE`,
  else the serial-port basename. `./flash.sh <device-name> [idf.py args]` is a thin wrapper that just sets
  `FUGU_DEVICE` (e.g. `./flash.sh fry -p /dev/cu.usbmodem1101 flash monitor`).

It stores one `zstd -19` ELF per unique build (~35 MB → ~9.7 MB) under the gitignored `elf-archive/`, deduped by sha,
with an `index.jsonl` flash log (time, device, method, version, sha). Compression runs **detached** so it never blocks
the flash flow; retention keeps the **30** most-recently-flashed builds (`ELF_ARCHIVE_KEEP`). Decode a dump:

```bash
python3 etc/idf-devtools/elf_archive.py decode <core.bin>           # auto-matches the ELF by sha bytes in the dump
python3 etc/idf-devtools/elf_archive.py decode --device flat <core.bin>  # fallback: latest build flashed to flat
python3 etc/idf-devtools/elf_archive.py list                        # flash history
python3 etc/idf-devtools/elf_archive.py find --device flat -o fw.elf # extract an archived ELF
```

Only builds flashed *after* this was added are archived; older dumps still need their ELF passed to `esp-coredump`
manually. After a fresh checkout run `git submodule update --init etc/idf-devtools`.

The submodule is also the single home for the generic ESP-IDF host tools: `etc/idf-devtools/flash-diff.sh`
(incremental `esptool --diff-with` flashing), `rts.py` (serial RTS/DTR reset), `nvs_dump.py` (NVS partition parser),
and `peek_symbols.py` (symbol→address — `fugu_console.py`'s `peek`/`sym` import it from there). Repo-root
`provision.py` is a thin wrapper that execs `etc/idf-devtools/provision.py`.

## Provisioning (board configs)

Hardware behavior (pins, ADC selection, voltage divider ratios, current sense factors, limits, MQTT broker, charger
params, coil inductance) is **not compiled in**. It lives as `.conf` files on the device's `littlefs` partition under
`/littlefs/conf/`. Folders under `config/` are the source-of-truth images:

- `config/fmetal/` — Fugu2 board (the "standard" hardware)
- `config/fugu1/` — original Fugu (ADS1015 or internal ADC variants)
- `config/lab/dry_mock`, `config/lab/wokwi_mock`, `config/lab/dry_int` — bench/sim setups
- `config/psu_12v`, `config/solar-boost` — alternate topologies

To flash a config without rebuilding the firmware:

```bash
export ESPPORT=/dev/cu.usbmodem...
./provision.py fmetal       # builds image with littlefs-python, writes via parttool.py
```

The set of conf files the firmware reads at boot (from `src/main.cpp::setup()`): `board.conf`, `sensor.conf`,
`limits.conf`, `coil.conf`, `converter.conf`, `charger.conf`, `tracker.conf`, `mqtt.conf`, `tele.conf`, `pprof.conf`.
Each `ConfFile` is a flat key=value parser (`src/conf.h`). Additionally each *service* (see Service architecture below)
reads its own conf file for `enabled` (0/1) and `log_level` (error/warn/info): `mqtt.conf`, `tele.conf` (telemetry),
`ftp.conf`, `telnet.conf`, `lcd.conf`, `scope.conf`.

`set-config <file>.conf <key> <value>` from the serial/telnet/MQTT console edits these in place without re-flashing —
useful when iterating. FTP is also enabled when Wi-Fi is up (Filezilla, 1 connection, no passive mode).

**When you add, rename, or remove a `.conf` key**, update all three sources together so they don't drift:
the per-file reference table in `doc/Configuration.md`, and the editor metadata in
`etc/config-tool/conf-editor.html` (`META` + `FILE_KEYS`).

## Tests

Unit tests live under `test/` and reuse the same firmware build, swapping `main.cpp` for `test/main.cpp` via
`main/CMakeLists.txt`:

```bash
RUN_TESTS=1 idf.py -B build-tests build flash monitor
```

To swap in a single alternate entry point without touching CMakeLists:

```bash
MAIN_SRC=../test/test_buck.cpp idf.py build
```

There's a tiny `test/host-stub/` for host-side compile checks, but the canonical test path is on-target via Unity.

## Architecture

### Dual-core task layout

ESP32-S3 cores are split deliberately (see `loopRT` in `src/main.cpp`):

- **Core 1 (RT_CORE)** — `loopRT` task at priority 20. ADC sampling, protection checks, MPPT/PD controllers, PWM
  updates. **Nothing else may run here**: `sdkconfig.defaults` pins Arduino, Wi-Fi/LWIP, MDNS, MQTT, and the esp_timer
  task to core 0; the timer ISR is on core 1 for low-latency callbacks. `loopRT` checks `xPortGetCoreID()` and `#error`s
  if config drifts.
- **Core 0** — Arduino `loop()` calls `loopNetwork_task`: UART/telnet console, Wi-Fi reconnect, FTP, telemetry (InfluxDB
  over UDP), MQTT, LCD, `loopLF` (low-frequency status line + LED).

The RT loop never sleeps voluntarily — it blocks waiting for the next ADC sample inside `adcSampler.update()`. Don't add
`vTaskDelay` to the RT path.

### Control-loop pipeline (each new ADC sample)

`src/main.cpp::loopRTNewData` → `mppt.update()` (`src/mppt.cpp`):

1. **ADC sampler** (`src/adc/sampling.h` — `ADC_Sampler`) schedules async reads round-robin across channels and
   per-sensor IIR notch + moving median + EWM filters (`src/math/`). `Vout` is always sampled last so the controller
   reacts to it with minimum latency.
2. **Protection** (`mppt.protect`, `mppt.protectLf`) — hard cutouts on Vout OV, Iout OC, Vin UV, temperature;
   `stopAndBackoff(seconds)` disables the converter on violation.
3. **PD controllers** (`src/pd_control.h`) — five units: `VinCTRL`, `IinCTRL`, `VoutCTRL`, `IoutCTRL`, `PowerCTRL` (
   thermal derate). The smallest response wins each tick; if negative, MPPT halts and duty decreases proportionally.
4. **MPPT tracker** (`src/tracker.h`) — three phases: global sweep (0→max duty, captures peak), fast P&O around the
   captured MPP, then slow P&O. Re-sweep every 30 min or on major power change.
5. **PWM update** via `SynchronousConverter` (`src/buck.h`) — uses Vin/Vout and coil L to compute when the LS rectifier
   may turn on (diode emulation, CCM↔DCM), with a slow fade-in and reverse-current pullback. `L0` in `coil.conf` is
   undershot by 5% (`InductivityDcBias`) to model core saturation. See `doc/Diode Emulation.md`.

### Sensor abstraction

`AsyncADC<float>` (`src/adc/adc.h`) is the interface; implementations: `ADC_ADS` (ADS1x15), `ADC_INA226`,
`ADC_ESP32_Cont` (continuous internal ADC DMA), `ADC_Fake` (sinusoidal mock). `setupSensors()` in `main.cpp` builds the
`Sensor` instances for `vin`/`iin`/`iout`/`vout`/`ntc` from `sensor.conf`: `<chn>_adc` picks the backend, `<chn>_ch`
picks the channel (255 = absent), and a missing current sensor is replaced by a `VirtualSensor` computing it from the
other side and `power_conversion_eff`. `LinearTransform{factor, midpoint}` scales raw ADC → physical units (auto-derived
from `*_rh`/`*_rl` for voltage dividers).

### Charger / battery termination

`src/charger.h::Li_ChgTerminationCondition` implements the LFP/Li termination line between `cv_min` (e.g. 3.37V/cell
float) and `cv_eoc` (e.g. 3.65V/cell at low current). When a BMS is reachable over MQTT it publishes the highest cell
voltage; the charger uses that instead of pack voltage / N_cells. If BMS data is stale (>180s,
`VCELL_EXPIRATION_TIME_SEC`), `Vbat_fallback` is used.

### Service architecture

The optional non-RT subsystems (MQTT, telemetry/InfluxDB, FTP, telnet, LCD, scope) are wrapped as **services** (
`src/service.h`). A `Service` exposes `start()`/`stop()`/`restart()`, reports a `ServiceState` (Running/Stopped/Failed),
has its own log level (its `name()` is the ESP_LOG tag; `setLogLevel()` → `esp_log_level_set`, ERROR/WARN/INFO only),
and an optional `onTick()` driven from `loopNetwork_task` (core 0 only — RT-critical ADC/MPPT/converter on core 1 are *
*not** services). `ServiceManager g_services` holds them; they're registered + started in `setup()`. Per-service
`enabled`/`log_level` persist in the service's own conf file. Wi-Fi stays a precondition (not a service): network
services fail-to-start until Wi-Fi is up, then self-heal on the Wi-Fi-up edge in `loopNetwork_task`. Concrete wrappers
live in `src/main.cpp` (they touch `mppt`/`lcd`/`sensors`); `MqttService` *is* the service (`src/tele/mqtt.*`), with a
`preStart` hook wired in `setup()` for the charger/HA coupling. Console: `svc [list]`,
`svc on|off|restart|rs <name>`, `svc log <name> <error|warn|info>`.

### Console & debugging surfaces

Single command dispatcher `handleCommand()` in `src/cli.cpp` handles input from UART, USB-CDC, telnet, **and** MQTT (
same string protocol). Notable commands: `+N`/`-N` (PWM step), `dc N` (manual duty, switches to manual mode),
`sweep`, `mppt` (re-enable auto), `psu <V>` (constant-voltage PSU mode), `sync on/off/forced`, `bf 0/1` (backflow switch), `fan N`, `set-config`/`get-config`,
`ota <url>`, `rt-stats`, `sensor`, `status` (charger/battery snapshot), `wifi-add ssid:psk`, `restart`. See
`doc/Console.md`.

Host-side console client: `etc/fugu_console.py` drives this protocol from a PC over **serial (`-p`),
TCP/telnet (`--ip`), BLE/NUS (`--ble`), BLE via an ESPHome bluetooth_proxy (`--ble-proxy <host>`,
plaintext API/no noise — by `--name` or `--address`), or MQTT (`--mqtt`)** — `-c "<cmd>"` runs one
command (repeat `-c` for several over one connection), `--stdin` runs newline-separated commands
from stdin over one connection (best for scripted/agent use — one connect, replies tagged `=== cmd ===`;
auto-selected when no mode flag is given and stdin is piped, so a heredoc just works),
and the default (a transport but no mode flag) is an
interactive REPL. With no args at all it scans every transport for devices. The PASS/FAIL/SKIP
command exerciser moved out of the client into the e2e suite
(`etc/e2e-test/test_console_plan.py`, with `--mock`/`--include-network` flags). Transport +
line-console mechanics live in the vendored `etc/fugu/` package (its own repo, `fl4p/fugu-py`):
`transport.py` (`SerialTransport`/`SocketTransport`/`BleTransport`/`EspHomeBleTransport`/
`MqttTransport`), `console.py` (`Console`: line assembly, `command()→Reply`); `fugu.py::FuguDevice`
wraps `Console` for the PWM-aware `etc/ota.py`.

In-firmware debug: `rtcount(label)` macros (`src/etc/rt.h`) accumulate per-section timings; `sprofiler` is a sampling
profiler (only useful with OpenOCD attached, configured by `pprof.conf::sprofiler_hz`); `scope` streams raw ADC over TCP
for noise debugging.

## Conventions that aren't obvious from skimming

- `component_compile_options(-Werror=missing-field-initializers)` and `-Werror=attributes` are on — designated
  initializers must cover every field or you'll fail the build.
- `-fexceptions` is on (`CMakeLists.txt` and menuconfig). Throwing from setup is OK; throwing from the RT loop is not —
  wrap in try/catch and call `stopAndBackoff`.
- `IRAM_ATTR` is required on anything called from the ADC continuous-mode ISR (sdkconfig has
  `CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE=y`).
- C++20 features beyond `gnu++17` are off because enabling `gnu++20` drags in `<clocale>` via `nvs.hpp` and breaks
  linking — see the comment in `main/CMakeLists.txt`.
- `FUGU_BAT_V` env var at build time hardcodes battery max voltage (14.25/28.5/57); leave unset to read from
  `charger.conf`.
- `partitions.csv` defines two OTA slots (`ota_0`/`ota_1`) at ~1.87 MB each plus a 128 KB `littlefs` data partition at
  the end of flash. Don't grow OTA slots without re-checking 4 MB headroom.
- When adding a sensor: register in `setupSensors()`, **`vout` must remain the last sensor added** (lowest latency for
  over-voltage protection).
- **No `<sstream>` / `std::stringstream` / `std::stringbuf`.** The xtensa-esp-elf 14.2 toolchain patches `<sstream>` so
  `basic_stringbuf`'s constructor calls an undefined `basic_stringbuf_nop()` — a deliberate Espressif anti-bloat hook.
  Any TU that constructs a `std::stringbuf` (directly or via `stringstream`/`ostringstream`) will fail to link with
  `undefined reference to 'basic_stringbuf_nop'`. Use `snprintf` / `UART_LOG(fmt, …)` / `std::string` concatenation
  instead.
- `sdkconfig` is gitignored — it's a generated artifact that varies with target + `WITH_*` flags + IDF version.
  Source of truth is `sdkconfig.defaults` (+ `sdkconfig.defaults.esp32` overlay, `sdkconfig.ble` when
  `CONFIG_FUGU_WITH_BLE=y`, `sdkconfig.no_netw` when `CONFIG_FUGU_WITH_NETW=n` — layered by the top
  `CMakeLists.txt`). Delete `sdkconfig` to force regeneration if it ever looks wrong.
- if you want to `git revert` but there are local dirty files, do a `git stash` before and `git stash pop` after
- **Never ever run `git reset --hard`**
- if you cannot find the `timeout` command, run `brew install coreutils` and try again
- Never copy/mirror an existing `#define` that defines a constant value to another file, just because you cannot include
  the file were it is defined. Look for a header file that both files already include and put it there (e.g. util.h).
- there is a console command `peek`, that allows you to read memory at an address. `fugu_console.py` implements a symbol
  resolver.

# Connecting to devices

You can connect to devices:

- serial port (preferred)
- telnet (discover devices on network with [discover.py](etc/fugu/discover.py))
    - requires '\n' line termination
- BLE (NUS Console)
    - pushing an update over BLE can be unreliable. better to start a local server to serve the image and invoke the
      `ota <url>` command via the ble console. this will work even when the device is behind the NAT.

When connected to the 192.168.1.x network, these devices might be behind a NAT router:

| IP          | telnet reachable via |
|-------------|----------------------|
| 192.168.4.2 | 192.168.1.231:232    |
| 192.168.4.3 | 192.168.1.231:233    |
| 192.168.4.4 | 192.168.1.231:234    |
| 192.168.4.5 | 192.168.1.231:235    |

IP addresses can change (not static), so always confirm the hostname in the telnet banner.
Check etc/nat.env for up-to-date addresses.

**IMPORTANT: fry & flat are both real power converters connected to solar panels and a battery.
Driving their Half-Bridge must be taken with care!**

Devices with hostnames like `fugu-esp32s3-*` are bench devices, not real power converters.

Confirm the hostname in the welcome message (`Welcome to <hostname> (192.168.4.2)`), as IPs are not static.
Confirm the device ip address with the `ip` command.

# Device Data

You have access to live and historic device telemetry and logs.
Read device log through `ssh havan.local` `tail pv/fugu_console.log -f -n 200`.
The telemetry data is at influxdb tm.fabi.me:8086 / db `ha_van` / measurement mppt.
(user `$INFLUX_HA_VAN_USER`, pass `$INFLUX_HA_VAN_PASS` — `.claude/memory/secrets.env`).

```bash
set -a && . .claude/memory/secrets.env && set +a   # loads $INFLUX_HA_VAN_USER/_PASS
curl -sG http://tm.fabi.me:8086/query --data-urlencode db=ha_van \
  --data-urlencode "u=$INFLUX_HA_VAN_USER" --data-urlencode "p=$INFLUX_HA_VAN_PASS" \
  --data-urlencode "q=SELECT * FROM mppt WHERE device='fry' ORDER BY time DESC LIMIT 3"
```

Fields: `Ui`=Vin, `Uo`=Vout, `P`=power, `I`=Iout, `pwm_duty`=HS count, `cv_lim_idx`=active CV
limiter, `lag`=loop latency, `mcu_temp`/`ntc_temp`, `mppt_state`. `device` is a tag (`fry`/`flat`).

Find battery data in InfluxDB with `batmon()` in
`/Users/fab/dev/ha/home-assistant-addons/batmon-ha/tools/impedance/datasets.py`, use default device="bat_caravan"

# Important

- whenever you create a mark-down (.md) or other documentation file, put
  "*this document is an LLM generated placeholder*" in the first line
- no `#include <>` hints
- when describing a function, interface or class, describe it with a local scope, not how it is used in the application
- when writing code, focus on low memory usage and small code size. re-use data that is available and when in non-
  time-critical code write a transformation or cast if necessary. think twice before creating a new member variable. if
  you could use a non-accessible (private) member, expose this with a getter. for vendor libraries ask for confirmation
  to change.
- keep code comments at a minimum and short
- if your identity is 'OpenCode (powered by moonshotai/kimi-k2.6)', set your git username to "kimi" and always commit
  with this name
- if your identity is 'Claude Code, Anthropic's official CLI for Claude', do commits under my name.
- before `git commit ...`, always check for staged files and unstage those changes that are unrelated to what you just
  did.
- a mock-ADC configuration for physical devices is in [dry_mock](config/lab/dry_mock)
    - [wokwi_mock](config/lab/wokwi_mock) is for work with the Wokwi simulator
- in commit messages, try to be short, it's not necessary to point out which functions are called, just an abstract
  summary. for example instead of
  `keep retrying that same network via WiFi.reconnect() for wifi.conf::switch_delay seconds (default 30, 0=off)` write
  `keep retrying that same network for switch_delay seconds`
- The devices print every error (or panic) on the console. Any error with the ADC like `E (9378) main: ADC error`
  is critical and you must report it. After a panic the device stores a coredump, which you can retrieve with the
  `coredump` command.
- When doing bisect to find a bad commit, work on a shadow repo: create a sub-folder '_shadow_XYZ' and copy the git
  index there and checkout


# Python Interpreter

use `.venv/bin/python3`. It already has `numpy` and `pandas`. If anything is missing, install through `pip`.