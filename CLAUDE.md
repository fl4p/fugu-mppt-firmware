# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build / Flash / Monitor

This is an ESP-IDF 6.0 (6.0.3) project using `espressif/arduino-esp32` as a component. Target is `esp32s3` (also
supports `esp32`). IDF 5.5 is no longer supported; don't spend effort keeping it building.

`main/idf_component.yml` picks arduino-esp32 >=3.3.12,<4.0.0 (4.0.0 fails configure) and the external `espressif/mqtt`.
IDF driver config structs change fields between releases, so zero-init (`= {}`) and assign instead of listing every
field.

Before any `idf.py` invocation, source ESP-IDF into the shell with the tracked helper. It works from any git worktree
(it finds `../../esp/idf6.0.3` relative to the main checkout via `git rev-parse --git-common-dir`; override with
`FUGU_IDF_PATH=<idf dir>` or `FUGU_ESP_DIR=<dir of idf versions>`). A fresh clone or worktree needs the submodules
first; without them CMake stops with "Submodules are not checked out". In a worktree, git clones some submodules
from the main checkout's `.git/modules` over the `file` transport, which is refused without
`-c protocol.file.allow=always`; `--force` repairs submodules a failed first update left half checked out
(idf-export.sh lists those).

```bash
git -c protocol.file.allow=always submodule update --init --recursive --force   # once per fresh clone/worktree
. ./idf-export.sh          # sources ESP-IDF, sets IDF_TARGET (arg `esp32`, else an exported IDF_TARGET, else esp32s3), sets $ESPPORT to the first USB port unless already set
idf.py set-target esp32s3  # only needed once per build dir
idf.py build               # feature flags live in Kconfig now (CONFIG_FUGU_WITH_*, idf.py menuconfig -> "Fugu MPPT firmware"): BLE/NETW default on, NETTOOLS/MCPWM/VCONV/SPROFILER/MEASURE_COIL/USB_MSC/EXTPORT off (EXTPORT = VE.Direct text output, extport.conf; MEASURE_COIL on in sdkconfig.d/fleet; NETTOOLS depends on NETW; USB_MSC = S3 USB config drive, website/docs/guide/usb-drive.md). The old WITH_* env vars are rejected. Binary telemetry is a tele.conf setting, not a build flag
idf.py -p $ESPPORT app-flash
./etc/fugu_console.py -p ESPPORT # see cli docs, it supports --stdin and -c
```

`idf.py flash`/`app-flash` automatically archive the build ELF for later coredump symbolication (via the project's
`idf_ext.py`). Name the device with `FUGU_DEVICE=<name> idf.py flash` or `./flash.sh <name>` (see "ELF archive" below).

Extra build directories (variants, tests, review or experiment builds) go under `build/<name>/`, never `build-<name>/` in
the repo root. Note that `idf.py fullclean` of the default `build/` deletes them too.

Default build target is `esp32s3`. Classic `esp32`: own build dir and sdkconfig, target from the environment
(`IDF_TARGET=esp32 . ./idf-export.sh` does not stick; `-DIDF_TARGET=esp32` alone clashes with the exported esp32s3):
`. ./idf-export.sh esp32 && idf.py -B build/esp32 -DSDKCONFIG=build/esp32/sdkconfig build`, or prefix each command
with `IDF_TARGET=esp32` in an esp32s3 shell.
Simulated-converter build on the S3 (provision `config/lab/vconv_mock`):
`idf.py -B build/vconv -DSDKCONFIG=build/vconv/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.d/vconv_s3" build`.

The top-level `CMakeLists.txt` invokes `littlefs_create_partition_image(... FLASH_IN_PROJECT)` so the **board config
selected by `FUGU_LITTLEFS_SRC` (`config/lab/dry_mock` on esp32s3, `config/fugu1/fugu1_esp32` on esp32) is flashed
together with the firmware** by `idf.py flash`. To target a different board, either change `FUGU_LITTLEFS_SRC` in
`CMakeLists.txt` before `idf.py flash`, or use `app-flash` and provision the littlefs partition separately (see
below).

**OTA update over Wi-Fi:** `./ota.sh` builds, then runs `etc/ota.py`, which serves `build/` over HTTP on port 9000 and tells
the device (via MQTT/serial) to pull the new image. The device must already have network and an `ota.conf`/MQTT broker
configured.

`etc/ota.py` discovers devices (scope-server broadcast + NAT-router scan), serves
`build/fugu-firmware.bin` on :9000 with a URL built from *this* host's IP as the device sees it (so it works through the
NAT router), sends `ota <url>` to each, then prints a before/after version table. Run it from the repo root
(`PYTHONPATH=./ .venv/bin/python3 etc/ota.py`, or `./ota.sh -n|-m <regex>` to build first; it forwards its args to
`ota.py` and refuses to run without `-n` or `-m`). Flags:

- `-m REGEX` / `--match` — target only devices whose hostname matches (e.g. `-m flat` for a single board). **Always
  scope with `-m` when you don't intend to update every discovered device.**
- `-n` / `--dry-run` — discover + show the version delta and which hosts *would* update, without sending anything.
- `-f` / `--force` — push even if the device already reports the local build's version.

**Always `-n` first, and `-m <name>` to target, before OTAing `fry`/`flat`** — they're live converters and an OTA
reboots them into the new image. Confirm versions in the after-table.

**fry/flat image: `etc/fleet-build.sh [rev]`** (default HEAD) builds the production config (`sdkconfig.d/fleet`, IDF 6.0.3)
in a clean, reused detached worktree `../fugu-fleet-wt` (`build/fleet/`), refuses a dirty worktree and prints the .bin
path, version and sha256. Never ship a main-checkout build: it carries other sessions' uncommitted edits.

**OTA over the MQTT console (preferred for fry/flat):** `.venv/bin/python3 etc/ota_mqtt.py --name fry,flat --bin
../fugu-fleet-wt/build/fleet/fugu-firmware.bin -n`, then without `-n`. Devices go one after the other; a failure stops
the rest. It reads the broker and credentials from `etc/mqtt.env` (main checkout's as fallback), tunnels through
`havan-ts` when the broker is not directly reachable, locks onto the exact hostname and runs pre-flight checks (also in
`-n`): image flags, a `PENDING_VERIFY` running image, a `-dirty` image (refused for fry/flat unless `--allow-dirty` when the tree it was built in has
uncommitted firmware inputs; non-firmware dirt does not block), conf keys the image fails setup on (renamed or role
keys, from `get-config`; refused even with `--yes`, prints the `migrate_side_keys.py` two-step fix; an unreadable conf
file asks), rect-cal running/unsaved, a sweep or top-balance run
(need `--yes`), the RTC symbol layout vs the running build's archived ELF, the device's previous OTA record, and
`--check CMD` outputs (repeatable, read-only commands, diffed after the OTA). A real OTA holds the `lock.sh` locks
`ota-<device>` and `ota-relay` (refuses when held, `--lock-wait S` queues), stages the image on the relay with
`etc/ota-via-relay.sh` when this host cannot serve it (own staging dir, `<name>-<sha8>.bin`, first free port
9000-9099; `status|stop <token>`), sends `ota <url>` and checks `bootinfo` for the image's version on the other slot
with a reset uptime. Failed downloads are retried (`--retries 2`, backoff 30/90 s) with throughput, relay status and
whether the device rebooted (the firmware's 300 s OTA watchdog restarts it). `--verify` (default; `--no-verify`) then
waits for `rollback=VALID` (`--valid-timeout 90`), watches the new boot's log (from the first line after the reboot) for ADC stall, INA timeout,
loop latency, protection backoff (not the boot's `sweep-no-mpp`), panic, non-SW reset and reboot lines (`--watch 120`) and prints power before/after. Each OTA is recorded in `elf-archive/ota-owners.jsonl` (who, version,
sha256, git sha, dirty paths, result). Around each transfer the NAT gateway's Wi-Fi TX power gets a timed boost
(`POST /txpower?wifi=20&for=N`), cancelled afterwards by a plain set of the persisted level, also on failure or Ctrl-C
(`--gw`, `--gw-tx-dbm`, `--no-gw-boost`); gateway errors only warn, and after a `kill -9` the boost reverts
on its own. `fugu_console.py --mqtt` (bare = `$MQTT_HOST`)
takes its credentials from the same file, tunnels through `havan-ts` the same way, and returns from `ota` at `OTA Succeed` or a failure line.

**OTAs release the other radio.** `ota <url>` shuts BLE down for the download (`bleConsoleRelease()`, frees the stack
and controller memory) and `ota-ble begin` turns Wi-Fi off unless the user already did (`wifiRelease()`, the `wifi off`
teardown). Neither comes back without a reboot, so after a release every failure, abort or timeout reboots into the
running image. A BLE
console that sends `ota <url>` loses its link; watch the result over serial or MQTT. Both refuse before stopping anything
while the running image is PENDING_VERIFY (`ROLLBACK_INVALID_STATE`, first ~20 s after an OTA), `ota <url>` also
without Wi-Fi, `ota-ble begin` also without a BLE client; `wifi on|off` and Improv Wi-Fi RPCs are refused during a BLE push.

**OTA safety net (rollback + boot watchdog).** `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` makes a freshly-OTA'd image
boot as `ESP_OTA_IMG_PENDING_VERIFY`. The app must confirm itself or the bootloader reverts to the previous slot on the
next reset:

- `lfMarkOtaValid()` (`main.cpp`, from `loopLF`) calls `esp_ota_mark_app_valid_cancel_rollback()` **only once the RT
  loop is proven healthy** (>20 s uptime + sampler producing samples). Without flash auto-suspend it then waits for
  output < 5 W, for up to 2 h (`OTA_CONFIRM_DEADLINE_MS`); a reset in that window rolls back, and `ota` is refused
  while the image is `PENDING_VERIFY` (check `bootinfo` `rollback=`). A directly-flashed image (state `UNDEFINED`, not
  `PENDING_VERIFY`) is left alone — the confirm is a no-op there.
- A one-shot `esp_timer` **boot watchdog** armed at the top of `setup()` (30 s) `esp_restart()`s if `setup()` never
  finishes. With rollback, that reboot then lands on the previous good slot. (The Task-WDT can't cover `setup()`: it
  watches idle, which a yielding hang keeps feeding, and `loopRT` — the only WDT-coupled task — doesn't exist yet.)

**Caveat that bit us:** rollback is a *bootloader* feature, and an OTA only writes the app — so a device that last got
the rollback bootloader via **serial** is protected, but one that's only ever been app-OTA'd keeps its old (
pre-rollback)
bootloader and will **brick, not revert**, on a bad image. To put rollback on a converter, do a full **serial** flash
(bootloader + app, omitting littlefs to keep its config) once, or push the bootloader over Wi-Fi (below); OTAs after that
self-revert.

**Bootloader OTA (unsafe, no recovery bootloader on S3).** `ota bootloader <url>` (host: `etc/ota.py --bootloader -m
<name>`, `-n` shows current vs new from the `bootinfo` `bootloader:` line). Refuses unless conversion was stopped with
`dc 0` (manual target 0, converter disabled), the app is confirmed, flash encryption/secure boot are off. Stages the
image in unallocated flash after the last partition (never the passive slot), checks chip id, appended SHA-256 (IDF only
checks the 8-bit checksum of bootloaders), flash mode and size; skips an identical image, then erases only the sectors
it needs at 0x0 and reads back (3 tries; on failure `systemRestart()` refuses until a retry succeeds). A reset or power
loss during that copy (~0.5 s) bricks the board until serial reflash.
Dangerous-write protection stays on except around the copy. Test: `test/main_bl_ota.cpp` + `test/bl_ota_qemu.py`. A bricked device (hung in
`setup()` before WiFi/services) can only be recovered by serial — see [website/docs/development/debugging/real-time-latency.md] for the
boot-time stack/cache traps that cause such hangs.

## ELF archive (for coredump symbolication)

`esp-coredump` is SHA-gated: a dump only decodes against the *exact* build ELF (its `app_elf_sha256`, stored in the
`.bin`/coredump but zeroed in the `.elf`). The archiver lives in the vendored **`etc/idf-devtools`** submodule
(github.com/fl4p/idf-devtools — generic ESP-IDF tooling); it keeps that ELF around per flash so a later coredump can
still be symbolicated. `idf_ext.py` at the repo root is a thin shim delegating to it.

- **OTA** (`etc/ota.py`, `etc/ota_ble.py`) archives automatically after each verified-successful push.
- **Serial:** `idf.py flash`/`app-flash` archive automatically — the `idf_ext.py` shim wraps the stock flash callback
  (so `idf.py flash monitor` still works; archiving happens before monitor). The device name comes from `$FUGU_DEVICE`,
  else the serial-port basename. `./flash.sh <device-name> [idf.py args]` is a thin wrapper that just sets
  `FUGU_DEVICE` (e.g. `./flash.sh fry -p /dev/cu.usbmodem1101 flash monitor`).

It stores one `zstd -19` ELF per unique build (~35 MB → ~9.7 MB) under the gitignored `elf-archive/`, deduped by sha,
with an `index.jsonl` flash log (time, device, method, version, sha). Compression runs **detached** so it never blocks
the flash flow; retention keeps the **30** most-recently-flashed builds (`ELF_ARCHIVE_KEEP`) plus the last **3** builds of every
device (`ELF_ARCHIVE_KEEP_PER_DEVICE`). The archive is found from any cwd inside the repo. Decode a dump:

```bash
python3 etc/idf-devtools/elf_archive.py decode <core.bin>           # auto-matches the ELF by sha bytes in the dump
python3 etc/idf-devtools/elf_archive.py decode --device flat <core.bin>  # fallback: latest build flashed to flat
python3 etc/idf-devtools/elf_archive.py list                        # flash history
python3 etc/idf-devtools/elf_archive.py find --device flat -o fw.elf # extract an archived ELF
```

Only builds flashed *after* this was added are archived; older dumps still need their ELF passed to `esp-coredump`
manually. After a fresh checkout run `git submodule update --init etc/idf-devtools etc/adcscope`.

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
useful when iterating. FTP is also enabled when Wi-Fi is up (Filezilla; 2 sessions; passive data port `ftp.conf::pasv_port`, default 50009, active mode also works).

**When you add, rename, or remove a `.conf` key**, update all three places together so they don't drift:
the code that reads it, the per-file reference table in `website/docs/reference/config/<file>.md`, and the editor
metadata in `etc/config-tool/conf-editor.html` (`META` + `FILE_KEYS`).

**Keep docs current.** When a change alters behaviour, a console command, a conf key, a build flag or a workflow that a
page in `website/docs/` or this file describes, update that text in the same commit. Stale docs count as a bug.

## Documentation site

`website/` is a Docusaurus site (GitHub Pages) with its pages in `website/docs/`: five sidebars (`guide/`,
`reference/`, `internals/`, `lab/`, `development/`), autogenerated from folders; order via `sidebar_position`
front matter and `_category_.json`. `doc/` keeps internal notes, plans and reviews, which are not published.
Published pages must be **generic**: no device names (fry/flat/flu/fboost/fbuck), IPs, hosts, credentials or
lab procedures; that material goes to `doc/lab/` (unpublished, but the repo is public, so still no
credentials). Check with `cd website && npm run build` (fails on broken links, including relative links to
missing repo files). Math (`$…$`, `$$…$$`, KaTeX) and ```` ```mermaid ```` blocks render there.

Published pages are **curated documentation, not research notes**. Keep the research record (search log,
provenance, reviews, source access log) unpublished in `doc/research/` and write the page from it. No "Bottom
line"/TL;DR lead heading, no process sections, no agent transcripts, no Codex/Claude mentions in prose.
Literature-based pages are surveys: scope, findings per topic with `[n]` citations, implications, limitations, and a
numbered References list whose bibliographic data is verified against the sources. Headings and titles are short
noun-phrase labels ("Temperature", "Float and CV holds", "LFP longevity"), not sentences, claims or questions
("Temperature is the first-order calendar factor", "How much does cycling matter?"); dates, conditions and the finding
go in the body. No walls of text: split a long study paragraph into a short setup (who tested what), a table when
3+ comparable conditions have numbers, a list for parallel figures, and a separate paragraph for caveats/limits.
A generic measurement method and its anonymised results ("board B, 83 µH") go on a published `website/docs/lab/`
page, linked from the `internals/` or `reference/` page that uses them; that page keeps at most a short summary table.
Bench-specific steps (which device, host, wiring) go to `doc/lab/`, the capture record to `doc/research/`.
The production build enforces the mechanical part (`website/src/remark/docs-lint.mjs`; the dev server only warns). `website/src/remark/docs-xlink.mjs` warns (never fails) when a page mentions a topic several times without linking its page; extend its `TOPICS` table when adding a page. Every doc review, by Codex,
Astra or an agent, includes the criteria "reads as reference documentation, not notes or a transcript" and "no paragraph is a wall of
text".
**Always run the `doc-polish` skill** on every page you write or substantially change under `website/docs/` (new pages, new
sections), before the docs build and the commit.

## Tests

Unit tests live under `test/` and reuse the same firmware build, swapping `main.cpp` for `test/main.cpp` via
`main/CMakeLists.txt`:

```bash
RUN_TESTS=1 idf.py -B build/tests build flash monitor
```

To swap in a single alternate entry point without touching CMakeLists:

```bash
MAIN_SRC=../test/main_ads_rate.cpp idf.py build
```

There's a tiny `test/host-stub/` for host-side compile checks, but the canonical test path is on-target via Unity.

## Architecture

### Dual-core task layout

ESP32-S3 cores are split deliberately (see `loopRT` in `src/main.cpp`):

- **Core 1 (RT_CORE)** — `loopRT` task at priority 20. ADC sampling, protection checks, MPPT/PD controllers, PWM
  updates. **Nothing else may run here**: `sdkconfig.defaults` pins Arduino, Wi-Fi/LWIP, MDNS, MQTT, and the esp_timer
  task **and the esp_timer ISR** to core 0 (nothing uses ESP_TIMER_ISR dispatch; see the comment in
  `sdkconfig.defaults`). `loopRT` `assert`s `xPortGetCoreID() == RT_CORE` if config drifts.
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
   undershot by 5% (`InductivityDcBias`) to model core saturation. See `website/docs/internals/diode-emulation.md`.

### Sensor abstraction

`AsyncADC<float>` (`src/adc/adc.h`) is the interface; implementations: `ADC_ADS` (ADS1x15), `ADC_INA226`,
`ADC_ESP32_Cont` (continuous internal ADC DMA), `ADC_Fake` (sinusoidal mock). `setupSensors()` in `src/adc/sensor_setup.cpp` builds the
`Sensor` instances for `vin`/`iin`/`iout`/`vout`/`ntc` from `sensor.conf`: `<chn>_adc` picks the backend, `<chn>_ch`
picks the channel (255 = absent). Hardware profiles name channels by side (`hv_v_`, `lv_v_`, `hv_i_`, `lv_i_`), mapped to
roles by `converter.conf::topo` in `src/conv_side.h` (same for `limits.conf` `hv_max`/`lv_max`/`hv_i_max`/`lv_i_max`);
the old role keys fail setup with their replacement (`etc/migrate_side_keys.py` migrates configs; live boards before the
OTA). `vconv`/`fake` mock channels are fixed by role, so their profiles use the side key of that role under their topo. A
missing current sensor is replaced by a `VirtualSensor` computing it from the other side at a fixed 97 % efficiency. `LinearTransform{factor, midpoint}` scales raw ADC → physical units (auto-derived
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
live in their own headers (`src/tele/*_service.h`, `src/viz/lcd_service.h`, `src/sync/bsync.h`); `main.cpp` only
registers them. `MqttService` *is* the service (`src/tele/mqtt.*`), with a
`preStart` hook wired in `setup()` for the charger/HA coupling. Console: `svc [list]`,
`svc on|off|restart|rs <name>`, `svc log <name> <error|warn|info>`.

### Console & debugging surfaces

Single command dispatcher `handleCommand()` in `src/cli.cpp` handles input from UART, USB-CDC, telnet, **and** MQTT (
same string protocol). Notable commands: `+N`/`-N` (PWM step), `dc N` (manual duty, switches to manual mode),
`sweep`, `mppt` (re-enable auto), `psu <V>` (constant-voltage PSU mode), `sync on/off/forced`, `bf 0/1` (backflow switch), `fan N`, `set-config`/`get-config`,
`ota <url>`, `rt-stats`, `sensor`, `status` (charger/battery snapshot), `wifi-add ssid:psk`, `restart`. See
`website/docs/reference/console.md`.

Host-side console client: `etc/fugu_console.py` drives this protocol from a PC over **serial (`-p`),
TCP/telnet (`--ip`), BLE/NUS (`--ble`), BLE via an ESPHome bluetooth_proxy (`--ble-proxy <host>`,
plaintext API/no noise — by `--name` or `--address`), or MQTT (`--mqtt`)** — `-c "<cmd>"` runs one
command (repeat `-c` for several over one connection), `--stdin` runs newline-separated commands
from stdin over one connection (best for scripted/agent use — one connect, replies tagged `=== cmd ===`;
auto-selected when no mode flag is given and stdin is piped, so a heredoc just works),
and the default (a transport but no mode flag) is an
interactive REPL. With no args at all it scans every transport for devices.
**Lock guard:** state-changing commands (anything not in the firmware's `kRoCmds` list in
`src/etc/con_guard.h` plus host-side `peek`/`sym`/`coredump info|get`/`pwr` reads; unknown commands
count as state-changing) to a hostname in `etc/protected-devices` (fry, flat; `$FUGU_PROTECTED`
overrides) are refused before sending unless the caller holds the device lock (`lock.sh mine fry`;
`ota-<dev>` counts for `ota`/`ota-ble` only). `--take-lock [--lock-note WHY]` takes it for the run,
`--no-lock-guard` sends anyway. Every state-changing command sent to fry/flat is recorded (masked) in
`elf-archive/console-commands.jsonl` with the same `who` as `ota-owners.jsonl`. `ota_mqtt.py` uses
`Console` directly with its own `ota-*` locks and is not guarded. The PASS/FAIL/SKIP
command exerciser moved out of the client into the e2e suite
(`etc/e2e-test/test_console_plan.py`, with `--mock`/`--include-network` flags). Transport +
line-console mechanics live in the vendored `etc/fugu/` package (its own repo, `fl4p/fugu-py`):
`transport.py` (`SerialTransport`/`SocketTransport`/`BleTransport`/`EspHomeBleTransport`/
`MqttTransport`), `console.py` (`Console`: line assembly, `command()→Reply`); `fugu.py::FuguDevice`
wraps `Console` for the PWM-aware `etc/ota.py`.

In-firmware debug: `rtcount(label)` macros (`src/etc/rt.h`) accumulate per-section timings; `sprofiler` is a sampling
profiler (only useful with OpenOCD attached, configured by `pprof.conf::sprofiler_hz`); `scope` streams raw ADC over TCP
for noise debugging.

The scope's host client is **`etc/adcscope`** — a submodule of `fl4p/adcscope`, a device-agnostic
soft oscilloscope extracted from this repo. Run it via `./etc/scope.py` (a wrapper that registers
the NAT/telnet discovery adapter for fry/flat and passes arguments through); the wire format is
`etc/adcscope/PROTOCOL.md`. Capture datasets (`fl4p/fugu-data`) check out at `etc/adcscope/data/`.
Filter-design studies that used to sit beside the client are now `etc/filter-studies/`.

## Conventions that aren't obvious from skimming

- `component_compile_options(-Werror=missing-field-initializers)` and `-Werror=attributes` are on — designated
  initializers must cover every field or you'll fail the build.
- `-fexceptions` is on (`CMakeLists.txt` and menuconfig). Throwing from setup is OK; throwing from the RT loop is not —
  wrap in try/catch and call `stopAndBackoff`.
- `IRAM_ATTR` is required on anything called from the ADC continuous-mode ISR (sdkconfig has
  `CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE=y`).
- The firmware compiles as `gnu++20` (`component_compile_options(--std=gnu++20)` in `main/CMakeLists.txt`).
- `FUGU_BAT_V` is passed as a compile definition by `main/CMakeLists.txt`, but no source reads it; the battery
  max voltage always comes from `charger.conf::vout_max`.
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
  Source of truth is `sdkconfig.defaults` (+ `sdkconfig.defaults.<target>` overlay, `sdkconfig.d/ble` when
  `CONFIG_FUGU_WITH_BLE=y`, `sdkconfig.d/usb_msc` when `CONFIG_FUGU_WITH_USB_MSC=y` — layered by the top
  `CMakeLists.txt`); opt-in and variant fragments live in `sdkconfig.d/`. Delete `sdkconfig` to force regeneration
  if it ever looks wrong. An existing sdkconfig wins over
  the defaults, so a changed default is ignored there; configure prints a WARNING listing such keys (mismatches a
  fresh sdkconfig already has are recorded in `<sdkconfig>.fugu-baseline` and not reported).
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
      `ota <url>` command via the ble console. this will work even when the device is behind the NAT. The BLE link
      closes when the download starts (BLE is released), so check the result with `bootinfo` after the reboot.

## A converter went silent: use the BLE backup link

No log line on `pv/fugu_console.log`, no MQTT console reply and no fresh InfluxDB point does **not** mean the
converter is down, and it is not a reason to wait. BLE keeps working when Wi-Fi, MQTT or the heap are gone
(2026-11-14: flat had Wi-Fi and an IP but no MQTT task after its heap ran out; BMS data went stale, the cap fell back
to 26.8 V and it sat at 1 W for 11 minutes until someone looked over BLE). After about 2 minutes of silence, do this
before anything else, and tell Fab what you find:

1. Direct BLE from this Mac: `.venv/bin/python3 etc/fugu_console.py --ble flat -c hostname -c bootinfo -c status
   -c "svc list" -c tasks -c heap` (advertised names `fugu-flat`, `fugu-fry`). Put all commands in one call: every
   connect costs the device several KB of heap, and back-to-back connects can hang.
2. Weak link (RSSI below about -85 dBm, connect or replies time out): go through the ESPHome proxy,
   `etc/fugu_console.py --ble-proxy <host> --name fugu-flat -c ...` with `<host>` from `etc/nat.env::BLE_PROXY`.
   If this Mac has no route to it, tunnel through havan (`ssh -f -N -L 16053:<host>:6053 havan-ts`, then
   `--ble-proxy 127.0.0.1:16053`). If the proxy does not answer either, say so; do not skip step 1.
3. Read before acting: `heap` (free / min-ever), `svc list` (mqtt, tele), `tasks` (is there an mqtt task), `status`
   (BMS data age), the `V=` line. Then recover with the smallest step (`svc restart mqtt`, else `restart`).

When connected to the 192.168.1.x network, these devices might be behind a NAT router:

| IP          | telnet reachable via | FTP reachable via                    |
|-------------|----------------------|--------------------------------------|
| 192.168.4.2 | 192.168.1.231:232    | 192.168.1.231:2142, PASV 50011-50012 |
| 192.168.4.3 | 192.168.1.231:233    |                                      |
| 192.168.4.4 | 192.168.1.231:234    | 192.168.1.231:2144, PASV 50009-50010 |
| 192.168.4.5 | 192.168.1.231:235    |                                      |

The NAT router is nimble-ble-proxy: forwards via `POST /portmap` (header `X-Nat-Confirm: yes`), list in `GET /nat`.
fry (.4.2) and flat (.4.4) have static leases. PASV ports pass through unchanged (the server announces its own port);
FTP clients must ignore the 192.168.4.x address in the PASV reply (curl `--ftp-skip-pasv-ip`; ftplib and FileZilla do).
fry has `ftp.conf pasv_port=50011` (needs firmware from a3f9df04 on; older builds still announce 50009, which goes to flat).

IP addresses can change (not static), so always confirm the hostname in the telnet banner.
Check etc/nat.env for up-to-date addresses (gitignored: a worktree has none, so `fugu_console.py` discovery skips NAT
there; copy it from the main checkout).

Off the LAN, or when this Mac's agents get "No route to host" for every 192.168.x address (macOS Local Network
privacy for the app hosting the agent, not a network fault; the Tailscale route is not affected), tunnel through havan:
`etc/nat-tunnel.sh up` forwards the NAT ports to `127.0.0.1:12232`..`12235` (`status`, `down`; jump host
`NAT_TUNNEL_HOST`, default ssh alias `havan-ts`). Then `etc/fugu_console.py --ip 127.0.0.1:12232` and check `hostname`.

**IMPORTANT: fry & flat are both real power converters connected to solar panels and a battery.
Driving their Half-Bridge must be taken with care!**

Devices with hostnames like `fugu-esp32s3-*` are bench devices, not real power converters.

Confirm the hostname in the welcome message (`Welcome to <hostname> (192.168.4.2)`), as IPs are not static.
Confirm the device ip address with the `ip` command.

# Device Data

You have access to live and historic device telemetry and logs.
Read device log through `ssh havan.local` `tail pv/fugu_console.log -f -n 200`.
havan also runs the MQTT broker, the only BMS feed (batmon) and the telemetry relay. Before rebooting it (or the .231
router), take `~/bin/lock.sh acquire havan` and announce on the `fugu` channel; if havan goes silent, check
`~/bin/lock.sh status havan` and the channel first (`~/dev/kb/agent-ops/reboot-a-shared-host-only-under-its-lock.md`).
The telemetry data is at influxdb tm.fabi.me:8086 / db `ha_van` / measurement mppt.
(user `$INFLUX_HA_VAN_USER`, pass `$INFLUX_HA_VAN_PASS` — `.claude/memory/secrets.env`).

```bash
set -a && . .claude/memory/secrets.env && set +a   # loads $INFLUX_HA_VAN_USER/_PASS
curl -sG http://tm.fabi.me:8086/query --data-urlencode db=ha_van \
  --data-urlencode "u=$INFLUX_HA_VAN_USER" --data-urlencode "p=$INFLUX_HA_VAN_PASS" \
  --data-urlencode "q=SELECT * FROM mppt WHERE device='fry' ORDER BY time DESC LIMIT 3"
```

Fields: `Ui`=Vin, `Uo`=Vout, `P`=power, `I`=Iout, `pwm_duty`=HS count, `cv_lim_idx`=active CV
limiter, `lag`=loop latency, `mcu_temp`/`ntc_temp`, `mppt_state`, `P_max_today`=today's peak power (W). `device` is a tag (`fry`/`flat`).

Find battery data in InfluxDB with `batmon()` in
`/Users/fab/dev/pv/bat-impedance/datasets.py` (db `open_pe` on influx.fabi.me:8086, creds in its `influxdb_server.json`), use device="bat_caravan"

# Important

- whenever you create a mark-down (.md) or other documentation file, put
  "*this document is an LLM generated placeholder*" in the first line — except published pages under
  `website/docs/`, which never carry it (the docs build rejects it)
- no `#include <>` hints
- when describing a function, interface or class, describe it with a local scope, not how it is used in the application
- when writing code, focus on low memory usage and small code size. re-use data that is available and when in non-
  time-critical code write a transformation or cast if necessary. think twice before creating a new member variable. if
  you could use a non-accessible (private) member, expose this with a getter. for vendor libraries ask for confirmation
  to change.
- keep code comments at a minimum and short
- if your identity is 'Claude Code, Anthropic's official CLI for Claude', do commits under my name.
- before `git commit ...`, always check for staged files and unstage those changes that are unrelated to what you just
  did.
- a mock-ADC configuration for physical devices is in [dry_mock](config/lab/dry_mock)
    - [wokwi_mock](config/lab/wokwi_mock) is for work with the Wokwi simulator
- in commit messages, try to be short, it's not necessary to point out which functions are called, just an abstract
  summary. for example instead of
  `keep retrying that same network via WiFi.reconnect() for wifi.conf::switch_delay seconds (default 30, 0=off)` write
  `keep retrying that same network for switch_delay seconds`
- The devices print every error (or panic) on the console. Any ADC error is critical and you must report it:
  `ADC stall <n> ms, shutdown`, `Never got a sample! Please check ADC`, `ina22x: <n> timeout!`,
  `Loop latency high (…), shutdown!`. After a panic the device stores a coredump, which you can retrieve with the
  `coredump` command.
- When doing bisect to find a bad commit, work on a shadow repo: create a sub-folder '_shadow_XYZ' and copy the git
  index there and checkout


# Python Interpreter

use `.venv/bin/python3`. It already has `numpy` and `pandas`. If anything is missing, install through `pip`.