---
title: Agentic programming
sidebar_position: 10
---

*this document is an LLM generated placeholder*

# Agentic Programming with Fugu MPPT Firmware

This document describes how the firmware is designed to be driven by an LLM agent or scripted
automation — from rapid host-side iteration to safe interaction with real converters.

---

## 1. Unified Text Console

The same line-oriented command protocol is served on every transport: **UART, USB-CDC, telnet,
MQTT, and BLE NUS**. A command is a plain text line terminated with `\n` or `\r`; the device
always replies with `OK: <cmd>` on success or `ERR: <cmd>` on failure — unambiguous, parseable,
and transport-agnostic.

This matters for agents because:

- **No transport lock-in.** An agent can reach any device — lab bench over USB, field unit over
  BLE or MQTT — without changing its command logic.
- **Machine-readable replies.** `OK:`/`ERR:` are stable markers; the reply body is human-readable
  but structured enough to regex-parse (e.g. `sensor avg` emits `sens: vin=… iout=… …` on one
  line).
- **Live config editing.** `set-config <file> <key> <value>` / `get-config` / `conf-check` let
  an agent tune parameters and verify them without reflashing.
- **Introspection without a debugger.** `peek <addr>` reads live memory; `fugu_console.py` extends
  this with DWARF-backed symbol resolution (`peek <symbol>[.field][+offset]` → numeric address →
  device read) and `peek-struct <symbol>` for typed object dumps. `tasks` shows stack headroom;
  `rt-stats` shows CPU load.

See [Console commands](../reference/console.md) for the full command reference.

### Batch / stdin mode

`fugu_console.py --stdin` (or auto-activated when stdin is a pipe) accepts newline-separated
commands over a **single connection** and tags each reply `=== <cmd> ===`. An agent can pipe a
heredoc and parse sections deterministically:

```bash
python etc/fugu_console.py -p <serial-port> --stdin <<'EOF'
hostname
mem
svc list
sensor avg
EOF
```

`-c` runs one-or-more commands over a single connection with the same tagging. Blank lines and
`#` comments are skipped. This is the preferred agent-facing path: one TCP/serial connect, ordered
replies, no interactive TTY.

---

## 2. `fugu_console.py` and the `fugu` Package

`etc/fugu_console.py` is the host-side CLI, but the **transport and console mechanics live in a
separate library** (`etc/fugu/`, github.com/fl4p/fugu-py). This separation makes the same
primitives reusable in test scripts and fuzzers without spawning a subprocess.

```python
from fugu.transport import SerialTransport
from fugu.console import Console

t = SerialTransport("<serial-port>", baud=115200)
c = Console(t, eol="\r\n")
reply = c.command("sensor avg", timeout=2.0)
assert reply.ok
```

`Console.command()` returns a `Reply` with `.ok`, `.timed_out`, `.rejected`, and the response
text — structured enough for an agent to act on without screen-scraping.

**Transport implementations** (`SerialTransport`, `SocketTransport`, `BleTransport`,
`EspHomeBleTransport`, `MqttTransport`) share the same interface. An agent switches from USB to
BLE by changing one constructor call; the command loop is identical.

**Device discovery** — called with no arguments, `fugu_console.py` scans USB ports, mDNS scope
broadcasts, configured telnet endpoints, and BLE advertisements, printing connection strings
for every live device. An agent can parse this output to auto-select a target without hardcoding
a port.

---

## 3. VirtualConverter (vconv)

`src/sim/vconv.h` is a **pure C++ physics model of a synchronous buck converter** — no Arduino,
no FreeRTOS, no ESP-IDF headers. It simulates:

- A single-diode PV source (Isc, Voc, fill-factor k)
- Battery/load (Vbat, Rbat)
- Passives (L, Cin, Cout)
- PWM gate timing (HS on-count, LS on-count, freq)
- Pluggable AC ripple models (sine inverter, |sin| rectifier, spiky China-inverter pulse)

With `CONFIG_FUGU_WITH_VCONV=y` the firmware replaces the real PWM driver with `PWM_VConv` and
the real ADC with `ADC_VConv`. The **complete control stack** — MPPT tracker, PD controllers,
charger, protection — runs against the software plant on a real ESP32, with no physical power
stage.

```
┌─────────────────┐  update_pwm()  ┌──────────────────────┐  getSample()  ┌───────────────┐
│   PWM_VConv     │ ─────────────> │   VirtualConverter   │ <──────────── │   ADC_VConv   │
│ (PwmDriver shim)│                │   (g_vconv singleton)│               │ (AsyncADC shim│
└─────────────────┘                └──────────────────────┘               └───────────────┘
```

Why this is agentic-friendly:

- **Safe to experiment on.** Changing MPPT parameters, protection thresholds, or diode-emulation
  timing has no electrical consequence. An agent can iterate tuning loops on a bench ESP32 at
  full speed before touching a live converter.
- **Reproducible.** The model is deterministic: identical inputs produce bit-identical outputs.
  Failures are repeatable; no flaky hardware.
- **Self-contained.** `vconv.conf` (in `config/lab/vconv_mock/`) sets PV curve, battery, and
  passives. An agent adjusts these via `set-config vconv.conf …` without a rebuild; `restart` to
  apply.
- **Observable.** The same `sensor avg`, `mppt`, `status`, `rt-stats`, and telemetry paths that
  work on a real converter work with vconv — including InfluxDB push, so time-series data from
  automated sweeps lands in the same dashboard.

The vconv build runs on both ESP32-S3 (`vconv_mock`) and classic ESP32 (`vconv_mock_esp32`),
holding MPP around 815 W in lab conditions, suitable for control-loop validation.

---

## 4. Host-Side Stubs (`test/host-stub/`)

For the lowest-overhead feedback loop — no flash, no device, instant iteration — many firmware
modules compile and run directly on a **macOS/Linux host** using thin stubs:

```
test/host-stub/
├── Arduino.h          # millis(), delay(), pinMode(), … no-ops
├── esp_log.h          # ESP_LOGx → printf
├── freertos/          # FreeRTOS types (no scheduler)
├── mock.h             # MOCK define, simple assert wrappers
└── arduino-shim/      # full arduino-esp32 header surface (types only)
```

Build and run a physics test without touching a device:

```bash
clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src \
    -o /tmp/vconv-test test/host-stub/vconv-test.cpp src/sim/vconv.cpp \
    && /tmp/vconv-test
```

Tests in `test/host-stub/` cover:

| File | What it tests |
|---|---|
| `vconv-test.cpp` | VirtualConverter: CCM/DCM physics, energy conservation, PV model, ripple models (21 cases A–U) |
| `converter-test.cpp` | `SynchronousConverter` buck math (diode emulation timing) |
| `integrator-test.cpp` | Integrator / coulomb-counter arithmetic |
| `mcpwm-timing-test.cpp` | MCPWM timing: prescaler / resolution selection (`bestTiming`) |
| `scope-test.cpp` | Scope streaming framing |
| `ripple-freq-test.cpp` | Adaptive inverter-ripple frequency detector (incl. a real Vout capture from a converter with an inverter on its DC bus) |
| `dawn-test.cpp` | Dawn replay: production filter pipeline driven through the vconv PV plant |
| `plot-test.cpp` | `plot.h` Series rendering at small/degenerate N (heap-safety) |
| `service-test.cpp` | `ServiceManager` state machine (boot without `wifi.conf`) |

An agent can run these as part of a pre-flash verification step, catching physics regressions
before any hardware is involved.

---

## 5. On-Target Unity Tests

For correctness that requires real hardware timing (ADC DMA, FreeRTOS tasks, MCPWM):

:::danger Bare board only
The Unity suite drives GPIO 1, 2, 4-9 and **21** as outputs. 21 is the high-side gate input on Fugu2
boards; the ISR tests pulse it for about 1 µs and leave it LOW, but it still switches the gate. `idf.py flash`
also overwrites the littlefs config with
`config/lab/dry_mock`. Run it on a dev board or a Fugu board with the power stage unpowered (no PV, no
battery). Use `app-flash` if the littlefs config must be kept.
:::

```bash
RUN_TESTS=1 idf.py -B build-tests build flash monitor
# or target a single entry point:
MAIN_SRC=../test/main_ads_rate.cpp idf.py build
```

Tests run on the device, report `PASS`/`FAIL`/`SKIP` via the console, and exit. The e2e harness
can drive this flow and collect results over serial.

---

## 6. E2E Test Harness (`etc/e2e-test/`)

`run_e2e.py` is a **cluster runner** that groups tests by their hardware requirements and skips
those whose prerequisites aren't met:

| Cluster | Requirement | Tests |
|---|---|---|
| `console` | Any device, non-destructive | `test_nettools.py`, `test_stdin_batch.py`, `test_mqtt_cmd_input.py`, `test_console_plan.py` |
| `mock` | Mock / fake-ADC build | `test_console_plan.py --mock`, `influx_test.py` |
| `destructive` | Bench unit only (panics/reboots) | `test_coredump.py`, plus `fuzz_sequences.py` + `fuzz_extreme.py` with `--with-fuzz` |
| `power` | Real converter + coil (sun/headroom), drives the half-bridge | `test_measure_coil.py` |
| `wifi` | Controllable AP/router rig | `test_wifi_off_timeout.py`, `test_wifi_reconnect_storm.py`, `test_wifi_outage.py` (stick + roam modes), `test_wifi_outage_service_recovery.py` |

The runner exits 1 on any FAIL and 2 when nothing ran (every test skipped), so an all-SKIP run is never read as
a pass. Run `python etc/e2e-test/run_e2e.py --list` for the authoritative cluster/test mapping and each
test's exact transport and setup requirements.

Run the non-destructive console cluster against any live device:

```bash
python etc/e2e-test/run_e2e.py --cluster console --serial <serial-port>
python etc/e2e-test/run_e2e.py --cluster console --telnet <device-ip>:23
```

`_harness.py` provides shared primitives used by all test modules: `Results` (PASS/FAIL/SKIP
bookkeeping), `wait_for(predicate, timeout)`, `EventLog` (timestamped parsed-event ring),
`Recorder` (panic-marker detection), and `PANIC_MARKERS` — the set of strings that indicate a
crash in the console stream.

### Fuzz testing

`fuzz_extreme.py` fires random commands (NaN/Inf arguments, garbage tokens, bursts with no
pauses) for a configurable duration and fails if any `PANIC_MARKERS` appear or the device stops
responding. It discovered a real bug: `wifi off N` over telnet triggered a use-after-free in
lwIP (the netif was torn down under the socket, and `UART_LOG` then mirrored to it reentrant).

```bash
ESPPORT=<serial-port> FUZZ_DURATION=300 python etc/e2e-test/fuzz_extreme.py
```

`fuzz_sequences.py` tests structured sequences (service on/off/restart cycles, OTA round-trip,
config edit + read-back) rather than pure random chaos.

---

## 7. OTA Automation (`etc/ota.py`)

`ota.py` discovers devices (mDNS scope broadcast, plus configured fallback hosts), serves the build
binary over HTTP on port 9000 (using this host's IP as the device sees it, so it also works through
a NAT), and tells each
matching device to pull and flash the new image.

Agent-safe workflow:

```bash
idf.py build                          # build only
python3 etc/ota.py -n -m <name>       # dry-run, scoped: target + version delta
python3 etc/ota.py -m <name>          # live, scoped to hostnames matching <name>
python3 etc/ota.py -m <name> -f       # only if the same version must be re-pushed
```

:::danger `./ota.sh` OTAs right after the build
`./ota.sh <args>` builds, then runs `etc/ota.py <args>`. It refuses an unscoped run (exit 2 without `-n` or
`-m`), but `./ota.sh -m <name>` pushes to every matching device immediately. Run `./ota.sh -n -m <name>` first.
:::

The `-n` / `--dry-run` flag is the agent's first move: confirm the target device, current
version, and what would change before committing. The before/after version table is printed at
the end of a live run for verification.

OTA archives the flashed ELF automatically (`etc/ota.py` calls `etc/idf-devtools/elf_archive.py`;
`idf_ext.py` does the same for serial flashes)
so coredumps from any subsequently-flashed build can always be symbolicated — even months later.

---

## 8. Live Telemetry and Observability

Beyond the console, the firmware pushes structured data to external systems an agent can query:

- **InfluxDB** (UDP line protocol, measurement `mppt`): up to 50 points/s while Wi-Fi, an Influx
  host and time sync are up: Vin (`Ui`), Vout (`Uo`), power-side current `I`, `P`, energy, HS duty.
  MPPT state, temperatures, loop lag and LS duty come on a decimated subset. For sample-level
  transients use the scope service. Grafana
  dashboards show real-time and historical behavior, so an agent can validate a parameter change
  by checking the time series rather than parsing console text.
- **`sensor avg`**: one compact line of EWM averages — fast polling without opening a full
  telemetry session.
- **Scope service**: raw ADC samples streamed over TCP for noise/ripple analysis. Used by
  `etc/adcscope/` (the adcscope submodule) and `etc/filter-studies/` scripts.
- **`coredump get`**: streams the on-flash panic dump as base64 over the console. An agent can
  retrieve a crash dump without physical access and decode it host-side with the archived ELF.
  Pull over serial/telnet/MQTT — the BLE transport truncates the stream past ~6 KB, so it's not
  reliable for a full dump.

---

## 9. Config System

All hardware parameters (pin assignments, sensor scaling, voltage/current limits, coil
inductance, MPPT settings, charger termination) live in flat `key=value` files on the device's
littlefs partition, editable at runtime:

```
set-config coil.conf L0 50e-6
set-config charger.conf cv_eoc 3.53
set-config limits.conf iout_max 35
conf-check          # report unknown/obsolete keys
get-config charger.conf
```

`set-config` only rewrites the file; boot-time confs (including `limits.conf`) take effect after
`restart`, see below. An agent can iteratively tune, restart, verify effects via `sensor avg` or
telemetry, and persist changes without a rebuild or reflash cycle. The HTML config editor (`etc/config-tool/conf-editor.html`)
provides a UI backed by the same `set-config`/`get-config` protocol.

---

## 10. Safety Guardrails for Agent Use

Several mechanisms make it safer to let an agent drive the firmware:

- **`-n` dry-run on OTA**: discover + version-check without flashing. Always `-n` first.
- **`-m <name>` OTA targeting**: never update all devices by accident.
- **Protection stack (software)**: the RT loop checks Vin/Vout over-voltage and Iin/Iout
  over-current on every new sample, plus temperature, and calls `stopAndBackoff` on a violation.
  These are firmware checks on sampled values: they run only while the RT loop runs, react one
  sample late at best, and pause during flash-cache-disabled windows. Manual PWM (`dc`) disables
  the supply-UV cutout and the loop-latency watchdog. A hardware OST brake exists only for the
  MCPWM driver, and only when `board.conf::pwm_fault_pin` is wired and set (default off).
  Protection does not guarantee against damage: bound every `dc`/`+N` command and use a
  current-limited supply on the bench.
- **OTA rollback**: `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` + a 30-second boot watchdog in
  `setup()`. A bad firmware image that hangs during setup reverts to the previous slot on the
  next reset — no physical recovery needed.
- **`fuzz_extreme.py` "safe" pool**: dangerous commands (`adc-reset`, `adc-restart`) are
  segregated into `FUZZ_POOL=danger` so the default run explores everything else without
  wedging the ADC.
- **Panic detection via `PANIC_MARKERS`**: the shared harness module defines the strings that
  indicate a crash; any test or fuzzer checking for these exits with a non-zero status code.
- **`coredump info`**: tells the agent whether a crash is pending before it starts work, and
  `coredump erase` clears it after retrieval.

### Working with a real converter

The firmware drives a real half-bridge; treat state-changing commands accordingly.

- **Confirm the target first.** Check `hostname` (and `ip`) before any state-changing command —
  port numbers, IPs and forwarded mappings are not stable identifiers.
- **Read-only commands:** `mem`, `bootinfo`, `status`, `sensor avg` and `coredump info`. `rt-stats` and
  `tasks` walk the FreeRTOS task list and can starve the continuous-ADC DMA on a busy configuration; use them
  sparingly on a converting device. InfluxDB telemetry gives a passive view without any console
  round-trip. While diagnosing, stick to these.
- **Config changes are reversible.** `set-config` / `get-config` / `conf-check` edit the littlefs
  partition in place without rebooting. `set-config` only rewrites the file. `board`, `sensor`,
  `limits`, `coil`, `converter`, `tracker`, `charger` and `vconv` confs are read once at boot: send
  `restart` before evaluating the change (a lowered limit is not in force until then). Service confs
  (`mqtt`, `tele`, `ftp`, `ble`, …) are re-read by `svc restart <name>`. Record the current value
  with `get-config <file> <key>`, apply the change, restart, observe the effect via telemetry or
  `sensor avg`, and revert with `set-config <file> <key> <original>` if it is wrong.
- **PWM commands need care.** `dc <duty>`, `+N`, `-N`, `sweep` and `mppt` directly manipulate the
  half-bridge:
    - Only drive these in **manual PWM mode** (`dc <duty>` engages it; `mppt` exits it).
    - Keep `+N` steps small (≤ 5) and watch Iin — large positive jumps cause current transients.
    - Protection cuts out at `iout_max` and `vout_max`; the converter stops and backs off.
    - `sync off` (diode emulation) is safer than `sync forced` (no reverse-current check).
    - `measure-coil l0` / `measure-coil ls` uses a controlled DCM sweep and restores MPPT when
      done — it is the intended on-device calibration path, not raw PWM stepping.

  Validate PWM command sequences on a vconv build first, then apply them to the real unit with
  telemetry open.
- **OTA to a real converter** halts the converter and ADC during the flash write (~30 s). Validate
  on vconv first, run `ota.py -n -m <name>` to confirm target and versions, then watch telemetry
  for the version flip and healthy resumption of MPPT within ~60 s. Check the device log for
  `ADC error`, `Loop latency high` or panic markers after every OTA. If `setup()` hangs for >30 s
  the boot watchdog restarts into the prior slot; if the device reports the old version after
  ~90 s, the new image has a bug.

---

## 11. Putting It Together: Typical Agent Flows

### Tune a parameter, validate on vconv, then push to a real converter

1. Flash a vconv build to a bench ESP32 (`config/lab/vconv_mock`).
2. Adjust `vconv.conf` and charger/mppt conf via `set-config`.
3. `python etc/e2e-test/run_e2e.py --cluster mock --serial <port>` and require `N passed, 0 failed`
   (an all-skipped run also exits 0): console-plan and Influx checks against the mock build.
4. Physics: the host build from section 4 (`/tmp/vconv-test`).
5. Rebuild for the target hardware with `CONFIG_FUGU_WITH_VCONV=n` and the board's PWM driver, in a
   separate build dir/project root (see [Build](build.md)). Then `ota.py -n -m <name>`, then
   `ota.py -m <name>`.

   :::danger Never OTA the vconv image to hardware
   A `CONFIG_FUGU_WITH_VCONV=y` image replaces the PWM driver with a simulator: the gate pins are never
   driven, and the "validated" behaviour was measured against a simulated plant.
   :::
6. Watch InfluxDB or poll `sensor avg` to confirm behavior.

### Reproduce and fix a crash on a remote device

1. `python etc/fugu_console.py --mqtt <broker> --mqtt-port <port> --name <dev> -c "coredump info"`
   — confirm a dump is present (the `--name` selects the device's topic on the broker).
2. `python etc/fugu_console.py --mqtt <broker> --mqtt-port <port> --name <dev> --coredump get`
   — stream the dump and write `coredump.bin` (use serial/telnet/MQTT, not BLE — BLE truncates).
3. `python etc/idf-devtools/elf_archive.py decode coredump.bin` — symbolicate against archived ELF.
4. Apply fix, build, `ota.py -n -m <name>`, confirm version, `ota.py -m <name>`.
5. Re-run step 1 — confirm the new run is clean.

### Fuzz the input parser before a release

```bash
ESPPORT=<serial-port> FUZZ_DURATION=600 FUZZ_POOL=safe \
    python etc/e2e-test/fuzz_extreme.py
```

Exit 0 = device alive. Exit 2 = panic seen — the trigger command and rolling log are printed.

---

## Related Documents

- [Console commands](../reference/console.md) — full command reference
- [Debugging](debugging/index.md) — coredump, ELF archive, peek
- [Automated bench tests](../lab/automated-bench-tests.md) — on-target test setup
- [`peek` command](debugging/peek.md) — live memory introspection
