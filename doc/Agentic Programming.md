*this document is an LLM generated placeholder*

# Agentic Programming with Fugu MPPT Firmware

This document describes how the firmware is designed to be driven by an LLM agent or scripted
automation — from rapid host-side iteration to safe remote interaction with live converters.

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

See [Console.md](Console.md) for the full command reference.

### Batch / stdin mode

`fugu_console.py --stdin` (or auto-activated when stdin is a pipe) accepts newline-separated
commands over a **single connection** and tags each reply `=== <cmd> ===`. An agent can pipe a
heredoc and parse sections deterministically:

```bash
python etc/fugu_console.py -p /dev/cu.usbmodem1101 --stdin <<'EOF'
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

t = SerialTransport("/dev/cu.usbmodem1101", baud=115200)
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
broadcasts, NAT-forwarded telnet endpoints, and BLE advertisements, printing connection strings
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
  passives. An agent adjusts these via `set-config vconv.conf …` without a rebuild.
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
| `ripple-freq-test.cpp` | Adaptive inverter-ripple frequency detector (incl. real `fry` Vout capture) |
| `dawn-test.cpp` | Dawn replay: production filter pipeline driven through the vconv PV plant |
| `plot-test.cpp` | `plot.h` Series rendering at small/degenerate N (heap-safety) |
| `service-test.cpp` | `ServiceManager` state machine (boot without `wifi.conf`) |

An agent can run these as part of a pre-flash verification step, catching physics regressions
before any hardware is involved.

---

## 5. On-Target Unity Tests

For correctness that requires real hardware timing (ADC DMA, FreeRTOS tasks, MCPWM):

```bash
RUN_TESTS=1 idf.py -B build-tests build flash monitor
# or target a single entry point:
MAIN_SRC=../test/test_vconv.cpp idf.py build
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

Run `python etc/e2e-test/run_e2e.py --list` for the authoritative cluster/test mapping and each
test's exact transport and setup requirements.

Run the non-destructive console cluster against any live device:

```bash
python etc/e2e-test/run_e2e.py --cluster console --serial /dev/cu.usbmodem1201
python etc/e2e-test/run_e2e.py --cluster console --telnet 192.168.4.2:23
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
ESPPORT=/dev/cu.usbmodem101 FUZZ_DURATION=300 python etc/e2e-test/fuzz_extreme.py
```

`fuzz_sequences.py` tests structured sequences (service on/off/restart cycles, OTA round-trip,
config edit + read-back) rather than pure random chaos.

---

## 7. OTA Automation (`etc/ota.py`)

`ota.py` discovers devices (mDNS scope broadcast + NAT router scan), serves the build binary
over HTTP on port 9000 (using this host's reachable IP so it works through NAT), and tells each
matching device to pull and flash the new image.

Agent-safe workflow:

```bash
./ota.sh                          # build, then:
python etc/ota.py -n              # dry-run: show which devices would update + version delta
python etc/ota.py -m flat         # update only 'flat' after confirming
python etc/ota.py -m flat -f      # force even if version matches
```

The `-n` / `--dry-run` flag is the agent's first move: confirm the target device, current
version, and what would change before committing. The before/after version table is printed at
the end of a live run for verification.

OTA archives the flashed ELF automatically (via `idf_ext.py` + `etc/idf-devtools/elf_archive.py`)
so coredumps from any subsequently-flashed build can always be symbolicated — even months later.

---

## 8. Live Telemetry and Observability

Beyond the console, the firmware pushes structured data to external systems an agent can query:

- **InfluxDB** (UDP line protocol): every MPPT cycle's Vin, Iin, Vout, Iout, power, duty, MPPT
  phase, charger state, BMS cell data. Measurement `mppt` at the configured broker. Grafana
  dashboards show real-time and historical behavior, so an agent can validate a parameter change
  by checking the time series rather than parsing console text.
- **`sensor avg`**: one compact line of EWM averages — fast polling without opening a full
  telemetry session.
- **Scope service**: raw ADC samples streamed over TCP for noise/ripple analysis. Used by
  `etc/scope_client/` scripts.
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
set-config coil.conf L0 50
set-config charger.conf cv_eoc 3.53
set-config limits.conf iout_max 35
conf-check          # report unknown/obsolete keys
get-config charger.conf
```

An agent can iteratively tune, verify effects via `sensor avg` or telemetry, and persist changes
without a rebuild or reflash cycle. The HTML config editor (`etc/config-tool/conf-editor.html`)
provides a UI backed by the same `set-config`/`get-config` protocol.

---

## 10. Safety Guardrails for Agent Use

Several mechanisms make it safer to let an agent drive the firmware:

- **`-n` dry-run on OTA**: discover + version-check without flashing. Always `-n` first.
- **`-m <name>` OTA targeting**: never update all devices by accident.
- **Protection stack**: over-voltage, over-current, under-voltage, and loop-latency watchdogs
  cut the converter independently of software. An agent that issues a reckless `dc 999` command
  will trigger protection before hardware damage.
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

---

## 11. Putting It Together: Typical Agent Flows

### Tune a parameter, validate on vconv, then push to a real converter

1. Flash a vconv build to a bench ESP32 (`config/lab/vconv_mock`).
2. Adjust `vconv.conf` and charger/mppt conf via `set-config`.
3. Run `etc/e2e-test/run_e2e.py --cluster mock` — physics tests pass on the virtual plant.
4. Run host-side `test/host-stub/vconv-test` — unit tests pass.
5. OTA the same firmware to the real converter with `ota.py -n` then `ota.py -m <name>`.
6. Watch InfluxDB or poll `sensor avg` to confirm behavior.

### Reproduce and fix a crash on a remote device

1. `python etc/fugu_console.py --mqtt <broker> --mqtt-port 1882 --name flat -c "coredump info"`
   — confirm a dump is present (the `--name` selects the device's topic on the broker).
2. `python etc/fugu_console.py --mqtt <broker> --mqtt-port 1882 --name flat --coredump get`
   — stream the dump and write `coredump.bin` (use serial/telnet/MQTT, not BLE — BLE truncates).
3. `python etc/idf-devtools/elf_archive.py decode coredump.bin` — symbolicate against archived ELF.
4. Apply fix, build, `ota.py -n -m <name>`, confirm version, `ota.py -m <name>`.
5. Re-run step 1 — confirm the new run is clean.

### Fuzz the input parser before a release

```bash
ESPPORT=/dev/cu.usbmodem101 FUZZ_DURATION=600 FUZZ_POOL=safe \
    python etc/e2e-test/fuzz_extreme.py
```

Exit 0 = device alive. Exit 2 = panic seen — the trigger command and rolling log are printed.

---

## 12. Keeping the Agent in the Loop with a Live Converter

The vconv path above avoids real hardware entirely, but the firmware is also designed so an
agent can observe, influence, and verify a **physical converter** (fry, flat) without standing
next to it.

### Remote access stack

| Layer | Mechanism | When to use |
|---|---|---|
| Primary | telnet via NAT router (:232–:235) | WiFi up, NAT reachable |
| Fallback 1 | MQTT console (`--mqtt <broker> --mqtt-port 1882 --name <dev>`) | telnet unreachable (NAT wedged) |
| Fallback 2 | BLE NUS via ESPHome proxy (`--ble-proxy 192.168.1.231`) | WiFi down, BLE range |
| Last resort | SSH to havan + `tail pv/fugu_console.log` | read-only observation |

Always confirm the device with `hostname` first — NAT port mappings are not static.

### Read-only observation (always safe)

An agent can continuously tail state without touching the control loop:

```bash
# poll sensor averages every 5 seconds
while true; do
    python etc/fugu_console.py --ip 192.168.1.231:232 -c "sensor avg"; sleep 5
done
```

`rt-stats`, `tasks`, `mem`, `bootinfo`, `status`, and `coredump info` are all read-only and
safe on a live converter. InfluxDB telemetry provides a passive view without any console round-trip.

### Config changes (low risk, instantly reversible)

`set-config` / `get-config` / `conf-check` edit the littlefs partition in place without
rebooting. Changes take effect on the next parameter re-read cycle (charger: ~1 s; most others:
at the next `svc restart` or reboot). An agent can:

1. Record the current value with `get-config <file> <key>`.
2. Apply the change with `set-config`.
3. Observe the effect via telemetry or `sensor avg`.
4. Revert with `set-config <file> <key> <original>` if the effect is wrong.

This loop is fast and safe because the protection stack (OV/OC/UV/loop-latency watchdogs) is
always active and cuts the converter independently of config.

### PWM commands (handle with care)

`dc <duty>`, `+N`, `-N`, `sweep`, and `mppt` directly manipulate the half-bridge. Safe use:

- Only drive these in **manual PWM mode** (`dc <duty>` engages it; `mppt` exits it).
- Keep `+N` steps small (≤ 5) and watch Iin — large positive jumps cause current transients.
- Protection cuts out at `iout_max` and `vout_max`; the converter stops and backs off.
- `sync off` (diode emulation) is safer than `sync forced` (no reverse-current check).
- `measure-coil l0` / `measure-coil ls` uses a controlled DCM sweep and restores MPPT when
  done — it is the intended on-device calibration path, not raw PWM stepping.

An agent should validate PWM commands on a vconv build first, then apply the same sequence to
the live unit with telemetry open.

### OTA to a live converter

OTA halts the converter and ADC during the flash write (~30 s). For fry and flat:

1. **Validate on vconv first** — same firmware image, different littlefs config.
2. **`ota.py -n -m <name>`** — confirm the target device, current version, and image version.
3. **Watch InfluxDB** for the version field flip and healthy resumption of MPPT within ~60 s.
4. **Check logs** for `ADC error`, `Loop latency high`, or panic markers — an agent should grep
   the device log (via `ssh havan.local tail pv/fugu_console.log`) after every OTA.
5. OTA rollback is active: if `setup()` hangs for >30 s, the boot watchdog restarts into the
   prior slot. An agent that sees the device not come back after 90 s should check the version
   — if it reverted, the new image has a bug.

### Automated regression check after an OTA

```python
from fugu.transport import SocketTransport
from fugu.console import Console
import time, re

c = Console(SocketTransport("192.168.1.231", 232), eol="\r\n")
# wait for the device to come back
assert c.wait_ready(probe="mem", timeout=90), "device did not recover"

# confirm ADC is healthy (climbing N= count, no 'ADC error' in recent lines)
reply = c.command("sensor avg", timeout=3.0)
assert reply.ok and re.search(r"vin=\d", reply.text), "sensor avg failed"

# check for panic marker in device log (host-side)
import subprocess
log = subprocess.check_output(["ssh", "havan.local", "tail -n 100 pv/fugu_console.log"],
                              text=True)
for marker in ("ADC error", "Guru Meditation", "Backtrace:", "assert failed"):
    assert marker not in log, f"found panic marker: {marker}"

print("post-OTA checks passed")
```

---

## Related Documents

- [Console.md](Console.md) — full command reference
- [doc/dev-notes/Debugging.md](dev-notes/Debugging.md) — coredump, ELF archive, peek
- [doc/superpowers/specs/2026-05-23-virtual-converter-design.md](superpowers/specs/2026-05-23-virtual-converter-design.md) — VirtualConverter architecture spec
- [doc/superpowers/specs/2026-05-24-wifi-outage-e2e-test-design.md](superpowers/specs/2026-05-24-wifi-outage-e2e-test-design.md) — WiFi e2e test design
- [doc/Automated Bench Tests.md](Automated%20Bench%20Tests.md) — on-target test setup
- [doc/Peek Command.md](Peek%20Command.md) — live memory introspection
