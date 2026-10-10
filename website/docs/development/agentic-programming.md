---
title: Agentic programming
sidebar_position: 10
---

# Agentic programming with Fugu MPPT firmware

An LLM agent or scripted automation can drive the firmware, from host-side iteration to
interaction with real converters. This page describes the tools and safeguards for that work.

---

## 1. Unified text console

Every transport serves the same line-oriented command protocol: UART, USB-CDC, telnet, MQTT,
and BLE NUS. A command is a plain text line terminated with `\n` or `\r`. The device always
replies with `OK: <cmd>` on success or `ERR: <cmd>` on failure.

The shared protocol gives an agent these properties:

- The command logic is the same for every device, whether it is a bench unit over USB or a field
  unit over BLE or MQTT.
- `OK:`/`ERR:` are stable markers. The reply body is human-readable but regular enough to parse
  with a regex (e.g. `sensor avg` emits `sens: vin=… iout=… …` on one line).
- `set-config <file> <key> <value>` / `get-config` / `conf-check` let an agent tune parameters
  and verify them without reflashing.
- `peek <addr>` reads live memory without a debugger. `fugu_console.py` extends
  this with DWARF-backed symbol resolution (`peek <symbol>[.field][+offset]` → numeric address →
  device read) and `peek-struct <symbol>` for typed object dumps. `tasks` shows stack headroom;
  `rt-stats` shows CPU load.

See [Console commands](../reference/console.md) for the full command reference.

### Batch / stdin mode

Batch mode is the preferred path for agents: one TCP/serial connection, ordered replies, and no
interactive TTY. `fugu_console.py --stdin` accepts newline-separated commands over a single
connection and tags each reply `=== <cmd> ===`. The mode also activates automatically when stdin
is a pipe. An agent can pipe a heredoc and parse the sections deterministically:

```bash
python etc/fugu_console.py -p <serial-port> --stdin <<'EOF'
hostname
mem
svc list
sensor avg
EOF
```

`-c` runs one or more commands over a single connection with the same tagging. Blank lines and
`#` comments are skipped.

---

## 2. `fugu_console.py` and the `fugu` package

`etc/fugu_console.py` is the host-side CLI. The transport and console mechanics live in a
separate library (`etc/fugu/`, github.com/fl4p/fugu-py), so test scripts and fuzzers can use the
same primitives without spawning a subprocess. The following example sends one command over
serial and checks the reply:

```python
from fugu.transport import SerialTransport
from fugu.console import Console

t = SerialTransport("<serial-port>", baud=115200)
c = Console(t, eol="\r\n")
reply = c.command("sensor avg", timeout=2.0)
assert reply.ok
```

`Console.command()` returns a `Reply` with `.ok`, `.timed_out`, `.rejected`, and the response
text, so an agent can act on the result without screen-scraping.

The transport implementations (`SerialTransport`, `SocketTransport`, `BleTransport`,
`EspHomeBleTransport`, `MqttTransport`) share the same interface. An agent switches from USB to
BLE by changing one constructor call. The command loop stays identical.

Called with no arguments, `fugu_console.py` discovers devices. It scans USB ports, mDNS scope
broadcasts, configured telnet endpoints, and BLE advertisements, and prints connection strings
for every live device. An agent can parse this output to auto-select a target without hardcoding
a port.

---

## 3. VirtualConverter (vconv)

`src/sim/vconv.h` is a physics model of a synchronous buck converter in plain C++, with no
Arduino, FreeRTOS, or ESP-IDF headers. It simulates the following parts:

- A single-diode PV source (Isc, Voc, fill-factor k)
- Battery/load (Vbat, Rbat)
- Passives (L, Cin, Cout)
- PWM gate timing (HS on-count, LS on-count, freq)
- Pluggable AC ripple models (sine inverter, |sin| rectifier, spiky China-inverter pulse)

With `CONFIG_FUGU_WITH_VCONV=y`, the firmware replaces the real PWM driver with `PWM_VConv` and
the real ADC with `ADC_VConv`. The complete control stack (MPP tracker, PD controllers,
charger, protection) runs against the software plant on a real ESP32, with no physical power
stage. The diagram shows how the two shims connect to the model:

```
┌─────────────────┐  update_pwm()  ┌──────────────────────┐  getSample()  ┌───────────────┐
│   PWM_VConv     │ ─────────────> │   VirtualConverter   │ <──────────── │   ADC_VConv   │
│ (PwmDriver shim)│                │   (g_vconv singleton)│               │ (AsyncADC shim│
└─────────────────┘                └──────────────────────┘               └───────────────┘
```

The simulated plant has these properties for agent use:

- Changing MPPT parameters, protection thresholds, or diode-emulation timing has no electrical
  consequence, so an agent can iterate tuning loops on a bench ESP32 at full speed before
  touching a live converter.
- The model is deterministic: identical inputs produce bit-identical outputs, so failures are
  repeatable.
- `vconv.conf` (in `config/lab/vconv_mock/`) sets the PV curve, battery, and passives. An agent
  adjusts these via `set-config vconv.conf …` without a rebuild, and `restart` applies them.
- The `sensor avg`, `mppt`, `status`, `rt-stats`, and telemetry paths work as on a real
  converter. This includes the InfluxDB push, so time-series data from automated sweeps lands in
  the same dashboard.

The vconv build runs on both ESP32-S3 (`vconv_mock`) and classic ESP32 (`vconv_mock_esp32`) and
holds MPP around 815 W in lab conditions, which is enough for control-loop validation.

---

## 4. Host-side stubs (`test/host-stub/`)

Host-side stubs give the shortest feedback loop: no flashing and no device. Many firmware modules
compile and run directly on a macOS/Linux host using thin stubs. The stub directory contains
these files:

```
test/host-stub/
├── Arduino.h          # millis(), delay(), pinMode(), … no-ops
├── esp_log.h          # ESP_LOGx → printf
├── freertos/          # FreeRTOS types (no scheduler)
├── mock.h             # MOCK define, simple assert wrappers
└── arduino-shim/      # full arduino-esp32 header surface (types only)
```

The following command builds and runs a physics test without touching a device:

```bash
clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src \
    -o /tmp/vconv-test test/host-stub/vconv-test.cpp src/sim/vconv.cpp \
    && /tmp/vconv-test
```

The tests in `test/host-stub/` cover the following areas:

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

An agent can run these before flashing to catch physics regressions without hardware.

---

## 5. On-target Unity tests

The on-target Unity tests check correctness that requires real hardware timing (ADC DMA,
FreeRTOS tasks, MCPWM).

:::danger Bare board only
The Unity suite drives GPIO 1, 2, 4-9 and **21** as outputs. 21 is the high-side gate input on Fugu2
boards; the ISR tests pulse it for about 1 µs and leave it LOW, but it still switches the gate. `idf.py flash`
also overwrites the littlefs config with
`config/lab/dry_mock`. Run it on a dev board or a Fugu board with the power stage unpowered (no PV, no
battery). Use `app-flash` if the littlefs config must be kept.
:::

The following commands build and flash the test suite, or build a single entry point:

```bash
RUN_TESTS=1 idf.py -B build-tests build flash monitor
# or target a single entry point:
MAIN_SRC=../test/main_ads_rate.cpp idf.py build
```

Tests run on the device, report `PASS`/`FAIL`/`SKIP` via the console, and exit. The e2e harness
can drive this flow and collect results over serial.

---

## 6. E2E test harness (`etc/e2e-test/`)

`run_e2e.py` is a cluster runner. It groups tests by their hardware requirements and skips
those whose prerequisites aren't met. The clusters are:

| Cluster | Requirement | Tests |
|---|---|---|
| `console` | Any device, non-destructive | `test_nettools.py`, `test_stdin_batch.py`, `test_mqtt_cmd_input.py`, `test_console_plan.py` |
| `mock` | Mock / fake-ADC build | `test_console_plan.py --mock`, `influx_test.py` |
| `destructive` | Bench unit only (panics/reboots) | `test_coredump.py`, plus `fuzz_sequences.py` + `fuzz_extreme.py` with `--with-fuzz` |
| `power` | Real converter + coil (sun/headroom), drives the half-bridge | `test_measure_coil.py` |
| `wifi` | Controllable AP/router rig | `test_wifi_off_timeout.py`, `test_wifi_reconnect_storm.py`, `test_wifi_outage.py` (stick + roam modes), `test_wifi_outage_service_recovery.py` |

The runner exits 1 on any FAIL and 2 when nothing ran (every test skipped), so an all-SKIP run never reads as
a pass. Run `python etc/e2e-test/run_e2e.py --list` for the authoritative cluster/test mapping and each
test's exact transport and setup requirements.

To run the non-destructive console cluster against any live device, use serial or telnet:

```bash
python etc/e2e-test/run_e2e.py --cluster console --serial <serial-port>
python etc/e2e-test/run_e2e.py --cluster console --telnet <device-ip>:23
```

`_harness.py` provides the shared primitives that all test modules use: `Results` (PASS/FAIL/SKIP
bookkeeping), `wait_for(predicate, timeout)`, `EventLog` (timestamped parsed-event ring),
`Recorder` (panic-marker detection), and `PANIC_MARKERS`, the set of strings that indicate a
crash in the console stream.

### Fuzz testing

`fuzz_extreme.py` fires random commands (NaN/Inf arguments, garbage tokens, bursts with no
pauses) for a configurable duration. It fails if any `PANIC_MARKERS` appear or the device stops
responding. The following command runs it for 300 seconds:

```bash
ESPPORT=<serial-port> FUZZ_DURATION=300 python etc/e2e-test/fuzz_extreme.py
```

The fuzzer found a real bug: `wifi off N` over telnet triggered a use-after-free in lwIP. The
netif was torn down under the socket, and `UART_LOG` then mirrored to it reentrant.

`fuzz_sequences.py` tests structured sequences (service on/off/restart cycles, OTA round-trip,
config edit + read-back) instead of random input.

---

## 7. OTA automation (`etc/ota.py`)

`ota.py` discovers devices (mDNS scope broadcast, plus configured fallback hosts), serves the build
binary over HTTP on port 9000 (using this host's IP as the device sees it, so it also works through
a NAT), and tells each matching device to pull and flash the new image.

An agent builds first, then runs a scoped dry run before the live push:

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

Run with `-n` / `--dry-run` first to confirm the target device, current version, and what would
change. A live run prints a before/after version table at the end.

OTA archives the flashed ELF automatically, so a coredump from any build flashed since then can be
symbolicated, even months later. `etc/ota.py` and `etc/ota_ble.py` call
`etc/idf-devtools/elf_archive.py`, and `idf_ext.py` does the same for serial flashes.

---

## 8. Live telemetry and observability

Besides the console, the firmware offers these data paths that an agent can query:

- InfluxDB (UDP line protocol, measurement `mppt`): while Wi-Fi, an Influx host, and time sync
  are up, the firmware pushes up to 50 points/s of Vin (`Ui`), Vout (`Uo`), power-side current
  `I`, `P`, energy, and HS duty. MPPT state, temperatures, loop lag, and LS duty come on a
  decimated subset. For sample-level transients, use the scope service. Grafana dashboards show
  real-time and historical behavior, so an agent can validate a parameter change by checking the
  time series rather than parsing console text.
- `sensor avg` prints one compact line of EWM averages, for fast polling without a full
  telemetry session.
- The scope service streams raw ADC samples over TCP for noise/ripple analysis. It is used by
  `etc/adcscope/` (the adcscope submodule) and `etc/filter-studies/` scripts.
- `coredump get` streams the on-flash panic dump as base64 over the console. An agent can
  retrieve a crash dump without physical access and decode it host-side with the archived ELF.
  Serial, telnet and MQTT are the fastest transports. Over BLE, the command waits after each line until
  the console's 8 KB transmit buffer drains below 2 KB. A client that stalls for more than 4 s still loses the
  overflow.

---

## 9. Config system

All hardware parameters (pin assignments, sensor scaling, voltage/current limits, coil
inductance, MPPT settings, charger termination) live in flat `key=value` files on the device's
littlefs partition. An agent can edit them at runtime:

```
set-config coil.conf L0 50e-6
set-config charger.conf cv_eoc 3.53
set-config limits.conf lv_i_max 35   # side key (output current limit in a buck)
conf-check          # report unknown/obsolete keys
get-config charger.conf
```

`set-config` only rewrites the file. Boot-time confs (including `limits.conf`) take effect after
`restart`, as described in [Working with a real converter](#working-with-a-real-converter). An agent
can iteratively tune, restart, verify effects via `sensor avg` or telemetry, and persist changes
without a rebuild or reflash cycle.

The HTML config editor (`etc/config-tool/conf-editor.html`) provides a UI backed by the same
`set-config`/`get-config` protocol.

---

## 10. Safety guardrails for agent use

The following mechanisms reduce the risk of letting an agent drive the firmware:

- `-n` dry run on OTA: discover and version-check without flashing. Always run `-n` first.
- `-m <name>` OTA targeting prevents updating all devices by accident.
- Software protection stack: the RT loop checks Vin/Vout over-voltage and Iin/Iout
  over-current on every new sample, plus temperature, and calls `stopAndBackoff` on a violation.
  These are firmware checks on sampled values. They run only while the RT loop runs, react one
  sample late at best, and pause during flash-cache-disabled windows. Manual PWM (`dc`) disables
  the supply-UV cutout and the loop-latency watchdog. A hardware OST brake exists only for the
  MCPWM driver, and only when `board.conf::pwm_fault_pin` is wired and set (default off).
  Protection does not guarantee against damage. Bound every `dc`/`+N` command and use a
  current-limited supply on the bench.
- OTA rollback: `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` and a 30-second boot watchdog in
  `setup()`. A bad firmware image that hangs during setup reverts to the previous slot on the
  next reset, without physical recovery.
- `fuzz_extreme.py` "safe" pool: dangerous commands (`adc-reset`, `adc-restart`) are
  segregated into `FUZZ_POOL=danger` so the default run explores everything else without
  wedging the ADC.
- Panic detection via `PANIC_MARKERS`: the shared harness module defines the strings that
  indicate a crash. Any test or fuzzer that checks for these exits with a non-zero status code.
- `coredump info` tells the agent whether a crash is pending before it starts work, and
  `coredump erase` clears it after retrieval.

### Working with a real converter

The firmware drives a real half-bridge, so treat state-changing commands with care. The
following rules apply to a real converter:

- Confirm the target first. Check `hostname` (and `ip`) before any state-changing command.
  Port numbers, IPs, and forwarded mappings are not stable identifiers.
- While diagnosing, use only read-only commands: `mem`, `bootinfo`, `status`, `sensor avg`, and
  `coredump info`. InfluxDB telemetry gives a passive view without any console round-trip.
  `rt-stats` and `tasks` walk the FreeRTOS task list and can starve the continuous-ADC DMA on a
  busy configuration, so use them sparingly on a converting device.
- Config changes are reversible. `set-config` / `get-config` / `conf-check` edit the littlefs
  partition in place without rebooting. `set-config` only rewrites the file. The firmware reads
  the `board`, `sensor`, `limits`, `coil`, `converter`, `tracker`, `charger`, and `vconv` confs
  once at boot, so send `restart` before evaluating the change. A lowered limit is not in force
  until then. `svc restart <name>` re-reads service confs (`mqtt`, `tele`, `ftp`, `ble`, …).
  Record the current value with `get-config <file> <key>`, apply the change, restart, observe the
  effect via telemetry or `sensor avg`, and revert with `set-config <file> <key> <original>` if it
  is wrong.
- PWM commands need care. `dc <duty>`, `+N`, `-N`, `sweep`, and `mppt` directly manipulate the
  half-bridge. Follow these rules:
    - Only drive these in manual PWM mode (`dc <duty>` engages it; `mppt` exits it).
    - Keep `+N` steps small (≤ 5) and watch Iin. Large positive jumps cause current transients.
    - Protection cuts out at the output current and voltage limits (`limits.conf` `lv_i_max`/`lv_max` in a
      buck, `hv_i_max`/`hv_max` in a boost). The converter stops and backs off.
    - `sync off` (diode emulation) is safer than `sync forced` (no reverse-current check).
    - For on-device calibration, use `measure-coil l0` / `measure-coil ls` instead of raw PWM
      stepping. It uses a controlled DCM sweep and restores MPPT when done.

  Validate PWM command sequences on a vconv build first, then apply them to the real unit with
  telemetry open.
- An OTA to a real converter halts the converter and ADC during the flash write (~30 s). Validate
  on vconv first, run `ota.py -n -m <name>` to confirm the target and versions, then watch
  telemetry for the version flip and healthy resumption of MPPT within ~60 s. After every OTA,
  check the device log for `ADC error`, `Loop latency high`, or panic markers. If `setup()` hangs
  for >30 s, the boot watchdog restarts into the prior slot. If the device reports the old version
  after ~90 s, the new image has a bug.

---

## 11. Typical agent flows

### Tune a parameter, validate on vconv, then push to a real converter

1. Flash a vconv build to a bench ESP32 (`config/lab/vconv_mock`).
2. Adjust `vconv.conf` and the charger/mppt confs via `set-config`.
3. Run the console-plan and Influx checks against the mock build with
   `python etc/e2e-test/run_e2e.py --cluster mock --serial <port>`, and require `N passed, 0 failed`
   (an all-skipped run exits 2).
4. Run the host physics test from section 4 (`/tmp/vconv-test`).
5. Rebuild for the target hardware with `CONFIG_FUGU_WITH_VCONV=n` and the board's PWM driver, in a
   separate build dir/project root (see [Build](build.md)). Then run `ota.py -n -m <name>`, then
   `ota.py -m <name>`.

   :::danger Never OTA the vconv image to hardware
   A `CONFIG_FUGU_WITH_VCONV=y` image replaces the PWM driver with a simulator: the gate pins are never
   driven, and the "validated" behaviour was measured against a simulated plant.
   :::
6. Watch InfluxDB or poll `sensor avg` to confirm behavior.

### Reproduce and fix a crash on a remote device

1. `python etc/fugu_console.py --mqtt <broker> --mqtt-port <port> --name <dev> -c "coredump info"`
   confirms a dump is present (the `--name` selects the device's topic on the broker).
2. `python etc/fugu_console.py --mqtt <broker> --mqtt-port <port> --name <dev> --coredump get`
   streams the dump and writes `coredump.bin` (serial, telnet or MQTT are faster than BLE).
3. `python etc/idf-devtools/elf_archive.py decode coredump.bin` symbolicates against the archived ELF.
4. Apply the fix, build, run `ota.py -n -m <name>`, confirm the version, and run `ota.py -m <name>`.
5. Re-run step 1 to confirm the new run is clean.

### Fuzz the input parser before a release

Run the safe fuzz pool for 600 seconds:

```bash
ESPPORT=<serial-port> FUZZ_DURATION=600 FUZZ_POOL=safe \
    python etc/e2e-test/fuzz_extreme.py
```

Exit 0 means the device is alive. Exit 2 means the fuzzer saw a panic and printed the trigger
command and the rolling log.

---

## Related documents

- [Console commands](../reference/console.md): full command reference
- [Debugging](debugging/index.md): coredump, ELF archive, peek
- [Automated bench tests](../lab/automated-bench-tests.md): on-target test setup
- [`peek` command](debugging/peek.md): live memory introspection
