*this document is an LLM generated placeholder*

# Handover, 2026-11-18: 1 ms loop, rectifier offset, coil measurement, current zero, tracker, MQTT recovery

Written at the end of a long working session (2026-11-13 to 2026-11-18) so the work can be picked up
in a new one. Times are UTC. "76 µH converter" and "51 µH converter" are the two installed units; the
first samples at 512 per second (2 ms loop period), the second at about 1475 per second (0.68 ms).

Companion document: [bench-converter-work.md](bench-converter-work.md), the plan for everything that
needs a converter on a bench.

## 1. Results

| Topic | Result | State |
|---|---|---|
| RT loop gap under 1 ms on the 51 µH converter | Held a full day on one build (2026-11-15 17:53 to 11-16 18:00): worst 883 µs, through night, dawn, 684 W, dusk, and a 13-minute network outage. On the build installed now it held from 11-17 12:54 through the 11-18 dawn: worst 881 µs, no reboot. | closed, confirmed by Fab on 11-18 |
| What made the loop fit | Loop code in IRAM with a symbol check, flash auto-suspend, no allocation in the loop (counter in `bootinfo`), lock-free log rings, `rt-stats`/`tasks` walking the task list in chunks | on `main`, deployed |
| MQTT stuck after a network outage | A guard restart racing a connect left a stale "connected" flag; the service stayed Failed, the charger lost BMS data and capped at 26.8 V (about 40 % power for 85 minutes on 11-16). Fixed; the network guard re-initialises after 2 minutes and reboots after 15 minutes without MQTT, at most once per 6 hours. | on `main` (`a011c18`), deployed; the escalation has not fired on hardware yet |
| Rectifier-offset calibration (`rect-cal`) | Three rounds of fixes after two reviews: stops one step after the first clear drop, never applies a positive offset (stored as a bound), refuses when the sense readings follow the LS time, wider margin when the fit ends at a ring dip. | on `main`; works on the 76 µH converter, cannot work on the 51 µH converter (see 3.3) |
| Offset on the 76 µH converter | Stored by hand: −1312 ns (−210 ct), from a run at 113 W (learned −162 ct); a repeat with fixed code at 73 W learned −142 ct. | active |
| Coil measurement (`measure-coil l0`) | Ring-aware fit; 76 µH converter measures 76.0–77.0 µH, 51 µH converter about 51 µH (±2 % run to run). The June "oscillating values" were the DCM switch-node ring (245 kHz and 307–323 kHz, about 5 nF node capacitance). Uncertainty is now split into noise and systematic parts. | on `main`; not in the installed build (option off) |
| Output-current zero | The periodic calibration took the zero 0.1 s after a stop from high current and stored up to +0.12 A. Now taken from settled data after a 400 ms wait, previous zero kept if it does not settle. | on `main`, deployed; real tail shape unknown (bench session 2) |
| Tracker | Measured sensor noise replayed in the simulator. Kept: reset evaluated before slow-mode entry; a limiter hand-back or sweep end ends slow mode; slow mode holds 45 s without a reversal. Dropped after review: the entry step and the 4 % Vin band. On the 76 µH converter slow-mode entries fell from one per 66 s to one per 160–200 s and the enter-then-reset churn disappeared. | on `main`, deployed |
| DC-bias curve of the coil (`measure-coil bias`) | Sweep at lowered switching frequency, built and host-tested through three fix rounds and two safety reviews. Never run on hardware. | branch `coil-lowfreq` (`6324ab1`), not on `main` |
| Unit tests in QEMU | Panic before the test framework was QEMU starting with the FPU off; test-only fix. 233 tests pass; 2 software tests fail (see 4). | on `main` (`d839164`) |

## 2. State of the installed converters (2026-11-18 08:00)

| | 76 µH converter | 51 µH converter |
|---|---|---|
| Build | `fry-brk1-608-ge4cb0616` (main `e4cb061`) | same |
| Rectifier offset | −1312 ns, calibrated flag set | 0 ns, uncalibrated |
| Idle power profile | `board.conf idle_cpu_mhz=0` since 11-18 07:35 (see 3.1); BLE/telemetry/Wi-Fi idle settings at defaults | defaults; the sample-rate guard trips at dusk and keeps the CPU at 160 MHz |
| `coil.conf` L0 | 80e-6 (used as 95 %: 76.0 µH; measured 76.0–77.0) | 56e-6 (used as 53.2 µH; measured about 51) |
| Coil measurement command | not in the build | not in the build |

`main` has moved past `e4cb061` (for example `670a3ef`, a calibration noise-gate change in
`src/adc/sampling.h`, host-tested only). Check what changed in the sampler before building the next
image for the converters.

## 3. Open problems

### 3.1 Idle profile: the start gate does not release at the real idle clock (no owner)

On 11-18 the 76 µH converter, whose CPU really ran at 80 MHz overnight, did not start at dawn: it
logged `START blocked: idle profile` every 48 s from 07:21 to 07:34 and never printed
`idle profile off (start)`. Recovered by `set-config board.conf idle_cpu_mhz 0` and `restart`. The
51 µH converter is not affected because its guard refuses 80 MHz (`sample rate 1076/1478 at 80 MHz`).
Before that fix existed, leaving the profile pushed the 51 µH converter's loop gap to 1025 µs.

To do: fix the gate (the 48 s start retry may re-arm it), test with the CPU actually at the idle
clock, then remove `idle_cpu_mhz=0` on the 76 µH converter. Also seen that night on that unit:
17 MQTT re-initialisations in under seven hours and a heap low-water mark of 1.2 KB, so MQTT flaps in
the idle profile (Wi-Fi in its deepest power save).

### 3.2 A failed update on 11-18 00:46

Another session's image booted on the 76 µH converter, logged `Restart after WDT trigger` 13 s after
boot and was rolled back. Which image and why is unknown here.

### 3.3 The 51 µH converter cannot learn a rectifier offset

Its Vout and Iout readings rise with the LS on-time (+1.3 % and +60 mA) and the current never drops,
so the calibration measures the sense path, not the zero crossing. The firmware refuses there
("Iout follows LS", "sense follows LS"). Hardware work: bench session 5.

### 3.4 Not done

| Item | Needs |
|---|---|
| `rect-cal force` on the 76 µH converter with the final code in full sun, two or three runs at different power | steady sun, device lock |
| Tracker count on a full day of the current build (the 11-17 day was overcast and had three builds) | a day of log; count `Slow mode, duty` lines and those followed by `Reset maxPower` within 1 s |
| Fourth review of `coil-lowfreq` | nothing; do it before bench session 3 |
| Two failing unit tests: `test_cell_voltage_stays_stale_across_us_wrap` (sets a clock variable the check no longer reads) and `test_buck_bootstrap_min_scales_with_conf` (481 expected, 480 computed, rounding in the test) | nothing |
| A script to run the unit tests in QEMU (recipe: test build with DIO flash mode, merge to a 4 MB image, launch as `test/bl_ota_qemu.py` does) | nothing |
| Rescan trigger for moving shade (largest remaining tracking loss in the simulator: 74–90 % harvest in the moving-shade scenarios) | design |
| `etc/adcscope` headless capture is committed on branch `headless-capture` inside the submodule (`9de1eb3`), not pushed; the main repository still points at the old submodule commit | a push of the submodule, then a pointer update |

## 4. Where things are

| What | Where |
|---|---|
| Everything listed as "on main" | `main` (at `fd54970` when this was written; not all of it pushed by this session) |
| DC-bias sweep | branch `coil-lowfreq`, `6324ab1` |
| Line of builds that ran on the converters until 11-16 | branch `flash-autosus`; its content is on `main` (the last tracker commits were cherry-picked, so the branch itself does not show as merged) |
| Old experiment, no gain shown | worktree `/tmp/trk`, branch `tracker-opt`, uncommitted |
| Worktrees under `/tmp` (`dawn`, `land`, `jit`, `lowf`, `fix2`, `izero`, `lbias`, `mqttfix`, `qemufix`, `fryimg`, `coil`, `rcal`, `rtalloc`) | disposable; `/tmp` is cleaned from time to time. Only `/tmp/lowf` (branch `coil-lowfreq`) holds something not on `main`, and that is committed |
| Noise captures used for the tracker | profiles committed under `test/host-stub/noise/`; raw captures were in `/tmp/scope-fry` and `/tmp/scope-flat` and may be gone |
| Reviews | notes of the electrical reviewer under `.claude/agent-memory/ee-code-verifier/` (`project_rectcal_review_*`, `project_coil_bias_*`) |

Host tests (commands in each file's header): `test/host-stub/rect-cal-test.cpp`, `coil-fit-test.cpp`,
`coil-bias-test.cpp` (branch), `izero-test.sh`, `mqtt-service-test.cpp`, `net-guard-test.cpp`,
`service-test.cpp`, `meas-gate-test.cpp` (branch), `test/test_tracker.cpp` with a host shim, and the
simulator `test/host-stub/mppt-sim.sh` (measured noise with `SIM_NOISE=fry|flat`).

## 5. Working rules that cost time when ignored

- **Device lock before any state-changing command** on a converter:
  `~/.claude/skills/lock/lock.sh acquire <name> "<what>" --session <id>`, release afterwards. Several
  sessions updated the converters during this work; evaluations were restarted five times by updates
  nobody announced.
- **Confirm `hostname` every time.** A BLE connect by name reached a bench board twice. Use the BLE
  address or telnet through `etc/nat-tunnel.sh up`; the BLE proxy did not answer on three occasions.
- **A converter whose log stops while telemetry still arrives has lost MQTT.** Compare the age of its
  last log line with the clock on every check, not only the error grep. Read `svc list` and `bootinfo`
  (the `netguard:` line) and tell Fab.
- **Console over MQTT:** `etc/fugu_console.py --mqtt 127.0.0.1 --mqtt-port 11882 --name <unit> -c ...`
  through `ssh -f -N -L 11882:127.0.0.1:1882 havan-ts`. Send `restart` as its own call with a timeout;
  the client can hang after it.
- **Telemetry and log:** InfluxDB measurement `mppt` (fields `lag`, `heap`, `P`, `Uo`, ...); the device
  log on the log host rotates at 10 MB, so scans over more than a day need the rotated file too. In the
  idle profile a unit sends one telemetry point per 15 minutes.
- **`lag` is a latched peak**, reset at sweeps and by the firmware's own `reset-lag` two hours after
  boot. Do not run `rt-stats` on a converting unit when judging the loop gap.
- **Simulator batches run on the remote hosts** with `~/bin/offload` (tagged, stopped by tag), never on
  the Mac. Results differ slightly per host: compare only runs from one host and one tree. The default
  plant of the weather scenarios changed on 11-15 (`SIM_RING=ideal` gives the old one).
- **Images for the converters:** build from a clean worktree with a fresh `sdkconfig`, check
  `etc/rtprof_map.py --check` for both `main/rt_iram.lf` and `main/rt_iram_extra.lf` on the exact ELF
  (0 not in IRAM), simulated-converter option off, then `etc/ota_mqtt.py --name <unit> --bin <bin> -n`
  followed by `--yes`. Updates of the converters need Fab's go-ahead.
- **Implementation went to subagents, with independent review afterwards.** The reviews found real
  defects every time (a fit landing late, a console gate that could be bypassed, a guard that could not
  see the failure it was built for). Simulator results were wrong twice because the simulated sensor
  was faster than the real one.

## 6. Suggested order when resuming

1. Idle profile start gate (3.1): it currently has no owner and leaves one converter without the
   CPU part of the power saving.
2. A sunny day: `rect-cal force` runs on the 76 µH converter and the tracker count (3.4).
3. Fourth review of `coil-lowfreq`, so it is ready when a bench converter is available.
4. With a bench converter: [bench-converter-work.md](bench-converter-work.md), sessions 1 to 7 in order.
