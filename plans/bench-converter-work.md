*this document is an LLM generated placeholder*

# Work that needs a bench converter

Status: 2026-11-18. Written from the session that worked on the 1 ms loop goal, the rectifier-offset
calibration, the coil measurement, the current zero, the tracker and the MQTT recovery. Everything in
sessions 1 to 6 is waiting for a converter on a bench with a scope; there is none at the moment. None
of it should be tried first on the two installed converters. The last two sections list what does not
need the bench and how to pick the work up again.

## What the bench needs

| Item | Why |
|---|---|
| A Fugu2 board with the standard power stage, serial console attached | first runs must not happen on an installed converter |
| Current-limited supply as the "PV" source, 60–65 V, adjustable limit 3–20 A | bounds the fault current in every test; a PV string or a battery as source does not |
| A second supply or series diode if the source cannot sink current | several tests push small reverse current to the input |
| Output: small LFP pack or a stiff bus with an electronic load, fused | not a BMS-protected live pack for the first runs |
| Scope, 4 channels, 100 MHz or more | switch node, HS gate (referenced to the switch node, differential probe), bootstrap voltage, low-side gate |
| Current probe on the coil lead (DC to 1 MHz or more, 50 A) | the only direct view of coil current and reverse current |
| LCR meter or a known reference coil (optional) | independent small-signal L |
| Calibrated DC current reference (shunt and DMM, or a calibrated load) | the converter's output current reading differs from the BMS by up to about 6 % |
| Thermal camera or thermocouples (optional) | output capacitors and coil during the bias sweep |

Record for the board under test: coil part and measured `L0`, input and output capacitor parts as built,
FETs as built, whether the INA226 inputs have the filter capacitor. The saturation current of the coils
is not in this repository or in the hardware notes, and neither is the as-built output capacitor.

## Where the code is

| Work | Where | State |
|---|---|---|
| Rect-cal fixes (stop rule, bound, refusals), settled current zero, `measure-coil l0` with uncertainty | `main` | host-tested; rect-cal and `l0` ran on the installed converters |
| `measure-coil bias` (low-frequency sweep) | branch `coil-lowfreq`, `6324ab1` (not on `main`) | host-tested only, never on hardware; two safety reviews done, the third fix round is not re-reviewed |
| MQTT recovery and network-guard escalation | `main` (`a011c18`), on both installed converters since 2026-11-16 | host-tested; the escalation path has not fired on hardware yet |
| Headless scope capture (`--capture SECONDS --out DIR`) | `etc/adcscope` submodule, uncommitted | used for the noise captures on 2026-11-15 |

Build for the bench with the coil measurement enabled:

```bash
. ./idf-export.sh
idf.py -B build-mc -DSDKCONFIG=build-mc/sdkconfig \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.measure_coil" build
python3 etc/rtprof_map.py --check main/rt_iram.lf - build-mc/fugu-firmware.elf        # 0 not in IRAM
python3 etc/rtprof_map.py --check main/rt_iram_extra.lf - build-mc/fugu-firmware.elf  # 0 not in IRAM
```

Before session 3: have the third round of `coil-lowfreq` reviewed once more, then merge it into the
build used on the bench.

## Session 1: baseline of the board (about 1 hour)

Goal: know the board before any new code drives it differently.

1. Flash, provision the board config, set `limits.conf` output current limit to what the bench source can
   deliver. Confirm `hostname`, `bootinfo` (heap, `rt-alloc`, flash auto-suspend line).
2. `measure-coil l0` three times at a steady source. Expect agreement within about 1 % and a ring
   frequency in the 245–330 kHz range. Set `coil.conf::L0` if it is off by more than 10 %.
   (`measure-coil ls` is not to be run.)
3. Scope the switch node in discontinuous mode at about 1 A output. Measure the ring frequency and
   compute the node capacitance from it and `L0`. Compare with what `l0` reports (5.5 nF on the 76 µH
   board, 4.5–5.0 nF on the 51 µH board). This is the independent confirmation that the oscillation in
   the coil measurement is the switch-node ring.
4. Compare the output current reading with the reference at 2, 5, 10, 20 A. Note gain and offset.
5. Compare Vin and Vout readings with a DMM while converting and while off. On the 51 µH installed board
   Vout reads 0.3–0.45 V low only while converting.

Pass: values recorded. Nothing here changes code.

## Session 2: current zero after a loaded stop (about 1 hour)

Open question: after a stop from high current, the output current reading is disturbed for some time.
The firmware fix on `main` waits 400 ms and takes the zero from settled blocks. Two logged cases fit a
40 ms tail and a 0.2–2 s tail equally well; with a slow tail the zero is still 8–49 mA off.

1. Headless scope capture of raw Iout across a stop:
   `./etc/scope.py --capture 20 --ip <board> --port 24 --out /tmp/zero-<n>` and, during the capture,
   stop from 5, 10, 20 A (console `sweep` triggers a stop and a calibration; `dc 0` gives a plain stop).
2. At the same time scope the shunt voltage (differential) and the INA226 supply.
3. Fit the tail: amplitude and time constant versus the current before the stop.
4. Decide: is it real current, a sensor or thermal effect, or the measurement chain. If the time
   constant is above about 100 ms, the 400 ms wait and the stationarity check need new numbers
   (`src/adc/sampling.h`, test `test/host-stub/izero-test.sh` has a "known limitation" table to update).

Pass: tail shape known; either the current constants are confirmed or new ones are derived from data.

## Session 3: `measure-coil bias`, first hardware run (half a day)

What it does: switches the converter off, lowers the switching frequency in steps (39, 19.5, 13, 9.75,
7.8 kHz; lowest reachable 5.57 kHz), sweeps the duty in discontinuous mode at each step and fits one
L-versus-peak-current curve. Syntax: `measure-coil bias [fmin_khz] [imax_a] [dwell_ms]`,
`measure-coil abort`.

Reach at 65 V in, 26.5 V out: 51 µH coil 28 A at 9.75 kHz and the 30 A cap at 7.8 kHz; 76 µH coil
24 A at 7.8 kHz (30 A needs about 5.6 kHz).

### Preconditions

- Bench board only. Current-limited source, Vin at least 1.6 × Vout and at least Vout + 15 V (the
  command refuses otherwise). Start with the source limit at about 3 A.
- `coil.conf::L0` from session 1.
- At least 25 KB free heap and an 8.3 KB largest block (the command prints both and refuses below).
  Switch off telemetry and BLE if needed.
- No update tooling and no second console connected.
- Send single lowercase commands during a run. Stop with `dc 0` or `measure-coil abort`. Keep the
  source's output switch within reach.

### Steps

1. `measure-coil bias 7.8 10`. On every frequency block check on the scope:
   - bootstrap voltage stays above about 9 V (driver undervoltage lockout about 8.5 V on, 7.9 V off),
     also right at the start of a block after the zero read;
   - the high-side gate switches at full amplitude on every cycle, no missing or bursting pulses;
   - coil current never goes below zero except during the refresh pulse. Expected peak during the
     pre-charge: 1.0 A at 39 kHz, 1.3 A at 13 kHz, 1.7 A at 9.75 kHz, 2.1 A at 7.8 kHz (51 µH);
   - the low side turns off before the coil current reaches zero.
2. Trip test: during a run, force a trip (lower the output over-voltage setting or pull the source).
   The converter must stay off afterwards (`st=MEAS`, HS 0) and must not restart after 5 s.
3. Command test during a run: `MPPT`, `status;;mppt`, `sync forced!`, `+50`, `ota <url>` must all be
   refused. `dc 0` must stop the run and leave the converter off.
4. After the run: `pwm-freq` (or `pwm-dump`) shows the nominal frequency; `mppt` works again. Read the
   stack high-water line and the heap numbers from the log.
5. Compare the reported peak current of two or three rows with the current probe. This checks the
   current axis of the result.
6. Raise in steps, repeating the scope checks: source limit up, then `bias 9.75 20`, then the defaults
   (7 kHz, 30 A). Go to 5.57 kHz only after the bootstrap has been seen above 9 V at 7.8 kHz.
   The output capacitors carry about 9 A rms near the top (about four times normal): check their
   temperature and their ripple rating first.
7. Keep every log. `etc/measure_coil.py --bias-log <log>` refits it offline.

### Pass and stop criteria

- Stop the session if: the high-side gate bursts or sags, coil current goes negative outside the
  refresh pulse, a trip does not hold the converter off, or the frequency is not restored.
- Pass: three runs to the full range agree within the stated uncertainty; the −10 % and −20 % drop
  currents (or "not reached up to X A") are reported; the probe agrees with the reported peak current
  within the stated bound.

### Known weak points to look at

- The frequency change and its restore have never run on a device.
- The refusal of dead-time and frequency requests on the control side could not be exercised in the
  simulator (no PWM driver there).
- The saturation stop reacts late on a sharp knee (at about 25–35 % drop).
- Uncertainty is wide by design: loop resistance 0–90 mΩ, 1 % on each voltage, 6 % on the current gain.
  Session 1's reference measurements can narrow it for this board.
- Two test thresholds were relaxed in the last round (Iout-offset z-rms limit 4; the no-lock
  reproduction needs continuous mode in only 3 of 4 runs).

## Session 4: rectifier-offset calibration against the scope (2–3 hours)

Open question: `rect-cal` finds where output current starts to drop, which is the onset of loss. With
switch-node capacitance the reverse current must first swing the node to Vin, so at Vin above 2 × Vout
the onset lies after the true zero crossing. On the 76 µH installed board it learned −142 and −162 ct
(−1312 ns is stored); the visible bend in the raw points is about 55 ct before the ideal time. Whether
the stored value is close to the true crossing or 100 ct early is not known.

1. Run `rect-cal force` at 70–110 W and at higher power, three times each. Record learned and applied.
2. With the scope: at the calibration point (HS fixed, as printed by `rect-cal status`), set the LS time
   manually around the learned value and watch the switch node after LS turn-off. If the node falls to
   −Vf after turn-off, the LS turned off early (current still positive). If it rises toward Vin, the LS
   was late. The LS time where it changes over is the true zero crossing.
3. Compare: true crossing, visible bend, learned value. Do it at Vin above and below 2 × Vout.
4. Measure the lag between the true crossing and the loss onset. Expected from theory:
   `Irev_th = sqrt((Vi−Vo)² − Vo²)/Z0`, lag `Irev_th·L/Vo`: 40–85 ct on the 51 µH board, 56–165 ct on
   the 76 µH board depending on node capacitance (session 1 gives the capacitance).
5. Reverse current at the stored offset when running below 2 × Vout: expected 0.21–0.26 A peak, a few
   milliwatts. Confirm with the current probe.

Outcome: either the fit's early bias is confirmed as harmless, or a computed lag (from Vi, Vo, L and a
measured node capacitance as a new config value) replaces the fixed margin. A positive result is
currently stored as a bound of 0; with a measured capacitance that could become a real value.

## Session 5: the sense path of the 51 µH board (hardware, 1–2 hours)

Finding: on the installed 51 µH board the calibration cannot work. Both the Vout and the Iout reading
rise steadily with the LS on-time (+1.3 % Vout, +60 mA Iout) and the current never drops, even past
the ideal time. The readings follow the switching, not the current. The 76 µH board, reported to have
a filter capacitor at the INA226 inputs, does not show it. The firmware now refuses there with
"sense follows LS" or "Iout follows LS".

1. On a bench board without the INA226 input filter: reproduce by running `rect-cal force` and by
   stepping the LS time by hand at fixed HS; log Vout and Iout readings against the LS time, with a DMM
   on Vout as the reference.
2. Scope IN+, IN− and VBUS of the INA226 against its ground during body-diode conduction and during
   LS conduction.
3. Fit the input RC as on the other board, repeat step 1. Expect a flat baseline (the 76 µH board's
   baseline changes by 2.3 mA; the refusal bound is about 27 mA).
4. If confirmed: retrofit the installed board, then run its calibration (acceptance there: applied
   offset not above 0).

This also bears on that board's known "Vout reads 0.3–0.45 V low while converting" and on its coil
measurement spread (about ±2 % run to run).

## Session 6: network guard under a real outage (1 hour, a bare bench board is enough)

Background: after a network outage the MQTT service could get stuck in state Failed (a restart racing a
connect left a stale "connected" flag). Without MQTT the charger has no BMS data, falls back to its
conservative voltage cap, and production dropped to about 40 % for 85 minutes on 2026-11-16. The fix
clears the flag on every teardown, and the network guard now re-initialises MQTT after 2 minutes down
(then with doubling gaps up to 30 minutes) and reboots after 15 minutes without MQTT while Wi-Fi is up,
at most once per 6 hours. `bootinfo` prints `netguard: mqtt down N s, re-inits N, guard reboots N`.

This needs no power stage: any board with Wi-Fi and a broker it can reach.

1. Block the broker only (firewall rule or stop the broker) while Wi-Fi stays up. Expect re-init log
   lines at about 2, 6 and 14 minutes and one reboot at 15 minutes; after the reboot no second reboot
   for 6 hours while the broker stays down (`reboot held` in the `netguard:` line).
2. Unblock the broker: MQTT must connect within one retry, the hold must clear, `svc list` must show
   `mqtt Running ... connected`.
3. The original race: restart the access point 20 times at random intervals between 30 s and 5 minutes
   and check after each that MQTT is back within 3 minutes and never in state Failed.
4. During all of it: heap minimum (`heap`), no `alloc failed` beyond the network task's own buffers,
   loop gap unchanged if the board samples at 1470 per second.

Pass: no stuck state in 20 outages; exactly one guard reboot per 6 hours with the broker down.

## Session 7: optional checks

- **Loop gap with a console command during conversion.** `rt-stats` and `tasks` used to stall the loop
  for 1.9–2.6 ms; they now walk the task list in chunks and print "loop gap during command". Check
  under load on the bench at 1470 samples/s (`ina22x_conv_time_us=332`): the printed gap stays under
  1 ms.
- **Network guard and log ring under fault.** Pull the access point during conversion, starve the heap
  (many BLE connects): MQTT must come back, no "dropped" log lines unless forced, loop gap unchanged.
- **Tracker.** Not a bench item by itself, but a second converter in PV-simulator mode (`pv`) can feed
  the board with a repeatable curve and a 0.2 Hz, 0.5 % wobble. A simulator bisect on `main`
  (`doc/research/sim-plant-spice-revalidation.md`) reports that the settle window costs 3–4 points
  under such a wobble; that claim is unverified on hardware.
- **Unit tests in QEMU.** The on-target suite panics in QEMU before the test framework starts
  (coprocessor exception in a static constructor, seen on `9dc0351`). Not a bench item; listed so it
  is not forgotten.

## Order and effort

| Order | Session | Blocks |
|---|---|---|
| 1 | Baseline of the board | everything else |
| 2 | Current zero after a loaded stop | confidence in low-current readings, coil measurement |
| 3 | `measure-coil bias` first run | the DC-bias curve; use on the installed converters |
| 4 | Rect-cal against the scope | trusting stored offsets; a computed lag |
| 5 | Sense path of the 51 µH board | that board's calibration |
| 6 | Network guard under a real outage | confidence in unattended recovery |
| 7 | Optional checks | nothing |

Sessions 1 and 2 need no new code. Session 3 needs the `coil-lowfreq` branch. After session 3 passes,
the bias sweep may run on an installed converter only with: strong steady sun (about 400 W for the top
of the range), pack state of charge below about 80 %, the other converter limited so the combined
current stays under the BMS limit, and no other session updating or commanding the unit.

## What does not need the bench

These can be done on the installed converters or without hardware, whenever someone picks them up.

| Item | Needs | State |
|---|---|---|
| Offset calibration on the 76 µH converter with the final code in full sun | steady sun, two or three `rect-cal force` runs at different power | not done; the only run of the fixed code was at 73 W (learned −142 ct). Stored value is −1312 ns (−210 ct), written by hand from a 113 W run |
| Tracker after-picture | one day of log on the current build | not done; before-picture on the previous build: 18–23 slow-mode entries per hour, no enter-then-reset churn. Count `Slow mode, duty` lines and those followed by `Reset maxPower` within 1 s |
| Fourth review of `coil-lowfreq` | nothing | open; do it before session 3 |
| Unit-test suite panics in QEMU before the test framework starts | nothing | an investigation was started on 2026-11-17 in worktree `/tmp/qemufix` (branch `qemu-unity-fix`); nothing committed |
| Idle power profile | a fix of the profile's start gate | on the 51 µH converter the guard now keeps the CPU at 160 MHz and the loop stays under 1 ms (worst 881 µs through the 2026-11-18 dawn). On the 76 µH converter the start gate did not release at the real idle clock and the unit did not start at dawn; it has `board.conf idle_cpu_mhz=0` until that is fixed |
| Headless scope capture | a commit in the `etc/adcscope` submodule | written and used, uncommitted |

## State to know when resuming

- The 1 ms loop goal is closed: the 51 µH converter held under 1 ms for a full day on one build
  (worst 883 µs, 2026-11-15 17:53 to 2026-11-16 18:00 UTC).
- Both installed converters run `main` at `e4cb061` (version string `fry-brk1-608-ge4cb0616`) without the
  coil measurement command. To use `measure-coil` on them, build with the `sdkconfig.measure_coil`
  fragment. The 76 µH converter has `board.conf idle_cpu_mhz=0` because the idle profile's start gate
  kept it from starting at dawn on 2026-11-18.
- The wider picture (results, open problems, where the code is, working rules) is in
  [handover-2026-11-18-loop-rectcal-coil-tracker.md](handover-2026-11-18-loop-rectcal-coil-tracker.md).
- The 51 µH converter has no rectifier offset (0 ns, uncalibrated) and cannot learn one until its sense
  path is fixed (session 5). The firmware refuses there with "Iout follows LS" or "sense follows LS".
- Coil values: 76 µH converter measures 76.0–77.0 µH (`coil.conf` 80e-6, used as 95 %); 51 µH converter
  measures about 51 µH (`coil.conf` 56e-6, used as 53.2 µH). Neither `coil.conf` was changed.
- Before any state-changing command on an installed converter: take its lock
  (`~/.claude/skills/lock/lock.sh acquire <name> "<what>" --session <id>`), confirm `hostname`, and
  reach it by address or telnet, not by BLE name (a name search connected to a bench board twice).
- Batch simulator runs go to the remote host with `~/bin/offload`, tagged, never on the Mac.

