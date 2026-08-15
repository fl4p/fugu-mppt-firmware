*this document is an LLM generated placeholder*

# Automated Bench Tests

Test matrix for exercising the converter against a **programmable power supply** on the input and a
**programmable electronic load / sink** (or battery emulator) on the output. The goal is repeatable,
scriptable coverage of the protection, control, MPPT, and charger paths under controlled load
conditions, without needing sun or a real battery.

For the older hand-run cases see [Test Cases.MD](Test%20Cases.MD); this doc supersedes them for
automated runs.

## Bench setup

```
   [ PSU / PV-sim ] --Vin--> [ FUGU under test ] --Vout--> [ e-load / battery-sim ]
                                     |
                              console (serial / telnet / BLE)
```

- **Input source**
  - For protection/transient tests a plain CV/CC bench PSU is enough (set V, set I-limit).
  - For real MPPT tracking you need a **PV/solar-array simulator** (Keysight, Chroma) or a PSU with
    a known **series resistance** so an MPP exists. A stiff CV source has no maximum power point —
    the tracker will just walk to the current limit.
  - In-house alternative: a second Fugu boost in **`mode=pv`** emulates the panel curve at its
    output (`pv` console command, `config/lab/fboost_pv`) — see the PV-sim section in
    [Power Loop.md](Power%20Loop.md) for the rig caveats.
- **Output sink** — an electronic load is the battery emulator:
  - **CV mode** = fixed pack voltage (charger sees a battery clamped at that voltage).
  - **CC mode** = fixed sink current (defines the operating point / charge current).
  - A **bidirectional source-sink** is required for reverse-current and battery-interrupt tests.
- **Control / observation** — drive everything from `etc/fugu_console.py`
  (`-p <serial>` / `--ip <host>` / `--ble`), one command per step with `-c "<cmd>"` (repeatable),
  a batch of commands piped to `--stdin`, or the interactive REPL (the default with no mode flag).
  Telemetry goes to InfluxDB; the per-sample `scope` TCP stream is useful for transient capture.
- **Config** — use `config/lab/fbuck_lab_bench` only with the 29 V battery/emulator setup. An
  open-output SW-node sweep must instead be provisioned with
  `config/lab/fbuck_lab_bench_open_output`; its explicit name and separate complete profile guard
  against carrying the 60 V open-output threshold back to a connected battery. Note
  `reverse_current_paranoia` differs between configs and changes several thresholds below.

Thresholds quoted below are from `config/fmetal/conf/limits.conf` /`charger.conf`
(`vin_max=85`, `vout_max=60`, `iin_max=30`, `iout_max=32`, `temp_max=90`, `temp_derate=70`,
`vout_max=29` pack, `cv_float=3.37`, `cv_eoc=3.57`). Read the actual conf on the unit under test —
they are not compiled in.

## Console commands used by the tests

| command | effect |
|---|---|
| `dc N [ls]` | manual PWM, fade HS duty to `N` (optional manual LS hold) |
| `mppt` | re-enable auto MPPT (leave manual mode) |
| `sweep` | force a global sweep / re-calibration |
| `sync on/off/forced` | LS rectifier mode |
| `bf 0/1` | backflow switch |
| `fan N` | fan duty |
| `set-config <file> <k> <v>` | edit a conf key live (e.g. lower a limit to provoke a trip safely) |
| `rt-stats`, `sensor` | timing / sensor readout for pass checks |

---

## 1. Protection / shutdown

Each test should assert **a single trip event**, not a repeating storm. Capture the console for N
seconds and count `shutdown` / `Converter enabled` lines.

### 1.1 Vout > Vin shutdown in manual mode
- **Setup**: Vin low or absent (e.g. PSU 8 V), output sink CV at 26 V (Vout > Vin).
- **Action**: `dc 10`.
- **Expected**: converter enables once, `protect()` trips on `Vout > 1.25·Vin`, logs one
  `MPPT: Vout .. > Vin .., shutdown duty=..`, then stays off. `setTargetDutyCycle(0)` on the trip
  prevents re-fade.
- **Pass**: exactly one enable/trip/disable cycle; no repeating warnings; unit stays in manual mode,
  converter disabled.
- **Fail mode this guards**: pre-fix it re-faded every few ms (warning flood)

### 1.2 Output over-voltage (OV)
- **Setup**: charging normally, then drive output sink **voltage** above
  `min(Vbat_max·1.03, vout_max)`.
- **Expected**: `shutdownDcdc()`, `Vout .. > .. + 5pct!` warning, LCD "OV shutdown". If condition
  persists >20 s and `autoDetectVout_max`, Vbat_max re-detects via calibration.
- **Pass**: shutdown within one control tick of the threshold crossing; recovers when voltage drops.

### 1.3 Battery interrupt (load disconnect)
- **Setup**: charging at moderate current, then **open the output** (sink off / relay).
- **Expected**: Vout spikes (target ≤ +10 %), OV protection catches it, slow recovery. Watch for
  overshoot beyond `vout_max` and any LS reverse-current event.
- **Pass**: no sustained OV, no reverse-current trip latch-up; energy meter committed.

### 1.4 Input over-voltage
- **Setup**: ramp PSU above `vin_max` (85 V).
- **Expected**: `Vin .. > ..!` warning, shutdown. (Mind the board's absolute max — ramp gently.)

### 1.5 Output over-current
- **Setup**: sink in CC, step current above `iout_max·1.5` (instantaneous), or above
  `iout_max·1.25` sustained (med3) / `1.15` averaged (ewm).
- **Expected**: `Iout .. >lim .., shutdown`. Verify each of the three thresholds (last/med3/ewm)
  trips on its own time-scale.

### 1.6 Input over-current
- **Setup**: low Vin, high duty so Iin climbs above `iin_max·1.3`.
- **Expected**: `Iin .. >1.3x lim .., shutdown`.

### 1.7 Supply under-voltage (board brownout)
- **Setup**: lower **both** Vin and Vout below ~9 V (board derives its own supply from
  max(Vin,Vout) − 0.3 V diode). Also check the `vin_min` (10.5 V) input floor in auto mode — at
  `Vin = vin_min` the unit should not start / should shut down.
- **Expected**: `Supply under-voltage!`, shutdown, meter commit. `startCondition` uses a higher 9.5 V
  bar (hysteresis) so it won't immediately restart.
- **Note**: manual mode passes `ignoreUV=true`, so this UV path is bypassed under `dc` — test in auto
  mode.

### 1.8 Reverse current
- **Setup**: bidirectional source-sink pushes current **into** the output (Vout source > converter).
- **Expected**: progressive response — disable backflow switch, set LS to min duty, then on negative
  averaged current `Reverse avg current .., shutdown!`. Behavior depends on
  `reverse_current_paranoia`.

### 1.9 Over-temperature derate & cutout
- **Setup**: heat the NTC (heat gun / climate chamber) or spoof via a thermistor box.
- **Expected**: above `temp_derate` (70 °C) `P_max` scales down linearly to zero at `temp_max`
  (90 °C); above `temp_max` (NTC or MCU) → shutdown. Also `startCondition` blocks restart while hot.
- **Pass**: power follows the derate ramp; clean cutout and recovery.

---

## 2. MPPT tracking  *(requires PV simulator or PSU + series R)*

### 2.1 Global sweep finds MPP
- **Setup**: PV-sim at a fixed I-V curve.
- **Action**: `sweep`.
- **Expected**: duty ramps 0→max, peak power captured, settles near the simulated MPP voltage.
- **Pass**: tracked power ≥ 98 % of the curve's true Pmax.

### 2.2 P&O convergence (fast then slow)
- **Setup**: steady curve after a sweep.
- **Expected**: fast P&O closes on MPP, then slow P&O; duty dither stays bounded.

### 2.3 Irradiance step
- **Setup**: step the PV-sim curve (e.g. 100 %→40 % Isc) and back.
- **Expected**: tracker re-converges to the new MPP without an OV/OC trip; measure settling time.

### 2.4 Periodic re-sweep
- **Expected**: a fresh sweep ~every 30 min (or on major power change). Confirm it is **suppressed**
  while the battery is full / in CV (recharge-after-full guard).

### 2.5 Partial shading / multi-peak
- **Setup**: PV-sim with a two-step (shaded) curve.
- **Expected**: global sweep finds the global peak, not a local one.

### 2.6 Low-power / cloudy
- **Setup**: very low Isc curve (a few W).
- **Expected**: stable tracking, no oscillation/chatter at minimum duty; correct CCM↔DCM handling.

---

## 3. Load-step & transient response

### 3.1 Output load step up/down
- **Setup**: sink CC, step current light↔heavy.
- **Expected**: Vout stays regulated; no OV on dump, no UV-driven trip on step-up. Capture with
  `scope`.

### 3.2 Input voltage step
- **Setup**: step PSU voltage within range.
- **Expected**: control re-settles; no trip.

### 3.3 CCM ↔ DCM transition
- **Setup**: sweep load current down through the DCM boundary (set by Vin/Vout and coil L).
- **Expected**: LS rectifier diode-emulation behaves (no reverse current); compare `CCM(H|L|Lm)`
  counters against expectation. See [Diode Emulation.md](Diode%20Emulation.md).

### 3.4 `sync` / `bf` mode coverage
- Exercise `sync on/off/forced` and `bf 0/1` at a fixed `dc` operating point; verify reverse-current
  safety in each combination (bench only).

---

## 4. Charger / CV-CC  *(battery emulator on output)*

### 4.0 CV with no battery (PV only)
- **Setup**: input source only, **no battery / open output** (or sink in high-impedance).
- **Expected**: output regulates to `Vout_max` (no pack to absorb current); converter holds CV at the
  output ceiling.

### 4.1 CC phase
- **Setup**: sink CV below the CV knee; ample input power.
- **Expected**: charger holds `Iout_max` / `charger.Iout_max()`; CC limit is the active controller.

### 4.2 CV phase & taper
- **Setup**: sink CV at/near `Vbat_max`.
- **Expected**: enters CV, current tapers along the LFP termination line between `cv_float` and
  `cv_eoc`. See [LFP Charging.md](LFP%20Charging.md) / [Termination.md](Termination.md).

### 4.3 Termination
- **Setup**: continue CV until tail current < `tail_c_rate·bat_c`.
- **Expected**: charge terminates; no re-dump.

### 4.4 Recharge-after-full
- **Setup**: emulate a full pack (Vout clamped high), leave running ≥ 30 min.
- **Expected**: periodic sweep is **gated off** while full/CV — no charge pulse into the full pack.

### 4.5 BMS cell-voltage coupling vs fallback
- **Setup**: publish a high cell voltage over MQTT, then stop publishing (let it go stale > 180 s).
- **Expected**: charger uses BMS max-cell while fresh; falls back to `Vbat_fallback` / pack-V÷N when
  stale.

---

## 5. Steady-state efficiency & thermal

### 5.1 Efficiency map
- Sweep (Vin, Vout, Iout) grid points; log Pin/Pout for an efficiency surface.

### 5.2 Sustained full power / thermal soak
- Hold near `P_max` and watch the temp-derate engage at `temp_derate`; confirm fan curve
  (`fan N` baseline vs auto).

---

## 6. Startup conditions

### 6.1 startCondition gating
- Verify no start when: NTC/MCU > `Temp_derate`; supply < 9.5 V; or the Vin/Vout polarity is wrong
  for the topology (buck needs Vin > Vout+1; boost needs Vin < Vout+1).

### 6.2 Cold/low-light start
- Ramp input from below threshold; confirm clean first sweep once `startCondition` is met.

---

## 7. Boost topology  *(if `converter.conf: topo=boost`)*

Mirror the buck cases with Vin < Vout: OV on output, reverse current, MPP tracking, and the
`startCondition` polarity check (Vin < Vout+1).

---

## Automated gate-driver test

`etc/mcpwm_gate_verify.py` — closed-loop PWM verifier. Drives the device over serial or
telnet while capturing HS/LS gates on a PicoScope 2000. Asserts frequency, HS-duty
linearity, LS pulse position/width across an HS × LS grid, dead-time + no shoot-through,
and the hardware fault brake. Refuses to run against `fry`/`flat` unless `--force-host` —
default allow-list is `fugu-esp32s3-*` mock boards.

    etc/mcpwm_gate_verify.py --serial /dev/cu.usbmodem2101 [--skip-fault]
    etc/mcpwm_gate_verify.py --ip 192.168.1.173 --port 232 [--skip-fault]

Wiring: Ch A on `board.conf::pwm_hi`, Ch B on `board.conf::pwm_li`, both DC-coupled at 5 V
range. For Phase 4, also wire a free GPIO (default 14, set with `--fault-driver-pin`) to
`board.conf::pwm_fault_pin`. See `docs/superpowers/specs/2026-05-23-pwm-gate-verifier-design.md`
for the full spec and PASS-gate math.

---

## Automation notes

- Wrap each case as: set PSU + load → issue console command(s) → wait → read console/telemetry →
  assert. `etc/fugu_console.py` already has a PASS/FAIL/SKIP exerciser model to extend.
- For trip tests, prefer lowering a limit with `set-config limits.conf <key> <v>` to provoke a trip
  at safe currents/voltages instead of driving real over-stress.
- Always assert **single-trip** behavior (count log lines) — repeating storms are themselves a bug
  class (see 1.1).
- Restore limits and `mppt` (auto mode) at the end of each case.
