*this document is an LLM generated placeholder*

# PV emulator partial shade: hardware test on the power-loop rig

Firmware: `73164e32` (untested on HW). It adds `pv shade` / `converter.conf::pv_shade`: the emulated curve becomes a series
string of 2..6 equal substrings with 0.5 V bypass diodes, a linear knee over the last 5 % of each substring's Isc, and an
Iout limit that follows the string's Isc (`src/math/pv_model.h` `PvShade`, `src/mppt.cpp` `pvAdvanceSetpoint`).
User docs: `website/docs/guide/operating-modes.md#partial-shade`.

## Goal

1. The boost (fboost) in `mode=pv` with `pv shade` produces the multi-peak curve the model predicts.
2. Its output stays stable while the load crosses a substring knee (no limit cycle, no OV trips).
3. The buck under test (fbuck) finds the global peak, both when it is the low-voltage and the high-voltage peak.
4. Record how the tracker behaves when the global peak moves under it.

## Status and preconditions

- The unshaded PV emulator has never been validated on hardware (`plans/pv-sim-mode.md` step 7, still pending).
  Its steps 2–5 are steps 1–3 below; do them first, since shading builds on the same loop.
- fbuck and fboost were not at this bench on 2026-11-24. The test waits until the rig is set up again.
- Both boards are bench units, so flashing is allowed, but take the locks first:
  `~/bin/lock.sh acquire fboost "pv shade test"`, the same for `fbuck`, `fugu-rig`, `korad` and `scope` if used.
- Before anything that switches, `hostname` must answer `fboost` (or `fbuck`) on that connection.
- Rig wiring as in `website/docs/lab/power-loop.md`: PSU into the boost input, boost output to the buck input, buck
  output back to the boost input. PSU in CV at the usual loop voltage (about 26–29 V) with a **low current limit**
  (start at 3 A). Below the Vin floor the boost's body diode passes current that no firmware limit controls, and the
  PSU limit is the only backstop there.

## Firmware and config

1. Build `73164e32` (or later) in a clean worktree: `. ./idf-export.sh && idf.py -B build/s3 build`.
2. `app-flash` fboost (keeps its littlefs config); note `bootinfo` (version, slot). fbuck needs no change unless its
   build is older than the scan/sweep code you want to evaluate; note its `bootinfo` either way.
3. fboost runs its usual bench profile; enter PV mode at runtime with `pv` (no re-provisioning).
4. fbuck: `config/lab/buck_bench` pins the operating point with `limits.conf::vin_min=72` for the stiff-source
   setup. Note the current value (`get-config limits.conf vin_min`), then lower it below the lowest peak:
   `set-config limits.conf vin_min 30` and `restart`. Restore it in step 8.

## Expected curves

All cases use `pv 2 60 0.8` (Isc 2 A, Voc 60 V, Vmp 48 V). Model values from `PvShade::voltage`
(the same function the firmware uses):

| Case | Shade | Peaks (model) | Global |
|---|---|---|---|
| U | off | 48.0 V / 87 W | 48.0 V |
| A | `1,1,1,0.4` | 54.3 V / 41 W, 35.5 V / 64 W | low-voltage peak |
| B | `1,1,1,0.7` | 51.9 V / 69 W, 35.5 V / 64 W | high-voltage peak, 7 % above the other |

Voltage and power at fixed output currents:

| I (A) | U (off) | A (`1,1,1,0.4`) | B (`1,1,1,0.7`) |
|---|---|---|---|
| 0.20 | 59.5 V / 12 W | 59.2 V / 12 W | 59.4 V / 12 W |
| 0.50 | 58.5 V / 29 W | 57.6 V / 29 W | 58.3 V / 29 W |
| 0.70 | 57.8 V / 40 W | 55.7 V / 39 W | 57.5 V / 40 W |
| 0.76 | 57.5 V / 44 W | 54.3 V / 41 W | 57.2 V / 43 W |
| 0.80 | 57.4 V / 46 W | 42.5 V / 34 W | 56.9 V / 46 W |
| 1.00 | 56.4 V / 56 W | 41.8 V / 42 W | 55.7 V / 56 W |
| 1.30 | 54.6 V / 71 W | 40.5 V / 53 W | 52.6 V / 68 W |
| 1.50 | 52.9 V / 79 W | 39.2 V / 59 W | 39.2 V / 59 W |
| 1.80 | 48.2 V / 87 W | 35.6 V / 64 W | 35.6 V / 64 W |
| 1.90 | 44.6 V / 85 W | 33.0 V / 63 W | 33.0 V / 63 W |

The knee of the shaded substring (A: 0.76–0.80 A, B: 1.33–1.40 A) is the steepest part of the shaded curves, about
350 Ω at this Isc (the slope in ohms scales with Voc/Isc). Iout noise σ after the emulator's EWMA becomes about
350 Ω × σ of setpoint jitter there, capped by `pv_slew` (200 V/s). fboost has no Iout sensor: its Iout is computed
from Iin, Vin and Vout (`src/adc/sensor_setup.cpp` virtual sensor), so a falling Vout raises the computed Iout. That
coupling is the main stability risk in `plans/pv-sim-mode.md`, and the knee makes it stronger.

## Steps

State-changing steps are marked [confirm]: announce them, then send. Abort on any ADC error line, an OV trip
escalation, a `LATCHED` PSU state, or a PSU in current limit for more than a few seconds: `dc 0` on fbuck, then
`pv off` on fboost.

1. **Preflight (read-only).** fboost and fbuck: `hostname`, `bootinfo`, `status`, `sensor`, `pv`. fbuck idle.
2. **Unshaded curve, open circuit.** [confirm] fboost: `ovset 70`, then `pv 2 60 0.8`. No load: Vout settles at 60 V.
   `pv` lists one peak, 48.0 V / 87 W.
3. **Unshaded curve, static and tracking.** [confirm] fbuck manual duty in small steps (`dc N`), from low to the
   current of table row 1.90 A. At each step record fboost Vout, Iout, `Vset` and fbuck Vin, Iin. Compare with
   column U (within a few %, limited by the virtual Iout's efficiency guess). Then fbuck `mppt`, `sweep`:
   expect 48 V ± 2 V and ≥ 0.97 × 87 W. This closes `plans/pv-sim-mode.md` steps 4–5.
4. **Iout noise.** With fbuck held at about 1 A in manual duty, record fboost Iout and `Vset` for 60 s
   (status lines or telemetry). Note σ(Iout) and Vset peak-to-peak. Predicted knee jitter = 350 Ω × σ(Iout).
   If that exceeds about 2 V, raise `pv_iout_span` (32, then 64) or lower `pv_slew` (100) before step 5, and note it.
5. **Case A, static.** [confirm] fboost: `pv shade 1,1,1,0.4`. `pv` must list 54.3 V / 41 W and 35.5 V / 64 W.
   fbuck manual duty steps as in step 3, with finer steps between 0.70 and 0.85 A. Compare with column A.
   **Knee crossing:** move fbuck slowly (one duty count per second) through the 0.76–0.80 A band, both directions.
   Pass: fboost Vout follows the table without sustained oscillation (Vout peak-to-peak below 2 V outside the
   band), no OV event, PSU trip count unchanged (`pv`).
6. **Case A, tracking.** [confirm] fbuck `mppt`, then `sweep`, three times, 2 min apart. Pass: each settles at
   35.5 V ± 1.5 V with ≥ 0.97 × 64 W. Record the sweep result line and whether the tracker ever settles at the
   54 V local peak.
7. **Case B, tracking.** [confirm] fboost: `pv shade 1,1,1,0.7` (`pv` lists 51.9 V / 69 W and 35.5 V / 64 W).
   fbuck `sweep` three times. Pass: 51.9 V ± 1.5 V and ≥ 0.97 × 69 W. The peaks are 7 % apart, so this checks
   the sweep's peak discrimination.
8. **Moving global peak.** With fbuck tracking at case A's global peak (35.5 V), switch fboost to
   `pv shade 1,1,1,0.7` without telling fbuck. The global peak moves to 51.9 V. Record how long fbuck takes to
   reach it (scan, re-sweep or the 30 min periodic sweep) and the energy lost meanwhile. Then switch back to
   case A and record the same. No pass/fail: this is the field case "shade moves", for comparison with the
   mppt-sim `wx-shade*-move` scenarios.
9. **Restore.** fbuck `dc 0`; fboost `pv shade off`, `pv off`, `ovset 0`; fbuck
   `set-config limits.conf vin_min <noted value>` and `restart`; PSU output off. Release the locks.

## Record

Per step: firmware versions, commands sent with timestamps, fboost Vout/Iout/Vset, fbuck Vin/Iin/P, `pv` and
`status` outputs, any trip or error line. Put the capture record in `doc/research/` and bench-specific notes in
`doc/lab/`. If the results hold, add a short anonymised result to the Partial shade section of the published docs
and update the PV emulator memory note.

## Open questions

- Is a 5 % knee enough on the rig, or does the virtual-Iout coupling need a wider knee or an emulator-side
  Iout filter that is only active near a knee?
- Does fbuck's scan learn the substring count from the emulator (`tracker.conf::pv_substrings`), and does that
  change step 8? Note the value before and after.
