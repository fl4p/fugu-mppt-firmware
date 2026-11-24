*this document is an LLM generated placeholder*

# Sweep stopped at a 2 A output-current limit (flat, 2026-11-23)

Firmware on flat: `fry-brk1-869-g99736a2e` (bootinfo 13:50:38). Code references are to `99736a2e` unless marked HEAD.
Investigation was read-only: log on havan (`pv/fugu_console.log*`, 2026-11-11 17:18 to 2026-11-23 18:48) and git. No
device commands were sent.

## Cause

The 1.99 A target was the **CV hold's output-current limit** (`_cvIlim`). The sweep's own zero-current start dragged
it down through the hold's anti-wind-up clamp.

1. At 13:45:31 flat entered the CV hold of a balance run:
   `charger: cv hold: vcHigh 3.516 > line 3.500 @ iBat 39.86, ilim 21.8 A`. Status lines then show
   `Charger: CV HOLD balance (…, ilim 8.0A) … Iout_max=8.0A` (13:47:56) and `ilim 8.5A` (13:48:08). Up to the sweep
   flat ran at `st=CC` at 9.2 to 10 A, about 258 W. That power was the hold's current cap, not the MPP.
2. Since 5e346dec (2026-11-23 12:08, "CV hold regulates pack current via an output-current limit", in 99736a2e),
   `charger.Iout_max()` includes `_cvIlim` (charger.h:1303-1315). `MpptController::update()` uses it as the
   IoutCTRL target, `Iout_max = min(limits.Iout_max, charger.Iout_max())` (mppt.cpp:83-86, controller at :148).
3. `_cvHoldStep()` runs on every BMS ibat frame, about every 4 s (charger.h:892-910). It sets
   `lim = min(_cvIlim + min(G·(target − ibat), 2 A), iout + HEADROOM_A)` with `HEADROOM_A = 2 A` (:893, :905).
   This clamp stops the limit winding up while the PV is short. Its `iout` is the Iout EWM passed from `loopLF`
   (main.cpp:1119, :1130).
4. `startSweep()` disables the converter and starts the zero-current calibration (mppt.h:1034 ff.). The log shows
   `PWM disabled` 13:52:04.89, `Calibration done` 13:52:05.81, `CCM -> DCM (… I=-0.01 …)`. An ibat frame landed
   while iout was about −0.01 A, so `_cvIlim = −0.01 + 2 = 1.99 A`. That matches `tgt=1.99` exactly.
5. Under that cap, IoutCTRL went negative 3.5 s into the ramp. The sweep ended
   `mode=CC (lim=3 tgt=1.99 act=1.99) PWM=1134 MPP=(54.3W,1134,67.4V)` with one V point (`sweep-curve` n=1).
   `sweepVerdict()` logged `sweep not verified (limiter)` (mppt.cpp:1012).
6. The recovery is the hold climbing back, one ibat frame at a time, capped at iout + 2 A:
   `Iout_max=5.17` at 13:52:25, then 6.0, 6.8, 7.6, 8.3, 8.9, 9.55 A, and back at 10.1 A `st=CC` by 13:52:56.
   That is about 50 s at reduced power, roughly 1.5 Wh less into an almost full pack.

The 13:52:04 `START blocked: calibrating` line is only a log artefact. `main.cpp:1191-1205` names the start-block
reason while the converter is disabled, and the sweep's calibration is that reason. It plays no part in the limit.

### Ruled out

| Candidate | Evidence |
|---|---|
| BMS `ibat_lim` topic | status: `Ibat_lim=39.0A` throughout; no `Ibat_lim=` change line in the log |
| Battery-temperature `ioutLim` | would show `pack … °C` lines; none; hold status shows `Iout_max` = `ilim` |
| Wire guard `ioutCap()` | warn-only; `Iout_max` equals the hold's `ilim` in every status line |
| Sweep-specific cap | none in `update()`: the IoutCTRL target is the same `Iout_max` in and out of a sweep |
| Holiday / partial hold | `holiday off (auto)`; partial hold off (status `CV HOLD balance`) |
| Stale-cell ramp | BMS frames `ok, 0-5 s ago` |
| cc30ca19 (approach band) | not in 99736a2e (it came later, 16:55); flat ran the plain hold from 5e346dec |

## Frequency

All `Stop sweep` lines from 2026-11-11 17:18 to 2026-11-23 18:48, by trigger and stop mode, with the hold state at
the time. "hold-ilim" means the output-current hold of 5e346dec. "restart/other" is the start path after a backoff;
most of those are no-MPP backoffs at dusk.

| Device | Trigger | Stops (mode, lim) | In hold |
|---|---|---|---|
| flat | periodic (30 min) | 169 CV/4, 13 CV/0, 10 CV/1, 1 SWEEP, 4 no-MPP | none |
| flat | power-doubled | 24 CV/4, 18 CV/0, 1 no-MPP | none |
| flat | boot | 17 CV/4, 2 CV/1, 17 no-MPP | 1 (old pin hold, 11-20, lim=1) |
| flat | console `sweep` | 9 CV/4, 7 CV/0, 1 SWEEP, 1 no-MPP, **1 CC/3** | **1 hold-ilim (13:52:08)** |
| flat | console `mppt` | 3 CV/4, **1 CC/3** | **1 hold-ilim (13:29:41)** |
| flat | restart/other, backoff cleared | 20 CV/4, 5 CV/0, 1 CV/1, 7 SWEEP, 286 no-MPP | none |
| fry | periodic | 138 CV/4, 14 CV/0, 5 CV/1, 1 SWEEP, 7 no-MPP | none |
| fry | power-doubled | 19 CV/4, 13 CV/0, 1 CV/1, 2 no-MPP | none |
| fry | boot | 37 CV/4, 1 CV/1, 17 no-MPP | none |
| fry | console `sweep` / `mppt` | 4 CV/4, 8 CV/0, 2 CV/1, 2 no-MPP | none |
| fry | restart/other, backoff cleared | 15 CV/4, 5 CV/0, 1 CV/1, 66 SWEEP, 327 no-MPP | none |

Only 2 of about 1300 sweeps stopped on IoutCTRL (`lim=3`). Both were on flat on 11-23, both inside the new current-limit
hold, and both were started by hand:

| Time | Trigger | Before | Stop | After |
|---|---|---|---|---|
| 13:29:37 | `mppt` after manual mode (MANU, 0 A) | 0.3 W, idle; hold limit already at iout + 2 A | 3.52 s, CC lim=3 tgt=2.01, MPP=(54.8 W, 1164, 66.8 V) | 6.1 A at :56, 11.3 A at 13:30:16 (hold `ilim 12.1A`) |
| 13:52:04 | console `sweep` (config editor) | 254-258 W, CC at 9.2 A (hold) | 3.54 s, CC lim=3 tgt=1.99, MPP=(54.3 W, 1134, 67.4 V) | 145 W at :26, back to 10.1 A / 278 W at 13:52:56 |

In the 13:29 case the limit had collapsed before the sweep: flat had sat at 0 A in manual mode, so the clamp had
already pulled the limit to 0.01 + 2 A. In the 13:52 case the sweep itself caused the collapse.

### Automatic sweeps

At 99736a2e, automatic sweeps are not affected:

- Periodic, retry and power-doubled sweeps are skipped while `batteryFull = charger.chargeHold() || mode == CV`
  (mppt.cpp:93, :109), and `chargeHold()` includes `termCond.cvHold()` (charger.h:1263). The table has no automatic
  sweep inside a hold.
- The start path in `main.cpp` (after a trip or backoff, the `mppt` re-enable, boot) gates only on
  `full = termCond && bmsFresh` (main.cpp:1419), **not** on the CV hold. A protection trip or no-MPP backoff during a
  hold therefore restarts with a sweep under an already collapsed limit (the 13:29 pattern). None occurred in the log
  window: the output-current hold has only been live since 12:39 (flat) and 14:08 (fry) today.
- fry runs the same code since its 14:08 OTA; it had no sweep inside a current-limit hold.

### HEAD exposure is wider (cc30ca19, untested on HW)

- The approach band (charger.h HEAD:988 ff.) also sets `_cvIlim` and runs `_cvHoldStep()` with the same
  `HEADROOM_A` (HEAD:917, :1031) before the hold proper: within 30 mV of `cv_eoc`. Periodic sweeps are still gated
  there only if `chargeHold()` is true. The approach band is not part of `chargeHold()`, so **at HEAD a periodic
  sweep in the approach band will collapse the limit the same way**.
- The boot idle (HEAD:999-1005) sets `_cvIlim = 0.25 A` near the top until termination is decided. A boot sweep after
  the 12 s BMS wait times out stops at 0.25 A. That idle is intended; only the sweep result is meaningless.

## Bug or intent

It is a bug, from an interaction nobody designed for. The 2 A headroom is deliberate: per the comment at
charger.h:886-891 and the termination guide, it stops the limit winding up while the PV is short, so the returning sun
does not push a cell over `cv_eoc`. A sweep is not a short PV, though. The converter is switched off on purpose and
ramped from zero, and the hold reads that as "PV collapsed" and cuts its own limit. Nothing in the sweep path knows
about the hold. The result:

- The sweep records a stop point as the MPP, overwrites `sweep-curve` with a single point (what the config editor
  shows), and is marked not verified.
- The pre-sweep current comes back only at the hold's step rate: about 50 s, about 1.5 Wh.

No safety problem: the cap is lower than intended, never higher.

A sweep cannot find the MPP inside a charge hold in any case. The converter is current-capped below the MPP (258 W
at 10 A against an MPP above 500 W). Even with the limit intact, the sweep would stop at about 10 A on `lim=3`. That
is why the periodic sweep is gated on `chargeHold()`.

## Fix

Recommended, minimal: **freeze the hold's current loop while a sweep runs.** Pass `mppt.isSweeping()` into
`charger.update()`, then into `_updatePackVoltagePinning()`, and call `_cvHoldStep(!psu && !sweep, …)`. With
`step == false` the function still consumes the ibat frames and only skips the update, the same way PSU mode freezes
it. So nothing winds up during the sweep, and the next post-sweep frame resumes from the pre-sweep limit. A hold or
approach entered during a sweep starts at the tail current (`tail_c_rate·Cbat`) instead of the near-zero iout.

The sweep then ramps to the pre-sweep current (about 10 A here) and stops there on `lim=3`. That is the correct CC
operating point, so power is back at once instead of after 50 s.

On main as `f091ac33` (2026-11-23, rebased onto the stale-BMS decay `4f1e4879`; host test passes). Not built for
target after the rebase, not HW-tested.

- `src/charger.h`: a `sweep` parameter on `update()` and `_updatePackVoltagePinning()`; the step is frozen and the
  entry limits use the tail current while `sweep` is set.
- `src/main.cpp`: `mppt.charger.update(vout, iout, voutAuthority, g_app.psuMode(), mppt.isSweeping())`.
- `test/host-stub/charger-term-test.cpp`: `test_hold_sweep_keeps_limit(true/false)`. In the plant sim the hold sits
  at about 24.8 A, then a 6 s zero-current sweep start runs. With the flag, the limit is kept and the current is back
  within 4 s of the sweep's end. Without the flag, the limit collapses to 2.00 A, the field failure. Known-bad check:
  with the freeze removed, the flag-true case fails with `limit kept 24.82 -> 2.00 A`. All charger-term tests pass with
  the fix.
- `website/docs/guide/charging/termination.md`: one bullet on the sweep freeze.

Risks:

- The cell loop does not step for the length of a sweep, 4 to 17 s or 1 to 4 ibat frames. A pending downward step is
  delayed by that time. During the sweep the output current stays at or below the frozen limit, which is at or below
  the pre-sweep current, so the pack gets no more current than before the sweep.
- `_sweeping` is a plain `bool` written on the RT core and read in `loopLF` on core 0. It is already read there the
  same way (main.cpp:391, :740); a one-tick-stale read only shifts the freeze by one LF tick.
- This does not help when the limit had already collapsed before the sweep (13:29: manual mode at 0 A, or a trip
  backoff in the hold). The sweep then still stops at about 2 A. That case is harmless (the hold ramps back), and
  only the stop point and the sweep curve are misleading.

Optional follow-ups, not implemented:

1. Console `sweep` during `chargeHold()`: reply "charge hold: output capped at X A, the sweep stops there", or refuse
   unless forced. This tells the config-editor user why the curve is short.
2. Do not replace the stored sweep curve (`sweep-curve`) and MPP when a sweep is stopped by IoutCTRL or IinCTRL
   inside a charge hold. Keep the last full curve so the editor plot stays useful.
3. At HEAD, include the approach band in the periodic-sweep gate (`batteryFull`), as `chargeHold()` does for the hold.
   Otherwise every 30-min re-sweep in the band starts from 0 A into the approach limit. With the freeze it would no
   longer collapse, but it would still be a pointless sweep at the top of charge.
