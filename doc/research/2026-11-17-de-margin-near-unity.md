*this document is an LLM generated placeholder*

# Diode-emulation window near D = M: reverse-current drain limit

Date: 2026-11-17. Branch `de-margin` (off main fd988888). **Simulator and field-telemetry analysis only; nothing ran on
hardware.** Tools and data: `doc/research/de-margin-near-unity/` (`tools/dewin.cpp`, `tools/pair.sh`, `tools/simrun.sh`,
`data/pair-summary.txt` for the first 0.02 T rule, `data/pair-summary-floor700.txt` for the final budget; `tools/pair.sh`
regenerates the full paired dumps, which are not committed).

## Problem

The rect-cal band sweep (`2026-11-17-rect-cal-late-knee.md`, "Out-of-scope finding") found that with offset 0 the DCM
LS window itself (`computePwmRectMax`, `M_WCE = M_read / 0.980198`) drives the mean output current negative near D = M
when Vout reads low or Vin high. flat reads Vout 0.3-0.45 V (1.1-1.7 %) low while converting.

## Mechanism

1. **The margin is fine as a ratio, the failure is in time.** The 2 % ratio bias puts the LS turn-off
   `0.0198 * t_HS / M` early, about `0.02 T` = 0.5 us (82 counts at 4103 / 39 kHz) near D = M. It does not collapse as
   M -> 1. A ratio reading error moves the crossing by about `(dM/M) * T` near D = M, so a 2 % error uses the margin up,
   as designed.
2. **Ring current at HS turn-on.** The DCM ring leaves `i0` (up to about `Vo/Z0`) in the coil when HS turns on; the
   crossing moves by `i0 * L / Vo`, up to `sqrt(LC)` (0.68 us fry, 0.5 us flat). With exact readings the 2 % margin just
   covers it (no negative points); a reading 1.5 % off (fry, 82 counts margin minus ~60 of error) does not.
3. **Drain and carry-over.** Late conduction `delta` leaves `-Vo*delta/L` at LS off, which drains through the HS body diode
   at `(Vi - Vo)/L`, i.e. in `delta * M/(1-M)` (9x at M = 0.9). Near D = M the off-time after the window is short, so
   the rest is still flowing at the next HS turn-on: the next peak is lower, the next crossing earlier, the window later
   again. The plant settles in a forced-CCM-like state (worst point fry, Vo -2 %: Vin 29.2 V, Vo 26.8 V, HS 3701 of
   M*T 3765, LS 333, coil current at HS turn-on -0.75 A, mean Iout -0.38 A).
4. **Sensor errors that matter:** Vout low or Vin high (both raise the true M above the read one). Vout high / Vin low
   only make the window earlier.

Separate, larger, not fixed here: **the LS minimum pulse** (`boot_refresh_ns` 2 us + dead time = 321 counts, 7.8 % of T).
With exact readings every negative-current point on the stiff plants is at the min pulse. The carry-over estimate
(near D = M the pulse's reverse current cannot drain before the next HS once M > 1 - t_min/T = 0.922) is approximate:
each period the pulse also pumps charge from the battery back into the input without any carry-over, and the plants go
negative from M ~ 0.92 at every HS count (stiff source: up to -234 A; the converter runs as a boost from the battery)
and from M 0.865-0.9 at HS <= ~1200. Below that only at light load, where the pulse exceeds twice the ideal window
(fry exact readings: M 0.8-0.92, D < 0.18-0.4, down to -0.27 A). In the closed loop
with a PV source it lifts Vin to about Vo/(1 - t_min/T) = 1.085 Vo; with a nearly dark panel the input rises until the
panel conducts. mppt-sim `wx-dusk` shows it: the converter starts at Vin 28.5 / Vout 26.6 (M 0.934, allowed by the
start condition Vin > Vout + 1 V) and trips `revI-ewm` (avg Iout < -1 A) 3 s later, 1-2 times per dusk, old and new
rule alike.

## Field exposure (InfluxDB ha_van/mppt, 2026-10-18 .. 11-17, read-only)

`pwm_dcm` is logged at ~4.7 Hz while converting. Converting time: fry 232 h, flat 227 h.

| M = Uo/Ui > | fry DCM h (% conv.) | fry CCM h | flat DCM h (% conv.) | flat CCM h |
|---|---|---|---|---|
| 0.80 (Vin/Vout < 1.25) | 12.0 (5.2) | 9.3 | 13.9 (6.1) | 0.6 |
| 0.90 | 6.2 (2.7) | 4.2 | 6.9 (3.0) | 0.2 |
| 0.95 | 2.8 (1.2) | 0.2 | 3.6 (1.6) | 0.1 |
| 1.00 | 2.4 (1.0) | 0 | 1.2 (0.5) | 0 |

- DCM at M > 0.8 carries little: median P about 2.2-2.4 W, p95 10 W, max 27-37 W; median I 0.08-0.09 A.
- Duty close to the ratio: while regulating, D/M > 0.98 in 18 % of the DCM M > 0.8 time (31 / 28 % at M > 0.9), i.e.
  about 2 h per board per month sit in the band where the old window could carry over.
- Measured I < 0 in DCM at M > 0.8: fry 26 % of the time (most of it 1.95 h at the duty floor in CV with a full pack:
  the min pulse), flat 4 %; min -0.15 / -0.13 A. Mostly at dusk (16-20 h local), some at dawn.
- Console log (Nov 3-17): `Low current ... disable backflow` (the -0.05 A backstop) a few times per day per board (flat
  568 on Nov 9, its MCPWM switch-over day), one `Reverse I -1.05A` on flat (Nov 4, dawn, pwm 568), no `revI-ewm`.

So the field sees the region daily, at low power, and the existing -0.05 A backstop catches what reaches the sensor.

Where the final limit binds (logged pwm_duty, Ui, Uo; exact-reading condition D/M_WCE > 1 - late/(T r_WCE), late
0.0273 T): about 3.4 h per board per month, 42 % (fry) / 32 % (flat) of the DCM M 0.8-1 regulating time; by M bin
0.8 / 0.85 / 0.9 / 0.95: fry 9 / 46 / 74 / 99 %, flat 11 / 14 / 67 / 100 % (with 0.02 T alone: 2.7 h, 33 / 26 %). It binds
at the power levels above (median ~2.3 W), so the field cost is a fraction of the mW-level extra loss below.
A stiff-source-like case (bench PSU, power-loop rig, a shared battery bus at the input) is where the old rule is
dangerous (amps).

## Fix

`SynchronousConverter::computePwmRectMax` (DCM branch), after the late-offset rule:

```
late = max(pwmMax * LateDrainBudget, lateDrainTicks)  // LateDrainBudget = 1 - (1-e)/(1+e) = 0.0198, e = VoltageMaxErr
if (r > 0 && r * (comp - ideal) < late)               // comp = pwmMax - pwmCtrl - 1, r = pwmRectRatioDCM
    ideal = comp - late / r;
```

`lateDrainTicks` = ceil(700 ns * tick rate), computed once in init. The fraction term is the time the WCE ratio bias
keeps the window early near D = M (0.5 us at 39 kHz), so the budget covers a reading error the size of the margin
again. The floor covers the ring shift sqrt(LC) = 1/(2 pi f_ring), which is a fixed time (0.68 us at the lowest
measured board ring, 233 kHz; flat 320 kHz = 0.5 us); the real configuration has no ring frequency or node C (only
`vconv.conf` does), so a constant floor. At 39 kHz the floor wins (0.0273 T); below ~28 kHz the fraction does.

Binds (exact readings) for D/M_WCE > 1 - late/(T r_WCE); at 39 kHz in true D/M from 0.99 (M 0.5), 0.98 (0.6),
0.95 (0.7), 0.90 (0.8), 0.84 (0.85), 0.71 (0.9), 0.14 (0.95). With 0.02 T alone: 0.97 (0.7), 0.93 (0.8), 0.79 (0.9),
0.39 (0.95). So "only within the last few percent below D = M" holds for M <= 0.7 only; above M ~ 0.9 the limit
binds in most of the DCM range, and above ~0.93 the min pulse is longer than the limit anyway.

The divide runs only where the limit binds; it is the ROM `__divsf3`. Inlined into the IRAM `pwmPerturb` /
`updateSyncRectMaxDuty`; `rtprof_map.py --check`: 0 not in IRAM. Where the limit is below the min pulse the min pulse is
issued (unchanged clamp).

Rules screened (`tools/dewin.cpp`, emulated rules, 4-count far steps; window points with Iout < 0, fry Vo -2.5 / -3 %):

| rule | fry -2.5 / -3 % | flat -3 % | exact-reading eta drop max, M 0.5-0.7 / 0.8-0.9 |
|---|---|---|---|
| none (old) | 8473 / 12699 | 10286 | - |
| T/32 gap | 3676 / 5827 | 3223 | 0.07 / 0.16 pp |
| T/32 + 20/r | 1451 / 3052 | 657 | 0.12 / 0.35 pp |
| T/32 + 40/r | 191 / 979 | 0 | 0.17 / 0.47 pp |
| 20/r | 4395 / 8633 | 5650 | 0 / 0.29 pp |
| 40/r | 1058 / 3919 | 924 | 0.02 / 0.35 pp |
| 80/r (first choice, 0.02 T) | 0 / 0 | 0 | 0.14 / 0.47 pp |
| no sync rect above M_WCE 0.85 | 3529 / 6394 | 5875 | 0 / 0.47 pp |

Disabling sync rect above some M does not help: the LS min pulse remains and is itself late there.

### Before / after with the real class, first rule (0.02 T) (`tools/pair.sh`, every HS count from 200 to M*T, Vo 26.8 / 29.2 V, Vin Vo+0.6 V .. 80 V)

Window points (LS above the min pulse) with mean Iout < 0, old -> new, offset 0, 287 889 points per case:

| reading | fry | fry-hq | flat | flat-hq |
|---|---|---|---|---|
| exact | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 |
| Vo -1 % | 0 -> 0 | 10 -> 0 | 0 -> 0 | 0 -> 0 |
| Vo -1.5 % | 818 (-0.13 A) -> 0 | 884 (-0.14 A) -> 0 | 0 -> 0 | 0 -> 0 |
| Vo -2 % | 3531 (-0.38 A) -> 0 | 3611 (-0.39 A) -> 0 | 1218 (-0.38 A) -> 0 | 1247 (-0.39 A) -> 0 |
| Vo -2.5 % | 7527 (-1.72 A) -> 0 | 7593 (-1.72 A) -> 0 | 4907 (-1.92 A) -> 0 | 4966 (-1.92 A) -> 0 |
| Vo -3 % | 11412 (-9.67 A) -> 0 | 11484 (-9.67 A) -> 0 | 8836 (-6.70 A) -> 0 | 8888 (-6.70 A) -> 0 |
| Vi +1.5 % | 759 (-0.12 A) -> 0 | 805 (-0.13 A) -> 0 | 0 -> 0 | 0 -> 0 |
| Vi +2 % | 2994 (-0.36 A) -> 0 | 3107 (-0.37 A) -> 0 | 780 (-0.21 A) -> 0 | 860 (-0.21 A) -> 0 |
| Vi +2.5 % | 7030 (-1.46 A) -> 0 | 7151 (-1.47 A) -> 0 | 4437 (-1.81 A) -> 0 | 4481 (-1.81 A) -> 0 |
| Vi +3 % | 10679 (-7.66 A) -> 0 | 10796 (-7.66 A) -> 0 | 8119 (-5.22 A) -> 0 | 8166 (-5.22 A) -> 0 |

Also with `rect_offset_ns` +63 / +125 / +250 (loaded as +125) at exact / Vo -2 / -2.5 / Vi +2.5 % (2-count steps):
0 window points negative after. In all 92 cases: **0 points where the new rule turns a non-negative mean current
negative.** The min-pulse negative count rises only by points that were already negative (window -> min pulse).

Review residuals of 0.02 T alone (budget a fraction of T, no time floor): fry Vo -2 % AND Vi +2 % 1333 window points
down to -0.22 A; Vo -4 % fry 1726 / -0.35 A, flat 210 / -0.56 A. I.e. it holds up to ~3 % total ratio error.

### Before / after, final budget max(0.0198 T, 0.7 us) (`data/pair-summary-floor700.txt`, 2-count far steps, 180 073 points per case)

Window points with mean Iout < 0, old -> new:

| reading | fry | fry-hq | flat | flat-hq |
|---|---|---|---|---|
| exact (offset 0, 125) | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 |
| Vo -2 % (offset 0, 125) | 3531 (-0.38 A) -> 0 | 3611 (-0.39 A) -> 0 | 1218 (-0.38 A) -> 0 | 1247 (-0.39 A) -> 0 |
| Vo -3 % (offset 0, 125) | 11317 (-9.67 A) -> 0 | 11371 (-9.67 A) -> 0 | 8836 (-6.70 A) -> 0 | 8888 (-6.70 A) -> 0 |
| Vi +2.5 % | 7030 (-1.46 A) -> 0 | 7151 (-1.47 A) -> 0 | 4437 (-1.82 A) -> 0 | 4481 (-1.82 A) -> 0 |
| Vo -2 % and Vi +2 % (offset 0, 125) | 18456 (-37.8 A) -> 0 | 18501 (-37.8 A) -> 0 | 16578 (-27.4 A) -> 0 | 16616 (-27.4 A) -> 0 |
| Vo -4 % | 19043 (-40.3 A) -> 1 (-0.001 A) | 19097 (-40.3 A) -> 10 (-0.009 A) | 17231 (-29.2 A) -> 0 | 17262 (-29.2 A) -> 0 |

(offset-125 counts differ by a few percent, all -> 0.) In every case 0 points where the new rule turns a non-negative
mean current negative.

### Cost (exact readings, points where the LS count changes)

| | M < 0.5 | 0.5-0.7 | 0.7-0.8 | 0.8-0.9 | > 0.9 |
|---|---|---|---|---|---|
| eta drop max, fry / flat [pp], final | 0.035 / 0.005 | 0.20 / 0.03 | 0.51 / 0.13 | 0.55 / 0.20 | 0 |
| extra loss mean, fry / flat [mW], final | -4.9 / -2.3 | -1.7 / -0.4 | 5.2 / 9.0 | 5.5 / 6.0 | 0 |
| eta drop max, fry / flat [pp], 0.02 T | 0.00 / 0.00 | 0.15 / 0.02 | 0.37 / 0.07 | 0.48 / 0.20 | 0 |

The largest mean-Iout changes (up to 2.1 A fry, 0.75 A flat) are at HS =
M*T exactly on the stiff plant, where the old window let the current build into CCM (2.9 A); the firmware would be in
CCM there by its own current test (DCM exit at ripple <= 1.8 I), so it is not a cost in operation.

Closed loop, first rule 0.02 T (mppt-sim, fry plant with ring, Vout gain 1 / 0.983 / 0.975 and Vin gain 1.02; pgx93): wx-sun-fry at Voc
44-52 V and 5-30 W (Vin 30-34 V, M 0.82-0.92), wx-shade3 (30 V peak), wx-dawn, wx-dusk, wx-cloud-cv, wx-low: harvest
old vs new within -0.3..+0.1 pp, trips identical (the dusk `revI-ewm` is the min pulse, both rules). The full default
scenario set: 40 of 44 identical, wx-dawn +0.1, wx-dusk -0.3, wx-low -0.5, wx-low-slow +0.1 pp; 8 seeds: wx-low old
98.13 +- 0.45 vs new 97.91 +- 0.53 %, wx-low-slow 98.48 vs 98.35 % (difference within the seed spread; in wx-dusk the
rule binds fewer than 2000 times in 2200 s, so the difference is trajectory divergence). With a PV source the old window's
negative band does not show up as lost energy: the operating point moves away (Vin rises) instead of the current
turning negative.

Closed loop, final budget (`msim-new2` vs old, pgx93, same scenario list and gains): harvest -1.2 .. +0.7 pp in single
runs, trips identical. 8 seeds: wx-low 98.12 +- 0.44 -> 97.78 +- 0.68 %, wx-low-slow 98.47 +- 0.17 -> 98.49 +- 0.14 %.
wx-dusk does not depend on the seed: 98.4 -> 97.7 % (all 8 identical); its trajectory splits at the first sweep
(t = 36 s, D 999 vs 996) and the old firmware alone gives 97.6-98.6 % across Vout/Vin gain errors, so one run does not
separate a cost from trajectory sensitivity. A real ~0.5 pp dusk cost cannot be excluded from these runs.

## Tests (final budget)

- `rect-cal-test`: PASS (fails=0).
  - `testDrainGap` (fry plant, Vo read 2 % low, Vin 29.2/30/32/36 V, HS over the last 150 counts below M*T): 51 window
    points, 0 negative, 153 at the min pulse. Known-bad (budget 0, floor 0): 125 of 204 negative, -0.38 A.
  - `testDrainNoop` (Vin 60 / Vout 30 exact, HS 200..M*T step 7): 245 points, the limit equals floor/ceil(hs*r)
    wherever it should not bind, 3 bound, first from D/M 0.991. Fails with budget/floor 0 (none bound) and with a 0.2
    budget (binds from D/M 0.81).
  - `testDrainBoost` (boost, ctrl counts over the last 300 below (1 - Vl/Vh) T, Vl read 2 % high): 101 points, 6
    bound, all leave late/r before the next ctrl edge. Structural only: no boost plant.
  - `testLateGap` min idle after a late LS 148 ct.
- vconv-spice-check: PASS (self-tests fail as expected).
- `rcal-vin.sh` (Mac, `msim-new2`, 192 runs): 0 outside the band, 23 failed runs (offset kept; 25 with 0.02 T). Board
  means fry +143, fry-hq +232, flat -228, flat-hq -188, pooled -10 (baseline -9) ns. `data/rcal-vin.txt`.
- dcm-current-test, vconv-test: pass (0.02 T rule; the limit does not touch them).
- Unity QEMU suite: 262 tests, 0 failures, 25 ignored. esp32s3 build (IDF 6.0.3): pass, 11 % free.
  `rtprof_map.py --check`: 0 not in IRAM; `lateDrainCounts`/`computePwmRectMax` have no out-of-line symbol (inlined
  into IRAM `pwmPerturb`).
- Docs build: pass.

## Proposals (not applied)

- **flat Vout offset.** Not recommended as a conf change: the error appears only while converting, so a static
  `lv_v_offset` would make the idle reading high (and the charger's termination later). If it is a load-dependent drop
  between the converter terminal and the sense point, measure the terminal voltage against the reading at 2-3 currents
  first. With the drain limit the window tolerates ratio errors up to 3 % in the models, so the correction is no longer
  needed for diode emulation.
- **LS min pulse at high M.** Block starts below `Vin = Vout / (1 - (pwmRectMin + T/32)/T)` (about 1.1 Vout with the
  defaults) instead of `Vout + 1 V`; costs nothing (a panel with Voc that close to Vbat delivers almost no power) and
  removes the dusk `revI-ewm` trips in mppt-sim. A running-converter guard (or a shorter refresh pulse at high M,
  bench-validated against bootstrap UVLO) would be the complete fix.

## Not done

- No hardware test; plants are the SPICE-fitted host models. The ring shift depends on the board's switch-node C.
- Current-sensor error, Vin/Vout noise and the firmware's DCM/CCM decision are not in the stiff sweeps (Iout read fixed
  at 0.05 A, DCM forced); the closed-loop mppt-sim runs cover them.
- Boost: the same expression applies (drain through the LS body diode in `delta / r`); only the structural host test,
  no boost plant sweep.
- The min-pulse hazard above M 0.92 (and M 0.865-0.9 at low HS) is documented, not fixed.
- The 0.7 us floor is a constant for the lowest measured ring frequency; a board with a lower ring (more node C, larger
  L) needs a longer floor. Untested at other switching frequencies.
- The wx-dusk -0.7 pp (single deterministic scenario) is not resolved into cost vs trajectory sensitivity.
