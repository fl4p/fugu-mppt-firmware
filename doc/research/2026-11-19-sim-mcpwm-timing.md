*this document is an LLM generated placeholder*

# Host simulator on the live MCPWM timing (4103 counts)

Date: 2026-11-19. Branch `sim4103` (off main e20e12ca). **Simulator only; no device was touched** (the live values
were read from the console log on havan, read-only). Data and tools: `doc/research/sim-mcpwm-timing/` (`run1.sh` one
job, `jobs.txt` the 2272 jobs, `results.txt` one line per job, `analyze.py` -> `report.md` all tables). All closed-loop
tables are pgx93 runs (tag `sim4103`); `rcalvin-*.txt`, `mps-*.txt` and the ring/rcal-flat checks are Mac runs and are
compared only with each other.

## Problem

The `PWM_VConv` shim of the host simulator mirrored the LEDC driver: 80 MHz, power-of-two period, so at
`pwm_freq` 39000 the fry/flat presets ran at pwmMax 2047 (12.53 ns per count). fry and flat have run the MCPWM driver
since 2026-11-08/09: 160 MHz, period 4103 (6.25 ns per count). Every tracker and diode-emulation sim result before
this note used the LEDC count resolution.

## Live boards (console log, read-only)

| | fry | flat |
|---|---|---|
| driver, period | MCPWM, `period_ticks=4103 res=160000000 pwmMax=4103 maxHS=3782`, realized 38995.86 Hz | same |
| board.conf | `pwm_freq=39000`, HiLi, no `pwm_deadtime_*`, no `boot_refresh_ns` (default 2000 ns) | same (`dt: module bypassed at boot`) |
| min LS pulse | 2000 ns = 320 counts (maxHS 3782 = 4103 - 320 - 1) | same |
| coil.conf | `L0=80e-6`, `rect_offset_ns=-1312`, `rect_offset_drv=mcpwm`, `rect_offset_cal=1` (rect-cal 2026-11-15: learned -162 ct, applied -210 ct) | `L0=56e-6`, `rect_offset_ns` 0 (uncalibrated; 875 until 11-09) |

## Presets: before and after

| | before (still `-ledc` / `SIM_DRV=ledc`) | now (fry, flat, -hq, -off, -damped) |
|---|---|---|
| shim timing | LEDC, 80 MHz | MCPWM `bestTiming`, 160 MHz (`vconv.conf::pwm_clk=160000000`) |
| pwmMax, count | 2047, 12.53 ns | 4103, 6.25 ns |
| minLS / maxHS | 160 / 1886 | 320 / 3782 (= live) |
| tick rate for ns -> counts | fsw x pwmMax (79.8 MHz) | 160 MHz (as MCPWM) |
| plant switching frequency | 39000 Hz | 38995 Hz (realized) |
| rect_offset | 0 | fry -1312 ns (-210 ct), flat 0; rcal* scenarios start at 0 |
| L0 conf / plant L, ring, t_dead 185 ns, R, t_sw | unchanged | unchanged |

`SIM_DRV=ledc` (or the `-ledc` suffix) reproduces main bit for bit (wx-dusk, wx-sun-flat full output compared).
`SIM_PLANT=fry:rect_offset_ns=0` gives the MCPWM timing without the live offset (`mcpwm0` below). Scenarios without a
board (sanity, a..e, shade, rcal-sun, ...) stay on LEDC timing; `SIM_DRV=mcpwm` switches them.

Not modelled: the TEZ-latched comparator commit (the shim applies writes at once, as before), the INA226 OV-alert
force-off ISR, and why fry needs -1312 ns: on the sim plant the zero crossing sits within about +0.3 us of ideal
(rcal-vin learns +0.2 us), so -1312 ns turns the LS off about 1.5 us early there. The live-offset preset is what the
firmware runs, not a net-timing match; `mcpwm0` is the other bound.

## Host tests

| test | result |
|---|---|
| vconv-test, rect-cal-test, dcm-current-test | pass (unchanged; they already run the MOCK driver at 4103) |
| minpulse-test | **4 failures, same on main e20e12ca** (`no pump above the onset`, Vout 13.4, all four boards): the known-bad pump check, pre-existing, not a preset effect; not fixed here |
| minpulse-sim.sh | pass on both timings (Mac; above onset 0 s, no revI, stops <= 1, DCM entry to stop 2.3 / 2.2 ms) |
| rcal-vin.sh (Mac, 4 seeds, 192 runs) | pass on both. Learned mean MCPWM / LEDC / baseline [ns]: fry +185 / +255 / 162, fry-hq +263 / +290 / 234, flat -240 / -160 / -231, flat-hq -202 / -198 / -190, pooled +4 / +38 / -9. Baselines not changed. The script now converts counts with each run's count duration |
| rcal-flat, ring-sweep/weak/coil (fry, flat) | same in time on both: rcal-flat learned -263 / -262 ns; ring-coil: new fit within 0.2 %, old 10-step median within 1.1 % |
| sim/build.sh | fixed (rect-cal persist signature, missing `g_hwOvTrip`) |

## Re-baseline (pgx93)

Standard set: fry and flat, 5 noise cases (noise-free, Gaussian seeds 1-2, replay offsets 0/8 s), paired by case.
Harvest = TRACKER line, % of the reachable GMPP. Full table in `report.md` (S1). Summary:

- **No scenario moved beyond the spread except two small gains**: wx-wob-fry +0.19 pp (se 0.09), wx-mp-ccm fry
  +0.12 (0.06). Sweeps, shade, cloud, cloud-cv, mid, sun, the minpulse set: |diff| <= 0.13 pp. Trips identical
  except wx-calfail fry `calib-fail` 10 -> 13 and wx-low-slow flat sweep-no-mpp 0 -> 2.
- **fry at dusk is slightly worse on MCPWM**: wx-dusk (SIM_VBAT 28, 36 paired runs) -0.26 pp (se 0.10) with the live
  offset, -0.22 (0.08) at offset 0; fry-hq -0.16 / -0.11; flat 0.00, flat-hq -0.09 (0.05). It is the resolution, not
  the offset. Single runs reach -2.5 pp; wx-dusk-land-fry -0.71 (se 0.68), wx-calfail fry -0.82 (0.51), wx-mp-ramp
  fry -1.16 (1.33) are within their spread.

| scenario (5 cases) | fry LEDC -> MCPWM | flat LEDC -> MCPWM |
|---|---|---|
| wx-dawn | 97.50 -> 97.87 | 98.21 -> 98.25 |
| wx-dusk | 98.52 -> 97.84 | 98.36 -> 98.47 |
| wx-cloud | 98.70 -> 98.72 | 98.71 -> 98.78 |
| wx-cloud-cv | 91.91 -> 91.92 | 88.33 -> 88.33 |
| wx-low | 97.24 -> 97.22 | 97.95 -> 97.55 |
| wx-low-slow | 98.40 -> 98.21 | 98.61 -> 98.43 |
| wx-mid | 99.74 -> 99.74 | 99.77 -> 99.75 |
| wx-shade-walk | 86.60 -> 86.62 | 86.60 -> 86.62 |
| wx-shade2-move | 96.27 -> 96.35 | 96.32 -> 96.36 |
| wx-sweep-fall / -rise | 93.29 / 94.37 -> 93.37 / 94.44 | 93.46 / 94.46 -> 93.54 / 94.46 |
| wx-sun / wx-wob (own board) | 99.88 / 99.58 -> 99.89 / 99.77 | 99.82 / 99.86 -> 99.80 / 99.82 |

### Dusk trap (55087dd7 + 7e31e1b9)

wx-dusk, SIM_VBAT 28, 4 boards x 4 readings x 9 noise cases (144 per row); `pretrap` = the fix disabled
(`plant = false` in `Tracker::update`).

| build | LEDC | MCPWM live offset | MCPWM offset 0 (fry, fry-hq: 72) |
|---|---|---|---|
| pretrap: trapped (< 96 %) / worst | 3 / 85.0 (fry-hq) | **0** / 96.5 | 2 / 92.2 (fry) |
| main: trapped / worst | 0 / 96.9 | 0 / 96.5 | 0 / 97.0 |
| main - pretrap, mean (se) | +0.27 (0.14), up to +13.5 | **0.00**, -0.2..+0.1 | +0.20 (0.11), up to +5.9 |

The trap and the fix both hold at 4103 counts with the LS turning off at the zero crossing. With fry's live
-1312 ns offset the trap does not occur at all (the plant exception never fires: main = pretrap in every run).
On today's main the trap is rarer than in the original study (3 of 144 vs 10 of 304) at LEDC timing.

### Drain limit (a424a779)

`nodrain` = the DCM late-drain rule removed. wx-dusk SIM_VBAT 28, 4 boards x 4 readings x 5 noise cases; wx-low.

| timing | main - nodrain, all (n 80) | Vout 0.975 | minpulse-M stops without / with the limit |
|---|---|---|---|
| LEDC | -0.15 (0.07) | -0.55 (0.13) | 29 / 0 (fry, fry-hq) |
| MCPWM live offset | -0.07 (0.05) | -0.03 (0.07) | 0 / 0 |
| MCPWM offset 0 (fry, fry-hq, n 40) | +0.23 / +0.22 exact, -0.12 / -0.22 misread | | 19 / 0 |

wx-low: -0.01 on both. The limit still removes the late-window stops at 4103 counts; its harvest cost, -0.55 pp at
Vout 0.975 on LEDC (the study had -1.26 on fry), is not measurable on MCPWM. With fry's -1312 ns offset the window
is early anyway and the limit does nothing.

### Min-pulse guard

`noguard` = `minPulseRatioMax()` returns 10. Replay seeds 1-4, fry and flat, as in the guard study.

| scenario | revI trips noguard / guard (LEDC; MCPWM) | on above onset [s] noguard / guard (LEDC; MCPWM) |
|---|---|---|
| wx-mp-psu | 172 / 0; 157 / 0 | 815 / 0; 834 / 0 |
| wx-mp-manual | 548 / 0; 548 / 0 | 430 / 0; 422 / 0 |
| wx-mp-ccm | 72 / 0; 72 / 0 | 107 / 0; 108 / 0 |
| wx-dusk | 0 / 0; 0 / 0 | 148 / 0; 148 / 0 |
| wx-dawn | 0 / 0; 0 / 0 | 57 / 0; 56 / 0 |

Unchanged: the guard removes all time above the onset and all revI trips at both resolutions; stops (16 manual, 8
ccm, 4 psu) identical. M_stop moves only through pwmRectMin/T (160/2047 vs 320/4103, 0.0782 vs 0.0780). wx-dusk
had no revI trips without the guard on today's main (the study had 16): main changed, not the timing.

### Peak scan (pv_substrings=6 minus default)

Gains are the same at both resolutions (6 noise cases): shade-walk +10.27 / +10.46, shade2-move +2.78 / +2.77,
shade3-move +23.61 / +23.63, static shade 0.00, shade-cloud -0.14..+0.01 (se 0.06-0.13). Two cells differ from the
2026-11-18 record at **both** resolutions, so from main changes since: wx-cloud +0.20 (was -0.04) and wx-sweep-shade
+0.87 on a base of 98.4 % (was +8.3 on 85.2 %; the sweep verdict now handles that case).

### Settle window in slow mode (dfde81e2)

Slow power filter (filter length 60, 200 Hz), `oldsettle` = the 51b3fa8 rule. main - old, LEDC / MCPWM:
wx-wob-fry noise-free +4.65 / +4.62, replay -0.02 / +0.11 (se 0.39); wx-wob-flat +1.02 / +1.25, replay +0.30 / +0.24;
wx-dusk fry noise-free -0.18 / +0.30, replay +0.94 / +0.08; flat noise-free +0.09 / -0.84, replay +0.16 / -0.06
(se 0.14-0.43). The wob gain holds; the dusk side stays within the spread (the record had fry -0.8 noise-free).

## Conclusions that changed

1. **Dusk trap: holds, but not on fry's live configuration.** At offset 0 the trap and the fix reproduce at 4103
   counts. With the live -1312 ns offset no run traps, with or without the fix. Whether the real board has the
   trap depends on its true LS timing, which the sim plant does not model (see "Not modelled").
2. **Drain limit cost**: the -1.26 pp (fry, Vout 0.975) closed-loop cost from the de-margin study is an LEDC-timing
   number; at 4103 counts the cost is not measurable. Its protective effect (no late-window `minpulse-M` stops)
   holds.
3. **fry dusk harvest -0.2 pp at MCPWM** relative to every earlier fry dusk number; flat unchanged.
4. Unchanged at 4103 counts: min-pulse guard, peak-scan gains, slow-mode settle rule, rect-cal bias (rcal-vin
   baselines), all sweep/shade/cloud scenarios.

Differences from earlier records that are not timing effects (same at both timings): wx-cloud scan gain +0.2,
wx-sweep-shade base 98.4 %, pretrap 3 of 144 trapped, no dusk revI without the guard.

## Open

- The live fry offset (-1312 ns) puts the LS off ~1.5 us before the sim plant's zero crossing. A plant with the
  board's real LS turn-off delay would make the live preset a net-timing match.
- minpulse-test known-bad check fails on main (pre-existing).
- The ESP32-S3 vconv firmware build (`sdkconfig.vconv_s3`) could not be configured in the worktree
  (`idf::espressif__network_provisioning` missing from managed components); the changed shim and `getPwmTickRate()`
  compile in the host sim with `WITH_VCONV`.
