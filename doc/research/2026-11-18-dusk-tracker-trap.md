*this document is an LLM generated placeholder*

# Dusk tracker trap: P&O reversing on plant-driven Vin steps

Date: 2026-11-18. Branch `dusk-trap` (off main b5ee4cc2). **Simulator only; nothing ran on hardware.** Tools and
data: `doc/research/dusk-tracker-trap/` (`tools/run1.sh` one paired job, `tools/jobs-*.sh` job lists, `tools/final.py`
the tables below from `data/results.txt`; `data/dstep-*.txt` static P(D) curves; `data/trace-*.txt` decision traces).
All numbers in the tables are pgx93 runs (tag `dusk-trap`); the static curves and the wobble trace are Mac runs and
are not compared with them.

Follow-up after review (same day, rebased onto main 0c7ac4aa, pgx93 tag `dusk-trap2`): hand-back skip (below,
"Hand-back"), the 5 % rule named as the mechanism and the band as a guard, the field scope, the stale sim board
config. See "Follow-up validation".

## Scope: this does not fix the field dusk loss

The field shows a dusk loss that looks similar but is a different mechanism, and this change does not address it.
Reviewer's analysis of the 1 s field data (2026-10-29 to 11-18, InfluxDB): on fry at dusk the tracker sits at
about 2 W near 29-33 V while the sweeps find 4-10 W at 45-60 V. There D/M is about 0.5 (deep DCM, far from
D = M), there are no Vin steps at an unchanged duty, and the P-V curve is flat with bypass-diode humps: the tracker
steers there itself, with power changes below its thresholds. The trap studied here needs D close to M and a
plant-driven Vin step; it is only reproduced in the simulator. Nothing below should be read as a fix for the field
behaviour.

## Sim board config is stale

The `wx-*` fry/flat plants run the converter at pwmMax 2047 (`drv=vconv f=39000 pwmMax=2047`, the LEDC period).
The real boards run MCPWM at pwmMax 4103 since 11-09. All HS counts below are on the 2047-count period; the
fractions of the period carry over, the counts do not. The sim config is not changed in this branch.

## Problem

The de-margin study (`2026-11-17-de-margin-near-unity.md`) found runs in `wx-dusk` on the stiff board models where
P&O walks to high duty and low Vin (34 V with the MPP at 48 V) and stays there until the next sweep, up to -13 pp
of harvest. Measured power followed the real panel power, so no sensor bias.

## Reproduction on main

`wx-dusk`, 4 board models (fry, fry-hq, flat, flat-hq) x 4 readings (exact; Vout gain 0.983, 0.975; Vin gain 1.02)
x (noise-free, Gaussian ADC noise seeds 1-9, measured-noise replay offsets 0-16 s) = 304 runs, `SIM_VBAT=28`.

- Trapped (harvest < 96 %): **10 of 304**, all on fry / fry-hq, all with a misread ratio (Vout 0.983: 2,
  Vout 0.975: 3, Vin 1.02: 5), 8 of them with replayed measured noise. None on flat, none with exact readings.
- Worst 84.7 % (fry, Vout 0.975, replay offsets 2 and 16): tracking loss 12.2 %, Vin mean 34 V.
- The min-pulse Vin floor (Vout / M_start, about 30.4 V here) does not prevent the trap; it only bounds how far
  down the tracker goes. In the worst runs the tracker sits on that floor (Vin limiter, `lim 0`) for the last
  third of the evening.

## Mechanism

### Static P(D) near D = M (`data/dstep-fry-vo*.txt`)

`wx-dstep` holds the dusk light at the level where the trap develops (MPP 26 W at 48 V; Voc 61.6 V) and steps the
duty in MPPT mode with the tracker frozen (diode emulation as in tracking), up and back:

| HS count | Vin [V] | P_pv [W] | coil current at HS on [A] |
|---|---|---|---|
| 1000 | 44.3 | 26.0 | -0.14 |
| 1200 | 37.6 | 23.1 | +0.11 |
| 1300 | 37.1 | 22.76 | +0.02 |
| 1400 | 37.2 | 22.81 | -0.14 |
| 1475 | 37.3 | 22.89 | -0.27 |
| 1600 | 35.7 | 22.0 | -0.23 |

- From HS 1250 to 1500 (12 % of the period, just below D = M) Vin stays within 0.3 V. The DCM ring after the
  zero crossing leaves a current in the coil at the next HS turn-on that goes from +0.1 to -0.3 A as the idle time
  shrinks, which cancels the extra on-time.
- P(D) is flat with a slight rise toward higher duty: a local maximum at HS ~1475 (37.3 V). The dip back toward the
  MPP is 0.12 W (0.5 %), below both P&O thresholds (fast 1 W / 1.5 %, slow 0.5 W / 0.15 %).
- Exact readings and Vout read 2.5 % low give the same curve: it is the plant, not the reading. Up and down sweeps
  agree (no static hysteresis at fixed light).

### Closed loop (`data/trace-*.txt`)

In the closed loop the plant has a second effect: **Vin steps down by 3 to 5 V within 0.3 s at an unchanged duty**
(fry, Vout 0.975, replay 2: Vin 49.5 -> 45.1 V at HS 1053 -> 1046, t = 90.8 s; replay 16: 50.3 -> 45.4 V at HS 1037,
t = 87.1 s, and 45.1 -> 41.5 V at HS 1152, t = 143 s, slow mode). The LS window is at its cap (HS + LS ~ the
period) in all of them.

Each step happened while the tracker steered Vin **up** (direction "-", lower duty). Two rules can then reverse it
toward lower Vin; the 5 % rule does almost all of it. An instrumented build (reviewer's, logging every held
reversal) held 36-47 reversals by the 5 % rule per trapped run and one by the power rule:

1. **The 5 % Vin rule** ("cloud recovery", reverse when Vin moved 5 % since the last reversal): at t = 91.1 s the
   power drop was 0.50 W, below both thresholds; the 9 % Vin drop reversed the tracker. The rule assumes Vin moved
   the way the tracker pushed it.
2. **The power rule**: at t = 143.9 s (slow mode) the step cost 1.35 W, above the threshold, and the drop was read as
   the result of the tracker's own step.

After the reversal the tracker steers down, the next step repeats it, and it ends on the flat local maximum of
P(D) far below the MPP, where neither threshold sees the way back. Light keeps falling, so the trap moves down with
it until it reaches the min-pulse Vin floor.

Ruled out: the settle window (the reversals were outside it), limiters (`lim 15`, no limiter until the floor),
DCM/CCM switching in the firmware (`dcm=1` throughout), quantization (the steps are 3-5 V).

## Fix

`Tracker::update()` (`src/tracker.h`): a reversal needs Vin to have moved the way the tracker steers.

- `against` = how far Vin moved against the steering since the last reversal (0 when the reference is unknown).
- 5 % rule: counts only a move in the steered direction. This is the change that removes the trap.
- Power rule, as a guard: a drop over the threshold reverses only if `against <= 0.01 * Vin_ref`
  (`VinAgainstBand`). The reference power still follows the drop. It mattered once per trapped run; it is kept so
  a power drop that comes with a plant step cannot do what the 5 % rule no longer does.
- Hand-back: both exceptions apply only from 3 s (`HandBackMs`) after a limiter hand-back (`resetDirection`,
  which also ends the fade after a sweep and the peak scan). Before that, both rules act as on main.
- The settle window keeps its "not back yet" test (`against > 0`), unchanged. It still ends after 1 s at the
  latest, but past that a power drop with Vin more than 1 % against does not reverse.

One float compare more per decision, no new state, inlined into the RT path (`rtprof_map.py --check`: 0 not in
IRAM). A NaN Vin or an unknown reference falls back to the old behaviour.

### Hand-back

The reviewer's -0.6 pp wx-low run (flat, exact readings, noise-free, 98.1 -> 97.5 %) came from the hand-back.
`resetDirection()` runs every tick while sweeping, fading or limiting, so the reference is the Vin EWM at the last
fade tick, while Vin is still moving toward the MPP. Vin then overshot it by 1.6 % in the steered-against
direction, the hand-back direction is that of the last fade step, and the first real reversals were held
(`BLOCKP vin=45.14 ref=44.43`). Skipping both exceptions for 3 s after a hand-back restores main's behaviour
there: the same run gives 98.1 % on the reviewer's base and on the rebased branch.

Test: `test_tracker_handback_overshoot_not_held` (Vin overshoots the hand-back reference by 1.6 % and power
drops: reverses; the same after the 3 s skip: held). Known-bad: fails on the branch's previous tracker.

### Why a 1 % band

Without a band (variant `v2a`) all 10 traps cleared, but **wx-wob-fry lost 1.95 pp on average, 5 of 20 runs below
96 %**: in DCM a light dip lowers Vin and power together (in CCM against the battery Vin = Vout/D does not move
with the light), so every reversal of a leg toward higher Vin was blocked and
none toward lower Vin; the tracker ratcheted up the steep side (flat model, noise-free: Vin 68 V vs MPP 64 V,
tracking loss 0.16 -> 4.6 %). The 0.5 % / 0.2 Hz wobble moves Vin by about 0.4 %; the plant steps are 5-10 %.

Screen (10 trapped dusk runs, 20 wx-wob-fry, 20 wx-low; paired with main):

| band | trapped dusk: mean diff, < 96 % after | wx-wob-fry mean / worst | wx-low mean / worst |
|---|---|---|---|
| 0 (v2a) | +6.61 pp, 0 | -1.95 / -4.7 pp | -0.19 / -2.4 pp |
| 1 % (d1, chosen) | +5.94 pp, 0 | 0.00 / 0.0 | -0.03 / -0.6 |
| 2 % | +5.70 pp, 1 | 0.00 / 0.0 | 0.00 / 0.0 |
| 3 % | +4.81 pp, 3 | 0.00 / 0.0 | 0.00 / 0.0 |

Rejected first step (`v1`, 5 % rule only): 2 of 10 trapped runs fixed; the power-rule path (slow mode) remains.

## Validation (pgx93, final build = d1, verified identical on 4 runs)

### wx-dusk, 304 paired runs

| subset | n | main mean | new mean | diff mean (se) | worst / best | worse / better | < 96 % |
|---|---|---|---|---|---|---|---|
| all | 304 | 97.83 | 98.16 | +0.33 (0.08) | -1.2 / +13.6 | 24 / 87 | 10 -> 0 |
| fry | 76 | 97.37 | 98.16 | +0.79 (0.26) | -1.2 / +13.6 | 9 / 40 | 5 -> 0 |
| fry-hq | 76 | 97.66 | 98.16 | +0.50 (0.14) | -0.9 / +6.1 | 11 / 37 | 5 -> 0 |
| flat | 76 | 98.10 | 98.10 | 0.00 | 0 / 0 | 0 / 0 | 0 -> 0 |
| flat-hq | 76 | 98.19 | 98.20 | +0.01 (0.01) | -0.1 / +0.2 | 4 / 10 | 0 -> 0 |
| exact readings | 76 | 98.09 | 98.09 | 0.00 | -0.1 / +0.2 | 2 / 3 | 0 -> 0 |
| Vout 0.983 | 76 | 97.97 | 98.19 | +0.22 (0.07) | -0.9 / +2.8 | 6 / 24 | 2 -> 0 |
| Vout 0.975 | 76 | 97.56 | 98.15 | +0.59 (0.26) | -1.2 / +13.6 | 10 / 32 | 3 -> 0 |
| Vin 1.02 | 76 | 97.69 | 98.19 | +0.50 (0.14) | -0.8 / +6.1 | 6 / 28 | 5 -> 0 |
| noise-free | 16 | 98.02 | 98.19 | +0.17 (0.07) | 0 / +0.8 | 0 / 5 | 0 -> 0 |
| Gaussian | 144 | 98.01 | 98.14 | +0.13 (0.04) | -1.2 / +2.8 | 13 / 36 | 2 -> 0 |
| replay | 144 | 97.62 | 98.16 | +0.54 (0.15) | -0.6 / +13.6 | 11 / 46 | 8 -> 0 |

Trips: identical in 303 of 304 (one `sweep-no-mpp` backoff more after the light is gone). Lowest new run 96.7 %
(fry, Vout 0.975, seed 4; main 97.9): late-dusk drift below 10 W (see risks), not the trap; the worse runs are
within the spread of main's own Gaussian seeds on that cell (97.3-98.6 %).

### No-regression set, 380 paired runs

fry and flat x (exact, Vout 0.975) x (noise-free, Gaussian seeds 1-2, replay offsets 0 and 8), 20 per scenario.

| scenario | main mean | diff mean | worst / best | runs changed |
|---|---|---|---|---|
| wx-low | 97.67 | -0.03 | -0.6 / +0.1 | 2 |
| wx-cloud-cv | 94.12 | +0.02 | 0 / +0.3 | 2 |
| wx-dawn, wx-cloud, wx-mid, wx-sun-fry, wx-wob-fry | | 0.00 | 0 / 0 | 0 |
| wx-shade2, -shade3, -shade2-move, -shade3-move, -shade-walk, -shade-cloud, -shade-learn | | 0.00 | 0 / 0 | 0 |
| wx-sweep-rise | | 0.00 | 0 / 0 | 0 |
| wx-mp-manual, wx-mp-ccm, wx-mp-ramp, wx-mp-psu | | 0.00 | 0 / 0 | 0 |

Trips identical in all 380. The -0.6 pp single wx-low run (flat, exact readings, noise-free: 98.1 -> 97.5 %) was
the hand-back problem (see "Hand-back"), fixed in the follow-up.

### Follow-up validation (hand-back skip, rebased onto main 0c7ac4aa)

pgx93, tag `dusk-trap2`, paired against the rebased main (`base`), 140 pairs. Job list `data/jobs-followup.txt`, runner
`tools/run1.sh` with the tag and paths changed; results in `data/results-followup.txt`.

| set | n | base mean | diff mean | worst / best | changed | < 96 % |
|---|---|---|---|---|---|---|
| wx-dusk fry, misread (Vout 0.983, 0.975; Vin 1.02) | 24 | 97.25 | +0.91 | -0.5 / +14.0 | 18 | 1 -> 0 |
| wx-dusk fry-hq, misread | 24 | 97.72 | +0.36 | -0.9 / +2.4 | 19 | 1 -> 0 |
| wx-low (fry, flat; exact, Vout 0.975; 5 noise cases) | 20 | 97.79 | 0.00 | 0 / 0 | 0 | 0 -> 0 |
| wx-low-slow (same) | 20 | 97.45 | 0.00 | 0 / 0 | 0 | 4 -> 4 |
| wx-wob-fry (fry, flat; 2 readings; noise-free, seeds 1-2) | 12 | 99.67 | 0.00 | 0 / 0 | 0 | 0 -> 0 |
| wx-cloud (same) | 12 | 96.54 | 0.00 | 0 / 0 | 0 | 0 -> 0 |
| 7 shade scenarios (fry, flat; seeds 1-2) | 28 | 92.93 | 0.00 | 0 / 0 | 0 | 12 -> 12 |

dusk cells are noise-free, Gaussian seeds 1-4 and replay offsets 0/4/8.

- Trips identical in every pair.
- The worse dusk runs (down to 97.3 %) are within base's spread across Gaussian seeds on the same cells
  (96.5-98.5 %).
- Against the previous fix on the same 48 dusk cells: -0.05 pp mean. The hand-back skip costs no measurable part of
  the dusk gain (the d1 runs are paired with the old main, so this comparison also carries the rebase).
- The reviewer's -0.6 run: wx-low flat, exact, noise-free gives 97.4 % on base and on the branch. On the reviewer's
  own base tree, with only `tracker.h` replaced, it gives 98.1 % (base 98.1, previous fix 97.5).

## Tests

- `test/test_tracker.cpp`: `test_tracker_vin_drop_against_steering_keeps_direction` (the dusk step: 9 % Vin drop
  with a 0.5 W loss, then a 2 W loss, no reversal; reverses again once Vin is back),
  `test_tracker_vin_rise_against_steering_keeps_direction` (other direction, and slow mode),
  `test_tracker_vin_against_band` (0.6 % against still reverses, 2 % does not). Known-bad: on main's tracker the
  first three fail (host harness); with the band at 0 the band test and the restored settle tests fail (the
  wobble regression). Two existing tests now prime `_lastVin` with `resetDirection` instead of a reversal from 0.
- `test_tracker_handback_overshoot_not_held`, see "Hand-back". The `steered()` helper now steps past the 3 s skip
  before the plant-exception tests.
- Unity QEMU (`test/unity_qemu.py -B build-qemu`): 305 tests, 0 failures, 25 ignored; after the follow-up
  (rebased): 308 tests, 0 failures, 25 ignored.
- esp32s3 build (IDF 6.0.3): pass, 10 % free (also after the follow-up); `rtprof_map.py --check main/rt_iram.lf /dev/null <elf>`: 79 symbol
  entries, 0 not in IRAM.
- Docs build (`website/`, private webpack cache): pass.

## Open risks

- **No hardware data.** The step and the flat P(D) come from the SPICE-fitted ring models (233 kHz). Whether a real
  board shows Vin steps at an unchanged duty near D = M is untested; the field shows dusk operation at M > 0.8 daily.
- **The field dusk loss is not this.** See "Scope" at the top.
- **The flat local maximum itself remains.** The fix keeps the tracker from being pushed onto it; a tracker that
  arrives there another way (e.g. after a limiter hand-back at high duty) is not pulled off. No such run in 684.
- **Light changes larger than 1 % of Vin** during a leg toward higher Vin still hold the reversal (clouds); in the
  cloud and shade scenarios this changed nothing, but a fast cloud edge in slow mode is a case to watch.
- **Low-light drift below ~10 W** is unchanged: the 1 W absolute threshold exceeds what a leg near the MPP changes,
  and Vin wanders (late dusk 35 V vs MPP 44 V in the lowest run). The `grew` re-sweep covers rising power only.
- Only paired against main at the default thresholds; the pending noise-derived threshold (rev_k) work changes the
  power rule and needs re-pairing with this change.
