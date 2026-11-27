*this document is an LLM generated placeholder*

# Partial peak scan in rising light (simulator, 2026-11-27)

Simulator only, not tested on hardware. Follows the fry 2026-11-27 08:12..08:42 UTC analysis in `README.md`
(scan gated by `pMin` = 50 W and by the stale `iEnv` bound).

## Change (`src/peak_scan.h`)

- Rising light: power above `riseK` (1.15) x a reference `pLo` (the sweep-only field, reused after the sweep).
  The reference is the sweep MPP, raised to the tracking power 5 s after the sweep, to the destination probe power
  when a scan moves, and to the start power of a rising-light scan.
- In rising light the `iEnv` prune of lower candidates is skipped and the power floor is `pMin / 5`
  (1 % of `p_max` instead of 5 %). Falling light (dusk) keeps the 5 % floor, which the 11-17 study introduced to remove
  the dusk loss.
- Probe measurements update `iEnv`, so a probed lower maximum stays a candidate after the light stops rising.
- Scan start moved into a `noinline begin()` (flash, like `armNear`): behaviour-identical (Mac runs of wx-shade-rise and
  wx-sweep-scan byte-identical before/after), IRAM .text -512 B.

Image (esp32s3, `build/scanfix`, default sdkconfig): 1754469 -> 1754377 B (-92 B); flash .text +420 B, IRAM/DIRAM
.text -512 B. Before the `begin()` split the change cost +644 B of IRAM.

### Follow-up after the EE review of f88142ab

1. The rise was consumed at the scan start (`pLo = pw` in `begin()`): a rising-light scan rejected as "light
   changed" left the 60 s re-scan with the 50 W floor and the prune, so below 50 W it never ran. Now `up` marks a
   rising-light scan; it raises `pLo` only when it ends steady with no probe within 2 margins (or has no candidate).
   A rejected or close scan keeps the reference, so the re-scan runs in rising light.
2. Settle: the probe settles when med3 Vin is within 1 % of the target. At ~20 W the duty command overshoots
   (`d sqrt(vinFast / vt)` with Vin lagging; duty 730 -> 414 counts on the way back to 72.8 V) and Vin passes the
   band on its way out; the 100 ms average then fell into the swing: Back read 18.2 W vs 21.1 W true, a false
   "light changed" (reviewer's case `SIM_RISE=0.03,0.24,0.04,60,1860`, Mac). The power settle time is not the main
   cause seen in the trace (per-tick dump, SIM_PDUMP). Fix: a probe that settled in the band and leaves 2 % travels again (`band` flag; the travel timeout path
   is unchanged, so an unreachable target still ends in a measurement). Same case, Mac: 81.4 % / 956 s -> 98.5 % /
   136 s, the Back reads 22.1 W.
3. Scan start not during a flash operation: `pMin = FLT max` while `flashOpActive()` (`src/mppt.cpp`).

Image after the follow-up: 1754749 B (base +280 B); flash .text +472 B, IRAM .text -168 B vs base.

## Scenario `wx-shade-rise`

fry6 string (6 substrings, Voc 80 V), pClear 600 W, `pv_substrings=6`, p_max 1000. Boot sweep in even dim light
(g = 0.03, ~18 W, single peak). From 60 s to 1860 s, 4 substrings ramp to 0.24, 2 stay shaded 0.03 -> 0.048. The
4-substring peak (45 V) passes the full-string peak (72-74 V, <= 32 W) at 274 s and reaches 91 W (2.9x) by 1800 s,
when the periodic sweep corrects it. Variants via `SIM_RISE=g0,gU,gS,t0,t1`, `SIM_P`.

Default (pgx93, no noise replay):

| | base (283c0248) | fix |
|---|---|---|
| PV energy / reachable GMPP | 17.72 / 30.63 Wh (57.8 %) | 30.35 Wh (99.1 %) |
| time on the wrong peak | 274..1800 s (1526 s, 12.77 Wh lost) | 274..302 s (28 s, 0.01 Wh) |
| scans | 0 | 5 (1 move at 311 s, gain x1.069) |
| energy lost while scanning | 0.025 Wh | 0.069 Wh |

## Matrix (pgx93 only, base vs fix, same host)

Rising-shade variants, each with no noise and `SIM_NOISE=fry|flat` at offsets 0/37/91 (7 runs per row): harvest delta
mean (min), wrong-peak time mean.

| variant | dHarvest pts | wrong-peak s base -> fix | scans base -> fix |
|---|---|---|---|
| def | +40.0 (+37.3) | 1526 -> 150 | 0 -> 5.3 |
| big (0.05 -> 0.4, shaded 0.07) | +44.1 (+43.9) | 1538 -> 39 | 0 -> 6.0 |
| dim (0.015 start, ~9 W) | +39.3 (+35.8) | 1380 -> 90 | 1 -> 4.4 |
| p300 (300 W string) | +39.8 (+37.7) | 1526 -> 219 | 0 -> 3.4 |
| sh06 (shaded to 0.06) | +24.3 (+22.7) | 1264 -> 168 | 1 -> 4.9 |
| slow (ramp over 3540 s) | +7.3 (+4.8) | 1319 -> 395 | 5 -> 8.9 |
| small (unshaded only to 0.12) | +10.4 (+8.9) | 1092 -> 423 | 0 -> 3.6 |
| sh04 (shaded 0.03 -> 0.04) | +44.1 (+42.5) | 1541 -> 183 | 0 -> 4.3 |
| sh03 (shaded flat at 0.03) | +0.0 | 1557 -> 1557 | 0 -> 0 |

After the follow-up (same host, same base):

| variant | dHarvest pts | wrong-peak s base -> fix |
|---|---|---|
| def | +41.0 (+40.7) | 1526 -> 42 |
| big | +44.0 | 1538 -> 39 |
| dim | +39.3 | 1380 -> 90 |
| p300 | +40.0 | 1526 -> 185 |
| sh06 | +25.1 | 1264 -> 34 |
| slow | +8.7 (+8.6) | 1319 -> 75 |
| small | +12.6 (+12.3) | 1092 -> 140 |
| sh04 | +44.0 | 1541 -> 183 |
| sh03 | -0.0 | 1557 -> 1557 |

Known limit (sh03): the detector uses the operating-point power, which the shaded substrings limit. With their
light flat since the sweep, power never rises 15 %, no rising-light scan runs, and the result equals the old rule
(48.8 % of GMPP). The fry field events had the tracked power rise 1.5-4x before each morning event, so they fall in
the covered case.

No false moves: the PROBES `wrong` count is 0 in every run, base and fix.

Regressions: all 183 default `mppt-sim.sh` scenarios, plus unshaded scenarios run with `pv_substrings=6` (fry's
config) and noise replays: same exit codes and PASS/FAIL lines everywhere (wx-calfail fails in base too;
`wx-sweep-rise` with `pv_substrings=6` fails its VERDICT in base too). Changed rows:

| cell | dHarvest pts mean (min) | extra scans |
|---|---|---|
| p6 wx-dawn (7 runs) | -0.05 (-0.15); follow-up -0.06 (-0.17) | +2.6 |
| p6 wx-low / wx-low-slow | +0.01 (-0.10; follow-up -0.13) / -0.03 (-0.08) | +1 |
| p6 wx-mid | -0.01 (-0.07) | +2 |
| p6 wx-ramp-fry / -flat | -0.03 (-0.04) | +1 |
| p6 wx-sweep-rise | -0.05 (-0.19) | +0.3 |
| wx-shade-gen lifting shade (l, l-lo) | -0.02..-0.04 (-0.06) | +0.5..1 |
| wx-sweep-learn-off | -0.05 | 0 (probes added) |
| wx-shade-learn | +0.09 | 0 (wrong-peak 102 -> 42 s) |

At these powers the point losses are a few mWh to 0.05 Wh.

Rise factor (same matrix): 1.25 gave wrong-peak 248 s (def), 348 s (big), 693 s (small) at the same regression cost;
1.40 lost most of the gain in slow and big (1223 s, 923 s). 1.15 shipped.

## Open

- Hardware: low-light probe noise on fry/flat is only covered by the replayed noise profiles (`fry`, `flat`), not by
  INA226 quantisation at 0.4-0.8 A beyond what the profiles contain.
- Morning ramps on unshaded strings: each 15 % rise runs one scan with every reachable lower candidate (up to n - 1
  probes, up to ~2.6 s of reduced power each). Bounded by sweeps (power-doubled re-sweep below 5 % of p_max, 30 min
  periodic).
- A cloud edge that makes the power jump > 15 % after the reference triggers one unbounded scan.

Driver scripts (not in the repo): `/tmp/scanfix/{gen.py,batch.sh,run1.sh,cmp.py,agg.py}`; results `/tmp/scanfix/res/`.
