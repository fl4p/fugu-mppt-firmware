*this document is an LLM generated placeholder*

# Partial peak scan and drift-compensated P&O (simulator study, 2026-11-17)

Simulator only. Nothing here ran on hardware. All paired numbers come from the host closed-loop sim
(`test/host-stub/mppt-sim.cpp`, real RT path on the `VirtualConverter` plant) run on pgx93 only; the Mac was used
for single-run sanity checks, which are not mixed into the tables.

## What was built

| Feature | Switch (`tracker.conf`) | Default | Code |
|---|---|---|---|
| Partial global-peak scan | `pv_substrings` (0 = off, fry/flat: 6), `scan_interval` [s] (300; 0 = triggered only), `scan_trigger` (1) | off | `src/peak_scan.h`, hook in `MpptController::update()` |
| Drift-compensated P&O (dP-P&O), slow mode only | `dp_po` (0/1) | off | `src/tracker.h` |

Both are inert when off: the sim output with the switches at default is byte-identical to main (checked on
wx-shade2-move, wx-cloud, wx-wob-flat, wx-dawn with fry noise replay).

RT cost: `PeakScan::update()` is `always_inline` into `MpptController::update()`; `rtprof_map.py --check` reports
79 entries, 0 not in IRAM; no outlined `PeakScan`/`Tracker::` symbols in the ELF. RAM: about 150 B in
`MpptController` (scan state) plus about 40 B in `Tracker` (two `MeanAccumulator`s, drift terms).

### Partial peak scan

Physics: with $n$ bypass-diode substrings and $m$ of them conducting, the local maximum sits near
$m/n \cdot V_{mpp}$ minus the drops of the $n-m$ conducting bypass diodes. The scan

1. estimates the substring share as $0.82\,V_{oc}/n$ from the Voc read at the start of the last sweep, picks the
   nearest multiple $m_0$ for the present Vin and rescales the share to $V_{in}/m_0$;
2. places candidates at $0.97\,m\,V_{in}/m_0$ ($m \neq m_0$), skips those below $\max(1.05\,V_{in,min}, 1.1\,V_{out})$,
   above $0.95\,V_{oc}$, and lower ones that cannot beat the present power even at the largest PV current seen since
   the sweep (this skips every candidate on an unshaded string in full sun);
3. measures the start point ($P_0$), each candidate, and the start point again ($P_0'$), each after the power
   filter settles (`powerSettleMs()`) plus a 100 ms mean; steers Vin by $D\sqrt{V_{in}/V_c}$ per tick, capped at the
   fade step;
4. compares each probe with $P_0$ linearly interpolated to the probe time, so a steady irradiance ramp cancels;
   aborts as "light changed" if $|P_0' - P_0| > 15\,\%$; moves if the best ratio exceeds 1.03.

Schedule: every `scan_interval` s since the last scan or sweep, and (with `scan_trigger`) when Vin is half a
substring share off, or power is below 70 % / above 140 % of the post-scan reference, for 15 s, with the power
steady (within ±10 %) for the last 5 s and at least 60 s since the last scan. Minimum power 5 % of `p_max`.

### dP-P&O

Reference verified: D. Sera, R. Teodorescu, J. Hantschel, M. Knoll, "Optimized maximum power point tracker for
fast-changing environmental conditions", IEEE Trans. Ind. Electron. 55(7), 2629-2637, 2008,
doi:10.1109/TIE.2008.924036 (Aalborg University repository record). The method adds a power measurement in the
middle of the MPPT period with no perturbation, so the change across that half is the irradiance alone.

Implementation: in slow mode the duty moves within the first tenth of the 500 ms period (10x speed, same mean
slew), then holds. Windows $a$ (ending at mid-period) and $b$ (ending at the period end) each start after the step
and $0.66\,t_f$. $2(b - a)$ is the drift per period; its EWMA (weight 0.6) is summed since the reference power and
subtracted from $\Delta P$.

## Harness changes (why the old shade scenarios were not realistic)

* The plant supported at most 3 substrings and no bypass-diode drop. fry and flat are two 60-cell modules in
  series: 6 substrings. `VirtualConverter::setPvString(n, g, w, vd)` now takes up to 6 substrings and a diode drop
  `vd` (0.45 V); a bypassed substring costs `vd` at the conducting ones.
* Field geometry, from the 20 most recent sweep plots of fry and flat in InfluxDB (read-only): local maxima at
  30-34, 42, 56-58 and 62-66 V on fry (Voc about 78 V) and 30-32, 36-44, 48 and 54-60 V on flat (Voc about 70 V);
  unshaded Vmpp/Voc 0.79-0.85. The 6-substring plant reproduces maxima at those fractions.
* Scenarios redefined on that plant (fry6, $V_{oc}$ 80 V): wx-shade2 (one substring at 0.70, 57 V maximum 18 %
  above the 68.8 V one), wx-shade2-move (that shade appears over 120-420 s while the tracker sits at 68.8 V),
  wx-shade3 (3 maxima, global at 57 V), wx-shade3-move (one module shaded 0.3 -> 0.72: global moves 34 V -> 68.8 V,
  +50 %), new wx-shade-walk (a 0.35 shadow walks across all substrings, 60 s ramps, 400 s holds), wx-shade-cloud
  and wx-sweep-shade on the same plant.
* `== SCAN` summary line: scans, moved, light-changed aborts, time scanning, energy lost while scanning (plant
  truth vs. the GMPP), per-scan cost; scan time is excluded from wrong-peak time.

## Method

* Cells: scenario x plant (fry, flat, fry-hq for selected cells); native wx-*-fry/-flat scenarios use their own
  plant.
* Realisations, paired across variants: measured noise replay (`SIM_NOISE=<board>`, offsets 0, 11, 23 ... s) and
  Gaussian seeds (`SIM_SEED=1..`, `noise_iout=0.02`, `noise_vout=0.03`; wx-low/-low-slow/-mid keep their own noise).
* v1: all wx-* cells, 5 + 5 realisations, variants main, scan, dp_po=1, dp_po=2, scan+dp_po=1.
* v2: contested cells, 9 + 9, trigger variants and dp_po=1 with a drift deadband.
* v3: final scan code (5 % p_max floor), 9 + 9.
* Harvest: the TRACKER line (plant PV energy / reachable GMPP energy, 2 decimals). Flag WORSE: mean delta < -0.05
  points and below -2 SE.

## Results

### Partial peak scan (v2, 18 paired runs per cell, scan vs main, harvest points)

| Cell | main | delta mean | worst..best | wrong-peak s main -> scan | scans | J/scan | scan time % |
|---|---|---|---|---|---|---|---|
| wx-shade2-move fry / flat | 90.30 / 90.32 | +7.51 / +7.51 | +7.49..+7.54 | 1491 -> 291 | 6.0 | 12 / 11 | 0.02 |
| wx-shade3-move fry / flat (v1, n=10) | 78.13 / 78.15 | +21.16 / +21.16 | +21.11..+21.22 | 1545 -> 45 | 6.4 | 29 / 26 | 0.04 |
| wx-shade-walk fry / flat | 84.19 / 84.18 | +13.71 / +13.75 | +13.36..+13.83 | 1171 -> 189 | 6.5 | 130 / 116 | 0.10 |
| wx-shade-cloud fry / flat | 90.83 / 90.86 | +4.37 / +4.36 | +3.81..+5.59 | 346 -> 98 | 8.6 | 85 / 79 | 0.26 |
| wx-sweep-shade fry / flat (v1, n=10) | 90.30 / 90.29 | +5.12 / +5.67 | +4.58..+8.30 | 586 -> 216..252 | 1.0 | 41 / 35 | 0.01 |
| wx-shade2 fry / flat (static, on the global) | 98.98 / 99.02 | -0.01 / -0.00 | -0.03..+0.02 | 0 | 1.0 | 13 / 11 | 0.01 |
| wx-shade3 fry / flat (static) | 99.02 / 99.07 | -0.03 / -0.03 | -0.07..+0.00 | 0 | 1.0 | 29 / 27 | 0.02 |
| wx-cloud fry / flat / fry-hq | 98.48 / 98.50 / 98.51 | -0.05 / -0.02 / -0.08 | -0.19..+0.13 | 0 | 4.7 | 54 | 0.07 |
| wx-cloud-cv fry / flat | 91.26 / 86.73 | -0.07 / -0.01 | -0.21..+0.13 | 0 | 4.1 / 1.6 | 52 / 58 | 0.06 / 0.02 |
| wx-dusk fry / flat (pMin 1 % p_max) | 98.24 / 98.23 | -0.29 / -0.09 | -1.40..+0.51 | 0 | 3.3 | 4 | 0.06 |
| wx-low, wx-mid, wx-sun-*, wx-wob-fry, wx-sweep-rise | | -0.01..+0.03 | | 0 | 0..0.4 | | 0.00 |

The scan recovers the global peak in every moving-shade case: wrong-peak time drops 75-97 %, harvest +4..+21
points. Cost in clear or uniformly cloudy light: -0.02..-0.08 points (wx-cloud, 4-5 scans per 900 s, about 54 J
each, flagged WORSE on fry and fry-hq) and dusk (-0.09..-0.29 with a 1 % power floor; the final 5 % floor removes it, v3).

Trigger variants (v2, same cells):

| Variant | wx-cloud fry-hq | wx-shade-walk fry | wx-shade-cloud fry | wx-dusk fry | ALL mean |
|---|---|---|---|---|---|
| period + trigger, power steady 5 s (shipped) | -0.08 | +13.71 | +4.37 | -0.29 | +1.95 |
| period + trigger, no steadiness (v1 rule) | -0.09 | +13.72 | +3.50 | -0.21 | +1.88 |
| period + trigger, power steady 15 s | -0.02 | +11.83 | +3.02 (-0.16..+5.39) | -0.24 | +1.77 |
| period only (`scan_trigger=0`) | -0.01 | +8.92 | +0.80 | -0.13 | +1.32 |

The 5 s steadiness window cuts cloud scans from 7.2 to 4.7 per 900 s and improves wx-shade-cloud; 15 s halves
the cloud cost again but costs 2 points in the shade walk and makes wx-shade-cloud unreliable. Periodic-only scans
are the cheapest in clouds but lose a third of the shade-walk gain.

### Final scan code (v3: power floor 5 % of `p_max`, 18 paired runs per cell)

| Cell | main | delta mean | worst..best | wrong-peak s main -> scan | scans | J/scan | flag |
|---|---|---|---|---|---|---|---|
| wx-shade3-move fry / flat | 78.13 / 78.14 | +21.16 / +21.17 | +21.11..+21.22 | 1545 -> 45 | 6.3 | 29 / 26 | |
| wx-shade-walk fry / flat | 84.19 / 84.18 | +13.71 / +13.75 | +13.36..+13.83 | 1171 -> 189 | 6.5 | 130 / 116 | |
| wx-shade2-move fry / flat | 90.30 / 90.32 | +7.51 / +7.51 | +7.49..+7.54 | 1491 -> 291 | 6.0 | 12 / 11 | |
| wx-sweep-shade fry / flat | 90.30 / 90.30 | +5.25 / +5.39 | +4.58..+8.30 | 586 -> 240 | 1.0 | 41 / 36 | |
| wx-shade-cloud fry / flat | 90.83 / 90.86 | +4.37 / +4.36 | +3.81..+5.59 | 346 -> 98 | 8.6 | 85 / 79 | |
| wx-cloud fry / flat / fry-hq | 98.48 / 98.50 / 98.51 | -0.05 / -0.02 / -0.08 | -0.19..+0.13 | 0 | 4.7 | 54 | WORSE (fry, fry-hq) |
| wx-dawn, wx-dusk, wx-low, wx-low-slow (all plants) | 97.06..98.64 | +0.00 | +0.00..+0.00 | 0 | 0 | | |
| wx-sun-fry/-flat, wx-wob-flat, wx-ramp-flat | 99.74..99.89 | +0.00 | -0.01..+0.02 | 0 | 0..0.2 | | |

The 5 % floor removes the dusk loss (v2: -0.09..-0.29) without touching the shade cells. Over all 486 paired
runs: mean +3.85 points, worst -0.19 (wx-cloud), 28 runs below -0.05, all of them in wx-cloud.

Energy cost per scan, plant truth (energy given up while scanning, relative to staying at the GMPP): 11-13 J with
one probe at about 200 W, 26-29 J with two, 54 J under clouds, 80-130 J with three or more probes at 300-400 W.
A scan takes 1.5-4 s; time spent scanning is 0.01-0.26 % of the run (the highest in wx-shade-cloud, where 8.6 scans
per 2400 s move 4 times).

### dP-P&O

| Variant (v1 n=10, v2 n=18) | wx-cloud | wx-cloud-cv | steady shade cells | wx-sun-fry | wx-low | ALL |
|---|---|---|---|---|---|---|
| dp_po=1 (slow mode), v1 | +0.01..+0.05 | -0.00..+0.03 | -0.04..-0.08 (WORSE in 4 cells) | -0.19 (WORSE) | -0.08..-0.14 | -0.005 |
| dp_po=1 + deadband 0.5 x threshold, v2 | +0.00..+0.04 | +0.00..+0.03 | -0.00..-0.03 | -0.08 (WORSE) | -0.09..-0.27 | -0.019 |
| fast mode too (period 33 -> 500 ms), v1 | +0.11..+0.17 | +0.09..+0.63 | -0.04..-0.09 | -0.22 | -7..-12 | -1.5 |

dP-P&O does not pay off here:

* In slow mode it is nearly inert: slow mode ends when power rises 0.5 % above the MPP record, so a rising ramp
  (the case the method targets) is handled in fast mode anyway. What remains is noise in the drift estimate
  ($2(b-a)$ from two 150-200 ms means), which costs 0.05-0.2 points in steady light.
* In fast mode the method needs a hold of several power-filter settling times ($t_f \approx 150$ ms with the INA226),
  which stretches the 33 ms period to 500 ms. It gains 0.1-0.6 points under clouds but loses 1-12 points at dawn,
  dusk, low light, wx-sun-flat and wx-wob-flat, where fast tracking speed matters. It was removed from the code.
* A deadband on the drift estimate removes part of the steady-light penalty and with it the cloud gain.

## Recommendation

* Enable the partial peak scan on strings with bypass-diode substrings: `pv_substrings=6` for two 60-cell modules
  in series (fry, flat). Keep `scan_interval=300`, `scan_trigger=1`. Expected: +4..+21 points of harvest while a
  moving shadow makes a different maximum global; -0.02..-0.08 points under broken clouds; nothing measurable in
  clear sky. Validate on hardware before enabling by default: the plant model, not the scan, is the largest
  uncertainty (below).
* Leave dP-P&O off (`dp_po=0`). With this power filter it either does nothing (slow mode) or slows fast tracking
  too much. A faster power measurement (shorter $t_f$) would be the precondition to revisit it.

## Not done / limitations

* No hardware run. Field evidence is limited to sweep-plot peak positions; no logged partial-shade event with a
  known global peak was available to replay.
* Plant approximation: conducting substrings share the voltage at the current of the weakest one; the bypass diode
  is a fixed drop. Real strings have soft diode curves and mismatch within a substring.
* The scan's lower-candidate filter uses the largest PV current since the sweep. When the light rises well above
  the sweep level, a lower maximum can be underestimated and skipped until the next sweep (the periodic sweep
  still covers it).
* Voc is the input voltage at the start of the last sweep; a Voc read in very low light (dawn) places the
  candidates slightly low. The candidates are rescaled from the operating point, which limits this.
* Moving to another maximum resets the MPP record and restarts fast mode; if the light changes during the
  travel, P&O corrects it, but a wrong move is only undone by the next scan or sweep.
