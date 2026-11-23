*this document is an LLM generated placeholder*

# Dusk-trap fix: field evaluation on fry and flat (dusks 11-10..11-22)

Date: 2026-11-23. Read-only. No command was sent to fry or flat, and no firmware was changed. Data: InfluxDB `mppt`
(1 s means 14:00-18:30 UTC per evening, 47 Hz raw for 204 sweeps that found an MPP) and the havan console log
(`pv/fugu_console.log.10` .. `.log`, 11-10 10:58 onwards: build strings, `Stop sweep`, received commands).

The fix under test is 55087dd7 + 7e31e1b9 in `src/tracker.h`. A reversal needs Vin to have moved in the steered
direction, the 5 % rule counts steered moves only, the power-drop rule gets a 1 % band, and there is a 3 s
hand-back skip (`HandBackMs`). It went live with 179ae6e9 (flat 11-18 15:18, fry 15:20) and is in every later
build (checked with `git merge-base --is-ancestor` for all 19 builds that ran since). Its own study
(`2026-11-18-dusk-tracker-trap.md`, "Scope") already says that it targets a simulator trap near D = M and **not**
the field slide at 2-10 W.

## Verdict

- **There is no measurable reduction in dusk loss from the fix.** In the trap regime (available power < 10 W),
  fry loses 0.21 / 0.35 / 0.17 Wh on the three clean post-fix dusks (11-20..11-22), against 0.28 / 0.32 Wh on
  the two clean dusks just before it (11-16, 11-17, E2). Tracking efficiency is 0.87-0.91 against 0.89-0.90.
  flat: 0.17 / 0.11 / 0.15 Wh against 0.19 / 0.08 Wh, efficiency 0.91-0.97 against 0.85-0.97. The means (fry
  0.24 vs 0.30 Wh, flat 0.14 vs 0.14 Wh) differ by less than the spread between evenings (±0.1 Wh). At
  C = 1.0 mF the result is the same.
- **The large improvement came earlier, at ed5ffe5e (11-15/16).** Before it (E0, 11-10..11-15), fry's
  operating point fell to 28.6-30 V once the light was below ~8 W. Between 10 W and 2 W it spent 12-38 min below
  40 V, and its efficiency below 10 W was 0.60-0.86. Since E2 the figure is 4-7 min (E2) and 0/13/5 min (post).
  flat went from 17-27 min to 0 in E2 and stayed at 0.
- **The fry slide is still there after the fix.** On 11-21 and 11-22 fry slides from 50-54 V to 34-36 V and
  31 V between ~5 W and 2 W, with the duty constant or rising: the DCM load-line mechanism of
  `2026-11-19-fry-dusk-field-loss.md`, which the fix does not address. 11-20 is the best fry dusk on record (Vin
  still 48 V at 2 W, 0 min below 40 V), but it did not repeat. The 16:58 sweep landed on the plateau (landing trim,
  1170e02c), so that evening cannot be attributed to the fix alone.
- **One pattern did change.** Before the fix, fry had a "stuck until the next sweep" episode on all 6 clean evenings
  (5-30 min, 1-10 W, Vin < 0.75 Vmpp; ended by a sweep). After the fix, the low-Vin episodes end at dark or
  recover without a sweep (11-21 16:23-16:31: P&O climbed from 36 V back to 52 V when the light came back). Only one
  episode, 3 min long, ended at a sweep. This fits the fix's intent (no reversals on noise), but 3 evenings and
  the sweep-phase dependence (below) make it suggestive only. In energy it is worth < 0.1 Wh per evening.
- Stakes: dusk (< 10 W) loss is 0.1-0.35 Wh per converter and evening, 0.01-0.05 % of a 650-3900 Wh day.

## Method

Same as `2026-11-19-fry-dusk-field-loss.md`, rebuilt as scratch tools (`/tmp/dte/tools`: `pull1s.py`,
`sweeps.py`, `curves.py`, `evening.py`, `loadline.py`, `presweep.py`; not kept).

- **Curves.** For each sweep, take the down-ramp (state 5, up to the Vin minimum, dVin/dt ≤ 0.5 V/s) in 100 ms
  bins and remove the capacitor term, P_pv = P_out + C·Vin·dVin/dt, with C = 1.5 mF (1.0 mF as the sensitivity
  case). Bin in 2 V steps and normalise to the peak. Curves whose corrected peak is below 2.5 W are not used. The
  correction is applied the same way to pre- and post-fix sweeps. The logged `MPP=` changed meaning at 1170e02c (the
  firmware now removes the discharge itself), so logged MPPs are not comparable across the fix and are not used.
  Check against the prior note: fry 11-17 16:33 corrected 6.7 W @ 57 V (prior 6.8 W @ 58 V), fry 11-16 16:52
  8.8 W @ 61 V (prior 8.9 W @ 61 V). The per-evening losses reproduce the prior table within 0.05-0.1 Wh (fry 11-12
  0.55 vs 0.52, 11-15 0.43 vs 0.44, 11-17 0.39 vs 0.40).
- **Available power.** For every second in MPPT with P > 0.05 W, available power is P / eta(Vin), with the curve
  interpolated in time between sweeps and held after the last one. The window runs from the last minute with a
  1 min mean above 25 W to the last second above 0.3 W. Seconds with the Vout limiter active (`cv_lim_idx` 1) are
  excluded as charger limiting. Off time (sweeps, backoffs) is booked separately.
- **Dusk band.** "lost < 10 W" sums the loss only where available power is below 10 W. This is the regime of the
  trap. Above 10 W the late-afternoon number is dominated by partial-shading curves and bad sweeps (below).
- **Curve-free check.** From 1 min medians in MPPT: Vin at the last minute with P ≥ 5 W and ≥ 2 W, and the minutes
  with Vin < 40 V between the last ≥ 10 W and the last ≥ 2 W minute.
- **Trap episodes.** At least 2 min with 1 W < P < 10 W and Vin < 0.75 × Vmpp of the last usable curve (gaps
  ≤ 20 s). Each is marked by how it ended: s = a sweep within 60 s, r = recovered, d = dark (P < 1.2 W after it).

## Firmware epochs and clean dusks

| epoch | dusks | tracker/sweep state |
|---|---|---|
| E0 | 11-10..11-15 | no ed5ffe5e (low-light ratchet), no settle window |
| E2 | 11-16, 11-17 | ed5ffe5e + 9e1dd138 (11-17 also 51b3fa85, dfde81e2) |
| fix, partial | 11-18 | 179ae6e9, a5f852f6-dirty from 16:25/16:28; manual session (fry 16:41-17:15, flat 6 manual sweeps) |
| fix, excluded | 11-19 | manual sweeps fry 16:23/16:44, fry rect-cal 16:49, flat restart 16:48, OTAs fry 16:59-17:02, flat 17:06-17:10; fry Vout-limited 11 min |
| post | 11-20 (53906f0b), 11-21 (ab6ac913-dirty), 11-22 (11042ac5) | fix + min-pulse Vin floor + a5f852f6 + 1170e02c (sweep pace, landing trim) + f8b6508e (sweep return pass); fry peak scan on (learned 11-19). 11-22 also 4d2d53b2 (no-MPP backoff doubling), 4258f880, 87306a01 |

Clean dusks (no OTA, restart or manual command in the window; no charger limiting): pre fry 11-10 (Vout limit
6 min), 11-12, 11-14, 11-15, 11-16 (Vout limit 4 min), 11-17; pre flat 11-10, 11-15, 11-16, 11-17. Post: both
devices on 11-20, 11-21 and 11-22 (11-21 OTAs at 17:43/17:54 were after dark at 17:09; 11-22 OTA and `selftest
bflow` 15:36-15:52 were 45-60 min before the window). Excluded: 11-11 and 11-13 (OTA churn), flat 11-12 (OTA
16:26-16:33, rect-cal 16:44), flat 11-14 (Vout-limited for the whole window), 11-19. The battery was not in
CV/float at dusk on any post-fix evening: the Vout limiter was never active, and Vout stayed at 25.6-27.2 V.

## Per dusk

Lost Wh at C = 1.5 mF; for "< 10 W" the 1.0 mF value is in brackets. V5/V2: Vin at the last minute with P ≥ 5 W /
≥ 2 W. "< 40 V" counts minutes between 10 W and 2 W. Episodes: count / minutes, with the end type.

| dev | dusk | clean | window | curves | harvested Wh | lost Wh (all) | lost < 10 W | eff < 10 W | V5 / V2 | < 40 V min | trap episodes |
|---|---|---|---|---:|---:|---:|---:|---:|---|---:|---|
| fry | 11-10 | E0 | 16:08-17:20 | 4 | 4.07 | 2.04 | 1.29 (1.42) | 0.60 | 36 / 29 | 38 | 4 / 34: r, s, d, d |
| fry | 11-12 | E0 | 16:50-17:22 | 2 | 2.29 | 0.55 | 0.28 (0.48) | 0.69 | 30 / 39 | 12 | 1 / 6: s |
| fry | 11-14 | E0 | 16:51-17:38 | 2 | 4.02 | 0.62 | 0.35 (0.61) | 0.84 | 33 / 29 | 24 | 1 / 30: s |
| fry | 11-15 | E0 | 16:50-17:33 | 2 | 3.25 | 0.43 | 0.38 (0.69) | 0.86 | 33 / 29 | 28 | 1 / 7: s |
| fry | 11-16 | E2 | 16:38-17:29 | 1 | 3.31 | 0.28 | 0.28 (0.35) | 0.89 | 42 / 36 | 7 | 1 / 21: s |
| fry | 11-17 | E2 | 16:09-17:13 | 2 | 4.47 | 0.39 | 0.32 (0.46) | 0.90 | 47 / 35 | 4 | 4 / 20: r, r, r, s |
| fry | 11-18 | partial | 15:44-17:27 | 9 | 14.77 | 0.44 | 0.06 (0.08) | 0.86 | 53 / 39 | 1 | 1 / 3: d |
| fry | **11-20** | post | 16:32-17:34 | 2 | 10.59 | 1.68 | **0.21 (0.22)** | **0.87** | 53 / 48 | **0** | 1 / 10: d |
| fry | **11-21** | post | 15:32-17:08 | 4 | 13.53 | 0.56 | **0.35 (0.42)** | **0.91** | 54 / 36 | **13** | 3 / 19: r, s, d |
| fry | **11-22** | post | 16:36-17:30 | 2 | 8.45 | 1.88 | **0.17 (0.18)** | **0.91** | 46 / 31 | **5** | 1 / 21: d |
| flat | 11-10 | E0 | 16:10-17:33 | 3 | 12.16 | 0.70 | 0.35 (0.39) | 0.89 | 46 / 30 | 19 | 3 / 20: s, r, d |
| flat | 11-15 | E0 | 16:49-17:36 | 2 | 5.08 | 0.76 | 0.26 (0.32) | 0.89 | 39 / 35 | 27 | 3 / 11: r, r, d |
| flat | 11-16 | E2 | 16:43-17:32 | 2 | 5.36 | 0.62 | 0.19 (0.12) | 0.85 | 48 / 44 | 0 | 0 |
| flat | 11-17 | E2 | 16:13-17:22 | 2 | 5.74 | 0.19 | 0.08 (0.09) | 0.97 | 46 / 41 | 0 | 0 |
| flat | 11-18 | partial | 16:26-17:30 | 8 | 10.22 | 0.31 | 0.14 (0.09) | 0.94 | 50 / 41 | 0 | 1 / 7: d |
| flat | **11-20** | post | 16:54-17:35 | 2 | 5.37 | 0.25 | **0.17 (0.19)** | **0.91** | 51 / 42 | **0** | 0 |
| flat | **11-21** | post | 16:08-17:09 | 3 | 7.79 | 0.16 | **0.11 (0.12)** | **0.97** | 47 / 43 | **0** | 2 / 7: r, d |
| flat | **11-22** | post | 16:49-17:34 | 2 | 6.13 | 1.65 | **0.15 (0.15)** | **0.91** | 51 / 43 | **0** | 1 / 5: d |

Contaminated dusks, for reference only: fry 11-11 lost < 10 W 0.47, 11-13 0.37, 11-19 0.16; flat 11-11 0.27, 11-12
0.49, 11-13 0.17, 11-19 0.08.

Pre-sweep point check (mean P 45-15 s before a sweep with a corrected MPP of 2.5-15 W, against that MPP). fry E0:
0.45-0.83 at Vin/Vmpp 0.50-0.61 (5 sweeps, sitting at 28-30 V). fry E2: 0.93-1.01 at 0.87-0.92 (2 sweeps). fry post:
0.86-1.08 at 0.71-0.97 (4 sweeps). flat: 0.84-1.43 in all epochs; above 1 the light is falling. No change from E2
to post.

## Where the post-fix "lost (all)" comes from

The 1.65-1.88 Wh totals on fry 11-20, fry 11-22 and flat 11-22 come from the 10-25 W band, not the dusk trap:

- **fry 11-20 16:29-16:50, ~1.3 Wh, real but a shading case.** The 16:28 sweep found the global peak at 31.9 V
  (49 W, partial shading). fry stayed there at 29-30 V and a duty of 3400-3620 (D ≈ M = 0.85, on the min-pulse Vin
  floor) while power fell from 43 to 15 W. At 16:48 the Vin limiter engaged, and P&O then climbed to 60 V in 5 min
  while P rose from 15 to 18 W under falling light, so the higher plateau had become better. The peak scan logged
  nothing in that span. This is the local-vs-global-peak problem. The same pattern occurred before the fix
  (flat 11-12 16:45-16:55 at 30-32 V, ~1.4 Wh).
- **fry 11-22 16:35-16:45, ~1.4 Wh, an artifact.** The 16:23 sweep logged `MPP=(122.6W,…,30.5V)`, was flagged
  (`landed 43 W, off by -11 %`), and its corrected curve (121 W @ 31 V) inflates the interpolated available power
  until the 16:53 sweep.
- **flat 11-22 16:45-17:05, ~1.4 Wh, uncertain.** The interpolation runs between a shading curve (16:41, 97 W @
  39 V) and the 17:11 plateau curve; flat sat at 42-48 V.

## Confounders

- **Bundled changes.** Every post-fix dusk also ran the min-pulse Vin floor (670ed04a/6e419abe), a5f852f6 (the
  floor ends the sweep), 1170e02c (sweep pace, discharge removal, landing trim with P&O held), f8b6508e (sweep return
  pass) and, on fry, the peak scan. The landing trim alone changes where the tracker starts after a dusk sweep (11-20
  16:58 landed at 60-63 V against a 62 V MPP; before 1170e02c landings were at 0.7-0.9 Vmpp). The fix cannot be
  separated from these in field data.
- **Sweep phase.** Whether an episode ends at a sweep or at dark depends on when the 30 min sweep falls relative
  to sunset (post: 16:58/17:28, 16:18/16:48, 16:53/17:23). The s-vs-d change is therefore weak evidence.
- **Weather.** There is no irradiance reference. Day yields varied from 270 to 3900 Wh, and 11-21 had a light
  recovery at 16:30. The window starts at the 25 W crossing, so cloudy evenings start earlier.
- **Sunset** moves from ~17:28 to ~17:19 UTC over the period (sun lower, more diffuse light).
- **Episode threshold.** fry's dusk plateau is flat from 48 to 67 V, so the corrected Vmpp, and with it the
  0.75 × Vmpp threshold, can move by ±7 V (fry 11-22: corrected 67 V, logged 54.7 V). Episode minutes are
  approximate; the "< 40 V" column does not depend on this.
- **Sample size.** Only 2 clean E2 dusks and 3 clean post-fix dusks.

## What could not be determined

- The effect of the fix alone, apart from the co-deployed sweep, floor and landing changes. The field has no A/B
  test; a controlled answer needs the simulator or alternate-night builds.
- Whether the simulator mechanism (Vin stepping 3-5 V within 0.3 s at an unchanged duty near D = M) occurs in the
  field at all. The one D ≈ M span seen (fry 11-20 16:29-16:48) was not analysed at 47 Hz for such steps.
- Losses below ~2 W: no reliable curve exists there (corrected sweep peaks < 2.5 W are dropped), so the last
  10-20 min of each dusk are judged only by Vin.
- Dusks after 11-22: the 11-23 dusk had not happened yet.
