*this document is an LLM generated placeholder*

# fry dusk field loss (2026-11-03..11-18)

Date: 2026-11-19. Field data only, read-only: no commands were sent to fry or flat, no firmware or simulator
changes. Data: InfluxDB `mppt` (1 s aggregates 13:00-18:30 UTC per evening, 48 Hz windows around 183 sweeps,
1 min means for the daily harvest) and the havan console log (`pv/fugu_console.log.10` .. `.log`, 11-06 onwards;
status lines, `Stop sweep`, `Slow mode`, `Reset maxPower`, boot version lines). 11-19 had no evening yet when this
was written; 11-03 has no usable sweep data.

## Result

- The dusk loss is real but small. In the window where 1 min power is below 25 W, fry loses **0.3-0.5 Wh per
  evening** on the clean recent evenings (11-12, 11-14..11-17) and 0.5-2.5 Wh on 11-05..11-11 (older firmware,
  highD trip loops). Tracking efficiency in that window is 0.87-0.92 (recent) and 0.6-0.85 (older). Against a daily
  harvest of 270-2800 Wh this is **0.01-0.15 % of the day** (mean 0.08 %). flat loses about as much in Wh
  (median 0.5 Wh, efficiency 0.85-0.98), 0.04 % of its day. It is not only fry, but fry is worse.
- The reviewer's ratio of 0.25-0.45 came from uncorrected sweep peaks. A dusk sweep overstates the available power by
  50-100 % (input-capacitor discharge, see `2026-11-18-fry-dusk-landing.md`). With the discharge removed, the
  operating point at 29-36 V gets **0.65-0.8** of the available power at 2-10 W, not 0.25-0.45.
- Mechanism: in deep DCM a fixed duty is a load line on which Vin falls with the light (P ~ D^2 Vin^2 (Vin-Vout)).
  At 2-10 W the P&O cannot see the P-V slope (fast mode: the capacitor term in dP is 2-3x the slope signal; slow
  mode: the threshold is below the noise), so its duty stays roughly constant (or creeps up on firmware before
  ed5ffe5e), and Vin slides from the sweep MPP at 50-60 V to 34-36 V, where fry's dusk curve has a shoulder.
- The min-pulse Vin floor (live since 11-18 15:20) is not where fry sits at 1-10 W. On 11-18 the Vin limiter
  (`cv_lim_idx` 0) was active for 3-5 samples (sweep ends) between 15:21 and 17:17 and only engaged from 17:18, when
  power fell below 1 W and the `power < 1` rule (`tracker.h`, "pump more power") drove the duty up to it. It costs
  nothing measurable (< 0.05 Wh); it only bounds the slide at ~30 V.
- Worth fixing for its Wh? No, not on its own: the recoverable part is about 0.3-0.5 Wh per evening per converter.
  A fix is only worth it if it is cheap and safe at other light levels. The larger late-afternoon losses
  (25-100 W, partial shading, up to 12-19 Wh on single days) are a different mechanism (below, "Other losses").

## Method

**Available power.** For each sweep, the 48 Hz ramp gives P(Vin). The capacitor term is removed:
P_pv = P_out + C * Vin * dVin/dt, with dVin/dt <= 0 on the ramp (100 ms bins, 300 ms smoothing). C = 1.5 mF is
the main value, 1.0 mF the sensitivity case. Check against the static P(D) pass of 11-18 (prior note):

| sweep | logged MPP | corrected, C = 1.0 / 1.5 mF | static |
|---|---|---|---|
| 11-18 16:44:56 | 6.5 W @ 48.1 V | 4.4 / 3.9 W @ 49-53 V | 3.9 W plateau 52-58 V |
| 11-18 17:12:31 | 7.5 W @ 53.1 V | 5.7 / 5.1 W @ 55 V | 4.2 W @ 52 V ~1 min later, light falling ~10 %/min; P before the sweep 4.9 W @ 52.6 V |

**Tracking efficiency.** Each sweep gives a normalised curve eta(Vin) = P(Vin)/P_mpp (2 V bins). Between two sweeps
the curve is interpolated linearly in time. Every second in MPPT with P > 0.05 W then gives available power
P/eta(Vin). Sweeps with a corrected peak below 2.5 W are not used as curves: there the ramp is too fast for the
correction, and flat's 11-16/11-17 curves put a false peak at 29-32 V. The window runs from the last minute with a
1 min mean above 25 W to the last second above 0.3 W. Off time (P <= 0.05 W: sweeps, backoffs) is booked
separately at the interpolated available power.

**Firmware epochs** (fry, from boot lines and `git merge-base`):

| epoch | evenings | tracker state |
|---|---|---|
| E0 | 11-05..11-15 | no ed5ffe5e (5 % band from the higher end, 150 ms post-reversal hold), no settle window; 11-05..11-11 also the highD-bflowOpen trip loop (fixed eecc43a, 11-12) |
| E2 | 11-16, 11-17 | ed5ffe5e + 9e1dd138 settle window (11-17 also 51b3fa85) |
| E3 | 11-18 from 15:20 | + 55087dd7/7e31e1b9 (Vin-against band), min-pulse Vin floor; a5f852f6 from 16:25. The sweep capacitor correction 1170e02c went live 18:22, after dusk. 16:41-17:17 was a manual `dc`/`sweep` session and is not field behaviour |

## Per evening

Window: 1 min power below 25 W. Lost Wh at C = 1.5 mF, in brackets 1.0 mF. "Off" is time at P <= 0.05 W inside the
window and its estimated Wh (not included in "lost").

| device | evening (UTC) | min | curves | harvested Wh | available Wh | lost Wh | tracking eff. | min at Vin < 0.75 Vmpp | off min (Wh) | day Wh | lost % of day |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fry | 11-04 16:55-17:32 | 37 | 6 | 1.73 | 2.00 | 0.27 (0.26) | 0.87 | 0 | 4 (0.44) | 2815 | 0.01 |
| fry | 11-05 16:55-18:03 | 67 | 7 | 2.76 | 3.31 | 0.54 (0.70) | 0.84 | 1 | 33 (0.45) | 1332 | 0.04 |
| fry | 11-06 16:59-17:40 | 41 | 8 | 2.42 | 3.14 | 0.71 (0.95) | 0.77 | 2 | 11 (0.15) | 1831 | 0.04 |
| fry | 11-07 16:33-17:27 | 54 | 3 | 5.45 | 6.28 | 0.83 (0.82) | 0.87 | 16 | 11 (0.23) | 603 | 0.14 |
| fry | 11-08 15:27-17:20 | 112 | 8 | 6.15 | 7.80 | 1.65 (1.92) | 0.79 | 31 | 32 (0.54) | 269 | 0.61 |
| fry | 11-09 16:47-17:28 | 41 | 8 | 5.44 | 6.44 | 1.00 (1.01) | 0.85 | 26 | 6 (0.14) | 1924 | 0.05 |
| fry | 11-10 16:08-17:20 | 72 | 10 | 4.01 | 5.83 | 1.82 (2.09) | 0.69 | 42 | 16 (0.37) | 860 | 0.21 |
| fry | 11-11 16:47-17:25 | 38 | 10 | 4.00 | 6.52 | 2.52 (3.23) | 0.61 | 32 | 5 (0.05) | 40 | (trips, OTAs) |
| fry | 11-12 16:50-17:22 | 32 | 3 | 2.27 | 2.79 | 0.52 (0.67) | 0.81 | 11 | 5 (0.06) | 1575 | 0.03 |
| fry | 11-13 16:22-17:37 | 75 | 1 | 4.22 | 6.34 | 2.12 (2.14) | 0.67 | 41 | 26 (1.15) | 1827 | low confidence: 1 curve, 8 OTAs |
| fry | 11-14 16:51-17:38 | 48 | 4 | 4.00 | 4.52 | 0.52 (0.73) | 0.88 | 37 | 4 (0.03) | 1392 | 0.04 |
| fry | 11-15 16:50-17:33 | 43 | 3 | 3.05 | 3.49 | 0.44 (0.65) | 0.87 | 11 | 4 (0.04) | 724 | 0.06 |
| fry | 11-16 16:41-17:29 | 48 | 4 | 3.05 | 3.32 | 0.28 (0.33) | 0.92 | 22 | 6 (0.06) | 1888 | 0.01 |
| fry | 11-17 16:09-17:13 | 64 | 4 | 4.33 | 4.73 | 0.40 (0.48) | 0.92 | 22 | 14 (0.13) | 270 | 0.15 |
| fry | 11-18 15:44-17:27 | 103 | 10 | 18.48 | 19.40 | 0.92 (1.02) | 0.95 | 14 | 2 (0.04) | 371 | manual session inside |
| flat | 11-04 16:55-17:46 | 51 | 5 | 4.56 | 5.00 | 0.44 (0.41) | 0.91 | 0 | 1 (0.01) | 4137 | 0.01 |
| flat | 11-05 17:05-17:41 | 36 | 6 | 4.49 | 5.18 | 0.68 (0.69) | 0.87 | 21 | 2 (0.06) | 3062 | 0.02 |
| flat | 11-06 17:01-17:43 | 42 | 6 | 5.11 | 6.02 | 0.91 (1.21) | 0.85 | 41 | 0 (0.00) | 2694 | 0.03 |
| flat | 11-07 16:47-17:35 | 48 | 5 | 5.52 | 5.95 | 0.42 (0.48) | 0.93 | 18 | 2 (0.04) | 908 | 0.05 |
| flat | 11-08 16:10-17:26 | 76 | 6 | 7.73 | 8.23 | 0.50 (0.63) | 0.94 | 33 | 2 (0.04) | 510 | 0.10 |
| flat | 11-09 16:57-17:42 | 45 | 4 | 5.89 | 6.27 | 0.38 (0.45) | 0.94 | 18 | 3 (0.05) | 3156 | 0.01 |
| flat | 11-10 16:10-17:33 | 83 | 5 | 11.96 | 12.43 | 0.48 (0.61) | 0.96 | 21 | 3 (0.05) | 1345 | 0.04 |
| flat | 11-11 16:58-17:39 | 41 | 6 | 5.15 | 5.65 | 0.50 (0.58) | 0.91 | 19 | 3 (0.05) | 108 | 0.46 |
| flat | 11-12 16:48-17:37 | 50 | 2 | 5.49 | 7.47 | 1.98 (2.10) | 0.74 | 39 | 4 (0.07) | 3315 | 0.06 |
| flat | 11-15 16:49-17:36 | 46 | 2 | 4.71 | 5.32 | 0.61 (0.73) | 0.89 | 11 | 5 (0.07) | 981 | 0.06 |
| flat | 11-16 16:43-17:32 | 49 | 3 | 5.34 | 6.25 | 0.91 (0.30) | 0.85 | 0 | 6 (0.08) | 2817 | 0.03 |
| flat | 11-17 16:12-17:22 | 69 | 3 | 5.73 | 5.85 | 0.12 (0.12) | 0.98 | 9 | 13 (0.24) | 398 | 0.03 |
| flat | 11-18 16:26-17:30 | 65 | 5 | 10.21 | 10.54 | 0.33 (0.29) | 0.97 | 3 | 4 (0.07) | 549 | 0.06 |

flat 11-13/11-14 and fry 11-03 are missing or excluded (OTA churn, no usable curves). Sensitivity: with the window at
10 W instead of 25 W, fry loses 0.3-0.5 Wh on the recent evenings (most of the loss is below 10 W); with C = 0
(the uncorrected sweeps) fry's mean loss becomes 1.4 Wh and the efficiency 0.76, which is where the reviewer's
ratios came from.

## Dusk P-V curves

Corrected sweep curves, eta = P/P_mpp at the given Vin:

| sweep | P_mpp @ V | 30 V | 34 V | 36 V | 40 V | 44 V | 48 V | 52-56 V | 60 V |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| fry 11-14 17:22 | 2.3 W @ 49 V | 0.83 | 0.91 | 0.86 | - | - | 1.00 | 0.84-0.88 | 0.75 |
| fry 11-15 17:15 | 4.0 W @ 53 V | 0.74 | 0.80 | 0.82 | 0.68 | 0.78 | 0.92 | 0.95 | 0.79 |
| fry 11-16 16:52 | 8.9 W @ 61 V | 0.71 | 0.77 | 0.77 | 0.81 | 0.88 | 0.92 | 0.92-0.95 | 1.00 |
| fry 11-17 16:33 | 6.8 W @ 58 V | 0.68 | 0.73 | 0.75 | 0.84 | 0.89 | 0.90 | 0.93-0.99 | 0.93 |
| fry 11-18 17:12 | 5.1 W @ 55 V | 0.67 | 0.72 | 0.75 | 0.75 | 0.72 | - | - | 0.86 |
| flat 11-17 16:22 | 8.7 W @ 53 V | 0.78 | 0.84 | 0.82 | 0.90 | 0.98 | 0.96 | 0.98-1.00 | 0.55 |
| flat 11-18 16:43 | 7.2 W @ 53 V | 0.79 | 0.80 | 0.83 | 0.93 | 0.86 | - | 0.91-1.00 | 0.53 |

- The curves are flat: 0.67-0.8 at 30 V, 0.75-0.9 at 34-36 V, a plateau from ~48 to 58 V. fry often has a shallow
  shoulder at 34-36 V with a dip at 38-44 V (bypass-diode signature); single 40 V values (0.63-0.68) sit at the knee
  where the correction is least reliable. The static 11-18 pass (3.2 W at 40 V, 3.9 W at 52-58 V) shows no dip.
- In the late afternoon (25-200 W) fry has a genuine low-voltage peak under partial shading: 11-12 16:48, 39.7 W at
  33 V against 13-15 W at 40-72 V; 11-14 15:52, 195 W at 32 V and 194 W at 44 V. At dusk, in diffuse light, the
  global peak moves back to 50-60 V and the 34-36 V shoulder remains.

## Mechanism

### DCM load line

In DCM the buck input power is P = K * D^2 * Vin^2 * (Vin - Vout) / Vout. The fitted K is roughly constant through
each dusk: fry 0.10-0.15, flat 0.15-0.21. The fry/flat ratio is 1.4-1.6, as expected from the coils (K ~ 1/L, 80 vs
51 uH). At a fixed duty, halving the light moves Vin from 49 V to about 41 V. To hold Vin, the duty must fall with
sqrt(P).

From the last minute above 10 W to the first below 2 W (MPPT only, 1 min medians):

| device | evening | Vin 10 W -> 2 W | duty 10 W -> 2 W | duty needed to hold the start Vin |
|---|---|---|---|---|
| fry | 11-09 (E0) | 42.2 -> 30.0 V | 1352 -> 1766 | 1266 -> 535 |
| fry | 11-10 (E0) | 42.3 -> 29.9 V | 1496 -> 1726 | 1511 -> 630 |
| fry | 11-13 (E0) | 42.9 -> 29.4 V | 1509 -> 1851 | 1375 -> 574 |
| fry | 11-14 (E0) | 40.9 -> 35.8 V | 1277 -> 754 | 1246 -> 516 |
| fry | 11-16 (E2) | 56.6 -> 35.1 V | 785 -> 675 | 753 -> 295 |
| fry | 11-17 (E2) | 56.2 -> 35.0 V | 922 -> 677 | 704 -> 281 |
| fry | 11-18 (E3) | 58.8 -> 34.3 V | 651 -> 646 | 667 -> 265 |
| flat | 11-16 (E2) | 51.7 -> 41.3 V | 690 -> 398 | 647 -> 280 |
| flat | 11-17 (E2) | 50.6 -> 42.3 V | 694 -> 347 | 621 -> 263 |
| flat | 11-18 (E3) | 50.6 -> 42.2 V | 686 -> 352 | 673 -> 295 |

- fry's duty stays put (E2, E3) or rises (E0) while it should fall by a factor of 2.4, so Vin follows the load line
  down with the light. On E0 the rise is the low-light ratchet that ed5ffe5e removed (capacitor charge after a
  reversal and the 5 % band from the leg start).
- flat's duty halves and Vin stays at 41-42 V. Why flat follows and fry does not is not established. Candidates:
  fry's flatter curve between 36 and 48 V (shoulder and dip), and a fixed-count P&O step being relatively larger at
  flat's lower duty, which gives a larger Vin swing and slope signal per leg.
- Example, fry 11-17 16:36-16:43 (E2): power falls 5.9 -> 2.4 W in 6 min while the duty moves 660 -> 680-900 and
  back to ~700. Vin falls 49 -> 35.5 V. `Reset maxPower` fires every 30-45 s (7 times), each one leaving slow mode
  for fast P&O. The tracker then settles at 35-36 V (slow mode, +-1 count) on the shoulder and stays until power is
  below 1 W. 11-18 17:13-17:16 after the manual session is the same: duty 597 -> 650, Vin 51 -> 34 V in 2 min.

### P&O resolution at 2-10 W

- fry, 11-17 16:41:36-16:42:12, slow mode at 2.4 W, duty 888-907: P sample noise sd 0.13 W (48 Hz). Slow-mode
  0.5 s means have sd 0.03 W against a threshold of 0.15 % * (P + 5) = 0.011 W, so slow-mode decisions are
  noise.
- fry, 11-17 16:37:40-16:38:40, fast mode at 4.4 W: |dVin/dt| p50 3.3 V/s, p90 5.1 V/s, so the capacitor term
  C * Vin * dVin/dt is 0.34 W at p90. The real power difference across a 5 % leg (~2.3 V) at a curve slope of about
  1.4 % of P_mpp per volt is ~0.14 W. The fast threshold is 1.5 % * (P + 5) = 0.14 W. The cap term dominates every
  comparison, and legs end on the 5 % Vin rule, so the duty does a symmetric zigzag with no net motion.
- Falling light: power fell 10-17 %/min. With the reference (`_lastPower`) only moved on threshold crossings, a
  falling P triggers reversals in both directions alike and carries no slope information.

### Ruled out or minor

- **Min-pulse floor:** not engaged at 1-10 W (above). At < 1 W the `power < 1` rule pushes the duty up until the floor
  holds Vin at 1.17 * Vout = 30.1 V (11-18 17:18-17:24, ~0.8 W, `lim` 0 in 22 % of samples). Before 11-18 the same
  rule ran the duty to 3200-3780 at Vin 28.7 V. Below 1 W the curve shape is not known; the energy at stake is
  < 0.1 Wh per evening.
- **Wrong peak captured by the sweep:** at 2-10 W the logged sweep MPP is the capacitor spike at 48-55 V. It is close
  to the true plateau in voltage but 1.5-2x too high in power. The landing then fades to the captured duty and lands
  at 0.7-0.9 of V_mpp (36-43 V; 11-14 17:22, 11-15 17:15, 11-18 16:44 and 17:12), as described in the 11-18 landing
  note. This is a second, smaller term; 1170e02c (sweep pace, discharge removed from the MPP, landing trim) went
  live after the 11-18 dusk and has no field evening yet. The low-voltage captures (`MPP=(3.2W,2937,28.7V)`, 11-14
  17:29) only happen below 1 W.
- **Dusk sweep floor/cap:** sweeps below 2 W end in `no MPP ... backing off` with 30 s backoffs; off time inside
  the window costs 0.03-0.15 Wh on the recent evenings (column "off").
- **Scan:** off (no scan lines in the log; `scan_interval` defaults to off).
- **Not the sim's D = M trap:** D/M is 0.25-0.5 here, and Vin moves with the light at a constant duty, not in steps
  at an unchanged duty and light.

### Other losses (not dusk)

With the window at 100 W instead of 25 W, fry's mean loss rises to 4.4 Wh per evening (median 1.5 Wh), driven by
three days:

- **11-10:** duty pinned at 3760 (Vin 28.6 V) at 30-90 W from 14:06 to 16:36, against an MPP of 62-88 W at 60-64 V
  (efficiency 0.46-0.53). Last seen 11-11 (16 min at duty > 3600 and P > 5 W); none from 11-12 on.
- **11-12:** before the 16:48 sweep, fry sat at 45 V and 14 W while partial shading had put the global peak at 33 V
  and 40 W.
- **11-13:** OTA churn, a single curve, low confidence.

These are shading transitions and old-firmware runaways, which the 30 min sweep only partly covers. They are bigger in
Wh than the dusk drift and deserve their own look (the peak scan exists but is off).

## Candidate fixes

Gains are per converter and evening. They assume the fix holds the plateau (eta >= 0.95) between 10 W and 1 W.

1. **Low-light re-sweep on Vin drift (simplest).** When P < ~5 % of p_max and Vin has fallen below 0.8 * V_mpp of
   the last sweep, re-sweep, at most every ~3-5 min. With the capacitor correction (1170e02c) the sweep lands on the
   plateau. A dusk sweep costs 5-20 s at 2-10 W (< 0.03 Wh). The slide takes 3-10 min, so this recovers about half:
   **+0.15-0.3 Wh** on fry, little on flat (flat seldom crosses 0.8 * V_mpp in E2/E3).
   - Sim: a fry-board dusk at pwmMax 4103 / 39 kHz MCPWM, L = 80 uH, C_in = 0.94 mF, Vbat ~26 V, 512 sps INA226
     noise. Light falling 10-17 %/min from 25 W to 1 W. A P-V curve fitted to the table above: plateau 48-58 V,
     0.75-0.9 at 34-36 V, 0.67-0.8 at 30 V. Metric: harvest 25 -> 1 W against the ideal, and the number of sweeps.
2. **Vin-referenced P&O in low-light DCM.** Below a power threshold, perturb a Vin setpoint and let the Vin
   controller move the duty, instead of perturbing the duty. When the P&O is blind (no reversal information), Vin
   then stays where the last sweep or slope put it, and the DCM load line no longer drags it down. This addresses the
   cause, also for flat. **+0.3-0.5 Wh** on fry, +0.1-0.3 Wh on flat.
   - Sim: the scenario of fix 1, plus a single-peak flat-like curve (no regression), a dawn ramp (in DCM rising light
     at a fixed duty pushes Vin up toward Voc, the mirror case), and the cloud/wobble scenarios for loop stability at
     the hand-over between duty and Vin steering.
3. **Low-light Vin floor at k * V_mpp of the last sweep.** While P < ~15 W (or < 5 % of p_max), set the Vin limiter
   target to max(min-pulse floor, k * V_mpp), k ~ 0.8, released by the next sweep. This reuses the existing Vin
   limiter (lim 0), a small change. **+0.2-0.4 Wh** on fry.
   - Risk: it must not hold Vin above a real low-voltage peak. The 25-200 W shading cases (11-12 16:48: 40 W at
     33 V) show such peaks exist, but none was seen below ~15 W.
   - Sim: the scenario of fix 1, plus a partial-shading case with the global peak at ~33 V at 20-50 W, to check that
     the power gate keeps the floor out of it.

Order by value for effort: 3, then 1, then 2. At 0.2-0.5 Wh per evening (0.02-0.05 % of a mean day, up to 0.2 % on a
dull day) none of them is urgent; the 25-100 W shading/peak losses are the larger target.

The simulator's dusk plant (`wx-dusk`) uses a single-peak PV model. It needs a curve option that reproduces the dusk
shape above (plateau plus low-voltage shoulder, bypass-diode substrings with a diffuse share), and the fry board
config with the MCPWM period, before any of these can be validated.

## Limitations

- **Available power:** it relies on the capacitor correction. C = 1.0 vs 1.5 mF changes fry's loss by ~20 %; the
  calibration rests on one static pass (11-18). Below ~2.5 W no reliable curve exists.
- **Curves between sweeps:** they are interpolated in time (one sweep per 30 min). A curve that changes shape faster,
  as with shading in the late afternoon, makes the per-evening number uncertain, more so at 25-100 W than below 25 W.
- **Epochs:** only 11-16/11-17 are clean E2 evenings. 11-18 is E3 but contains the manual session, and 11-19 (E3 +
  1170e02c) had not happened yet. The before/after comparison of the min-pulse floor and the Vin-against band rests
  on the 11-18 tail (17:13-17:27) and the absence of floor engagement at 1-10 W.
- **P measurement:** `P` is output power, so the efficiency includes converter efficiency changes along the curve
  (small at the same output power).
- **Scripts:** the analysis scripts were scratch tools (`/tmp/fdl/tools`: Influx pull, sweep analyser with capacitor
  correction, per-evening model, DCM load-line table); the method above is enough to redo them.
