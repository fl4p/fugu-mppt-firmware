*this document is an LLM generated placeholder*

# Adaptive P&O thresholds, decision period and step: host-simulator study (2026-11-17)

**Simulator only, not tested on hardware.** Every number below is from `test/host-stub/mppt-sim` runs on
**vup.fabi.me** (one host for the whole comparison; pgx93 was fully booked by another batch at the start). Mac,
pgx93 and vup results are not bit-identical, so do not mix these numbers with runs on another host.

Companion study: a sibling session measured its own B (AR(1)-corrected noise estimate, `tracker.conf` `rev_k`,
`rev_floor_w`) and C (continuous period) on the Mac. See
`doc/research/2026-11-17-tracker-noise-threshold-and-period-fade.md`. My B/C here is an independent replication with
a different estimator, on a different host. The sibling's B is also re-measured here (`Bs*` rows).

## Question

Does replacing the tracker's binary fast/slow switch (`src/tracker.h`) with a continuous, noise-adaptive scheme
improve harvest? Field data (fry/flat, 2026-11-14) puts the calm-window P&O loss at about 0.5-0.6 %, so gains of
tenths of a percent at most were expected.

## Variants

| label | what |
|---|---|
| A | baseline (main). Fast: 30 Hz, speed 0.5, θ = min(1 W, 1.5 % (P+5)). Slow: 2 Hz, speed 0.02, θ = min(0.5 W, 0.15 % (P+5)) |
| B2 / B3 | my B: θ = clamp(k·√2·σ̂, 0.05 % (P+5), min(1 W, 1.5 % (P+5))), k = 2 / 3. σ̂² from an EWMA (1/16) of the squared second difference of period means, /6, clipped at 9× the mean, normalised by T. No filter-correlation correction. Baseline mode switch kept |
| C2 / C3 | B plus a continuous decision period T 33-500 ms, speed 0.5·(30 T)^-1.19 (25× slew over the 15× period range, as A). T×1.25 per regular reversal (leg ≥ 3 periods, ≥ 10 s since a new MPP, ≥ 5 s since takeover); T÷1.5 on a 1-period leg, a 15 s one-way run, or a power trend > θ/4; T = 33 ms on a new MPP, a 3 % jump, P < 2 W, or takeover. No MPP-distance kick on slow entry |
| D2 / D3 | C plus speed × clamp(\|s̄\|/0.3, 0.5, 4), capped at 0.5, s̄ = EWMA(1/4) of (ΔP/(P+5))/(ΔV/V) per period |
| AF.33 / .2 / .1 | control: A with only the fast-mode thresholds scaled by 0.33 / 0.2 / 0.1 |
| Bs | sibling's B (patch `/tmp/nt-edit/step1-full.patch`, with the starvation fallback): θ = max(2.5·√2·σ, 0.05 W), AR(1)-corrected σ, no upper cap |
| BsD | Bs + the D step rule above |
| BsR1 / R2 / R4 | Bs with floor max(0.05 W, f·P), f = 0.01 / 0.02 / 0.04 |
| BsR2D | BsR2 + D |

All variants sit behind `FUGU_TRACKER_STUDY` (defined only by `mppt-sim.sh`) and env knobs (`TK_VARIANT`, `TK_K`,
`TK_THSCALE`, `TK_FASTSCALE`, `TK_D`, `TK_RELFLOOR`). **Baseline identity:** variant 0 of both study binaries
reproduced the unmodified build's complete output on 15 scenarios (ignoring the noise-file path line), and the
A1 re-run matched A on vup in all 256 keys, to the last digit.

## Method

- Scenarios (`mppt-sim`, fry board plant unless `-flat`): steady sun wx-sun-fry (70 W, also 200 W), wx-sun-flat;
  wobble wx-wob-fry/-flat (0.5 % at 0.2 Hz); wx-ramp-fry; wx-dawn, wx-dusk; wx-low, wx-low-slow, wx-mid (Gaussian
  sensor noise from the scenario); wx-cloud, wx-cloud-cv; cloud edge during a sweep wx-sweep-fall/-rise/-shade;
  partial shade wx-shade2, wx-shade3, wx-shade2-move, wx-shade3-move, wx-shade-cloud.
- Noise realisations: measured noise replay `SIM_NOISE=fry|flat|fry-wander|flat-wander` at 9 offsets
  (`SIM_NOISE_OFS` 0-16 s), Gaussian scenarios with 9 seeds (`SIM_SEED` 1-9), plus one noise-free run per scenario.
  Extra family on request: wx-low with flat noise at gain 0.3, offsets 0, 100, 200, 2-16 (11 runs).
- Metric: plant-truth harvest, % of the reachable GMPP energy, from the `== ADAPT` line (3 decimals). Differences
  are paired with A on the same scenario and realisation. Reversals per minute while tracking.
- Runs: 1792 (A-D), 1024 (fast controls), 1602 (Bs family), 22 (gain-0.3 family for A/AF.33), 150 (θ scale). 0 errors.
- Tools: `doc/research/adaptive-po/` (`sweep.py`, `analyse.py`, `analyse2.py`, `ident.sh`, `results/`, and
  `study-v1.patch`, which holds my A-D tracker and harness code).

## Results

Harvest change vs A in pp, mean over realisations / worst single run. Cases are `scenario noise`; n = 9 except
noise-free (1) and the gain-0.3 family (11).

| case | A % | B2 | D2 | AF.33 | AF.2 | Bs | BsD | BsR2 |
|---|--:|---|---|---|---|---|---|---|
| wx-dawn fry | 97.11 | +1.12/+0.91 | +1.12/+0.90 | +1.01/+0.81 | +1.10/+0.93 | +1.09/+0.91 | +1.06/+0.90 | +0.54/+0.25 |
| wx-dawn flat | 97.62 | +0.69/+0.62 | +0.65/+0.59 | +0.72/+0.68 | +0.73/+0.68 | +0.72/+0.70 | +0.69/+0.66 | +0.56/+0.46 |
| wx-dusk fry | 98.21 | +0.52/+0.18 | +0.65/+0.33 | +0.50/+0.01 | +0.57/+0.11 | +0.57/+0.16 | +0.61/+0.30 | +0.31/+0.12 |
| wx-dusk flat | 98.19 | +0.63/+0.38 | +0.73/+0.49 | +0.58/+0.01 | +0.68/+0.45 | +0.68/+0.44 | +0.68/+0.46 | +0.41/+0.20 |
| wx-ramp-fry fry | 99.68 | +0.21/+0.17 | +0.23/+0.20 | +0.15/+0.12 | +0.19/+0.12 | +0.20/+0.15 | +0.20/+0.16 | −0.01/−0.06 |
| wx-low gauss | 97.61 | +0.35/−1.13 | +0.41/−0.95 | +0.61/−0.83 | +0.64/−0.86 | **−1.42/−5.88** | −1.16/−4.97 | −1.34/−5.73 |
| wx-low fry | 98.27 | +0.27/−0.87 | +0.38/−0.59 | +0.23/−0.90 | +0.14/−0.82 | −0.06/−2.61 | +0.03/−2.28 | −0.28/−2.68 |
| wx-low flat-g0.3 | 98.45 | - | - | +0.27/−0.83 | - | −0.97/−3.32 | −0.83/−5.97 | −1.06/−3.30 |
| wx-low-slow gauss | 98.55 | −0.00/−1.21 | +0.04/−0.66 | +0.15/−0.07 | +0.08/−0.37 | +0.07/−0.70 | +0.16/−0.18 | +0.07/−0.23 |
| wx-mid gauss | 99.60 | −0.02/−0.06 | −0.01/−0.06 | +0.04/−0.03 | +0.01/−0.13 | −0.02/−0.05 | −0.01/−0.09 | −0.12/−0.33 |
| wx-sun-fry fry | 99.86 | +0.07/+0.04 | +0.06/+0.03 | +0.03/−0.02 | +0.05/+0.01 | +0.04/−0.02 | +0.04/+0.01 | −0.20/−0.30 |
| wx-sun-fry 200 W | 99.91 | +0.01/−0.02 | +0.01/+0.00 | +0.02/−0.00 | +0.03/+0.00 | −0.01/−0.03 | −0.00/−0.02 | −0.27/−0.50 |
| wx-sun-flat flat | 99.82 | +0.13/+0.02 | +0.12/+0.04 | +0.04/−0.06 | +0.08/+0.01 | +0.14/+0.06 | +0.11/+0.02 | −0.12/−0.32 |
| wx-wob-fry fry | 99.74 | +0.06/−0.14 | +0.13/−0.01 | −0.10/−0.75 | +0.03/−0.26 | −0.35/−0.94 | −0.43/−0.55 | −0.15/−0.90 |
| wx-wob-fry fry-wander | 99.60 | +0.16/−0.13 | +0.26/+0.09 | +0.06/−0.27 | +0.06/−0.16 | −0.47/−1.22 | −0.33/−0.68 | +0.09/−0.18 |
| wx-wob-flat flat | 99.80 | −0.04/−0.27 | +0.06/−0.08 | −0.28/−1.13 | −0.22/−0.50 | +0.00/−0.19 | +0.00/−0.12 | −0.11/−0.74 |
| wx-wob-flat flat-wander | 99.87 | −0.08/−0.38 | −0.01/−0.08 | −0.42/−0.83 | −0.30/−0.74 | −0.13/−0.30 | −0.08/−0.19 | −0.16/−0.45 |
| wx-cloud fry | 98.52 | −0.03/−0.16 | +0.01/−0.08 | −0.02/−0.17 | −0.03/−0.21 | −0.06/−0.25 | −0.10/−0.25 | −0.12/−0.23 |
| wx-cloud-cv fry | 92.36 | +0.03/−0.11 | −0.08/−0.38 | −0.11/−0.23 | −0.15/−0.26 | +0.07/−0.01 | −0.01/−0.18 | +0.16/+0.00 |
| wx-sweep-rise fry | 96.42 | −0.10/−0.43 | −0.12/−0.27 | **−2.88/−3.59** | **−4.42/−4.96** | +0.19/+0.04 | +0.12/−0.01 | −1.89/−2.23 |
| wx-sweep-shade fry | 90.15 | −0.06/−0.11 | −0.03/−0.05 | −0.20/−0.56 | −0.44/−1.53 | −0.05/−0.08 | −0.03/−0.05 | −0.62/−0.97 |
| wx-sweep-fall fry | 90.19 | +0.02/−0.08 | +0.05/+0.00 | +0.04/−0.04 | +0.04/−0.00 | +0.02/−0.13 | +0.04/−0.02 | −0.30/−0.70 |
| wx-shade2 fry | 99.00 | +0.02/−0.03 | +0.04/+0.02 | +0.05/+0.01 | +0.04/+0.00 | −0.00/−0.06 | +0.03/−0.00 | −0.38/−1.22 |
| wx-shade3 fry | 98.96 | +0.03/−0.02 | +0.04/+0.02 | +0.04/+0.00 | +0.05/+0.01 | +0.02/−0.02 | +0.03/+0.00 | −0.27/−0.49 |
| wx-shade2-move fry | 90.02 | +0.02/−0.01 | +0.04/+0.02 | +0.04/−0.00 | +0.03/+0.01 | +0.03/+0.00 | +0.03/+0.01 | −0.19/−0.33 |
| wx-shade3-move fry | 74.17 | +0.00/−0.02 | +0.04/+0.03 | +0.04/+0.02 | +0.04/+0.01 | +0.03/+0.00 | +0.03/+0.02 | −0.14/−0.21 |
| wx-shade-cloud fry | 93.28 | +0.05/+0.01 | +0.03/−0.03 | +0.01/−0.05 | +0.02/−0.05 | −0.00/−0.09 | −0.02/−0.12 | −0.28/−0.51 |

Mean of all 41 case means (pp): B2 +0.16, B3 +0.08, C2 +0.17, C3 +0.12, D2 +0.18, D3 +0.16, AF.33 +0.06, AF.2 +0.03,
AF.1 +0.00, Bs +0.07, BsD +0.07, BsR1 −0.09, BsR2 −0.25, BsR4 −1.11, BsR2D −0.18. No variant added protection trips.
Wrong-peak time is unchanged on all shade cases, so global-peak handling does not regress for B, C, D or Bs. The
BsR* rows lose 0.1-0.4 pp on shade, and BsR4 loses up to 12 pp on wx-sweep-rise. Full per-case table with C2, AF.1,
BsR1, BsR4 and BsR2D: `adaptive-po/results/analysis2.txt`.

Worst single runs per variant:

| variant | worst runs (pp vs A) |
|---|---|
| B2 | wx-low-slow seed2 −1.21; wx-low seed7 −1.13; wx-low fry@16 −0.87 |
| C2 | wx-low seed6 −1.28; wx-low seed2 −0.96 |
| D2 | wx-low seed8 −0.95; wx-low seed7 −0.69 |
| AF.33 / AF.2 / AF.1 | wx-sweep-rise fry@8 −3.59 / fry@10 −4.96 / fry@4 −6.23 |
| Bs | wx-low seed6 −5.88; wx-low seed8 −5.60; wx-low flat-g0.3@6 −3.32; @0 −3.03; @200 −2.33; @100 −0.46 |
| BsD | wx-low flat-g0.3@14 −5.97; wx-low seed3 −4.97 |
| BsR1 | wx-dusk fry@12 −14.18 (stuck far off the MPP, tracking loss 15 %); wx-low seed6 −5.88 |
| BsR4 | wx-sweep-rise fry@4 −12.16 (all 9 realisations about −12) |

### Is B's gain adaptivity or just a smaller fast threshold?

It is the smaller fast threshold. At low power A's fast threshold is 1.5 % of (P+5), which is 5 % of P at 2 W. dawn
and dusk spend about 60 % of their time in fast mode. Scaling only that fixed threshold (AF.2) gives the same gain
as B: dawn/dusk group mean +0.77 pp vs B2 +0.74, Bs +0.77, D2 +0.79.

A fixed small fast threshold is not safe, though. On a cloud edge during a sweep, wx-sweep-rise loses 2.9 / 4.4 /
5.8 pp at AF.33 / .2 / .1 (every realisation; reversals 169-240/min against A's 61). It also costs up to 1.1 pp on
the flat wobble. B2's estimate rises with the transient and returns θ to the baseline cap, so B2 keeps the
dawn/dusk gain without those losses. In short, the smaller threshold supplies the gain and the noise estimate
makes it safe, provided θ stays capped at the baseline value (next point).

### Sibling B on vup: the tail is θ too large, not too small

Bs reproduces the dawn/dusk gain (+0.57..+1.09 pp) but has a large wx-low tail on vup: Gaussian mean −1.42 pp, worst
−5.88 (seed 6), and the gain-0.3 family −0.97 mean, worst −3.32 @6.

In the worst runs the fast-mode θ is too **high**: Bs wx-low seed 6 averages θ = 11.1 % of P in fast mode, against
3.1 % for A and 3.2 % for B2. With Gaussian noise of a few % of P per fast period, k·√2·σ̂ exceeds the baseline's
fast threshold, Bs has no upper cap, and the tracker cannot follow the rising light (tracking loss 7.1 % against
1.0 % for A). In flat-g0.3@6, Bs averages 4.9 % against AF.33's 1.0 %.

So on vup the tail comes from a missing cap, not from the floor. B2's cap at the baseline threshold,
min(1 W, 1.5 % (P+5)), holds the same family to a worst of −1.2 pp. This complements the sibling's finding that the
0.05 W floor lets θ chase the input-cap lag. The family is chaotic either way: A alone spans 97.1-99.0 % over the
11 gain-0.3 offsets.

### Relative floor (max(0.05 W, f·P))

It does not help. It loses everywhere except dawn/dusk, and loses more as f grows:

- f = 0.02: −0.2..−0.3 pp in steady sun and −0.1..−0.4 pp on shade.
- On wx-sweep-rise: −0.4 / −1.9 / −11.9 pp at f = 0.01 / 0.02 / 0.04.
- It does not fix the wx-low tail (BsR2 wx-low gauss −1.34 mean).

The floor needed for the gain-0.3 case is far above what steady sun tolerates. One floor does not fit both.

### D (slope-scaled step)

D adds nothing measurable: D2 − B2 = +0.02 pp, BsD − Bs = +0.00 pp mean of cases, inside the realisation spread.
It moves single-run tails both ways (BsD wx-low flat-g0.3@14 −5.97 vs Bs −0.77; BsD wx-cloud noise-free −0.50).

### C (continuous decision period)

My C does not lose overall (C2 +0.17 pp vs B2 +0.16), but it adds nothing over B, and it took three design rounds
to get there:

1. With growth on every reversal, C collapsed at dusk (85-87 % harvest). A steady irradiance decline of about
   0.17 % per period is comparable to θ in slow mode. The tracker reversed every 1-3 periods, lengthened T on each
   reversal, slowed 25×, and parked at a fixed duty while Vin fell 48 → 29 V. A leaves this state because its MPP
   record reset (85 % after 30 s) drops it to fast mode every ~75 s.
2. With a one-way limit counted in periods, C never lengthened T in steady sun, because fast legs are 30+ periods.
3. The final rules make a new MPP reset T, require 10 s without one before growing, and add a trend guard. With
   these, C behaves like a smoothed version of A's switch. It reverses 2-4× more often on the wobble (89/min vs 37).

The sibling rejects C at −0.25 pp. I agree that C is not worth its code. Our signs differ because our rules differ.

### Reversal rates

B2/C2/D2 reverse 2-3× more often than A at dawn and dusk (55-90/min vs 23-31). Bs, with k = 2.5 and the AR(1)
correction, stays at A's rate (24-29/min) with the same harvest. Its fast-mode θ at dusk is 2.1× B2's (1.12 % vs
0.52 % of P). The sibling's explanation accounts for about 1.5× of that: the AR(1) factor raises σ about 1.2× at
450 Hz (2(1−ρ)(3−ρ) ≈ 4.2 instead of 6 for ρ ≈ 0.24) and k is 1.25× higher. The remainder is estimator detail (my
clipping at 3 rms, EWMA length 16 vs 32). The extra reversals cost no harvest in this simulator. On hardware they
mean more duty steps and more input-cap current.

## Why the sibling's θ is large in low light, and capped vs uncapped B

Second sweep on vup, 580 runs, 0 errors. All variants run on the sibling's B (patch `step1-full.patch`). Each changes
one thing:

| label | change |
|---|---|
| BsN | sibling's B unchanged |
| BsCap | θ = min(estimate, A's fixed threshold for the mode) |
| BsH4 | Huber clamp at 4× the running mean instead of 9× |
| BsS64 | estimator EWMA span 64 instead of 32 |

The diagnosis binary reproduces the earlier Bs runs exactly (145 of 145 harvests equal). A is the baseline from the
first sweep, same keys.

Scenarios:

- wx-low: flat noise at gain 0.3 (11 offsets), Gaussian (9 seeds), fry noise (9 offsets).
- wx-dawn and wx-dusk: noise-free, fry and flat noise (9 offsets each).
- wx-sweep-rise and the five shade scenarios: noise-free and fry noise.
- wx-wob-flat and wx-wob-fry with measured noise.

The `== NOISE` line counts, per mode:

- how many noise samples reach the estimator, per period and per second;
- which share of them are the first two after a (p1, p2) reset, i.e. right after a reversal or a settle window;
- the rms of those early samples against the later ones.

### Diagnosis: the noise is real, not an estimator artefact

Uncapped B (BsN), per-case means over runs:

| case | fixed-rule share fast / slow | samples/s fast / slow | samples per period fast / slow | early share fast | rms early/late fast / slow |
|---|---|---|---|---|---|
| wx-low gauss | 0.1 % / 9.4 % | 24.9 / 1.76 | 0.83 / 0.88 | 5.9 % | 1.01 / 0.95 |
| wx-low flat-g0.3 | 0.1 % / 10.2 % | 23.8 / 1.60 | 0.79 / 0.80 | 7.1 % | 1.00 / 0.89 |
| wx-low fry | 0.1 % / 13.9 % | 19.0 / 1.48 | 0.63 / 0.74 | 12.5 % | 0.99 / 0.76 |

- **No starvation.** The estimator gets a sample in about 80 % of fast periods (19-25 per second), and the
  fixed-rule fallback is active in only 0.1 % of fast periods.
- **No transients.** The samples that arrive right after a reversal or settle window have the same rms as the
  rest (ratio 0.99-1.01 in fast mode), so input-cap transients do not inflate the estimate.
- **The estimate matches the measured noise.** The uncapped θ equals 1.8-2.0 × √2 × the period-mean noise that A
  measures on the same run (consecutive-period differences/√2, including stepping; `adaptive-po/keff.py`):

| wx-low | σ fast, % of P (A run) | A θ fast | BsN θ fast | BsN θ / (√2 σ) | σ slow | A θ slow | BsN θ slow |
|---|--:|--:|--:|--:|--:|--:|--:|
| gauss | 3.48 | 3.11 | 9.83 | 2.00 | 1.03 | 0.33 | 2.35 |
| flat-g0.3 | 1.68 | 3.08 | 4.40 | 1.85 | 0.67 | 0.35 | 2.18 |
| fry | 1.49 | 3.08 | 3.70 | 1.76 | 1.19 | 0.38 | 2.69 |

So the large θ is the k√2σ rule doing what it is designed to do, on noise of several % of P per fast period. The
baseline runs far below that rule in low light: k ≈ 0.6 in fast mode and about 0.2-0.3 in slow mode. It wins
there, consistent with the θ-scale result below. Once θ falls below about σ the loss stops falling and settles near
1σ, while θ = 2.5√2σ costs about θ/3 ≈ 1.2σ and, in rising light, longer legs before the tracker follows. Low light
wants a threshold below the noise. k√2σ gives the opposite. The Huber factor and the EWMA span change the
estimate only by a few percent (fast θ 9.4 % and 9.7 % against 9.8 % on Gaussian wx-low), so they cannot fix this.

### Capped vs uncapped

Harvest vs A, pp: mean / worst run (run name).

| case | n | A % | BsN (no cap) | BsCap | BsH4 | BsS64 |
|---|--:|--:|---|---|---|---|
| wx-low gauss | 9 | 97.61 | −1.42 / −5.88 (seed6) | **+0.19 / −0.44 (seed9)** | −2.67 / −14.24 (seed9) | −1.15 / −4.92 (seed1) |
| wx-low flat-g0.3 | 11 | 98.45 | −0.97 / −3.32 (@6) | **−0.19 / −1.28 (@10)** | −0.10 / −1.05 (@16) | −0.94 / −5.84 (@200) |
| wx-low fry | 9 | 98.27 | −0.06 / −2.61 (fry@12) | −0.07 / −1.14 (fry@16) | +0.22 / −0.78 (fry@6) | +0.06 / −1.14 (fry@8) |
| wx-dawn fry | 9 | 97.11 | +1.09 / +0.91 | +1.09 / +0.91 | +1.09 / +0.93 | +1.08 / +0.92 |
| wx-dawn flat | 9 | 97.62 | +0.72 / +0.70 | +0.72 / +0.70 | +0.71 / +0.67 | +0.71 / +0.65 |
| wx-dusk fry | 9 | 98.21 | +0.57 / +0.16 (fry@14) | +0.58 / +0.24 (fry@14) | +0.57 / +0.22 | +0.61 / +0.27 |
| wx-dusk flat | 9 | 98.19 | +0.68 / +0.44 (flat@0) | +0.68 / +0.42 (flat@0) | +0.69 / +0.41 | +0.67 / +0.45 |
| wx-sweep-rise fry | 9 | 96.42 | +0.19 / +0.04 | −0.03 / −0.36 (fry@8) | +0.15 / −0.05 | +0.13 / −0.05 |
| wx-wob-fry fry | 9 | 99.74 | −0.35 / −0.94 (fry@10) | −0.02 / −0.52 (fry@14) | −0.28 / −1.02 | −0.50 / −1.29 |
| wx-wob-flat flat | 9 | 99.80 | +0.00 / −0.19 (flat@8) | −0.08 / −0.31 (flat@0) | −0.06 / −0.27 | −0.04 / −0.45 |
| shade (5 scenarios, fry) | 45 | | −0.09 worst | −0.06 worst | −0.03 worst | −0.09 worst |

Noise-free dawn, dusk, sweep-rise and shade runs are equal for all four variants (+0.07..+0.16 pp). Group summary
(mean / worst run): wx-low BsN −0.83 / −5.88, BsCap −0.04 / −1.28, BsH4 −0.80 / −14.24, BsS64 −0.70 / −5.84;
dawn/dusk +0.76..+0.77 for all.

θ/P in % (mean [min..max] over runs), fast | slow:

| case | A | BsN | BsCap | BsH4 | BsS64 |
|---|---|---|---|---|---|
| wx-low gauss | 3.11 [3.01..3.29] \| 0.33 | 9.83 [9.18..11.14] \| 2.35 | 3.13 [3.03..3.32] \| 0.34 | 9.37 [8.51..11.55] \| 2.43 | 9.65 [8.59..11.26] \| 2.90 |
| wx-low flat-g0.3 | 3.08 [3.02..3.13] \| 0.35 | 4.40 [3.92..4.90] \| 2.18 | 2.88 [2.81..2.94] \| 0.34 | 4.03 [3.71..4.31] \| 1.61 | 4.46 [4.14..5.48] \| 2.09 |
| wx-low fry | 3.08 [2.99..3.24] \| 0.38 | 3.70 [3.56..3.87] \| 2.69 | 2.79 [2.68..2.93] \| 0.39 | 3.46 [3.29..3.61] \| 3.25 | 3.81 [3.67..3.98] \| 2.44 |
| wx-dawn fry | 2.84 \| - | 0.90 \| - | 0.90 \| - | 0.89 \| - | 0.90 \| - |
| wx-dusk fry | 4.49 \| 0.26 | 1.12 \| 0.76 | 1.13 \| 0.26 | 1.12 \| 0.76 | 1.12 \| 0.76 |
| wx-sweep-rise fry | 0.14 \| 0.07 | 0.27 \| 0.15 | 0.14 \| 0.06 | 0.25 \| 0.12 | 0.27 \| 0.13 |
| wx-wob-fry fry | 1.45 \| 0.16 | 0.28 \| 0.27 | 0.28 \| 0.16 | 0.26 \| 0.26 | 0.28 \| 0.27 |
| wx-shade2 fry | 0.64 \| 0.15 | 0.28 \| 0.17 | 0.28 \| 0.15 | 0.26 \| 0.15 | 0.28 \| 0.17 |

Per-run values for every case, including θ/P per run, are in `adaptive-po/results/diag/per-run.tsv` and
`analysis3.txt`.

Findings:

- **The cap is needed.** Neither the Huber 4× (b) nor the span 64 (c) removes the tail. Huber 4× is worse: Gaussian
  wx-low −2.67 pp mean, one run −14.24 pp (seed 9). Span 64 leaves −5.84 (flat-g0.3@200) and −4.92 (seed 1).
- **The cap removes the tail** (worst wx-low run −1.28 pp against −5.88) and keeps the whole dawn/dusk gain
  (+0.76 pp group mean). It costs a little on wx-sweep-rise (−0.03 mean, −0.36 worst) and recovers the wx-wob-fry
  loss (−0.02 against −0.35).
- **Where the cap binds.** In slow mode the estimate is above A's fixed slow threshold in every case here, so with
  the cap slow mode behaves like A (θ/P 0.26 % at dusk, as A). In fast mode the cap only binds in low light, where
  it brings θ back to A's (~3 % of P in wx-low). Everywhere else the estimate is below A's fast threshold, and that
  is where the gain comes from.
- **Capped B equals AF plus a noise guard.** BsCap's dusk θ/P (fast 1.13 %, slow 0.26 %) is close to A with a
  smaller fast threshold. The estimate only decides when the fast threshold returns to A's. Compare the AF controls
  above, which lost 3-6 pp on wx-sweep-rise without that.
- **Side effects of the cap.** It roughly halves the sample rate (0.41 vs 0.83 per fast period on Gaussian wx-low)
  and pushes the early-sample share to 26 % fast and 81 % slow, because the lower θ reverses more often. The early
  samples still show no excess variance (rms ratio 0.95-1.02).

## A lower k without a cap

Can a smaller `rev_k` replace the cap? Third sweep on vup: 978 runs on the sibling's estimator plus 18 baseline runs,
0 errors.

Variants: uncapped k = 0.7, 1.0, 1.4 and 2.5 (the default); capped k = 1.0 and 2.5. The uncapped-2.5 and capped-2.5
runs are the diagnosis-sweep runs, reused unchanged.

Scenarios: the diagnosis set plus a high-noise cloud case, wx-cloud with flat noise at gain 3, and gain 1 as a
reference (9 offsets each). The baseline collapses at gain 3 here as in the sibling's Mac sweep: 96.91 % against
98.55 % at gain 1, with 186 reversals/min against 106.

Harvest vs A, pp: mean / worst run (run).

| case | n | A % | k 0.7 | k 1.0 | k 1.4 | k 2.5 | cap k 1.0 | cap k 2.5 |
|---|--:|--:|---|---|---|---|---|---|
| wx-cloud flat-g3 | 9 | 96.91 | −0.00 / −0.30 (@4) | +0.30 / −0.04 (@14) | +0.61 / +0.33 (@16) | **+0.91 / +0.74** (@10) | −0.05 / −0.66 (@0) | −0.10 / −0.57 (@16) |
| wx-cloud flat-g1 | 9 | 98.55 | +0.01 / −0.07 | +0.07 / −0.09 | +0.08 / −0.02 | +0.02 / −0.16 | +0.07 / −0.05 | +0.03 / −0.11 |
| wx-low gauss | 9 | 97.61 | −0.07 / −1.57 (seed9) | +0.32 / −1.67 (seed7) | −0.03 / −1.82 (seed2) | −1.42 / −5.88 (seed6) | +0.36 / −0.51 (seed7) | +0.19 / −0.44 (seed9) |
| wx-low flat-g0.3 | 11 | 98.45 | +0.31 / −0.47 (@14) | +0.18 / −1.01 (@2) | +0.37 / −0.54 (@2) | −0.97 / −3.32 (@6) | +0.28 / −0.29 (@2) | −0.19 / −1.28 (@10) |
| wx-low fry | 9 | 98.27 | +0.34 / −0.50 (fry@4) | +0.42 / −0.71 (fry@8) | +0.35 / −1.10 (fry@8) | −0.06 / −2.61 (fry@12) | +0.39 / −0.55 (fry@8) | −0.07 / −1.14 (fry@16) |
| wx-dawn fry | 9 | 97.11 | +1.13 / +0.96 | +1.12 / +0.95 | +1.12 / +0.93 | +1.09 / +0.91 | +1.12 / +0.95 | +1.09 / +0.91 |
| wx-dawn flat | 9 | 97.62 | +0.74 / +0.69 | +0.74 / +0.71 | +0.74 / +0.71 | +0.72 / +0.70 | +0.74 / +0.71 | +0.72 / +0.70 |
| wx-dusk fry | 9 | 98.21 | +0.53 / +0.20 | +0.52 / +0.20 | +0.55 / +0.23 | +0.57 / +0.16 | +0.49 / +0.19 | +0.58 / +0.24 |
| wx-dusk flat | 9 | 98.19 | +0.68 / +0.49 | +0.68 / +0.49 | +0.66 / +0.42 | +0.68 / +0.44 | +0.66 / +0.46 | +0.68 / +0.42 |
| wx-sweep-rise fry | 9 | 96.42 | **−1.81 / −2.34** (fry@16) | −0.87 / −1.29 (fry@12) | −0.09 / −0.37 (fry@8) | +0.19 / +0.04 | −0.84 / −1.11 (fry@12) | −0.03 / −0.36 (fry@8) |
| wx-wob-fry fry | 9 | 99.74 | −0.02 / −0.42 | −0.14 / −0.66 | +0.13 / −0.06 | −0.35 / −0.94 | −0.14 / −0.66 | −0.02 / −0.52 |
| wx-wob-flat flat | 9 | 99.80 | −0.03 / −0.45 | −0.10 / −0.34 | +0.04 / −0.20 | +0.00 / −0.19 | −0.10 / −0.34 | −0.08 / −0.31 |
| shade (5, fry), worst run | 45 | | −0.09 | −0.04 | −0.02 | −0.09 | −0.04 | −0.06 |

Group means / worst run (pp): low light (29 runs) k 0.7 +0.20 / −1.57, k 1.0 +0.30 / −1.67, k 1.4 +0.24 / −1.82,
k 2.5 −0.83 / −5.88, cap k 1.0 +0.34 / −0.55, cap k 2.5 −0.04 / −1.28. Dawn/dusk is +0.75..+0.77 for all.
Noise-free runs are equal across variants.

Reversals/min (mean over runs):

| case | A | k 0.7 | k 1.0 | k 1.4 | k 2.5 | cap 1.0 | cap 2.5 |
|---|--:|--:|--:|--:|--:|--:|--:|
| wx-cloud flat-g3 | 186 | 279 | 243 | 189 | 88 | 265 | 194 |
| wx-cloud flat-g1 | 106 | 243 | 217 | 181 | 114 | 221 | 131 |
| wx-low gauss | 142 | 225 | 193 | 136 | 39 | 197 | 142 |
| wx-low flat-g0.3 | 71 | 222 | 197 | 152 | 48 | 198 | 79 |
| wx-dusk fry | 26 | 32 | 32 | 31 | 27 | 38 | 33 |
| wx-sweep-rise fry | 61 | 134 | 99 | 57 | 23 | 97 | 60 |
| wx-shade2 fry | 8 | 59 | 47 | 33 | 13 | 47 | 13 |
| wx-wob-fry fry | 36 | 65 | 60 | 50 | 29 | 60 | 41 |

θ/P in %, fast | slow (mean over runs):

| case | A | k 0.7 | k 1.0 | k 1.4 | k 2.5 | cap 1.0 | cap 2.5 |
|---|---|---|---|---|---|---|---|
| wx-cloud flat-g3 | 0.44 \| 0.10 | 0.17 \| 0.29 | 0.24 \| 0.36 | 0.34 \| 0.47 | 0.67 \| 0.77 | 0.20 \| 0.10 | 0.35 \| 0.10 |
| wx-cloud flat-g1 | 0.49 \| 0.08 | 0.08 \| 0.11 | 0.10 \| 0.16 | 0.13 \| 0.20 | 0.24 \| 0.34 | 0.10 \| 0.06 | 0.21 \| 0.08 |
| wx-low gauss | 3.11 \| 0.33 | 2.24 \| 1.13 | 3.11 \| 1.13 | 4.64 \| 1.25 | 9.83 \| 2.35 | 2.62 \| 0.35 | 3.13 \| 0.34 |
| wx-low flat-g0.3 | 3.08 \| 0.35 | 1.18 \| 1.03 | 1.53 \| 1.18 | 2.17 \| 1.43 | 4.40 \| 2.18 | 1.53 \| 0.32 | 2.88 \| 0.34 |
| wx-low fry | 3.08 \| 0.38 | 1.15 \| 1.02 | 1.46 \| 1.49 | 1.98 \| 1.80 | 3.70 \| 2.69 | 1.45 \| 0.39 | 2.79 \| 0.39 |
| wx-dusk fry | 4.49 \| 0.26 | 1.10 \| 0.75 | 1.10 \| 0.75 | 1.10 \| 0.75 | 1.12 \| 0.76 | 1.10 \| 0.26 | 1.13 \| 0.26 |
| wx-sweep-rise fry | 0.14 \| 0.07 | 0.07 \| 0.03 | 0.10 \| 0.04 | 0.14 \| 0.07 | 0.27 \| 0.15 | 0.10 \| 0.04 | 0.14 \| 0.06 |
| wx-wob-fry fry | 1.45 \| 0.16 | 0.08 \| 0.08 | 0.11 \| 0.11 | 0.15 \| 0.16 | 0.28 \| 0.27 | 0.11 \| 0.11 | 0.28 \| 0.16 |
| wx-shade2 fry | 0.64 \| 0.15 | 0.08 \| 0.04 | 0.11 \| 0.06 | 0.15 \| 0.08 | 0.28 \| 0.17 | 0.11 \| 0.06 | 0.28 \| 0.15 |

Per-run values: `adaptive-po/results/diag/per-run-kscan.tsv`, `analysis4.txt`.

Findings:

- **The cap trades the high-noise cloud gain for the low-light tail.** On wx-cloud flat-g3, uncapped k 2.5 recovers
  +0.91 pp (all 9 runs ≥ +0.74), because its fast θ (0.67 % of P) rises above A's (0.44 %). Capped k 2.5 is held at or below
  A's θ (mean 0.35 %) and loses the gain (−0.10, worst −0.57).
- **The cloud gain scales with k:** +0.91 / +0.61 / +0.30 / 0.00 pp at k 2.5 / 1.4 / 1.0 / 0.7.
- **The low-light tail shrinks with k:** worst −5.88 at k 2.5, −1.82 at 1.4, −1.67 at 1.0, −1.57 at 0.7.
- **k below about 1.4 fails at a cloud edge in a sweep:** wx-sweep-rise −0.87 at k 1.0 and −1.81 at k 0.7 (worst
  −2.34), the same failure as the fixed-threshold controls. A cap does not help there (cap k 1.0 −0.84).
- **No single uncapped k matches capped 2.5 in low light and keeps the full cloud gain.** k 1.4 is the closest
  compromise:
  - low light: mean +0.24 vs −0.04 for capped 2.5, worst −1.82 (wx-low seed 2) vs −1.28;
  - cloud gain: +0.61 of the +0.91;
  - wx-sweep-rise: same as capped 2.5 (−0.09 / −0.37);
  - wobble: best of all variants (+0.13 / +0.04);
  - shade: unchanged.
- **Cost of k 1.4:** 2-4× A's reversal rate in steady and shaded fry cases (shade2 33/min vs 8) and 2× in
  wx-low flat-g0.3 (152 vs 71). That costs no harvest in this simulator but means more duty movement on hardware.
- **Uncertainty:** low-light means scatter by about 0.3 pp of standard error per case. The differences between
  k 1.0, k 1.4 and cap k 1.0 in low light are inside that. The cloud-gain ordering and the sweep-rise failure of
  k ≤ 1.0 are consistent across all 9 realisations.

Not measured: a cap above A's threshold (e.g. 1.5-2× A's θ). At k 2.5 it would keep part of the cloud gain while
still bounding the low-light θ (9.8 % of P uncapped vs 3.1 % for A).

## k 2.0, and 120 Hz ADS1015-like sampling

The refreshed sibling patch (default `rev_k` 1.4) was re-applied, so these runs use a fresh build. With explicit k it
reproduces the k scan run for run: k 1.4 and k 2.5 match in 163/163 runs, and A matches in 163/163. Variants: A, uncapped
k 1.4 / 2.0 / 2.5. Scenarios: the k-scan set, cloud g1/g3, plus `SIM_ADC_HZ=120 SIM_NOISE=fry` on wx-dusk and wx-low
at all 18 whole-second offsets of the 18 s capture. That is 796 runs on vup, 0 errors. Data:
`adaptive-po/results/k2/`, script `analyse5.py`.

Harvest vs A, pp: mean / worst run (n runs):

| group | A % | k 1.4 | k 2.0 | k 2.5 |
|---|---|---|---|---|
| low light, all (29) | 98.13 | +0.24 / −1.82 (gauss seed2) | −0.59 / **−6.50** (gauss seed8) | −0.83 / −5.88 (gauss seed6) |
| wx-low gauss (9) | 97.61 | −0.03 / −1.82 | −2.02 / −6.50 | −1.42 / −5.88 |
| wx-low flat-g0.3 (11) | 98.45 | +0.37 / −0.54 | −0.25 / −1.05 (@10) | −0.97 / −3.32 (@6) |
| wx-low fry (9) | 98.27 | +0.35 / −1.10 (@8) | +0.42 / −0.19 (@4) | −0.06 / −2.61 (@12) |
| wx-dusk fry 120 Hz (18) | 97.77 | −0.36 / −1.93 (@13) | +0.09 / −0.91 (@10) | **+0.42 / −0.18** (@4) |
| wx-low fry 120 Hz (18) | 98.18 | +0.10 / −0.43 (@4) | +0.16 / −0.06 (@17) | +0.10 / −1.14 (@1) |
| cloud flat g3 (9) | 96.91 | +0.61 / +0.33 | +0.80 / +0.53 | +0.91 / +0.74 |
| cloud flat g1 (9) | 98.55 | +0.08 / −0.02 | −0.03 / −0.20 | +0.02 / −0.16 |
| dawn/dusk 512/1468 Hz (38) | 97.78 | +0.76 / +0.23 | +0.77 / +0.28 | +0.77 / +0.16 |
| wx-sweep-rise (10) | 96.46 | −0.07 / −0.37 | +0.14 / +0.00 | +0.18 / +0.04 |
| wob fry+flat (18) | 99.77 | +0.08 / −0.20 | +0.01 / −0.33 | −0.17 / −0.94 |
| shade (50) | 91.08 | +0.07 / −0.02 | +0.05 / −0.02 | +0.02 / −0.09 |

Reversals/min (A / k 1.4 / k 2.0 / k 2.5) and θ/P % fast | slow:

| case | rev/min | θ/P A | k 1.4 | k 2.0 | k 2.5 |
|---|---|---|---|---|---|
| wx-dusk fry 120 Hz | 25 / 25 / 23 / 20 | 4.44 \| 0.26 | 1.15 \| 0.79 | 1.17 \| 0.84 | 1.21 \| 0.89 |
| wx-low fry 120 Hz | 39 / 66 / 62 / 58 | 3.07 \| 0.44 | 1.16 \| 1.91 | 1.43 \| 2.67 | 1.74 \| 3.51 |
| wx-low gauss | 142 / 136 / 62 / 39 | 3.11 \| 0.33 | 4.64 \| 1.25 | 7.53 \| 2.06 | 9.83 \| 2.35 |
| wx-low flat-g0.3 | 71 / 152 / 81 / 48 | 3.08 \| 0.35 | 2.17 \| 1.43 | 3.35 \| 1.33 | 4.40 \| 2.18 |
| cloud flat g3 | 186 / 189 / 117 / 88 | 0.44 \| 0.10 | 0.34 \| 0.47 | 0.53 \| 0.65 | 0.67 \| 0.77 |
| wx-shade2 fry | 8 / 33 / 19 / 13 | 0.64 \| 0.15 | 0.15 \| 0.08 | 0.22 \| 0.13 | 0.28 \| 0.17 |

Findings:

- **k 2.0 does not keep the low-light tail near k 1.4.** On Gaussian wx-low it is as bad as k 2.5, worst −6.50
  (seed 8), with 5 of 9 seeds below −1.7. The fast θ is already 7.5 % of P there. On the measured-noise low-light
  cases it is fine: worst −1.05 (flat-g0.3) and −0.19 (fry), better than k 1.4's −1.10 on fry. The tail family is the
  white-noise wx-low, the same one that inflates θ in the diagnosis above.
- **120 Hz dusk rises with k, as the sibling saw on the Mac,** but the mean gain is smaller than a single run
  suggests. At offset 0 alone, vup gives A 96.67, k 1.4 97.60, k 2.0 97.94, k 2.5 98.09 %. Over 18 offsets the
  differences are k 1.4 −0.36 (se 0.14, worst −1.93 @13), k 2.0 +0.09 (se 0.12), k 2.5 +0.42 (se 0.11). k 2.0
  keeps about a fifth of k 2.5's 120 Hz dusk gain. 120 Hz wx-low is flat for all k (+0.10..+0.16), but k 2.5 has a
  −1.14 run.
- **Cloud g3:** k 2.0 keeps +0.80 of k 2.5's +0.91.
- Dawn/dusk at the boards' rates, shade and sweep-rise are equal within 0.1-0.25 pp for k 1.4-2.5. k 1.4 is the only
  variant that is slightly negative on sweep-rise.

No uncapped k from 1.4 to 2.5 meets all three conditions: low-light tail ≈ −1.8, 120 Hz dusk gain, and most of the
cloud gain. k 2.0 meets the last two only partly (120 Hz dusk +0.09 of +0.42, cloud +0.80 of +0.91) and fails the
first on white noise. What separates the variants is the low-light white-noise case. If that case is accepted as
unrepresentative (see "What limits trust"), k 2.0 is the best uncapped compromise. If it is not, the remaining option
is a cap above A's threshold, which still has not been measured.

## θ scale and the "loss between θ/3 and 2θ/3" claim

A with both thresholds scaled ×0.25-4 (`TK_THSCALE`), wx-sun-fry at 20, 70 and 200 W, noise-free plus fry noise at 9
offsets. The ratio of the slow-mode tracking loss to the slow-mode θ (both relative to the GMPP):

- **Noise-free: 0.54-0.80** for scale 0.25-2 at every power level, slightly above the 2θ/3 limit. The cause is
  filter and period lag: the reversal comes after the crossing. At scale 4, 0.31-0.42, because the 5 % Vin reversal
  rule cuts the band. **The claim holds** to within lag. The reference is held between Pm−θ and Pm, and the
  reversal level sits θ below it.
- With measured noise and θ ≳ 3σ (scale ≥ 1 at 70 W, where slow θ = 0.16 % and σ_slow = 0.057 %): 0.43-0.69.
- Below θ ≈ σ (scale 0.25) the loss stops falling with θ and settles at about **1σ**: 0.06-0.09 % with σ_slow ≈
  0.057 %. That is about twice the doc's 0.44σ random-walk figure, consistent with its "within a factor of 2". The
  ratio rises to 1.5-1.9, with 33-49 reversals/min.
- A's own slow θ sits at about 2.8σ (k ≈ 1.9). Its fast θ (1.45 % at 70 W) is about 16σ_fast (k ≈ 11).

## Coloured noise

With the duty held (`SIM_HOLD=150`, wx-sun-fry 70 W), the period-mean noise is σ_fast = 0.090 % (33 ms) and σ_slow =
0.059 % (500 ms) for fry noise, and 0.079 % / 0.059 % for flat noise. That is **σ_fast/σ_slow ≈ 1.3-1.5**, not the
√15 ≈ 3.9 of white noise. The fry3-wander profile is even inverted (0.10 % fast, 0.31 % slow). Lengthening the period
buys little noise reduction on measured noise, which matches the sibling's reason for rejecting C. My σ̂ (second
difference, no correlation correction) reads about 15 % below the held-duty value in slow mode (0.050 vs 0.059 %).

## What limits trust

- **Noise model.** Steady-sun tracking loss in the simulator is 0.1-0.14 % (A), well below the 0.5-0.6 % seen in the
  field. Either the replayed noise (an 18 s capture, looped) lacks the slow content of real days, or other field
  effects are missing: temperature drift, real irradiance micro-variation, current-sensor offset drift. The
  absolute gains and losses are therefore lower bounds, and their ranking may not survive real noise.
- **Gaussian low-light cases (wx-low) dominate the tails.** Their noise is white with σ of several % of P, unlike
  the measured profiles. That makes them the most sensitive cases for θ tails and the least field-like.
- **Low-light scenarios are chaotic.** Single runs swing several pp on any change. Means over 9-11 realisations are
  needed. Standard errors of the large case means are about 0.1-0.3 pp.
- **One host (vup).** Values differ from the Mac and pgx93 (wx-sun-fry fry: Mac 99.90, vup 99.93 %). The sibling's
  Mac numbers are a different realisation set.
- **Fixed plant.** One board plant (fry, flat for `-flat`) and one input capacitance.

## Recommendation

1. Do not put C or D on hardware. Neither adds harvest over B.
2. Do not use a relative floor of 1-4 % of P. It costs on every non-low-light case and up to 12 pp at a
   cloud edge in a sweep.
3. The gain (+0.5..1.1 pp at dawn/dusk, +0.2 pp on a slow ramp, about 0..+0.1 pp elsewhere) comes from a lower
   fast-mode threshold at low power. A fixed lower fast threshold is unsafe at cloud edges, so it needs the noise
   estimate. If B goes to hardware, use B with an upper cap at A's own threshold, i.e.
   θ = clamp(k√2σ̂, floor, A's threshold for the mode). Bs without a cap lost up to 5.9 pp per run and 1.4 pp on
   average in wx-low. The cap on the sibling's estimator (BsCap, section above) brings the worst wx-low run to
   −1.28 pp and keeps the dawn/dusk gain. A Huber 4× clamp or a longer EWMA does not replace it. The cap gives up
   the high-noise cloud gain (+0.91 pp at gain 3 uncapped). Uncapped k 1.4 is the compromise if that case matters:
   low-light mean +0.24 pp, worst run −1.82, +0.61 of the cloud gain, at 2-4× the reversal rate. k 1.4 loses at
   120 Hz dusk (−0.36 mean, worst −1.93), where k 2.5 gains +0.42. k 2.0 sits in between on the measured-noise
   cases, but its Gaussian wx-low tail (−6.50) is as bad as k 2.5's. A cap at 1.5-2× A's threshold has not been
   measured.
4. Expect at most a few tenths of a percent of daily energy, concentrated in the first and last hour of light. A
   hardware A/B on fry vs flat with a day-long energy comparison can resolve that only over many days. Check the
   reversal rate on hardware as well (2-3× with k = 2 and my estimator, unchanged with Bs).
