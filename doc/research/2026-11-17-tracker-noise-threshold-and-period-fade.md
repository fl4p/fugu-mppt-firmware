*this document is an LLM generated placeholder*

# Tracker: noise-derived reversal threshold, and a continuous fast/slow period (simulator study)

Date: 2026-11-17. Simulator only, NOT tested on hardware. Code state: worktree
`.claude/worktrees/agent-ac8074df43b552ff5` (step 1 applied, uncommitted); patches
`/tmp/nt-edit/step1-full.patch` (recommended) and `/tmp/nt-edit/fade-full.patch` (rejected, record only).

## Question

Fab asked whether the tracker's binary fast/slow switch should fade (a continuous or adaptive transition), and
flagged the hard-coded reversal thresholds (1.0/0.5 W, 1.5 %/0.15 % of P + 5 W) as magic constants.

## Field evidence (status-line arrows, havan `pv/fugu_console.log`, 2026-11-17 12:31 to 17:03 UTC, 40 to 70 W)

| device | slow-mode entries | slow share | median episode | longest |
|---|---|---|---|---|
| fry | 113 in 4.4 h | 38 % | ~36 s | ~4.3 min |
| flat | 129 in 4.5 h | 40 % | ~30 s | ~4 min |

The 30 s entry gates and the exit on any 0.5 % new MPP record limit-cycle in variable light: an entry every ~2.4 min.

## Method

Host closed-loop sim (`test/host-stub/mppt-sim.sh`), scenarios wx-mid, wx-dusk, wx-low, wx-wob-fry, wx-cloud,
replayed sensor noise from fry and flat (`SIM_NOISE`), noise gain 0.3 / 1 / 3 (`SIM_NOISE_GAIN`), and for the
headline figure three replay offsets 0 / 100 / 200 s (`SIM_NOISE_OFS`). New scripts
`test/host-stub/trk-noise-sweep.sh` (parallel sweep) and `trk-sweep-diff.py` (tables, deltas). ADS1015-like
case: `SIM_ADC_HZ=120 SIM_NOISE=fry`. The sim is deterministic per input; the same case at a different replay
offset differs by 0.2 pp median, up to 1.0 pp (baseline, 30 cases), so single-case deltas below about 0.5 pp
are trajectory sensitivity, not an effect. Only sweep means are interpreted.

Baseline (current `main`, ef1e4cf5) signature of fixed constants: at gain 3 the noise drives reversals
(wx-cloud flat: 188/min, harvest 96.6 % vs 98.6 % at gain 1); at gain 0.3 nothing improves, the threshold cannot
shrink below its constants.

## Step 1: noise-derived threshold (recommended)

`Tracker` estimates the noise of a period mean from the second difference of successive period means,
d = P_n - 2 P_{n-1} + P_{n-2}, EWMA(32) of d^2 per mode, Huber clamp at 9x the current estimate (and at least
the floor-equivalent variance, see defect below), 8-sample warm-up on the old fixed rule. The power filter
correlates successive means; with rho = exp(-T/tau), tau = settleMs/6.64 (settleMs is the tracker's existing
99 % filter settling time), var(d) = 2 sigma^2 (1 - rho)(3 - rho), and the decision quantity P_n - P_ref has
variance up to 2 sigma^2:

    theta = max(rev_k * sqrt(2 sigma^2), rev_floor_w),   sigma^2 = E[d^2] / (2 (1 - rho)(3 - rho))

Without the correction sigma is underestimated by 1.2x at 450 Hz (rho 0.24) and 2.6x at 120 Hz (rho 0.80); the
uncorrected version lost 0.9 pp on the 120 Hz dusk case, the corrected one gains 0.7 pp there.

New conf keys (tracker.conf): `rev_k` (default 2.5), `rev_floor_w` (default 0.05; the sensor's power step is
not exposed by `AsyncADC`, so the floor is a key for now). `minPowerStep` holds the threshold of the last
decision (debug field); `minPowerStepRel` is gone.

Results, 30-run sweep (offset 0), harvest vs baseline:

| group | delta |
|---|---|
| mean | +0.21 pp |
| wx-dusk (6 cases) | +0.4 to +0.7 pp |
| wx-cloud flat gain 3 | +1.2 pp (reversals 188 -> 85/min) |
| worst case (wx-low fry gain 3) | -0.4 pp (within offset sensitivity) |
| 120 Hz dusk / low | 96.9 -> 97.6 % / 98.3 -> 98.4 % |

90-run sweep (offsets 0/100/200): see "Headline" below. `rev_k` = 4: +0.16 pp vs baseline, i.e. equal to 2.5
within noise at 450/512 Hz; 120 Hz dusk still prefers 4 (97.6 -> 98.4 %). Default left at 2.5.

Known limits and the two guard defects found on the way:
- Slow-mode warm-up trap: slow mode reverses every other period, each reversal clears the history and a sample
  needs three quiet periods, so the slow estimator can stay below 8 samples and run on the fixed rule for a
  while (7 to 44 % of slow periods depending on the run). The old constants are still in the code for that.
- Zero lock (found by reading `test/test_tracker.cpp`): with an estimate of exactly 0 the Huber clamp zeroed
  every later sample. Fixed by clamping against max(9 var, floor-equivalent variance).
- Starvation lock (found by the host test `test/host-stub/tracker-noise-test.cpp`): noise that reverses the tracker
  every other period keeps clearing the estimator's history, no sample ever arrives, the threshold sits on the floor
  and causes more reversals. Fixed by the guard rule "unevaluable is not OK": after 32 periods without a sample the
  fixed rule applies until samples resume (`_noiseIdle`). The host test feeds 200 constant periods then alternating,
  random and staircase noise; the pre-fix tracker fails it (threshold stuck at 0.05 W), the final one passes
  (1.0 / 0.75 / 3.42 W). In the sweep the fallback engages in 2 to 9 % of fast periods during clouds and costs
  nothing (+0.03 pp vs without, 15 of 30 runs bit-identical). The slow-mode warm-up trap above is the same
  mechanism and is now covered by the same fallback.
- `test/test_tracker.cpp` compiles; constant-power tests now run on the floor, and
  `test_tracker_converges_to_peak` gets its deadband from the parabola's curvature, not a noise model. Not run
  on target.

## Step 2: continuous decision period ("fade") - rejected

T continuous in [33, 500] ms, x1.3 per reversal without a new record, snap to 33 ms on a new record, takeover,
45 s one-way or the 5 % Vin rule; slew 0.5 x 33/T; one noise estimator on fixed 33 ms sub-means, the threshold
at period T from the closed-form variance of a mean of n = T/33 AR(1) sub-means (formula verified against a
brute-force sum and Monte Carlo). Entry jump and 30 s gates removed.

| comparison | harvest delta (30 runs) |
|---|---|
| fade vs baseline | -0.25 pp (wx-cloud flat 3: -3.9) |
| fade vs step 1 | -0.46 pp |
| fade k=4 vs step 1 k=4 | -0.26 pp |
| 120 Hz dusk / low vs baseline | -2.9 / -0.9 pp |

Why it loses: the AR(1) extrapolation from 33 ms to 500 ms assumes the noise averages down ~1/n, but the power
signal's slow content (flicker, the 0.2 Hz wobble, the dusk ramp's curvature) does not; the long-period
threshold landed on the floor (0.7 to 0.9 permille of P in wx-wob vs 2.1 to 2.8 for step 1's slow estimator,
which measured the 500 ms means directly) and irradiance drift within a leg read as a power drop. T also grew
to its maximum within ~8 reversals and rarely snapped back in falling or wobbling light (wx-wob: 2 to 5 snaps in
13 min). The old slow mode has the same geometry but is hard to enter and easy to leave, which turns out to be a
sound policy. Gating the settle window on T < 2 settleMs instead of T < 250 ms made it worse (-0.09 pp).

Untried ideas, recorded for completeness: an Allan-style multi-octave estimator (block means at n = 1, 2, 4, 8,
16) interpolated at the actual T; growing T only while reversals occur at about equal Vin.

## Conclusion

The magic-constant complaint is answered for the threshold: step 1 removes four constants and the 5 W knee in
favour of a measured sigma and one k, is never worse on average, and fixes the two ends of the noise range
(too many reversals at high noise, a frozen blind band at low noise). The fade is not worth it in this form; the
field bounce costs less than the long-period threshold problem it creates. Harvest numbers are simulator only.

## Headline (final build, 90 runs: 5 scenarios x fry/flat noise x gain 0.3/1/3 x offsets 0/100/200 s)

| group | harvest delta vs baseline |
|---|---|
| overall | +0.14 pp (se 0.07 over the 30 case means, see below), no trip changes, reversals -3.5/min on average |
| wx-dusk | +0.49 pp (sd 0.32, all 18 runs positive or zero) |
| noise gain 3 | +0.25 pp |
| noise gain 1 / 0.3 | +0.11 / +0.05 pp |
| wx-low | -0.04 pp (sd 0.64) |
| wx-mid, wx-wob, wx-cloud | +0.04 / +0.06 / +0.14 pp |

Distribution of the 90 run deltas: 50 positive, 15 zero (bit-identical runs), 25 negative; 20 runs at or above
+0.5 pp, 6 at or below -0.5 pp. The three offsets of one noise file are shifted replays, not independent samples,
so the standard error is taken over the 30 case means (sd 0.37, se 0.07), not over 90 runs (0.05).

Tail: the worst two runs are the same case, wx-low with flat noise at gain 0.3, -1.4 pp (offset 100) and -1.6 pp
(offset 200); the next three are -0.6 pp (wx-low flat 1, wx-wob flat 1, wx-wob fry 1). The wx-low case is a real
regression at low light and low noise, not trajectory noise: with little noise the threshold shrinks to the
0.05 W floor and at a few watts the tracker then reacts to the input-cap lag and the light itself. A floor scan on
that case (3 offsets each, harvest %):

| rev_floor_w | 0.05 | 0.1 | 0.2 | 0.4 | baseline |
|---|---|---|---|---|---|
| ofs 0 / 100 / 200 | 98.3 / 97.5 / 97.1 | 97.2 / 98.0 / 94.5 | 95.7 / 98.5 / 98.6 | 98.9 / 99.2 / 99.0 | 98.6 / 98.9 / 98.7 |

The case is chaotic (4 pp spread at floor 0.1) and a 0.4 W floor holds it at or above the baseline, but at 2 W a
0.4 W floor is 20 % of P, far above the baseline's effective threshold there (1.5 % of P + 5 W = 0.1 W). That
such a tracker still harvests 99 % says the 5 % Vin band alone does better than P&O decisions in that regime. Open
question, not tuned here: whether the floor should be relative to P (one constant with a meaning: the smallest
relative power change worth reacting to), or whether the sibling study's fixed-smaller-threshold sweep (below)
explains the dusk gain without an estimator at all.

Independent replication (sibling session fugu-mppt-firmware-4a, its own estimator without the AR(1) correction,
41 cases x 9 realisations on vup): +0.16 pp at k = 2, dawn +0.7 to +1.1, dusk +0.5 to +0.7, shade no regression;
k = 3 worse where the wobble inflates sigma (wob-fry -0.8 pp). Its reversal rate rose 2 to 3x at dusk where mine
did not change; the uncorrected sigma is 1.2x low at 450 Hz, so its effective k is lower. It also measured the
noise as coloured, sigma_fast / sigma_slow ~ 1.5 instead of sqrt(15), which is the fade post-mortem in one number.
Its follow-up sweeps on vup (write-up doc/research/2026-11-17-adaptive-po-sim.md in its worktree
`.claude/worktrees/agent-a0a7027d0a9b09689`, tables in `doc/research/adaptive-po/results/analysis2.txt` there):

- Fixed smaller thresholds do not replace the estimator. At dawn/dusk a fixed 0.2x scale gains +0.77 pp, the same as
  B (+0.77 mine, +0.74 its own), but on wx-sweep-rise it loses in every realisation: 0.33x -2.9 pp (worst -3.6),
  0.2x -4.4 (-5.0), 0.1x -5.8 (-6.2), and up to -1.1 pp on the flat-board wobble. The threshold has to rise during
  transients, so the adaptivity is what earns the gain.
- My low-light tail reproduces on vup (wx-low flat gain 0.3 over 11 offsets: -0.97 pp mean, worst -3.3; Gaussian
  wx-low -1.4 mean, worst -5.9) and its cause is the opposite of the floor hypothesis above: in that run the fast
  threshold averaged 11 % of P against 3.1 % for the baseline. At low light the estimator inflates. The floor scan
  above is therefore a chaotic coincidence on 3 offsets, not evidence for a higher floor.
- Rejected: the relative floor max(0.05 W, f P) is worse everywhere (f = 0.01/0.02/0.04: -0.09/-0.25/-1.11 pp) and
  does not fix wx-low. The variable step D (step from |dP/dV|) adds +0.00 pp on top of B.
- Reversals: my B keeps the baseline rate because its fast theta at dusk is 2.1x the sibling B's; AR(1) and k 2.5
  vs 2 explain about 1.5x of that.
- Diagnosis of the low-light inflation (vup, 580 runs, `doc/research/adaptive-po/results/diag/analysis3.txt` in
  its worktree): it is not an artefact. The estimator gets a sample in ~80 % of fast periods, the starvation
  fallback is active in 0.1 %, and samples taken right after a reversal or settle window have the same rms as the
  rest. The uncapped theta is 1.8 to 2.0 x sqrt(2) x the period-mean noise measured on the baseline itself; on
  Gaussian wx-low that noise is 3.5 % of P per fast period. The noise really is that large, and the baseline wins
  there because its fixed rule runs at an effective k of about 0.6 in fast mode and 0.2 to 0.3 in slow mode, i.e.
  below the noise. Below theta ~ sigma the loss is about 1 sigma; at k = 2.5 it is about 1.2 sigma plus a slower
  reaction to rising light. The threshold-band model (theta/3 .. 2 theta/3) does not apply in that regime.
- Neither estimator lever removes the tail: Huber clamp 4x gives Gaussian wx-low -2.7 pp mean (worst -14), span 64
  gives -1.2 / -4.9; both move theta by only a few percent.
- Capped B, theta = min(estimate, baseline fixed threshold for the mode): Gaussian wx-low +0.19 (worst -0.44),
  flat gain 0.3 -0.19 (worst -1.28), fry -0.07 (worst -1.14), dawn/dusk +0.76 (same as uncapped), sweep-rise -0.03
  (worst -0.36), wob-fry -0.02 (worst -0.52), shade worst -0.06. The cap binds in every slow-mode case, so slow mode
  behaves like the baseline; in fast mode capped B is a smaller threshold that returns to the baseline's when the
  noise is high. It keeps the old constants load-bearing as a ceiling.
- k scan, uncapped vs capped (vup, 996 runs, `analysis4.txt` in the same results folder), mean / worst vs baseline:

  | case | k 0.7 | k 1.0 | k 1.4 | k 2.5 | capped k 1.0 | capped k 2.5 |
  |---|---|---|---|---|---|---|
  | wx-cloud flat gain 3 (baseline 96.9 %) | 0.00 / -0.30 | +0.30 / -0.04 | +0.61 / +0.33 | +0.91 / +0.74 | -0.05 / -0.66 | -0.10 / -0.57 |
  | low light (29 runs) | +0.20 / -1.57 | +0.30 / -1.67 | +0.24 / -1.82 | -0.83 / -5.88 | +0.34 / -0.55 | -0.04 / -1.28 |
  | wx-sweep-rise | -1.81 / -2.34 | -0.87 / -1.29 | -0.09 / -0.37 | +0.19 | -0.84 / -1.11 | -0.03 / -0.36 |
  | dawn / dusk | +0.75 to +0.77 for every variant | | | | | |
  | shade | unchanged, worst -0.09 | | | | | |

  The cloud gain rises with k and so does the low-light tail; the cap hides the cloud trade (it gives the whole
  high-noise gain back). k <= 1 loses on the rising sweep, capped or not. **k = 1.4 uncapped is the best single
  setting**: two thirds of the cloud gain, low light within the baseline's own offset spread (97.1 to 99.0 %),
  on par at sweep-rise, best on the wobble, full dawn/dusk gain, no ceiling. Its cost: 2 to 4x the baseline reversal
  rate in steady light and shade (shade2 8 -> 33/min), no harvest cost in the sim; to be looked at on the bench
  (dither on the bus, switching transients). Not run: a cap at 1.5 to 2x the baseline at k 2.5; it would reintroduce
  a constant for at most a third of the cloud gain.

Mac rerun of the step-1 sweep with k 1.4 (30 runs, offset 0, `/tmp/trk-sweep-nt5.txt`): vs baseline +0.13 pp
(min -0.9, max +0.9); vs k 2.5 -0.11 pp, three cases below -0.5 (wx-dusk flat gain 3 -1.3, wx-wob-fry flat 0.3
-1.1, wx-low fry 1 -0.9), reversals +55 to +106/min in mid/low/cloud. 120 Hz dusk: baseline 96.9, k 2.5 97.7, k 1.4
97.0 %, so k 1.4 gives the ADS1015-like dusk gain back (the earlier k scan there rose monotonically with k). The two
sims therefore disagree in direction on the mean by about 0.1 pp, inside the single-offset sensitivity; the worst
case decides (-1.8 pp at k 1.4 vs -5.9 pp at k 2.5 on vup), and favours 1.4. k 2.0 is unmeasured on both sides and
was proposed to the sibling study together with the two 120 Hz cases.

k 2.0 run (vup, 796 runs from the refreshed patch, `results/k2/analysis5.txt`), mean / worst vs baseline:

| case | k 1.4 | k 2.0 | k 2.5 |
|---|---|---|---|
| Gaussian wx-low | -0.03 / -1.82 | -2.02 / -6.50 (5 of 9 seeds below -1.7, fast theta 7.5 % of P) | -1.42 / -5.88 |
| measured-noise low light, flat gain 0.3 / fry | ok | -0.25 / -1.05 and +0.42 / -0.19 | see above |
| 120 Hz dusk, 18 offsets | -0.36 / -1.93 | +0.09 / -0.91 | +0.42 / -0.18 |
| 120 Hz wx-low | +0.10 to +0.16 for all | | |
| wx-cloud gain 3 | +0.61 | +0.80 | +0.91 |
| dawn/dusk at board rates, shade, sweep-rise | within 0.25 pp across k | | |

The single-offset 120 Hz dusk numbers above (96.7 / 97.6 / 97.9 / 98.1 at offset 0) overstate the gain; over 18
offsets k 1.4 loses there on average. No uncapped k from 1.4 to 2.5 meets all three conditions (low light bounded,
120 Hz dusk gain kept, cloud gain kept); the decision hinges on whether Gaussian white noise at low light is
representative. The measured profiles were captured at 2.7 to 4 A and are scaled down with current by a power law,
so they may under-represent the INA226's current-independent noise floor at a few hundred mA, which is the
regime where k 2.0 and 2.5 fail; the Gaussian case may be the more realistic one there. That is a measurement
question, not a simulation question.

## Decision and next step

**Superseded 2026-11-18:** the dusk captures showed an Iout noise floor of about 8 to 11 mA that does not scale
with current, and no rev_k won on both the measured dusk profiles and the old ones. Fab decided not to land
rev_k; the fixed reversal rule stays. See `doc/research/2026-11-18-filter-chain-noise-study.md` (51d0ac7c,
5436dfee). The text below is the state before that decision.

`rev_k` default 1.4, no ceiling, until the low-light noise is measured: it is the only setting whose worst case is
bounded under both noise models, at the cost of the 120 Hz dusk gain (-0.36 pp there) and a third of the cloud gain.
Next step before landing or retuning: an adcscope capture on fry or flat at dawn or dusk below 0.5 A, turned into a
noise profile with `noise_profile.py`, and the k scan repeated on it. If the real low-light noise looks like the
scaled measured profiles, k 2.0 is the better default; if it looks Gaussian-white at the LSB floor, 1.4 stands. A cap
at 1.5 to 2x the old rule remains the fallback option if neither k satisfies Fab; it keeps the old constants as a
ceiling. The old fixed values otherwise stay only as the warm-up and starvation fallback. The reversal rate is 2 to
4x in steady light at any k below 2.5, to be checked on the bench. The
threshold is one constant with a measured meaning (theta = 1.4 sqrt(2) sigma) plus a floor of 0.05 W. The
sibling session puts the regime note and the new rule on the docs page; this patch (worktree above, rev_k 1.4)
is the candidate for main, flagged simulator-only, with `test/test_tracker.cpp` to be run on target and a bench
flash before fry or flat.

Results: /tmp/trk-sweep-base90.txt vs /tmp/trk-sweep-nt490.txt (Mac host sim; pgx93 and vup results differ bit-wise
and must not be mixed in). The harvest gain is small and real at dusk and at high noise, neutral elsewhere; the case
for the change rests on removing the constants and on behaving the same across sensors and noise levels, and it is
not settled until the fixed-threshold sweep and the low-light floor question are answered.
