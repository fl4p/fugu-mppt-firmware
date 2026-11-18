*this document is an LLM generated placeholder*

# Sensor filter chain vs tracker noise (offline study on stored captures)

Date: 2026-11-18. Offline analysis of stored adcscope captures only; no firmware change, NOT tested on hardware.
Scripts: `doc/research/filter-chain/` (`chain_study.py`, `latency_study.py`, `band_split.py`, `lowband_vin.py`),
run from the repo root against `etc/adcscope/data`. 17 captures (fry, flat, bench; inverters, EV charger,
heater, cheap China inverter, quiet 30 W).

## Question

Does the firmware chain (notch → median-of-5 → 2-pass EWMA span 20 → tracker period mean) leave noise in the
tracker's power estimate that a different filter could remove? Candidates: a boxcar over a whole number of
ripple periods without the EWMA, the outlier-only `despike` instead of the unconditional median, an adaptive or
"learned" filter.

## Method

P = |Iout| · Vout per sample (output side, as the tracker on fry/flat uses it). The residual is P minus its
zero-phase 0.3 Hz low-pass, plus the mean (same convention as `test/host-stub/noise_profile.py`). The chain is
modelled in Python: the firmware biquad notch (Q 20, −30 dB) at the capture's strongest line in 0.12..0.34
cycles/sample, median-of-5, despike k=8 (firmware rule), 2-pass EWMA span 20, period 33.3 ms at the file's
labelled rate. Labelled rates are approximate (fry's INA226 runs ~10 % fast), so frequencies are normalised.

## Results

### Equal-latency comparison

The chain needs 3 to 5 fast periods (100 to 167 ms) until a step shows at 95 % in the period mean. A boxcar of
the same latency (whole ripple periods) has period-mean noise of 0.87 to 0.99x the chain's on 13 of 17 captures;
half that latency costs 0 to 28 % more noise. The 2-pass EWMA is therefore close to the best linear filter for
this latency. Replacing it with a boxcar (step 3 of the plan) buys at most ~10 % noise or ~2x latency at
roughly equal noise.

### Where the remaining noise sits

Raw P residual variance is dominated by ripple lines and content above 60 Hz (inverter captures: 81 to 96 % in
lines), which the notch and the averaging already remove. What reaches the tracker decision is the 0.3..15 Hz
band: e.g. fry 2 kW inverter, raw sd 22 %, of which 2.4 % of the variance is below 15 Hz (≈3.4 % sd); the
chain's period-mean sd is 4.1 %. Period means therefore do not average down with a longer window, which agrees
with the 11-17 sim result that the period fade lost ("slow content does not average down like noise").

Regressing that band on Vin (linear + quadratic) explains 0 to 2 % of it on 12 of 17 captures (23 to 70 % on
three inverter captures of flat/fry, 42 % on 30w-low-noise). So it is mostly not the tracker's own dither; it is
load / bus / irradiance wander or measurement drift at a fixed operating point. Caveat: Vin is resampled by
labelled rates; a rate error smears the alignment and would lower R².

### Median vs despike

| effect | captures |
|---|---|
| median bias (notch → med5 vs raw mean) | −0.5..+0.4 % on most; −6.8 % sunny (kurtosis 79), +14 % fry/145718 (near-zero current, degenerate) |
| median without notch first | −1..−5 % on inverter captures (fry 2 kW, China inverter), confirms notch-before-median |
| despike k=8 bias | within ±0.1 % except sunny (−7.4 %, spikes clipped as glitches) and heater (−1.3 %) |
| despike vs median noise at equal latency | ~equal on 12 captures; 0.47..0.55x on pulsed loads (pv8A-inv500W, heater500w, fry/145718); 1.46x on china-inverter-1kw-load |

## Implications

- The chain is not the bottleneck. Filter rework (boxcar, adaptive span, LMS ripple canceller) targets content
  the notch and EWMA already remove; expected tracker gain is small.
- The tracker's decision noise is low-frequency wander that no causal per-period filter separates from a duty
  step. Levers that address it: a noise-derived reversal threshold (11-17 step 1, pending), drift-compensated
  P&O (dP-P&O, a mid-period sample to subtract a linear trend), dither demodulation (lock-in / extremum seeking),
  or a Vin-referenced loop that rejects bus disturbances before they reach P.
- `despike` is the one filter change with a measurable effect (unbiased mean, half the noise on pulsed loads,
  worse on the China-inverter spike train). Host-sim result below: +0.31 pp mean once the calibration gate was
  fixed.

## Boxcar tracker window in the host sim (rejected)

Prototype `tracker.conf::pwr_box=1` (patch `doc/research/filter-chain/pwr_box.patch`, base 858fe4a4, not on
main): tracker power from `I.med3 * V.med3` per tick (no EWMA), Vin from `med3`, decision window a whole number
of detected ripple periods counted in Iout samples (fast ~30 ms, slow ~500 ms), settle time = median delay only.
pwr_box=0 is bit-identical to main. Same 90-run sweep on vup, pwr_box=0 vs 1 (`/tmp/mppt-sim-1e/box/`):

| scenario | harvest delta mean | min | max | reversals |
|---|---|---|---|---|
| wx-low | -0.84 pp | -3.90 | +1.60 | +108 % |
| wx-cloud | -0.28 pp | -1.40 | +0.10 | up (1.3-2.5x in spot checks) |
| wx-mid | -0.08 pp | -0.50 | +0.60 | +65 % |
| wx-wob-fry | -0.13 pp | -0.50 | +0.30 | +6 % |
| wx-dusk | +0.06 pp | -0.40 | +0.60 | +3 % |
| all 90 | -0.26 pp | -3.90 | +1.60 | |

Why the offline result did not carry over: the offline metric was the noise of one period mean against a held
reference. The tracker decides on dP between successive periods, and the EWMA correlates successive means, which
lowers the noise of that difference; independent box windows lose it. The tracker's fixed reversal thresholds
are tuned to the correlated chain. A noise-derived threshold (rev_k) would scale with the box noise, but the
offline gain (at most ~10 %) leaves little to win. Not pursued further.

## Despike in the host sim

`trk-noise-sweep.sh`, 5 scenarios x fry/flat noise x gain 0.3/1/3 x offsets 0/100/200, vup, despike=0 vs
despike=8 via `SIM_CONF=sensor.conf:despike=N`. Results `/tmp/mppt-sim-1e/{ds0,ds8,fx-ds0,fx-ds8}.txt`.

First run (main 858fe4a4): despike=8 lost ~2 pp in wx-low, from extra sweeps (258 -> 315 in total). Cause: the
calibration noise gate computed `ewm.std * |avg|`, i.e. rms(dx)^2 / |avg| on a normalised EWM variance, which
blows up at a ~0 A offset. With the median, successive samples were often identical and it passed; with despike
every calibration failed, `timeLastCalibration` never advanced and the 30-min periodic sweep fired back to back.
Fixed in 670a3ef9 (sd from successive differences of the calibration samples, NaN fails). The same gate also
failed now and then with despike=0 (2 of 4 calibrations in one wx-low run).

After the fix (main 670a3ef9), despike=8 vs 0, harvest delta:

| scenario | mean | min | max |
|---|---|---|---|
| wx-low | +0.63 pp | -0.40 | +4.50 |
| wx-cloud | +0.11 pp | -0.20 | +0.80 |
| wx-dusk | ~0 (one +14 pp baseline outlier excluded) | -0.80 | |
| wx-mid | ~0 | -0.10 | +1.00 |
| wx-wob-fry | -0.04 pp | -0.50 | +0.50 |

Sweep counts equal (258 vs 257). The fix alone (despike=0, before vs after) is neutral: -0.01 pp mean.

Deployment order matters: despike on a device without 670a3ef9 reproduces the back-to-back sweep loop.

## Low-light noise capture and the rev_k scan

Dusk captures 2026-11-18 with `test/host-stub/noise_capture.sh` (read-only scope stream): fry 16:29 UTC at 0.45 A
(5 min, steady 0-45 s), fry 16:41 UTC at 0.14 A (sweep at 180-240 s, profile from 30-170 s), flat 16:39 UTC at
0.28 A (another session's capture, 0-55 s before a sweep). Raw data in `etc/adcscope/data/{fry,flat}/dusk-lowlight-*`
(fugu-data submodule, not committed there yet); profiles `test/host-stub/noise/{fry-dusk045,fry-dusk014,flat-dusk028}.fnz`.

| profile | I | Iout rms | 30 ms mean rms | old profile scaled to this I |
|---|---|---|---|---|
| fry-dusk045 | 0.45 A | 8.1 mA (kurtosis 18) | 2.9 mA (0.65 % of I) | 17 mA rms (fry.fnz 3.8 %) |
| fry-dusk014 | 0.14 A | 10.9 mA | 7.9 mA (5.6 %) | 5.4 mA rms |
| flat-dusk028 | 0.28 A | 11.0 mA | 4.4 mA (1.6 %) | 2.6 mA rms (flat.fnz 0.9 %) |

The Iout noise has a floor of ~8-11 mA that does not scale with current, so the old profiles (scaled with I)
understate low-light period noise 10-30x. fry-dusk014 carries 44 % of its variance below 2 Hz, most likely the
tracker's own dither at a current that follows the duty; it overstates noise.

rev_k scan (port of the 11-17 patch onto main 0e039be1, worktree `agent-a1533e95825db54ec`, uncommitted) vs a main
build, vup, 5 scenarios (wx-low/dusk/dawn/cloud/mid) x the 3 dusk profiles with `SIM_NOISE_EXP=0` x offsets
0/10/20 s, plus the old fry/flat profiles (exp 1, offsets 0/100/200). Harvest delta vs main, mean / worst [pp]:

| k | dusk profiles (45 runs) | of which fry-dusk045 / flat-dusk028 / fry-dusk014 | old profiles (30 runs) |
|---|---|---|---|
| 1.4 | -0.56 / -5.40 | +0.12 / -1.12 / -0.67 | +0.26 / -0.60 |
| 2.0 | -0.47 / -8.70 | +0.38 / -0.35 / -1.43 | +0.25 / -1.80 |
| 2.5 | -0.21 / -7.80 | +0.33 / -0.14 / -0.81 | +0.11 / -5.20 (wx-low) |

Trip counts equal to main. The large losses are wx-dusk with fry-dusk014 (main 92-93 %, k 1.4 87-89 %, k 2.5
85-90 %). No k beats the fixed rule under both the measured low-light noise and the old profiles: k 1.4 loses on
flat's floor, k 2.5 loses on the old wx-low case, and the best gains are +0.1..+0.4 pp. Recommendation: do not
land rev_k as the default.

## Limits

Captures are 2025-03 to 2026-06, mostly mid/high current; the low-light case (< 0.5 A, INA226 floor) that
decides `rev_k` is not in the set. The decision noise metric ignores plant settling (input-cap lag), which is
equal for all variants.
