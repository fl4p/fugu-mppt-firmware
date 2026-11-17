*this document is an LLM generated placeholder*

# rect-cal on the corrected board plants: bias, spread and the late-knee rule

Date: 2026-11-17. Branch `rcal-vin3` (off main e4cb0616): c58ce398 (sim harness), 2aa807ce (port of rcal-vin2),
3ee071b3 (late rule + regression), 344f77ab (docs), then the review fixes (T/32 idle gap, positive results only below
2 Vout, load/measure-coil clamp, rcal-vin.sh bias checks), then the second review fixes (drain gap and M limit, load
clamp in counts, baseline-relative rcal-vin.sh checks; section "Second review" below). **Simulator only, nothing ran on
hardware.**

All numbers are from the host simulator on the Mac (none from pgx93). Sim counts are 12.5 ns (pwmMax 2047 at 39 kHz);
tables are converted to ns. Data and tools: `doc/research/rect-cal-late-knee/` (`data/*.tsv`, `tools/`).

## Method

**Plants.** `SIM_RING` presets fry, fry-hq, flat, flat-hq (`test/host-stub/plant_boards.h`, fitted to the SPICE
half-bridge bench: fry 76 µH / ring 233 kHz, flat 51 µH / 320 kHz; `-hq` = higher ring Q). Scenario `rcal-pv`
(new): PV panel `SIM_PV="isc,voc,k"`, battery 26.8 V, `rect-cal force`, ADC rate 513 Hz (fry) / 1469 Hz (flat).

**Matrix.** Voc 58 / 68 / 79.6 V (calibration Vin about 2.0-2.9 x Vout), Isc 3 / 6 A, fill factor 0.72 / 0.80,
noise seeds 1-3: 36 runs per board. Low-Voc matrix: Voc 40 / 48 V (Vin 1.4-1.7 x Vout), seeds 1-3, 24 per board.

**Truth, two definitions.**

1. *Timing truth = 0.* The plant has no driver or gate delay: dead time leaves the LS-off edge where it is and shifts
   the crossing only about -5 ns (Vf/Vo). The offset rect-cal is meant to learn (delays between commanded and real
   LS turn-off) is therefore 0 on this plant. "learned vs 0" below is the error against this truth.
2. *Cal-point truth* = plant zero crossing at the calibration HS minus the ideal window `(Vi/Vo - 1)·HS`. Measured by
   `plantZc()` in mppt-sim: copy of the running plant, stiff sources at the mean Vin/Vout of the calibration steps,
   600 cycles per LS count, first +/- sign change of the coil current at LS turn-off (`VirtualConverter::getILsOff`).
   It is not 0 because HS turns on into the ring: the ring's coil current at HS turn-on (up to about Vo/Z0) shifts the
   crossing by up to `±sqrt(LC)` (about ±0.68 µs fry, ±0.5 µs flat), with a phase that depends on the off-time, so it
   varies per operating point and no constant offset can follow it.

The LS operating rule in DCM is not the ideal window but the worst-case ratio `M/0.980198` (2 % margin), i.e. the LS
turns off `0.0198·t_HS/M` early (about 507 ns at D = M, about 279 ns at the calibration point 0.55·M).

## Results

### Main matrix (Voc 58-79.6), learned against the timing truth 0 [ns]

| board | n done | cal-point truth mean [range] | main learned mean ± sd | port/final learned mean ± sd |
|---|---|---|---|---|
| fry | 35 | +305 [-306..+652] | +657 ± 453 | +247 ± 232 |
| fry-hq | 35 | +357 [-130..+752] | +648 ± 390 | +342 ± 153 |
| flat | 35 | -31 [-298..+290] | -16 ± 438 | -291 ± 223 |
| flat-hq | 32 | +20 [-431..+438] | 0 ± 459 | -261 ± 239 |

Learned against the cal-point truth (learned - truth, mean ± sd):

| board | main | port = final |
|---|---|---|
| fry | +352 ± 201 | -58 ± 310 |
| fry-hq | +291 ± 135 | -15 ± 186 |
| flat | +15 ± 250 | -261 ± 231 |
| flat-hq | -21 ± 214 | -281 ± 177 |

Applied offset range [ns] and bound count:

| board | main | port | final |
|---|---|---|---|
| fry | [-375..0], 30 bounds | [-375..0], 22 | [-375..+112], 18 |
| fry-hq | [-212..0], 33 | [-212..0], 32 | [-212..+112], 27 |
| flat | [-1250..0], 12 | [-1250..0], 3 | [-1250..+38], 0 |
| flat-hq | [-1000..0], 14 | [-1000..-25], 0 | [-1000..-25], 0 |

The port and the final rule learn the same values; they differ only in what is applied for late results.

### Low-Voc matrix (Voc 40/48, Vin 1.4-1.7 x Vout)

| board | n done main / port | learned - truth, main | learned - truth, port = final | final applied |
|---|---|---|---|---|
| fry | 15 / 17 | +48 ± 166 | +25 ± 160 | [-825..+125] |
| fry-hq | 15 / 18 | +79 ± 154 | +63 ± 150 | [-662..+88] |
| flat | 20 / 20 | +533 ± 590, max +1498 | -193 ± 182 | [-538..0] |
| flat-hq | 21 / 21 | +368 ± 562, max +1544 | -197 ± 151 | [-650..0] |

Main's flat knees land up to 1.5 µs late at low Vin/Vo: with a negative ring current at HS turn-on the output current
bends *up* at the crossing and the down-only fit finds the later clamp onset. The port's up-bend fit removes that.
Failures (V_step, LS_headroom, Iout_follows_LS) are the same in all variants; main additionally fails 2-3 runs per fry
preset with no_knee (fry 9 vs 7, fry-hq 9 vs 6).

### Bias and spread, summary

Against the timing truth (0), per board, high-Vin / low-Vin matrix (port = final):

- fry: +247 ± 232 (high Vin), +71 ± 291 (low Vin). Main: +657 ± 453 / +135 ± 361.
- fry-hq: +342 ± 153 / +79 ± 295. Main: +648 ± 390 / +158 ± 370.
- flat: -291 ± 223 / -168 ± 185. Main: -16 ± 438 / +559 ± 575.
- flat-hq: -261 ± 239 / -167 ± 252. Main: 0 ± 459 / +398 ± 585.

The port halves the spread. Residual bias: fry late by +250-340 ns at Vi > 2Vo (knee-vs-crossing lag of the plant,
below), flat early by 170-290 ns (charge zc late by the L mismatch, below).

### Where the residual bias comes from

- *Knee - plant crossing* (main fit): fry +340 ns mean (sd 130-270); flat -22..+82 ns. The fry lag above 2Vo is the
  node-clamp lag of the plant (reverse current lifting the node to Vin before the loss). SPICE B shows knee - zc of
  only -14..+132 ns, so most of the fry lag is **probably a plant artefact** (inferred, not proven).
- *Charge zc - plant crossing*: fry -40..-102 ns (Voc >= 58), -98..-178 ns (low Voc): the ring returns about C·Vo
  per period, which the charge model cannot subtract. flat +135..+254 ns: model L = 0.95·56 µH = 53.2 µH vs plant
  51 µH (+4.3 % -> zc +2.1 % of the window). sd 8-18 ns: the charge-based zc tracks the plant crossing closely.
- zc-map on stiff plants (4103 / 39 kHz): plant zc - ideal ranges -308..+686 ns fry, -385..+256 ns flat; the ideal
  plant (no ring) gives exactly 0.

### HS on-time extension (paired runs, `tHsExt` 0 vs 80 ns, n 12-16, high Voc)

| | fry | flat |
|---|---|---|
| truth shift (≈ τ·Vi/Vo) | +122 | +210 |
| learned shift, main | +41 | +199 |
| learned shift, port/final | +179 | +135 |

sd about 220 ns per run, so the shifts are uncertain to about ±60 ns. Per the derivation, the port should learn about
τ = 80 ns of the τ·Vi/Vo shift (LS ends early, the safe side); the measured values are within noise of that.

## Late-knee decision

Options: apply late results (unbounded), refuse them (main: a positive result is a bound, applied min(0, active)),
or apply them bounded. Decision after review: **bounded at LateMaxNs = 125 ns, only from calibration points below
Vin = 2 Vout, and never into the complementary clamp.**

- `SynchronousConverter::computePwmRectMax`: a positive offset is applied only as far as it leaves the LS off for
  `T/32` (128 ct at 4103, 0.8 µs at 39 kHz) plus the drain `off/ratio` before the next HS turn-on, not at all where the
  window without it already ends later, and not at all where `ratio < 0.25` (buck M_WCE > 0.8):
  `ls = ratio < 0.25 ? base : min(base + off, max(base, T - HS - 1 - T/32 - off/ratio))` (second review; the first
  fix had the T/32 gap only).
- A rect-cal result whose applied offset is ≤ +125 ns from a calibration point below 2 Vout is a measurement
  (`rect_offset_cal=1`). A later one, or any positive one above 2 Vout, is a bound (`rect_offset_cal=2`): applied
  min(limit, active) with limit +125 ns below and 0 above 2 Vout; a measured shorter offset is kept. Bounds are retried
  weekly, and only below 2 Vout.
- `coil.conf` `rect_offset_ns` above +125 ns is loaded as +125 ns with `rect_offset_cal=2`; `measure-coil ls apply`
  clamps to +125 ns and stores 2 when it clamped.
- The runtime backstop is unchanged (a positive offset drops to 0 on the collapse signature).

Rationale: driver/gate delays are tens of ns; one run scatters by ~0.2 µs; the plant clamp lag above 2 Vout is
hundreds of ns. The gain of a late correction is ~1 mW (body diode on the last 0.5 µs), the cost of a wrong one
11-55 mW, so late results are taken only where the knee has no node-lift lag (below 2 Vout). The 2 Vout rule was chosen
over "two agreeing runs" because it needs no stored history and matches the physical mechanism of the lag.

### Clamp band near D = M (review finding)

With the earlier rule `ls = min(base + off, T - HS - 1)` a Vout reading 1.5-2 % low (it uses up the 2 % WCE margin) let
a positive offset reach the complementary limit for HS in about `[M·(T - off), M·T]`: LS off only at HS on, forced CCM
below D = M, negative mean current. The first hazard sweep stepped HS by 23 counts and missed this band.

`tools/band.cpp <board> <off_ns> <vout_err> [old|new]`: stiff plant, 4103 ct / 39 kHz, Vout 26.8 / 29.2 V, Vin 31-80 V,
every HS count in the 60 below M·T, WCE window from the biased reading, LS min pulse 320 ct (points where it sets the
LS skipped). "into clamp": the offset, not the reading, reaches T - HS - 1; "made Iout<0": mean Iout < 0 where offset 0
gives ≥ 0; "drop": largest Iout loss against offset 0. Full output: `data/band-sweep.txt`, `data/band-sweep-hq.txt`.

| board | offset | Vout err | old: into clamp / made Iout<0 / min Iout | new (T/32): into clamp / made Iout<0 / max drop |
|---|---|---|---|---|
| fry | +63 | -1.7 % | 0 / 352 / -0.18 A | 0 / 0 / 0.053 A |
| | +63 | -2.0 % | 540 / 1359 / -0.83 A | 0 / 0 / 0.055 A |
| | +125 | -1.0 % | 0 / 0 / - (drop 2.65 A) | 0 / 0 / 0.078 A |
| | +125 | -1.5 % | 80 / 699 / -0.34 A | 0 / 0 / 0.085 A |
| | +125 | -1.7 % | 520 / 1874 / -0.71 A | 0 / 0 / 0.090 A |
| | +125 | -2.0 % | 1071 / 2271 / -1.46 A | 0 / 0 / 0.093 A |
| | +250 | -1.0 % | 49 / 571 / -0.29 A | 0 / 0 / 0.133 A |
| | +250 | -2.0 % | 2140 / 3594 / -5.56 A | 0 / 0 / 0.154 A |
| flat | +63 | -2.0 % | 540 / 1341 / -1.57 A | 0 / 0 / 0.031 A |
| | +125 | -1.5 % | 80 / 466 / -0.33 A | 0 / 0 / 0.046 A |
| | +125 | -1.7 % | 520 / 1641 / -1.28 A | 0 / 0 / 0.049 A |
| | +125 | -2.0 % | 1071 / 2346 / -2.36 A | 0 / 0 / 0.051 A |
| | +250 | -1.0 % | 49 / 313 / -0.24 A | 0 / 0 / 0.066 A |
| | +250 | -2.0 % | 2140 / 3760 / -4.74 A | 0 / 0 / 0.082 A |

New rule, all of fry, fry-hq, flat, flat-hq × offsets 0 / +63 / +125 / +250 × Vout err 0 / -0.5 / -1 / -1.5 / -1.7 / -2 %:
0 points into the clamp, 0 points with the mean current turned negative. Offset 0 is the same in both rules (0 / 0).
Gap choice: an `off`-count gap (the reviewer's first proposal) still left 505 negative points on flat (+125, -1.7 %,
min -0.34 A); 40 / 60 counts left 188-418 / 31-88; 82 counts (2 %) left 5 points (fry -1.5 %, -0.05 A); 103 and 128
counts left none. T/32 = 128 ct has headroom over 103 and is a shift.

`test/host-stub/rect-cal-test.cpp` `testLateGap` checks the rule on `SynchronousConverter` (Vin 60, Vout read 1.7 %
low, HS over the 150 counts below M·T, offset +125 ns): no point into the clamp, idle ≥ T/32 wherever the offset acts.
With the rule disabled it fails (2 points into the clamp, 23 closer than T/32).

### Hazard numbers (stiff plant, field timing 4103 / 39 kHz, LS min pulse 320 ct, Vout 26.8 / 29.2 V, Vin 31-80 V)

`tools/hazard.cpp <board> <offset_ns> [wce|ideal] [-] [step]`, firmware rule (T/32 gap), every HS count, all DCM points
where the offset acts. `data/hazard-gap.txt`. These replace the earlier 23-count table.

| case | offset | fry | flat |
|---|---|---|---|
| WCE LS, exact sensors | 0 | 0.07 A at LS off | 0 A |
| | +125 | 0.14 A (0.72 µJ/cycle, 28 mW circulating), ≤ 14 mW extra loss | 0.08 A (0.18 µJ, 7 mW), ≤ 11 mW |
| | +250 | 0.19 A (1.4 µJ), ≤ 16 mW | 0.19 A (0.9 µJ), ≤ 12 mW |
| | +375 (7-count steps) | 0.24 A, ≤ 31 mW | 0.27 A, ≤ 25 mW |
| | +500 (7-count steps) | 0.29 A, ≤ 67 mW | 0.34 A, ≤ 47 mW |
| LS = ideal + offset (margin used up) | 0 | 0.25 A | 0.28 A |
| | +125 | 0.30 A (3.4 µJ), ≤ 46 mW | 0.35 A (3.1 µJ), ≤ 55 mW |
| | +250 | 0.35 A, ≤ 98 mW | 0.42 A, ≤ 117 mW |

The coil current at HS turn-on stays at its offset-0 value (-0.25..-0.36 A, the ring) for all offsets: no carry-over.
Without the gap rule the earlier 23-count sweep showed carry-over from +375 ns and runaway (2.6-3.9 A) at +500 ns;
with it +500 ns stays below 0.35 A. The +125 ns cap now limits loss, the gap limits the hazard.

**Known-bad checks:** LateMaxNs = 500 fails `rcal-vin.sh` (28 runs outside the band, earlier version); main fails its
sd criterion; a +400 ns learned bias (`l += pwmCountsFromNs(400)` in a scratch build) fails the new bias checks:
pooled mean +411 ns (> 200), fry +596 / fry-hq +691 ns (> 400), fry 100 % / fry-hq 95 % bounds (> 90 %)
(`data/rcal-vin-bias400.txt`). The disabled gap rule fails `testLateGap`.

**Known limitation:** the applied value is not monotone in the result: a result just under the cap applies up to
+125 ns, a later one (bound) applies min(+125, active), which is 0 with an active 0. The non-monotone side is the
safe one. A positive measurement (below 2 Vout) is not re-run; the gap rule bounds its hazard.

### Final matrices with the review fixes (3 seeds; `data/m-fin2*.tsv`)

Learned values are identical to the port. Applied: Voc 58-79.6 (above 2 Vout) at most 0 on every board
(fry [-375..0], fry-hq [-212..0], flat [-1250..0], flat-hq [-1000..-25]); Voc 40/48 (below) fry [-825..+125],
fry-hq [-662..+88], flat [-538..0], flat-hq [-650..0].

## Tests run (worktree, Mac)

- Host: rect-cal-test (late-knee fixtures below 2 Vout: +35 -> +15 ct, +80 -> +20 ct clamped; above: +35 -> 0
  clamped; manual +40 above 2 Vout -> 0, level 2; coil.conf 875 ns -> 20 ct level 2, 100 ns -> 16 ct level 1;
  `testLateGap`), vconv-test 1309/1309, dcm-current, coil-fit, izero 511/511, mcpwm-timing, vconv-energy-audit and
  the other buildable host tests: pass. Pre-existing (also on main e4cb0616): converter-test does not build
  (logging.h); conf-hash, integrator, ripple-freq, scope have no build line.
- `vconv-spice-check.sh`: PASS (its self-tests fail as designed).
- mppt-sim rcal, rcal-bus, rcal-cloud, rcal-flat, rcal-low, rcal-low-bus, rcal-ov, rcal-sun: no FAIL; rcal-sun learns
  +15 ct above 2 Vout -> bound, applied 0; rcal-flat learns -22 / applies -32 ct.
- `test/host-stub/rcal-vin.sh` (192 runs, 4 seeds): 0 outside the band, 22 failed runs (offset kept); baselines and
  known-bads in "Second review".
- esp32s3 firmware build (IDF 6.0.3): pass, 11 % app partition free; also with `CONFIG_FUGU_WITH_MEASURE_COIL=y`.
- Docs: `npm run build` pass.

## Not done / limits

- Not on hardware. LS-side delays (driver/gate delay at LS off) are tested only in rect-cal-test fixtures; the plant
  has none.
- No pgx93 offload: runs were small (≤ 144 runs), local -j4.
- The fry knee lag above 2Vo is probably a plant artefact (SPICE knee - zc much smaller); if the field shows it, fry
  runs above 2 Vout end as bounds (rcal-vin.sh: 55-85 % of fry runs).
- rcal-vin.sh baselines are Mac numbers; the cross-machine margin is estimated from seed realisations (not measured on
  pgx93). A plant or rect-cal change that moves a board mean by > 150 ns needs new baselines.
- The band sweeps skip points where the LS min pulse sets the LS and HS above pwmCtrlMax, and use stiff sources.
- flat's -170..-290 ns bias comes from the preset's L0 vs plant L; on hardware the same ε/2 rule applies to the
  coil.conf L0 error.

## Second review (high M, load clamp units, rcal-vin.sh baselines)

### Reverse current that has not drained (high M)

The T/32 gap only covers the clamp band. After a late LS turn-off the reverse current drains through the HS body diode
at (Vi - Vo)/L, i.e. in `t_late·M/(1 - M)` = `t_late/ratio` (boost: the same expression with its ratio). At Vin =
1.1-1.2 Vout that is 5-10x the late time, so with Vout read 1.7 % low a +125 ns offset still turned the mean current
negative 120-130 counts below M·T (outside the 60-count window of the first band sweep): fry 253 points, -0.23 A.

Rules screened with the full-range probe (`tools/band.cpp`, 1-count HS steps in the last 600 counts below M·T, 4-count
steps below; `data/band-wide-rules.txt`), points where the offset turned the mean current negative:

| rule | fry +125, Vo -2 / -2.5 % | fry +250, -2 / -2.5 % | flat +125, -2.5 % | flat +250, -2 / -2.5 % |
|---|---|---|---|---|
| T/32 + off/ratio | 72 / 441 | 34 / 417 | 132 | 0 / 66 |
| T/32, no offset at M_WCE > 0.8 | 0 / 192 | 191 / 759 | 71 | 70 / 555 |
| T/32, no offset at M_WCE > 0.75 | 0 / 21 | 20 / 334 | 0 | 0 / 224 |
| T/32 + off/ratio, none at M_WCE > 0.8 (chosen) | 0 / 0 | 0 / 0 | 0 | 0 / 0 |

Full check of the chosen rule (`data/band-wide-fw.txt`): fry, fry-hq, flat, flat-hq × +63 / +125 / +250 ns × Vout read
0 / -0.5 / -1 / -1.5 / -1.7 / -2 / -2.5 % low and Vin read +1 / +2 / +2.5 % high, every HS count from the LS min pulse
to M·T (1-count steps), 85 000-168 000 points per case: 0 points into the clamp, 0 points where the offset turns the
mean current negative. Largest loss of mean output current against offset 0 (all plants):

| reading error | +63 ns | +125 ns | +250 ns | offset 0 itself: points with Iout < 0, min |
|---|---|---|---|---|
| none | 0.016 A | 0.027 A | 0.043 A | 0 |
| Vout -1 % | 0.037 | 0.059 | 0.084 | 0 |
| Vout -1.5 % | 0.060 | 0.094 | 0.137 | fry 170 / fry-hq 184, -0.13 A |
| Vout -1.7 % | 0.076 | 0.117 | 0.170 | fry 362 / fry-hq 373, -0.21 A |
| Vout -2 % | 0.111 | 0.166 | 0.230 | 638-1532 per plant, -0.37..-0.42 A |
| Vout -2.5 % | 0.283 | 0.416 | 0.506 | 3796-4889, -1.7..-2.4 A |
| Vin +2 % | 0.106 | 0.157 | 0.224 | 349-1188, -0.22..-0.35 A |
| Vin +2.5 % | 0.250 | 0.373 | 0.454 | 3418-4516, -1.4..-2.3 A |

**Out-of-scope finding (offset 0):** on the ring plants the 2 % WCE window itself drives the mean output current
negative near D = M from a Vout reading 1.5 % low (fry) and on all plants at 2 % low. The ring current at HS turn-on moves
the zero crossing by up to ±sqrt(LC), which the 2 % margin does not cover at high M. This is the existing diode-emulation
window, not the offset; worth a separate look given the known 1.1-1.7 % low Vout reading of one board while converting.

Hazard table with the chosen rule (`data/hazard-final.txt`, 1-count steps): +125 ns WCE exact sensors fry 0.14 A /
14 mW, flat 0.055 A / 11 mW; +250 ns fry 0.19 A, flat 0.15 A; margin used up (LS = ideal + off): unchanged from the
table above (0.30-0.35 A, ≤ 55 mW at +125 ns), the worst points are below M = 0.8.

`testLateGap` adds a DCM case at Vin 34 V, Vout read 29.5 V (M_WCE 0.885): the offset must not act on any of the 122
DCM points. With the M limit removed it fails (52 points).

### Load clamp in counts

`rect_offset_ns` is compared with the limit in counts at the active tick rate: a saved value is the ns of whole counts
(LEDC 35 kHz: 125 ns -> 9 ct -> 126 ns), which compared in ns turned a measurement into a bound with a warning each boot.
rect-cal-test: 128 ns at 160 MHz (20 ct) loads as 20 ct, level 1.

### rcal-vin.sh checks against baselines

Seeds default to 4 (192 runs). Per board the learned mean must stay within 150 ns of its baseline (Mac, 4 seeds: fry
+162, fry-hq +234, flat -231, flat-hq -190 ns), the pooled mean within 100 ns of -9 ns. Seeds 1-2 vs 3-4 move a board
mean by 4-104 ns (fry-hq most), the pooled mean by about 50 ns, so a 4-seed mean on another machine should stay within
about ±75 ns per board. The bound share is printed only (fry-hq 74 %).

| build | fry | fry-hq | flat | flat-hq | pooled | result |
|---|---|---|---|---|---|---|
| final | +162 | +234 | -231 | -190 | -9 | pass |
| +200 ns learned bias | +362 | +435 | -31 | +10 | +192 | fail on every board and pooled |
| +400 ns learned bias | +562 | +635 | +170 | +210 | +392 | fail on every board and pooled |

Not run on pgx93: mirroring the worktree (with the cloned docs node_modules) exceeded 30 min and was stopped, so the
cross-machine margin is estimated from seed realisations, not measured.
