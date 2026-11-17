*this document is an LLM generated placeholder*

# rect-cal on the corrected board plants: bias, spread and the late-knee rule

Date: 2026-11-17. Branch `rcal-vin3` (off main e4cb0616): c58ce398 (sim harness), 2aa807ce (port of rcal-vin2),
3ee071b3 (late rule + regression), plus the docs commit. **Simulator only, nothing ran on hardware.**

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
or apply them bounded. Decision: **bounded at LateMaxNs = 125 ns**.

- A result whose applied offset is ≤ +125 ns is applied as a measurement (`rect_offset_cal=1`).
- A later one is a bound (`rect_offset_cal=2`): applied min(+125 ns, active offset); a measured shorter offset is
  kept. Bounds are retried weekly or when Vin < 2Vo.
- The runtime backstop is unchanged (a positive offset drops to 0 on the collapse signature).

Rationale: driver/gate delays are tens of ns and must stay correctable; clamp-lag artefacts on the plant are hundreds
of ns. A positive offset is only a hazard once it eats the 2 % WCE margin near D = M.

### Hazard numbers (stiff plant, field timing 4103 / 39 kHz, LS min pulse 320 ct, Vout 26.8 / 29.2 V, Vin 31-80 V)

`tools/hazard.cpp <board> <offset_ns> [wce]`. Over all DCM points where the offset acts (LS > min pulse):

| case | offset | fry | flat |
|---|---|---|---|
| WCE LS, exact sensors | 0 | 0.07 A | 0 A |
| | +125 | 0.13 A at LS off (0.66 µJ/cycle, 26 mW circulating), ≤ 11 mW extra loss | 0.09 A (0.21 µJ, 8 mW), ≤ 10 mW |
| | +250 | 0.19 A | 0.24 A |
| | +375 | 0.41 A, -0.49 A at HS on (carry-over) | 0.83 A, -0.86 A at HS on |
| | +500 | runaway, 2.6 A | runaway, 3.9 A (near D = M, HS + LS ≈ T) |
| LS = ideal + offset (sensor errors ate the margin) | 0 | 0.25 A | 0.27 A |
| | +125 | 0.29 A (3.3 µJ), ≤ 45 mW extra loss | 0.35 A (3.0 µJ), ≤ 54 mW |
| | +500 | 0.44 A | 0.63 A |

In the margin-eaten case there is no HS-on current growth up to +375 ns, but points at the period limit were skipped
(`HS + LS ≥ pwmMax - 1`), so carry-over there is not covered. The gain from correcting a late LS is small: body-diode
loss at 460 ns early is about 1 mW. The worst cost of the cap (+125 ns wrongly applied) is about 0.13 A reverse
current and ≤ 11 mW (exact sensors) or 0.35 A and ≤ 54 mW (margin consumed); runaway onset is +300..+375 ns.

**Known-bad check:** LateMaxNs = 500 fails `rcal-vin.sh` (28 runs outside the band); main fails the sd ≤ 350 ns
criterion (sd 440-450).

**Known limitation:** the applied value is not monotone in the result: a result just under the cap applies up to
+125 ns, a later one (bound) applies min(+125, active), which is 0 with an active 0. The non-monotone side is the
safe one.

## Tests run (worktree, Mac)

- Host: rect-cal-test (new late-knee fixture: +35 -> applied in (0, 20] ct, +80 -> 20 ct clamped; manual +40 -> 20),
  vconv-test 1309/1309, dcm-current, coil-fit, izero 511/511, mcpwm-timing, vconv-energy-audit, the other buildable
  host tests: pass. Pre-existing (also on main e4cb0616): converter-test does not build (logging.h); conf-hash,
  integrator, ripple-freq, scope have no build line.
- `vconv-spice-check.sh`: PASS (its self-tests fail as designed).
- mppt-sim rcal* scenarios: non-ring scenarios identical to main; rcal-sun learns +15 / applies +5 ct; rcal-flat
  learns -22 / applies -32 ct.
- `test/host-stub/rcal-vin.sh` (96 runs, 2 seeds): 0 outside the band, 13 failed runs (offset kept, ≤ 25 %).
- esp32s3 firmware build (IDF 6.0.3): pass, 11 % app partition free.
- Docs: `npm run build` pass.

## Not done / limits

- Not on hardware. LS-side delays (driver/gate delay at LS off) are tested only in rect-cal-test fixtures; the plant
  has none.
- No pgx93 offload: runs were small (≤ 144 runs), local -j4.
- The fry knee lag above 2Vo is probably a plant artefact (SPICE knee - zc much smaller); if the field shows it, fry
  will hit the +125 ns bound often (final: 18-27 of 35 runs at high Vin).
- flat's -170..-290 ns bias comes from the preset's L0 vs plant L; on hardware the same ε/2 rule applies to the
  coil.conf L0 error.
