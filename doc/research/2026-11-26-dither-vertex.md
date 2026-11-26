*this document is an LLM generated placeholder*

# flat ≥300 W operating-point offset from the P&O dither (11-26)

Tests whether the ~22 Wh/day "flat sits 1.5–5 V below Vmpp above 300 W" result of `2026-11-21-mppt-vs-fixed-voltage.md`
is real or an artifact of the sweep curve's voltage axis (that study's `scan:` check found flat's curves +2.2 %
optimistic below the MPP, removed by a +2 V shift; `2026-11-24-mppt-tracking-at-limit.md` left it open).

## Method

`2026-11-26-dither-vertex/dither_slope.py`: 60 s windows of the 48 Hz Influx data (`Ui`, `P`, `pwm_duty`), 1-min mean
in the band, P stddev < 3 %, no sweep, no CV limiter (`cv_lim_idx` unset, since a limiter shapes P(V)). Each duty
plateau is averaged (first 3 samples dropped), then P = b0 + b1·t + b2·(V − Vm) is fitted; the normalised slope
b2/P converts to an offset with the sweep-measured curvature (s = 1 − c·x², c ≈ 15.7 flat, 16 fry). The curvature is
the sweep curve's shape only, not its voltage position, which is what is in doubt. Up to 30 windows a day, 30 days.

Check: `synth_check.py` plants offsets −2..+3 V in synthetic dither with 1 % drift and noise; recovered within ~0.2 V
(slightly low at +3 V). It does not model P&O reacting to light changes.

## Results

| Series | Windows | Days | Median P | Offset, weighted mean | 90 % CI (day bootstrap) | Share > 0 |
|---|---|---|---|---|---|---|
| flat ≥300 W | 495 | 23 | 460 W | −0.13 V | −0.35..+0.06 V | 0.52 |
| flat 100–300 W (control) | 428 | 25 | 169 W | +0.16 V | −0.08..+0.28 V | 0.51 |
| fry ≥300 W (control) | 507 | 22 | 361 W | +0.07 V | +0.02..+0.13 V | 0.57 |

Positive = the maximum lies above the operating point. By band, flat's median is −0.53 V (300–400 W), −0.08 V
(400–500 W), +0.76 V (≥500 W).

flat at ≥300 W sits on its local maximum within a few tenths of a volt, not 1.5–5 V below it. A 0.13 V offset costs
~0.01 %; even the ≥500 W band's +0.76 V costs ~0.3 %. The ~22 Wh/day reading is most likely the sweep-curve
voltage bias, not a tracking loss.

## Caveats

- Dither span per window is only 0.3–0.7 V, so the result is a local slope, scaled by the sweep curvature.
- CV-limited minutes are excluded; any offset there is the limiter's, not the tracker's.
- Correlation between P&O moves and light changes within a window is not modelled beyond a linear time trend.
- The near probes of 11-26 (+2 V ratio 1.033, 1.003, 0.986 at ≥300 W) are consistent with this within their 1.5 %
  reference noise.
