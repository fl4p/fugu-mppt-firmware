*this document is an LLM generated placeholder*

# Power recovered by global sweeps (fry, flat, 2026-11-13 .. 11-27)

## Method

`sweep_gap.py` takes every sweep in InfluxDB `sweep` (collector, from 11-27 10:46) and `sweep_ascii` (log backfill,
from 11-13), and from `mppt` telemetry (~50 Hz):

- pre: median P, Vin over the last 30 s of uninterrupted MPPT tracking before the sweep's first state-5 point
- post: median P, Vin over T+20..T+50 s (T = sweep end)
- excluded: a limiter active in either window (`cv_lim_idx`, mostly VoutCTRL at the battery limit), a window not
  entirely in state 4, missing telemetry

Of 574 fry/flat sweeps, 393 are clean (fry 222, flat 171); 102 excluded for a limiter, 47 for missing data.

A wrong-peak event: MPP > 1.10 × pre, post > 1.10 × pre and Vin moved ≥ 5 V. Sweeps with |ΔVin| < 3 V give the
noise floor from changing light over the ~1.5 min between windows: post/pre − 1 is −24 % .. +34 % (5..95 %), median
+0.5 %, so small events are not separable and are not counted.

Energy lost per event: gap = min(post, MPP) − pre, times the time since the previous sweep (capped at 1800 s). The
onset is unknown: the estimate takes half that time, the bound all of it.

## Results

| device | events | lost (estimate) | bound | energy 15 d | share (bound) |
|--------|--------|-----------------|-------|-------------|---------------|
| fry    | 16     | 127 Wh          | 253 Wh | 21.8 kWh   | 0.58 % (1.16 %) |
| flat   | 9      | 59 Wh           | 118 Wh | 30.0 kWh   | 0.20 % (0.39 %) |

- fry loses its peak almost every morning 08:40–08:55 UTC (11-14, -15, -17, -20, -22, -23, -26, -27): 8 events,
  ~78 Wh. Operating point at 70–75 V, the real peak at 44–47 V, a gap of 11–130 W. Same shade, same time each day.
- fry afternoons: 11-14 15:22 (140 W gap, 35 Wh) and 11-16 14:48 (48 W, 12 Wh).
- flat: mostly 07:45–08:00 (small, < 36 W); one large event on 11-13 08:55, stuck at 39 V and 108 W with 339 W at
  56 V (49 Wh, the largest single event).
- Low-light events (< 10 W, fry dawn) are frequent but worth < 1 Wh each.

## Cost of sweeping

Six fry sweeps at ~520 W (11-27, 5-min interval) each cost 3.4–3.6 s of full power (calibration + sweep + landing,
energy over start..T+8 s against the mean of pre and post).

| interval | sweeps/h | full-power time lost |
|----------|----------|----------------------|
| 30 min   | 2        | 0.2 %                |
| 5 min    | 12       | 1.2 %                |

5-min sweeps cost ~1 % more than 30-min ones at all hours, while recovering at most ~5/6 of the 0.2–0.6 % (bound
0.4–1.2 %) lost to wrong peaks. On fry that is about −14 Wh/day for +7 Wh/day; on flat about −20 Wh/day for
+3 Wh/day. A shorter fixed interval is a net loss on both.

## Implications

- The losses come from one repeatable morning shade on fry and a few dawn events: a scan that runs at low power and
  does not discount a lower-voltage peak by the stale `iEnv` would catch them without extra sweeps (see the 11-27
  08:12–08:42 analysis: scan gated by `pMin` = 50 W and by `iEnv`).
- `sweep_interval` should go back to 1800 s on both converters.

## Limits

- Events are only seen where a sweep happened; a wrong peak the tracker left by itself is not counted.
- Onset time is unknown (estimate vs bound above).
- Sweep cost measured on one converter at midday only; at low power the cost in Wh is smaller, but so is the loss.
- 102 limiter-held sweeps are excluded; at the battery limit the MPP is not the reachable power anyway.
- `sweep_ascii` curves are not used for the gap, only the logged MPP and the telemetry.

Files: `sweep_gap.py` (extraction), `sweep_gap.csv` (all sweeps), `sweep_gap_clean.csv`, `sweep_gap_wrongpeak.csv`.
