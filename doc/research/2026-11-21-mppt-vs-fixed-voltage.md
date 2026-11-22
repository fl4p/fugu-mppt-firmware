*this document is an LLM generated placeholder*

# MPPT vs fixed input voltage on fry and flat

How much energy does the tracker win over a cheap charger that holds the panels at one fixed input voltage, or at a
fixed fraction of a periodically measured open-circuit voltage (k·Voc)? Analysis of 30 days of field data,
2026-10-22 .. 2026-11-21 (late autumn, Portugal). Scripts and raw output: `2026-11-21-mppt-vs-fixed-voltage/`.

## Results

E(strategy) / E(MPPT as harvested), battery-limited periods excluded. "Ideal MPP" is the power at the global maximum of
each measured P-V curve.

| strategy | fry setting | fry | flat setting | flat |
|---|---|---|---|---|
| ideal MPP (perfect tracker) | | 1.025 | | 1.027 |
| MPPT, this firmware | | **1.000** | | **1.000** |
| best single fixed V (hindsight) | 64.5 V | 0.985 | 56.0 V | 1.000 |
| best fixed V re-chosen every day (hindsight) | 59-67.5 V | 0.991 | 54-58.5 V | 1.008 |
| 0.80 × Voc_nom | 62.5 V | 0.977 | 55.9 V | 1.000 |
| 0.78 × Voc_nom | 61.0 V | 0.966 | 54.5 V | 0.993 |
| 0.76 × Voc_nom | 59.4 V | 0.955 | 53.1 V | 0.983 |
| k·Voc, best k | k = 0.84 | 0.988 | k = 0.81 | 1.002 |
| k·Voc, k = 0.85 | | 0.986 | | 0.978 |
| k·Voc, k = 0.80 | | 0.973 | | 1.001 |
| k·Voc, k = 0.75 | | 0.943 | | 0.969 |
| k·Voc, k = 0.70 | | 0.900 | | 0.916 |

Voc_nom is the median measured Voc of the bright curves (Pmax above half the 95th percentile): 78.2 V on fry and 69.9 V
on flat. There is no panel datasheet in the repo, so this stands in for the STC rating.

- **fry:** the tracker harvested 1.5 % more than the best fixed voltage chosen in hindsight, 2.3 % more than
  0.80 × Voc, and 4.7 % more than 0.76 × Voc. A k·Voc tracker at k = 0.84-0.85 comes within 1.2-1.4 %.
- **flat:** a well-chosen fixed 56 V (= 0.80 × Voc) or k·Voc with k = 0.80-0.81 would have matched the tracker. At
  0.76 × Voc the tracker is 1.7 % ahead.
- **Headroom:** both trackers left about 2.5 % on the table against the ideal MPP. The cost splits into roughly 2 %
  operating-point error and 0.5 % for the sweeps (see Tracking efficiency). On flat, all of the fixed-voltage handicap
  is used up by that loss.

Measured against the ideal MPP instead of the harvest, the fixed voltage loses 3.9 % on fry (0.961) and 2.6 % on flat
(0.974). Both ratios against the fixed voltage and against k are plotted in `2026-11-21-mppt-vs-fixed-voltage/ratio_vs_v.png`.

## Where the fixed-voltage loss comes from

Loss is 1 − E(best fixed V)/E(ideal MPP) inside each category, at the device's global best V. "Loss share" is the
category's part of the total fixed-V loss. Categories overlap across groups. "own best V" is the best fixed V for that
category alone.

### fry

| category | energy share | fixed-V ratio | loss share | own best V | k·Voc (k=0.84) | MPPT η_op |
|---|---|---|---|---|---|---|
| clear days (7) | 48 % | 0.968 | 40 % | 64.5 V | 0.966 | 0.986 |
| mixed days | 45 % | 0.960 | 47 % | 65.0 V | 0.964 | 0.976 |
| overcast days | 7 % | 0.926 | 14 % | 63.0 V | 0.955 | 0.972 |
| unshaded curve | 77 % | 0.984 | 31 % | 64.5 V | 0.984 | 0.983 |
| partially shaded curve | 23 % | 0.882 | 69 % | 67.5 V | 0.898 | 0.972 |
| dawn/dusk (first/last 90 min) | 11 % | 0.867 | 38 % | 60.5 V | 0.895 | 0.957 |
| core day | 89 % | 0.973 | 62 % | 65.0 V | 0.973 | 0.984 |
| before 13 UTC | 69 % | 0.967 | 59 % | 65.0 V | 0.971 | |
| 13 UTC and later | 31 % | 0.949 | 41 % | 64.5 V | 0.949 | |
| unshaded, core day, clear | 36 % | 0.988 | 11 % | 64.5 V | 0.983 | |

Partial shading dominates on fry. Shaded curves carry 23 % of the energy but 69 % of the fixed-voltage loss: a fixed
voltage gets 88 % of the ideal there, while the tracker gets 97 %. Shaded curves cluster at 15-16 UTC (35-55 % of
curves), but also at 08 UTC (50 %). In the afternoon the median peak drops from about 64.5 V to 62 V at 15 UTC and
59 V at 16 UTC, and individual curves have their global peak at 32-35 V. Within the 90-minute dawn/dusk windows a
fixed voltage gets 87 % of the ideal and the tracker 96 %. These windows overlap with the shading.

### flat

| category | energy share | fixed-V ratio | loss share | own best V | k·Voc (k=0.81) | MPPT η_op |
|---|---|---|---|---|---|---|
| clear days (7) | 46 % | 0.977 | 42 % | 56.0 V | 0.978 | 0.983 |
| mixed days | 47 % | 0.973 | 49 % | 56.5 V | 0.974 | 0.975 |
| overcast days | 7 % | 0.965 | 10 % | 57.5 V | 0.964 | 0.974 |
| unshaded curve | 93 % | 0.975 | 89 % | 56.5 V | 0.976 | 0.979 |
| partially shaded curve | 7 % | 0.961 | 11 % | 55.0 V | 0.965 | 0.964 |
| dawn/dusk (first/last 90 min) | 13 % | 0.951 | 25 % | 57.5 V | 0.953 | 0.975 |
| core day | 87 % | 0.977 | 75 % | 56.0 V | 0.978 | 0.979 |
| unshaded, core day, clear | 38 % | 0.980 | 29 % | 56.5 V | 0.981 | |

flat is barely shaded: 70 of 426 curves, 7 % of the energy. Its fixed-voltage loss is spread evenly across sky
conditions. It comes from the Vmpp scatter on ordinary unshaded curves, which is ±2.4 V (see Temperature).

### Sky condition

Clear days lose least and overcast days most, but the spread is small: on fry 3.2 % (clear) vs 7.4 % (overcast), on
flat 2.3 % vs 3.5 %. Overcast days carry only 7 % of the energy on each device, so they contribute 10-14 % of the
loss.

### Temperature

The subset here is bright (Pmax above 30 % of the 95th percentile), unshaded, single-peak curves: 122 on fry and 162
on flat.

| | fry | flat |
|---|---|---|
| Vmpp | 64.9 ± 2.1 V | 56.7 ± 2.4 V |
| Voc | 77.9 ± 1.1 V | 69.6 ± 0.9 V |
| Vmpp/Voc | 0.834 ± 0.025 | 0.813 ± 0.032 |
| fit Vmpp = a + b·Voc + c·ln(Pmax) | −5.7 + 1.20·Voc − 3.97·ln P, R² 0.57 | 3.8 + 1.18·Voc − 4.92·ln P, R² 0.77 |
| Vmpp scatter from the Voc term / ln P term | 1.4 V / 1.6 V | 1.1 V / 1.9 V |

- **Temperature drift was small in this period.** Voc moved only ±1.1 V (±1.4 %), and the weekly medians were flat
  (fry 76.8-78.2 V, flat 69.0-70.6 V). At about −0.3 %/K, that is a cell-temperature spread of about ±5 K.
- **Irradiance moves Vmpp at least as much as Voc does.** At higher power, Vmpp falls (series resistance, cell
  heating). This is also why k·Voc does no better than a fixed voltage on bright unshaded curves (fry 0.983 vs 0.988,
  flat 0.981 vs 0.980): Voc does not follow the irradiance-driven part.
- **Curvature around the MPP.** Averaged over the subset, s(Vmpp·(1+x)) = P/Pmax is:

  | x | fry | flat |
  |---|---|---|
  | −10 % | 0.933 | 0.916 |
  | −5 % | 0.970 | 0.962 |
  | +5 % | 0.950 | 0.959 |
  | +10 % | 0.754 | 0.833 |

  A summer swing of 30-40 K in cell temperature moves Vmpp by about ±5 % around a year-round setting, which costs
  3-5 % at the extremes and much more on the high-voltage side. The November numbers above therefore understate the
  year-round cost of a fixed voltage. A k·Voc tracker would follow that drift. This is an extrapolation from the
  measured curve shape; there is no summer data in this window.

## Battery-limited periods

These were excluded: 10-s bins where a Vout, Iin, Iout or power limiter was sampled in at least 10 % of the points
(`cv_lim_idx` ≥ 1), or where the CV/CC/CP mode was seen.

| | fry | flat |
|---|---|---|
| total harvested energy | 26.0 kWh | 44.8 kWh |
| battery/output limited | 1.9 kWh (7.4 %), 15.5 % of daytime bins | 4.7 kWh (10.4 %), 10.9 % of bins |
| unlimited, no valid sweep curve within 40 min | 0.4 kWh | 3.0 kWh (mostly 10-22..10-30) |
| analysed | 23.6 kWh | 37.2 kWh |

Almost all the limited energy is the Vout (CV) limiter (`cv_lim_idx` = 1). flat also sees a little of the power
limiter (idx 4).

## Data

- **InfluxDB** `ha_van`/`mppt`, 2026-10-22 03:17 .. 2026-11-21 10:15 UTC. `P`, `Ui` and `pwm_duty` come at about
  50 Hz (4 M points a day per device until 11-16, about 1.7 M after). `mppt_state` and `cv_lim_idx` come at about
  1 Hz.
  - Energy bins: 10-s means of P and Ui, plus the mode/limiter statistics (`fetch_bins.py`).
  - Sweeps: every window with `mppt_state` = 5, fetched raw. The ramp from duty 0 to maximum duty gives a full P-V
    curve of 400-600 points from Voc down to about 28 V (`fetch_sweeps.py`, `curves.py`). A curve is valid if Pmax
    > 3 W, the ramp starts at duty 0, and it reaches below 0.65 Voc.
  - Result: fry 480 valid curves (201 with two or more peaks, 213 shaded by the rule below), flat 426 valid curves
    (57 multi-peak, 70 shaded).
- **Console log** on havan, `~/pv/fugu_console.log*`, 11-07 .. 11-21. It was used only for validation:
  - 63 distinct `sc:` curves (from the dashboard's `sweep-curve` polls, 11-17 .. 11-21; 31 fry, 32 flat);
  - 113 `scan:` peak-scan results (11-19 .. 11-21).
  - The log also has an ASCII P-V and P-D plot after every sweep (`Grouping … V points` and the box-drawing lines). The
    Influx curves made it unnecessary to digitise them.
- **Exclusions:**
  - flat 2026-11-12 19:30 .. 11-13 08:50 UTC: a simulated converter ran under the name flat (constant Voc 50.0 V,
    108 W around the clock).
  - Bins outside 06-19 UTC.

## Method

1. For each 10-s bin take the latest valid curve that ended before it, up to 40 min old (else the next one within
   15 min). Normalise the curve to s(V) = P(V)/Pmax. The curve shape is assumed to hold until the next sweep, with
   power scaling with irradiance.
2. The available MPP power of the bin is P_mpp = P / s(Ui). The floor of s at 0.8 limits the error from stale curves.
3. A fixed-V charger gets P_mpp · s(Vfix). A k·Voc charger gets P_mpp · s(k · Voc_curve). Here Voc comes from the
   latest sweep (at most 30-40 min old), which is slower than a "measure Voc every few minutes" design.
4. E(fixed)/E(ideal) = Σ P_mpp·s(Vfix) / Σ P_mpp. Then E(fixed)/E(harvested) = E(fixed)/E(ideal) / η.
5. η = η_op × (1 − sweep cost):
   - η_op is the P-weighted s(Ui) over bins 1-10 min after a sweep, outside sweeps;
   - the sweep cost is the power dip during sweep bins against the median of the neighbouring bins.
6. Shaded means the curve has two or more peaks (prominence ≥ 6 % and height ≥ 30 % of Pmax), or its global peak is
   below 0.75 Voc.
7. Day classes use both devices' summed power:
   - overcast: daily peak of the 30-min mean below 0.4 of a 15-day rolling maximum;
   - clear: peak at least 0.7 and roughness (Σ|2nd difference| / Σ of 5-min means, 08-16 UTC) at most 0.25;
   - mixed: everything else.

   Clear days: 10-25, 10-28, 11-04, 11-05, 11-06, 11-16, 11-19.
8. Dawn/dusk is the first or last 90 min of each day's production (P > 2 W).

### Tracking efficiency

| | fry | flat |
|---|---|---|
| η_op, fresh curves | 0.981 | 0.978 |
| sweep cost | 0.50 % | 0.49 % |
| η | 0.976 | 0.973 |
| η including stale curves (all ages) | 0.973 | 0.971 |

η_op falls with curve age, from 0.986 (0-3 min) to 0.965 (20-45 min). That decline is mostly the curve going stale
rather than the tracker drifting, so the fresh value is used. With the all-ages η, the fixed-V ratios rise by about
0.3 %.

On bright, unshaded, fresh bins, the operating point sits on the curve's Vmpp: fry median offset 0.0 V (IQR −0.9 to
+0.7), s_op median 0.996. flat has a tail below the MPP: median −0.2 V, mean −0.6 V, IQR −2.0 to +0.6, s_op median
0.989. The fry tracker loses its 2-4 % at dawn/dusk (η_op 0.957), under shading (0.972) and on overcast days (0.972).
The flat tracker loses about 2 % even on clear days.

## Validation

**Measured vs modelled.** All results rest on measured P-V curves; no single-diode or fill-factor model is used. Two
things are assumed rather than measured:

- the curve shape persists between sweeps (up to 40 min);
- s(mean Ui of a 10-s bin) is the operating-point efficiency. Because s is concave this overstates η_op slightly,
  since dither loss is not seen, which makes the fixed-V comparison conservative.

**Against the device's own sweep bins (`sc:`).** Matched by sweep end time within 30 s:

| | fry (31 curves) | flat (30 curves) |
|---|---|---|
| Pmax difference, median | 0.2 % | 0.3 % |
| Vmpp difference, median | 0.48 V | 0.25 V |
| mean \|s difference\|, median | 0.010 | 0.012 |

This shows the Influx samples reproduce the device's sweep. It is not independent of the sweep dynamics.

**Against static two-point measurements (`scan:`).** The probe at V2 vs the operating point V0, 10-15 V apart, was
compared with s(V2)/s(V0) on the latest curve (under 30 min old, steady power):

| | fry (66 scans) | flat (42 scans) |
|---|---|---|
| measured mean | 0.878 | 0.801 |
| predicted mean | 0.870 | 0.823 |
| bias | −0.008 | +0.022 |
| MAE | 0.052 | 0.032 |

Shifting flat's curves by up to +2 V in voltage would remove the bias. fry's best shift is 0 to −0.5 V. The per-scan
correlation is near zero because the ratios span only 0.75-0.95 and the curve can be up to 30 min old.

## Caveats

- **Main uncertainty: the absolute tracking efficiency η.** The ratios against the ideal MPP are robust:
  - a Pmax normalisation bias scales s everywhere and cancels;
  - fresh-only curves give the same numbers (fry 0.962, flat 0.973 at the same best V).

  The comparison with the harvest divides by η. A 1 V offset between the dynamic sweep curve and the static P-V curve
  near the MPP would move η, and with it every "vs harvested" number, by about 1 %. flat's `scan:` bias points in
  that direction: the curves are optimistic below the MPP. fry shows no offset. Read the flat result as "fixed
  56 V ≈ MPPT within about ±1.5 %" and the fry result as "MPPT ahead by 1.5 ± 1 %".
- **One season only:** 30 late-autumn days with little temperature range. The year-round cost of a fixed voltage is
  higher (see Temperature); the cost of k·Voc probably is not. A year model calibrated to this month
  (`2026-11-22-mppt-vs-fixed-voltage-year.md`) puts the best fixed voltage about 6 % (fry) / 4.5 % (flat) behind the
  MPPT over a year, and k·Voc 2-3.5 % behind.
- **Output power, not panel power.** P is the converter's output, so conversion efficiency at each V is folded into
  the curves. A fixed-V charger would have its own, similar loss.
- **Gaps:**
  - fry produced little or nothing on several late-October days (10-23, 10-24, 10-26, 10-27, 10-29);
  - flat has 3.0 kWh without a valid curve, mostly 10-22..10-30;
  - 7-10 % of the energy was battery-limited and is excluded. During those periods neither strategy is
    power-limited.
- **k·Voc reuses the sweep's Voc,** up to 40 min old. A real fractional-Voc controller measuring every few minutes
  would track a little better on fast irradiance changes.
- **"Best fixed V" uses hindsight** on this month and this site. The day-to-day best V scatters 59-67.5 V on fry
  (59 V on the cloudy 11-14) and 54-58.5 V on flat.
