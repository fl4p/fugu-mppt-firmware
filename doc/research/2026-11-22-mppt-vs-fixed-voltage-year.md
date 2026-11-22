*this document is an LLM generated placeholder*

# Fixed input voltage vs MPPT over a year

How much yearly PV energy would a charger that holds the panels at one fixed input voltage lose on fry and flat,
compared with a perfect MPP tracker, this firmware's MPPT as measured, and a fractional-Voc tracker (V = k·Voc)?
The field study `2026-11-21-mppt-vs-fixed-voltage.md` covers 30 late-autumn days only. This study extends it to a whole
year with a weather-driven P-V model calibrated to those 30 days. Scripts and raw output:
`2026-11-22-mppt-vs-fixed-voltage-year/`.

## Results

Main case: generic Portuguese site (Lisbon city centre, PVGIS typical meteorological year), horizontal panels,
close-mount cell temperature, Voc tempco −0.32 %/K. Bands are the range over three mount types × Voc tempco
−0.28/−0.32/−0.35 %/K, plus two structural variants (the non-temperature loss factor and the effective series
resistance, see Model). E is the yearly energy of the strategy, not limited by the battery.

### fry (Voc_nom 78.2 V)

| strategy | setting | E / E(ideal MPP) | E / E(MPPT, this firmware) |
|---|---|---|---|
| ideal MPP | | 1 | 1.025 |
| MPPT, this firmware (November η, summer unmeasured) | | 0.976 | 1 |
| best single fixed V for the year (hindsight) | 57.7 V (56.6-59.7) | **0.916** (0.908-0.930) | **0.939** (0.930-0.953) |
| best fixed V re-chosen every month | 54.7-67.4 V | 0.938 (0.931-0.946) | 0.961 (0.954-0.969) |
| fixed V = November field optimum | 64.5 V | 0.836 (0.802-0.885) | 0.857 (0.821-0.907) |
| fixed 0.80 × Voc_nom | 62.5 V | 0.877 (0.850-0.915) | 0.899 (0.871-0.938) |
| fixed 0.76 × Voc_nom | 59.4 V | 0.912 (0.896-0.930) | 0.934 (0.918-0.953) |
| k·Voc, best k | k = 0.78 | 0.941 (0.940-0.946) | 0.965 (0.963-0.970) |
| k·Voc, k = 0.80 | | 0.937 (0.935-0.946) | 0.960 (0.958-0.969) |
| k·Voc, k = 0.76 | | 0.938 (0.938-0.940) | 0.961 (0.961-0.964) |

### flat (Voc_nom 69.9 V)

| strategy | setting | E / E(ideal MPP) | E / E(MPPT, this firmware) |
|---|---|---|---|
| ideal MPP | | 1 | 1.027 |
| MPPT, this firmware (November η, summer unmeasured) | | 0.973 | 1 |
| best single fixed V for the year (hindsight) | 50.1 V (49.2-51.6) | **0.930** (0.923-0.943) | **0.956** (0.948-0.968) |
| best fixed V re-chosen every month | 47.1-58.7 V | 0.951 (0.947-0.958) | 0.977 (0.973-0.984) |
| fixed V = November field optimum | 56.0 V | 0.866 (0.840-0.904) | 0.889 (0.863-0.929) |
| fixed 0.80 × Voc_nom | 55.9 V | 0.867 (0.842-0.905) | 0.891 (0.865-0.930) |
| fixed 0.76 × Voc_nom | 53.1 V | 0.915 (0.898-0.939) | 0.940 (0.922-0.965) |
| k·Voc, best k | k = 0.75 | 0.951 (0.950-0.958) | 0.977 (0.975-0.985) |
| k·Voc, k = 0.80 | | 0.927 (0.926-0.935) | 0.952 (0.951-0.960) |
| k·Voc, k = 0.76 | | 0.950 (0.949-0.958) | 0.976 (0.975-0.984) |

- **Fixed voltage over a year:** the best single setting loses about 6 % (fry) and 4.5 % (flat) against this
  firmware's MPPT, and 8.4 % / 7.0 % against a perfect tracker. In November alone it was 1.5 % / 0 %.
- **Setting matters more than in November:** the year-optimal setting is 7 V (fry) and 6 V (flat) below the
  November optimum. Keeping the November setting all year loses 14 % / 11 % against the MPPT.
- **Monthly re-tuning** cuts the fixed-voltage loss against the MPPT to 4 % on fry and 2.3 % on flat.
- **k·Voc** loses 3.5 % (fry) and 2.3 % (flat) against the MPPT, and only with k near 0.75-0.78. That is well below the
  k = 0.84/0.81 that was best in November. With k = 0.80 it is no better than a monthly fixed voltage.

## Seasonal drift

Monthly values, main case. Tc and Vmpp are energy-weighted; ratios are against the ideal MPP, with the November field
loss factor applied.

### fry

| month | E share | Tc | Vmpp | best V that month | fixed 57.7 V | fixed, monthly best | k·Voc, k = 0.78 |
|---|---|---|---|---|---|---|---|
| Jan | 4.5 % | 26 °C | 67.5 V | 66.5 V | 0.870 | 0.965 | 0.926 |
| Feb | 5.8 % | 32 °C | 65.0 V | 63.8 V | 0.903 | 0.959 | 0.944 |
| Mar | 8.5 % | 41 °C | 61.3 V | 59.4 V | 0.933 | 0.938 | 0.951 |
| Apr | 8.4 % | 36 °C | 62.9 V | 60.7 V | 0.912 | 0.924 | 0.941 |
| May | 11.0 % | 45 °C | 59.2 V | 56.7 V | 0.922 | 0.924 | 0.940 |
| Jun | 12.0 % | 49 °C | 57.8 V | 55.6 V | 0.921 | 0.928 | 0.940 |
| Jul | 12.6 % | 52 °C | 56.9 V | 54.7 V | 0.915 | 0.930 | 0.937 |
| Aug | 12.0 % | 52 °C | 56.9 V | 54.9 V | 0.923 | 0.937 | 0.942 |
| Sep | 9.5 % | 52 °C | 57.8 V | 55.9 V | 0.930 | 0.936 | 0.950 |
| Oct | 7.1 % | 40 °C | 62.4 V | 60.8 V | 0.932 | 0.950 | 0.951 |
| Nov | 5.3 % | 32 °C | 65.4 V | 64.3 V | 0.898 | 0.962 | 0.939 |
| Dec | 3.3 % | 25 °C | 68.2 V | 67.4 V | 0.860 | 0.968 | 0.916 |

### flat

| month | E share | Tc | Vmpp | best V that month | fixed 50.1 V | fixed, monthly best | k·Voc, k = 0.75 |
|---|---|---|---|---|---|---|---|
| Jan | 4.6 % | 26 °C | 58.6 V | 57.8 V | 0.888 | 0.978 | 0.933 |
| Feb | 5.9 % | 32 °C | 56.3 V | 55.3 V | 0.924 | 0.971 | 0.954 |
| Mar | 8.5 % | 41 °C | 52.8 V | 51.3 V | 0.948 | 0.951 | 0.961 |
| Apr | 8.5 % | 36 °C | 54.3 V | 52.8 V | 0.928 | 0.938 | 0.950 |
| May | 11.0 % | 45 °C | 50.8 V | 49.0 V | 0.935 | 0.937 | 0.949 |
| Jun | 11.9 % | 49 °C | 49.5 V | 47.9 V | 0.933 | 0.941 | 0.949 |
| Jul | 12.5 % | 52 °C | 48.7 V | 47.1 V | 0.927 | 0.943 | 0.946 |
| Aug | 11.9 % | 52 °C | 48.7 V | 47.3 V | 0.933 | 0.949 | 0.952 |
| Sep | 9.4 % | 52 °C | 49.6 V | 48.2 V | 0.942 | 0.949 | 0.960 |
| Oct | 7.1 % | 40 °C | 53.8 V | 52.7 V | 0.950 | 0.963 | 0.962 |
| Nov | 5.3 % | 32 °C | 56.7 V | 55.8 V | 0.919 | 0.975 | 0.949 |
| Dec | 3.4 % | 25 °C | 59.3 V | 58.7 V | 0.877 | 0.981 | 0.922 |

The energy-weighted Vmpp falls by about 11 V (fry) and 10 V (flat) from December to July. Two effects add up:

- **Temperature:** the energy-weighted cell temperature rises from 25 °C to 52 °C, which lowers Vmpp by about
  0.44 %/K.
- **Irradiance:** summer midday irradiance on a horizontal panel is about 2.5 times the November midday value, and
  Vmpp falls with current (see Model).

With the cell temperature held at 34 °C all year, the clean-model fixed-voltage loss shrinks from 6.9 % to 3.5 % on
fry and from 6.7 % to 3.6 % on flat. Temperature and irradiance therefore contribute roughly equal parts.

Even the monthly best fixed voltage loses 5-8 % against the ideal in April to August, because the irradiance-driven Vmpp swing within
each day is largest then. In winter the monthly setting is within 2-4 % of the ideal.

## Sensitivity

One change at a time from the main case. Ratios are against the ideal MPP.

| case | fry best V | fry fixed | fry monthly | fry k·Voc (k) | flat best V | flat fixed | flat monthly | flat k·Voc (k) |
|---|---|---|---|---|---|---|---|---|
| main | 57.7 V | 0.916 | 0.938 | 0.941 (0.78) | 50.1 V | 0.930 | 0.951 | 0.951 (0.75) |
| Évora (inland) | 55.7 V | 0.916 | 0.946 | 0.944 (0.77) | 48.5 V | 0.930 | 0.958 | 0.953 (0.74) |
| tilt 30° south | 60.3 V | 0.940 | 0.950 | 0.958 (0.81) | 52.5 V | 0.954 | 0.963 | 0.968 (0.785) |
| Évora, tilt 30° south | 58.4 V | 0.944 | 0.959 | 0.960 (0.80) | 50.9 V | 0.958 | 0.971 | 0.971 (0.775) |
| open-rack mount | 59.3 V | 0.927 | 0.944 | 0.943 (0.78) | 51.3 V | 0.940 | 0.957 | 0.952 (0.75) |
| insulated back | 57.1 V | 0.912 | 0.936 | 0.941 (0.78) | 49.7 V | 0.926 | 0.949 | 0.950 (0.75) |
| Évora, insulated, −0.35 %/K | 54.3 V | 0.908 | 0.943 | 0.942 (0.765) | 47.3 V | 0.923 | 0.956 | 0.952 (0.74) |
| no field loss factor (clean model) | 57.7 V | 0.931 | 0.954 | 0.964 (0.78) | 50.1 V | 0.933 | 0.954 | 0.960 (0.75) |
| loss factor = field shaded/unshaded | 57.7 V | 0.909 | 0.931 | 0.944 (0.78) | 50.1 V | 0.932 | 0.953 | 0.958 (0.75) |
| effective Rs matched to field irradiance slope | 58.9 V | 0.921 | 0.941 | 0.946 (0.795) | 50.4 V | 0.931 | 0.952 | 0.952 (0.755) |
| effective Rs pinned to 0.05 (rejected, see Validation) | 62.9 V | 0.941 | 0.952 | 0.961 (0.84) | 55.5 V | 0.960 | 0.967 | 0.972 (0.825) |

- **Orientation is the largest single factor.** A 30° south tilt flattens the seasonal energy profile and
  cuts the summer irradiance peak. That brings the fixed-voltage loss to 6 % (fry) and 4.6 % (flat) against the ideal,
  and k·Voc then works with k ≈ 0.8.
- **Inland heat (Évora)** barely changes the best ratio. It moves the best voltage 2 V lower, and the November setting
  then loses 21 % (fry) against the ideal instead of 16 %.
- **Mount and tempco** move the fixed ratio by ±1 %. Of the three mounts, close mount reproduces the measured
  November Voc spread best (Validation).

## Battery-limited days

A van battery is often full in summer, and then no strategy is panel-limited. The load is unknown, so a daily
delivered-energy cap is used as a guess: each strategy delivers min(its daily energy, cap). The cap is the largest
measured November day (fry 2.8 kWh, flat 4.1 kWh), scaled ×1 and ×1.5. The model's energy scale is set so that its
largest day in the 10-22..11-21 window equals that measured day. The best fixed V and the best k are re-chosen under
the cap.

| | fry ×1.0 | fry ×1.5 | flat ×1.0 | flat ×1.5 |
|---|---|---|---|---|
| days at the cap | 203 | 104 | 203 | 98 |
| ideal energy below the cap | 75 % | 94 % | 76 % | 95 % |
| best fixed V | 0.982 @ 63.9 V | 0.947 @ 59.4 V | 0.987 @ 56.1 V | 0.957 @ 51.6 V |
| k·Voc, best k | 0.985 (k 0.83) | 0.966 (k 0.80) | 0.989 (k 0.805) | 0.972 (k 0.77) |
| MPPT, this firmware | 0.991 | 0.985 | 0.990 | 0.983 |

Ratios are against the capped ideal. If the battery fills on most days from spring to autumn (cap ×1), the
fixed-voltage handicap against the MPPT shrinks to about 1 % (fry) and 0.3 % (flat). The optimal fixed voltage
then moves back up towards the winter value, because the summer surplus no longer counts. At cap ×1.5 the handicap is
3.9 % / 2.7 %.

## Model

### Weather

PVGIS 5.3 typical meteorological year (`/api/v5_3/tmy`; SARAH-3 radiation, ERA5 meteorology, 2005-2023, DEM
horizon). It provides hourly GHI, DNI, DHI, 2-m air temperature and 10-m wind. Two generic public sites were requested,
both city centres:

- **Lisbon, 38.72 N, −9.14 E:** main case.
- **Évora, 38.57 N, −7.91 E:** inland sensitivity case, with hotter summers.

The converters' real location was not used. PVGIS radiation is instantaneous at HH:10, so the solar position is
taken at HH:10.

### Plane of array

- **Horizontal (main):** GHI split into beam and diffuse.
- **30° south:** Perez transposition.

The beam component gets the pvlib physical IAM (glass reflection), and diffuse is not corrected. There is no shading,
soiling or spectral model, since those are covered by the field loss factor below.

### Cell temperature

SAPM (King 2004) with its three standard parameter sets:

- **close mount, glass/glass (main):** panels a few cm above a van roof;
- **open rack, glass/polymer:** coolest;
- **insulated back, glass/polymer:** a panel glued to the roof.

Wind is the TMY 10-m wind. A parked or sheltered van would run hotter than the insulated case on calm days.

### P-V curve

The model is a single-diode curve per hour (pvlib `i_from_v`, `max_power_point`), in units relative to the string's
Voc at 25 °C and 1000 W/m²:

- photocurrent ∝ G, Isc tempco +0.05 %/K;
- diode thermal voltage ∝ T;
- saturation current set at each temperature so that Voc at 1000 W/m² follows the chosen tempco exactly;
- shunt resistance ∝ 1/G (De Soto).

**Calibration.** Three free parameters (ideality·Ns·Vth, series resistance, shunt resistance) are fitted on the
TMY hours of 10-22..11-21 that pass the field study's "bright" rule (Pmp > 30 % of the window's P95). They are fitted
to the measured mean Vmpp/Voc and to the curvature table P/Pmax at Vmpp·(1 ± 5/10 %). The voltage scale is then set so
that the mean Voc of those hours equals the measured 77.9 V (fry) and 69.6 V (flat). The fit is redone for every
mount, tempco and site.

| | fry | flat |
|---|---|---|
| Voc at 25 °C, 1000 W/m² | 82.4 V | 74.1 V |
| fit residual, Vmpp/Voc | −0.001 | −0.006 |
| fit residual, s at −10/−5/+5/+10 % | −0.002 / +0.006 / +0.002 / −0.001 | +0.021 / +0.018 / +0.005 / −0.007 |
| effective series resistance, Rs·Isc/Voc | 0.20 | 0.24 |
| resulting Pmp / Vmpp tempco | −0.44 / −0.44 %/K | −0.47 / −0.45 %/K |

The main case (close mount, −0.32 %/K) is shown. flat's curve is broader below the MPP than a single diode can
produce together with its low Vmpp/Voc (0.813).

**Effective series resistance.** The fitted series resistance is about 0.2 of Voc/Isc, roughly five times that of a
single module. It stands in for everything that makes the measured output-power curve soft below the MPP:

- module mismatch;
- wiring;
- converter loss rising with current, since P is output power.

It also makes Vmpp fall with irradiance, which matches the field data (Validation). Pinning it at a module-like 0.05
fits the curvature worse and underestimates the measured irradiance dependence by a factor of 3, so that variant is
rejected.

### Temperature coefficient

There is no datasheet, so an estimate from the field data was tried (`tempco_field.py`). Sweep Voc was regressed on
the converter NTC and ln(Pmax). The NTC coefficient came out between −0.01 and −0.42 %/K, depending on the subset and
the device. It changes sign without the irradiance term, and on flat the NTC reads −16 to +77 °C. The board
temperature carries no usable cell-temperature information, so a typical c-Si range is used instead: Voc
−0.28/−0.32/−0.35 %/K. The resulting Pmp tempco is −0.38 to −0.48 %/K on fry and −0.41 to −0.52 %/K on flat. The
high end is slightly above the typical −0.35 to −0.45 %/K, because the large effective series resistance adds its own
temperature sensitivity.

### Non-temperature losses

The model curves are clean: unshaded, single peak, hourly. The field month also lost energy to partial shading,
dawn/dusk and fast irradiance changes. This is carried over as one factor per device and strategy:

- loss factor = field ratio (all curves, November) / model ratio (clean, same window);
- fixed V: 0.983 on fry, 0.997 on flat; k·Voc: 0.977 / 0.990.

It is applied to every hour of the year, which assumes the November shading and cloud losses carry over to every
season in proportion. In summer the sun is higher and the dawn/dusk share is smaller, so this probably overstates the
summer shading loss on fry. Two alternatives are in the sensitivity table: no factor, and the field shaded/unshaded
ratio (0.976 / 0.999).

The firmware MPPT is modelled as a constant η = 0.976 (fry) / 0.973 (flat). That is the November operating-point
efficiency × (1 − sweep cost), and it already contains the shading and dawn/dusk losses.

## Validation

### Known case

A constant 25 °C cell and 800 W/m² all year gives one curve. The best fixed voltage gets 0.999997 (fry) and 0.9999999
(flat) of the ideal, at 62.90 V vs Vmpp 62.87 V on the 0.08 V grid. The loss is zero, as it must be.

### November window

The model was run on the TMY days 10-22..11-21 (Lisbon, horizontal, main mount and tempco) and compared with the
field month. The curvature table and mean Vmpp/Voc are fit targets, so they do not count. The spreads, the regression
coefficients and the fixed-voltage result do.

| bright, unshaded | fry model | fry field | flat model | flat field |
|---|---|---|---|---|
| Vmpp mean / std | 64.9 / 2.89 V | 64.9 / 2.09 V | 56.3 / 2.82 V | 56.7 / 2.37 V |
| Voc std | 1.14 V | 1.13 V | 0.94 V | 0.91 V |
| Vmpp/Voc std | 0.027 | 0.025 | 0.032 | 0.032 |
| Vmpp = a + b·Voc + c·ln P: b, c | 1.31, −4.76 | 1.20, −3.97 | 1.37, −5.16 | 1.18, −4.92 |
| best fixed V, clean / unshaded | 63.4 V | 64.5 V | 55.0 V | 56.5 V |
| ratio at that V | 0.977 | 0.984 | 0.977 | 0.975 |
| k·Voc best k | 0.82 | 0.84 | 0.795 | 0.81 |
| mean / largest day | 2.24 / 2.81 kWh | 1.18 / 2.81 kWh | 3.30 / 4.13 kWh | 1.49 / 4.13 kWh |

- **Voc spread** matches within 4 %. Open rack gives 0.63-0.81 V on fry (too little) and insulated back 1.13-1.58 V.
  Close mount at −0.32 %/K and insulated back at −0.28 %/K fit best on both devices.
- **Irradiance dependence** of Vmpp (c) is 20 % too strong on fry and 5 % on flat. On fry the Vmpp spread is 38 % too
  wide, so the clean November fixed-voltage loss is 2.3 % against the field's 1.6 %. Pinning the series resistance to
  match the field c (0.163 on fry, 0.227 on flat) raises the yearly fixed ratio by 0.5 % (fry) and 0.1 % (flat).
- **Best fixed voltage** sits 1.1 V (fry) and 1.5 V (flat) below the field value, and the best k is 0.015-0.02 low.
  The absolute voltages in the results may be low by about 1-1.5 V. The ratios hardly change.
- **Daily energy:** the TMY November is much sunnier than the field month. The field mean also contains fry's outage
  days and battery-limited time. This matters only for the battery cap.

## Caveats

- **Summer is extrapolated, not measured.** No converter data exists before 2026-10-22. Two extrapolations dominate the
  error:
  - the Vmpp drop with irradiance, from November's 250-650 W/m² to summer's 1000 W/m²;
  - the Vmpp drop with temperature (tempco and mount).

  Each is worth about half the loss. The bands cover the tempco and mount, but not a different physical origin of the
  effective series resistance.
- **Orientation is unknown** and moves the fixed-voltage result by 2.4 % (horizontal vs 30° south).
- **Firmware MPPT in summer is unmeasured.** Its η is held at the November value. A summer η 1 % lower or higher
  shifts every "vs MPPT" number by 1 %.
- **Shading carried over** as a constant factor; see Non-temperature losses.
- **Hourly weather:** no sub-hourly irradiance variability in the model. The field factor absorbs part of it.
- **Battery cap is a guess.** Both converters charge the same battery, so the caps are not independent. The cap uses
  daily totals and ignores when in the day the battery fills.
- **Typical year:** each TMY month comes from a different real year, so year-to-year variation is not represented.
- **Output power:** the calibration curves are converter output power, as in the field study.

## Data and requests

- **Field data:** the 11-21 study's 10-s bins and sweep curves (`/tmp/mpptfix/*.pkl`, re-fetchable with its
  `fetch_bins.py`/`fetch_sweeps.py`). `measured.py` reduces them to `nov_measured.json`. No new InfluxDB queries were
  made.
- **External requests:** two HTTPS GETs to PVGIS, as in `fetch_tmy.sh`:
  - `https://re.jrc.ec.europa.eu/api/v5_3/tmy?lat=38.72&lon=-9.14&outputformat=json`
  - `https://re.jrc.ec.europa.eu/api/v5_3/tmy?lat=38.57&lon=-7.91&outputformat=json`

  Nothing else was sent. `pvlib` 0.16.1 was installed into `.venv` from PyPI.
- **Scripts**, in `2026-11-22-mppt-vs-fixed-voltage-year/`:
  - `tmy.py`: JSON to `tmy_<site>.csv.gz`;
  - `model.py`: weather, plane of array, cell temperature, single-diode model, calibration;
  - `year.py`: strategies, validation and sensitivities, output in `year_output.txt`;
  - `tempco_field.py`: the tempco attempt, output in `tempco_field_output.txt`.

  Run from the repo root with `.venv/bin/python3 doc/research/2026-11-22-mppt-vs-fixed-voltage-year/year.py` (about
  20 s).
