*this document is an LLM generated placeholder*

# vconv plant corrected against SPICE, tracker commits re-validated (2026-11-15)

Follow-up to `doc/research/vconv-vs-spice.md` (the SPICE cross-check; its artifacts are in `/tmp/vconv-spice/`).
Step 1: fix the buck plant in `src/sim/vconv.cpp`, make the tracker scenarios run on it, check it against the SPICE
numbers, and re-run the tracker commits de37d33, ed5ffe5, 9e1dd13, 8b68511 and 738cc16 on it. No tracker code was
changed. Nothing was flashed or sent to a device.

## 1. Plant changes

### Model (buck only; boost unchanged)

| change | report item | how |
|---|---|---|
| Dead time | fix 5 | `t_dead` before each rising gate edge. HS off → LS body diode (node −Vf) for `t_dead`, then LS on; the LS window loses `t_dead` at its start, its end stays. An LS pulse shorter than `t_dead` is swallowed. If the idle time before the next HS is shorter than `t_dead`, the HS starts late (sync CCM: two diode intervals per cycle). |
| HS on-time extension | fix 5 | `t_hs_ext` (turn-off minus turn-on delay). Implemented, **set to 0** on both boards: the 40-80 ns that the SPICE peak currents suggest drives the stiff-source P2 boundary point into a CCM runaway, and every SPICE comparison matched better without it. |
| Nonlinear node C | fix 3 | `ring_c` applies at `Vin = ring_v`; C(Vin) = `ring_c`·(`ring_v`/Vin)^`ring_cexp`, recomputed when Vin moves > 1 %. SPICE free ring: fry 200 kHz at 45 V, 212 at 55, 233 at 72-78; flat 286 / 303 / 314 kHz at 45 / 55 / 60 V, i.e. C ∝ Vin^-0.59 (fry) / ^-0.65 (flat). |
| Separate edge C | fix 3 | `ring_ct`: the hard edges move `ring_ct`·C (HS turn-on charge from the input, node swing at HS turn-off); the free ring uses C. 0.75 from the SPICE flu bench (rise transit 2.98 nF vs free ring 3.9 nF). |
| Soft HS turn-off | fix 3/4 | During the HS→LS dead time a positive coil current swings the node down and takes its energy (`ring_ct`·C·Vin²·f(2−f)/2, f = share of the swing done in `t_dead`); the rest is lost when the LS turns on hard. Before, the whole node energy was lost at every cycle, which made the old ring plant lose 0.24 W at 4-20 W where SPICE loses 0.02-0.06 W. |
| HS turn-off loss | fix 4 | `t_sw`: channel current falls linearly in `t_sw`, the node C takes the rest until the node reaches 0 V. Loss I²·t²/(24·C) at small current, → ½·Vin·I·t at large current, paid from the input. A plain ½·Vin·I·t overlap (first try) cost ~0.1 W at dawn currents, where SPICE shows 0.02 W in total. |
| Conduction | fix 4 | `r_hs`, `r_ls`: R·i against the coil while that switch conducts (one corrector step on the phase-mean current), so the loss lowers the delivered current at a given duty. |
| Diode on the return path | fix 6 | `diode_vf` now also on the non-ring HS-body-diode return (node at Vin + Vf) and in the dead time; it is a `vconv.conf` key now (was only `vconv set diode_vf` / `setDiodeVf`). |

All new keys default to 0 (or 1 / 0.6 for `ring_ct` / `ring_cexp`, inert without `ring_v`), so a plant without them is
bit-identical to the old one: `SIM_RING=ideal` reproduces the baseline numbers exactly (wx-dawn 97.4, wx-low 98.5,
wx-cloud-cv 91.4 checked), and vconv-test (1309/1309), dcm-current-test, rect-cal-test, coil-fit-test and
bms-cutoff-test pass unchanged.

### New `vconv.conf` keys

`ring_v` (V, 0), `ring_cexp` (0.6), `ring_ct` (1), `diode_vf` (V, 0), `t_dead` (s, 0), `t_hs_ext` (s, 0), `r_hs` (Ω,
0), `r_ls` (Ω, 0), `t_sw` (s, 0). Read in `src/adc/sensor_setup.cpp::configureVirtualConverter`; documented in
`website/docs/reference/config/vconv.md` (table plus a "Switch losses and timing" section);
`config/lab/vconv_mock/conf/vconv.conf` carries a commented example. `etc/config-tool/conf-editor.html` has no
`vconv.conf` entries at all (no `ring_c` either), so nothing was added there.

### Board sets (`test/host-stub/plant_boards.h`)

| | fry | flat | source |
|---|--:|--:|---|
| coil.conf L0 / plant L | 80 / 76 µH | 56 / 51 µH | unchanged field fits |
| ring f at `ring_v` | 233 kHz at 75 V (6.14 nF) | 320 kHz at 60 V (4.85 nF) | SPICE free ring (report §5) |
| `ring_cexp` | 0.6 | 0.6 | SPICE ring f vs Vin |
| `ring_q` | 25 | 15 | field DCM sweeps (unchanged) |
| `ring_q` of the `-hq` variant | 150 | 100 | chosen so the dawn ripple matches the undamped SPICE bench |
| `ring_ct` | 0.75 | 0.75 | SPICE flu transit vs ring |
| `diode_vf` | 0.7 V | 0.7 V | |
| `t_dead` | 185 ns | 185 ns | UCC21330B, RDT 20k |
| `t_hs_ext` | 0 | 0 | see above |
| `r_hs` / `r_ls` | 8 / 14 mΩ | 12 / 18 mΩ | fitted to SPICE CCM η |
| `t_sw` | 100 ns | 100 ns | fitted to SPICE CCM η and dawn η 8-30 W |

The old ring boards in mppt-sim were fry 254 kHz / flat 326 kHz, linear C, lossless, no dead time.

### Simulator wiring (`test/host-stub/mppt-sim.cpp`)

- Board plants are written into `vconv.conf` before `SIM_CONF` (so `SIM_CONF` still overrides), i.e. the conf path
  is what the sim exercises. `SIM_PLANT="fry:t_sw=50e-9,..."` overrides board fields.
- **wx-\* scenarios now default to the fry board plant** (ring on, Vf, dead time, losses). `SIM_RING=flat` /
  `fry-hq` / `flat-hq` select others, `SIM_RING=ideal` the old lossless no-ring plant. wx-sun-\* keep their explicit
  `-off` boards. ring-\* and rcal-flat pick up the new board numbers (ring frequency, losses); rcal/rcal-sun are
  unchanged.
- `SIM_SEED=<n>` reseeds the vconv ADC Gaussian noise (host sim only, `FUGU_HOST_SIM`).
- The host sim still runs the LEDC-like driver at pwmMax 2047 / 39 kHz; fry/flat run MCPWM 4103. The plant is
  time-based, so the physics is the same; the tracker's step in counts is relative to its own pwmMax.

### On-target build

`SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.vconv_s3" idf.py -B build-vconv -DSDKCONFIG=build-vconv/sdkconfig
build` (ESP-IDF 6.0.3, esp32s3, `CONFIG_FUGU_WITH_VCONV=y`) builds: fugu-firmware.bin 0x18fa30, 13 % of the smallest
app slot free; no warnings from the touched files. Not run on a device.

## 2. Plant vs SPICE

Repeatable with `test/host-stub/vconv-spice-check.sh` (`-v` for every point); it exits 1 when a value leaves its
band. It drives the plant with stiff Vin/Vout at pwmMax 4103 / 38996 Hz, the firmware DCM LS rule and **the bench's
coil** (fry 80 µH, flat 51 µH) with the board's ring frequency, so it compares like with like; the SPICE reference
numbers are embedded from the report (`/tmp/vconv-spice/results/dawn_summary.json`, `dawn_b.json`, `p2.json`,
`ccm_table.md`). "ideal" = the old plant without ring; "old ring" = the old mppt-sim ring boards (254 / 326 kHz, linear
C, lossless) run through the same check. B = SPICE bench (undamped, no core loss), Brp = bench with the board's Q as a
parallel R (only run at fry-35/45 and flat-45).

Known-bad calibration: the old ring plant fails 12 bands (near-boundary Pout +45 %, CCM η 99.8-99.9 %, dawn η 8-30 W
at 55 V, ripple at 30 V); the ideal plant fails all ripple, η and near-boundary bands. The corrected plant passes all.

### Dawn: Pout(HS) ripple, falling steps, efficiency (stiff Vin, Vout 26.5 V, 45-count HS steps)

Ripple = half 5-95 % range of Pout / smooth log-log fit (Pout > 1 W); falling = share of 45-count steps where Pout
drops although HS rises.

| set | ripple ideal | old ring | **new** | new -hq | SPICE B | Brp | falling: old ring | **new** | new -hq | B | Brp | ripple period new / B (counts) |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| fry-30 | 0.005 | 0.095 | **0.166** | 0.183 | 0.196 | - | 0.33 | **0.50** | 0.50 | 0.51 | - | 855 / 900 |
| fry-35 | 0.009 | 0.114 | **0.162** | 0.192 | 0.208 | 0.174 | 0.10 | **0.13** | 0.20 | 0.28 | 0.17 | 630 / 645 |
| fry-45 | 0.003 | 0.141 | **0.200** | 0.265 | 0.286 | 0.223 | 0.00 | **0.00** | 0.09 | 0.22 | 0.03 | 465 / 465 |
| fry-55 | 0.001 | 0.170 | **0.200** | 0.322 | 0.338 | - | 0.00 | **0.00** | 0.00 | 0.04 | - | 321 / 345 |
| flat-30 | 0.009 | 0.060 | **0.092** | 0.134 | 0.141 | - | 0.19 | **0.24** | 0.36 | 0.36 | - | 645 / 756 |
| flat-35 | 0.009 | 0.061 | **0.084** | 0.143 | 0.165 | - | 0.00 | **0.00** | 0.05 | 0.16 | - | 450 / 472 |
| flat-45 | 0.009 | 0.071 | **0.089** | 0.200 | 0.256 | 0.103 | 0.00 | **0.00** | 0.00 | 0.23 | 0.00 | 309 / 330 |
| flat-55 | 0.004 | 0.070 | **0.074** | 0.221 | 0.287 | - | 0.00 | **0.00** | 0.00 | 0.25 | - | 231 / 244 |
| mean falling | | | | | | | 0.08 | **0.11** | 0.15 | 0.26 | (0.07) | |

| set | η 1-3 W ideal | old ring | **new** | B | Brp | η 8-30 W old ring | **new** | B | Brp |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| fry-30 | 1.000 | 0.930 | **0.926** | 0.940 | - | - | - | - | - |
| fry-35 | 1.000 | 0.915 | **0.919** | 0.910 | 0.900 | 0.989 | **0.9935** | 0.998 | 0.993 |
| fry-45 | 1.000 | 0.842 | **0.856** | 0.779 | 0.838 | 0.984 | **0.9914** | 0.997 | 0.989 |
| fry-55 | 1.000 | 0.777 | **0.839** | 0.871 | - | 0.975 | **0.9878** | 0.993 | - |
| flat-30 | 1.000 | 0.929 | **0.956** | 0.955 | - | 0.991 | **0.9932** | 0.998 | - |
| flat-35 | 1.000 | 0.927 | **0.960** | 0.974 | - | 0.991 | **0.9936** | 0.998 | - |
| flat-45 | 1.000 | 0.865 | **0.933** | 0.866 | 0.870 | 0.984 | **0.9906** | 0.997 | 0.983 |
| flat-55 | 1.000 | 0.764 | **0.919** | 0.969 | - | 0.974 | **0.9868** | 0.993 | - |

(old-ring values from the same check run; the -hq η 8-30 W is within 0.003 of the field-Q values, its η 1-3 W up to 0.035 higher.)

- **Ripple period** now matches SPICE within 0-15 % on every set (old ring: 10-25 % short for fry), from the 233 kHz
  ring and the Vin-dependent C.
- **Ripple amplitude** at the field Q sits at or slightly below the damped bench (fry-45 0.200 vs Brp 0.223, flat-45
  0.089 vs 0.103) and at 0.3-0.85 of the undamped bench. The SPICE deck has no core loss, so the true Q is not known
  from SPICE: the field-fit Q (25 / 15) and the undamped bench bracket it. The `-hq` variant (Q 150 / 100) reproduces
  the undamped ripple within 3-25 %.
- **Falling steps**: the corrected plant has them where Vin ≤ 35 V (fry-30 0.50 vs SPICE 0.51, flat-30 0.24 vs 0.36),
  none at 45-55 V with the field Q, like the damped bench (0.00-0.03). The undamped bench has 0.04-0.25 there; the
  `-hq` plant reaches 0.09 at fry-45 and 0 elsewhere. The plant ripple is closer to a sine than the SPICE plateau-and-
  jump staircase, which is what makes falling steps at 45-55 V; that shape is not reproduced (measured, not fixed).
- **Efficiency**: 1-3 W mean |plant − B| 0.033 (old ring 0.056, ideal 0.09; -hq 0.032). 8-30 W within 0.3-0.7 pp of B and inside
  the B..Brp span where Brp exists; at 55 V the plant is 0.6 pp below B (old ring 1.8-1.9 pp).

### DCM near Vin ≈ Vout (Vin 30 V, LS near its minimum, 7 / 6 SPICE points)

| | ideal | old ring | **new** |
|---|--:|--:|--:|
| fry, mean Pout vs SPICE | +79 % | +45 % | **−1 %** (HS 3182: 5.29 vs 5.31 W; falls with HS like SPICE) |
| flat, mean Pout vs SPICE | +29 % | +16 % | **+8 %** |

### ~100 W DCM/CCM boundary staircase (flat 60 V 14 points, fry 75 V 13 points)

| | ideal mean / rms | old ring | **new** |
|---|--:|--:|--:|
| flat | +0.9 / 7.2 % | +0.4 / 4.5 % | **+0.3 / 3.7 %** |
| fry | +0.8 / 7.4 % | +0.7 / 6.4 % | **+0.1 / 1.8 %** |

The fry point at HS 1449 / LS 2571 (D·Vin = Vout within 0.05 %) is excluded: with stiff sources the corrected plant
runs into CCM there (94 W vs SPICE 75.6 W), which a soft PV source cannot do.

### CCM efficiency (synchronous, LS = pmax − HS − 1, soft battery)

| point | plant η | SPICE η |
|---|--:|--:|
| flat 60 V 3.8 / 11.3 / 22.6 A | 99.25 / 98.99 / 98.38 % | 98.94 / 98.88 / 98.38 % |
| flat 30 V 3.0 / 11.3 A | 99.64 / 99.30 % | 99.52 / 99.31 % |
| fry 75 V 3.8 A | 99.15 % | 98.75 % |
| fry 60 V 11.3 / 22.6 A | 99.20 / 98.74 % | 99.11 / 98.74 % |
| fry 30 V 3.0 / 11.3 A | 99.66 / 99.47 % | 99.54 / 99.48 % |

Mean |error| 0.12 pp (ideal and old ring: 100 % everywhere, 0.64 pp). The plant is 0.3-0.4 pp too efficient at light
CCM load and high Vin (Coss energy at 60-75 V with the nonlinear, 0.75-scaled C).

### Not re-checked

LS zero-crossing shift (+40..90 counts in SPICE), return fraction of reverse energy (report §7) and the rect-cal knee
(§8) were not re-measured on the corrected plant; rcal-\* scenarios were not re-run.

## 3. Tracker commits on the corrected plant

Variants: the five tracker commits were reverse-applied (`src/` hunks only) to the corrected tree, giving pre
(= d62bb11 tracker, parent of de37d33), +de37d33, +ed5ffe5, +9e1dd13, +8b68511 and +738cc16 (= main). Each variant
ran the nine wx-\* scenarios on five plants: `ideal` (no ring, no losses), `fry`, `flat` (field Q), `fry-hq`,
`flat-hq` (bench Q). Noise:

- Gaussian: the scenario's own ADC noise, `SIM_SEED` 1, 2, 3 for the four noisy scenarios (wx-low, wx-low-slow,
  wx-mid, wx-cloud-cv-ripple); the other five are deterministic and run once.
- Measured: `SIM_NOISE=fry|flat` replays that converter's captured sensor noise, `SIM_NOISE_OFS` 0, 6, 13 s, on the
  matching plant only.

Cells are mean (min..max) over the runs; a single number is one deterministic run. Metric: SUMMARY "harvest % of
GMPP". 834 runs, 0 errors (27 flat-hq measured-noise runs of the pre
variant from an aborted first pass are left out). All numbers below are measured in the host sim; nothing was run on hardware.

### Summary (mean of the nine scenarios)

| plant / noise | pre | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | main |
|---|--:|--:|--:|--:|--:|--:|
| ideal, Gaussian | 91.71 | 93.59 | 95.05 | 95.46 | 95.46 | 95.47 |
| fry, Gaussian | 92.56 | 93.60 | 95.14 | 95.49 | 95.48 | 95.50 |
| fry, measured noise | 93.51 | 93.83 | 95.30 | 95.54 | 95.62 | 95.56 |
| flat, Gaussian | 92.18 | 93.56 | 95.20 | 95.56 | 95.54 | 95.55 |
| flat, measured noise | 89.88 | 90.92 | 92.83 | 93.53 | 93.41 | 93.44 |
| fry-hq, Gaussian | 92.37 | 93.34 | 95.07 | 95.47 | 95.45 | 95.48 |
| flat-hq, Gaussian | 92.66 | 93.11 | 95.11 | 95.46 | 95.45 | 95.49 |

Tracking loss (SUMMARY, % below the instantaneous MPP, mean of scenarios):

| plant / noise | pre | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | main |
|---|--:|--:|--:|--:|--:|--:|
| ideal, Gaussian | 6.09 | 4.17 | 2.56 | 2.17 | 2.16 | 2.16 |
| fry, Gaussian | 5.29 | 4.19 | 2.57 | 2.22 | 2.21 | 2.20 |
| fry, measured noise | 4.43 | 4.01 | 2.49 | 2.21 | 2.08 | 2.19 |
| flat, Gaussian | 5.65 | 4.21 | 2.42 | 2.12 | 2.11 | 2.13 |
| flat, measured noise | 7.71 | 6.60 | 4.60 | 3.90 | 4.01 | 3.98 |
| fry-hq, Gaussian | 5.48 | 4.42 | 2.75 | 2.32 | 2.31 | 2.29 |
| flat-hq, Gaussian | 5.18 | 4.61 | 2.64 | 2.34 | 2.32 | 2.30 |

### Findings per commit

- **de37d33** (dawn): the largest single gain at dawn on every plant, wx-dawn 79.9-87.2 -> 91.2-93.6 (Gaussian),
  fry replay 90.1 -> 93.2, flat replay 84.2 (74.9..91.3) -> 92.6. Neutral elsewhere; wx-low-slow on flat-hq -1.0
  (89.8 -> 88.8, within the seed spread of 87.7..90.1).
- **ed5ffe5**: +1.5 to +2.0 on the scenario mean on every plant and noise; wx-low-slow +2.9..+6.3, wx-dawn +2.9..+6.2.
  No regression found.
- **9e1dd13** (settle window): wx-low-slow +0.5..+2.8 and wx-cloud-cv-ripple +0.7..+2.5 everywhere, and +2.1 / +2.5
  on flat replay wx-cloud-cv / -ripple. **Worse** on wx-low with Gaussian noise on two plants:
  fry 98.3 (98.2..98.4) -> 97.6 (96.9..98.1) and flat-hq 98.1 (98.0..98.2) -> 97.1 (95.4..98.0); the ranges barely
  overlap. Not reproduced on ideal (98.3 -> 98.2), flat (98.2 -> 98.2), fry-hq (98.0 -> 98.2) or with measured noise
  (fry 98.1 -> 98.1, flat 98.6 -> 98.6). wx-dusk -0.3 on fry (98.6 -> 98.3) and -0.1 on flat (98.1 -> 98.0), single
  deterministic runs; 0..+0.4 on the other plants and with measured noise.
- **8b68511** (limiter hand-back keeps the slow-mode clock): **worse** on flat with measured flat noise, wx-low-slow
  98.5 (98.4..98.6) -> 97.5 (96.1..98.5). Per offset: 0 s 98.6 -> 96.1, 6 s 98.5 -> 97.9, 13 s 98.4 -> 98.5; tracking
  loss at offset 0 rises 0.57 -> 3.05 % and reversals fall 114 -> 45 /min, i.e. the tracker stays in slow mode while
  the MPP walks away. 738cc16 does not change these runs (identical numbers). Elsewhere 8b68511 is within ±0.1,
  except fry replay wx-cloud-cv +0.7 (90.9 -> 91.6), which 738cc16 gives back (91.0).
- **738cc16** (45 s slow-mode hold): within ±0.1 of 8b68511 on every Gaussian cell. With measured noise, fry
  wx-cloud-cv 91.6 -> 91.0 (ranges 91.3..91.7 vs 90.7..91.3, no overlap); the scenario mean is -0.06 fry, +0.03 flat.

Net, pre -> main: +2.0 to +3.8 on the scenario mean on every plant (fry replay +2.05, flat replay +3.56). No scenario
ends lower at main than at pre. The two cells where a single commit loses more than its spread (9e1dd13 on Gaussian
wx-low for fry/flat-hq, 8b68511 on flat-replay wx-low-slow) are sub-scenario regressions of 0.7-1.0 pp in mean and
up to 2.6 pp in the worst run, against gains of several pp from the same commits elsewhere.

Plant effect at main versus the old-plant baseline (wx-dawn 97.4, wx-low 98.5, wx-low-slow 98.8, wx-mid 99.5, wx-dusk
98.2, wx-cloud 98.5, wx-cloud-cv 91.4, wx-cloud-cv-ripple 86.5, wx-sweep-fall 90.2): at main the board plants stay
within 1.0 pp of it, except wx-low on flat-hq (97.1, -1.4, the 9e1dd13 cell above), wx-cloud-cv-ripple +0.8..+0.9
Gaussian and +2.3 fry replay (losses raise PV power at the same capped output, see Limitations) and,
with measured flat noise, wx-cloud-cv 83.1 and wx-cloud-cv-ripple 77.1. Those two drops come from the flat noise,
not the plant: the same flat plant with Gaussian noise gives 92.2 / 87.4. The pre-fix tracker loses 7-17 pp at
wx-dawn on every plant and noise (79.9-90.1 vs 97.0-98.0 at main).

### Full tables

### plant ideal, noise scenario Gaussian, SIM_SEED 1-3 (harv)

| scenario | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn (old plant 97.4) | 79.9 | 93.6 | 96.5 | 97.4 | 97.4 | 97.4 | +17.50 |
| wx-low (old plant 98.5) | 92.0 (91.7..92.4) | 95.2 (94.9..95.6) | 98.3 (97.6..98.7) | 98.2 (97.5..98.7) | 98.2 (97.5..98.7) | 98.2 (97.5..98.7) | +6.17 |
| wx-low-slow (old plant 98.8) | 92.7 (92.5..93.1) | 92.7 (92.2..93.1) | 97.5 (97.2..97.7) | 98.8 (98.7..98.9) | 98.8 (98.7..98.9) | 98.8 (98.7..98.9) | +6.07 |
| wx-mid (old plant 99.5) | 99.5 (99.5..99.6) | 99.5 (99.5..99.6) | 99.6 (99.5..99.6) | 99.5 (99.5..99.6) | 99.5 (99.5..99.6) | 99.6 ±0 (n3) | +0.07 |
| wx-dusk (old plant 98.2) | 96.7 | 96.7 | 97.9 | 98.2 | 98.2 | 98.2 | +1.50 |
| wx-cloud (old plant 98.5) | 98.3 | 98.3 | 98.5 | 98.5 | 98.5 | 98.5 | +0.20 |
| wx-cloud-cv (old plant 91.4) | 90.9 | 90.9 | 91.5 | 91.4 | 91.4 | 91.4 | +0.50 |
| wx-cloud-cv-ripple (old plant 86.5) | 85.2 (85.0..85.3) | 85.2 (85.0..85.3) | 85.5 (85.4..85.5) | 86.9 (86.8..87.0) | 86.9 (86.8..87.0) | 86.9 (86.8..87.0) | +1.73 |
| wx-sweep-fall (old plant 90.2) | 90.2 | 90.2 | 90.2 | 90.2 | 90.2 | 90.3 | +0.10 |
| mean of scenarios | 91.71 | 93.59 | 95.05 | 95.46 | 95.46 | 95.47 | |

### plant fry, noise scenario Gaussian, SIM_SEED 1-3 (harv)

| scenario | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 83.7 | 93.0 | 96.6 | 97.4 | 97.4 | 97.4 | +13.70 |
| wx-low | 96.2 (95.1..98.0) | 96.1 (95.6..96.8) | 98.3 (98.2..98.4) | 97.6 (96.9..98.1) | 97.6 (96.9..98.1) | 97.6 (96.9..98.1) | +1.40 |
| wx-low-slow | 91.8 (91.1..92.5) | 92.0 (91.3..92.5) | 96.7 (96.0..97.3) | 98.6 (98.5..98.6) | 98.6 (98.5..98.6) | 98.6 (98.5..98.6) | +6.80 |
| wx-mid | 99.5 (99.4..99.5) | 99.5 (99.4..99.5) | 99.5 ±0 (n3) | 99.5 ±0 (n3) | 99.5 ±0 (n3) | 99.6 ±0 (n3) | +0.13 |
| wx-dusk | 96.4 | 96.4 | 98.6 | 98.3 | 98.3 | 98.3 | +1.90 |
| wx-cloud | 98.5 | 98.5 | 98.6 | 98.6 | 98.6 | 98.5 | +0.00 |
| wx-cloud-cv | 91.4 | 91.4 | 91.9 | 91.9 | 91.9 | 92.0 | +0.60 |
| wx-cloud-cv-ripple | 85.6 (85.4..85.7) | 85.6 (85.4..85.7) | 86.1 (86.0..86.2) | 87.4 (87.3..87.5) | 87.4 (87.3..87.5) | 87.4 (87.3..87.5) | +1.83 |
| wx-sweep-fall | 90.0 | 90.0 | 90.0 | 90.1 | 90.0 | 90.1 | +0.10 |
| mean of scenarios | 92.56 | 93.60 | 95.14 | 95.49 | 95.48 | 95.50 | |

### plant fry, noise measured fry noise, 3 offsets (harv)

| scenario | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 90.1 (89.0..90.7) | 93.2 (91.9..94.1) | 96.5 (96.2..97.0) | 97.0 (96.9..97.2) | 97.0 (96.9..97.2) | 97.0 (96.9..97.2) | +6.93 |
| wx-low | 96.0 (95.7..96.4) | 96.2 (96.0..96.6) | 98.1 (97.7..98.5) | 98.1 (97.8..98.7) | 98.1 (97.8..98.7) | 98.1 (97.8..98.7) | +2.10 |
| wx-low-slow | 93.3 (92.1..94.9) | 92.9 (92.0..94.3) | 97.4 (97.1..97.7) | 98.4 (98.3..98.5) | 98.4 (98.3..98.5) | 98.4 (98.3..98.5) | +5.10 |
| wx-mid | 99.6 (99.5..99.6) | 99.6 (99.5..99.6) | 99.6 ±0 (n3) | 99.6 ±0 (n3) | 99.6 ±0 (n3) | 99.6 (99.6..99.7) | +0.07 |
| wx-dusk | 96.5 (96.4..96.6) | 96.5 (96.4..96.6) | 98.3 (98.1..98.5) | 98.4 (98.3..98.4) | 98.4 (98.3..98.4) | 98.4 (98.3..98.4) | +1.87 |
| wx-cloud | 98.2 (97.8..98.4) | 98.2 (97.8..98.4) | 98.5 (98.4..98.6) | 98.5 (98.4..98.6) | 98.5 (98.4..98.6) | 98.5 (98.5..98.6) | +0.33 |
| wx-cloud-cv | 90.8 (90.6..91.1) | 90.8 (90.6..91.1) | 91.2 (91.0..91.4) | 90.9 (90.7..91.1) | 91.6 (91.3..91.7) | 91.0 (90.7..91.3) | +0.20 |
| wx-cloud-cv-ripple | 87.0 (87.0..87.1) | 87.0 (87.0..87.1) | 88.1 (88.0..88.1) | 88.8 (88.7..88.8) | 88.8 (88.7..89.0) | 88.8 (88.7..88.8) | +1.73 |
| wx-sweep-fall | 90.1 (90.1..90.2) | 90.1 (90.1..90.2) | 90.1 ±0 (n3) | 90.2 (90.1..90.2) | 90.2 (90.1..90.2) | 90.2 ±0 (n3) | +0.07 |
| mean of scenarios | 93.51 | 93.83 | 95.30 | 95.54 | 95.62 | 95.56 | |

### plant flat, noise scenario Gaussian, SIM_SEED 1-3 (harv)

| scenario | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 79.9 | 92.6 | 96.1 | 97.3 | 97.3 | 97.3 | +17.40 |
| wx-low | 95.4 (95.0..95.8) | 95.9 (95.5..96.2) | 98.2 (97.9..98.6) | 98.2 (97.6..98.7) | 98.2 (97.6..98.7) | 98.2 (97.6..98.7) | +2.80 |
| wx-low-slow | 92.9 (92.5..93.2) | 92.1 (91.5..92.6) | 97.4 (97.1..97.6) | 98.6 (98.5..98.6) | 98.6 (98.5..98.6) | 98.6 (98.5..98.6) | +5.67 |
| wx-mid | 99.5 ±0 (n3) | 99.5 ±0 (n3) | 99.5 (99.5..99.6) | 99.5 ±0 (n3) | 99.5 ±0 (n3) | 99.6 ±0 (n3) | +0.10 |
| wx-dusk | 96.2 | 96.2 | 98.1 | 98.0 | 98.0 | 98.0 | +1.80 |
| wx-cloud | 98.2 | 98.2 | 98.5 | 98.6 | 98.6 | 98.5 | +0.30 |
| wx-cloud-cv | 91.6 | 91.6 | 92.4 | 92.3 | 92.2 | 92.2 | +0.60 |
| wx-cloud-cv-ripple | 85.8 (85.7..86.1) | 85.8 (85.7..86.1) | 86.5 (86.3..86.6) | 87.4 (87.3..87.5) | 87.4 (87.3..87.5) | 87.4 (87.3..87.5) | +1.57 |
| wx-sweep-fall | 90.1 | 90.1 | 90.1 | 90.2 | 90.1 | 90.2 | +0.10 |
| mean of scenarios | 92.18 | 93.56 | 95.20 | 95.56 | 95.54 | 95.55 | |

### plant flat, noise measured flat noise, 3 offsets (harv)

| scenario | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 84.2 (74.9..91.3) | 92.6 (91.8..93.4) | 96.5 (96.3..96.6) | 97.3 (97.3..97.4) | 97.3 (97.3..97.4) | 97.3 (97.3..97.4) | +13.17 |
| wx-low | 95.1 (93.7..96.2) | 95.6 (94.6..96.4) | 98.6 (98.5..98.8) | 98.6 (98.1..98.9) | 98.6 (98.1..98.9) | 98.6 (98.1..98.9) | +3.47 |
| wx-low-slow | 94.7 (94.4..94.9) | 95.1 (94.6..95.5) | 98.0 (97.9..98.2) | 98.5 (98.4..98.6) | 97.5 (96.1..98.5) | 97.5 (96.1..98.5) | +2.80 |
| wx-mid | 99.4 (99.3..99.5) | 99.4 (99.3..99.5) | 99.6 (99.5..99.6) | 99.6 ±0 (n3) | 99.6 ±0 (n3) | 99.7 ±0 (n3) | +0.27 |
| wx-dusk | 96.6 (96.3..97.0) | 96.6 (96.3..97.0) | 98.2 (98.1..98.3) | 98.6 (98.4..98.8) | 98.6 (98.4..98.8) | 98.6 (98.4..98.8) | +2.07 |
| wx-cloud | 98.3 (98.2..98.4) | 98.3 (98.2..98.4) | 98.5 (98.5..98.6) | 98.5 (98.4..98.7) | 98.5 (98.4..98.7) | 98.6 ±0 (n3) | +0.30 |
| wx-cloud-cv | 78.8 (78.4..79.0) | 78.8 (78.4..79.0) | 81.0 (80.9..81.1) | 83.1 (83.0..83.1) | 83.1 (83.0..83.1) | 83.1 (83.0..83.1) | +4.30 |
| wx-cloud-cv-ripple | 71.5 (71.3..71.7) | 71.5 (71.3..71.7) | 74.6 (74.5..74.6) | 77.1 (77.0..77.2) | 77.1 (77.0..77.2) | 77.1 (77.0..77.2) | +5.60 |
| wx-sweep-fall | 90.4 ±0 (n3) | 90.4 ±0 (n3) | 90.4 ±0 (n3) | 90.4 (90.4..90.5) | 90.4 ±0 (n3) | 90.5 ±0 (n3) | +0.10 |
| mean of scenarios | 89.88 | 90.92 | 92.83 | 93.53 | 93.41 | 93.44 | |

### plant fry-hq, noise scenario Gaussian, SIM_SEED 1-3 (harv)

| scenario | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 84.0 | 91.2 | 97.0 | 97.1 | 97.1 | 97.1 | +13.10 |
| wx-low | 96.3 (94.4..97.8) | 96.5 (95.3..97.4) | 98.0 (97.8..98.2) | 98.2 (98.1..98.3) | 98.2 (98.1..98.3) | 98.2 (98.1..98.3) | +1.87 |
| wx-low-slow | 89.9 (87.7..91.9) | 91.3 (90.9..91.8) | 96.4 (95.6..96.9) | 98.3 (98.0..98.8) | 98.3 (98.0..98.8) | 98.3 (98.0..98.8) | +8.43 |
| wx-mid | 99.5 ±0 (n3) | 99.5 ±0 (n3) | 99.5 ±0 (n3) | 99.5 (99.5..99.6) | 99.5 (99.5..99.6) | 99.6 (99.5..99.6) | +0.07 |
| wx-dusk | 96.3 | 96.3 | 98.2 | 98.2 | 98.2 | 98.2 | +1.90 |
| wx-cloud | 98.2 | 98.2 | 98.5 | 98.5 | 98.5 | 98.5 | +0.30 |
| wx-cloud-cv | 91.5 | 91.5 | 91.9 | 92.0 | 91.9 | 92.0 | +0.50 |
| wx-cloud-cv-ripple | 85.6 (85.4..85.9) | 85.6 (85.4..85.9) | 86.2 (86.1..86.2) | 87.3 (87.1..87.5) | 87.3 (87.1..87.5) | 87.3 (87.1..87.5) | +1.70 |
| wx-sweep-fall | 90.0 | 90.0 | 90.0 | 90.1 | 90.0 | 90.1 | +0.10 |
| mean of scenarios | 92.37 | 93.34 | 95.07 | 95.47 | 95.45 | 95.48 | |

### plant flat-hq, noise scenario Gaussian, SIM_SEED 1-3 (harv)

| scenario | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 87.2 | 91.7 | 97.9 | 98.0 | 98.0 | 98.0 | +10.80 |
| wx-low | 94.5 (93.6..95.5) | 95.0 (94.5..95.9) | 98.1 (98.0..98.2) | 97.1 (95.4..98.0) | 97.1 (95.4..98.0) | 97.1 (95.4..98.0) | +2.60 |
| wx-low-slow | 89.8 (89.2..90.1) | 88.8 (87.7..89.6) | 95.1 (94.0..95.8) | 97.9 (97.6..98.6) | 97.9 (97.6..98.6) | 97.9 (97.6..98.6) | +8.17 |
| wx-mid | 99.6 ±0 (n3) | 99.6 ±0 (n3) | 99.5 (99.5..99.6) | 99.5 (99.5..99.6) | 99.5 (99.5..99.6) | 99.6 ±0 (n3) | +0.00 |
| wx-dusk | 96.5 | 96.5 | 98.3 | 98.5 | 98.5 | 98.5 | +2.00 |
| wx-cloud | 98.4 | 98.4 | 98.3 | 98.5 | 98.5 | 98.6 | +0.20 |
| wx-cloud-cv | 92.1 | 92.1 | 92.4 | 92.2 | 92.1 | 92.2 | +0.10 |
| wx-cloud-cv-ripple | 85.8 (85.7..85.9) | 85.8 (85.7..85.9) | 86.3 (86.2..86.3) | 87.3 (87.2..87.4) | 87.3 (87.2..87.4) | 87.3 (87.2..87.4) | +1.53 |
| wx-sweep-fall | 90.1 | 90.1 | 90.1 | 90.1 | 90.1 | 90.2 | +0.10 |
| mean of scenarios | 92.66 | 93.11 | 95.11 | 95.46 | 95.45 | 95.49 | |

## 4. Limitations

- Q is the dominant unknown for the dawn ripple; field Q and the undamped-bench Q bracket it. Results on `-hq` are
  given for that reason.
- One noise realisation per Gaussian seed; deterministic scenarios (no noise in the scenario) have a single value
  per plant. The measured-noise replay loops a 19 s (flat) / ~10 s (fry) capture.
- The host sim drives pwmMax 2047 (LEDC-like); the boards run MCPWM 4103.
- Intermediate states are the tracker commits reverse-applied (`src/` hunks only) to the corrected tree, so every
  state shares today's sampler, rect-cal and plant. On the ideal plant these states reproduce the numbers quoted in
  the commit messages (wx-dawn 93.6 / 96.5 / 97.4 after de37d33 / ed5ffe5 / 9e1dd13), which checks the construction.
- "harvest % of GMPP" is PV-side energy over the reachable maximum. Converter losses do not lower it; in the
  cap-limited scenarios (wx-cloud-cv\*) losses raise PV power at the same output, so those differ from the ideal plant
  for that reason too.
