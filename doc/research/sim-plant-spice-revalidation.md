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
| Soft HS turn-off | fix 3/4 | During the HS→LS dead time a positive coil current swings the node down along the undamped L-`ring_ct`·C resonance (closed form, cos/sin of the dead-time angle cached). If the node reaches −Vf the LS body diode clamps it (coil current from the swing energy, clamp time from the mean swing current, then diode slope); if not, the coil current is the exact LC value and the rest of the node energy is lost when the LS turns on hard. Before, the whole node energy was lost at every cycle, which made the old ring plant lose 0.24 W at 4-20 W where SPICE loses 0.02-0.06 W. |
| Edge recharge | fix 3 | The HS turn-on charges `ring_ct`·C from the node voltage it finds (ring end, diode clamp, LS 0 V, or Vout at rest) back to Vin, from the input, on every HS pulse. Before, this charge was only paid when the off phase ran through the ring branch, so with Vin ≤ Vout or without an off phase the dead-time swing gave the coil energy nobody paid for. |
| HS turn-off loss | fix 4 | `t_sw`: channel current falls linearly in `t_sw`, the node C takes the rest until the node reaches 0 V. Loss I²·t²/(24·C) at small current, → ½·Vin·I·t at large current, paid from the input. A plain ½·Vin·I·t overlap (first try) cost ~0.1 W at dawn currents, where SPICE shows 0.02 W in total. |
| Conduction | fix 4 | `r_hs`, `r_ls`: R·i against the coil while that switch conducts (one corrector step on the phase-mean current), so the loss lowers the delivered current at a given duty. The loss is R·∫i²: the mean-current part R·m²·t comes out of the coil current, the ripple part R·t·ΔI²/12 is paid from the input (a DCM triangle has ∫i² = b²t/3, 4/3 of m²t). |
| HS window | fix 5 | The realised HS on-time is capped at T (relevant only with `t_hs_ext` > 0). |
| Diode on the return path | fix 6 | `diode_vf` now also on the non-ring HS-body-diode return (node at Vin + Vf) and in the dead time; it is a `vconv.conf` key now (was only `vconv set diode_vf` / `setDiodeVf`). |

All new keys default to 0 (or 1 / 0.6 for `ring_ct` / `ring_cexp`, inert without `ring_v`), so a plant without them is
bit-identical to the old one. Checked against main (10014968) by the full mppt-sim stdout: `SIM_RING=ideal` on this
branch and main without it give identical output for a, b, c, d-cloud, d-noise, e, sanity, shade-cv, rcal-sun and the
nine wx-\* tracker scenarios plus wx-shade2. rcal differs in one log digit (789.30 vs 789.31 W) because it sets
`diode_vf` 0.7 V, which now also applies on the HS-body-diode return. vconv-test (1309/1309), dcm-current-test,
rect-cal-test, coil-fit-test, bms-cutoff-test and dawn-test pass unchanged.

**The wx-\* scenarios now run on the fry board plant by default.** Numbers measured before this change (the old
baselines, e.g. wx-dawn 97.4) need `SIM_RING=ideal` to reproduce.

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
build` (ESP-IDF 6.0.3, esp32s3, `CONFIG_FUGU_WITH_VCONV=y`) builds: fugu-firmware.bin 0x18fb30, 13 % of the smallest
app slot free; no warnings from the touched files. Not run on a device.

Cost on the LX7 (no hardware float divide; measured from the ELF): `stepOneCycleBuck` is 1761 B plus a 3-call shared
`diodeRun` helper (827 B on main by the reviewer's count, 1946 B at f42c3a6). Per cycle the new terms use no divide in
the defaults-off path and in the dawn regime (partial dead-time swing): 1/Vin, the edge-C impedance, cos/sin of the
dead-time angle and t_sw²/(24·C) are cached and rebuilt with the reciprocals when Vin moves > 1 %. A clamped
dead-time swing (larger currents) costs one sqrt and one divide; the t_sw loss above its knee one sqrt. The body-diode
freewheel keeps main's one divide when the current reaches zero.

## 2. Plant vs SPICE

Repeatable with `test/host-stub/vconv-spice-check.sh` (`-v` for every point); it exits 1 when the fitted plant leaves
a band or a known-bad plant does not. It drives the plant with stiff Vin/Vout at pwmMax 4103 / 38996 Hz, the firmware
DCM LS rule and **the bench's coil** (fry 80 µH, flat 51 µH) with the board's ring frequency, so it compares like with
like. The SPICE reference rows it embeds are in `doc/research/vconv-spice-data/` (`dawn.csv`, `near_boundary.csv`,
`p2.csv`, `ccm.csv`), extracted from the bench results by `extract.py` there, which also checks that the embedded
constants match them (0 mismatches). The SPICE report itself is `doc/research/vconv-vs-spice.md`. "ideal" = the plant
without ring, diode, timing and losses; "old ring" = the old mppt-sim ring boards (254 / 326 kHz, linear C, lossless),
measured with the f42c3a6 code. B = SPICE bench (undamped, no core loss), Brp = bench with the board's Q as a parallel
R (only run at fry-35/45 and flat-45).

Bands (fitted plant inside, each known-bad plant outside):

| check | band |
|---|---|
| dawn ripple, field Q | 0.75..1.02 × Brp where Brp was run; elsewhere 0.5 × (Brp estimated from its ratio to B at 45 V) .. 1.0 × B |
| dawn ripple, -hq | 0.7..1.05 × B |
| dawn η 8-30 W | min(B, Brp) − 0.6 pp .. max + 0.3 pp; without Brp, B − 1.0 pp .. B + 0.3 pp |
| dawn η 1-3 W | mean \|plant − B\| ≤ 0.045 |
| falling-step share | mean 0.05 (field Q) / 0.10 (-hq) .. 0.45, ≥ 3 sets > 0 |
| near Vin ≈ Vout | mean Pout within ±15 % per board |
| ~100 W staircase | mean ≤ 3 %, rms ≤ 5 % per board |
| CCM η | −0.2 pp .. +0.15 pp + the SPICE avalanche loss share (`ccm.csv`; the plant has no avalanche term) |

Self-tests in the script (each must fail): the ideal plant (40 checks out of band, among them P2 rms 7.2 / 7.4 % and
CCM 30 V / 3 A at 100 %), `t_sw` × 2 (6 checks: CCM η at 60 V 11.3 / 22.6 A, flat-55 η 8-30 W) and `ring_q` 60 on both
boards (3 checks: fry-35, fry-45, flat-45 ripple above 1.02 × Brp). Each also fails with the change on one board only.
`ring_ct` = 1 **passes** every band and cannot be made to fail without a band tuned to it: it moves dawn η 1-3 W
closer to B (mean error 0.024 vs 0.033) and CCM η at 75 V / 3.8 A closer to SPICE (99.03 vs 99.13 %, SPICE 98.75). The
fry/flat bench rows do not constrain `ring_ct`; 0.75 rests on the flu bench's rise transit alone. `ring_ct` 0.5 and
`t_sw` 50 ns fail (CCM 30 V / 3 A), `t_sw` 150 ns passes.

### Dawn: Pout(HS) ripple, falling steps, efficiency (stiff Vin, Vout 26.5 V, 45-count HS steps)

Ripple = half 5-95 % range of Pout / smooth log-log fit (Pout > 1 W); falling = share of 45-count steps where Pout
drops although HS rises.

| set | ripple ideal | old ring | **new** | new -hq | SPICE B | Brp | falling: old ring | **new** | new -hq | B | Brp | ripple period new / B (counts) |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| fry-30 | 0.005 | 0.095 | **0.170** | 0.183 | 0.196 | - | 0.33 | **0.52** | 0.50 | 0.51 | - | 855 / 900 |
| fry-35 | 0.009 | 0.114 | **0.162** | 0.193 | 0.208 | 0.174 | 0.10 | **0.15** | 0.20 | 0.28 | 0.17 | 630 / 645 |
| fry-45 | 0.003 | 0.141 | **0.199** | 0.262 | 0.286 | 0.223 | 0.00 | **0.00** | 0.09 | 0.22 | 0.03 | 465 / 465 |
| fry-55 | 0.001 | 0.170 | **0.199** | 0.319 | 0.338 | - | 0.00 | **0.00** | 0.00 | 0.04 | - | 321 / 345 |
| flat-30 | 0.009 | 0.060 | **0.093** | 0.134 | 0.141 | - | 0.19 | **0.24** | 0.36 | 0.36 | - | 645 / 756 |
| flat-35 | 0.009 | 0.061 | **0.084** | 0.142 | 0.165 | - | 0.00 | **0.00** | 0.05 | 0.16 | - | 461 / 472 |
| flat-45 | 0.009 | 0.071 | **0.088** | 0.200 | 0.256 | 0.103 | 0.00 | **0.00** | 0.00 | 0.23 | 0.00 | 309 / 330 |
| flat-55 | 0.004 | 0.070 | **0.075** | 0.219 | 0.287 | - | 0.00 | **0.00** | 0.00 | 0.25 | - | 231 / 244 |
| mean falling | | | | | | | 0.08 | **0.11** | 0.15 | 0.26 | (0.07) | |

| set | η 1-3 W ideal | old ring | **new** | B | Brp | η 8-30 W old ring | **new** | B | Brp |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| fry-30 | 1.000 | 0.930 | **0.930** | 0.940 | - | - | - | - | - |
| fry-35 | 1.000 | 0.915 | **0.923** | 0.910 | 0.900 | 0.989 | **0.9938** | 0.998 | 0.993 |
| fry-45 | 1.000 | 0.842 | **0.862** | 0.779 | 0.838 | 0.984 | **0.9918** | 0.997 | 0.989 |
| fry-55 | 1.000 | 0.777 | **0.845** | 0.871 | - | 0.975 | **0.9882** | 0.993 | - |
| flat-30 | 1.000 | 0.929 | **0.956** | 0.955 | - | 0.991 | **0.9935** | 0.998 | - |
| flat-35 | 1.000 | 0.927 | **0.961** | 0.974 | - | 0.991 | **0.9938** | 0.998 | - |
| flat-45 | 1.000 | 0.865 | **0.937** | 0.866 | 0.870 | 0.984 | **0.9907** | 0.997 | 0.983 |
| flat-55 | 1.000 | 0.764 | **0.923** | 0.969 | - | 0.974 | **0.9870** | 0.993 | - |

(the -hq η 8-30 W is within 0.003 of the field-Q values, its η 1-3 W up to 0.035 higher.)

The dead-time swing fix matters at dawn currents: at 75 V, 4.6 nF edge C, 76 µH and 50 mA at HS turn-off the node
swings only from 75 to ~70.7 V in 185 ns and the coil current rises to 0.163 A (exact LC); the f42c3a6 code applied
the diode slope from the start and gave 0.076 A. On the check the fix and the R·∫i² / recharge changes together move
the numbers by at most 0.006 in η 1-3 W and 0.004 in ripple.

- **Ripple period** matches SPICE within 0-15 % on every set (old ring: 10-25 % short for fry), from the 233 kHz
  ring and the Vin-dependent C.
- **Ripple amplitude** at the field Q sits below the damped bench (fry-35 0.93, fry-45 0.89, flat-45 0.85 × Brp) and
  at 0.26-0.87 of the undamped bench. The SPICE deck has no core loss, so the true Q is not known from SPICE: the
  field-fit Q (25 / 15) and the undamped bench bracket it. The `-hq` variant (Q 150 / 100) reproduces the undamped
  ripple within 6-24 %.
- **Falling steps**: the corrected plant has them where Vin ≤ 35 V (fry-30 0.52 vs SPICE 0.51, flat-30 0.24 vs 0.36),
  none at 45-55 V with the field Q, like the damped bench (0.00-0.03). The undamped bench has 0.04-0.25 there; the
  `-hq` plant reaches 0.09 at fry-45 and 0 elsewhere. The plant ripple is closer to a sine than the SPICE plateau-and-
  jump staircase, which is what makes falling steps at 45-55 V; that shape is not reproduced (measured, not fixed).
- **Efficiency**: 1-3 W mean |plant − B| 0.033 (old ring 0.056, ideal 0.09; -hq 0.033). 8-30 W within 0.4-0.6 pp of B
  and inside the B..Brp span where Brp exists.

### DCM near Vin ≈ Vout (Vin 30 V, LS near its minimum, 7 / 6 SPICE points)

| | ideal | old ring | **new** |
|---|--:|--:|--:|
| fry, mean Pout vs SPICE | +79 % | +45 % | **−1 %** (HS 3182: 5.27 vs 5.31 W; falls with HS like SPICE) |
| flat, mean Pout vs SPICE | +29 % | +16 % | **+8 %** |

### ~100 W DCM/CCM boundary staircase (flat 60 V 14 points, fry 75 V 13 points)

| | ideal mean / rms | old ring | **new** |
|---|--:|--:|--:|
| flat | +0.9 / 7.2 % | +0.4 / 4.5 % | **+0.3 / 3.7 %** |
| fry | +0.8 / 7.4 % | +0.7 / 6.4 % | **+0.1 / 1.8 %** |

The fry point at HS 1449 / LS 2571 (D·Vin = Vout within 0.05 %) is excluded: with stiff sources the corrected plant
runs into CCM there (94 W vs SPICE 75.6 W), which a soft PV source cannot do.

### CCM efficiency (synchronous, LS = pmax − HS − 1, soft battery)

| point | plant η | SPICE η | SPICE avalanche loss |
|---|--:|--:|--:|
| flat 60 V 3.8 / 11.3 / 22.6 A | 99.19 / 98.97 / 98.37 % | 98.94 / 98.88 / 98.38 % | 0.36 / 0.83 / 1.74 W |
| flat 30 V 3.0 / 11.3 A | 99.64 / 99.30 % | 99.52 / 99.31 % | - / 0.27 W |
| fry 75 V 3.8 A | 99.13 % | 98.75 % | 0.37 W |
| fry 60 V 11.3 / 22.6 A | 99.19 / 98.74 % | 99.11 / 98.74 % | 0.36 / 0.77 W |
| fry 30 V 3.0 / 11.3 A | 99.67 / 99.47 % | 99.54 / 99.48 % | - |

Mean |error| 0.11 pp (ideal and old ring: 100 % everywhere, 0.64 pp). The plant is 0.25 / 0.38 pp too efficient at
3.8 A and 60 / 75 V; the SPICE bench lists 0.36 / 0.37 W avalanche loss there (0.36 / 0.37 pp of 101 W), a loss the
plant has no term for. At higher currents the fitted `r_hs`, `r_ls` and `t_sw` absorb the avalanche share, so they
are effective values, not device parameters ("-" = avalanche not among the three largest SPICE loss terms).

### Not re-checked

LS zero-crossing shift (+40..90 counts in SPICE), return fraction of reverse energy (report §7) and the rect-cal knee
(§8) were not re-measured on the corrected plant; rcal-\* scenarios were not re-run.

## 3. Tracker commits on the corrected plant

All numbers in this section come from one sweep run on the Linux host pgx93 (`results2-pgx93.tsv`, 958 runs,
0 errors after one retry of 56 runs that hit a missing remote build stamp). mppt-sim output on pgx93 is not
bit-identical to macOS (libm differs), so a pgx93 run is a different noise realisation of the same case. No Mac rows
are mixed in, and every variant of a paired comparison ran on pgx93. The remote binaries were built from the same
trees and carry the sha256 of the Mac builds (checked by `offload run`). The f42c3a6 version of this section (Mac,
3 seeds) is superseded.

**Variants.** The tracker commits were reverse-applied (`src/` hunks only) to the corrected tree: pre (= d62bb11
tracker, parent of de37d33), +ed5ffe5 (v3), +9e1dd13 (v4), +8b68511 (v5) and +738cc16 (= main). +de37d33 alone (v2)
was not re-run: it was not in dispute.

**Noise and seeds.** wx-low, wx-low-slow, wx-mid and wx-cloud-cv-ripple set their own Gaussian ADC noise
(`noise_iout`/`noise_vout` in mppt-sim), and `SIM_SEED` picks its realisation (unset = the built-in seed, "d"). The
other five scenarios are noise-free. In the "measured" rows `SIM_NOISE=fry|flat` replays the converter's captured
sensor noise from `SIM_NOISE_OFS` seconds into the capture **on top of** the scenario's Gaussian noise, so those rows
depend on the seed as well. Coverage:

| cells | Gaussian | measured (fry, flat plants) |
|---|---|---|
| pre and main, fry/flat, noisy scenarios | 9 seeds (d, 1-8); wx-mid 3 | offsets 0/6/13 s × 6 seeds (d, 1-5) = 18; wx-mid 3 offsets, seed d |
| pre and main, -hq plants, noisy scenarios | 3 seeds, except wx-low and wx-low-slow 9 | - |
| disputed cells: 9e1dd13 (v3, v4) on wx-low, wx-low-slow, wx-dusk; 8b68511 (v4, v5) on wx-low-slow; 738cc16 (v5, main) on wx-cloud-cv; all four plants | 9 seeds for noisy scenarios | 18 for noisy scenarios |
| noise-free scenarios | 1 run (deterministic) | 3 offsets; wx-cloud-cv on v5/main 9 offsets (0-16 s, step 2) |

A seed has no effect on a noise-free scenario, so wx-dusk and wx-cloud-cv with Gaussian noise are single
deterministic runs, and their regression calls rest on the measured-noise offsets.

Cells are mean (min..max) over the runs; n is the run count per variant. Metric: SUMMARY "harvest % of GMPP".

### Summary (mean of the nine scenarios, pre vs main)

| plant, noise | pre | main | main − pre |
|---|--:|--:|--:|
| fry, Gaussian | 93.17 | 95.49 | +2.32 |
| fry, measured | 93.24 | 95.63 | +2.39 |
| flat, Gaussian | 92.63 | 95.48 | +2.85 |
| flat, measured | 89.86 | 93.52 | +3.66 |
| fry-hq, Gaussian | 93.13 | 95.30 | +2.17 |
| flat-hq, Gaussian | 91.36 | 95.61 | +4.25 |

No scenario mean is lower at main than at pre. Paired per seed/offset, pre → main loses more than 1 pp only on wx-low
of the -hq plants (fry-hq 4 of 9 seeds worse, worst −1.2, mean +0.29; flat-hq 3 of 9, worst −1.5, mean +1.63).

### Findings per commit (paired: same seed and offset, variant before vs after)

- **9e1dd13** (settle window) is **worse on wx-low with the scenario's Gaussian noise** on three of four plants:
  fry −0.43 pp (6 of 9 seeds worse, worst −1.4), fry-hq −1.17 pp (9 of 9 worse, worst −2.7), flat-hq −0.63 pp (6 of 9,
  worst −4.5); flat ±0.0 (2 of 9 worse). With measured noise it is better on wx-low: fry +0.68 (0 of 18 worse), flat
  +0.15 (5 of 18 worse, worst −2.2). On wx-low-slow it gains everywhere (+0.5 to +2.5 pp mean; 1 of 9 seeds worse on
  flat and fry-hq, 3 of 18 on flat measured). wx-dusk: fry +0.6, flat −0.1 (deterministic), measured +0.4 on both;
  the f42c3a6 "wx-dusk −0.3 on fry" does not reproduce here.
- **8b68511** (limiter hand-back keeps the slow-mode clock) changes nothing on wx-low-slow with Gaussian noise (bit
  for bit on all four plants) and is neutral with measured noise: fry −0.03 (6 of 18 worse, −0.4..+0.4), flat +0.09
  (6 of 18 worse, −0.9..+1.0). **Retraction:** the f42c3a6 report of an 8b68511 regression on flat measured-noise
  wx-low-slow (98.5 → 97.5, with a reversal-rate mechanism) came from a single Gaussian seed (`SIM_SEED=1`) on the Mac.
  It does not hold across seeds, and the mechanism is withdrawn.
- **738cc16** (45 s slow-mode hold) is **worse on fry wx-cloud-cv with measured fry noise**: −0.49 pp, 9 of 9 offsets
  worse (−0.2..−0.8), 91.60 → 91.11. Flat measured and all Gaussian runs: 0.0 to +0.1. On wx-low-slow it is
  identical to 8b68511 everywhere.
- **ed5ffe5** (only its effect into 9e1dd13 was re-run; vs pre it includes de37d33): wx-low, wx-low-slow and wx-dusk
  rise 1.5-6.5 pp over pre on every plant and noise.

### Paired deltas (pp of GMPP)

Every disputed cell; pre → main only where its mean is below −0.15 pp or one pair loses more than 1 pp.

| commit | plant, noise | scenario | mean | worst | best | pairs worse by > 0.1 / n |
|---|---|---|--:|--:|--:|--:|
| +9e1dd13 | fry, Gaussian | wx-low | -0.43 | -1.4 | +0.5 | 6 / 9 |
| +9e1dd13 | fry, Gaussian | wx-low-slow | +1.74 | +1.0 | +4.8 | 0 / 9 |
| +9e1dd13 | fry, Gaussian | wx-dusk | +0.60 | +0.6 | +0.6 | 0 / 1 |
| +9e1dd13 | fry, measured | wx-low | +0.68 | -0.1 | +2.6 | 0 / 18 |
| +9e1dd13 | fry, measured | wx-low-slow | +1.32 | +0.1 | +2.3 | 0 / 18 |
| +9e1dd13 | fry, measured | wx-dusk | +0.37 | +0.0 | +0.6 | 0 / 3 |
| +9e1dd13 | flat, Gaussian | wx-low | +0.01 | -1.2 | +1.4 | 2 / 9 |
| +9e1dd13 | flat, Gaussian | wx-low-slow | +0.84 | -1.1 | +1.7 | 1 / 9 |
| +9e1dd13 | flat, Gaussian | wx-dusk | -0.10 | -0.1 | -0.1 | 1 / 1 |
| +9e1dd13 | flat, measured | wx-low | +0.15 | -2.2 | +1.2 | 5 / 18 |
| +9e1dd13 | flat, measured | wx-low-slow | +0.48 | -0.7 | +3.1 | 3 / 18 |
| +9e1dd13 | flat, measured | wx-dusk | +0.43 | +0.0 | +0.7 | 0 / 3 |
| +9e1dd13 | fry-hq, Gaussian | wx-low | -1.17 | -2.7 | -0.2 | 9 / 9 |
| +9e1dd13 | fry-hq, Gaussian | wx-low-slow | +2.40 | -0.2 | +3.5 | 1 / 9 |
| +9e1dd13 | fry-hq, Gaussian | wx-dusk | +0.30 | +0.3 | +0.3 | 0 / 1 |
| +9e1dd13 | flat-hq, Gaussian | wx-low | -0.63 | -4.5 | +1.9 | 6 / 9 |
| +9e1dd13 | flat-hq, Gaussian | wx-low-slow | +2.52 | +0.6 | +3.8 | 0 / 9 |
| +9e1dd13 | flat-hq, Gaussian | wx-dusk | +0.40 | +0.4 | +0.4 | 0 / 1 |
| +8b68511 | fry, Gaussian | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 9 |
| +8b68511 | fry, measured | wx-low-slow | -0.03 | -0.4 | +0.4 | 6 / 18 |
| +8b68511 | flat, Gaussian | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 9 |
| +8b68511 | flat, measured | wx-low-slow | +0.09 | -0.9 | +1.0 | 6 / 18 |
| +8b68511 | fry-hq, Gaussian | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 9 |
| +8b68511 | flat-hq, Gaussian | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 9 |
| +738cc16 = main | fry, Gaussian | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 9 |
| +738cc16 = main | fry, Gaussian | wx-cloud-cv | +0.10 | +0.1 | +0.1 | 0 / 1 |
| +738cc16 = main | fry, measured | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 18 |
| +738cc16 = main | fry, measured | wx-cloud-cv | -0.49 | -0.8 | -0.2 | 9 / 9 |
| +738cc16 = main | flat, Gaussian | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 9 |
| +738cc16 = main | flat, Gaussian | wx-cloud-cv | +0.00 | +0.0 | +0.0 | 0 / 1 |
| +738cc16 = main | flat, measured | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 18 |
| +738cc16 = main | flat, measured | wx-cloud-cv | +0.00 | +0.0 | +0.0 | 0 / 9 |
| +738cc16 = main | fry-hq, Gaussian | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 9 |
| +738cc16 = main | fry-hq, Gaussian | wx-cloud-cv | +0.10 | +0.1 | +0.1 | 0 / 1 |
| +738cc16 = main | flat-hq, Gaussian | wx-low-slow | +0.00 | +0.0 | +0.0 | 0 / 9 |
| +738cc16 = main | flat-hq, Gaussian | wx-cloud-cv | +0.00 | +0.0 | +0.0 | 0 / 1 |
| pre -> main | fry-hq, Gaussian | wx-low | +0.29 | -1.2 | +3.2 | 4 / 9 |
| pre -> main | flat-hq, Gaussian | wx-low | +1.63 | -1.5 | +8.3 | 3 / 9 |

### Full tables

#### fry, Gaussian scenario noise (harv)

| scenario | n | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 1 | 90.0 | - | - | - | - | 97.3 | +7.30 |
| wx-low | 9 | 95.78 (93.8..97.6) | - | 98.38 (97.6..98.8) | 97.94 (97.2..98.4) | - | 97.94 (97.2..98.4) | +2.17 |
| wx-low-slow | 9 | 91.46 (90.1..93.3) | - | 96.64 (93.8..97.6) | 98.39 (97.8..98.7) | 98.39 (97.8..98.7) | 98.39 (97.8..98.7) | +6.93 |
| wx-mid | 3 | 99.47 (99.4..99.5) | - | - | - | - | 99.60 ±0 | +0.13 |
| wx-dusk | 1 | 96.2 | - | 97.7 | 98.3 | - | 98.3 | +2.10 |
| wx-cloud | 1 | 98.4 | - | - | - | - | 98.5 | +0.10 |
| wx-cloud-cv | 1 | 91.6 | - | - | - | 91.9 | 92.0 | +0.40 |
| wx-cloud-cv-ripple | 9 | 85.63 (85.2..85.9) | - | - | - | - | 87.27 (87.0..87.5) | +1.63 |
| wx-sweep-fall | 1 | 90.0 | - | - | - | - | 90.1 | +0.10 |
| mean of scenarios | | 93.17 | - | - | - | - | 95.49 | |

#### fry, measured noise replay + Gaussian scenario noise (harv)

| scenario | n | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 3 | 88.47 (84.3..90.7) | - | - | - | - | 97.20 (97.1..97.3) | +8.73 |
| wx-low | 18 | 95.89 (94.1..98.0) | - | 97.73 (95.9..98.5) | 98.41 (97.8..98.9) | - | 98.41 (97.8..98.9) | +2.52 |
| wx-low-slow | 18 | 92.59 (90.3..93.6) | - | 97.14 (96.2..98.4) | 98.46 (98.2..98.7) | 98.43 (98.1..98.6) | 98.43 (98.1..98.6) | +5.83 |
| wx-mid | 3 | 99.53 (99.5..99.6) | - | - | - | - | 99.60 ±0 | +0.07 |
| wx-dusk | 3 | 96.60 (96.4..96.8) | - | 98.10 (98.0..98.3) | 98.47 (98.3..98.6) | - | 98.47 (98.3..98.6) | +1.87 |
| wx-cloud | 3 | 98.43 (98.3..98.6) | - | - | - | - | 98.60 ±0 | +0.17 |
| wx-cloud-cv | 9 | 90.67 (90.6..90.7) | - | - | - | 91.60 (91.3..91.9) | 91.11 (90.8..91.3) | +0.44 |
| wx-cloud-cv-ripple | 18 | 86.85 (86.6..87.1) | - | - | - | - | 88.66 (88.4..88.9) | +1.81 |
| wx-sweep-fall | 3 | 90.10 ±0 | - | - | - | - | 90.17 (90.1..90.2) | +0.07 |
| mean of scenarios | | 93.24 | - | - | - | - | 95.63 | |

#### flat, Gaussian scenario noise (harv)

| scenario | n | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 1 | 85.8 | - | - | - | - | 97.3 | +11.50 |
| wx-low | 9 | 93.91 (89.1..96.1) | - | 98.06 (97.0..98.7) | 98.07 (97.3..98.5) | - | 98.07 (97.3..98.5) | +4.16 |
| wx-low-slow | 9 | 91.96 (90.3..92.9) | - | 97.37 (96.9..97.8) | 98.21 (95.9..98.7) | 98.21 (95.9..98.7) | 98.21 (95.9..98.7) | +6.26 |
| wx-mid | 3 | 99.50 ±0 | - | - | - | - | 99.57 (99.5..99.6) | +0.07 |
| wx-dusk | 1 | 96.5 | - | 98.2 | 98.1 | - | 98.1 | +1.60 |
| wx-cloud | 1 | 98.3 | - | - | - | - | 98.4 | +0.10 |
| wx-cloud-cv | 1 | 91.8 | - | - | - | 92.1 | 92.1 | +0.30 |
| wx-cloud-cv-ripple | 9 | 85.80 (85.4..86.3) | - | - | - | - | 87.39 (87.2..87.6) | +1.59 |
| wx-sweep-fall | 1 | 90.1 | - | - | - | - | 90.2 | +0.10 |
| mean of scenarios | | 92.63 | - | - | - | - | 95.48 | |

#### flat, measured noise replay + Gaussian scenario noise (harv)

| scenario | n | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 3 | 83.97 (75.0..91.2) | - | - | - | - | 97.33 (97.3..97.4) | +13.37 |
| wx-low | 18 | 95.10 (91.6..96.4) | - | 98.08 (96.8..98.8) | 98.23 (96.6..98.9) | - | 98.23 (96.6..98.9) | +3.13 |
| wx-low-slow | 18 | 94.82 (93.6..96.1) | - | 97.93 (95.6..98.6) | 98.41 (97.3..99.0) | 98.50 (97.6..99.1) | 98.50 (97.6..99.1) | +3.68 |
| wx-mid | 3 | 99.30 ±0 | - | - | - | - | 99.70 ±0 | +0.40 |
| wx-dusk | 3 | 96.27 (96.0..96.4) | - | 98.10 (97.9..98.3) | 98.53 (98.3..98.7) | - | 98.53 (98.3..98.7) | +2.27 |
| wx-cloud | 3 | 98.37 (98.2..98.6) | - | - | - | - | 98.60 ±0 | +0.23 |
| wx-cloud-cv | 9 | 78.87 (78.8..79.0) | - | - | - | 83.11 (83.0..83.2) | 83.11 (83.0..83.2) | +4.24 |
| wx-cloud-cv-ripple | 18 | 71.76 (71.3..72.6) | - | - | - | - | 77.22 (77.0..77.4) | +5.46 |
| wx-sweep-fall | 3 | 90.33 (90.2..90.5) | - | - | - | - | 90.47 (90.4..90.5) | +0.13 |
| mean of scenarios | | 89.86 | - | - | - | - | 93.52 | |

#### fry-hq, Gaussian scenario noise (harv)

| scenario | n | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 1 | 90.0 | - | - | - | - | 96.5 | +6.50 |
| wx-low | 9 | 96.61 (95.1..97.3) | - | 98.07 (97.3..98.6) | 96.90 (95.5..98.3) | - | 96.90 (95.5..98.3) | +0.29 |
| wx-low-slow | 9 | 90.14 (88.1..91.7) | - | 95.98 (95.0..97.3) | 98.38 (97.1..98.8) | 98.38 (97.1..98.8) | 98.38 (97.1..98.8) | +8.23 |
| wx-mid | 3 | 99.50 ±0 | - | - | - | - | 99.60 ±0 | +0.10 |
| wx-dusk | 1 | 96.6 | - | 98.1 | 98.4 | - | 98.4 | +1.80 |
| wx-cloud | 1 | 98.3 | - | - | - | - | 98.5 | +0.20 |
| wx-cloud-cv | 1 | 91.5 | - | - | - | 91.9 | 92.0 | +0.50 |
| wx-cloud-cv-ripple | 3 | 85.50 ±0 | - | - | - | - | 87.33 (87.2..87.4) | +1.83 |
| wx-sweep-fall | 1 | 90.0 | - | - | - | - | 90.1 | +0.10 |
| mean of scenarios | | 93.13 | - | - | - | - | 95.30 | |

#### flat-hq, Gaussian scenario noise (harv)

| scenario | n | pre (d62bb11) | +de37d33 | +ed5ffe5 | +9e1dd13 | +8b68511 | +738cc16 = main | main - pre |
|---|--:|--:|--:|--:|--:|--:|--:|--:|
| wx-dawn | 1 | 75.2 | - | - | - | - | 98.4 | +23.20 |
| wx-low | 9 | 95.42 (89.8..97.6) | - | 97.69 (96.2..98.3) | 97.06 (93.8..98.1) | - | 97.06 (93.8..98.1) | +1.63 |
| wx-low-slow | 9 | 89.27 (87.1..90.8) | - | 95.78 (95.1..96.4) | 98.30 (96.9..99.0) | 98.30 (96.9..99.0) | 98.30 (96.9..99.0) | +9.03 |
| wx-mid | 3 | 99.53 (99.5..99.6) | - | - | - | - | 99.60 ±0 | +0.07 |
| wx-dusk | 1 | 96.4 | - | 98.2 | 98.6 | - | 98.6 | +2.20 |
| wx-cloud | 1 | 98.5 | - | - | - | - | 98.5 | +0.00 |
| wx-cloud-cv | 1 | 92.0 | - | - | - | 92.2 | 92.2 | +0.20 |
| wx-cloud-cv-ripple | 3 | 85.80 (85.7..86.0) | - | - | - | - | 87.60 (87.4..87.8) | +1.80 |
| wx-sweep-fall | 1 | 90.1 | - | - | - | - | 90.2 | +0.10 |
| mean of scenarios | | 91.36 | - | - | - | - | 95.61 | |

## 4. Limitations

- Q is the dominant unknown for the dawn ripple; field Q and the undamped-bench Q bracket it. Results on `-hq` are
  given for that reason.
- Seed coverage is trimmed (section 3): de37d33 alone was not re-run, the -hq plants have 3 seeds outside the
  disputed cells, and noise-free scenarios with Gaussian noise are single deterministic runs. The measured-noise
  replay loops a 19 s (flat) / ~10 s (fry) capture, so offsets are not independent realisations.
- Tracker numbers are from pgx93 (Linux); the Mac gives other noise realisations of the same cases.
- `ring_ct` is not constrained by the fry/flat bench rows (section 2): 1.0 passes every band.
- The host sim drives pwmMax 2047 (LEDC-like); the boards run MCPWM 4103.
- Intermediate states are the tracker commits reverse-applied (`src/` hunks only) to the corrected tree, so every
  state shares today's sampler, rect-cal and plant. On the ideal plant these states reproduce the numbers quoted in
  the commit messages (wx-dawn 93.6 / 96.5 / 97.4 after de37d33 / ed5ffe5 / 9e1dd13), which checks the construction.
- "harvest % of GMPP" is PV-side energy over the reachable maximum. Converter losses do not lower it; in the
  cap-limited scenarios (wx-cloud-cv\*) losses raise PV power at the same output, so those differ from the ideal plant
  for that reason too.
