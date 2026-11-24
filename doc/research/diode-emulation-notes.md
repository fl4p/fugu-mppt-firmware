*this document is an LLM generated placeholder*

# Diode emulation: simulation and field record

Research-log material moved out of the published page `website/docs/internals/diode-emulation.md`
(2026-11-24 docs audit). The rules, thresholds and formulas stay on the page; this file keeps the
board-model sweeps, harvest statistics and field observations they were derived from. Section names
refer to the headings of that page.

## Reverse-current drain near D = M

Onset of the forced-CCM-like state without the drain limit: with a stiff source and battery the
board models showed this from a ratio reading 1.5 % off (−0.13 A, two models), 2 % off (−0.21 to
−0.39 A, all four), 2.5 % off (−1.5 to −1.9 A) and 3 % off (down to −9.7 A).

Checked over the whole DCM range (`V_o` = 26.8 and 29.2 V, `V_i` from `V_o + 0.6` V to 80 V,
every HS count from 200 to `D = M`, stiff source and battery) on the four board models, with
readings of `V_o` up to 4 % low or `V_i` up to 2.5 % high, both together, and timing offsets of 0 to
+125 ns. Points with the reading error past the margin, before → after (count of points where the
window, LS above its minimum pulse, drives the mean output current negative; most negative value;
offset 0; the 1–1.5 % rows are from an earlier sweep with a 0.02·T budget alone, which the
floor only lengthens):

| reading error | before, per model | after |
|---|---|---|
| none, `V_o` −1 % or `V_i` +1 % | 0–10 points, ≥ −0.01 A | 0 |
| `V_o` −1.5 % or `V_i` +1.5 % | 760–880 points, −0.13 A (two models) | 0 |
| `V_o` −2 % | 1220–3610 points, −0.38 to −0.39 A | 0 |
| `V_i` +2.5 % | 4440–7150 points, −1.5 to −1.8 A | 0 |
| `V_o` −3 % | 8840–11370 points, −6.7 to −9.7 A | 0 |
| `V_o` −2 % and `V_i` +2 % | 16580–18500 points, −27 to −38 A | 0 |
| `V_o` −4 % | 17230–19100 points, −29 to −40 A | 0–10 points, ≥ −0.01 A |

No point turned a non-negative mean current negative. A budget of $0.0198 \cdot T$ alone (no time
floor) leaves points negative once the total ratio error exceeds about 3 %: `V_o` −2 % and `V_i`
+2 % together give down to −0.22 A, `V_o` −4 % down to −0.35 to −0.56 A. Past about 4 % the floor
also runs out.

Gain and efficiency: the mean output current at the same HS count drops by up to 2 A in the board
models. Cost with exact readings, from the plant efficiency at the points where the limit changes
the LS count: at most 0.01–0.04 percentage points at `M` < 0.5, 0.03–0.20 at `M` = 0.5–0.7,
0.13–0.51 at 0.7–0.8 and 0.20–0.55 at 0.8–0.9, where the DCM output power is at most a few tens of
watts; the mean extra loss over those points is below 10 mW. In closed-loop PV simulations near
`V_i ≈ V_o` (low-voltage strings, partial shade, dawn, dusk, low light, a battery at the voltage
cap) the harvest changed by −1.2 to +0.7 percentage points in single runs, with identical
protection trips. A paired dusk study (four board models, three noise conditions, 76 pairs each)
found no cost with exact readings (−0.1 ± 0.1 percentage points). With `V_o` read 2.5 % low on the
two models whose ring leaves reverse current at HS turn-on, the harvest dropped by a median of 0.85
percentage points: without the limit, the late window carries reverse current over and runs the
converter forced-CCM-like at `D ≈ M`, where the MPP tracker happens to track better on a PV source.
The limit returns these cases to the exact-reading behaviour. A smaller time floor recovers part of
this only by giving up the combined-error case above.

Repeated at the 4103-count MCPWM resolution (four board models, 20 dusk runs each, low side turning
off at the modelled zero crossing), the limit cost nothing with exact readings (+0.17 ± 0.13
percentage points) and 0.48 ± 0.14 with `V_o` read 2.5 % low. It removed the 19 `minpulse-M` stops
that the late window caused without it (29 at the 2047-count resolution).

## Measuring it (rect_offset_ns)

Field values on two boards with different gate-driver / FET combinations are +100 and +57
counts (at LEDC 12.5 ns/tick).

## Self-calibration: signal size

The signal-size table is a lossless estimate. On an 80 µH board at low irradiance, the measured
late side was about 30 times steeper (−20 to −70 mA at 70 counts past the ideal window, −160 mA at
140 counts), and the early side fell by 10–15 mA across the sweep instead of rising by a few mA.

## Self-calibration: voltage changes (simulation)

- A 1% (also 0.5% and 0.3%) `V_o` dip during the steps 100–200 counts past the zero crossing is
  rejected; since it repeats at each LS it ends the run with `sense follows LS`. Without the step
  check, a fit learned +36 counts (a later LS turn-off) from the 1% dip.
- A 0.19% `V_o` square wave, just under the limit, with periods of 0.9–2.3 s against the 1.05 s
  step cycle and a plant 0× or 2× as sensitive to voltage as the model, biases the result by −17
  to +24 counts. After the applied margin, the worst case leaves the LS 4 counts (25 ns)
  late, where the reverse current is well below 1 mA.
- A `V_i` rise confined to the steps past the crossing, with a plant that does not follow `V_i`,
  is learned toward a shorter LS (−22 counts at 0.2%) or rejected (0.5% and above).
- A linear `V_o` drift of ±2.5% over 30 s is learned within a few counts.
- Replicas of three low-irradiance runs (soft panel, 30× knee, falling early side, swinging bus)
  calibrate within ±5 counts. Before these changes they all ended with `V step`. A cloud that
  moves `V_i` by 2.6% still ends the run with `Vin shift`.

## Self-calibration: sense path (field)

On a 51 µH board whose shared current/voltage monitor has no input filter, three field runs at
about 1.6–1.7 A rose by 36–52 mA between 0.5× and 1× ideal and never dropped; past ideal the `V_o`
reading rose by up to 1.3% on every retry. Replayed, each run ends with `Iout follows LS` at
0.73–0.93× ideal, and with `sense follows LS` within 0.07× of ideal when only the `V_o` reading
follows the LS. Three runs on an 80 µH board rose by at most 2.3 mA, the simulated scenarios by
at most 12 mA.

## Self-calibration: noise replicas

On a replica of a full-sun run at 78 V, 20 runs per noise level learned within a standard deviation
of 2.0 counts at 10 mA of sensor noise per sample and 3.3 counts at 30 mA, without a guard action.
At 100 mA the bend is lost in the noise in 2 of 20 runs (`no knee`, `few points`). Without the 10
extra points the spread rises to 1.8 and 4.7 counts and the result moves 26 counts earlier.

## Self-calibration: learned offsets in the board models

In the board models (no driver or gate delay, so the true offset is 0) over `V_i` = 1.4–2.9·`V_o`,
`I_sc` 3 and 6 A, two fill factors and three noise seeds, the run learns a mean of −0.29 to +0.34 µs
per model with a spread (standard deviation) of 0.15–0.26 µs and applies −1.25 to +0.13 µs, positive
values only from calibration points below `V_i = 2·V_o`. Referenced to the ideal window alone with a
downward bend only, the same runs learn with a spread of 0.39–0.46 µs, and up to +1.5 µs late below
`V_i = 2·V_o`, where the current bends up at the crossing.

## Late results (board models)

In the board models, +125 ns with `V_o` read 1.7% low reached the complementary limit in a band up
to 20 HS counts wide below `D = M` (down to −1.3 A mean output current) and, near
`V_i = 1.1–1.2·V_o`, left undrained reverse current up to 130 counts below `D = M` (down to −0.23 A).

Each part of the positive-offset rule alone was not enough in the board models: the gaps alone left
negative mean current at `V_o` read 2% low, the `M` limit with the fixed `T/32` gap alone left it
near `M = 0.79`.

Checked over the whole DCM range (`V_i` = 31–80 V, `V_o` = 26.8 and 29.2 V, every HS count from the
LS minimum pulse to `D = M`) on four board models, with offsets of +63, +125 and +250 ns and readings
of `V_o` 0 to 2.5% low or `V_i` up to 2.5% high: an offset never reaches the complementary limit and
never turns the mean output current negative where offset 0 keeps it positive. Against offset 0 it
lowers the mean output current by at most 0.12 A at +125 ns and `V_o` 1.7% low, 0.17 A at 2% low,
and 0.42 A at 2.5% low.

In the board models with exact voltage readings, +125 ns leaves at most 0.06–0.14 A of reverse
current at LS turn-off (0.1–0.7 µJ in the coil per period) and up to 14 mW of extra loss; with
readings that use up the 2% margin, 0.30–0.35 A (offset 0: 0.25–0.28 A) and up to 55 mW.

## Bounds: save duration

On a bench ESP32-S3 the `coil.conf` save, including the rename, took about 0.4 s (about 0.05 s for
an in-place rewrite).
