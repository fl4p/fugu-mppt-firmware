---
title: "Diode Emulation"
sidebar_position: 6
mdx:
  format: mdx
---

import {LsSweepChart, LsTimingChart, MSensitivityChart} from '@site/src/components/charts/DiodeEmulationCharts';

# Diode Emulation

## Nomenclature

```
HS:    high-side switch
LS:    low-side switch
cntrl: control switch        — HS in buck,  LS in boost
rect:  rectification switch  — LS in buck,  HS in boost
```

## Diode emulation

The rectifier switch (LS in a buck) can be left off and the coil-discharge current will
flow through its body diode. The converter then operates non-synchronously: trivial to
control, but you pay the body-diode `V_f` loss every cycle.

Synchronous operation turns the rect switch on for the window the body diode would
otherwise conduct, shorting it out and removing the `V_f` loss. The cost is that the
on-time has to be *right*: too long and the inductor current reverses (forced PWM —
charge flows back from output to input); too short and the body diode picks up the
remainder (small `V_f` loss, no danger).

**Diode emulation** in this firmware is the sensor-less computation of that on-time. In
CCM it is just `(1 − D)/f_sw`. In DCM it depends on the conversion ratio `M = V_o/V_i`.

The alternative is a current sensor with hardware zero-cross detection (analog
comparator into the gate driver `DIS`/`EN` pin, or fast ADC with µs-scale latency).
This board doesn't have one, hence the sensor-less approach below.

### CCM / DCM decision

The converter is in DCM whenever half the ripple current exceeds the dc output current:

$$
\frac{\Delta I_L}{2} > I_o
$$

For a buck:

$$
\Delta I_L = \frac{V_o}{f_{sw} \cdot L} \cdot \left(1 - \frac{V_o}{V_i}\right)
$$

`L` depends on dc bias current: powder cores droop with `H`. With `N` turns and effective
magnetic path length `l_e`,

$$
H_{dc} = \frac{N \cdot I_o}{l_e}, \qquad L(I_o) = L_0 \cdot \frac{\mu(H_{dc})}{\mu_i}
$$

where `μ_i` is the initial permeability and `μ(H)` the permeability under dc bias. Core
datasheets plot this ratio in percent, `%μ_i(H) = 100 · μ(H)/μ_i`.

Implementing the full `μ(H)` model needs the core datasheet and a per-board geometry
table. The firmware instead uses a flat margin: `L = L_0 · InductivityDcBias`
with `InductivityDcBias = 0.95` (`μ/μ_i` as a fraction, i.e. a 5 % reduction, `src/buck.h`).
This is adequate because (a) the CCM/DCM boundary region is narrow in normal operation, (b) the saturation curve of well-chosen
powder cores is flat through that region, and (c) the dc margin in CCM at high power is
much larger than the ripple, so a few-percent `L` error doesn't shift the boundary far.
`L_0` is per board (`coil.conf::L0`); `etc/measure_coil.py` measures it.

The DCM conversion ratio (Erickson, *Fundamentals of Power Electronics*, 3e, pp. 145,
597) is

$$
M_{DCM} = \frac{2}{1 + \sqrt{1 + 4 R_e / R}}, \qquad R_e = \frac{2 L \cdot f_{sw}}{D^2}
$$

where `R` is the load. The firmware doesn't use `M_DCM` directly — it measures `V_o` and
`V_i` — but the formula matters for understanding that in DCM `M ≠ D`.

### DCM rectifier on-time

During HS conduction the inductor sees `V_i − V_o`, so starting from zero,

$$
I_L(t) = \frac{V_i - V_o}{L} \cdot t, \qquad I_{L,\mathrm{peak}} = \frac{V_i - V_o}{L} \cdot t_{on,HS}
$$

During rect conduction the inductor sees `−V_o` and the current falls linearly. Solving
for the time it takes to reach zero,

$$
0 = I_{L,\mathrm{peak}} - \frac{V_o}{L} \cdot t_{on,LS}
$$

$$
\boxed{\; t_{on,LS} = t_{on,HS} \cdot \left(\frac{V_i}{V_o} - 1\right) = t_{on,HS} \cdot \left(\frac{1}{M} - 1\right) \;}
$$

With `t_{on,HS} = D / f_{sw}`,

$$
t_{on,LS,\mathrm{DCM}} = \frac{D}{f_{sw}} \cdot \left(\frac{1}{M} - 1\right)
$$

**`L` cancels.** Any error in the inductance model only affects whether we *think* we're
in DCM (the boundary check above), not the rect on-time itself. This is the load-bearing
property — voltage measurements set the timing; inductance just sets the regime.

Setting `M = D` (the CCM identity) recovers the CCM formula:

$$
t_{on,LS,\mathrm{CCM}} = \frac{1 - D}{f_{sw}}
$$

### Sensitivity to voltage measurement error

`M = V_o / V_i`. With independent fractional errors `ε_i` on `V_i` and `ε_o` on `V_o`,
the relative error in `M` is

$$
\frac{\Delta M}{M} \approx \varepsilon_o - \varepsilon_i
$$

The rect on-time depends on `M` through `1/M − 1`. To first order its sensitivity is

$$
\frac{\Delta t_{on,LS}}{t_{on,LS}} \approx -\frac{1}{1 - M} \cdot \frac{\Delta M}{M}
$$

So, to first order:

| operating point | 2% M-error → t_on,LS error |
|-----------------|----------------------------|
| `M = 0.5`       | ≈ 4%                       |
| `M = 0.8`       | ≈ 10%                      |
| `M = 0.9`       | ≈ 20%                      |
| `M = 0.95`      | ≈ 40%                      |

For finite errors the response is asymmetric: at `M = 0.95`, a +2 % error in `M` shortens
`t_on,LS` by 39.2 % and a −2 % error lengthens it by 40.8 %.

<MSensitivityChart />

The sensitivity blows up as `M → 1` (low `V_i − V_o`, where the falling slope `V_o/L` is
much steeper than the rising slope and small mistakes in the rising-slope estimate get
amplified). The controller needs a wider margin at high `M` — better to turn LS off
*slightly early* (pay a tiny body-diode `V_f` loss) than late (reverse current).

### Timing correction (`rect_offset_ns`)

The formula above gives the *ideal* zero-crossing time from the measured voltages. The LS
window realized in hardware differs from it for reasons the voltage model does not see:

- **edge delays** — gate-driver and FET propagation on both switches. A longer realized HS
  pulse raises the peak current and moves the zero crossing later; a delayed LS turn-off moves
  the actual turn-off later;
- **dead time** — the commanded LS span starts at the HS turn-off, but the LS channel conducts
  only after the HS→LS dead time (the body diode carries the current meanwhile);
- **sensing error** — `V_i`/`V_o` errors shift `M` and with it the ideal window.

`coil.conf::rect_offset_ns` is an **empirical net correction** added to the computed LS
window, `>0` = LS off later. It is found by measurement (below), not derived from any single
effect, and its sign is whatever the measurement gives. A pure LS turn-off delay on its own
would call for a *negative* value; measured values have been positive, so on those boards
the other effects outweigh it. It is per board: gate driver, FETs, layout.

The firmware adds it as a constant time, independent of `M` and of the HS on-time. That
suits edge delays; a voltage-sensing error scales with `t_on,HS` instead, so a calibration is
only exact near the operating point where it was measured.

It is stored as **nanoseconds** and converted to counts at boot (`ns·1e-9·tick_rate`, where
the tick rate is the full timer period × fsw or the MCPWM peripheral resolution — the same
basis as `boot_refresh_ns`). Storing a time rather than counts keeps the calibration valid
across changes of PWM resolution and `pwm_freq`. (Concretely: at the LEDC-equivalent 2048
counts/period at 39 kHz one count ≈ 12.5 ns, while MCPWM `bestTiming` gives ~4103
counts/period from a 160 MHz source clock, ≈ 6.25 ns/count — so the same correction would
*double* in counts between the two, which is exactly the breakage the ns form avoids.)

#### Measuring it

Hold a steep-edge HS duty in DCM and sweep LS on-time up from zero. Reading the
converter's output charge per period as the measured `I_out` assumes that `V_in` and
`V_out` stay stable and that the converter is in periodic steady state at every step (no
net charging of the input or output capacitors). A current-regulated load, or a PV input
whose voltage moves with the extracted power, can shift or hide the peak. Under those
conditions `I_out`

1. **rises** as LS replaces the body diode (recovering the `V_f` loss);
2. **peaks** when LS turns off exactly at the zero crossing (clean ideal triangle);
3. **falls** as LS is held past zero and reverse current starts.

<LsSweepChart />

The body-diode side is a broad plateau — turning LS off early just hands conduction
back to the diode, a small `V_f` loss, no cliff. The late side also leaves the peak with
zero slope, but the loss grows with the square of the overshoot and with a much larger
coefficient (about 60× the early side in the charted example), so the drop steepens quickly.
That asymmetry is what makes the peak locatable. The offset between the peak and the
firmware's predicted point `rectCtrlRatio(M)·pwmCtrl` is the timing correction.

`measure-coil ls [hs]` (on-device) or `etc/measure_coil.py --ls-sweep --hs N` brackets the
peak. `--apply` computes `peak − ideal − --apply-margin` (default 12 counts), converts it to
time, and writes `coil.conf::rect_offset_ns`. The helper infers the timer period from
`pwmCtrlMax/0.94`, which the current firmware no longer satisfies, so the written ns value can
be off by a few percent. Pass `--pwm-max`, or check the `rect_offset=… ns (… ct)` boot log line
after the reboot. Use
a steep-edge HS where the peak is genuinely locatable; flat plateaus yield no reliable
peak. Field values on two boards: +100 and +57 counts (at LEDC 12.5 ns/tick), different
gate-driver / FET combinations.

Both helpers locate the peak by fitting two half-parabolas that share an apex, one per side, which
is the shape derived above: the early side is shallow, the reverse-current side steep. The apex is
searched over the whole sweep. On the ideal model charted above (6.25 ns counts, default sweep of
0.5–1.4× the ideal window in 24 steps) the fit lands within one count of the true peak; with 1 mA of
measurement noise on a 0.25 A output the error stays around ±12 counts and leans early, the safe
side. The steep side's curvature also gives the `L` cross-check. The printed table shows the raw
maximum next to the fitted peak; on the flat side the raw maximum is not a good estimate.

### Implementation (`src/buck.h`)

The formulas above give the *ideal* LS on-time. What the firmware commands on top of that:

| step | behaviour |
|---|---|
| CCM/DCM decision (`computeDCM`) | Enters DCM when `ΔI_L > 2.0·I_o`, leaves when `ΔI_L ≤ 1.8·I_o` (hysteresis). Always DCM when `I_o < 0.1 A`. Effective forced PWM overrides all of this (never DCM). |
| M bias | Assumes a ±1 % error on each voltage and divides `M` by `0.99/1.01`, i.e. biases it ≈ 2 % toward the early-turn-off side, then clamps it off unity (`≤ 0.99` buck, `≥ 1.01` boost). On a buck the full bias applies up to `M ≈ 0.97`; between 0.97 and 0.99 the clamp eats into it; above 0.99 the clamped `M` is *below* the true one and the LS window comes out longer than ideal (at `M = 0.995` the ratio is 0.0101 against an ideal 0.0050, ≈ 2×). |
| low-current cut-off | In DCM the LS ratio is set to 0 when `I_o ≤ 0.01 A` or the low-side voltage is `< 1 V`; it stays at 0 until `I_o ≥ 0.04 A` and the voltage is back at `≥ 1 V`. The offset is still added, so the target becomes `rect_offset` counts. |
| DCM LS limit (`pwmRectMax`) | `pwmCtrl·ratio + rect_offset` counts, with first-order error-feedback dither of the rounding remainder (so the limit's time-average tracks the unrounded target). |
| CCM LS limit | `pwmMax − pwmCtrl − 1` (complementary). |
| final clamp | The limit is clamped to `[pwmRectMin, pwmMax − pwmCtrl − 1]`. On a buck `pwmRectMin` = `boot_refresh_ns` in ticks + the HS→LS dead time, for bootstrap refresh. |
| commanded LS count (`pwmRect`) | Fades in toward the limit (+1 count per update at first, then 1/64 of the remaining gap), follows it down immediately, drops to `pwmRectMin` on a large duty decrease, and stays at `pwmRectMin` while sync rect is off. |

The ideal `t_on,LS` is therefore the **pre-clamp target** of the limit, not the commanded
window. The dither's average carries over to the command only once it has settled at the
limit; while it ramps, or while a clamp is active, the average LS window differs from the
ideal. The LS channel also conducts for less than the commanded span: the span starts at the
HS turn-off, and the HS→LS dead time passes before the LS turns on.

Because of the clamp, a buck with sync rect off (console `sync off`) still issues a
`pwmRectMin` LS pulse every period, and the low-current cut-off issues the larger of
`pwmRectMin` and the offset. At zero load that pulse builds `V_o·t/L` of reverse current. Keep
this in mind when comparing scope traces with the ideal triangle.

### Failure modes at the LS boundary

<LsTimingChart />

Late: reverse current. Output charge is pulled back toward the input through the rect
switch — efficiency loss plus an anti-boost effect that can lift `V_in`.

Early: body diode conducts the remainder. Pure `V_f · I` loss, bounded, no instability.

The asymmetry is why the `M` bias leans toward the safe (early) side and why the measured
`rect_offset_ns` is applied minus a margin. The bias protects only over the unclamped range:
on a buck above `M ≈ 0.99`, wherever `pwmRectMin` exceeds the ideal window, and wherever the
offset over-corrects, the commanded LS window is longer than ideal and some reverse current flows.

## Boost converter

The roles flip — control switch is LS, rectifier is HS — but the derivation is the same.

$$
M_{CCM} = \frac{1}{1 - D}
$$

$$
t_{on,HS} = t_{on,LS} \cdot \frac{1}{M - 1} = \frac{D}{f_{sw}} \cdot \frac{1}{M - 1}
$$

where `D` is now the LS (control) duty. Same `L`-cancellation, same `M → 1` sensitivity
blow-up, here at the near-unity (low step-up) corner.

References

- Erickson, Maksimović. *Fundamentals of Power Electronics*, 3rd ed., ch. 5 and 15.
