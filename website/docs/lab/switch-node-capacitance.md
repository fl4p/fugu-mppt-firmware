---
title: Switch-node capacitance
sidebar_position: 8
---

# Switch-node capacitance

The switch-node capacitance `C_sw` and the coil `L` set the frequency of the DCM coil ring (see
[DCM coil ringing](../internals/dcm-ringing.mdx#ring-frequency-per-board)). `C_sw` depends on the FETs and layout of
each board, so measure it per board. This page describes the capture and the fit, the result for one board, and the
limits of the method.

## Operating point

Run the converter in DCM: diode emulation at a load below the CCM boundary, `I_out < ΔI_L/2`. Above that boundary
the coil current never reaches zero, so no ring forms. Forced PWM holds the LS on through the zero crossing and
suppresses the ring at any load.

## Trigger

Probe the switch node on one channel and the LS gate on another, and trigger on the falling edge of the LS gate. The
LS gate is ground-referenced and driven every cycle, so it marks the start of the ring reliably.

A trigger on the switch node itself fires on the node's fastest transition. That is the HS turn-on edge (a few
hundred ns or less), not the ring transit of about 1 µs. The capture then shows a clean, plausible-looking rise that
isn't the ring.

:::warning Probing the switch node
The switch node swings between ground and `V_in`. Use a differential probe or an isolated scope, as described in
[Measurements](measurements.md#scope-capture).
:::

## Fit

Fit a cosine to the unclamped part of the rise, from 3 % to 93 % of the swing:

$$
v(t) = V_c - A \cdot \cos(\omega \cdot (t - t_0))
$$

and take the capacitance from the fitted angular frequency and the measured coil inductance:

$$
C_{sw} = \frac{1}{\omega^2 \cdot L}
$$

Take `C_sw` from the fitted `ω` rather than inverting the quarter period `T/4 = (π/2)·√(L·C_sw)` from the 0 V to
`V_in` transit. That transit isn't a quarter cycle from rest:

- The node swings about a midpoint `V_c`, so `dV/dt` peaks near mid-swing, not at the rail.
- The HS body diode clamps the node one diode drop above `V_in`, which cuts the cosine short before its peak.

## Board C result

Board C has one IPP022N12NM6 on the low side and an 81–83 µH coil (83 µH measured with a 4-wire DMM, 81.2 µH from
`A_L·N²`). The capture ran with the output open, near-zero current and a low duty, over several events.

The fit gives the following values:

| Quantity | Value |
|---|--:|
| `ω` | 2.012·10⁶ rad/s (spread 0.5 % across events) |
| `f_r` | 320.2 kHz |
| `V_c` | 37.84 V |
| `A` | 41.96 V |
| Unclamped peak `V_c + A` | 79.8 V |
| Clamp | 72.84 V (`V_in + V_f`) |

With `L` = 81–83 µH, `C_sw` is 2.98–3.04 nF.

## Board D

Board D has two 100 V FETs on each side (2× IPP050N10NF2S HS, 2× IPP039N10N5 LS) and a 40 µH coil. Its `C_sw` is
unmeasured.

Its existing captures were triggered on the switch node, and they don't fit the resonant model:

- The 0 V to `V_in` transit falls by 30 % and 43 % over the two `V_in` sweeps, and not monotonically.
- `dV/dt` peaks at 50–60 % of the swing.
- The resonant fit does not give a stable `C_sw` across `V_in`, and diverges on some low-voltage points.

Measuring the value needs a new capture triggered on the LS gate.

## Limitations

Two models can explain the rise after the LS turns off:

- **Resonance** of `L` with `C_sw`. The angular frequency `ω` does not depend on `V_in`.
- **Constant-current charge** of `C_sw` by the negative coil current that the LS minimum on-time pumps,
  `I_neg = V_out·t_LS/L`. The transit is then proportional to `V_in`.

A straight line fits 15–85 % of every rise in these captures to R² ≈ 0.999, board C's included. Over that span a
cosine and a ramp are barely distinguishable, so the fit alone can't separate the two models. The board C value rests
on two observations: its fit converges, where board D's does not, and the node shows the body-diode clamp.

A `V_in` sweep with the gate trigger would separate the models. Compare the fitted `ω` across the sweep: a resonance
keeps it constant. The 0 V to clamp transit changes with `V_in` even for a resonance, because the clamp position on
the cosine depends on `V_in/V_out`.
