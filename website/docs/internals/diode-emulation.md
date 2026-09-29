---
title: "Diode Emulation"
sidebar_position: 6
---

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

$$\frac{\Delta I_L}{2} > I_o$$

For a buck:

$$\Delta I_L = \frac{V_o}{f_{sw} \cdot L} \cdot \left(1 - \frac{V_o}{V_i}\right)$$

`L` depends on dc bias current: powder cores droop with `H`. With `N` turns and effective
magnetic path length `l_e`,

$$H_{dc} = \frac{N \cdot I_o}{l_e}, \qquad L(I_o) = \frac{\%\mu_i(H_{dc})}{\mu_i} \cdot L_0$$

Implementing the full `%μ(H)` model needs the core datasheet and a per-board geometry
table. The firmware instead uses a flat margin: `L = L_0 · InductivityDcBias`
with `InductivityDcBias = 0.95` (a 5 % reduction, `src/buck.h`). This is adequate because (a) the CCM/DCM
boundary region is narrow in normal operation, (b) the saturation curve of well-chosen
powder cores is flat through that region, and (c) the dc margin in CCM at high power is
much larger than the ripple, so a few-percent `L` error doesn't shift the boundary far.
`L_0` is per board (`coil.conf::L0`); `etc/measure_coil.py` measures it.

The DCM conversion ratio (Erickson, *Fundamentals of Power Electronics*, 3e, pp. 145,
597) is

$$M_{DCM} = \frac{2}{1 + \sqrt{1 + 4 R_e / R}}, \qquad R_e = \frac{2 L \cdot f_{sw}}{D^2}$$

where `R` is the load. The firmware doesn't use `M_DCM` directly — it measures `V_o` and
`V_i` — but the formula matters for understanding that in DCM `M ≠ D`.

### DCM rectifier on-time

During HS conduction the inductor sees `V_i − V_o`, so starting from zero,

$$I_L(t) = \frac{V_i - V_o}{L} \cdot t, \qquad I_{L,\mathrm{peak}} = \frac{V_i - V_o}{L} \cdot t_{on,HS}$$

During rect conduction the inductor sees `−V_o` and the current falls linearly. Solving
for the time it takes to reach zero,

$$0 = I_{L,\mathrm{peak}} - \frac{V_o}{L} \cdot t_{on,LS}$$

$$\boxed{\; t_{on,LS} = t_{on,HS} \cdot \left(\frac{V_i}{V_o} - 1\right) = t_{on,HS} \cdot \left(\frac{1}{M} - 1\right) \;}$$

With `t_{on,HS} = D / f_{sw}`,

$$t_{on,LS,\mathrm{DCM}} = \frac{D}{f_{sw}} \cdot \left(\frac{1}{M} - 1\right)$$

**`L` cancels.** Any error in the inductance model only affects whether we *think* we're
in DCM (the boundary check above), not the rect on-time itself. This is the load-bearing
property — voltage measurements set the timing; inductance just sets the regime.

Setting `M = D` (the CCM identity) recovers the CCM formula:

$$t_{on,LS,\mathrm{CCM}} = \frac{1 - D}{f_{sw}}$$

### Sensitivity to voltage measurement error

`M = V_o / V_i`. With independent fractional errors `ε_i` on `V_i` and `ε_o` on `V_o`,
the relative error in `M` is

$$\frac{\Delta M}{M} \approx \varepsilon_o - \varepsilon_i$$

The rect on-time depends on `M` through `1/M − 1`. Its sensitivity is

$$\frac{\Delta t_{on,LS}}{t_{on,LS}} = -\frac{1}{1 - M} \cdot \frac{\Delta M}{M}$$

So:

| operating point | 2% M-error → t_on,LS error |
|-----------------|----------------------------|
| `M = 0.5`       | 4%                         |
| `M = 0.8`       | 10%                        |
| `M = 0.9`       | 20%                        |
| `M = 0.95`      | 40%                        |

The sensitivity blows up as `M → 1` (low `V_i − V_o`, where the falling slope `V_o/L` is
much steeper than the rising slope and small mistakes in the rising-slope estimate get
amplified). The controller needs a wider margin at high `M` — better to turn LS off
*slightly early* (pay a tiny body-diode `V_f` loss) than late (reverse current).

### Hardware delay (`rect_offset_ns`)

The formula above gives the *ideal* zero-crossing time. In hardware a constant delay
sits between the commanded LS-off count and the moment the FET actually stops
conducting: gate-driver propagation, FET turn-off, switch-node decay. This delay is

- **per board** — gate driver part, FET selection, layout parasitics;
- **independent of `M`** — it's a fixed propagation time, not a ratio;
- **independent of `L`** — same reason the ideal time was;
- **a fixed time** — nanoseconds, set by the gate-drive circuit and MOSFET, not by the PWM.

Because it's a fixed *time*, it's stored in `coil.conf::rect_offset_ns` as **nanoseconds**
and converted to counts at boot (`ns·1e-9·tick_rate`, where the tick rate is the full timer period × fsw or
the MCPWM peripheral resolution — the same basis as `boot_refresh_ns`); `>0` = LS off later, toward the zero crossing. Storing the time, not
counts, makes the calibration **invariant to the PWM resolution and `pwm_freq`** — the
same physical delay maps to the right count on any driver. (Concretely: at the LEDC-equivalent
2048 counts/period at 39 kHz one count ≈ 12.5 ns, while MCPWM `bestTiming` gives ~4103
counts/period from a 160 MHz source clock, ≈ 6.25 ns/count — so a fixed delay would *double*
in counts between the two, which is exactly the breakage the ns form avoids.)

#### Measuring it

Hold a steep-edge HS duty in DCM and sweep LS on-time up from zero. `I_out`

1. **rises** as LS replaces the body diode (recovering the `V_f` loss);
2. **peaks** when LS turns off exactly at the zero crossing (clean ideal triangle);
3. **falls** as LS is held past zero and reverse current starts.

```
 Iout                      peak = LS off exactly at i_L=0  (clean ideal triangle)
   |                       .--''''--.
   |                  .--''          ''--.
   |              .-''                    ''-.        reverse current:
   | body-diode .-'                          '-.     LS held past zero,
   |  only   _.-'                                '   i_L goes negative,
   |     _.-'                                         charge pulled back
   +----+-----------------------+-------------------> LS on-time (counts)
      LS=0                  t_on,LS = (1/M - 1) * t_on,HS
   (Vf loss, low Iout)         = rectCtrlRatio(M) * pwmCtrl
```

The body-diode side is a broad plateau — turning LS off early just hands conduction
back to the diode, a small `V_f` loss, no cliff. The informative feature is the sharp
reverse-current edge just past the peak. The offset between the peak and the firmware's
predicted point `rectCtrlRatio(M)·pwmCtrl` is the hardware offset.

`measure-coil ls [hs]` (on-device) or `etc/measure_coil.py --ls-sweep --hs N` brackets the
peak. `--apply` computes `peak − ideal − --apply-margin` (default 12 counts), converts it to
time, and writes `coil.conf::rect_offset_ns`. Use
a steep-edge HS where the peak is genuinely locatable; flat plateaus yield no reliable
peak. Field values on two boards: +100 and +57 counts (at LEDC 12.5 ns/tick) — different
boards, different gate-driver / FET combinations, as expected for an `L`- and
`M`-independent constant.

### Failure modes at the LS boundary

```
   I_L ▲
       │      ▲ I_peak
       │     ╱╲
       │    ╱  ╲
       │   ╱    ╲                     ideal: LS off exactly at ZC
       │  ╱      ╲                    — clean triangle, no V_f loss,
       │ ╱        ╲                     no reverse current
       │╱          ╲
     0 ●────────────●────────●─────▶ t
       │             ╲      ╱
       │              ╲    ╱   ◄── reverse current
       │               ╲  ╱        (slope = −V_o/L
       │                ╲╱          continues through 0)
       │                 ●
       │← HS →│←── LS ──→│
```

```
   I_L ▲
       │            ▲ I_peak
       │           ╱╲
       │          ╱  ╲
       │         ╱    ╲
       │        ╱      ╲
       │       ╱        ╲ ◄── LS opens early
       │      ╱          ╲╲
       │     ╱            ╲╲   ◄── body diode picks up
       │    ╱              ╲╲       remaining I_L (V_f loss)
     0 ┼───●────────────────●●●────────────▶ t
       │
       │←── HS on ──→│ LS on │diode│  idle  │
                     (short)
```

Late: reverse current. Output charge is pulled back toward the input through the rect
switch — efficiency loss plus an anti-boost effect that can lift `V_in`.

Early: body diode conducts the remainder. Pure `V_f · I` loss, bounded, no instability.

The asymmetry is why the controller always biases toward the safe (early) side and why
`rect_offset_ns` is applied with a margin.

## Boost converter

The roles flip — control switch is LS, rectifier is HS — but the derivation is the same.

$$M_{CCM} = \frac{1}{1 - D}$$

$$t_{on,HS} = t_{on,LS} \cdot \frac{1}{M - 1} = \frac{D}{f_{sw}} \cdot \frac{1}{M - 1}$$

where `D` is now the LS (control) duty. Same `L`-cancellation, same `M → 1` sensitivity
blow-up at the high-step-up corner.

References

- Erickson, Maksimović. *Fundamentals of Power Electronics*, 3rd ed., ch. 5 and 15.
