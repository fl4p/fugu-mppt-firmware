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

The rectifier switch (LS in a buck) can stay off, and the coil-discharge current then
flows through its body diode. The converter operates non-synchronously. This is simple to
control, but the converter pays the body-diode `V_f` loss every cycle.

Synchronous operation turns the rect switch on for the window the body diode would
otherwise conduct, shorting it out and removing the `V_f` loss. The on-time then has to be
correct. If it is too long, the inductor current reverses (forced PWM: charge flows back
from output to input). If it is too short, the body diode carries the remainder (small
`V_f` loss, no danger).

In this firmware, diode emulation is the sensor-less computation of that on-time. In
CCM it is just `(1 − D)/f_sw`. In DCM it depends on the conversion ratio `M = V_o/V_i`.

The alternative is a current sensor with hardware zero-cross detection (analog
comparator into the gate driver `DIS`/`EN` pin, or fast ADC with µs-scale latency).
This board has no such sensor, so the firmware uses the sensor-less approach below.

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

The firmware uses a flat margin instead of the full `μ(H)` model, which would need the core
datasheet and a per-board geometry table. The margin is `L = L_0 · InductivityDcBias`
with `InductivityDcBias = 0.95` (`μ/μ_i` as a fraction, i.e. a 5 % reduction, `src/buck.h`).
The flat margin is adequate for three reasons:

- The CCM/DCM boundary region is narrow in normal operation.
- The saturation curve of well-chosen powder cores is flat through that region.
- The dc margin in CCM at high power is much larger than the ripple, so a few-percent `L`
  error doesn't shift the boundary far.

`L_0` is per board (`coil.conf::L0`). `etc/measure_coil.py` measures it.

The DCM conversion ratio (Erickson, *Fundamentals of Power Electronics*, 3e, pp. 145,
597) is

$$
M_{DCM} = \frac{2}{1 + \sqrt{1 + 4 R_e / R}}, \qquad R_e = \frac{2 L \cdot f_{sw}}{D^2}
$$

where `R` is the load. The formula shows that in DCM `M ≠ D`. The firmware measures `V_o`
and `V_i` instead of using `M_DCM` directly.

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

`L` cancels. An error in the inductance model only affects whether the firmware decides it
is in DCM (the boundary check above), not the rect on-time itself.

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

To first order, a 2 % error in `M` produces these errors in `t_on,LS`:

| operating point | 2% M-error → t_on,LS error |
|-----------------|----------------------------|
| `M = 0.5`       | ≈ 4%                       |
| `M = 0.8`       | ≈ 10%                      |
| `M = 0.9`       | ≈ 20%                      |
| `M = 0.95`      | ≈ 40%                      |

For finite errors the response is asymmetric: at `M = 0.95`, a +2 % error in `M` shortens
`t_on,LS` by 39.2 % and a −2 % error lengthens it by 40.8 %.

<MSensitivityChart />

The sensitivity grows without bound as `M → 1` (low `V_i − V_o`, where the falling slope
`V_o/L` is much steeper than the rising slope and amplifies small mistakes in the
rising-slope estimate). The controller therefore needs a wider margin at high `M`. Turning
LS off slightly early costs a small body-diode `V_f` loss. Turning it off late causes reverse
current.

### Timing correction (`rect_offset_ns`)

The formula above gives the *ideal* zero-crossing time from the measured voltages. The LS
window realized in hardware differs from it for reasons the voltage model does not see:

- Edge delays: gate-driver and FET propagation on both switches. A longer realized HS
  pulse raises the peak current and moves the zero crossing later. A delayed LS turn-off moves
  the actual turn-off later.
- Dead time: the commanded LS span starts at the HS turn-off, but the LS channel conducts
  only after the HS→LS dead time (the body diode carries the current meanwhile).
- Sensing error: `V_i`/`V_o` errors shift `M` and with it the ideal window.

`coil.conf::rect_offset_ns` is an empirical net correction that the firmware adds to the
computed LS window, `>0` = LS off later. It comes from measurement (below), not from any
single effect, and its sign is whatever the measurement gives. A pure LS turn-off delay on
its own would call for a *negative* value. Measured values have been positive, so on those
boards the other effects outweigh it. The value is per board, because it depends on the gate
driver, FETs, and layout.

The firmware adds the correction as a constant time, independent of `M` and of the HS
on-time. That suits edge delays. A voltage-sensing error scales with `t_on,HS` instead, so a
calibration is only exact near the operating point where it was measured.

The firmware stores the correction in nanoseconds and converts it to counts at boot (`ns·1e-9·tick_rate`, where
the tick rate is the full timer period × fsw or the MCPWM peripheral resolution, the same
basis as `boot_refresh_ns`). Storing a time instead of counts keeps the calibration valid
across changes of PWM resolution and `pwm_freq`. At the LEDC-equivalent 2048
counts/period at 39 kHz one count is ≈ 12.5 ns, while MCPWM `bestTiming` gives ~4103
counts/period from a 160 MHz source clock, ≈ 6.25 ns/count. The same correction stored in
counts would therefore be off by a factor of two after switching between the two drivers.

Storing a time does not make the value portable between gate drivers, though: the dead time
and edge delays it absorbs belong to the driver it was measured on. A value carried over to
another driver can hold the LS past the zero crossing. In DCM near `D ≈ M` the output current
then collapses and reverses as the duty rises, which opens the backflow switch during a global
sweep. `coil.conf::rect_offset_drv` therefore records the driver (`mcpwm`, `ledc`) next to the
value. At boot the firmware uses `rect_offset_ns` only if `rect_offset_drv` names the active
driver; otherwise it uses 0, logs a warning and marks the offset as uncalibrated. A value with
`rect_offset_cal=0` (a backstop reduction, not a measurement) is used but also counts as
uncalibrated; `rect_offset_cal=2` marks a bound from a calibration run (see *Bounds* below).

#### Measuring it

To measure the correction, hold a steep-edge HS duty in DCM and sweep LS on-time up from
zero. The steep edge lets you locate the peak. Flat plateaus yield no reliable peak.

Reading the converter's output charge per period as the measured `I_out` assumes that
`V_in` and `V_out` stay stable and that the converter is in periodic steady state at every
step (no net charging of the input or output capacitors). A current-regulated load, or a PV
input whose voltage moves with the extracted power, can shift or hide the peak. Under those
conditions, `I_out` goes through three phases as the LS on-time grows:

1. It rises as LS replaces the body diode (recovering the `V_f` loss).
2. It peaks when LS turns off exactly at the zero crossing (clean ideal triangle).
3. It falls as LS is held past zero and reverse current starts.

<LsSweepChart />

The body-diode side is a broad plateau: turning LS off early hands conduction back to the
diode at a small `V_f` loss, with no sharp drop. The late side also leaves the peak with
zero slope, but the loss grows with the square of the overshoot and with a much larger
coefficient (about 60× the early side in the charted example), so the drop steepens quickly.
This asymmetry makes the peak locatable. The offset between the peak and the
firmware's predicted point `rectCtrlRatio(M)·pwmCtrl` is the timing correction.

`measure-coil ls [hs]` (on-device) or `etc/measure_coil.py --ls-sweep --hs N` brackets the
peak. `--apply` computes `peak − ideal − --apply-margin` (default 12 counts), converts it to
time with the tick rate read from `pwm-dump` (the basis the firmware uses to convert it back), and
writes `coil.conf::rect_offset_ns`. After the reboot, check the `rect_offset=… ns (… ct)` boot
log line.

Field values on two boards with different gate-driver / FET combinations are +100 and +57
counts (at LEDC 12.5 ns/tick).

Both helpers locate the peak by fitting two half-parabolas that share an apex, one per side.
This is the shape derived above: the early side is shallow, the reverse-current side steep. The
fit searches for the apex over the whole sweep. On the ideal model charted above (6.25 ns counts,
default sweep of 0.5–1.4× the ideal window in 24 steps), the fit lands within one count of the
true peak. With 1 mA of measurement noise on a 0.25 A output, the error stays around ±12 counts
and leans early, the safe side. The steep side's curvature also gives the `L` cross-check. The
printed table shows the raw maximum next to the fitted peak. On the flat side, the raw maximum
is not a good estimate.

#### Self-calibration (`rect-cal`)

The firmware runs the same LS sweep itself (`src/rect_cal.h`). It lowers HS to `0.55·M` of the
period and holds it there. That point draws well below the panel's MPP current, so `V_i` sits on
the flat, voltage-source part of the PV curve near `V_oc`. At the MPPT operating point itself the
panel limits the power, the LS window hardly changes the output current, and in simulation the
peak landed up to ~200 counts late, toward a longer LS.

**Starting a run.** `rect-cal` starts from a DCM operating point at 0.3–4 A. `rect-cal force`
starts from any MPPT operating point above the calibration HS, including CCM at full power. Both
take the duty over from the tracker and ramp HS to the calibration point by `pwmMax/512` counts
per control sample (about 0.3 s from a full-power operating point at ~450 samples/s). Before the
first HS step the LS is pinned at its minimum (`pwmRectMin`), and it stays there through the ramp
and the settling phase. The converter is then a non-synchronous buck: the body diode carries the
freewheel current and no reverse current can build, however fast the current falls through the
CCM/DCM boundary and however late the CCM/DCM decision follows it. Lowering the duty only moves
every limiter away from its limit (power, currents and `V_o` fall, `V_i` rises), so the ramp cannot
make a limiter act. `V_i` rises toward `V_oc`; the run stops at 97% of the input over-voltage trip
(`limits.conf::hv_max`) before the trip itself.

**Settling.** At the calibration HS the run waits until the converter reports DCM and two
consecutive 0.5 s windows (each after 150 ms settling) agree: `I_out` within 0.02 A + 2%, `V_i`
and `V_o` within 0.3%. It gives up after 12 s from the start (`not in DCM`, `unsettled`).

**Sweep.** The firmware computes the ideal window from the averaged voltages of the last settling
window (`(V_i/V_o − 1)·HS`, without the worst-case ratio margin) and steps LS from 0.5× to 1.4× of
it in 24 steps. Each step settles for 150 ms and averages `I_out` for 500 ms. Between steps, a
reference LS of 0.6× ideal is held for 400 ms. Each point is the step's current minus the mean of
the two references around it, which cancels any change of the source that is linear in time, such
as slowly changing irradiance. Each point is placed relative to the ideal window of its own
averaged `V_i`/`V_o`, so a voltage drift moves the coordinate with the zero crossing instead of
reading as a timing error.

The coarse sweep stops one step after the first clear drop, a step at least 10 mA below the step
before and 20 mA below the best point so far. If the step after it is still 10 mA below the best
point, the drop stands. If it is back up, the drop is measured again: a second low reading confirms
it (a ring dip keeps the lower of both), otherwise it was an outlier, its point is replaced and the
coarse sweep continues. After a confirmed drop the run measures 10 more points spread evenly over
the four coarse intervals before the drop, never later than it, which places points about every
20–30 counts where the bend starts. The sweep also stops when the average current turns negative. The LS never exceeds the ideal window by more than 2.5 µs, nor 90% of the off-time
volt-second limit `(1 − V_o/V_i)·T` (see *Switch-node ringing* below); a run whose cap leaves less
than half of that 2.5 µs beyond the ideal window fails with `LS headroom`.

**Signal size.** With the LS held `Δt` past the inductor zero crossing, reverse current builds to
`V_o·Δt/L` and returns through the HS body diode at the slope `(V_i − V_o)/L`. The output loses

$$
\Delta I_{out} \approx \frac{f_{sw}\, V_o\, V_i}{2 L\,(V_i - V_o)}\,\Delta t^2 .
$$

With the LS turned off `Δt` early, the body diode carries the rest of the fall at the slope
`(V_o + V_f)/L`, and the output loses only `ΔI_out ≈ f_sw·V_f·Δt²/(2L)`, 60–90× less at `V_f` = 0.7 V. Neither
term depends on the operating current: the peak is located by the curvature of the late side,
which is set by `L`, `V_i` and `V_o`. With 6.25 ns counts and `V_o` = 26.5–27.8 V:

| `L` | `V_i` | `I_out` at the calibration point | 50 counts late | 100 counts late | 200 counts late | 1.4× ideal | early side, 0.5× ideal |
|---|---|---|---|---|---|---|---|
| 56 µH | 51–70 V | 0.9–1.2 A | 1.4–2.1 mA | 5.8–8.3 mA | 23–33 mA | 140–186 mA | 2.5–4.7 mA |
| 80 µH | 62–78 V | 0.7–0.9 A | 1.0–1.2 mA | 3.8–4.8 mA | 15–19 mA | 118–139 mA | 2.6–3.7 mA |

A higher operating current widens the swept range in counts (the ideal window scales with the
peak current) but does not steepen the curve, so the calibration point stays at `0.55·M`. A sweep
that has not yet passed the zero crossing shows only the early side, a few mA in total, which is
within the noise of a short run.

These are lossless estimates. On an 80 µH board at low irradiance, the measured late side was
about 30 times steeper (−20 to −70 mA at 70 counts past the ideal window, −160 mA at 140 counts),
and the early side fell by 10–15 mA across the sweep instead of rising by a few mA. The run
therefore assumes neither the size of the knee nor a flat early side (see *Locating the zero
crossing* below).

**Voltage changes.** In DCM at fixed HS the output current is
`D²·V_i·(V_i − V_o)/(2·f_sw·L·V_o)`. 1% of `V_o` changes it by about 1.5–2.2% (13–19 mA at the
calibration point), more than the signal within 100 counts of the peak. A step of the battery
voltage during one measurement window would therefore read as a current peak. Two measures keep
voltage changes out of the result:

- Every window current is scaled to the voltages of the settling window with that model,
  `I·g(V_i0, V_o0)/g(V_i, V_o)` with `g = V_i·(V_i − V_o)/V_o`.
- The averaged `V_o` of each step must lie within 0.2% of the mean of its two references, and
  `V_i` within 0.3%. The windows are equally spaced, so a linear drift passes this check and a
  step does not. A step that fails is measured again, up to 6 times per run, so a single bus
  transient costs about one second; the seventh failure ends the run (`V step`). A `V_o` reading
  that is off by more than 0.2% on both tries at the same LS, in the same direction and within
  0.2% of each other, while `V_i` passes, follows the LS window rather than the bus and ends the
  run at once (`sense follows LS`, see *Sense path* below).

The knee itself moves `V_i`. Reverse current lowers the power drawn from the panel by the
fraction `ΔI/I` of the measured (unscaled) output current drop, and on the PV curve
`ΔV_i/V_i = S·ΔI/I` with `S = |dV/dI|·I/V`, the panel's slope relative to its static
resistance. `S` is 1 at the MPP and smaller on the voltage-source side where the run operates:
about 0.05–0.07 at an `I_sc` of 1.5–2 A with 0.6 A drawn near a 76 V `V_oc`. There, a 160 mA knee
lifts `V_i` by 0.4%, and the next step by about 1%, so a fixed 0.3% limit rejects exactly the
signal the run looks for. A step's `V_i` may therefore exceed its references by 0.3% plus
`0.25·ΔI/I`, but only upward and only by what the step's own measured current drop explains. A
model error cannot create that allowance: if the plant current does not follow `V_i` at all, the
raw current shows no drop and the limit stays 0.3%. `V_o` gets no allowance: the knee moves it by
`R_bus·ΔI`, under 0.1% for a 0.4 A knee on a 40 mΩ bus.

`V_i` of the reference windows must stay within 2% of the settling value (`Vin shift`): a cloud
lowers `V_oc` by about 2% for a 30% drop in irradiance. `V_o` may drift by up to 5% (`Vout
shift`): on a battery bus shared with other chargers and loads, `V_o` moves with their current,
independent of the run. A scaled current that changes by more than 25% against the settling
window fails the run (`V shift`).

In simulation:

- A 1% (also 0.5% and 0.3%) `V_o` dip during the steps 100–200 counts past the zero crossing is
  rejected; since it repeats at each LS it ends the run with `sense follows LS`. Without the step
  check, a fit learned +36 counts (a later LS turn-off) from the 1% dip.
- A 0.19% `V_o` square wave, just under the limit, with periods of 0.9–2.3 s against the 1.05 s
  step cycle and a plant 0× or 2× as sensitive to voltage as the model, biases the result by −17
  to +24 counts. After the applied margin (below), the worst case leaves the LS 4 counts (25 ns)
  late, where the reverse current is well below 1 mA.
- A `V_i` rise confined to the steps past the crossing, with a plant that does not follow `V_i`,
  is learned toward a shorter LS (−22 counts at 0.2%) or rejected (0.5% and above).
- A linear `V_o` drift of ±2.5% over 30 s is learned within a few counts.
- Replicas of three low-irradiance runs (soft panel, 30× knee, falling early side, swinging bus)
  calibrate within ±5 counts. Before these changes they all ended with `V step`. A cloud that
  moves `V_i` by 2.6% still ends the run with `Vin shift`.

**Sense path.** Before the zero crossing the LS window can only raise `I_out` by the body-diode
loss it recovers, a few mA (0.3% of the current or less between 0.5× and 1× ideal), and reverse
current only lowers it. A current reading that climbs with the LS window therefore comes from the
sense path: switching noise coupled into a current/voltage monitor that samples during the LS
on-time. Such a monitor usually disturbs its voltage channel as well, so a run that trusts neither
reading is refused rather than corrected. Once both of two consecutive coarse points up to the
ideal window read more than `20 mA + 0.4%·I` (`I`: the settling current) above the higher of the
first two points, the run ends with `Iout follows LS`. The bound grows with the current because
the diode loss does; the switch-node ring adds 10–15 mA on an 80 µH board and noise does not
scale. `rect-cal status` prints the largest rise and its bound.

On a 51 µH board whose shared current/voltage monitor has no input filter, three field runs at
about 1.6–1.7 A rose by 36–52 mA between 0.5× and 1× ideal and never dropped; past ideal the `V_o`
reading rose by up to 1.3% on every retry. Replayed, each run ends with `Iout follows LS` at
0.73–0.93× ideal, and with `sense follows LS` within 0.07× of ideal when only the `V_o` reading
follows the LS. Three runs on an 80 µH board rose by at most 2.3 mA, the simulated scenarios by
at most 12 mA.

**Switch-node ringing.** While both switches are off, the inductor rings with the switch-node
capacitance `C` (FET output capacitances, driver and layout) at `Z_0 = √(L/C)`; the mechanism and
the field fits of `C` and `Q` are in [Switch-node ringing in
DCM](../lab/coil-inductance.md#switch-node-ringing-in-dcm). In DCM this is the ring around `V_o`
that ripples `I_out` against the duty. When the LS turns off with a reverse current `I_r`, the node
swings up from 0 V; it reaches `V_i`, where the HS body diode clamps it and the remaining current
returns to the input, only if `I_r·Z_0 ≥ √((V_i − V_o)² − V_o²)`. A smaller reverse current rings
back into the inductor instead and is not lost. The steep part of the late side therefore starts a
fixed reverse current after the zero crossing, about 0.35 A at `V_i` = 78 V, `V_o` = 26.5 V and
`Z_0` = 126 Ω (80 µH, 5 nF), which is 170 counts of 6.25 ns. The coil current left over from the
ring when HS turns on shifts the next switching period. This tilts the early side by a few mA and,
past the bend, makes the curve non-monotonic: the loss reaches a minimum about 250 counts past the
crossing, recovers, and turns into a gain around 1.2× ideal. The converter model with this node
(`C` = 5 nF, `Q` = 40, and an LS that conducts 35–70 counts longer than commanded) reproduces three
field sweeps on an 80 µH board within 3–18 mA rms, including both tails; the model and its
parameters are described in [vconv.conf](../reference/config/vconv.md#switch-node-ring).

HS also turns on into the ring. The coil current it finds there, up to about `±V_o/Z_0` depending on
the ring phase, adds to the peak current and moves the zero crossing by that current times `L/V_o`,
up to `±√(LC)`. The window `(V_i/V_o − 1)·HS` does not contain it. The phase follows the off-time
after the crossing, `T − HS·V_i/V_o`, so the shift differs at every operating point and no constant
offset can follow it. In the board models fitted to a SPICE half-bridge bench (76 µH with a 233 kHz
ring and 51 µH with 320 kHz, at the run's calibration point over `V_i` = 40–78 V) the crossing lies
between −0.43 and +0.75 µs from the ideal window. Holding the LS past the crossing changes the ring
current HS turns on into: with a negative ring current the output current bends up at the crossing;
with a positive one it stays flat past the crossing and bends down only at the clamp onset.

Two consequences follow for the run:

- A downward bend fit alone finds the steep loss, between the zero crossing and the clamp onset,
  and misses an upward bend at the crossing (see *Locating the zero crossing*). Points past the
  loss minimum pull the fit far early (the full field sweep does not fit at all; truncated two
  points past the bend it lands within the expected range), which is why the sweep stops one step
  after the first clear drop and the fit ends at the floor of the first dip. On the ringing model
  of a full-sun run the run learns 25–30 counts before the ideal window against a zero crossing at
  −35 counts, and the applied offset (after the margin) lands 10–15 counts before the crossing.
- An LS longer than the off-time `(1 − V_o/V_i)·T` leaves reverse current in the inductor when
  HS turns on. The converter then runs as a synchronous buck backward from the battery, and the
  reverse current grows by several amperes within a few periods until the reverse-current guard
  clamps the LS. Because the calibration HS is set from the voltages before the ramp, while `V_i`
  rises toward `V_oc` afterward, this limit can sit at only 1.3× the ideal window, inside a
  0.5–1.4× sweep. The 90% cap keeps the sweep below it.

With the cap and the stop rule, the LS ends at most 2.5 µs past the ideal window, so the peak
reverse current is bounded by `V_o·2.5 µs/L` plus the LS turn-off delay: about 1 A at 80 µH and
1.5 A at 51 µH with `V_o` = 26.5 V. A sweep that stops one coarse step past the first clear drop
reaches about half of that. On a replica of a full-sun run at 78 V, 20 runs per noise level learned within
a standard deviation of 2.0 counts at 10 mA of sensor noise per sample and 3.3 counts at 30 mA,
without a guard action. At 100 mA the bend is lost in the noise in 2 of 20 runs (`no knee`, `few
points`).
Without the 10 extra points the spread rises to 1.8 and 4.7 counts and the result moves 26 counts
earlier.

**Aborts.** The run is aborted, and the LS returned to diode emulation, on a limiter, a mode
change, a sweep, `rect-cal abort`, the input voltage near its trip, leaving DCM or a change of HS
during the sweep, after 60 s, and on any protection action: a shutdown, the backflow switch or
sync rect turned off, or the LS clamped to its minimum. The protection check runs right before each
LS update of the run, after the protection pass of the same sample, so the run never raises an LS
that a protection has just lowered. Afterwards HS fades back to the operating point it started
from, unless the backflow switch is open (reverse-current guard) or the mode changed; then the
tracker continues from where the run stopped.

**Locating the zero crossing.** A DCM triangle of peak current `p` carries
`p²·L·V_i/(2·V_o·(V_i − V_o))` per period and ends `p·L/V_o` after HS turns off. This holds
whatever coil current HS turned on into (to second order) and however long HS really conducted, so
the measured current places the actual zero crossing `zc` without knowing the ring phase: the run
uses the mean current of its steps at 0.8–1× ideal (below the crossing), the PWM period and the
coil inductance of the converter model, `0.95·L0`. Since `zc ∝ √L`, an inductance error of `ε`
moves `zc` by `ε/2` of the window. In the board models `zc` follows the plant's crossing at the
calibration point within a spread of 10–20 ns: 40–180 ns early with the model inductance equal to
the plant's (the ring returns about `C·V_o` per period from the output to the node, which the run
cannot subtract without the node capacitance), and 25–250 ns late with a model inductance 4% above
the plant's.

The run then fits a line with a quadratic bend from `x_0` on,
`y = c_0 + c_1·d + c_2·min(0, d)² + c_3·max(0, d)²` with `d = x − x_0`, and searches `x_0` over the
sweep. Two bends are fitted: a downward one (`c_2 ≤ 0`, `c_3 < c_2`) over all points up to a dip
floor (below) and, if the highest of them lies past `zc`, an upward one (`c_3 > max(c_2, 0)`)
through that point. Each must exceed 10 mA and 4× its residual RMS at the last point, counting
only the bend it adds to the early side's own curvature; otherwise a concave early side and a
noise point past `zc` pass as an upward bend. The earlier bend is the crossing in commanded counts.
The line takes any slope of the early side, so `x_0` is where the curve starts to bend, not where
it is highest. With a falling early side, the maximum sits at the start of the sweep and a peak
fit lands far before the crossing (about −190 counts on the 80 µH field data, where the bend fit
finds −9 and +47).

Past the bend the ring can turn the current back up, and the points of a recovery pull the
quadratic, and with it `x_0`, early. Once two consecutive points lie at least 10 mA below the
highest point and a later one rises 10 mA above the lowest point since, the fit uses the points up
to that lowest one, the floor of the first dip. An upward bend at the crossing comes before any
dip and stays in. A dip followed by a recovery smaller than 10 mA is fitted in full. Ending at the
floor leaves few points past the bend: on a knee followed within 60 counts by a dip floor and a
partial recovery of 4–8 mA, the fit lands between 210 counts before and 17 counts after the knee
(ending after the recovery instead, 17 to 240 counts before it).

`zc − ideal` contains the ring current at HS turn-on, which no constant offset can follow, but also
the extra charge of an HS that conducts longer than commanded and half of a voltage sensing error
(`zc` depends half as strongly on `V_o` as the window does). The run therefore learns
`learned = x_0 − max(zc, ideal)`: the delays of driver, gate and sensing between the commanded and
the real LS turn-off, rounded early. An HS that conducts `τ` longer than commanded moves the real
crossing by `τ·V_i/V_o`, of which the result contains about `τ`, which leaves the LS early.

The run is rejected, and the previous value kept, with fewer than 6 points, no step at 0.8–1×
ideal, `|zc − ideal| > 2.5 µs` (`charge model`: `L0` or the current reading far off), no bend
(`no knee`: the bend at the last point under 10 mA or under 4× the residual RMS) or
`|x_0 − max(zc, ideal)| > 2.5 µs` (`implausible`). Otherwise the applied offset is
`learned − max(125 ns, 0.3·|learned|)`, or `learned − max(250 ns, 0.3·|learned|)` when the fit
ended at a dip floor, at most +125 ns. With the larger floor the partial-recovery case above
applies at least 23 counts (6.25 ns each) before the knee. The margin grows with the offset,
because a large offset comes from large unmodelled delays; a fixed fraction of `learned` would move
toward a longer LS when `learned` is negative. The result takes effect at once.

In the board models (no driver or gate delay, so the true offset is 0) over `V_i` = 1.4–2.9·`V_o`,
`I_sc` 3 and 6 A, two fill factors and three noise seeds, the run learns a mean of −0.29 to +0.34 µs
per model with a spread (standard deviation) of 0.15–0.26 µs and applies −1.25 to +0.14 µs.
Referenced to the ideal window alone with a downward bend only, the same runs learn with a spread
of 0.39–0.46 µs, and up to +1.5 µs late below `V_i = 2·V_o`, where the current bends up at the
crossing.

**Late results.** A late result is applied up to +125 ns. A later LS turn-off than the zero
crossing builds reverse current at `V_o/L` per second; near `D = M` the 2% ratio margin of the
diode-emulation window (about `0.02·T` = 0.5 µs at 39 kHz) is all that keeps the LS from filling the
off-time, and an LS that does so runs the converter backward from the battery (see *Switch-node
ringing*). In the board models with exact voltage readings, an offset of +125 ns over the DCM range
(`V_i` = 31–80 V, `V_o` = 26.8 and 29.2 V) leaves at most 0.09–0.13 A of reverse current at LS
turn-off (0.2–0.7 µJ in the coil per period) and up to 11 mW of extra loss; +250 ns leaves
0.19–0.24 A; at +375 ns the reverse current carries into the next period near `D = M` (0.4–0.9 A)
and at +500 ns it runs away to 2.6–3.9 A. With voltage readings that use up the 2% margin, +125 ns
leaves 0.29–0.35 A (offset 0: 0.25–0.27 A) without carry-over. The +125 ns limit takes a quarter of
the margin.

**Bounds.** Above `V_i = 2·V_o` the reverse current must first lift the switch node to `V_i`
before it is lost, which can put the downward bend after the zero crossing by a lag that the node
capacitance sets. Driver and gate delays explain tens of nanoseconds. A result whose applied
offset would exceed +125 ns is therefore a bound, not a measurement: the run applies the shorter of
+125 ns and the offset in use, and never moves an active offset later. A shorter measured offset
stays as it is and keeps its state. Otherwise the value is stored with
`rect_offset_cal=2`, which `rect-cal status` shows as `bound` also after a reboot. Writing it to `coil.conf` (`rect_offset_ns`, `rect_offset_drv`, `rect_offset_cal`) waits
until the converter has been off without interruption for 10 s with a valid output-power reading
below 1 W, typically at night: a flash write that erases a sector stalls both cores, including the
control loop, for tens of milliseconds. Any converter start, however short, restarts the 10 s.
The control loop then blocks converter starts until the write is done: it grants the request,
the low-frequency task admits the write within milliseconds and rewrites `coil.conf` once, and
starts are allowed again as soon as it finishes, however long the write takes. A grant that is not
admitted within 8 s is withdrawn. Only starts are blocked; protection and shutdown are not. The new
`coil.conf` is written to `coil.conf.cft` and renamed over the old file, which LittleFS does
atomically, so a reset during the save leaves either the old or the new file complete; a leftover
`.cft` file is deleted at boot. Console configuration commands (`set-config`, `del-config`,
`svc`) write the same way; FTP uploads and the USB config drive have their own write paths. On a
bench ESP32-S3 the save, including the rename, took about 0.4 s (about 0.05 s for an in-place
rewrite), so a start that falls into a save is delayed by that much. A failed or withdrawn save is
retried a minute later; successful writes are at least 10 minutes apart. What is
written is always the offset in use at that moment together with its state (measured, bound or
uncalibrated): if the backstop below has shortened a fresh result before it was saved, the shorter
value is saved as uncalibrated.

Preconditions for a run: buck topology, converter on in MPPT mode (not sweeping, not PSU/PV-sim
or manual PWM, no forced PWM, no limiter active), sync rect and backflow switch on,
`V_i > V_o + 1 V`, a physical `I_out` sensor reading at least 0.3 A and an HS count above
`0.55·M`. `rect-cal` further needs DCM and at most 4 A. At the calibration point the run further
needs more than 0.15 A and an LS headroom of at least 1.15× ideal below the period. With
`coil.conf::rect_cal_auto=1` (default) the firmware starts a run (`rect-cal`, not `force`) by
itself while the offset is uncalibrated, at most once a day, not in the first 10 minutes after
boot and only at a steady operating point above 5 W. A refused start is retried after 10 minutes.
A bound is retried only while the operating `V_i` is below `2·V_o` (the calibration point sits
higher on the PV curve, so this is necessary, not sufficient) and not within 7 days of a bound
result since boot: a board that always operates above `2·V_o` gives a bound on every run and
would otherwise spend a run of 35–40 s every day.

A run ends without a result, and the previous value stays, on any refusal or failure: `not in
DCM`, `unsettled`, `low I`, `LS headroom`, `Vin shift`, `Vout shift`, `V shift`, `V step`,
`sense follows LS`, `Iout follows LS`, `few points`, `no knee`, `implausible`, an abort, or
`timeout`.

#### Runtime backstop

In DCM with sync rect on, a stored offset that is too long shows a distinct signature: the
output current collapses towards zero while the HS duty rises towards `D = M`, and `V_i` rises
because less power is drawn from the panel. A global sweep past the MPP does not do this,
because the panel current rises towards `I_sc` as `V_i` falls; a source drop such as a cloud
lowers `V_i`. When the current (the protection's fast filter) drops below 25% of the peak seen
on the rising duty run, with that peak above 0.5 A, HS above 0.8·M and `V_i` 0.5% above its
value at the peak, the firmware shortens the in-RAM offset: a positive offset drops to 0, a
zero or negative one by 125 ns, at most once per 2 s. It never lengthens the offset and stops
at the lowest value a calibration can apply, −(2.5 µs + 0.75 µs); an offset already below that
stays. The backstop is armed only in MPPT mode (including sweeps; not in PSU/PV-sim or manual
PWM, not during a calibration run) and from 2 s after the converter starts; its peak history
is cleared whenever it is disarmed.

Each trigger marks the offset uncalibrated in RAM, so that the automatic calibration runs
again. After the second trigger the reduced value is written to `coil.conf` (with the converter
off, as above) with `rect_offset_cal=0`, so after a reboot it is used but still counts as
uncalibrated.

The active offset is published as the `rect_off` telemetry field (counts).

### Implementation (`src/buck.h`)

The formulas above give the *ideal* LS on-time. The firmware applies the following steps on
top of that value:

| step | behaviour |
|---|---|
| CCM/DCM decision (`computeDCM`) | Enters DCM when `ΔI_L > 2.0·I_o`, leaves when `ΔI_L ≤ 1.8·I_o` (hysteresis). Always DCM when `I_o < 0.1 A`. Effective forced PWM overrides all of this (never DCM). |
| M bias | Assumes a ±1 % error on each voltage and divides `M` by `0.99/1.01`, i.e. biases it ≈ 2 % toward the early-turn-off side, then clamps it off unity (`≤ 0.99` buck, `≥ 1.01` boost). On a buck the full bias applies up to `M ≈ 0.97`; between 0.97 and 0.99 the clamp eats into it; above 0.99 the clamped `M` is *below* the true one and the LS window comes out longer than ideal (at `M = 0.995` the ratio is 0.0101 against an ideal 0.0050, ≈ 2×). |
| low-current cut-off | In DCM the LS ratio is set to 0 when `I_o ≤ 0.01 A` or the low-side voltage is `< 1 V`; it stays at 0 until `I_o ≥ 0.04 A` and the voltage is back at `≥ 1 V`. The offset is still added, so the target becomes `rect_offset` counts. |
| DCM LS limit (`pwmRectMax`) | `pwmCtrl·ratio + rect_offset` counts, with first-order error-feedback dither of the rounding remainder (so the limit's time-average tracks the unrounded target). |
| CCM LS limit | `pwmMax − pwmCtrl − 1` (complementary). |
| final clamp | The limit is clamped to `[pwmRectMin, pwmMax − pwmCtrl − 1]`. On a buck `pwmRectMin` = `boot_refresh_ns` in ticks + the HS→LS dead time, for bootstrap refresh. |
| commanded LS count (`pwmRect`) | Fades in toward the limit (+1 count per update at first, then 1/64 of the remaining gap), follows it down immediately, drops to `pwmRectMin` on a large duty decrease, and stays at `pwmRectMin` while sync rect is off. |

The ideal `t_on,LS` is therefore the pre-clamp target of the limit, not the commanded
window. The dither's average carries over to the command only once it has settled at the
limit; while it ramps, or while a clamp is active, the average LS window differs from the
ideal. The LS channel also conducts for less than the commanded span: the span starts at the
HS turn-off, and the HS→LS dead time passes before the LS turns on.

Because of the clamp, a buck with sync rect off (console `sync off`) still issues a
`pwmRectMin` LS pulse every period, and the low-current cut-off issues the larger of
`pwmRectMin` and the offset. At zero load that pulse builds `V_o·t/L` of reverse current.
Account for this pulse when you compare scope traces with the ideal triangle.

### Failure modes at the LS boundary

<LsTimingChart />

An LS turn-off error has a different effect in each direction:

- Late: reverse current. Output charge is pulled back toward the input through the rect
  switch. This costs efficiency and causes an anti-boost effect that can lift `V_in`.
- Early: the body diode conducts the remainder. The only cost is a bounded `V_f · I` loss,
  with no instability.

This asymmetry is why the `M` bias leans toward the safe (early) side and why the measured
`rect_offset_ns` is applied minus a margin. The bias protects only over the unclamped range:
on a buck above `M ≈ 0.99`, wherever `pwmRectMin` exceeds the ideal window, and wherever the
offset over-corrects, the commanded LS window is longer than ideal and some reverse current flows.

## Boost converter

The roles flip (the control switch is LS, the rectifier is HS), but the derivation is the same.

$$
M_{CCM} = \frac{1}{1 - D}
$$

$$
t_{on,HS} = t_{on,LS} \cdot \frac{1}{M - 1} = \frac{D}{f_{sw}} \cdot \frac{1}{M - 1}
$$

where `D` is now the LS (control) duty. `L` cancels in the same way, and the sensitivity
grows without bound as `M → 1` in the same way, here at the near-unity (low step-up) corner.

References

- Erickson, Maksimović. *Fundamentals of Power Electronics*, 3rd ed., ch. 5 and 15.
