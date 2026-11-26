---
title: "Diode emulation"
sidebar_position: 8
mdx:
  format: mdx
description: "The rectifier switch (LS in a buck) can stay off, and the coil-discharge current then flows through its body diode."
---

import {LsSweepChart, LsTimingChart, MSensitivityChart} from '@site/src/components/charts/DiodeEmulationCharts';

# Diode emulation

*Original hand-written notes: [doc/Diode Emulation.md](https://github.com/fl4p/fugu-mppt-firmware/blob/main/doc/Diode%20Emulation.md) and the README section [Synchronous Buck and Diode Emulation](https://github.com/fl4p/fugu-mppt-firmware#synchronous-buck-and-diode-emulation).*

## Nomenclature

```
HS:    high-side switch
LS:    low-side switch
cntrl: control switch        — HS in buck,  LS in boost
rect:  rectification switch  — LS in buck,  HS in boost
```

## Operating principle

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
comparator into the gate driver `DIS` or `EN` pin, or fast ADC with µs-scale latency).
This board has no such sensor, so the firmware uses the sensor-less approach below.

### Mode decision {#ccmdcm-decision}

The converter enters DCM when half the ripple current exceeds the dc output current:

$$
\frac{\Delta I_L}{2} > I_o
$$

It leaves DCM only once $\Delta I_L \le 1.8 \cdot I_o$ (hysteresis). Below $I_o$ = 0.1 A it is
always in DCM, and in forced PWM never. See [Implementation](#implementation-srcbuckh) for the
full rule. For a buck:

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

- The boundary region between CCM and DCM is narrow in normal operation.
- The saturation curve of well-chosen powder cores is flat through that region.
- The dc margin in CCM at high power is much larger than the ripple, so a few-percent `L`
  error doesn't shift the boundary far.

`L_0` is per board (`coil.conf::L0`). `etc/measure_coil.py` measures it (see
[Coil inductance](../lab/coil-inductance.md)).

The DCM conversion ratio (Erickson, *Fundamentals of Power Electronics*, 3e, pp. 145,
597) is

$$
M_{DCM} = \frac{2}{1 + \sqrt{1 + 4 R_e / R}}, \qquad R_e = \frac{2 L \cdot f_{sw}}{D^2}
$$

where `R` is the load, so in DCM `M ≠ D`. The firmware measures `V_o` and `V_i` instead of
computing `M_DCM`.

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

`L` cancels. An error in the inductance model affects only the [DCM decision](#ccmdcm-decision),
not the rect on-time itself.

Setting `M = D` (the CCM identity) recovers the CCM formula:

$$
t_{on,LS,\mathrm{CCM}} = \frac{1 - D}{f_{sw}}
$$

### Voltage error sensitivity

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

The sensitivity grows without bound as `M → 1`. There `V_i − V_o` is low, the falling slope
`V_o/L` is much steeper than the rising slope, and it amplifies small mistakes in the
rising-slope estimate. The controller therefore needs a wider margin at high `M`: turning LS
off slightly early costs a small body-diode `V_f` loss, turning it off late causes reverse
current.

### Reverse-current drain near `D = M`

The window above assumes the coil current is zero when HS turns on. In DCM the switch node rings
after the zero crossing, so HS turns on into a ring current $i_0$ of up to about $V_o/Z_0$
($Z_0 = \sqrt{L/C}$, $C$ the switch-node capacitance). Starting from $i_0$, the current reaches
zero after

$$
t_{on,LS} = t_{on,HS} \cdot \left(\frac{1}{M} - 1\right) + i_0 \cdot \frac{L}{V_o}
$$

so the crossing moves by up to $\sqrt{LC}$ (0.5–0.7 µs in the board models) with a phase that
depends on the off-time. A ratio reading error moves it by $t_{on,HS} \cdot \Delta M / M^2$, about
$(\Delta M/M) \cdot T$ near `D = M`. The `M` bias keeps the window $0.02 \cdot t_{on,HS}/M$ early,
about $0.02 \cdot T$ (0.5 µs at 39 kHz) near `D = M`.

Conduction past the crossing by $\delta$ builds a reverse current $V_o \delta / L$. After LS turns off it
drains through the HS body diode at $(V_i - V_o)/L$, which takes

$$
t_{drain} = \delta \cdot \frac{M}{1 - M} = \frac{\delta}{r}, \qquad r = \frac{1}{M} - 1
$$

Near `D = M` the off-time after the window is short, and at high `M` the drain is slow (9 $\delta$
at `M = 0.9`). Reverse current left at the next HS turn-on lowers the next peak, moves the next
crossing earlier and makes the next window later still. The converter then settles in a
forced-CCM-like state with a negative mean output current. With a stiff source and battery, a
ratio reading about 1.5 % off is enough.

The firmware therefore ends the DCM window early enough that the drain of a late-conduction budget
$t_{late}$ finishes before the next HS turn-on:

$$
t_{on,LS} \le T - t_{on,HS} - \frac{t_{late}}{r_{WCE}}, \qquad
t_{late} = \max\left(\left(1 - \frac{1 - e}{1 + e}\right) \cdot T,\; 0.7\ \mu s\right)
$$

with $r_{WCE}$ the window ratio from the biased `M` and $e$ = 1 % the assumed error per voltage
reading (`VoltageMaxErr`). The first term is the time the `M` bias keeps the window early near
`D = M`, so the budget covers a ratio error the size of the margin again; it is 0.5 µs at 39 kHz.

The second term covers the ring-current shift $\sqrt{LC} = 1/(2\pi f_{ring})$, which is a fixed time
and does not scale with the period: 0.68 µs at the lowest measured ring frequency of the boards,
233 kHz. The firmware configuration has no ring frequency or node capacitance, so the floor is a
constant. At 39 kHz the floor sets the budget (2.7 % of the period); below about 28 kHz the
fraction does.

With exact readings the limit binds for

$$
\frac{D}{M_{WCE}} > 1 - \frac{t_{late}}{T \cdot r_{WCE}}
$$

which at 39 kHz, converted to the true ratio (`M_WCE = M · 1.01/0.99`), is the range above these
fractions of `D = M`:

| `M` | 0.5 | 0.6 | 0.7 | 0.8 | 0.85 | 0.9 | 0.95 |
|---|---|---|---|---|---|---|---|
| limit binds from `D/M` | 0.99 | 0.98 | 0.95 | 0.90 | 0.84 | 0.71 | 0.14 |

At low `M` it binds only in the last percent of HS counts below `D = M`; above `M` ≈ 0.9 it binds in
most of the DCM range, and above about 0.88 the LS minimum pulse is longer than the limit. Where
the limit would leave less than the LS minimum pulse, the minimum pulse is issued.

The limit keeps the mean output current positive up to a combined ratio reading error of about
4 %.

The limit also lowers the stiff-source DCM gain near `D = M`: the body diode carries the tail of the
falling current, whose extra $V_f$ ends it sooner, so the mean output current at the same HS count
drops. The MPPT and voltage loops close around this.

## Timing correction (`rect_offset_ns`)

The formula above gives the *ideal* zero-crossing time from the measured voltages. The LS
window realized in hardware differs from it for reasons the voltage model does not see:

- Edge delays: gate-driver and FET propagation on both switches. A longer realized HS
  pulse raises the peak current and moves the zero crossing later. A delayed LS turn-off moves
  the actual turn-off later.
- Dead time: the commanded LS span starts at the HS turn-off, but the LS channel conducts
  only after the HS→LS dead time (the body diode carries the current meanwhile).
- Sensing error: errors in `V_i` or `V_o` shift `M` and with it the ideal window.

`coil.conf::rect_offset_ns` is an empirical net correction that the firmware adds to the
computed LS window, `>0` = LS off later. It comes from measurement (below), not from any
single effect, and its sign is whatever the measurement gives. A pure LS turn-off delay on
its own would call for a *negative* value. Measured values have been positive, so on those
boards the other effects outweigh it. The value is per board, because it depends on the gate
driver, FETs, and layout.

The firmware adds the correction as a constant time, independent of the HS on-time. That suits
edge delays. A voltage-sensing error scales with `t_on,HS` instead, so a calibration is only exact
near the operating point where it was measured. A positive offset is left out where the window
ratio is below 0.25 (buck `M` above about 0.8) and is otherwise capped so that the LS turns off
`T/32` plus the offset's drain time (`offset/ratio`) before the next HS turn-on; see
[Late results](rect-cal.md#late-results).

The firmware stores the correction in nanoseconds and converts it to counts at boot (`ns·1e-9·tick_rate`, where
the tick rate is the full timer period × fsw or the [MCPWM](pwm-drivers.md) peripheral resolution, the same
basis as `boot_refresh_ns`). Storing a time instead of counts keeps the calibration valid
across changes of PWM resolution and `pwm_freq`. At the LEDC-equivalent 2048
counts/period at 39 kHz one count is ≈ 12.5 ns, while MCPWM `bestTiming` gives ~4103
counts/period from a 160 MHz source clock, ≈ 6.25 ns/count. The same correction stored in
counts would therefore be off by a factor of two after switching between the two drivers.

The stored time still belongs to one gate driver: the dead time and edge delays it absorbs are
those of the driver it was measured on. A value carried over to another driver can hold the LS
past the zero crossing. In DCM near `D ≈ M` the output current then collapses and reverses as the
duty rises, which opens the backflow switch during a [global sweep](mppt-tracker.md).

`coil.conf::rect_offset_drv` therefore records the driver (`mcpwm`, `ledc`) next to the
value. At boot the firmware uses `rect_offset_ns` only if `rect_offset_drv` names the active
driver; otherwise it uses 0, logs a warning and marks the offset as uncalibrated. A value with
`rect_offset_cal=0` (a backstop reduction, not a measurement) is used but also counts as
uncalibrated; `rect_offset_cal=2` marks a [bound](rect-cal.md#bounds) from a calibration run.

### Bench measurement

To measure the correction, hold a steep-edge HS duty in DCM and sweep LS on-time up from
zero. The steep edge lets you locate the peak. Flat plateaus yield no reliable peak.

Reading the converter's output charge per period as the measured `I_out` assumes that
`V_in` and `V_out` stay stable and that the converter is in periodic steady state at every
step (no net charging of the input or output capacitors). A current-regulated load, or a PV
input whose voltage moves with the extracted power, can shift or hide the peak. With stable
voltages, `I_out` goes through three phases as the LS on-time grows:

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
peak (see [SR timing and the `Iout` peak](../lab/coil-inductance.md#6-synchronous-rectifier-timing-and-the-iout-peak)). `--apply` computes `peak − ideal − --apply-margin` (default 12 counts), converts it to
time with the tick rate read from `pwm-dump` (the basis the firmware uses to convert it back), and
writes `coil.conf::rect_offset_ns`. After the reboot, check the `rect_offset=… ns (… ct)` boot
log line.

Both helpers locate the peak by fitting two half-parabolas that share an apex, one per side.
This is the shape derived above: the early side is shallow, the reverse-current side steep. The
fit searches for the apex over the whole sweep. On the ideal model charted above (6.25 ns counts,
default sweep of 0.5–1.4× the ideal window in 24 steps), the fit lands within one count of the
true peak.

With 1 mA of measurement noise on a 0.25 A output, the error stays around ±12 counts
and leans early, the safe side. The steep side's curvature also gives the `L` cross-check. The
printed table shows the raw maximum next to the fitted peak; on the flat side the raw maximum
is a poor estimate.

### Self-calibration (`rect-cal`)

The firmware measures `rect_offset_ns` itself on a running buck: `rect-cal` lowers HS to a
calibration point in DCM, sweeps the LS window across the zero crossing and stores the result. See
[Rectifier timing calibration](rect-cal.md).

### Runtime backstop

In DCM with sync rect on, a stored offset that is too long shows a distinct signature: the
output current collapses towards zero while the HS duty rises towards `D = M`, and `V_i` rises
because less power is drawn from the panel. A global sweep past the MPP does not do this,
because the panel current rises towards `I_sc` as `V_i` falls; a source drop such as a cloud
lowers `V_i`.

When the current (the protection's fast filter) drops below 25% of the peak of the averaged
current seen on the rising duty run, with that peak above 0.5 A, HS above 0.8·M and `V_i` 0.5% above its
value at the peak, the firmware shortens the in-RAM offset: a positive offset drops to 0, a
zero or negative one by 125 ns, at most once per 2 s. It never lengthens the offset and stops
at the lowest value a calibration can apply, −(2.5 µs + 0.75 µs); an offset already below that
stays.

The backstop is armed only on a buck in MPPT mode (including sweeps; not in PSU or PV emulator mode
or manual PWM, not during a calibration run or an ADC calibration), with no manual LS count and no
forced PWM, while `V_i > V_o + 1 V`, and from 2 s after the converter starts. Its peak history is
cleared whenever it is disarmed.

Each trigger marks the offset uncalibrated in RAM, so that the automatic calibration runs
again. After the second trigger the reduced value is written to `coil.conf` (with the converter
off, see [saving the result](rect-cal.md#saving-the-result)) with `rect_offset_cal=0`, so after a reboot it is used but still counts as
uncalibrated.

The active offset is published as the `rect_off` telemetry field (counts).

## Implementation (`src/buck.h`)

The formulas above give the *ideal* LS on-time. The firmware applies the following steps on
top of that value:

| step | behaviour |
|---|---|
| mode decision (`computeDCM`) | Enters DCM when `ΔI_L > 2.0·I_o`, leaves when `ΔI_L ≤ 1.8·I_o` (hysteresis). Always DCM when `I_o < 0.1 A`. Effective forced PWM overrides all of this (never DCM). |
| M bias | Assumes a ±1 % error on each voltage and divides `M` by `0.99/1.01`, i.e. biases it ≈ 2 % toward the early-turn-off side, then clamps it off unity (`≤ 0.99` buck, `≥ 1.01` boost). On a buck the full bias applies up to `M ≈ 0.97`; between 0.97 and 0.99 the clamp eats into it; above 0.99 the clamped `M` is *below* the true one and the LS window comes out longer than ideal (at `M = 0.995` the ratio is 0.0101 against an ideal 0.0050, ≈ 2×). A buck without a valid reading (`V_i ≤ V_o` or `V_i ≤ 0.1 V`) uses the clamp value 0.99, the shortest window. |
| low-current cut-off | In DCM the LS ratio is set to 0 when `I_o ≤ 0.01 A` or the low-side voltage is `< 1 V`; it stays at 0 until `I_o ≥ 0.04 A` and the voltage is back at `≥ 1 V`. The offset is not applied (a positive one needs a ratio of at least 0.25, a negative one is clamped up), so the target is `pwmRectMin`. |
| DCM LS limit (`pwmRectMax`) | `pwmCtrl·ratio + rect_offset` counts, at most `pwmMax − pwmCtrl − 1 − late/ratio`, `late` the larger of `LateDrainBudget·pwmMax` (≈ 0.0198, from `VoltageMaxErr`) and `LateDrainMinNs` = 700 ns (the drain limit near `D = M`), with first-order error-feedback dither of the rounding remainder (so the limit's time-average tracks the unrounded target). A positive `rect_offset` is dropped at a ratio below 0.25 and otherwise capped at `pwmMax − pwmCtrl − 1 − pwmMax/32 − offset/ratio`, but not below `pwmCtrl·ratio`. |
| CCM LS limit | `pwmMax − pwmCtrl − 1` (complementary). |
| final clamp | The limit is clamped to `[pwmRectMin, pwmMax − pwmCtrl − 1]`. On a buck `pwmRectMin` = `boot_refresh_ns` in ticks + the HS→LS dead time, for bootstrap refresh. |
| commanded LS count (`pwmRect`) | Fades in toward the limit (+1 count per update at first, then 1 + 1/64 of the remaining gap), follows it down immediately, and stays at `pwmRectMin` while sync rect is off. While the gap is larger than `pwmMax/64` (in forced PWM at any gap), the fade also advances on control updates that leave the duty unchanged. Duty decreases and a manual LS count override it (below the table). |

The commanded LS count also follows these rules:

- A duty decrease of more than `pwmMax/50` counts in one step (not in forced PWM) forces the DCM
  assumption, whose limit is never longer, and drops the command to `pwmRectMin`; the fade-in then
  restarts.
- Smaller decreases that add up to more than `pwmMax/50` within the lag of the current filter also
  drop the command to `pwmRectMin` (see [cumulative decrease](#cumulative-decrease)).
- A manual LS count (`rect-cal`, bench tests) replaces all of this: the command holds that count,
  clamped to `[pwmRectMin, pwmMax − pwmCtrl − 1]`.

The ideal `t_on,LS` is therefore the pre-clamp target of the limit, not the commanded
window. The dither's average carries over to the command only once it has settled at the
limit; while it ramps, or while a clamp is active, the average LS window differs from the
ideal. The LS channel also conducts for less than the commanded span: the span starts at the
HS turn-off, and the HS→LS dead time passes before the LS turns on.

Because of the clamp, a buck with sync rect off (console `sync off`) still issues a
`pwmRectMin` LS pulse every period, and so does the low-current cut-off. At zero load that pulse
builds `V_o·t/L` of reverse current. Account for this pulse when you compare scope traces with the ideal triangle.

The drain limit leaves out the minimum pulse. With the default `boot_refresh_ns` of 2 µs at
39 kHz it is 7.8 % of the period, and above a voltage ratio of about 0.92 its reverse current no longer
drains within the period and pumps charge from the output back into the input. The firmware keeps a
buck below that ratio with a start block, a `V_in` floor and a stop; see
[LS minimum pulse near unity ratio](#ls-minimum-pulse-near-unity-ratio).

### Cumulative decrease

The decision between CCM and DCM uses the filtered output current, which lags the duty. A ramp of small
steps can pull the duty below `M` while the decision still reads CCM. The complementary LS
window then forces `V_i = V_o/D`, and once that is above the panel's open-circuit voltage the
coil current reverses.

The firmware therefore keeps a leaky sum of the net duty decrease; increases subtract from it.
With each new current sample the sum loses `1/N` of its value, where `N` is the mean delay of
the filtered current in samples:

| filter stage | mean delay (samples) |
|---|---|
| two-pass EWMA of span `filt_len` | `filt_len − 1` |
| median of five (bypassed with `despike`) | 2 |
| ripple comb, when on | `L/2` |

A steady ramp of `r` counts per current sample settles at `r·N`. When the sum exceeds
`pwmMax/50`, the command drops to `pwmRectMin` and the sum starts again from zero. Ramps faster
than `pwmMax/(50·N)` counts per sample (about 4 counts with `filt_len` 20) trigger it; the P&O
dither and slow limiter retreats do not. On a fast-sampling board the fade from the end of a sweep
to its MPP also triggers it, and the LS stays near its minimum until that fade ends. So does the partial peak
scan's travel to a probe above the operating point; the scan waits for the fade-in before it measures
([schedule and cost](mppt-tracker.md#schedule-and-cost)). Forced PWM and a manual LS count bypass the rule.

Unlike the single large step, this rule leaves the DCM state to `computeDCM`. A forced DCM state
at a high `M` would count as an uncovered LS minimum pulse, and the minimum-pulse protection
would stop the converter.

## LS turn-off errors

<LsTimingChart />

An LS turn-off error has a different effect in each direction:

- Late: reverse current. Output charge is pulled back toward the input through the rect
  switch. This costs efficiency. The returned charge lowers the net input current, so `V_in` rises when the source
  cannot sink it (a panel; see [LS minimum pulse near unity ratio](#ls-minimum-pulse-near-unity-ratio)).
- Early: the body diode conducts the remainder. The only cost is a bounded `V_f · I` loss,
  with no instability.

This asymmetry is why the `M` bias leans toward the safe (early) side and why the measured
`rect_offset_ns` is applied minus a margin. The bias protects only over the unclamped range:
on a buck above `M ≈ 0.99`, wherever `pwmRectMin` exceeds the ideal window, and wherever the
offset over-corrects, the commanded LS window is longer than ideal and some reverse current flows.

## Reverse-current response

Sync rect is on only while the filtered current of the physical current sensor is above a threshold
that depends on the backflow switch (hysteresis):

| `limits.conf::reverse_current_paranoia` | backflow switch closed | backflow switch open |
|---|---|---|
| 1 (default) | 0.05 A | 0.2 A |
| 0 | 0 A | 0.1 A |

Above 30 % duty, 0.05 A is enough in either case. The backflow switch closes under the same condition
(always in a boost or PSU mode). With `reverse_current_paranoia` it stays open while the averaged
current is below −0.05 A.

When the sensor reads reverse current, the firmware cuts the LS window back to `pwmRectMin`
(`syncRectMinDuty`) and opens the backflow switch:

- Two consecutive samples below −1 A (not in forced PWM or during the backflow-switch self-test): LS to minimum, backflow switch open. If
  the averaged output current is above 10 A at the same time, the reading counts as noise and the
  converter stops (`revI-highAvg`).
- Averaged current below −1 A: the converter stops (`revI-ewm`).
- Buck with `reverse_current_paranoia`, outside the backflow-switch self-test: a filtered current below −0.05 A (the smaller of the average
  and the larger of the last two samples) sets the LS to minimum and opens the backflow switch.

The converter keeps switching after a reduction. The LS window recovers through the fade-in once the
current is back above the threshold, and the backflow switch closes again. A buck also stops when
`V_o` exceeds its input, which forward conversion cannot produce: averaged
`V_o > 1.25·(V_i + 1 V)` (`Vout>Vin-avg`) or a sample `V_o > 2·(V_i + 0.5 V)` (`Vout>2Vin`). See
[Sub-trip reverse current](protection.md#sub-trip-reverse-current) and the
[trip reference](protection.md#trip-reference).

## LS minimum pulse near unity ratio

A buck issues at least `pwmRectMin` (the bootstrap refresh `boot_refresh_ns` plus the HS→LS
dead time, `t_min`) of LS on-time every period, also in DCM and with sync rect off. When the
ideal window is shorter, the pulse drives the coil current negative by `V_o·δ/L`, where `δ` is
the part of the pulse after the zero crossing. After the LS turns off, that current drains
through the HS body diode into the input at `(V_i − V_o)/L`, which takes `δ·M/(1 − M)`. The
drain fits into the period only while

$$
\frac{t_{min}}{1 - M} \le T \quad\Leftrightarrow\quad M \le M_c = 1 - \frac{t_{min}}{T},
$$

independent of the HS on-time. Above `M_c` the reverse current carries over into the next
period and grows by `V_i·T·(M − M_c)/L` per period: the converter runs as a boost from the
output into the input.

With a source that can sink current (a battery bus, a sinking supply)
the mean current settles only where the loop resistance limits it, `≈ −V_i·(M − M_c)/R`,
which is tens of amps a few percent above `M_c`. One percent above `M_c` the current
falls by about 4 A per millisecond (`V_i` 29 V, `L` 76 µH), so no sensor-based reaction can
follow the pump once it runs. A panel limits it by rising in voltage until `M` is back at `M_c`.
With the defaults (`boot_refresh_ns` 2 µs at 39 kHz) `t_min/T` is 7.8 to 8.6 % depending on
the dead time, so `M_c` ≈ 0.914–0.922.

Below `M_c` the pulse's reverse current drains within the period. The mean output current
is still negative when the pulse exceeds twice the ideal window, i.e. at
`t_HS < t_min·M/(2(1 − M))`; that current is bounded, about `V_o·t_min²/(2·L·T·(1 − M))`
and does not grow. Near `M` 0.89 this lossless upper bound is 0.24–0.29 A on an 80 µH coil and
0.37–0.45 A on 51 µH (`V_o` 27 V, 0 to 200 ns dead time); the board models, with losses, give 0.2–0.3 A.

The firmware keeps a buck in DCM (diode emulation, not forced PWM) below `M_c` from the
measured voltages:

| | ratio `V_out/V_in` (readings) | defaults, dead time 0 / 200 ns |
|---|---|---|
| stop | `M_stop = (1 − (t_min + T/32)/T) · 0.99/1.01` | 0.873 / 0.865 (`V_in` < 1.145 / 1.155 `V_out`) |
| start, `V_in` floor | `M_stop − 0.02` | 0.853 / 0.845 (`V_in` > 1.172 / 1.183 `V_out`) |

The factor `0.99/1.01` is the worst-case ratio error of two voltage readings with 1 % error
each (`V_out` read low, `V_in` read high). `T/32` (0.031 in `M`) is a further margin for
reading errors beyond that: an output voltage sensor can read 1.1–1.7 % low while the
converter switches. The ring current that a DCM period leaves in the coil at HS turn-on moves
the zero crossing by only about 0.002 in `M`. With both readings at their 1 % limit the
highest true ratio the stop admits is 0.891 with no dead time and 0.883 with 200 ns, 0.03 below
`M_c`.

- **Start:** a buck starts only below the start ratio, in addition to `V_in > V_out + 1 V`
  (start-block reason `minpulse-M`). In PSU and PV emulator mode the larger of `V_out` and
  the setpoint counts, so a setpoint at or above `M_start·V_in` does not start. A manual duty
  (`dc`) start, including the restart after a backoff, follows the same rule.
- **`V_in` floor:** while the converter is in DCM, the input-voltage controller holds
  `V_in ≥ V_out / (M_stop − 0.02)` (in addition to `limits.conf::vin_min`). A sweep that
  reaches the floor ends there like at any other limit, and the tracker stays on the floor
  while the panel's MPP is below it. The floor appears as limiter index 0 (`Vin`). The
  input-voltage controller's derivative memory is reset when the floor comes or goes, so the
  setpoint step is not taken as a voltage change.
- **Stop:** the converter stops with a 5 s backoff (reason `minpulse-M`) when, in DCM,
  either the ratio reaches `M_stop`, or it is above the start ratio while the averaged output
  current is below −0.05 A (a source the floor cannot move). Both are evaluated on the
  averaged voltages and need 4 consecutive fresh `V_out` samples, except when the converter
  enters DCM (from CCM) with the ratio already at `M_stop`: then the pump is already running
  and the stop is immediate. After the stop, the start ratio applies again.

In CCM the coil current covers the LS window and none of this applies; a converter in CCM
near unity ratio keeps running until the current drops into DCM, where it stops at once if
the ratio is at or above `M_stop`.

Limits of the protection: the margin between `M_stop` and `M_c` is about 1 V of `V_in` at a
27 V output. A stiff source that steps across it faster than the averaged readings follow
(about 10 samples of the voltage filter plus 4 samples) pumps until the stop, at the
resistive level above.

A converter that leaves CCM above `M_c` (a stiff input falling while the
HS runs at its maximum, which is about `1 − t_min/T`) pumps until the averaged current marks
DCM. The bounded negative current at short HS times below the start ratio
remains (a sweep starting from zero duty passes through it). A boost has no rect minimum
pulse (`pwmRectMin` is 0) and is not affected.

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

In a boost the control (LS) duty is capped at 90 % of the period, so the LS always turns off and
releases the coil energy. `pwmRectMin` is 0: the rectifier HS needs no minimum pulse. `M = V_o/V_i`
is clamped to at most 10 before the bias and to at least 1.01 after it; without a valid reading
(`V_o ≤ V_i` or `V_i ≤ 0.1 V`) it is taken as 10, the shortest rectifier window.

## References

- Erickson, Maksimović. *Fundamentals of Power Electronics*, 3rd ed., ch. 5 and 15.
