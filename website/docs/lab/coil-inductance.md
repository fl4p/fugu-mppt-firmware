---
title: Coil inductance
sidebar_position: 6
---

# Coil inductance (`coil.conf::L0`)

`etc/measure_coil.py` measures the coil inductance `L0` from the three sensors the board already
has (`Vin`, `Vout`, `Iout`). The synchronous converter needs `L0` to emulate a diode, although it
has no inductor-current probe: `L0` decides CCM vs DCM and times the low-side turn-off (see
[Diode emulation](../internals/diode-emulation.md)). This page covers the theory of obtaining
`L0`, how a miscalibrated current sensor distorts the measurement, what a wrong `L0` does to diode
emulation, and how the script works.

This page uses the following symbols: `Vin`, `Vout` the converter terminal voltages; `M = Vout/Vin` the buck ratio; `D` the
high-side on-fraction of a switching period; `fsw` the switching frequency; `L` the inductance;
`Iout` the average inductor (= output) current; `ΔI` the peak-to-peak ripple current.

## 1. Background relations

In **continuous conduction (CCM)** the inductor sees `Vin−Vout` while the high side is on and
`−Vout` while it is off. This gives a triangular ripple:

```
ΔI = (Vin − Vout)·D / (fsw·L) ,   with D = M = Vout/Vin in CCM
   = Vout·(1 − Vout/Vin) / (fsw·L)
```

This is exactly `SynchronousConverter::rippleCurrent()` (`src/buck.h`), where `fL = fsw·L·0.95`
(the 0.95 `InductivityDcBias` undershoots `L` slightly; see §4).

At the CCM/DCM boundary, the valley of the ripple touches zero. The average current then equals
half the ripple:

```
Iout,crit = ΔI/2 = Vout·(1 − Vout/Vin) / (2·fsw·L)
```

In discontinuous conduction (DCM) into a stiff voltage sink (a battery clamps `Vout`), the
current is a triangular pulse that returns to zero each cycle. The current charges for `D·T` and
reaches `Ipk = (Vin−Vout)·D·T/L`. It then discharges into `Vout` for `t2 = (Vin−Vout)·D·T/Vout`
and idles. Averaging the pulse over the period gives the DCM transfer relation:

```
Iout = (Vin − Vout)·Vin·D² / (2·Vout·fsw·L)          (DCM, Vout clamped)
  ⟹  L = (Vin − Vout)·Vin·D² / (2·Vout·fsw·Iout)
```

At the boundary `D = M` the two collapse to the same expression, so the DCM relation is the
inverse of `rippleCurrent()`.

The following diagram shows both conduction modes:

```
CCM (heavy load) - inductor current never reaches zero
  i_L |    /\      /\      /\
      |   /  \    /  \    /  \        ripple  dI = (Vin-Vout)*D/(fsw*L)
 Iout |--/----\--/----\--/----\--     average current = Iout
      | /      \/      \/      \
    0 +/-------------------------> t  valley > 0  (continuous)
       |<-D*T->|

 boundary: the valley just touches 0   =>   Iout = dI/2

DCM (light load, Vout clamped by battery) - current returns to 0, then idles
  i_L |   /|       /|       /|        rise slope  (Vin-Vout)/L  during D*T
      |  / |      / |      / |        fall slope  -Vout/L       during t2
 Iout |-/--+-----/--+-----/--+----      average = Iout
    0 +/---+____/---+____/---+____> t        = (Vin-Vout)*Vin*D^2 / (2*Vout*fsw*L)
       D*T  t2  gap
```

Only DCM (the lower diagram) is solvable for `L` from the three DC averages. The current pulse is
fully determined by `L` and the terminal voltages, so a measured average `Iout` fixes `L`.

## 2. Methods to obtain `L0`

### With a current probe

A current probe or bench instrument allows these methods:

- **Impedance / LCR meter.** The reference method: off-circuit, gain-independent, and the only
  practical way to trace the full DC-bias curve (inductance vs DC current) with a bias-current
  fixture. Use this if the coil can be disconnected.
- **CCM ripple, scoped current.** Run in CCM, capture the inductor-current triangle, read `ΔI`
  peak-to-peak, then `L = (Vin−Vout)·D / (fsw·ΔI)`. Works at any DC bias point, so it can also map
  the bias curve in-circuit. Needs HF current capture and trusts the probe's AC gain.
- **di/dt over a known interval.** Apply a known voltage across the coil for a measured time and
  read the current slope; `L = V·Δt/ΔI`.

### Without a current probe

Only `Vin`, `Vout` and `Iout` are available.

The following table evaluates four candidates against this hardware (battery output,
`fs = 511 sps` control rate):

| method                    | idea                             | verdict here                                                                                                                            |
|---------------------------|----------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| DCM voltage ratio         | `Vout/Vin` depends on `L` in DCM | **fails**: the battery clamps `Vout`, so the ratio is pinned by the pack, not by `L`                                                    |
| Output-voltage ripple     | `ΔVout ≈ ΔI/(8·fsw·Cout)`        | **fails**: switching ripple is invisible at 511 sps, swamped by 100 Hz mains ripple on an inverter-fed bus, and needs a trusted `Cout`  |
| Load-step transient       | current slew bounded by `V/L`    | **impractical**: hard to command cleanly with a battery clamp + MPPT, unobservable at 511 sps                                           |
| **DCM transfer relation** | invert `Iout = f(D, V, L)`       | **works**: uses only the three DC averages; battery clamp is what *makes* it work                                                       |

The script implements the DCM transfer method (§5). The CCM/DCM boundary by itself is unusable
from DC data into a battery. With `Vout` clamped there is no kink in `Vout` vs load, and the
firmware's own `inDCM()` flag is computed *from* `L0`, so trusting it to find the boundary is
circular.

## 3. Broken current sensor

Let the current sensor have a linear gain error `g`, reporting `Iout_meas = g·Iout_true` (`g < 1`
under-reads). `L` enters the DCM relation only through `Iout`, so:

```
L_meas = (Vin−Vout)·Vin·D² / (2·Vout·fsw·Iout_meas) = L_true / g
```

A sensor that reads low (`g < 1`) inflates the measured inductance. A sensor that reads high
shrinks it. The error is a pure scale factor, so run-to-run repeatability doesn't reveal it. A
precise sensor with a wrong shunt or calibration looks trustworthy yet biases `L_meas`.

A sensor offset (non-zero reading at zero current) doesn't scale. It dominates at low current,
where it makes `L_meas` grow very large (since `L ∝ 1/Iout`). For this reason the script discards
near-zero points (§5).

Gain errors do occur: a mis-scaled or non-genuine INA226 (wrong shunt LSB) reads cleanly low or
high (see [dev-notes/ina226.md](../../../doc/dev-notes/ina226.md)). The DCM method cannot tell a
current-gain error apart from a wrong true `L`, because both move `L_meas` by the same pure scale
factor. Attributing a board-to-board discrepancy to the sensor therefore needs an independent
current reference. §9 is a worked example: two boards self-measured ~1.57× apart because they carry
different coils, and an independent current cross-check put both sensor gains near 1.

If `L0` is derived from the same biased sensor and then used by the firmware, the gain cancels
for diode emulation specifically (see §4).

## 4. Wrong `L0`

`L0` feeds only `rippleCurrent()` → `computeDCM()` (`src/buck.h`). The low-side turn-off ratio
`rectCtrlRatio(M) = 1/M − 1` depends on the voltage ratio alone, not on `L0`. The only influence
of `L0` is therefore this single decision:

```
in DCM when   ΔI_fw(L0) > 2·Iout      (with hysteresis)
```

Here `ΔI_fw ∝ 1/L0`. The two error directions are asymmetric:

- `L0` too high → `ΔI_fw` underestimated → the converter believes current is continuous when it
  is actually discontinuous → the low side is left on past the zero crossing → reverse current.
  Energy flows back, the switch node boosts, and the low-side switch (and anything on the input)
  can be destroyed. This is the dangerous direction.
- `L0` too low → `ΔI_fw` overestimated → DCM is declared too eagerly → the low side turns off
  early even in CCM, and current commutates to the body diode for the remainder. This is safe but
  adds body-diode conduction loss (lower efficiency near the boundary).

The firmware therefore undershoots on purpose with `InductivityDcBias = 0.95`. Erring toward
"too low" trades a little efficiency for safety margin. When in doubt, round `L0` down.

### Sensor-gain cancellation

A sensor-gain error cancels in the DCM decision. This only matters when a real gain error exists,
which isn't established on the two boards of §9. The argument still makes the self-measured `L0`
the safe choice whatever causes a discrepancy. Suppose `L0 = L_true/g` (measured with a sensor of
gain `g`) and the firmware compares against the *same* sensor's `Iout_meas = g·Iout_true`:

```
ΔI_fw(L0) = g · ΔI_phys / 0.95        (∝ 1/L0)
2·Iout_meas = g · 2·Iout_true
```

`g` appears on both sides of the inequality and divides out, so the DCM decision is correct even
though both `L0` and `Iout` are wrong. This has the following consequences:

- On a board with a biased sensor, the self-measured `L0` (e.g. the inflated value) gives correct
  diode emulation; substituting the physically true `L0` would mistime it.
- The cancellation is local to diode emulation. Everything that uses `Iout` in an absolute sense
  (reported power, energy metering, charge-termination current, the output current cutout `lv_i_max` in a buck) stays wrong
  by `g`.
- Only the linear gain cancels. A sensor offset or nonlinearity does not.
- `L0` and the sensor are coupled. Fixing the sensor calibration requires updating `L0` to the
  true value in the same step, or the previously-canceling pair becomes a real error.

## 5. The measurement script (`etc/measure_coil.py`)

`etc/measure_coil.py` is a host tool that drives the firmware console (serial / TCP-telnet / BLE)
via the `fugu` package. It implements the DCM transfer method because, per §2, that is the only
method that works into a battery load with the sensors present. It reuses constants the firmware
already holds and inverts the firmware's own `rippleCurrent()`.

The script runs these steps:

1. Read the realized switching frequency, the timer period and `pwmMax` from `pwm-dump`. The period is
   both the duty basis and, with the frequency, the time per count, the same basis the firmware uses
   (`period_ticks` on MCPWM, `pwmMax` on LEDC; see [MCPWM driver](../internals/pwm-drivers.md)). Obtain `pwmCtrlMax` from the `dc` out-of-range reply as
   the upper duty bound, and read the current `coil.conf::L0` for reference. Firmware without
   `pwm-dump` falls back to `board.conf::pwm_freq` and `pwmCtrlMax/(1−0.06)` with a warning; pass
   `--fsw` and `--pwm-max` in that case.
2. Read an idle status line for `M = Vout/Vin`; require `Vin > Vout` (buck headroom / sun).
3. Sweep the high-side duty count `H` upward across the DCM band (`--lo`..`--hi` × `M·pwmMax`),
   holding each step `--dwell` seconds. The script reads `Vin`, `Vout`, and `Iout` from the
   `sensor avg` compact line (`ewm.avg`), the firmware's notch+median+EWMA DC average ([Signal filters](../internals/signal-filters.md))
   (offset-corrected for current). It polls this line at ~10 Hz across the dwell and takes the
   median. The status line prints the raw instantaneous `last` for the voltages (no averaging, no
   notch) and is emitted only every `lfPeriod` (~3 s), too sparse to average a 100 Hz-rippled bus,
   so the script doesn't use it for these values. The status line still provides the exact control
   state: the applied `H`, the CCM/DCM flag, and the LS rect count. Staying below `M` keeps the
   converter in DCM and the current bounded. The sweep stops when the firmware reports CCM or
   `Iout` exceeds `--i-max`.
4. Per point compute `D = H/pwmMax` and `L = (Vin−Vout)·Vin·D² / (2·Vout·fsw·Iout)`.
5. Read `Iout` with the converter off before and after the sweep: `dc 0`, wait until the status line
   shows the converter off, wait one dwell (at least 2 s), then take the median of five `sensor avg`
   reads. Subtract this zero from the points, interpolated over the sweep rows in order (see
   [`Iout` zero](#iout-zero-during-the-sweep)). The device does the same.
6. Discard points below an `Iout` floor (sensor offset territory, §3). Fit the remaining points
   with the ring fit of §8 and report `L = 1/c0`, its noise and systematic error (see
   [Stated uncertainty](#stated-uncertainty)), the ring frequency and the median of the points for
   comparison. With fewer than 16 points, or when no
   periodic term of at least 1 % is found, report the median and IQR instead. Report `L` as a
   function of the peak coil current (§8, [Peak-current dependence](#peak-current-dependence)).
   Restore MPPT (or `dc 0`) on exit.

The script has two built-in checks. If all points are CCM, it warns that `forced_pwm` may be on
(which suppresses DCM). `--bidir` compares up/down sweeps for settling. A wrong `pwmMax` biases `L`
by a constant factor, and the script doesn't detect it.

The method has these limitations:

- Low bias only. The peak coil current stays below the DCM/CCM boundary, and the `Iout` limit of
  the sweep (2 A or the configured output limit) stops it at about 85–90 % of the boundary
  current. The `L(Ipk)` report therefore covers only the start of the DC-bias curve, a factor of
  about 2–2.5 in peak current. Beyond the boundary the transfer ratio no longer depends on `L`.
  A lower switching frequency extends it somewhat (`measure-coil l0 f=`, see
  [Lower switching frequency](#lower-switching-frequency)). For the full bias curve use a scoped
  CCM-ripple measurement or an LCR meter with bias injection (§2).
- The result scales with the `Iout` calibration (§3) and with `pwmMax`. The remaining biases
  (switching transitions, gate delays, node charge) are listed in §8. Together they can move `L` a
  few percent, mostly low, and most at the low-current end of the band.
- The reported value is the physical inductance. Put it in `coil.conf` as `L0` directly. The
  firmware applies its own 0.95 bias.

The following commands run the script over TCP and over serial:

```bash
python etc/measure_coil.py --ip <device-ip> --i-max 1.0
python etc/measure_coil.py -p <serial-port> --steps 48 --dwell 3
```

### On-device equivalent (`measure-coil`)

The `measure-coil` console command runs the same two sweeps on the device with no host
(`src/selftest/measure_coil.cpp`, a spawned non-RT-core task). It requires a build with
`CONFIG_FUGU_WITH_MEASURE_COIL=y` (off by default, on in the `sdkconfig.d/fleet` build). The command
has two forms:

```
measure-coil l0 [steps] [dwell_ms] [f=<Hz>] [apply]     # the §5 inductance sweep
measure-coil ls [hs]    [dwell_ms] [apply]              # the §6 LS-timing / rect_offset_ns sweep
```

It reads `Vin`/`Vout`/`Iout` directly from each sensor's `ewm.avg` (the same DC average the host
medians off the `sensor avg` line), uses `pwmMaxDriver()` for the exact PWM period (no
`MinDutyCycleLS` reconstruction), and applies the identical formula, `Iout` floor and ring fit
(`src/selftest/coil_fit.h`, a single-precision copy of the script's `fit_l0()`; a host test checks
that both agree to 0.1 %). The `l0` defaults are 48 steps and a 1 s dwell, about one minute in
total: the device reads the averages directly and needs no console polling. `ls` keeps a 3 s
dwell. `apply` writes `coil.conf::L0` (next boot) or `rect_offset_ns` with `rect_offset_drv` and
`rect_offset_cal=1`, or 2 when the offset was clamped to +125 ns (live at once). Either is written
to flash only after the converter has been disabled, because a flash write stalls the control loop.
`L0` is not written when the run is marked `DISTURBED`. Like the script, it reports the physical
inductance. A median-only version of the command agreed with the script to within ~2 % on board A
of §9.

The command starts only from a converter stopped with `dc 0` (manual PWM, duty 0, converter off)
and leaves it there; `mppt` resumes tracking. Manual PWM keeps the tracker, its sweeps and its
start condition out for the whole run, and the console refuses the commands that would change the
mode or the drive meanwhile. The refusals are listed in the
[console reference](../reference/console.md).

For a tight result, use many small steps. Otherwise the i-max abort leaves only the low end of the
band sampled, and the ring fit falls back to the median.

After the result line the command prints the `Iout` zero and the peak-current dependence:

```
measure-coil: L = 76.40 uH  (ring fit, 33 DCM pts, Iout>=0.44A, +-0.4% noise +-1.6% syst; ring 247 kHz ...)
measure-coil: Iout zero +0.001/-0.003 A (off, before/after), subtracted
measure-coil: L(Ipk), +-noise/offsets: 2.2-3.1A 75.71uH+-0.7/5.4% | 3.1-4.0A 77.07uH+-0.7/3.4% | 4.0-4.8A 76.45uH+-0.7/2.4%
measure-coil: L(Ipk) slope +0.76 +-0.53 %/A over 2.2-4.8 A, Iout/on-time offsets add up to +-1.73: coil -2.04..+3.56 %/A, drop <= 2.0 %/A
```

The `Iout zero` line gives the current read with the converter off before and after the sweep. If
it moved by more than 0.02 A, the line ends with `DISTURBED`, and `apply` does not write `L0`. The
first `L(Ipk)` line gives `L` per bin of peak coil current with two relative errors: noise, and the
shift that the `Iout` and on-time offset bounds can cause in that bin. The second gives the apparent
slope with its noise error, the slope the offset bounds can produce, and the resulting range for
the coil's own slope. Read the lower end as an upper bound on how much the coil's `L` drops over
the band (see [Peak-current dependence](#peak-current-dependence)). Both lines are omitted when
the band covers fewer than about 1.6 ring periods. The host script prints the same lines.

#### Lower switching frequency

In DCM the peak coil current at a given output current is

```
Ipk = sqrt(2·Iout·Vout·(Vin − Vout) / (Vin·fsw·L)) = 2·Iout·Vout / (D·Vin)
```

so it grows as `1/sqrt(fsw)` at the sweep's `Iout` limit, and the DCM/CCM boundary current grows as
`1/fsw`. `measure-coil l0 f=<Hz>` runs the sweep at a lower frequency to reach a higher `Ipk`: half
the frequency gives about 1.4 times the peak current. The firmware changes the frequency with the
converter off, through the same path as `pwm-freq` (`pwmMax`, the duty band, the CCM/DCM decision
and the dead-time in counts all follow), and restores the previous frequency once the converter is
off again, also after an abort or an error. The sweep header prints the frequency it ran at
(`fsw=`), and every row's duty `D` refers to that frequency's `pwmMax`. `f=` needs the MCPWM gate
driver (the ESP32-S3 default) and is refused for `ls`.

The frequency is bounded:

- At least 20 kHz. The low-side refresh pulse keeps its fixed length (`boot_refresh_ns`, default
  2 µs), so the sag of the high-side bootstrap supply grows with the period: about
  `Iq·T·R/t_refresh` for a driver quiescent current `Iq` through a bootstrap resistor `R`. With
  the standard board's worst case (2.2 mA, 22 Ω) that is 0.6 V at 39 kHz and 1.1 V at 20 kHz, about
  the whole margin between its 9.3 V gate supply (less the bootstrap diode) and the driver's
  lockout near 7.9 V. The sag is that large only with no coil current: once current flows, the
  freewheel interval adds refresh time. A lockout holds the high side off, which is safe but
  makes a row read low. 20 kHz is also the upper edge of the audible range.
- `fsw·L0·0.95` of at least 1 Ω with `coil.conf::L0`, the same ripple bound `pwm-freq` and the boot
  configuration apply. For a 51 µH coil that is about 20.7 kHz.
- At most the current frequency.

Each row also stops the sweep once its peak current `2·Iout·Vout/(D·Vin)`, which needs no `L`,
exceeds the configured output-current limit (`limits.conf`), the current the coil and switches
carry in normal operation.

Runs at different frequencies give comparable `L(Ipk)` bins, with one caveat. The duty basis is
`pwmMax`, the period less the wrap-side dead-time. A non-zero wrap-side dead-time biases `L` high by
about twice its share of the period, and that share halves at half the frequency.

The ring fit's frequency ceiling, set by the spacing of the points in dead time, scales with the
switching frequency at a given step count: with 48 steps it is about 0.65 MHz at 20 kHz, still above
the 250–330 kHz rings of §7. More steps raise it.

## 6. Synchronous-rectifier timing and the `Iout` peak

The DCM relation (§1) assumes the ideal triangle: the inductor discharges at exactly `−Vout`,
reaches zero, and idles. The synchronous-rectifier (low-side) turn-off timing decides whether the
real waveform matches. A deviation biases `Iout` and therefore `L`:

- LS off or too short: the body diode finishes the discharge at `−(Vout+Vf)`. This gives a steeper
  decay, shorter `t2`, and less charge delivered per cycle → `Iout` *below* ideal → `L` over-estimated (and
  the formula's `Vout` is really `Vout+Vf`).
- LS too long: held past the zero crossing, the inductor current goes negative and pulls
  charge back to the input → net `Iout` *reduced* → `L` over-estimated.

In DCM the firmware turns the low side off at the ratio `rectCtrlRatio(M)` computed with 1 %
worst-case voltage errors on each side, about 2 % before the ideal zero crossing, and the body
diode finishes the discharge. A reverse-current notch therefore does not occur in a normal `l0`
sweep, and the tail at `−(Vout+Vf)` changes `Iout` by well under 0.1 %. The oscillation of `L` in
a fine duty sweep is the switch-node ring of §7, not SR timing.

`Iout`, and the `L` derived from it, is therefore least biased when LS turns off exactly at the zero
crossing, the peak of an LS sweep at fixed duty. Measuring at or near that per-point optimum removes
the SR-timing oscillation.

That peak calibrates timing rather than `L` (at the optimum
`t2/t1 = (Vin−Vout)/Vout = 1/M − 1 = rectCtrlRatio(M)`, with `L` cancelling out). The `--ls-sweep`
peak-bracketing and the `coil.conf::rect_offset_ns` dead-time calibration it drives (`--apply`) are
rectifier-timing concerns. [Diode emulation](../internals/diode-emulation.md) covers them with
the timing theory. They complement the inductance measurement and don't replace it. The ring of §7
also modulates an LS sweep, so a shallow `Iout` peak can be a ring crest rather than the timing
optimum.

The peak curvature does give an alternative way to obtain `L`. Just past the peak,
`Iout ≈ Iout_peak − Vout·δt² / (2·L·T)`, so `L = −Vout·fsw / (d²Iout/dδt²)`. This extraction uses the
LS-time counts instead of `D`/`pwmMax`, so it cross-checks the duty scale. It still scales with
the `Iout` gain (§3), so it does not cross-check the sensor.

## 7. Switch-node ringing in DCM {#switch-node-ringing-in-dcm}

### Mechanism

In DCM the coil current reaches zero before the high side turns on again. Both switches are then
off, and the coil `L` rings with the switch-node capacitance `C` about `Vout`. `C` is the output
capacitance of both transistors plus the winding and layout capacitance. With `x = Vnode − Vout`,
`i` the coil current toward the output and the losses lumped into a conductance `1/(Q·Z0)` across
the node, KCL and KVL give

$$
C\,\frac{dx}{dt} = -i - \frac{x}{Q Z_0}, \qquad L\,\frac{di}{dt} = x, \qquad
f_r = \frac{1}{2\pi\sqrt{LC}}, \qquad Z_0 = \sqrt{L/C}, \qquad \alpha = \frac{\pi f_r}{Q}.
$$

The ring starts with the node at the low-side body-diode drop, `x ≈ −(Vout + Vf)`, and decays as
$x = A\,e^{-\alpha t}\cos(2\pi f_r t + \varphi)$, $i \approx (A/Z_0)\,e^{-\alpha t}\sin(2\pi f_r t + \varphi)$.
When `Vin < 2·Vout`, the first upswing reaches `Vin + Vf` and the high-side body diode clamps it,
returning a little charge to the input.

The same ring, started by a reverse current at low-side turn-off, shapes the late side of the
rect-cal sweep (see [switch-node ringing in the rectifier timing
calibration](../internals/rect-cal.md#switch-node-ringing)). The converter model implements it
with `vconv.conf::ring_c` and `ring_q` (see [vconv.conf](../reference/config/vconv.md#switch-node-ring)).
[DCM coil ringing](../internals/dcm-ringing.mdx) covers the tank, the node capacitance per board and
how to reduce the ring.

The high side turns on after the dead time

$$
t_d = T\left(1 - D\,\frac{V_{in}}{V_{out}}\right),
$$

with the coil current at whatever value the ring has reached, `i0`. Its magnitude is of order
`(Vout+Vf)/Z0`, a few hundred mA. The whole current triangle shifts by `i0` for the on-time and the
discharge, so the charge per cycle changes by about `i0·(t_on + t2)` and

$$
\frac{\Delta I_{out}}{I_{out}} \approx \frac{2\,i_0}{I_{pk}} .
$$

`Iout` is high when the high side turns on while the node falls from its crest (`i0 > 0`), and low
when it turns on near the valley. As the duty sweeps, `t_d` moves by `T·Vin/Vout` per unit of `D`,
so the ripple of `Iout` (and of the per-point `L`) is periodic in the duty count with period

$$
N = \frac{1}{f_r\,t_{tick}}\cdot\frac{V_{out}}{V_{in}} \quad\text{counts},
$$

where `t_tick` is the time per PWM count. The relative ripple is largest at small `D`, where `Ipk`
is small, as long as `Q` is high enough for the ring to survive the long dead time. It fades toward
the CCM boundary, where `t_d → 0` and the ring has no time to develop. The body diode delays the
first zero crossing of the current by `i0·L/(Vout+Vf)`, which warps the sinusoid into a
sawtooth-like shape. The warp does not move the mean: the average of `sin E` over the mean anomaly
is zero.

### Field sweeps

Global MPPT sweeps on two boards (the boards of §9) show an `Iout` ripple of ±10–25 % in DCM that
matches this model:

| quantity                                   | board B (≈80 µH) | board A (≈51 µH) |
|--------------------------------------------|------------------|------------------|
| ring frequency, free fit per sweep         | 248–257 kHz      | 324–330 kHz      |
| `N·Vin/Vout` (6.25 ns counts)              | ≈ 625            | constant to ±1.5 % over `Vin/Vout` 2.5–2.7 |
| implied node capacitance `C = 1/((2πf)²L)` | ≈ 4.9 nF         | ≈ 4.7 nF         |
| fitted `Q`                                 | ≈ 25             | ≈ 15             |

The period scales with `Vout/Vin` as predicted, and the implied `C` is of the order of the
transistors' output capacitance at the operating voltage. A direct scope fit of `C` is described in
[Switch-node capacitance](switch-node-capacitance.md). The model reproduces the absolute ripple
on board B (≈ 300 mA peak-to-peak) and underestimates it on board A by about a quarter, which
suggests a higher `Q` there. The following alternatives were checked and do not fit:

- Low-side dither or quantization would have a period of about one count.
- The bootstrap minimum low-side pulse is constant over the sweep and was not binding.
- Dead time is constant and cannot produce a periodic pattern.
- Aliasing of the current sensor against the PWM would be periodic in time, not locked to
  `Vin/Vout`. Slow fixed-duty sweeps show the same pattern.
- Telemetry lag shifts the pattern by at most a few counts and does not create it.

The ring frequency is measured from the pattern; the amplitude at turn-on, `Q` and `C` are inferred
from DC averages through the model. A scope capture settles them: probe the switch node and the coil
current at a fixed duty in DCM, step the high-side count through one period `N`, and read `f_r`, the
decay and `i0` at each turn-on.

### Effect on MPPT

Model runs with the fitted ring show the ripple in a global sweep. At 150–180 W the static power
curve still has a single maximum above 60 % of the peak, at the DCM/CCM boundary, and the sweep
settles within 1 % of it. At a steady 45 W, where the whole operating range is DCM, P&O holds within
1 % of the maximum with or without the ring. Ripple-induced local maxima appear only below about
60 % of the peak power.

## 8. Ring-fit estimator {#ring-fit-estimator}

The [switch-node ring](#switch-node-ringing-in-dcm) ripples the per-point `L` of a DCM sweep, so a median over
the band can be off by a few percent. The script and `measure-coil` fit the ripple out instead. The fit also gives
the dependence of `L` on the peak current and a stated uncertainty; a few biases remain outside it.

### Fit model

Each DCM point gives `y_i = 1/L_i = 2·Vout·fsw·Iout / ((Vin−Vout)·Vin·D²)` and a dead time
`t_i = (1 − D·Vin/Vout)/fsw` from its own measured voltages. The estimator fits

$$
y_i = c_0 + \sum_{h=1}^{H}\left[a_h\cos(2\pi h f t_i) + b_h\sin(2\pi h f t_i)\right] + s\,x_i
$$

by linear least squares at each trial frequency `f`, with `H = 2` from 24 points (`H = 1` from 16).
Here $x_i = I_{pk,i} - \overline{I_{pk}}$ is the peak coil current $I_{pk} = (V_{in}-V_{out})\,D/(f_{sw} L)$
relative to its mean. The linear term absorbs a dependence of `L` on the current, which a
frequency scan without it would otherwise fit as a ring of about one period over the band.

The frequency is scanned from `max(80 kHz, 1/span)` to `min(1.2 MHz, 0.45/Δt)` in steps of
`0.1/span`, then refined twice around the best value. Here `span` is the range of `t_i` and `Δt`
their median spacing. The result is `L = 1/c0`, the mean of `1/L` over whole ring periods at the mean
peak current. Fitting `1/L` rather than `L` keeps the model linear in `Iout`, so a ripple of `Iout`
maps to an additive term.

At the chosen frequency a final fit adds the envelope terms $x_i(a\cos + b\sin)$ of the first
harmonic, because the ring decays over the dead time and its amplitude changes along the band. The
fit is accepted only if it halves the residual against a straight line in $x_i$ and the first
harmonic is at least 1 % of `c0`. Otherwise the median of the points is returned.

Because the coordinate is the dead time computed from measured voltages, a soft source (a PV array
whose `Vin` sags as the current rises) does not smear the ring. The estimator degrades as follows:

- **Few points.** Below 16 points the median is used, which carries up to about ±5 % ring-phase
  error. Each step must stay under half a ring period. At 48 steps a 50–80 µH coil gets about 5–10
  points per period.
- **Short span.** If the band covers less than one ring period, the ring and the mean cannot be
  separated, and the result falls back to the median.
- **Strong damping.** A low-`Q` ring has mostly decayed before the high side turns on. The ripple
  then falls below 1 %, and the median is unbiased anyway.

On synthetic sweeps from the converter model ([Simulation](../development/simulation.md)) with
the ring enabled (random `L` 40–100 µH, `C` 3–8 nF, `Q` 10–40, stiff and soft sources, sensor noise), the 10-step median scatters 2–4 % rms
(95th percentile 4–7 %). The 48-step fit scatters 0.8–1.3 % rms (95th percentile 1.3–2.0 %), and
the fitted ring frequency is within 1 %.

### Peak-current dependence {#peak-current-dependence}

The slope `s` of the final fit gives `L` as a function of the peak current, reported as
`k = (dL/dIpk)/L` in %/A. The normalization is the line's value at zero current; the report does not
print that value, because it extrapolates the offsets described below. The peak current of each
point is recomputed with `L(Ipk)` of the line before a second fit, so that a falling `L` does not
stretch the current axis. The bins hold equal numbers of points sorted by `Ipk`. There are at most
five bins of at least six points, and each spans at least about 0.8 ring periods so that the ring
does not alias into a bin. A bin's value is the mean of the ring-corrected `1/L_i`.

In DCM each cycle ramps the coil current from zero to `Ipk` and back. The DCM relation inverts the
charge per cycle, so for a current-dependent inductance the measured `L(Ipk)` is `λ²/(2E)`, with `λ`
the flux linkage and `E` the energy stored at `Ipk` (and the L-free `Ipk = 2·Iout·Vout/(D·Vin)` is
`2E/λ`). A datasheet DC-bias curve shows the incremental inductance `L_inc(I) = dλ/dI`, the value that
sets the ripple at a DC bias `I`; the secant inductance is `L_s(I) = λ(I)/I`. With
`L_inc(I) = L(0)·(1 + c·I)` the secant is `L(0)·(1 + c·I/2)` and the measured value `L(0)·(1 + c·I/3)`
to first order. With the fitted line `L(Ipk) = L(0)·(1 + k·I)` the incremental inductance is therefore
`L(0)·(1 + 3k·I)`: it falls three times as fast as the measured value.

Two offsets tilt the apparent `L(Ipk)` and cannot be told apart from the coil's own DC-bias
dependence over one sweep:

- An `Iout` offset `δI` biases each point by $\delta I/I_{out}$, most at low current. Sources are
  the measured zero and the net node charge per cycle (see [Remaining biases](#remaining-biases)).
- An on-time offset `δt` (gate-drive and switching delays) biases each point by
  $2\,\delta t/(D\,T)$, again most at low duty.

Over the DCM band the columns `1`, `D`, `1/D` and `1/D²` of these effects are nearly collinear. On
synthetic sweeps with the ring, a fit that adds `δt` as a parameter has a condition number of
10–13 (columns normalized), a correlation of 0.97–0.98 between `δt` and the slope, and recovers an
injected 30–60 ns offset only to ±50–75 ns per sweep. A prior on `δt` of the order of the dead
time adds nothing the data can confirm. The report therefore keeps the slope and states what it
can mean:

- The response of the fit to each offset is computed by fitting the per-point sensitivities
  (`dy/dIout`, `dy/dδt`) with the same model. Their bounds, the node charge
  $C f_{sw} \max\!\left(V_{out},\ V_{in}^2/(2V_{out}) - V_{out}\right)$ with `C` from the fitted
  ring and the on-time bound (the configured dead time, at least 50 ns), give the slope the
  offsets can produce.
- The coil's own slope then lies within the apparent slope ± (that slope + 2 noise errors). Its
  lower end is an upper bound on the coil's drop over the band, valid while the offsets stay within
  their bounds. A lower end at or above zero is reported as `no drop resolved`.

An on-time offset of +30 ns shifts the apparent slope by about +0.3 %/A and `L` by about −1 %; +60 ns
doubles both. An `Iout` offset of +12 mA gives about +0.7 %/A. A slope of order +1 %/A, with `L`
rising with the current, points to such offsets rather than to the core.

On the converter model ($L = L_0(1 - k\,I_{pk})$ per switching cycle; ring 200–400 kHz, `Q` 10–40;
stiff and soft sources; 100 sweeps per case), with the significance test against the noise error
alone:

| plant slope | slope error mean / rms | noise error vs actual (rms z) | reported significant |
|-------------|------------------------|-------------------------------|----------------------|
| 0 %/A       | −0.3 / 0.4 %/A         | 1.0                           | 3–5 % (false positives) |
| −1 %/A      | −0.3 / 0.3 %/A         | 1.0                           | 94 %                 |
| −3 %/A      | −0.2 / 0.3 %/A         | 0.9                           | 100 %                |

The −0.3 %/A offset is the ring's node charge, which the model includes. With an injected on-time
offset of 0, +31 and +62 ns the apparent slope stays within the offset bound plus two noise errors
in every sweep.

### Stated uncertainty {#stated-uncertainty}

The result line gives two relative errors of `L`, both one standard deviation:

- **Noise**: from the fit residuals (the variance of `c0` in the least-squares fit) and the
  uncertainty of the `Iout` zero.
- **Systematic**: the node-charge bound, the on-time bound and an assumed 1 % `Iout` gain tolerance,
  each treated as a uniform distribution within its bound (divided by √3) and added in quadrature,
  plus ring-model mismatch. The mismatch term scales the noise term by the lag-1 autocorrelation
  `a` of the residuals in sweep order: $\sqrt{(1+a)/(1-a) - 1}$.

The `Iout` gain enters `L` one to one and is the only term that run-to-run repeatability cannot
show. On the converter model with an on-time offset within ±50 ns and a gain error within ±1 %
drawn per sweep, the actual error divided by the stated one (both in quadrature) has an rms of
0.9–1.2 for stiff, soft and changing sources, 0.6 for a corrected stale zero and 0.7 for a drifting
zero. Without those two errors in the model the ratio is 0.4–0.8: the stated error then includes
effects the model does not have.

### `Iout` zero during the sweep {#iout-zero-during-the-sweep}

An error in the current sensor's zero biases `L` by $-\delta/I_{out}$ per point. A stale offset
calibration of −0.12 A, for example from a calibration taken while the current was still decaying
after high-power operation, raises `L` by about 14 % in the model. The ring fit absorbs part of the
smooth bias pattern, and the stated error stays below 2 %.

Both sweeps therefore read `Iout` with the converter off before and after the sweep (once the
converter is off, after one dwell of at least 2 s). They subtract the zero interpolated over the
sweep rows in order. The zero's uncertainty is the 2 mA read noise and half the difference taken as
a uniform bound, in quadrature. A zero that moved by more than 0.02 A marks the run as disturbed:
the zero drifted during the sweep, and the interpolation is only an approximation.

On the model, a correction of a constant −0.12 A error leaves +0.4 % mean error, and a −0.12 A error
decaying with a 20 s time constant is flagged in all runs. A change of the PV source during the sweep (a −40 %
irradiance ramp) does not bias the result, because every point uses its own measured `Vin` and
`Vout`.

### Remaining biases

The fit removes the oscillating part of the ring. The following effects move the mean:

| effect | size | direction on `L` |
|--------|------|------------------|
| Node charge drawn from the output by the ring (the node starts near 0 and settles near `Vout`) | $\Delta I_{out} \approx -C\,V_{out}\,f_{sw}$, about −5 mA | high, ≈ 0.5–2 % |
| High-side turn-off transition: the coil current discharges `C` from `Vin`, delivering $\tfrac12 C V_{in}^2$ per cycle to the output | $\Delta I_{out} \approx C\,V_{in}^2 f_{sw} / (2 V_{out})$, about +15–20 mA at `Vin` ≈ 70 V | low, ≈ 1.5–6 % |
| Gate-drive delays that lengthen the effective on-time by `δ` | $\Delta L/L \approx -2\delta / (D\,T)$ | low, ≈ 1 % per 10 ns at `D` = 0.1 |
| Early low-side turn-off (body-diode tail at `−(Vout+Vf)`) | < 0.1 % | high |
| Winding resistance (`Iout·DCR` against `Vin−Vout`) | < 0.1 % | high |
| `InductivityDcBias` (0.95) | not applied to the measurement; the firmware applies it to `L0` | none |

The two capacitive terms act like a current-sensor offset (§3): they matter most at the
low-current end of the band, which the `Iout` floor partly excludes. The converter model includes
the first term but not the transition term (it discharges the node through the low side at its
turn-on, a loss) or gate delays, so the accuracy figures of the estimator do not cover them. The
stated systematic error includes both through their bounds. A current-probe or LCR-meter reference
(§2) is needed to quantify them on a given board.

## 9. Case study: two coils

Two Fugu2 boards in service self-measured ~1.57× apart because they have different coils. If both
are assumed to carry the same coil, the gap looks like a sensor fault. The following table lists
their hand-wound inductors:

| board  | core                  | turns | `Al`        | nominal `Al·N²`         | measured (median) |
|--------|-----------------------|-------|-------------|-------------------------|-------------------|
| A      | 2× stacked KS130-060A | 20–21 | 122 nH/N²   | 48.8 – 53.8 µH          | **50.9 µH**       |
| B      | 2× KDM KS184-125A     | 10    | 562 nH/N²   | ~56 µH (documented)     | **79.8 µH**       |

A full step-1 duty sweep on each (`measure_coil.py --steps 600 --i-max 6`, ~550 DCM points apiece)
gives the real `L` vs `H` (PWM count) shown in the following plots. The dashed line is each
board's median. Points near the CCM boundary are excluded.

```
 BOARD B  L/µH   (547 DCM pts, median 79.8, IQR 16%; CCM rolloff below 68 clipped)
  98.2 |      o
  96.2 |     ooo
  94.2 |    oooo
  92.2 |   ooo o            ooo
  90.2 |   oo  o            o o             ooo
  88.2 |   o   oo          o   o            o oo             oo             ooo
  86.2 |  oo    o         oo   o           o   o           oo oo           oo o
  84.2 | oo     o         o    o          oo    o         oo   o          oo  o
  82.2 | oo     o       oo     o        oo      o        oo    o         oo   o
  80.2 |-o------o-------o------o-------ooo------o-------oo-----o-------ooo-----o   median 79.8
  78.2 |oo      o      oo       o      o        o      oo       o     oo
  76.2 |o             oo        o    oo         o    oo         o    oo        o
  74.2 |         o   oo         o   oo          o   oo          o  ooo
  72.2 |         o  ooo         ooooo            oooo           oooo           o
  70.2 |         oooo             o               o                            o
  68.2 |          oo
       +------------------------------------------------------------------------
        206                                                                  736   H (PWM ct)
```

```
 BOARD A  L/µH   (573 DCM pts, median 50.9, IQR 11%)
  60.2 |                                                                      +
  59.0 |
  57.9 |                                             +           ++           +
  56.7 |                                +           +++         + ++          +
  55.5 |                    ++         +++++       +  ++        +  ++        ++
  54.3 |         +++        + +        +   +       +   ++      ++  +++       + +
  53.2 |         +++       ++ ++      ++   +      ++   +++     +    ++       +
  52.0 |        +   +      +   ++     +    ++     +      +     +     +++    ++
  50.8 |--------+---+-----++----+----++-----+----++-------+---+-------++----+---   median 50.9
  49.6 |++     +    +     +      +   +      +++ ++        ++ ++        +  ++
  48.5 |++     +     +   ++      ++ ++       ++++          +++         ++++
  47.3 | +    ++     ++ ++        +++
  46.1 |++   ++      ++++
  44.9 | +  ++
  43.8 |  +++
  42.6 |  ++
       +------------------------------------------------------------------------
        219                                                                  799   H (PWM ct)
```

The measurements lead to these findings:

- **The 1.57× gap is two different coils.** Board A (KS130, ~20 t) measures 50.9 µH, inside its
  own computed `Al·N²` of 48.8–53.8 µH, so its sensor and the DCM method are validated against a
  known nameplate. Board B (KS184, nominally 10 t) measures ~80 µH. Its documented `~56 µH` was only
  an unverified figure and is the suspect number: the turns/`Al` look off, since 80 µH needs ~12
  turns at that `Al`. An independent battery-shunt cross-check puts board B's current gain at
  ≈ 1.0, so its sensor is fine too. The gap doesn't come from a measurement error on either board,
  and in particular not from an INA226 reading ~1.5× low.
- **Each board's measured value is its true inductance, to a few %.** Take board A ≈ 51 µH,
  board B ≈ 80 µH. The shunt cross-check hints board A under-reports current ~6–7 %, which would inflate
  its `L` by the same factor and put the true value nearer the `N = 20` end (~48 µH). That correction
  is small compared with the 1.5× gap.
- **Later ring fits read a few percent lower.** On-device ring-fit runs (§8) give 76.0–76.4 µH on board B,
  4–5 % below the 79.8 µH median of this sweep, and 49.7 µH on board A, 2 % below its median. The fit returns the
  mean of `1/L` over whole ring periods, which the median of a skewed ripple need not match, and the runs are
  months apart with a different `Iout` zero handling. Which effect accounts for the gap is not established.
- **Both means are flat across the whole current range** (≈0.3–3 A). There is no downward trend,
  so this is not core saturation. The point-to-point wiggle of `L` (IQR 11–16 %) is the switch-node ring of
  §7 (its period in counts follows `Vout/Vin`), not noise in `L` and not SR timing. A median over a
  band that does not span whole ring periods can be off by a few percent; the §8 fit removes that.
- **Boundary breakdown (the §5 limit, observed).** Pushing board B and board A toward `H ≈ M·pwmMax`
  makes the DCM estimate diverge (board B rolls down to ~37 µH, board A up past ~100 µH) as
  the waveform enters CCM and `Iout` stops obeying the DCM transfer relation. Both are artifacts of
  measuring outside DCM, not changes in the coil; the median over the clean DCM band is the result.

Each board's `coil.conf::L0` was then set to its own measured value:

- Board A carried `56e-6`, about 10 % high and the riskier direction per §4. It now carries the
  49.7 µH of its latest ring fit.
- Board B carried `40e-6`, very conservative and costing body-diode loss near the boundary. It now
  carries `80e-6`.

The measurements don't indicate a sensor recalibration for either board.

## Open questions

- Confirm board B's ~80 µH with an LCR meter or scoped CCM-ripple measurement (§2), and reconcile it
  with the documented 10 turns / `Al` (it implies ~12 effective turns).
- The shunt cross-check hints board A under-reports current ~6–7 %. If this is real, trim its `Iout`
  calibration, and its `L0` follows.
- Scope the switch node and the coil current in DCM (§7) to confirm the ring frequency, its damping
  and the turn-on current, and to size the turn-off transition bias against an LCR-meter value of
  `L`.
