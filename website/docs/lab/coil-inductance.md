---
title: Coil inductance
sidebar_position: 6
---

# Measuring the coil inductance (`coil.conf::L0`)

`etc/measure_coil.py` measures the coil inductance `L0` from the three sensors the board already
has (`Vin`, `Vout`, `Iout`). The synchronous converter needs `L0` to emulate a diode, although it
has no inductor-current probe: `L0` decides CCM vs DCM and times the low-side turn-off (see
[Diode Emulation.md](../internals/diode-emulation.md)). This page covers the theory of obtaining
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

### With a current probe / bench instrument

A current probe or bench instrument allows these methods:

- **Impedance / LCR meter.** The reference method: off-circuit, gain-independent, and the only
  practical way to trace the full DC-bias curve (inductance vs DC current) with a bias-current
  fixture. Use this if the coil can be disconnected.
- **CCM ripple, scoped current.** Run in CCM, capture the inductor-current triangle, read `ΔI`
  peak-to-peak, then `L = (Vin−Vout)·D / (fsw·ΔI)`. Works at any DC bias point, so it can also map
  the bias curve in-circuit. Needs HF current capture and trusts the probe's AC gain.
- **di/dt over a known interval.** Apply a known voltage across the coil for a measured time and
  read the current slope; `L = V·Δt/ΔI`.

### Without a current probe (only `Vin`, `Vout`, `Iout`)

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

## 3. How a broken current sensor affects the measurement

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
current reference. §7 is a worked example. Two boards self-measured ~1.57× apart, which looked like
a sensor gain error until it turned out they carry different coils. An independent current
cross-check confirmed that both sensors are fine.

If `L0` is derived from the same biased sensor and then used by the firmware, the gain cancels
for diode emulation specifically (see §4).

## 4. How a wrong `L0` affects diode emulation

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

### Why a sensor-gain error cancels

A sensor-gain error cancels in the DCM decision. This only matters when a real gain error exists,
which isn't established on the two boards of §7. The argument still makes the self-measured `L0`
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
   (`period_ticks` on MCPWM, `pwmMax` on LEDC). Obtain `pwmCtrlMax` from the `dc` out-of-range reply as
   the upper duty bound, and read the current `coil.conf::L0` for reference. Firmware without
   `pwm-dump` falls back to `board.conf::pwm_freq` and `pwmCtrlMax/(1−0.06)` with a warning; pass
   `--fsw` and `--pwm-max` in that case.
2. Read an idle status line for `M = Vout/Vin`; require `Vin > Vout` (buck headroom / sun).
3. Sweep the high-side duty count `H` upward across the DCM band (`--lo`..`--hi` × `M·pwmMax`),
   holding each step `--dwell` seconds. The script reads `Vin`, `Vout`, and `Iout` from the
   `sensor avg` compact line (`ewm.avg`), the firmware's notch+median+EWMA DC average
   (offset-corrected for current). It polls this line at ~10 Hz across the dwell and takes the
   median. The status line prints the raw instantaneous `last` for the voltages (no averaging, no
   notch) and is emitted only every `lfPeriod` (~3 s), too sparse to average a 100 Hz-rippled bus,
   so the script doesn't use it for these values. The status line still provides the exact control
   state: the applied `H`, the CCM/DCM flag, and the LS rect count. Staying below `M` keeps the
   converter in DCM and the current bounded. The sweep stops when the firmware reports CCM or
   `Iout` exceeds `--i-max`.
4. Per point compute `D = H/pwmMax` and `L = (Vin−Vout)·Vin·D² / (2·Vout·fsw·Iout)`.
5. Discard points below an `Iout` floor (sensor offset territory, §3) and report the median of the
   rest, plus an IQR spread. Restore MPPT (or `dc 0`) on exit.

The script has two built-in checks. If all points are CCM, it warns that `forced_pwm` may be on
(which suppresses DCM). `--bidir` compares up/down sweeps for settling. A wrong `pwmMax` biases `L`
by a constant factor, and the script doesn't detect it.

The method has these limitations:

- Low bias only. Measurements are taken well below the boundary current, so the result is the
  near-unbiased small-signal inductance. The method cannot trace the DC-bias curve: pushing the
  current up crosses into CCM, where the transfer ratio no longer depends on `L`. For the bias
  curve use a scoped CCM-ripple measurement or an LCR meter with bias injection (§2).
- The result scales with the `Iout` calibration (§3) and with `pwmMax`. Dead-time and DCR
  bias it a few percent low.
- The reported value is the physical inductance. Put it in `coil.conf` as `L0` directly. The
  firmware applies its own 0.95 bias.

The following commands run the script over TCP and over serial:

```bash
python etc/measure_coil.py --ip <device-ip> --i-max 1.0
python etc/measure_coil.py -p <serial-port> --steps 12 --dwell 6
```

### On-device equivalent (`measure-coil`)

The `measure-coil` console command runs the same two sweeps on the device with no host
(`src/selftest/measure_coil.cpp`, a spawned non-RT-core task). It requires a build with
`CONFIG_FUGU_WITH_MEASURE_COIL=y` (off by default). The command has two forms:

```
measure-coil l0 [steps] [dwell_ms] [apply]     # the §5 inductance sweep
measure-coil ls [hs]    [dwell_ms] [apply]     # the §6 LS-timing / rect_offset_ns sweep
```

It reads `Vin`/`Vout`/`Iout` directly from each sensor's `ewm.avg` (the same DC average the host
medians off the `sensor avg` line), uses `pwmMaxDriver()` for the exact PWM period (no
`MinDutyCycleLS` reconstruction), and applies the identical formula, `Iout` floor, and median/IQR.
`apply` writes `coil.conf::L0` (next boot) or `rect_offset_ns` (also live). Like the script, it
reports the physical inductance. It was validated against the script on one Fugu2 board (board A
of §7) to within ~2 % (≈50 µH).

For a tight result, use many small steps. Otherwise the i-max abort leaves only the low end of the
band sampled.

## 6. Synchronous-rectifier timing and the `Iout` peak

The DCM relation (§1) assumes the ideal triangle: the inductor discharges at exactly `−Vout`,
reaches zero, and idles. The synchronous-rectifier (low-side) turn-off timing decides whether the
real waveform matches. A deviation biases `Iout` and therefore `L`:

- LS off or too short: the body diode finishes the discharge at `−(Vout+Vf)`. This gives a steeper
  decay, shorter `t2`, and less charge delivered per cycle → `Iout` *below* ideal → `L` over-estimated (and
  the formula's `Vout` is really `Vout+Vf`).
- LS too long: held past the zero crossing, the inductor current goes negative and pulls
  charge back to the input → net `Iout` *reduced* → `L` over-estimated. This reverse-current notch,
  beating against the duty as it sweeps, is what makes `L` oscillate in a fine duty sweep.

`Iout`, and the `L` derived from it, is therefore least biased when LS turns off exactly at the zero
crossing, the peak of an LS sweep at fixed duty. Measuring at or near that per-point optimum removes
the SR-timing oscillation.

That peak calibrates timing rather than `L` (at the optimum
`t2/t1 = (Vin−Vout)/Vout = 1/M − 1 = rectCtrlRatio(M)`, with `L` cancelling out). The `--ls-sweep`
peak-bracketing and the `coil.conf::rect_offset_ns` dead-time calibration it drives (`--apply`) are
rectifier-timing concerns. [Diode Emulation.md](../internals/diode-emulation.md) covers them with
the timing theory. They complement the inductance measurement and don't replace it.

The peak curvature does give an alternative way to obtain `L`. Just past the peak,
`Iout ≈ Iout_peak − Vout·δt² / (2·L·T)`, so `L = −Vout·fsw / (d²Iout/dδt²)`. This extraction uses the
LS-time counts instead of `D`/`pwmMax`, so it cross-checks the duty scale. It still scales with
the `Iout` gain (§3), so it does not cross-check the sensor.

## 7. Case study: board A vs board B (two different coils)

Two field boards (Fugu2 converters in service) self-measured ~1.57× apart because they have
different coils. Assuming they shared one made the gap look like a sensor fault. The following
table lists their hand-wound inductors:

| board  | core                  | turns | `Al`        | nominal `Al·N²`         | measured (median) |
|--------|-----------------------|-------|-------------|-------------------------|-------------------|
| A      | 2× stacked KS130-060A | 20–21 | 122 nH/N²   | 48.8 – 53.8 µH          | **50.9 µH**       |
| B      | 2× KDM KS184-125A     | 10    | 562 nH/N²   | ~56 µH (documented)     | **79.8 µH**       |

A full step-1 duty sweep on each (`measure_coil.py --steps 600 --i-max 6`, ~550 DCM points apiece,
telnet) gives the real `L` vs `H` (PWM count) shown in the following plots. The dashed line is each
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
  ≈ 1.0, so its sensor is fine too. The gap doesn't come from a measurement error on either board.
  The earlier "fake INA226 reading ~1.5× low" story is withdrawn.
- **Each board's measured value is its true inductance, to a few %.** Take board A ≈ 51 µH,
  board B ≈ 80 µH. The shunt cross-check hints board A under-reports current ~6–7 %, which would inflate
  its `L` by the same factor and put the true value nearer the `N = 20` end (~48 µH). That correction
  is small compared with the 1.5× gap.
- **Both means are flat across the whole current range** (≈0.3–3 A). There is no downward trend,
  so this is not core saturation. The ±10–16 % point-to-point wiggle is the duty-pinned SR-timing
  reverse-current notch of §6, not noise in `L`.
- **Boundary breakdown (the §5 limit, observed).** Pushing board B and board A toward `H ≈ M·pwmMax`
  makes the DCM estimate diverge (board B rolls down to ~37 µH, board A up past ~100 µH) as
  the waveform enters CCM and `Iout` stops obeying the DCM transfer relation. Both are artifacts of
  measuring outside DCM, not changes in the coil; the median over the clean DCM band is the result.

Set each board's `coil.conf::L0` to its own measured value:

- Board A ≈ `51e-6`. Its current `56e-6` is ~10 % high, the riskier direction per §4.
- Board B ≈ `80e-6`. Its current `40e-6` is very conservative, costing body-diode loss near the
  boundary.

The measurements don't indicate a sensor recalibration for either board.

## Open questions

- Confirm board B's ~80 µH with an LCR meter or scoped CCM-ripple measurement (§2), and reconcile it
  with the documented 10 turns / `Al` (it implies ~12 effective turns).
- The shunt cross-check hints board A under-reports current ~6–7 %. If this is real, trim its `Iout`
  calibration, and its `L0` follows.