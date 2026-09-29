---
title: Coil inductance
sidebar_position: 6
---

# Measuring the Coil Inductance (`coil.conf::L0`)

The synchronous converter has no inductor-current probe. It still needs the coil inductance `L0`
to emulate a diode (decide CCM vs DCM and time the low-side turn-off — see
[Diode Emulation.md](../internals/diode-emulation.md)). This note covers the theory of obtaining `L0`, how a
miscalibrated current sensor distorts the measurement, what a wrong `L0` does to diode emulation,
and how `etc/measure_coil.py` measures it from the three sensors the board already has
(`Vin`, `Vout`, `Iout`).

Symbols: `Vin`, `Vout` the converter terminal voltages; `M = Vout/Vin` the buck ratio; `D` the
high-side on-fraction of a switching period; `fsw` the switching frequency; `L` the inductance;
`Iout` the average inductor (= output) current; `ΔI` the peak-to-peak ripple current.

## 1. Background relations

In **continuous conduction (CCM)** the inductor sees `Vin−Vout` while the high side is on and
`−Vout` while it is off, giving a triangular ripple

```
ΔI = (Vin − Vout)·D / (fsw·L) ,   with D = M = Vout/Vin in CCM
   = Vout·(1 − Vout/Vin) / (fsw·L)
```

This is exactly `SynchronousConverter::rippleCurrent()` (`src/buck.h`), where `fL = fsw·L·0.95`
(the 0.95 `InductivityDcBias` undershoots `L` slightly — see §4).

The **CCM/DCM boundary** is where the valley of the ripple touches zero, i.e. the average current
equals half the ripple:

```
Iout,crit = ΔI/2 = Vout·(1 − Vout/Vin) / (2·fsw·L)
```

In **discontinuous conduction (DCM)** into a stiff voltage sink (a battery clamps `Vout`), the
current is a triangular pulse that returns to zero each cycle. Charging for `D·T` reaches
`Ipk = (Vin−Vout)·D·T/L`, discharging into `Vout` for `t2 = (Vin−Vout)·D·T/Vout`, then idle.
Averaging the pulse over the period gives the DCM transfer relation:

```
Iout = (Vin − Vout)·Vin·D² / (2·Vout·fsw·L)          (DCM, Vout clamped)
  ⟹  L = (Vin − Vout)·Vin·D² / (2·Vout·fsw·Iout)
```

At the boundary `D = M` the two collapse to the same expression — the DCM relation is the
inverse of `rippleCurrent()`.

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

Only DCM (right) is solvable for `L` from the three DC averages: the current pulse is fully
determined by `L` and the terminal voltages, so a measured average `Iout` pins down `L`.

## 2. Methods to obtain `L0`

### With a current probe / bench instrument

- **Impedance / LCR meter** — the gold standard. Off-circuit, gain-independent, and the only
  practical way to trace the full **DC-bias curve** (inductance vs DC current) with a bias-current
  fixture. Use this if the coil can be disconnected.
- **CCM ripple, scoped current** — run in CCM, capture the inductor-current triangle, read `ΔI`
  peak-to-peak, then `L = (Vin−Vout)·D / (fsw·ΔI)`. Works at any DC bias point, so it can also map
  the bias curve in-circuit. Needs HF current capture and trusts the probe's AC gain.
- **di/dt over a known interval** — apply a known voltage across the coil for a measured time and
  read the current slope; `L = V·Δt/ΔI`.

### Without a current probe (only `Vin`, `Vout`, `Iout`)

Four candidates, evaluated against this hardware (battery output, `fs = 511 sps` control rate):

| method                    | idea                             | verdict here                                                                                                                            |
|---------------------------|----------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| DCM voltage ratio         | `Vout/Vin` depends on `L` in DCM | **fails** — the battery clamps `Vout`, so the ratio is pinned by the pack, not by `L`                                                   |
| Output-voltage ripple     | `ΔVout ≈ ΔI/(8·fsw·Cout)`        | **fails** — switching ripple is invisible at 511 sps, swamped by 100 Hz mains ripple on an inverter-fed bus, and needs a trusted `Cout` |
| Load-step transient       | current slew bounded by `V/L`    | **impractical** — hard to command cleanly with a battery clamp + MPPT, unobservable at 511 sps                                          |
| **DCM transfer relation** | invert `Iout = f(D, V, L)`       | **works** — uses only the three DC averages; battery clamp is what *makes* it work                                                      |

The DCM transfer method is the one implemented (§5). Note the CCM/DCM **boundary** by itself is not
usable from DC data into a battery: with `Vout` clamped there is no kink in `Vout` vs load, and the
firmware's own `inDCM()` flag is computed *from* `L0`, so trusting it to find the boundary is
circular.

## 3. How a broken current sensor affects the measurement

Let the current sensor have a linear gain error `g`, reporting `Iout_meas = g·Iout_true` (`g < 1`
under-reads). Because `L` enters the DCM relation only through `Iout`:

```
L_meas = (Vin−Vout)·Vin·D² / (2·Vout·fsw·Iout_meas) = L_true / g
```

A sensor reading **low** (`g < 1`) inflates the measured inductance; reading **high** shrinks it.
The error is a pure scale factor, so it is invisible in the run-to-run *repeatability* — a precise
sensor with a wrong shunt/calibration looks trustworthy yet biases `L_meas`.

A sensor **offset** (non-zero reading at zero current) does not scale; it dominates at low current,
where it makes `L_meas` blow up (since `L ∝ 1/Iout`). This is why the script discards near-zero
points (§5).

A gain error is a real failure mode, not just a worry — a mis-scaled or non-genuine INA226 (wrong
shunt LSB) reads cleanly low or high (see [dev-notes/ina226.md](../../../doc/dev-notes/ina226.md)). The catch is
that the DCM method **cannot tell a current-gain error apart from a wrong true `L`**: both move
`L_meas` by the same pure scale factor. Pinning a board-to-board discrepancy on the sensor
therefore needs an *independent* current reference. §7 is a cautionary worked example — two boards
self-measured ~1.57× apart, which looked like a sensor gain error until it turned out they simply
carry **different coils**, with an independent current cross-check confirming both sensors are fine.

**Self-consistency note.** If `L0` is *derived from* the same biased sensor and then used by the
firmware, the gain cancels for diode emulation specifically — see §4.

## 4. How a wrong `L0` affects diode emulation

`L0` feeds only `rippleCurrent()` → `computeDCM()` (`src/buck.h`). The low-side turn-off ratio
`rectCtrlRatio(M) = 1/M − 1` depends on the voltage ratio alone, not on `L0`. So `L0`'s entire
influence is the single decision

```
in DCM when   ΔI_fw(L0) > 2·Iout      (with hysteresis)
```

and `ΔI_fw ∝ 1/L0`. The two error directions are **asymmetric**:

- **`L0` too high** → `ΔI_fw` underestimated → the converter believes current is continuous when it
  is actually discontinuous → the low side is left on past the zero crossing → **reverse current**.
  Energy flows back, the switch node boosts, and the low-side switch (and anything on the input)
  can be destroyed. This is the dangerous direction.
- **`L0` too low** → `ΔI_fw` overestimated → DCM is declared too eagerly → the low side turns off
  early even in CCM; current commutates to the body diode for the remainder. **Safe**, but extra
  body-diode conduction loss (lower efficiency near the boundary).

That asymmetry is why the firmware deliberately undershoots with `InductivityDcBias = 0.95`: erring
toward "too low" trades a little efficiency for safety margin. When in doubt, round `L0` down.

**Why a sensor-gain error (if present) cancels here.** This is conditional — it only matters when a
real gain error exists, which on the two boards of §7 is *not* established; but the argument is worth
following because it makes the self-measured `L0` the safe choice regardless of the discrepancy's
cause. If `L0 = L_true/g` (measured with a sensor of gain `g`) and the firmware compares against the
*same* sensor's `Iout_meas = g·Iout_true`:

```
ΔI_fw(L0) = g · ΔI_phys / 0.95        (∝ 1/L0)
2·Iout_meas = g · 2·Iout_true
```

`g` appears on both sides of the inequality and divides out — the DCM decision is correct despite
both `L0` and `Iout` being wrong. Consequences:

- On a board with a biased sensor, the *self-measured* `L0` (e.g. the inflated value) gives correct
  diode emulation; substituting the physically-true `L0` would actually mistime it.
- The cancellation is local to diode emulation. Everything that uses `Iout` in an absolute sense —
  reported power, energy metering, charge-termination current, the `iout_max` cutout — stays wrong
  by `g`.
- Only the linear gain cancels; a sensor offset/nonlinearity does not.
- `L0` and the sensor are therefore coupled: fixing the sensor calibration requires updating `L0`
  to the true value in the same step, or the previously-canceling pair becomes a real error.

## 5. The measurement script (`etc/measure_coil.py`)

A host tool that drives the firmware console (serial / TCP-telnet / BLE) via the `fugu` package. It
implements the DCM transfer method because, per §2, that is the only one that survives a battery
load with the sensors present — and it reuses constants the firmware already holds, inverting its
own `rippleCurrent()`.

Procedure:

1. Read the realized switching frequency, the timer period and `pwmMax` from `pwm-dump`. The period is
   both the duty basis and, with the frequency, the time per count, the same basis the firmware uses
   (`period_ticks` on MCPWM, `pwmMax` on LEDC). Obtain `pwmCtrlMax` from the `dc` out-of-range reply as
   the upper duty bound, and read the current `coil.conf::L0` for reference. Firmware without
   `pwm-dump` falls back to `board.conf::pwm_freq` and `pwmCtrlMax/(1−0.06)` with a warning; pass
   `--fsw` and `--pwm-max` in that case.
2. Read an idle status line for `M = Vout/Vin`; require `Vin > Vout` (buck headroom / sun).
3. Sweep the high-side duty count `H` upward across the DCM band (`--lo`..`--hi` × `M·pwmMax`),
   holding each step `--dwell` seconds. `Vin`, `Vout`, `Iout` are read from the `sensor avg`
   compact line (`ewm.avg`) — the firmware's notch+median+EWMA DC average, offset-corrected for
   current — polled at ~10 Hz across the dwell and medianed. The status line is *not* used for these: it
   prints the raw instantaneous `last` for the voltages (no averaging, no notch) and is emitted
   only every `lfPeriod` (~3 s), too sparse to average a 100 Hz-rippled bus. The status line still
   provides the exact control state — the applied `H`, the CCM/DCM flag, and the LS rect count.
   Staying below `M` keeps the converter in DCM and the current bounded; the sweep stops on the
   firmware reporting CCM or `Iout` exceeding `--i-max`.
4. Per point compute `D = H/pwmMax` and `L = (Vin−Vout)·Vin·D² / (2·Vout·fsw·Iout)`.
5. Discard points below an `Iout` floor (sensor offset territory, §3) and report the median of the
   rest, plus an IQR spread. Restore MPPT (or `dc 0`) on exit.

Built-in checks: all-CCM points warn that `forced_pwm` may be on (which suppresses DCM); `--bidir`
compares up/down sweeps for settling. A wrong `pwmMax` biases `L` by a constant factor and is **not**
detected.

Limitations:

- **Low-bias only.** Measurements are taken well below the boundary current, so the result is the
  near-unbiased small-signal inductance. The method cannot trace the DC-bias curve — pushing the
  current up crosses into CCM, where the transfer ratio no longer depends on `L`. For the bias
  curve use a scoped CCM-ripple measurement or an LCR meter with bias injection (§2).
- The result scales with the **`Iout` calibration** (§3) and with **`pwmMax`**; dead-time and DCR
  bias it a few percent low.
- The reported value is the *physical* inductance — put it in `coil.conf` as `L0` directly; the
  firmware applies its own 0.95 bias.

Example:

```bash
python etc/measure_coil.py --ip <device-ip> --i-max 1.0
python etc/measure_coil.py -p <serial-port> --steps 12 --dwell 6
```

### On-device equivalent (`measure-coil`)

The same two sweeps run **on the device** with no host, via the console command
(`src/selftest/measure_coil.cpp`, a spawned non-RT-core task). Requires a build with
`CONFIG_FUGU_WITH_MEASURE_COIL=y` (off by default):

```
measure-coil l0 [steps] [dwell_ms] [apply]     # the §5 inductance sweep
measure-coil ls [hs]    [dwell_ms] [apply]     # the §6 LS-timing / rect_offset_ns sweep
```

It reads `Vin`/`Vout`/`Iout` directly from each sensor's `ewm.avg` (the same DC average the host
medians off the `sensor avg` line), uses `pwmMaxDriver()` for the exact PWM period (no
`MinDutyCycleLS` reconstruction), and applies the identical formula, `Iout` floor, and median/IQR.
`apply` writes `coil.conf::L0` (next boot) or `rect_offset_ns` (also live). The result is the *physical*
inductance, same as the script. Validated against the script on one Fugu2 board (board A of §7) to within ~2 % (≈50 µH).
For a tight result use many small steps (the i-max abort otherwise leaves only the low end of the
band sampled).

## 6. Synchronous-rectifier timing and the `Iout` peak

The DCM relation (§1) assumes the *ideal* triangle: the inductor discharges at exactly `−Vout`,
reaches zero, and idles. The synchronous-rectifier (low-side) turn-off timing is what makes the
real waveform match — or deviate, biasing `Iout` and therefore `L`:

- **LS off / too short** — the body diode finishes the discharge at `−(Vout+Vf)`: steeper decay,
  shorter `t2`, less charge delivered per cycle → `Iout` *below* ideal → `L` over-estimated (and
  the formula's `Vout` is really `Vout+Vf`).
- **LS too long** — held past the zero crossing, the inductor current goes negative and pulls
  charge back to the input → net `Iout` *reduced* → `L` over-estimated. This reverse-current notch,
  beating against the duty as it sweeps, is what makes `L` oscillate in a fine duty sweep.

So `Iout` — and the `L` derived from it — is least biased when LS turns off exactly at the zero
crossing, the peak of an LS sweep at fixed duty; measuring at or near that per-point optimum removes
the SR-timing oscillation. **That peak calibrates *timing*, not `L`** (at the optimum
`t2/t1 = (Vin−Vout)/Vout = 1/M − 1 = rectCtrlRatio(M)`, with `L` cancelling out). The `--ls-sweep`
peak-bracketing and the `coil.conf::rect_offset_ns` dead-time calibration it drives (`--apply`) are
rectifier-timing concerns, covered with the timing theory in
[Diode Emulation.md](../internals/diode-emulation.md) — a companion to the inductance measurement, not a
substitute.

The peak *curvature*, though, is a genuine alternative `L` handle. Just past the peak,
`Iout ≈ Iout_peak − Vout·δt² / (2·L·T)`, so `L = −Vout·fsw / (d²Iout/dδt²)`. This extraction uses the
LS-time counts instead of `D`/`pwmMax`, so it cross-checks the duty scale — but it still scales with
the `Iout` gain (§3), so it does not cross-check the sensor.

## 7. Case study: board A vs board B (two different coils)

Two field boards (Fugu2 converters in service) do **not** share a coil — assuming they did is what made the 1.57× gap look
like a sensor fault. Their hand-wound inductors:

| board  | core                  | turns | `Al`        | nominal `Al·N²`         | measured (median) |
|--------|-----------------------|-------|-------------|-------------------------|-------------------|
| A      | 2× stacked KS130-060A | 20–21 | 122 nH/N²   | 48.8 – 53.8 µH          | **50.9 µH**       |
| B      | 2× KDM KS184-125A     | 10    | 562 nH/N²   | ~56 µH (documented)     | **79.8 µH**       |

Board A's 50.9 µH lands squarely inside its own computed nominal — which *validates* its sensor and
the DCM method. Board B's `~56 µH` was only an unverified documentation figure (the turns/`Al` look
off — 80 µH needs ~12 turns at that `Al`); its measured ~80 µH is the better estimate, and an
independent battery-shunt cross-check finds board B's current gain ≈ 1.0, so its sensor is fine too.
So the 1.57× is simply two different inductors, not a measurement error on either board.

A full step-1 duty sweep on each (`measure_coil.py --steps 600 --i-max 6`, ~550 DCM points apiece,
telnet) gives the real `L` vs `H` (PWM count) below — dashed line is each board's median, points
near the CCM boundary excluded.

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

Findings:

- **The 1.57× gap is two different coils, not a sensor error.** Board A (KS130, ~20 t) measures
  50.9 µH — inside its own computed `Al·N²` of 48.8–53.8 µH — so its sensor *and* the DCM method are
  validated against a known nameplate. Board B (KS184, nominally 10 t) measures ~80 µH; its documented
  56 µH disagrees and is the suspect number (turns/`Al` likely off). An independent battery-shunt
  cross-check puts board B's current gain at ≈ 1.0, so its sensor is fine too. **The earlier "fake
  INA226 reading ~1.5× low" story is withdrawn.**
- **Each board's measured value is its true inductance, to a few %.** Take board A ≈ 51 µH,
  board B ≈ 80 µH. The shunt cross-check hints board A under-reports current ~6–7 %, which would inflate
  its `L` by the same factor and put the true value nearer the `N = 20` end (~48 µH) — a small
  correction, not a 1.5× one.
- **Both means are flat across the whole current range** (≈0.3–3 A): no downward trend, so this is
  *not* core saturation. The ±10–16 % point-to-point wiggle is the duty-pinned SR-timing
  reverse-current notch of §6, not noise in `L`.
- **Boundary breakdown (the §5 limit, observed).** Pushing board B and board A toward `H ≈ M·pwmMax`
  makes the DCM estimate diverge — board B rolls *down* (to ~37 µH), board A *up* (past ~100 µH) — as
  the waveform enters CCM and `Iout` stops obeying the DCM transfer relation. Both are artifacts of
  measuring outside DCM, not changes in the coil; the median over the clean DCM band is the result.

Practical consequence: set each board's `coil.conf::L0` to its own measured value — board A ≈ `51e-6`
(its current `56e-6` is ~10 % high, the riskier direction per §4), board B ≈ `80e-6` (its current
`40e-6` is very conservative, costing body-diode loss near the boundary). No sensor recalibration is
indicated for either.

## Open questions

- Confirm board B's ~80 µH with an LCR meter or scoped CCM-ripple measurement (§2), and reconcile it
  with the documented 10 turns / `Al` (it implies ~12 effective turns).
- The shunt cross-check hints board A under-reports current ~6–7 %; if real, trim its `Iout` calibration and its `L0` follows.