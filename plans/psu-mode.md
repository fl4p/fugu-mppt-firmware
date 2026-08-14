*this document is an LLM generated placeholder*

# PSU mode — constant-voltage operating mode

> **Revision 3.** Most of PSU mode is now implemented (by a parallel session) and validated on a
> bench boost at 24 V. `VirtualConverter` gained a boost branch, which **resolved §4a and §4b by
> simulation** and turned up a finding that changes a BOM decision: the 14 mF/rail capacitance was
> derived from the wrong model. Revisions 1 and 2 are superseded — rev 1 in particular had a
> confirmed boot-sweep bug.

## Context

The firmware had no regulate-to-setpoint mode. `updateCV()` looked like one but was **unreachable**
(confirmed independently): the only non-init write of `targetPwmCnt` is `begin()`, which immediately
sets manual mode, and every path clearing it calls `clearBootTarget()` first. It has been deleted.

Running MPPT against a stiff battery has no MPP — the tracker pushes up, the Vout limiter pushes
back, and steady state is duty dither at the setpoint. It also inherits battery-chemistry setpoints,
a 30-minute re-sweep that ramps duty 0 → max, and `stopAndBackoff(5…30 s)` on any trip.

**Target application:** two series-stacked 80 V boosts (isolated inputs, two batteries) feeding a
full-bridge class-D subwoofer amp at 1 kW on the resulting 160 V rail.

## Decisions

| | |
|---|---|
| Mode entry | `converter.conf` sets the boot default; `psu <V>` / `mppt` switch live |
| Trip policy | Fast auto-retry, escalating to a hard latch |
| Overcurrent | CV/CC foldback — the existing limiter chain acts as the regulator |
| Crossover target | **30–50 Hz**, below the LC pole. Not 200–500 Hz (see §2) |

## Implementation status

Done: `OpMode` enum + accessors · limiter-chain PSU branch · `updateCV()`/`kCtrlSlewCV` deleted ·
boot-sweep guard in `begin()` (calibration-only init) · PSU cold-start arm · `delayStartUntil` set to
the fast-retry value · double-count guard in `stopAndBackoff` · `autoDetectVout_max`/`haveVbatMax`
bypassed in PSU · trip escalation (>4) and latch (>8) in a 60 s window · `psu-latch` in
`startBlockReason` · Vout PD reset on setpoint change · `+N`/`-N` rejected in PSU ·
OTA-fail and `measure-coil` save/restore prior mode.

**Still open:**

1. **Vout ≥ Vin setpoint guard** — reject `psu <V>` when `V <= Vin + margin`, plus a distinct
   "regulation lost" state when Vin rises toward the setpoint. Now simulation-confirmed as required
   (§4a).
2. **Fast-retry bucket is too broad** — gated on `backoffSec <= 5`, which also contains
   sensor-failure, reverse-current and `highI-noSyncRect`. Narrow to Vout-OV and supply-UV.
3. **`startCondition()` does not require the OV cause to have cleared**, so a 100 ms retry can
   re-enable into a still-overvoltage output cap. Needs a Vout-below-threshold check in the PSU
   retry path.
4. **`opMode` + `psuVsetpoint` are plain non-atomic writes.** Ordering is fixed both directions
   (setpoint before mode on entry, mode before clearing on exit), which removes the realistic
   failure — but it works by how xtensa happens to behave, not by the C++ memory model. A seqlock or
   RT-owned command mailbox is the honest fix.

## Design

### 1. Regulator: the limiter chain, not a new control law

The five-limiter `min()` already *is* CV/CC foldback (signs verified: Vin negative gain, rest
positive, error = setpoint − measurement). The only change is what happens when nothing limits — in
PSU mode take `limitingControlValue` directly instead of calling the tracker. Positive below
setpoint, ~0 at setpoint, negative when a current or power limit binds.

### 2. Crossover must sit BELOW the LC pole

The plant has a double pole at `f_LC = 1/(2π√(L_e·C))`, `L_e = L/(1−D)²`. Crossing above it with a
single-zero PI-on-duty compensator is unstable — phase budget at a 300 Hz crossover:

```
integrator −90° · PI zero @35Hz +83° · LC double pole −170°
sampled delay −11° · anti-alias RC −33°     →  −221°, i.e. PM ≈ −41°
```

≈ −8° PM even without the RC. Two zeros (type III) or current-mode control would be required, and
both are architecture changes. **Target 30–50 Hz.**

`f_LC` is inductance-sensitive, and `L` is not yet measured on this hardware (47 µH is the fboost
config value; fry is 79.8 µH, flat 50.9 µH):

| | L = 47 µH | L = 80 µH |
|---|---|---|
| f_LC (C = 14 mF) | 118 Hz | 90 Hz |
| crossover target | 30–40 Hz | 25–30 Hz |
| f_RHP | 14 kHz | 8.2 kHz |

Run `measure-coil` on the actual board before finalising gains.

### 3. Loop rate and gains

```
sensor.conf:    esp32adc1_sr=83333, esp32adc1_avg=4    -> fs ≈ 13.9 kSPS
```

Prefer this over `sr≈22000, avg=1`: identical loop rate, 4× averaging, **2× less noise** into both
the D term and the single-sample OV trip. `avg=32` at `sr=22000` was genuinely too slow — 437 sps
gives a 3.4 ms sampled delay, ~37° of phase lag at 30 Hz.

Two things must move with the rate:

- **`ctrl_vout_td` must be set.** On legacy `kd` the D contribution scales with `dt`, so an 8× rate
  increase silently costs 8× of D gain (PI zero 35 Hz → 280 Hz). Pick `Td` from zero placement
  against the target crossover, **not** by back-deriving from a legacy `Ts` — the defaults were
  tuned on fry/flat, which read Vout from the INA226 at ~450 Hz, so they encode ~18 ms, not 4.6 ms.
- **`*_filt_len` must scale.** 20 was an 11.5 ms EWM at 1.7 kHz; at 13.9 kHz it is 1.4 ms, and that
  EWM gates sync-rect enable and reverse-current protection. Scale to ~160.

### 4a. Vout ≥ Vin — RESOLVED, guard still required

A boost's output is tied to its input through the inductor and HS rectifier, so the steady-state
equilibrium is `Vin/(1−D) ≥ Vin` for all D<1. Simulation (vconv-test X, X2):

- Zero duty from 60 V with Vin=24 V into 12 Ω → decays to **23.6 V and stops**, not to zero.
- Starting below Vin, the passthrough pulls the rail **up** to the floor unbidden.
- Reverse coil current does **not** break the floor — an over-driven rect just bucks Vout back
  toward the same ratio.

So **the setpoint guard is required**: a commanded voltage at or below Vin is unreachable, and the
firmware must say so rather than wind duty up forever.

One genuine escape: `pwmCtrl == pwmMax` leaves no path to the output at all and Vout collapses to 0.
Unreachable only because `buck.h` caps duty at `pwmCtrlMax = 0.9·driverPwmMax` — the floor is
duty-clamp enforced at the top end and should not be leaned on as a safety property there.

### 4b. Duty-floor hysteresis — RESOLVED, not needed (with a stated assumption)

`pwmPerturb()` clamps to `pwmCtrlMin`, not zero, so a negative control value cannot stop the stage.
Injected power in DCM:

```
P = ½·L·I_pk²·fsw · Vout/(Vout−Vin)        I_pk = Vin·t_on/L
```

The `Vout/(Vout−Vin)` factor is **not optional** — during the DCM decay the input keeps supplying
energy alongside the coil. Omitting it understates by 2.5× at 48→80 V. At a 300 ns minimum pulse
that is **~215 mW**, not the 86 mW an earlier draft claimed. Model and formula agree to 4 digits.

Against a 1.5 kΩ bleeder per rail (needed anyway for the stored energy): **4.3 W**, ~20× more.
Simulation confirms the floor duty cannot hold 80 V into 12 Ω — it collapses to the Vin floor.

**So no disable-and-rearm hysteresis is needed — while the load or bleeder exceeds the injected
power.** That is a design assumption to state, not a property of the converter: into ~1 MΩ the floor
pumps 48 → 56 V and keeps climbing (vconv-test Y2). Fit the bleeder.

### 5. Output capacitance — the 14 mF figure was derived from the wrong model

The original sizing used `C = I_r/(π·f·ΔV_pp)`, i.e. "the cap supplies the audio-band AC component
because the loop cannot respond above 30 Hz." **That does not apply to a CCM boost.** Two reasons,
both simulated:

- The conversion ratio `Vin/(1−D)` is **load-independent** in CCM, so a fixed duty already produces
  the right output at any load. No loop action is required for a load step.
- The coil is not the limit either: slewing 11.6 → 23 A takes `11.4·L/Vin` ≈ **11 µs at 47 µH
  (0.44 switching periods)**, 19 µs at 80 µH.

Simulated 1 kW impulse (80 V, 6.94 A avg → 13.9 A peak, 25 ms burst) gives a **~1 V excursion**, and
that is largely the solver's own limit cycle (±0.37 V) rather than sag. It is near-identical from a
23 Ω or a near-open standby.

**The real sag mechanisms are ones the plant cannot currently model:**

- **Input battery droop** — likely dominant. At the impulse peak the input draws ~23 A/rail; at
  30 mΩ pack resistance that is 0.7 V, becoming ~1.2 V at the output. `VirtualConverter` has no
  Thévenin input (only `setBat()` on the output).
- **Conduction losses** (coil DCR, Rds(on), ESR) give real output impedance. The model is lossless.
- **DC-bias saturation.** The plant takes a single constant `l_`; a sendust-60 core at 23 A has
  materially less inductance than its small-signal value. `buck.h` at least undershoots `L0` by 5%
  (`InductivityDcBias`); the sim does not model it at all.

**Action: do not order 14 mF/rail on the old formula.** Re-derive from switching ripple + input-droop
buffering, and either add series input resistance to the plant or bench-measure pack droop under a
23 A step. Ripple-current rating (~4.9 A rms audio + ~5.7 A rms switching) may end up setting the
part count regardless.

### 6. Other design points (unchanged from rev 2)

- **Charger bypass** is wider than the Vout setpoint: output current still comes from
  `min(limits.Iout_max, charger.Iout_max())`, `lfControl()` still calls `charger.update()`, and
  `charger.begin()` is unconditional and asserts a positive `charger.conf::vout_max`.
- **Invalid setpoint must fail safe.** NaN means *stay disabled*, never "run the other four
  limiters" — a NaN Vout response is silently dropped from the `min()` (`NaN < x` is false).
  Missing `mode` key → MPPT; `mode=psu` with a bad setpoint → disabled + setup error; unknown
  `mode` → disabled, not silent MPPT.
- **Stuck watchdog** (`main.cpp:742`) premise is invalid in PSU at light load. For a boost
  `headroom` (Vin > Vout+8) can never hold so it cannot fire, but on a buck PSU it would reboot an
  idle rail. Gate it on *Vout below setpoint*.
- **Mode reporting** stays `MpptControlMode::CV` — no new enumerant, so the 3-bit bitfield, BLE
  advert nibble, `MpptState2String`, and LED switch are untouched. PSU-vs-MPPT surfaces via the
  status line and `cmdStatus`.
- **Config**: `converter.conf::mode = mppt|psu` (reviving the dormant key) plus `psu_vout`, and a
  decision on whether PSU uses `limits.Iout_max` or its own `psu_iout`.

## The vconv plant — what it can and cannot test

A boost branch now exists (`stepOneCycleBoost`), wired from `converter.conf::topo`. In boost,
`pwmCtrl` is the LS charging switch and `pwmRect` the HS delivering switch — the same role swap
`buck.h` does — so the coil is in series with the **input** and only the Rect / HS-body-diode path
reaches the output. Buck behaviour is bit-identical to before (verified by randomized digest).

Host build and run is ~1 s, no hardware:

```bash
clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src \
    -o /tmp/vc test/host-stub/vconv-test.cpp src/sim/vconv.cpp && /tmp/vc
```

**Covered:** mode state machine · no boot sweep · setpoint validation and NaN fail-safe · trip
escalation/latch · CV/CC foldback convergence · boost CCM ratio and volt-second balance (incl. under
real DC bias) · the Vout ≥ Vin floor and its one escape · duty-floor injected power and its
load-dependence.

**Not covered — known plant limitations:**

- **Lossless**: no coil DCR, Rds(on) or ESR, so no real output impedance.
- **No input series resistance**: the dominant sag mechanism (§5) is invisible.
- **Fixed inductance**: no DC-bias saturation model.
- **Load is a Thévenin `setBat()`**, not a modulated current sink, so audio-rate load shapes must be
  approximated by stepping `r_bat`.

**Forward-Euler instability — read before trusting any loop measurement.** `iL` is advanced using
the *old* `vOut_`, so the L/C_out loop grows energy at `(ω₀T)²/4` per cycle against `ζ·ω₀T` damping,
and is unstable when `ζ < ω₀T/4` (i.e. at light load). It is firmware-reachable — `buck.h` commands
`rect = driverPwmMax − pwmCtrl − 1` and the limit cycle appears at 1, 2 and 4 counts of slack. A
lightly loaded rig can therefore show a wild Vout/iL oscillation that is a **model defect, not a
controller bug**.

The criterion is necessary but *not sufficient* for practical trouble — magnitude matters. At
C_out = 14 mF the flagged config settles with only a ±0.37 V band (0.9% p-p), which is usable; at
470 µF it limit-cycles 0…70 V, which is not. Keep `r_bat` on the damped side for anything
quantitative, and check the criterion before blaming the loop.

## Verification

**Unit / host** (~1 s): OV-threshold derivation incl. `ovset` override and `limits.Vout_max` clamp ·
setpoint validation (negative/zero/NaN/over-limit/≤Vin → disabled) · fault-supervisor escalation,
**calibrated against a permanent fault so the latch is seen to fire** · PSU branch selects
`limitingControlValue` and never calls the tracker.

**Build matrix** — one esp32s3 build is insufficient. Run `etc/matrix_build.sh` (includes
esp32-classic) plus `CONFIG_FUGU_WITH_VCONV=y` and `WITH_MEASURE_COIL`.

**On-target, bench unit, resistive dummy load — before any amp:**
`measure-coil` to pin `L` · `mode=psu, psu_vout=80` → reboot → confirm **no sweep at boot** and the
rail reaches 80 V, not the charger's `floor(vout_max/cv_eoc)·cv_float` · `psu 70`→`psu 80` live, no
derivative kick · reject a setpoint ≤ Vin · force the current limit, confirm voltage sags rather than
tripping · force OV, confirm retry then escalation, both visible in `status` · 45+ min soak with no
periodic sweep · `grep` the log for `ADC error`/`backoff`.

**e2e:** add `psu` to `PLAN` in `etc/e2e-test/test_console_plan.py`, restoring with `mppt`.

## Risks

- **OTA rollback will not catch a control regression.** `lfMarkOtaValid` marks the image good once
  sampling is alive, not once control is proven (`main.cpp:382`). Full bench + matrix validation must
  precede any OTA to fry or flat.
- **Only one boost validation exists** — a single hardware run at 24 V from 12.7 V. Everything at
  80 V into 14 mF is still simulation plus algebra.
- **The `OpMode` refactor touches fry/flat.** Verify MPPT is unchanged on a bench unit first.
- **Series-stack hazards** (separate from PSU mode itself): the upper board floats at 80 V, so never
  plug USB into it while live — use the BLE console. Wired PWM sync cannot cross the isolation
  barrier. Mismatched batteries mean one hits `vin_min` first, and a boost with its switch off still
  passes current, so the rail lands near `80 V + Vbat` rather than halving.
- Pre-existing, worth fixing separately: `mppt.cpp:135-145`, a NaN control value is dropped from the
  `min()` (false for NaN), so a limiter that cannot evaluate its input reads as "not limiting".
