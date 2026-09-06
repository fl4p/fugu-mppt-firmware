*this document is an LLM generated placeholder*

# DCM Coil Ringing

In discontinuous conduction mode (DCM), when the inductor current reaches zero the LS
rectifier turns off. The residual energy in the inductor then resonates with the parasitic
capacitance at the switch node — MOSFET C_oss, LS FET junction capacitance, PCB trace
capacitance — forming an LC tank. In a synchronous buck with diode emulation (this
firmware, `src/buck.h`), the LS FET's C_oss is the dominant capacitance and the **main
inductor L** is the tank inductance.

**Not to be confused with the HF transition ring.** There are two distinct ringing
phenomena on the switch node:

- **HF transition ringing** — occurs at *every* HS/LS switching edge, caused by the
  *parasitic loop inductance* (trace + package ~10–30 nH) resonating with C_oss at
  tens to hundreds of MHz. Duration is a few ns; addressed by snubbers, gate-R, and
  layout. This is what most app notes (TI slyt465, Specter Engineering, ADI) focus on.
- **DCM coil ringing** (this document) — occurs *only after* the inductor current
  reaches zero and both FETs are off, caused by the **main power inductor L**
  resonating with the switch-node capacitance C_sw. Frequency f_r ≈ 1/(2π√(L·C_sw)),
  well above f_sw (39 kHz) but far below the HF transition ring. The ring duration is
  hundreds of ns to µs, and its lower frequency makes firmware active damping feasible
  (T_ring/4 is comfortably within MCPWM tick resolution).

  **C_sw is per-board and is dominated by how many FETs are paralleled** — it is NOT a
  universal constant. An earlier version of this document quoted "C_oss ~100–200 pF →
  1.3–2 MHz" as if it applied to this hardware generally. That is roughly an order of
  magnitude low for any board with paralleled 100 V FETs:

  | board | L | C_sw | f_r | T_ring/4 |
  |---|--:|--:|--:|--:|
  | fry / flat | 50–80 µH | ~100–200 pF *(estimate, unverified)* | 1.3–2 MHz | 125–192 ns |
  | **fbuck** | **40 µH** (`coil.conf L0`) | **~3.3 nF** (measured) | **~435 kHz** | **~574 ns** |

  fbuck carries 2× IPP050N10NF2S (HS) + 2× IPP039N10N5 (LS) — four 100 V dies on the
  node, and C_oss rises steeply toward 0 V, which is precisely where this ring lives.

  **Provenance of the fbuck number.** With the output open and near-zero current, the
  switch node was measured (MXO44, 12 events/point, triggered on the LS gate falling
  edge) rising from 0 V to Vin in **470–690 ns**, monotone, with the duration nearly
  INDEPENDENT of Vin across 14.9–46.1 V while the slope scaled ~linearly (18.7 →
  80.3 V/µs). A Vin-independent duration is the signature of a resonant quarter-cycle:
  `(π/2)·√(L·C_sw)`. Inverting the 574 ns measured at Vin 46 against the configured
  L = 40 µH gives **C_sw = 3.34 nF**, and the same model predicts 84.7 V/µs against
  80.3 measured (−5%). Note this C_sw is a FIT to one measurement, not a datasheet sum
  or a bridge reading; the agreement with the ~3 nF expected from four 100 V dies is a
  cross-check, not an independent measurement. The low-Vin end fits worse (27.4 V/µs
  predicted vs 18.7 measured), as expected when more of the swing sits in the
  high-C_oss region near 0 V.

  **This number also tests the curated Coss curves, and they run high against it.** For
  fbuck's exact stack the curves give `Q_node` = 319.5 nC at 46 V, i.e. a **charge-equivalent
  6.945 nF** (= Q/V, the quantity a hard-switched edge actually moves) — **2.08×** the 3.34 nF
  measured here. That is NOT 2.08× of error: the two are different averages of the same
  nonlinear `C(v)`, and the *energy*-equivalent of the same curve is already 1.61× the measured
  value. But it does mean a datasheet-summed `Q_node` should not be treated as a fixed point.
  Measured 2026-09-06 from `pv/ee/dcdc-tools/scratch/coss_provenance_20260906.py`; the context
  is `pv/ee/plans/RESULTS-flu-fsw-lowduty-20260906.md` §R4, where a hard-switching loss ceiling
  built on the curated `Q_node` left 5.8 µJ/cycle unexplained on flu.

  **If flu's ring is ever measured, do it at Vin 46**, the same point fbuck was taken at. The
  method's bias moves with where the swing sits on `C(v)` — this doc's own fit runs −5 % at
  Vin 46 but −32 % at the low end — so a ratio taken at a common voltage cancels it and a ratio
  across two voltages does not. At a common 46 V the curated curves predict flu/fbuck = **1.113**
  (flu's node is the *larger* of the two, the 2.2 mΩ IPP022N12NM6 being over half of it), i.e.
  **expect C_sw ≈ 3.72 nF**. Measuring instead at flu's own 70.55 V would predict 3.01 nF but
  carries an uncancelled method bias.

  A competing model — a constant current `I_neg = Vout·t_LS/L` pumped by the low-side
  minimum on-time charging C_sw — was tested and **refuted**: with the same constants it
  over-predicts dV/dt by 8.6× and predicts a Vin-INDEPENDENT 67 ns rise, contradicting
  the measured 470–690 ns. Both models predict slope ∝ Vin, so that scaling alone does
  not discriminate between them; the magnitude and the constant duration do.

  Raw traces and the full write-up: `dcdc-tools/verifications/vin-sweep/`.

This is a companion to [Diode Emulation.md](Diode%20Emulation.md), which covers the ZCD
timing. Here we cover what happens *after* the LS FET turns off at the zero crossing.

## Reduction techniques

### 1. RC snubber (hardware)

Series R+C from the switch node to ground, placed as close to the LS FET as possible. The
standard design procedure (ADI, TI, Severns):

1. **Measure ringing frequency** f_r on the switch node with a scope (bandwidth limiting
   off, short ground spring).
2. **Find parasitic C**: add capacitance C_ext from SW to GND until f_r drops to **half**.
   Then C_parasitic = C_ext / 3 (total C quadrupled → f halved, since f ∝ 1/√(LC)).
3. **Find parasitic L**: L_parasitic = 1 / (4π² · f_r² · C_parasitic).
4. **R_snubber = √(L_parasitic / C_parasitic)** — the characteristic impedance of the tank.
   This is the value that critically damps it.
5. **C_snubber = 3–10× C_parasitic** (ADI says 1–4×, TI says 3–10×). Larger C = more
   damping but more loss.
6. **Snubber loss**: P = ½ · C_snub · V² · f_sw — the efficiency penalty per cycle.

Practical tuning (from the EE StackExchange thread on an LM2576 buck): the calculations
get you close, but the final value should be found empirically — solder in a pot and tune
R for best waveform. The RC time constant should be ≥ 3× the ringing rise time. Don't
chase critical damping; "tidy waveform" costs more efficiency than it's worth. Final
values for that example: 1 nF + 470 Ω.

For a nonsynchronous buck, the ringing voltage never exceeds the switching voltage, so
it's primarily an EMI concern, not a reliability one. In a synchronous buck the same
applies — the ring is bounded by the tank energy, not by V_in.

### 2. Gate resistance tuning (hardware / firmware)

Increase the LS FET turn-off gate resistance to slow dv/dt at the moment the FET opens.
This reduces the excitation energy injected into the LC tank. Trade-off: higher switching
loss. In this firmware the MCPWM dead-time pair and gate driver strength already influence
this; adjusting the dead-time can marginally affect ringing amplitude. Specter Engineering
showed a near-linear decrease in overshoot vs. gate resistance from 1–20 Ω.

### 3. Layout optimization (hardware)

Minimize the high-di/dt commutation loop area: keep the input capacitor, HS FET, and LS
FET as physically close as possible. ~10 nH per 25 mm of trace. Smaller loop = less
parasitic L = less stored energy = lower overshoot. This is the single most effective
hardware change but requires a board respin. Planar/laminated busbars or adjacent PCB
power layers for magnetic field cancellation.

### 4. Forced CCM (firmware)

Keep the converter in continuous conduction mode so the inductor current never reaches
zero and DCM ringing never occurs. Options:

- **Minimum preload** (bleeder resistor on the output): the converter always draws enough
  current to stay in CCM. Simple but wastes power continuously.
- **Force CCM in firmware**: keep the LS FET on even at zero current, allowing negative
  inductor current (forced PWM). Eliminates DCM ringing entirely but introduces
  circulating current losses — at light loads this can be worse than the ringing.
  **Already available in this firmware**: `sync forced` console command or
  `forced_pwm=1` in `converter.conf` (`src/buck.h:453`). Sets `forcedPwm=true`, which
  bypasses DCM detection (`computeDCM` returns false, `src/buck.h:721`) and keeps LS
  on for the full complementary half-cycle.
- **Lower inductance**: raises the critical load boundary (I_crit = ΔI_L / 2), extending
  CCM to lower loads. Trade-off: higher ripple, larger core losses.

### 5. Pulse skipping / burst mode (firmware)

Instead of operating in DCM at light loads, skip switching pulses entirely. The converter
fires a burst of CCM pulses to charge the output, then goes idle until V_out droops below
a threshold. Each pulse is a CCM pulse, so DCM ringing doesn't occur. Trade-off:
lower-frequency output ripple at the burst frequency, potential audible noise. Many IC
controllers (onsemi FAN65008, TI LM5146) do this automatically.

For this firmware: implement by setting a minimum PWM duty floor — if the computed duty
falls below a threshold, either hold PWM at 0 (skip) or fire a minimum-width CCM pulse
periodically.

### 6. Active damping via LS FET (firmware, specific to synchronous buck)

The ringing is between L and C_oss of the LS FET. Instead of letting it ring freely after
the LS FET turns off:

- **Keep the LS FET on briefly** after zero-current detection to clamp the switch node to
  ground, dissipating the residual energy in R_ds(on) before releasing it. The FET acts as
  a controlled dissipative element. This is the most promising firmware-only approach for
  this project — the existing `SynchronousConverter` (`src/buck.h`) already computes
  diode-emulation timing; a brief LS-FET re-trigger pulse at the first ring valley
  (~T_ring/4 after LS-off) could be added. **The valley time is per-board** — see the
  C_sw table above: ~125–250 ns on fry/flat, but **~574 ns on fbuck** (measured). A
  re-trigger hard-coded to the fry/flat timing would fire at roughly the ring's peak on
  fbuck, i.e. pump energy in rather than clamp it out, so this delay must be derived
  from the board's own `L0` and its FET population, not from a constant.
  - **MCPWM**: the ESP32-S3 has 2 comparators per operator (`SOC_MCPWM_COMPARATORS_PER_OPERATOR=2`),
    both already used for HS and LS edges (`cmpHS_`, `cmpLS_` in `src/pwm/mcpwm.h`). An LS
    re-trigger pulse would need a different mechanism — e.g. a second operator's comparator
    cross-triggered, or the dead-time submodule, or a software-triggered one-shot event.
    Feasible but not trivial; no spare hardware comparator is available.
  - **LEDC**: not feasible — LEDC uses 2 channels (one for HS, one for LS), each producing
    exactly one pulse per period via duty/hpoint registers. There is no event system for
    generating an additional edge.
  - Note: `boot_refresh_ns` / `pwmRectMin` (2000 ns default, `src/buck.h:574`) is a
    *minimum LS on-time* for HS bootstrap cap refresh — it keeps LS on *before* the ring
    starts, not after. It is not a ring damper; active damping needs a *new* LS re-trigger
    pulse timed to the ring valley, which is a different mechanism.
  - **`rect_offset_ns`** (`coil.conf`, `src/buck.h:588`) already provides partial
    damping: it delays LS turn-off slightly past the true zero crossing, so a small
    reverse current flows, loading the tank and dissipating ring energy. Calibrated per
    board (fry +100, flat +78 counts). Increasing it further damps more but pushes
    operating point toward the reverse-current cliff (see
    `project_rect_offset_and_intrinsic_oscillation.md`).
- **Turn the HS FET on early** (before the next nominal cycle) to clamp the switch node to
  V_in, absorbing the ring energy into the input cap. Requires precise timing; used in
  some advanced controllers.
- **Briefly re-turn the LS FET on** for a few ns at the zero-crossing to short the tank.
  This is the "active clamp" approach (see US8933635B2 patent for LED driver
  application).

### 7. Schottky diode in parallel with LS FET body diode (hardware)

An external Schottky diode has near-zero reverse recovery current, reducing the current
step that excites the RLC tank at turn-off. Specter Engineering's simulation showed ~90 V
overshoot reduction. This is a BOM change, not firmware.

## Summary

| Technique | HW/FW | Effectiveness | Efficiency cost | No board change? |
|---|---|---|---|---|
| RC snubber | HW | High | Low–moderate (½CV²f) | No |
| Gate R tuning | HW/FW | Medium | Moderate | Partially (dead-time) |
| Layout optimization | HW | High | None | No (respin) |
| Forced CCM | FW | Complete | High at light load | Yes |
| Pulse skipping / burst | FW | High | Low | Yes |
| Active damping (LS keep-on) | FW | High | Low | Yes (experimental) |
| Schottky parallel diode | HW | Medium | Low | No |

For this project, the most promising **no-hardware-change** approaches are burst/pulse-skip
at light load and active damping via a brief LS-FET keep-alive pulse at the zero crossing.
Both build on the existing diode-emulation logic in `src/buck.h`.

## Sources

- Analog Devices, "The Unseen Ring: Taming Parasitics in Buck Converters Using a Snubber"
  (2026-03-16) — https://www.analog.com/en/resources/technical-articles/the-unseen-ring.html
  — full snubber calculation + LTspice optimization method
- Specter Engineering, "Switch Node Ringing" (2019-10-04) —
  https://www.specterengineering.com/blog/2019/9/26/switch-node-ringing — first-principles
  RLC analysis, gate-R sweep, Schottky parallel diode, snubber
- EE StackExchange, "Snubbing DCM (nonsynchronous) Buck converter" —
  https://electronics.stackexchange.com/questions/246301/ — practical tuning outcome
  (1 nF + 470 Ω on LM2576), "snubbing improves waveforms, not makes them perfect"
- TI, "Controlling switch-node ringing in synchronous buck converters" —
  https://www.ti.com/lit/pdf/slyt465
- ResearchGate, "Impact of inductor current ringing in DCM on output voltage of DC-DC buck
  power converters" (2017) —
  https://www.researchgate.net/publication/317525001
- US Patent 8933635B2, "Method of preventing spurious ringing" —
  https://patents.google.com/patent/US8933635B2/en — active clamp for LED drivers
