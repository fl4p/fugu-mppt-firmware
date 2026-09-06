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
  | **flu** | **83 µH** (measured, DMM6500 4-wire) | **2.98–3.04 nF** (measured 2026-09-06) | **320 kHz** | **~780 ns** |
  | fbuck | 40 µH (`coil.conf L0`) | **unmeasured** — see the retraction below | — | — |

  flu carries a single IPP022N12NM6 on the low side; fbuck carries 2× IPP050N10NF2S (HS) +
  2× IPP039N10N5 (LS), four 100 V dies on the node. C_oss rises steeply toward 0 V, which is
  precisely where this ring lives.

  ### How to measure it, and the mistake this document previously made

  **Trigger on the LOW-SIDE GATE, falling.** The switch node cannot be used to trigger its own
  measurement: `scope_backends.mxo.capture()` defaults to a *slew-rate* trigger
  (`max_transition_s = 100e-9`) that by construction fires only on transitions **faster** than
  100 ns, and this ring's transit is of order 1 µs. A run armed on the node therefore records
  the HS turn-on edge and looks like a successful ring capture. That is exactly what
  `dcdc-tools/verifications/vin-sweep/flu-c1ring-20260814b` contains — its rise durations fall
  217 → 132 ns across Vin 35–70 V, and it was mistaken for this measurement for three weeks.
  The LS gate is ground-referenced and driven every cycle regardless of what the power stage
  does, so it is the only sound arming point.

  **Fit the resonance; do NOT invert a quarter-cycle.** This document previously computed
  `C_sw` from `T/4 = (π/2)·√(L·C_sw)` applied to the 0 → Vin transit. **That is wrong for this
  waveform and the number it produced is retracted** (see below). The transit is not a
  quarter-cycle starting from rest: the node swings about a *midpoint* and is truncated when the
  HS body diode clamps it one V_f above V_in, so `dV/dt` peaks near **mid-swing** rather than at
  the rail. Fit `v(t) = V_c − A·cos(ω(t − t₀))` over the unclamped part of the rise (3–93 %) and
  take `C_sw = 1/(ω²L)`.

  **Run it in diode emulation, not forced PWM.** Forced PWM holds the LS on through the zero
  crossing and abolishes the ring entirely.

  **Provenance of the flu number (2026-09-06).** Output open, near-zero current, Vin 71.10 V,
  `dc 200` (D = 0.049). MXO44, C1 on the switch node, C2 on the LS gate, armed on the gate's
  falling edge, 12 events. Fit gives `ω = 2.012e6 ± 9.6e3 rad/s` (sd 0.5 % across events),
  `V_c = 37.84 ± 0.11 V`, `A = 41.96 ± 0.18 V`, `f_r = 320.2 kHz`; the swing would reach
  `V_c + A` = 79.8 V and is clamped at **72.84 ± 0.10 V**, one diode drop above V_in. With
  L = 83 µH (measured) that is **2.977 nF**; with `A_L·N²` = 81.2 µH, **3.042 nF**. Driver:
  `dcdc-tools/bench/flu_dcm_ring.py`. Raw: `verifications/vin-sweep/flu-dcmring-20260906/`.
  Write-up: `pv/ee/plans/RESULTS-flu-csw-ring-20260906.md`.

  ### RETRACTED: fbuck's 3.34 nF, and the 2.08× Coss claim built on it

  This document previously reported fbuck at **C_sw = 3.34 nF**, fitted from a 574 ns transit at
  Vin 46, and used it to conclude that the curated Coss curves run **2.08× high** on a
  charge-equivalent basis. **Both are withdrawn (2026-09-06).**

  The identification rested on the transit being *"nearly independent of Vin across
  14.9–46.1 V"* — the signature of a resonant quarter-cycle. The source traces are
  `verifications/vin-sweep/run2-20260811` and `run3-20260811` (identity confirmed by reproducing
  this document's own published slopes: 19.0 → 81.9 and 19.6 → 83.7 V/µs against the quoted
  18.7 → 80.3). In those traces the 0 → Vin transit is **not** Vin-independent:

  | Vin (V) | 14.9 | 20.2 | 25.3 | 30.4 | 35.7 | 40.9 | 46.0 |
  |---|--:|--:|--:|--:|--:|--:|--:|
  | run2 (ns) | 781 | 695 | 633 | 589 | 623 | 598 | 546 |
  | run3 (ns) | 833 | 712 | 677 | 524 | 547 | 533 | 479 |

  A monotone fall of 30 % (run2) and 43 % (run3). The published "470–690 ns" range is
  recoverable only by discarding the two lowest-Vin points of each run. fbuck's `dV/dt` also
  peaks at 50–60 % of swing, like flu's, where a quarter-cycle from rest must peak at the rail.

  **No replacement value is offered, deliberately.** Refitting these traces with the resonant
  model gives a `C_sw` that drifts 7.97 → 3.88 nF with Vin and diverges outright on run3's three
  lowest points (`V_c` = −5582 V, f = 8.4 kHz — a cosine fitted to a nearly straight segment).
  At Vin 46 the candidates span 2.3–4.4 nF, which brackets 3.34 without confirming it. **fbuck's
  node capacitance is unmeasured, not measured wrong**, and anything that depended on 3.34 nF or
  on the 2.08× ratio needs re-deriving. A fresh acquisition by the gate-triggered method above
  would settle it.

  **What is still genuinely open on flu, too.** A straight line fits the central 15–85 % of
  *every* rise in this tree to R² ≈ 0.999, flu's included — over that span a cosine and a ramp
  are barely distinguishable. flu's number rests on its fit converging where fbuck's does not,
  plus the body-diode clamp; it does **not** rest on having excluded a constant-current charge.
  The test that separates them is a Vin sweep with the gate trigger: a resonance gives a transit
  invariant with Vin, a current-driven charge gives one proportional to it. That sweep has not
  been run.

  For the same reason, the constant-current model — `I_neg = Vout·t_LS/L` pumped by the low-side
  minimum on-time — is **no longer refuted**. Its rejection was argued from the transit being
  Vin-independent, which the table above shows it is not. Treat it as an open alternative.

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
  C_sw table above: ~125–250 ns on fry/flat, but **~780 ns on flu** (measured
  2026-09-06). A re-trigger hard-coded to the fry/flat timing would fire at roughly the
  ring's peak on flu, i.e. pump energy in rather than clamp it out, so this delay must be
  derived from the board's own `L0` and its FET population, not from a constant. **fbuck has
  no usable valley time** — the 574 ns previously quoted here came from the retracted 3.34 nF
  and must not be used to time a re-trigger; measure it first.
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
