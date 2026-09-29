*this document is an LLM generated placeholder*

# DCM ringing: per-board data

Internal lab notes, not published. Extracted verbatim from the pre-scrub `doc/DCM Ringing.md` (commit 695feee); the generic part is `website/docs/internals/dcm-ringing.md`. Relative links inside the extract point at the old `doc/` layout; Diode Emulation is now `website/docs/internals/diode-emulation.md`.



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

