*this document is an LLM generated placeholder*

# Switch-node capacitance captures (board B, board C)

Research record behind the published page `website/docs/lab/switch-node-capacitance.md`. Moved here from
`website/docs/internals/dcm-ringing.mdx` (sections "How to measure the ring", "Board B measurement (2026-09-06)",
"Board C node capacitance", "Open question") on 2026-11-18. The numbers below are copied from that page, not
re-derived.

## Board B (2026-09-06)

- Coil: L = 83 µH, measured with a DMM6500 (4-wire). `A_L·N²` gives 81.2 µH.
- LS FET: one IPP022N12NM6.
- Conditions: output open, near-zero current, Vin 71.10 V, `dc 200` (D = 0.049).
- Scope: MXO44, C1 on the switch node, C2 on the LS gate, armed on the gate's falling edge. 12 events recorded.
- Fit `v(t) = V_c − A·cos(ω(t − t₀))` over 3–93 % of the rise:
  - `ω = 2.012e6 ± 9.6e3 rad/s` (sd 0.5 % across events), `f_r = 320.2 kHz`
  - `V_c = 37.84 ± 0.11 V`, `A = 41.96 ± 0.18 V`
  - The swing would reach `V_c + A` = 79.8 V; clamped at 72.84 ± 0.10 V, one diode drop above Vin.
- `C_sw = 1/(ω²L)`: 2.977 nF with L = 83 µH, 3.042 nF with L = 81.2 µH.

## Mistaken capture (HS edge, not the ring)

`scope_backends.mxo.capture()` defaults to a slew-rate trigger (`max_transition_s = 100e-9`), which fires only on
transitions faster than 100 ns; the ring transit is of order 1 µs. A run armed on the switch node therefore records
the HS turn-on edge and looks like a successful ring capture. An earlier capture on board B did exactly that: its rise
durations fall 217 → 132 ns across Vin 35–70 V, and it was mistaken for the ring measurement for three weeks.

## Board C (2× IPP050N10NF2S HS, 2× IPP039N10N5 LS, L0 = 40 µH)

C_sw unmeasured. The existing captures are two Vin sweeps (run2, run3) armed on the switch node. 0 → Vin transit:

| Vin (V) | 14.9 | 20.2 | 25.3 | 30.4 | 35.7 | 40.9 | 46.0 |
|---|--:|--:|--:|--:|--:|--:|--:|
| run2 (ns) | 781 | 695 | 633 | 589 | 623 | 598 | 546 |
| run3 (ns) | 833 | 712 | 677 | 524 | 547 | 533 | 479 |

- The transit falls by 30 % (run2) and 43 % (run3) over the sweep. The earlier page called this monotonic; it is
  not (run2 589 → 623 ns at 30.4 → 35.7 V, run3 524 → 547 ns at the same step).
- `dV/dt` peaks at 50–60 % of swing, like board B's; a quarter-cycle from rest would peak at the rail.
- Refitting with the resonant model: `C_sw` drifts 7.97 → 3.88 nF with Vin. The fit diverges on run3's three lowest
  points (`V_c` = −5582 V, f = 8.4 kHz, a cosine fitted to a nearly straight segment). At Vin 46 V the candidates span
  2.3–4.4 nF.
- Needs a fresh acquisition triggered on the LS gate.

## Resonance vs constant-current charge

A straight line fits the central 15–85 % of every rise in these captures (board B included) to R² ≈ 0.999. Board B's
number rests on its fit converging where board C's does not, plus the body-diode clamp; it does not rest on having
excluded a constant-current charge (`I_neg = Vout·t_LS/L`, pumped by the LS minimum on-time). A Vin sweep with the
gate trigger would separate the two; it has not been run.
