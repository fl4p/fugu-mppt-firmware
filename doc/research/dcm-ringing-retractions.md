*this document is an LLM generated placeholder*

# DCM ringing: retraction history

Unpublished record. These passages were removed from `website/docs/internals/dcm-ringing.mdx` on 2026-10-09, when the page was reduced to the corrected facts. They record what earlier versions of the page claimed and why those claims were withdrawn. Per-board lab data lives in `doc/lab/dcm-ringing-boards.md`.

## Removed passages (verbatim)

C_sw is per-board and depends mostly on how many FETs are paralleled. It is not a universal
constant. An earlier version of this document quoted "C_oss ~100–200 pF → 1.3–2 MHz" as if it
applied to this hardware generally. That value is roughly an order of magnitude low for any board
with paralleled 100 V FETs.

-----

| **unmeasured** (see the retraction below) |

-----

   This document previously computed `C_sw` from `T/4 = (π/2)·√(L·C_sw)` applied to the
   0 → Vin transit. That is wrong for this waveform, and the number it produced is retracted (see
   below). The transit is not a quarter-cycle starting from rest.

-----

### Retracted: board C's 3.34 nF and the 2.08× Coss claim

**Both values are withdrawn (2026-09-06).** This document previously reported board C at
C_sw = 3.34 nF, fitted from a 574 ns transit at Vin 46. It used that value to conclude that the
curated Coss curves run 2.08× high on a charge-equivalent basis.

The identification rested on the transit being *"nearly independent of Vin across
14.9–46.1 V"*, the signature of a resonant quarter-cycle. The source traces are two Vin sweeps,
run2 and run3. Reproducing this document's own published slopes confirms their identity:
19.0 → 81.9 and 19.6 → 83.7 V/µs against the quoted 18.7 → 80.3.

The following table shows that the 0 → Vin transit in those traces is not Vin-independent:

| Vin (V) | 14.9 | 20.2 | 25.3 | 30.4 | 35.7 | 40.9 | 46.0 |
|---|--:|--:|--:|--:|--:|--:|--:|
| run2 (ns) | 781 | 695 | 633 | 589 | 623 | 598 | 546 |
| run3 (ns) | 833 | 712 | 677 | 524 | 547 | 533 | 479 |

The transit falls monotonically by 30 % (run2) and 43 % (run3). The published "470–690 ns" range
is recoverable only by discarding the two lowest-Vin points of each run. Board C's `dV/dt` also
peaks at 50–60 % of swing, like board B's, where a quarter-cycle from rest must peak at the rail.

This page deliberately offers no replacement value. Refitting these traces with the resonant
model gives a `C_sw` that drifts 7.97 → 3.88 nF with Vin. The fit diverges outright on run3's
three lowest points (`V_c` = −5582 V, f = 8.4 kHz, a cosine fitted to a nearly straight segment).
At Vin 46 the candidates span 2.3–4.4 nF, which brackets 3.34 without confirming it.

Board C's node capacitance is unmeasured, not measured wrong. Anything that depended on 3.34 nF
or on the 2.08× ratio needs re-deriving. A fresh acquisition by the gate-triggered method above
would settle it.



-----

For the same reason, the constant-current model (`I_neg = Vout·t_LS/L` pumped by the low-side
minimum on-time) is **no longer refuted**. Its rejection was argued from the transit being
Vin-independent, which the table above shows it is not. Treat it as an open alternative.

-----

**Board C has no usable valley time.** The 574 ns previously
  quoted here came from the retracted 3.34 nF and must not be used to time a re-trigger. Measure
  it first.
