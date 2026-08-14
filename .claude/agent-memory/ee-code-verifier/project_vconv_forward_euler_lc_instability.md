---
name: vconv-forward-euler-lc-instability
description: VirtualConverter (src/sim/vconv.cpp) L-Cout loop is forward-Euler unstable at light load; the vOut>=0 / vIn<=1.05*voc clamps only bound the blow-up into a limit cycle
metadata:
  type: project
---

The vconv plant's inductor state is advanced with the OLD `vOut_` and the cap update
then uses that cycle's average current — an explicit (forward-Euler) coupling of the
L–C_out oscillator. FE on an undamped oscillator *injects* energy, so the plant is
unstable whenever the physical damping is smaller than the numerical growth:

  growth/cycle = (ω0·T)²/4      damping/cycle = ζ·ω0·T      →  unstable when ζ < ω0·T/4

with ω0 = (1−D)/√(L·C_out) for boost, 1/√(L·C_out) for buck, T = 1/fsw, and ζ ∝ 1/R_load.

Measured (clamps compiled out, 10 mV perturbation at the exact operating point,
L=50 µH, C_out=470 µF, fsw=39 kHz, rect = pwmMax−pwmCtrl−1):

| case | ζ | predicted | observed |
|---|---|---|---|
| boost D=0.3 R=100 Ω | 0.0033 | +0.304 %/cyc | +0.303 %/cyc (→1e26) |
| boost D=0.3 R=12 Ω  | 0.028  | +0.018 %/cyc | ~0 (marginal) |
| boost D=0.3 R=2 Ω   | 0.166  | −1.6 %/cyc   | −0.044 %/cyc (decays) |
| buck  D=0.4 R=12 Ω  | 0.014  | +0.47 %/cyc  | +0.40 %/cyc (→1e34) |

**Why:** it is pre-existing (buck shows it too) and it is NOT caused by the
`if (vOut_ < 0) vOut_ = 0` clamp — with all state clamps removed the trajectory runs to
NaN instead. The clamps (`vOut_ ≥ 0` and `vIn_ ≤ voc·1.05`) are one-way valves that
convert the divergence into a bounded ~1 Hz relaxation limit cycle (Vout swinging
0…70 V, iL ±95 A). A boost hits the *input* clamp especially easily because iL is
input current in every phase and the PV source model can never absorb reverse current
(`pvCurrent() ≥ 0`).

**How to apply:** any vconv test or bench experiment that lets Vout settle must sit on
the damped side of ζ > ω0·T/4 — heavy load (e.g. r_bat 2 Ω, not 12 Ω), big C_out, or a
stiff source. If a vconv run shows a slow large-amplitude Vout/iL oscillation, suspect
this before suspecting the controller. Do not "explain" it as a clamp artifact or as an
LC ring: the ring frequency (≈730 Hz here) is not the limit-cycle frequency (≈1.4 Hz).
Related: [[vconv-boost-floor-and-ceiling]].
