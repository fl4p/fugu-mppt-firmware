---
name: rectcal-signal-physics
description: rect-cal LS-sweep physics -- field knee 30x lossless formula = switch-node ring (5 nF/Q40) clamp delay, forced-CCM runaway at (1-Vo/Vi)T tripped the guard, soft-panel Vin rise, knee fit vs peak fit (2026-11-14)
metadata:
  type: project
---

DCM LS sweep: output-current loss vs LS turn-off time dt relative to the inductor zero crossing
- late (lossless): dI = fsw*Vo*Vi*dt^2 / (2L(Vi-Vo)); early: fsw*Vf*dt^2/(2L), RISING toward the crossing.
Neither depends on the operating current. 80 uH, Vi 72.8/Vo 26, 6.25 ns: 3.9e-7 A/ct^2 (1.9 mA @70 ct, 8 mA @143).

**Field 2026-11-14 (80 uH board, low light, shared bus):** knee ~1.3e-5 A/ct^2 (-21..-67 mA @+70 ct, -162 @+143),
~34x the formula, onset at ideal -10..+50 ct; early side FELL 10-15 mA over 0.5..1.0 ideal (theory: rises 2-6 mA).
**Ring model fits (no scope yet):** node Cn 5 nF, parallel damping Q 40 (Rp=Q*Z0), LS conducts 35-70 ct longer
than commanded; reproduces field runs 1/3/4 at 2.4/2.7/17.6 mA rms incl. falling early side + non-monotonic tail
(dip min ~+250 ct past crossing, gain at ~1.2x ideal). Node reaches Vi only once Irev*Z0 >= sqrt((Vi-Vo)^2-Vo^2)
(0.35 A / 170 ct at 78/26.5 V): knee fit = between zero crossing and clamp onset, NOT the crossing. For Vi<2Vo no
clamp delay -> offset learned near Voc is late at low Vin (<~0.3 A reverse, harmless). vconv: setNodeRing(cn,q).
**Run-4 guard trip (full sun):** forced-CCM runaway when LS > (1-Vo/Vi)*T; calHs set from pre-ramp Vin, so at the
hold hs/M=0.755 -> runaway at ideal/0.755 = 1.32x ideal, inside the old 0.5-1.4x sweep. Fix: LS cap
min(ideal+2.5us, 0.9*(1-Vo/Vi)*T), stop 2 steps past the bend (points past the dip min wreck kneeFit), fine pass.
Guard toggle: protect() opens bflow, update() tail re-closes it same pass when ewm>-0.05 (pre-existing).

**Why it failed "V step":** knee drop dI lowers input power by dI/I; Vin rises S*dI/I with S=|dV/dI|*I/V
(S=1 at MPP; ~0.05-0.07 at Isc 1.5-2 A, 0.6 A drawn, Voc 76). -160..-360 mA knee -> Vin +0.4..1%: a fixed 0.3%
check rejects the signal itself. Bus Vout effect of the knee is only R_bus*dI (<0.1%).

**How to apply (rect_cal.h after this work, uncommitted in /tmp/rcal):** Vin step limit 0.3% + 0.25*(unscaled
raw current drop)/I, upward only (raw, not model-scaled: a 0x plant then gets no allowance -- using the scaled y
let a +1% Vin step learn +280 ct); Vout 0.2%; up to 6 re-measures; StiffV only on refs. Fit = kneeFit (line +
downward bend, left curvature <= 0): the old asym peak fit gave -193 ct / "peak at edge" on the field data.
Knee fit is ~1.5x more sensitive to an alternating bus square than the peak fit (0.19% square: -17..+24 ct).
vconv input pumping (setInputPump) is opt-in: default-on it changed wx-dawn harvest 94.5 -> 79.9% and removed
2 revI-ewm trips -- worth investigating separately. Related: [[normalized-ctrl-zero-setpoint]]
