---
name: vconv-boost-floor-and-ceiling
description: Boost plant/hardware facts — Vout>=Vin floor only holds while reverse current is blocked, D=1 breaks it, and vOutMax=8*voc clips the D=0.9 (pwmCtrlMax) operating point
metadata:
  type: project
---

Facts established against `stepOneCycleBoost` (src/sim/vconv.cpp) and buck.h, 2026-08-13.

**The "Vout can never go below Vin" floor is conditional, not absolute.** It holds only
while reverse current through the HS rectifier is blocked. Two firmware-relevant escapes:
- `pwmRect` large relative to the volt-second balance point → the synchronous HS actively
  bucks Cout back into Cin. Measured: ctrl=200/rect=799, Vin=25.2 V → Vout=9.1 V, iL=−73 A.
  This is exactly the reverse-current hazard buck.h's diode emulation exists to prevent,
  and the plant reproduces it.
- `pwmCtrl == pwmMax` (D=1): LS on for the whole period, no off-time, output isolated →
  Vout decays to the sink. Not reachable through `pwmPerturb` because
  `pwmCtrlMax = 0.9·driverPwmMax` for boost (buck.h ~line 578), so the floor is
  *duty-clamp-enforced*, not intrinsic.

`disable()` sets pwmCtrl=pwmRect=0, which for a boost still passes Vin→Vout through the
HS body diode. A boost stage is never "off" — a PSU setpoint at or below Vin is
unreachable and any Vout integrator will wind up.

**Duty-floor pumping (pwmCtrlMin=1) is only a no-load hazard.** At 48→80 V, 39 kHz,
50 µH, 1 count of 1000 injects ~1.5 mW (P = ½·L·Ipk²·fsw · Vout/(Vout−Vin); the
Vout/(Vout−Vin) factor is the input's contribution during the DCM decay — omitting it
understates by 2.5x here). It cannot hold a rail against anything heavier than ~1 MΩ;
measured floor-duty settling: R ≤ 12 kΩ stays pinned at Vin, R = 1 MΩ pumped 48→64 V in
100 s. Real pwmCtrlMin is 1 count of driverPwmMax (2047 LEDC / 4069 MCPWM), so on
hardware the injected energy is 4–16x smaller still.

**Cross-checks that came out clean:** buck.h's `rippleCurrent(vh,vl)=vl/(fL)(1−vl/vh)`
equals the true boost ripple Vin·D/(f·L) when called with vh=Vout, vl=Vin; and
`rectCtrlRatio = 1/(M−1)` is the correct DCM boost tRect/tCtrl. The plant's DCM output
power matches ½·L·Ipk²·fsw·Vout/(Vout−Vin) to 0.00% over M=1.5…3.

**vOutMax = max(2·vbat, 2·voc, 8·voc) clips a reachable operating point.** With
pwmCtrlMax=0.9 the ideal ratio is 10, and vIn_ is allowed up to 1.05·voc, so a legitimate
Vout reaches 10.5·voc. Measured: voc=48, D=0.9, stiff source → plant pins at 384 V
(=8·voc) while the ideal point is 392.5 V, and the plant then stops responding to duty.
Use ≥ 11–12·voc for boost (or derive it from MaxBoostRatio).

**How to apply:** when writing/reviewing boost plant tests, don't assert an unconditional
Vout≥Vin — assert it for the diode case (pwmRect small) and assert the *reverse* behaviour
when sync rect is over-driven. Related: [[vconv-forward-euler-lc-instability]].
