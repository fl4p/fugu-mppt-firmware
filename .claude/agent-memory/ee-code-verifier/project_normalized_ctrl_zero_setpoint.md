---
name: normalized-ctrl-zero-setpoint
description: Any current/voltage limit fed into the mppt CVP array must never be 0 — the PD controllers use normalize=true (measurement/setpoint), so a 0 A limit divides by ~0 and either slams duty to 0 or trips shutdownDcdc("ctrl-nan")
metadata:
  type: project
---

`PD_Control::update()` with `normalize=true` computes `measurement /= setpoint` (src/pd_control.h).
`VinController/VoutController/IinController` and `IoutCurrentController`/`powerController`
(PD_Control_SmoothSetpoint, which forces normalize=true, src/mppt.h:307-311) all use it. The CVP
array in `MpptController::update()` (src/mppt.cpp ~line 98) feeds `charger.Iout_max()` in as the CC
setpoint.

**Rule: never hand these controllers a 0 (or NAN) limit.** Clamp any derived limit to a small
positive floor (~0.25 A / a few hundred mV) and shut the converter down explicitly instead.

**Why:** setpoint→0 gives `measurement/0`:
- iout>0 → cv = -inf → `!isfinite(cv)` → `shutdownDcdc("ctrl-nan")` + 5 s backoff (only fires while
  `!disabled && duty>0.01`), i.e. an ERROR-level log for a normal policy decision.
- iout==0 → NaN → `NaN < limitingControlValue` is false, so the limiter is *silently skipped*.
- before the underflow, iout/tiny is a huge negative cv → `fp` is clamped to `-pwmCnt`, duty to 0 in
  one tick, then bounces at pwmCtrlMin. Works, but as a limit cycle, not as a clean stop.
`IoutCurrentController` is a *SmoothSetpoint* (EWMA span 200), so the setpoint decays toward 0 over
seconds-to-minutes and reaches exactly 0 only on float underflow — the failure is delayed and
therefore easy to miss on the bench.

**How to apply:** check this whenever a new source writes `BatteryCharger::ioutLim`, `limits.Iout_max`,
`Iin_max`, `P_max` or the PSU setpoint. Historical note: `ioutLim` was dead (only ever assigned NAN)
until the 2026-09 battery-temperature policy made it live with a literal `0.f` for the cold block and
for t >= bat_temp_max.
