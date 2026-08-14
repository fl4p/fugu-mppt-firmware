---
name: target_duty_cycle enters manualPwm at boot with persistent manualTarget
description: tracker.conf::target_duty_cycle (fraction of pwmMaxDriver) enters manualPwm at boot with hard-fixed duty via persistent manualTarget; startSweep() skip was the key fix; clearBootTarget() on mppt/sweep/ota-fail/measure-coil; duty_max is dead
created: 2026-08-10T10:38:43.403Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_01457eb6fffe29KVup0e8IgI21
---

`tracker.conf::target_duty_cycle` is the config key for fixing the converter duty at boot. It's a **fraction** (0–1) of `converter.pwmMaxDriver` (MCPWM ~4070), not a raw count. When non-zero and finite, `mppt.begin()` (mppt.cpp:410) computes `targetPwmCnt = round(frac * pwmMaxDriver)`, sets `g_app.manualPwm = true`, sets `manualTarget = min(targetPwmCnt, pwmCtrlMax)`, and **skips `startSweep()`** (the old `startSweep()` at end of `begin()` was clearing the target — that was the root-cause bug). bflow/syncRect are enabled after `bflow.init()` (not before), gated by `!reverse_current_paranoia`.

**Runtime model (refactored 2026-08-10, commit 35f83ec0):**
- `manualTarget` (public, persistent) — survives backoff, cleared only by `dc 0`. `updateManual()` ramps to it at ±4/tick; after backoff expiry, re-enables syncRect+bflow then ramps back up. `manualTarget==0` → ramp down + disable.
- `targetDutyCycle` (public, one-shot) — used only by the sweep/MPP fade path in `update()`. Cleared on arrival. Set via `setAutoRampTarget()` for `+N/-N` in auto mode.
- `targetPwmCnt` (private) — config-derived boot ceiling. When non-zero, `update()` gates to `updateCV()` (CV with duty cap). `clearBootTarget()` clears it, called from `cmdMppt`, `cmdSweep`, `cmdOta` (on failure), and `measureCoilTask` (on restore).
- `_targetDisable` flag removed — `manualTarget==0` IS the disable signal.

**To convert raw PWM counts to the fraction:** `count / pwmMaxDriver`. fboost: `pwmMaxDriver≈4070`, so 2499 → `0.6138`. The fraction is validated (must be finite, >0, ≤1) before conversion.

`limits.conf::duty_max` is a **dead key** — the firmware `Limits` struct never reads it.

**How to apply:** `set-config tracker.conf target_duty_cycle 0.6138` for fboost at 2499 counts. Don't bother with `duty_max`. Related: [[project_cv_min_dead_conf_key]], [[project_fboost_rig_fixed_duty_2499]].
