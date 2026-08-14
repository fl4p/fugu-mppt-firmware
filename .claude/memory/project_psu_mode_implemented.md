---
name: PSU constant-voltage mode (vconv-validated, real-HW gate driver broken)
description: PSU mode implemented 2026-08-12; 11 review bugs fixed; vconv fully validated; real-HW testing found bflow/startCondition/Vr-sensor-fail/duty-cap issues (all fixed); bench device gate driver supply confirmed broken — need different board for real-HW validation
created: 2026-08-12T15:09:10.371Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_007fe60e6ffe8bY56mhRNIii2X
---

PSU constant-voltage mode implemented 2026-08-12 per plans/psu-mode.md. Third operating mode (`OpMode::Psu` in app_state.h) alongside MPPT and Manual. The limiter chain in `update()` regulates Vout to `psuVsetpoint` with CV/CC foldback — no MPPT tracker, no periodic sweep, no charger-layer battery semantics. Entry: `converter.conf::mode=psu` + `psu_vout=<V>` (boot default), or `psu <V>` console command (live). Exit: `psu off`, `mppt`, `sweep`, or `dc N`. Trip policy: fast 100ms auto-retry for ≤5s trips, escalation after 5, hard latch after 8 in 60s window. Dead `updateCV()` + `kCtrlSlewCV` deleted. `manualPwm` bool → `OpMode` enum. 16 unit tests in test/test_psu_mode.cpp.

**Code review (Codex agent) found 3 CRITICAL + 12 WARNING bugs — 11 FIXED:**
1. +-N rejected in PSU mode. 2. Trip double-counting (stopAndBackoff skips if already disabled). 3. stopAndBackoff sets 100ms delay in PSU. 4. VoutController.reset() on setpoint/mode change. 5. OTA saves/restores prior opMode. 6. measure_coil saves/restores prior opMode. 7. Stuck watchdog gates on Vout<setpoint-2V in PSU. 8. Boot calibration started in PSU. 9. Unknown mode/invalid psu_vout → setupErr. 10. PSU current limit uses limits.Iout_max directly. 11. Cross-core ordering: opMode before psuVsetpoint on exit.

**vconv validation PASSED** (bench ESP32-S3, MAC 34:85:18:82:40:28):
- Vout=28.00V exact, 577W, 0 trips, no sweep. Setpoint changes work. `mppt`→MPPT (813W), `psu 28`→PSU (no sweep on either). OV/OC trips work. +-N rejected. CC foldback works. e2e ALL PASS.

**Real-hardware fixes (all applied, compiled, app-flashed to bench device):**
- **bflow enable in PSU**: `update()` enables bflow when `psuMode()` (mppt.cpp:285); cold-start arm also enables bflow before pwmPerturb(1) (main.cpp:975). Without this, `highD-bflowOpen` protect trips immediately in buck PSU mode.
- **startCondition bypass**: PSU skips `Vin < Vout + 1` boost check (mppt.h:407) — boost PSU starts with Vout=0.
- **Vr-sensor-fail bypass**: Skipped in PSU when `Vout < psuVsetpoint` (mppt.h:639) — no-load boost at high duty has low Vout, falsely triggers sensor-fail check.
- **Boost duty cap**: `pwmCtrlMax` for boost capped at 90% (buck.h:577) — at 100% LS FET is permanently on, shorting inductor.
- **ADC attenuation fix**: See [[project_adc_uniform_atten_requirement]].

**Real-HW result (2026-08-12/13):** Buck PSU mode on bench device (fmetal config, `psu 10`): converter starts, no trips, bflow on, duty commanding (HS~3782), Vout slowly rising (0.9→1.25V over 30s) but Iin=0.00A — **no power transfer**. User confirmed: **gate driver supply on this bench board is broken**. Not a firmware issue. Need a board with working gate driver (fry/flat or another bench unit) to validate real-HW PSU mode.

**New config:** `config/psu/boost80V/` — boost, `mode=psu`, `psu_vout=24`, MCPWM, `reverse_current_paranoia=1`. Uses INA226 for Vin/Iin, internal ADC for Vout/NTC (same as solar-boost). `sdkconfig.vconv_s3` fragment for vconv builds on S3 (see [[project_vconv_on_esp32s3]]).

**Why:** Boost supply for audio PA needs real CV mode; MPPT against stiff battery = sub-30Hz rail wander (in-band for subwoofer) + 30-min re-sweep.

**How to apply:** Do NOT OTA to live converters until on-target verification (plan items 5-11) with resistive dummy load on a board with working gate driver. Before connecting any amp: verify rail comes up to setpoint, no sweep at 45+ min, CC foldback, OV escalation.
