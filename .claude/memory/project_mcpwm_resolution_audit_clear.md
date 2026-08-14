---
name: MCPWM resolution-dependent audit: only boot_refresh_ns regressed
description: Comprehensive audit 2026-08-11: boot_refresh_ns (500→2000) was the ONLY silent LEDC→MCPWM resolution regression; ±4 ramp and 32 fade-in also fixed. All committed in 452761c6.
created: 2026-08-11T09:41:40.439Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_00fc47d03ffeOjgi2pCRIFkyJz
---

# MCPWM resolution-dependent audit: only boot_refresh_ns regressed

Audited all resolution-dependent values in the codebase after the LEDC→MCPWM migration (~2047→~4103 pwmMax). **boot_refresh_ns default 500→2000 was the only silent regression** (starved HS gate driver, measured on fbuck). Everything else is safe or has been fixed. All fixes committed in **452761c6** (2026-08-11).

**Already scaled by pwmCtrlMax/driverPwmMax** (resolution-independent):
- PD controller gains: `controlValue * (1/pwmCtrlMax) * dt` (mppt.cpp:247,257,328,359)
- Lock-in threshold: `pwmCtrlMax / 512` (mppt.cpp:347,350)
- Boot-target fade rate: `pwmCounts() / 64` (mppt.cpp:205-206)
- `largerDecrease`: `driverPwmMax / 50` (buck.h:626)
- LS overshoot log thresholds: `driverPwmMax / 64`, `/16`, `/40` (buck.h:643,647,809)

**Stored as ns, converted via tick rate** (resolution-independent):
- `rect_offset_ns` (coil.conf → rectOffsetCountsFromNs)
- `pwm_deadtime_ns` (board.conf → dtTicks)
- Low-duty gate thresholds: `pwmCountsFromNs(2000, tickRate)` (mppt.cpp:334-336)

**Fixed in 452761c6** (were hardcoded counts, now resolution-scaled):
- boot_refresh_ns default: 500→**2000** ns (buck.h:570). Not 1500: 1500 merely restores the legacy 6% (~244 counts), which is BELOW the measured 300-count safe floor on fbuck. 2000 ns = 320 counts at 39.3 kHz on the 4069-count MCPWM driver.
- Manual ramp step: was ±4 counts (mppt.cpp:391,400), now `±max(1, lround(pwmMaxDriver/512))` — preserves ~0.2%/tick across drivers
- LS fade-in threshold: was `32` (buck.h:654), now `driverPwmMax / 64` — preserves ~1.6% of range

**Intentionally 1 count** (not a calibration value):
- `pwmCtrlMin = 1` (buck.h:574): minimum nonzero duty, ~6.3ns MCPWM vs ~12.5ns LEDC, physically meaningless

**Why:** The MCPWM driver doubled pwmMax (~2047→~4103). Any hardcoded count or duty fraction calibrated against LEDC counts could silently change behavior. The audit confirmed boot_refresh_ns was the only case where this happened.

**How to apply:** If a future MCPWM resolution change is contemplated (e.g. different timing table), re-audit hardcoded counts. The values above are the complete inventory as of 2026-08-11. All hardcoded counts have been resolved — no remaining resolution-dependent issues known. Related: [[project_diode_emulation_tests_todo]] (boot_refresh_ns fix + test), [[project_tracker_pwmtable_oob_mcpwm]] (already-fixed tracker array OOB), [[project_mcpwm_validated_on_live_flat]].
