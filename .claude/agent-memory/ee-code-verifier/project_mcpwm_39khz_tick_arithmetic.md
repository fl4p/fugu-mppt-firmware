---
name: mcpwm-39khz-tick-arithmetic
description: Worked MCPWM/LS-timing numbers at 39 kHz (period 4103, 6.25 ns/tick), the realized-edge model, and why the CAP-rig dead-time tests cannot resolve the 1-tick (dt-1) artifact
metadata:
  type: project
---

**Timer numbers, every shipped config (all are `pwm_freq=39000`).** `bestTiming(39000)` →
prescaler 1, `resolution_hz = 160 MHz` (6.25 ns/tick), `period_ticks = 4103`, realized
fsw 38 995.9 Hz. The prescaler stays 1 for any `pwm_freq` in the asserted 5 kHz–500 kHz range,
so the tick is ALWAYS 6.25 ns and a "2 tick" floor is always 12.5 ns.

**Realized edge model** (HiLi, dt armed, HS=path 1 with FED=1, LS=path 0 with RED=dt):
- HS rises at TEZ (undelayed), falls at `cmpHS + 1`  → HS on-time = `pwmCtrl + 1` ticks
- LS rises at `cmpHS + dt`, falls at `cmpLS` (undelayed, FED belongs to HS)
- realized HS→LS gap = `dt − 1`; realized LS conduction = `pwmRect − dt`
- LS→HS gap = `periodTicks − cmpLS` ≥ `dt` (cmpLS ≤ pwmMax = periodTicks − dt)
- **reverse-current risk is set by the LS turn-OFF instant, which is dead-time invariant** — a
  dt change moves only the LS turn-on, so it trades body-diode loss, not reverse current.

**boot_refresh_ns = 2000 worked numbers at dt = 32 ct (200 ns):**
`getPwmTickRate = fsw*(pwmMax+dt) = 39000*4103 = 160.017 MHz` (106 ppm above the true 160 MHz
because `pwmFrequency` is the nominal conf value, not `res/period` — safe direction, it pushes
the `ceil` up by one). `rectRefreshTicks = ceil(320.03) = 321`, `pwmRectMin = 321 + 32 = 353`,
realized LS = 321 ct = 2006 ns. The pre-fix formula (`fsw*pwmMax`) gave 318 / 286 ct / 1788 ns
— under the 300-count fbuck floor. Cost: `pwmCtrlMax` 3753 → 3718, D_max 0.9219 → 0.9133
(Vin_min for a 57 V pack rises ~0.6 V). Only fbuck_lab_bench* are affected; `config/fmetal`
runs mcpwm with `pwm_deadtime_ns` unset (=0, bypassed) and fry/flat are on LEDC.

**CAP-rig resolution trap.** The capture clock is 80 MHz (12.5 ns/tick) = TWO MCPWM ticks. The
dead-time tests (`test_mcpwm_deadband_hs_to_ls` ±50 ns, `test_mcpwm_deadtime_linearity`
|intercept| ≤ 30 ns, `test_mcpwm_deadtime_runtime_change` ±30 ns) all have tolerances ≥ 4.8
timer ticks, so **none of them can distinguish `dt` from `dt − 1`** — the 1-tick path-claim
artifact is a pure 6.25 ns intercept. Test 4a's `kExpectedDtHsLsNs` still uses `dt` while
Test 10b uses `dt − 1`; both pass. A meaningful guard needs ±3 ns on the mean (below half a
timer tick) or an explicit "closer to (dt−1) than to dt" assertion.

Related: [[mcpwm-deadtime-update-method-immediate]], [[project-mcpwm-dt-one-submodule]].
