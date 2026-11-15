---
name: mcpwm-deadtime-update-method-immediate
description: MCPWM RED/FED delay registers are IMMEDIATE (not TEZ-shadowed) because mcpwm_operator_config_t.flags is left {} — the fix for a glitch-free runtime `dt` change is update_dead_time_on_tez=1
metadata:
  type: project
---

`mcpwm_generator_set_dead_time()` writes RED/FED through `mcpwm_ll_deadtime_set_rising_delay`
(which stores `red - 1`, so a config of N ticks = N delay cycles). Whether that write is
shadowed is decided ONCE, per operator, in `mcpwm_oper.c:97-98` from
`mcpwm_operator_config_t.flags.update_dead_time_on_tez / _on_tep`. `MCPWM_SyncLeg::init`
(src/pwm/mcpwm.h:129) passes `.flags = {}`, so both are 0 and `mcpwm_hal.c:40`'s
`mcpwm_ll_deadtime_update_delay_at_once()` default stands: **the dead-time delay takes effect
on the next 6.25 ns clock, mid-period, unlike the comparators which latch at TEZ.**

Consequence for the runtime `dt` console command: SHRINKING the dead-time while the RED delay
counter is mid-flight (timer count c with cmpHS < c < cmpHS + dtOld) can release the LS rising
edge immediately. Realized HS→LS gap that period = (c − cmpHS − 1) ticks, which is 0 exactly
when the write lands one tick after cmpHS — a ~1/periodTicks (0.024 % at 39 kHz) zero-dead-time
event per command.

**Why:** dead-time on a 40–80 V / 900 W half-bridge is the only thing between the two FETs.
**How to apply:** if a glitch-free runtime dead-time change is wanted, set
`update_dead_time_on_tez = 1` (and `_on_sync` for wsync followers) so RED latches at the same
boundary as cmpHS/cmpLS — then the clamp-before-write dance in
`SynchronousConverter::applyPendingDeadTimeRt` becomes atomically correct instead of correct
only by argument. Caveat: with TEZ latching the very first period after `start()` runs with the
shadow default (1 tick), which is harmless only because `forceShutdown()` is applied there.
Alternative: keep immediate at init and flip the upmethod via `mcpwm_ll_*` (the file already
uses `mcpwm_ll_timer_get_count_value`) just before a retune.

Related: [[mcpwm-39khz-tick-arithmetic]], [[project-mcpwm-dt-one-submodule]].
