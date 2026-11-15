---
name: mcpwm-cmphs-before-cmpls-shootthrough
description: MCPWM HiLi leg — raising cmpHS before cmpLS can put both gates high for most of a period if a TEZ splits the two writes; the ordering rule and the worked 75->39 kHz case
metadata:
  type: project
---

On a HiLi `MCPWM_SyncLeg` the LS generator has **no TEZ action** (`src/pwm/mcpwm.h`, `init()`:
LS HIGH at `cmpHS`, LOW at `cmpLS`; HS HIGH at TEZ, LOW at `cmpHS`). So the LS gate **holds its
level across the period wrap**. If a latched period ever has `cmpHS > cmpLS`, the LS-low event has
already passed when LS is switched high, LS stays high through TEZ, and HS goes high on top of it:
**both gates on from TEZ until the next `cmpLS`**.

Rule: **never raise `cmpHS` before `cmpLS` has been raised.** Both comparators latch on TEZ, but
they are two separate `mcpwm_comparator_set_compare_value()` calls and a TEZ can fall between them
(≈40 µs period at 25 kHz vs a driver call of a few hundred ns, plus any ISR in between — a percent
or two per change, and an ADC-continuous ISR can widen it to µs).

- widening HS (duty up in counts, or a frequency DECREASE which rescales counts up by
  `r = newTicks/oldTicks > 1`): write **LS first, then HS**.
- narrowing HS: HS first, then LS (the existing order is fine).
- `drvCommit()` is safe only because its steps are a few counts and `pwmRect >= pwmRectMin` (≈352 ct
  at `boot_refresh_ns=2000`) covers them. Anything that steps HS by more than the current LS span is
  exposed.

Worked case (found in the `pwm-freq` runtime-frequency change, `applyPendingPwmFreqRt`): 75 kHz →
39 kHz, `r=1.9236`, duty 0.61. Old `cmpHS=1281`, `cmpLS=2100` (CCM, `pwmMax-1`). New `cmpHS=2464`.
TEZ between the two writes → one period latched with `{period 4103, cmpHS 2464, cmpLS 2100}`; at the
next TEZ both gates are high for 4102 ticks = **25.6 µs**.

Related ordering fact that DOES hold: the period must be written **before** the comparators when it
grows and **after** them when it shrinks — IDF's `mcpwm_comparator_set_compare_value()` rejects
`cmp_ticks > timer->peak_ticks` (software-tracked, updated immediately by
`mcpwm_timer_set_period()`), and the hardware latches period and comparators on the same TEZ, so a
comparator can never latch above a still-running shorter period. See
[[project_mcpwm_cmp_update_on_tez]], [[project_mcpwm_39khz_tick_arithmetic]].
