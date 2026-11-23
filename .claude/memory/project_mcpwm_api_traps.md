---
name: MCPWM API traps (ACTION_END, force latch, CAP order)
description: MCPWM API traps: ACTION_END sentinel breaks -Werror, force-level latches, out-of-order CAP ISR enqueues
created: 2026-11-23T09:47:20.691Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: project
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
---

Three ESP-IDF MCPWM API traps that cost real debugging time on this project.

**1. `ACTION_END()` breaks the build.** IDF 5.5 `driver/mcpwm_gen.h` defines
`MCPWM_GEN_{TIMER,COMPARE,BRAKE}_EVENT_ACTION_END()` as `(mcpwm_gen_*_event_action_t){}`.
Used with the variadic `mcpwm_generator_set_actions_on_*_event(gen, action, END())` it trips the
project-wide `-Werror=missing-field-initializers`, because the `{}` sentinel leaves `direction` and
`action` uninitialized. Won't be fixed upstream. **Use the singular variants instead:**
`mcpwm_generator_set_action_on_timer_event(gen, MCPWM_GEN_TIMER_EVENT_ACTION(...))` (and
`_on_compare_event`, `_on_brake_event`) — one action per call, no sentinel. See `src/pwm/mcpwm.h`.

**2. A force level latches.** `mcpwm_generator_set_force_level(gen, level, hold_on=true)` writes a
*continuous* force that overrides all generator actions until
`mcpwm_generator_set_force_level(gen, -1, hold_on=true)` clears it. It survives
`mcpwm_timer_start_stop`, comparator updates and action reconfiguration, so the pin produces no
output while every register looks right (enable=1, sig=160, oen_sel=0, io_mux=0x1a02, counter
advancing). Any `forceShutdown()` must be paired with a `clearForce()` on the next enable —
`src/buck.h::pwmPerturb` calls `pwmDriver.clearForce()` on the disabled→enabled transition under
`WITH_MCPWM` for exactly this reason. **Debugging signature:** gates flat at 0 V despite the timer
counter advancing in `mcpwm_dump 0`, `gpiodump` showing the right signal routed to the pin, and
`digitalWrite` driving it cleanly; a minimal IDF example (`mcpwmtest`) works on the same pin because
it never calls `set_force_level`.

**3. CAP ISRs complete out of order.** Each MCPWM CAP channel has its own ISR, so two channels firing
within ~1 µs of each other (HS-fall at t, LS-rise at t+200 ns) can push samples to the shared ring in
either order. The hardware timestamps are correct — only the **enqueue order** is wrong, which breaks
any state machine that iterates the ring in time order (e.g. `analyse_deadbands` in
`test/test_pwm.cpp`): it drops pairs or pairs across periods. **Fix:** insertion-sort the ring by `ts`
before sequential analysis, using a signed-difference compare for uint32 wraparound:
`while (j > 0 && (int32_t)(g_ring[j-1].ts - cur.ts) > 0) { … }` — O(N) on a near-sorted ring. Also:
state machines that pair edges across periods (LS-fall[k] → HS-rise[k+1]) must invalidate their
"have prev event" flag on every period boundary *and* in every no-match branch, or a missed event
leaks a stale timestamp into the next period's pairing and produces N-period outliers in the gap
distribution. Any future test with multiple CapChan on one CapTimer needs the sort. Underlying fix
that brought these tests online: [[project_mcpwm_dt_pair_two_calls]].
