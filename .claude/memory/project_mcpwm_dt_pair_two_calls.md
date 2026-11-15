---
name: mcpwm-dt-pair-two-calls
description: "IDF mcpwm_generator_set_dead_time configures one output path; asymmetric configs need a call per generator, not one"
metadata: 
  node_type: memory
  type: project
  originSessionId: d4ceb270-1324-43fd-9eb4-1d2cc8a41800
  modified: 2026-08-16T18:53:24.390Z
---

`mcpwm_generator_set_dead_time(in, out, cfg)` configures **one** of the
operator's two dead-time paths (path 0 = RED, path 1 = FED) and, in its
non-bypass branch, calls `swap_out_path` to route **one** output to it. A
single call leaves the OTHER output reading whatever path it happened to be
reading — so asymmetric configs (delay on one gen only, complementary pairs,
mode changes) silently mirror both pins from the same source.

**Always make two `set_dead_time` calls — one per generator.** Even when the
second would otherwise be a bypass: bypass calls skip `swap_out_path` AND undo
the previous call's bypass flag.

For HS-direct + LS-with-RED-delay (buck diode-emulation pattern), the working
sequence is:
```c
mcpwm_dead_time_config_t hs = {.posedge_delay_ticks = 0, .negedge_delay_ticks = 1};
mcpwm_dead_time_config_t ls = {.posedge_delay_ticks = dt, .negedge_delay_ticks = 0};
mcpwm_generator_set_dead_time(genHS, genHS, &hs);  // claim path 1 (FED) for HS
mcpwm_generator_set_dead_time(genLS, genLS, &ls);  // claim path 0 (RED) for LS
```

The 1-tick FED on HS is a ~6.25 ns workaround to force non-bypass mode so
swap_out_path runs. It adds 6.25 ns to HS-on, negligible in our application.

**Two consequences, both found 2026-08-16 while adding the runtime `dt` console
command (codex review):**

1. **The realized HS→LS gap is `dtTicks - 1`, not `dtTicks`** — the HS falling
   edge is itself delayed one tick by that path claim. So `dtTicks = 1` means
   NO dead-time at all, not a small one. Floor any dt at 2 ticks. (This also
   explains the ~6 ns mean shortfall in bench Test 4a.)
2. **Going from bypassed (dt=0) to armed at RUNTIME crosses the gate outputs.**
   Between the two calls, call 1 has swapped output A onto path 1 while output B
   still reads path 1 → the LS pin follows the HS waveform, both gates high
   together. Re-tuning an already-armed pair is safe (both swaps already set, only
   the RED delay register changes), so runtime dead-time changes must be
   **retune-only**: refuse when `dtTicks_ == 0`.

**Why:** the IDF docs (programming guide §Dead Time) don't say two calls are
mandatory. The signal is `mcpwm_gen.c`'s internal comment about needing the
TRM topology diagram and the fact that every test in `test_mcpwm_gen.c` does
two calls. We filed espressif/esp-idf#18654 asking for a doc note.

**How to apply:** any MCPWM driver code touching `set_dead_time` must mirror
this pattern. See `src/pwm/mcpwm.h::MCPWM_SyncLeg::init` and
`::setDeadTimeTicks` (the runtime path, with both guards above). The on-target
tests (`test_mcpwm_deadband_hs_to_ls` / `4b` / Test 10 linearity / Test 10b
`test_mcpwm_deadtime_runtime_change`) catch a regression on the next bench run.

Related: the dead-time also DELAYS LS turn-on, so a commanded LS span of
`pwmRect` is only `pwmRect - dtTicks` of actual conduction — see
[[project-mcpwm-deadtime-ls-minpulse]]; `pwmRectMin` must reserve the dead-time
on top of the bootstrap-refresh pulse.

**Third consequence, found 2026-08-19 (review round 3): the dead-time delay
registers are NOT shadowed by default, while the comparators are.**

`mcpwm_hal_operator_reset()` calls `mcpwm_ll_deadtime_update_delay_at_once()`
(`hal/mcpwm_hal.c:40`), setting `dt_red_upmethod = dt_fed_upmethod = 0` =
apply-immediately. `mcpwm_operator_config_t.flags.update_dead_time_on_tez` is
the only thing that changes it (`esp_driver_mcpwm/src/mcpwm_oper.c:97`), and
passing `.flags = {}` leaves it off — even when the comparators in the same
operator use `update_cmp_on_tez = 1`.

RED is a counter-compare, so **lowering it while an LS rise is already counting
fires that rise immediately**. Change dt from a task at an arbitrary phase and
there is a ~1/periodTicks chance the write lands between the HS fall and the
pending LS rise, collapsing the HS→LS gap to as little as zero. Growing dt is
safe; shrinking is not. Shoot-through, on a live half-bridge.

Fix: set `update_dead_time_on_tez = 1` (plus `_on_sync` for wsync followers) in
the operator config, so the delay latches on the same period boundary as the
comparators and a retune is atomic against the waveform. The init-time writes
then land in shadow and latch at the first TEZ after `start()`, which is
harmless: timer stopped, comparators 0, `forceShutdown()` immediately after.

Corollary for ordering: with TEZ latching, write the dead-time FIRST and clamp
the duty second. A rejected driver call then leaves duty and limits untouched,
and the one-period (new dt, old comparators) window is still safe because the
LS→HS band is `periodTicks - cmpLS` and depends on neither delay register.

Also: the 2-tick floor is only the arithmetic minimum (`dt-1` must be ≥1). At
160 MHz that is 6.25 ns realized — far under any MOSFET turn-off. Any
user-facing dead-time knob needs a *physical* floor (we use 50 ns realized) and
must not describe the arithmetic one as a shoot-through guard.

**Round 4 (codex, 2026-08-19) — two corrections to the above:**

1. **The counter-compare mechanism is UNPROVEN.** The IDF sources and register
   descriptions establish only that `dt_red_cfg`/`dt_fed_cfg` are shadow
   registers with a selectable update method. Neither they nor the S3 TRM say
   what RED does when its threshold drops below an already-running delay count.
   Keep `update_dead_time_on_tez = 1` — it costs nothing and gives a defined
   period-boundary update — but describe it as a precaution against unspecified
   behaviour, NOT as a fix for a measured bug. Deciding it needs an on-silicon
   test that shrinks RED at a controlled timer count and captures the FIRST
   rise; a steady-state test (like Test 10b) cannot answer it.
   Note also: the topology bits (`dt_a_outswap`/`dt_b_outswap`, bypass, input
   select) are ordinary live bits and are NOT covered by the update method.

2. **Changing dtTicks at runtime must raise `pwmRect`, not only cap it.** The
   dead-time delays LS turn-on, so realized conduction is `pwmRect - dtTicks`.
   Growing dt 32->128 at pwmRect=353 realizes 225 counts (1.41 us) — under the
   measured 300-count bootstrap floor, so the HS gate supply starves. Clamp the
   LS span into `[newRectMin, newMax - pwmCtrl - 1]` in the same transition;
   waiting for the next control tick is not safe, a stalled sampler defers it.

Related off-by-one, pre-existing at init and easy to repeat: `cmpLS` must stay
below the period, so `pwmCtrlMax = pwmMax - pwmRectMin - 1`. Without the -1 the
widest LS span at max duty is one count SHORT of pwmRectMin, permanently.

And `getPwmTickRate()` should return the MCPWM `resolution_hz` directly —
`pwmFrequency * periodTicks` is 106 ppm off (nominal vs realized fsw) and 0.05 %
off on a wsync follower, whose period is shortened while its clock is not.
