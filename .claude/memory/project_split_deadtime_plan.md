---
name: Split dead-time plan (per-transition HL/LH)
description: Per-transition MCPWM dead-time (pwm_deadtime_hl_ns/lh_ns) IMPLEMENTED 2026-08-26 in 40b1676; on-target + fbuck bench validation still outstanding
created: 2026-08-26T07:44:55.249Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: 01a03cfb-9c94-7c74-a6a8-ca7b53dfb57c
  modified: 2026-08-26T08:52:20.919Z
---

# Split dead-time (per-transition HL/LH)

Status 2026-08-26: **implemented and committed as `40b1676`**, built clean for esp32s3 (app +
test), 33/33 host tests pass, BLE-OTA'd to `flu`. Plan at `plans/split-deadtime.md` was followed
as written. **Still outstanding: the on-target Unity suite (needs a bench S3 attached) and the
fbuck split-value scope check.** fry/flat need no action — configs unchanged means identical
symmetric behavior — and per the plan should not be OTA'd until that bench check is green.

Keys `pwm_deadtime_hl_ns` / `pwm_deadtime_lh_ns` in `board.conf`, each defaulting to
`pwm_deadtime_ns`. hl = ctrl-off→rect-on (hardware RED, realized gap is hl−1); lh = rect-off→ctrl-on
at the wrap (`pwmMax = periodTicks − lh`, realized band is lh+1 at its tightest because every caller
caps `cmpLS` at `pwmMax−1`). Console: `dt` reports both, `dt <ns>` sets both, `dt <hl> <lh>` splits.

Three defects a codex review caught in the first cut, all fixed in the commit — re-check these if
the code is ever restructured:

- **Boot validation must not fail open.** A malformed per-edge value falling back to 0 bypasses the
  dead-time module entirely on a HiLi board, i.e. both gates switch on the same comparator event.
  It now falls back to the validated common value; an explicit 0 stays legal.
- **A runtime hl increase must widen the LS span BEFORE arming the longer delay.** Both the delay
  registers and the comparators latch on TEZ; a boundary between the two writes runs one period
  whose realized rect pulse is short by the delay increase — under the bootstrap-refresh floor.
  The clamp is therefore split around the driver write (widen before, cap after).
- **The gate verifier's `dt_lh` row must command LS to its maximum.** Its `tick_s` is the nominal
  `1/(freq*pwmMax)`, ~0.05 ns off the true tick — nothing over the 32 ticks `dt_hl` spans, but 49 ns
  over the ~1019 ticks of slack a half-width LS leaves, against a 6.3 ns tolerance.

**Why:** the plan's design decisions were user-reviewed and are settled; the three defects above are
the non-obvious traps, and the outstanding hardware validation is the reason not to touch fry/flat.
**How to apply:** don't re-litigate naming/packing/back-compat. Run the on-target suite and the
fbuck scope check before extending this to the live converters. See
[[project_mcpwm_dt_pair_two_calls]].
