---
name: fboost rig requires fixed duty 2499 for measurements
description: fboost must be at exactly 2499 PWM counts (target_duty_cycle=0.6138) for rig measurements; acquisition software refuses to run otherwise
created: 2026-08-10T12:00:00.000Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_01457eb6fffe29KVup0e8IgI21
---

fboost must be at exactly 2499 PWM counts for rig measurements; the acquisition software refuses to run otherwise (detects duty != 2499 and exits with "monitor-linearity exit 1"). Set via `tracker.conf::target_duty_cycle = 0.6138` (2499/4070, confirmed 2026-08-10). The device boots into `MANU` mode with duty hard-fixed at 2499 — no PD controllers, no MPPT tracking. To restore: `dc 2499` from console, or reboot (config persists). To leave manual mode: `mppt` (also calls `clearBootTarget()` to resume real tracking).

**Why:** every ladder on the rig was calibrated at 2499 counts; measurements at any other duty are meaningless.
**How to apply:** after OTAing fboost, verify `get-config tracker.conf target_duty_cycle` returns 0.6138 and `status` shows `st=MANU` with `H=2499`. If not, `set-config tracker.conf target_duty_cycle 0.6138` + `restart`. See [[project_target_duty_cycle_conf_key]].
