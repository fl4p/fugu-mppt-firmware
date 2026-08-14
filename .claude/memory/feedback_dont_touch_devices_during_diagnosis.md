---
name: Don't send state-changing commands to devices during diagnosis
description: When investigating a device problem, only send read-only commands (status, sensor, get-config, rt-stats); ask before any state-changing command (bf, dc, sweep, mppt, +N, -N, fan, sync, restart, ota)
created: 2026-08-10T19:09:28.039Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: feedback
  originSessionId: ses_012ed403affe3kAPtBg3JZmSKr
---

When the user asks to investigate/diagnose a device problem, send ONLY read-only commands (status, sensor, get-config, rt-stats) and do NOT send state-changing commands (bf, dc, sweep, mppt, +N, -N, fan, sync, restart, ota) without explicit permission.

**Why:** During a power loop rig test on 2026-08-10, the agent sent `bf` (no arg → disabled backflow switch) and `dc` (no arg → triggered PWM shutdown + 5s backoff) to fbuck while diagnosing a power drop. This altered the device state mid-test. The user corrected: "dont touch the device." The agent then tried to "fix" it by sending `bf 1`, which the user also aborted. The existing [[feedback_confirm_before_flashing]] rule ("bench units: flash without asking") was over-generalized to "send any command without asking" — flashing firmware is NOT the same as toggling converter state during someone's test.

**How to apply:** During diagnostic/investigative work, restrict to read-only console commands. If a state-changing command seems necessary, ask first — even on bench devices (fbuck, fboost). This applies to ALL devices, not just fry/flat.
