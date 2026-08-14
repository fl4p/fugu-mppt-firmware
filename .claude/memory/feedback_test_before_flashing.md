---
name: Test with console commands before flashing
description: User preference: verify hypotheses on current firmware via console commands before flashing a new build
created: 2026-08-12T21:36:49.090Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: feedback
  originSessionId: ses_0081b0bb2fferLlB0o5WHIKxuY
---

Test changes on the current firmware using console commands before flashing a new build.

**Why:** The user directed "before flashing, test it with the console commands" when the agent was about to flash a new build to test a fix. Flashing takes time and risks the USB-JTAG port dropping (macOS enumeration instability). Console commands (dc, bf, sync, sensor, psu) can validate many hypotheses on the currently-running firmware without a reflash.

**How to apply:** When you have a code fix and are about to flash to test it, first check: can I test this hypothesis on the current firmware using console commands? If yes, do that first. Only flash when the fix requires code changes that can't be simulated via console.
