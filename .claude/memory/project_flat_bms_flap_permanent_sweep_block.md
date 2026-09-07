---
name: flat BMS-flap permanently blocks startup sweep (7-6)
description: "FIXED 2026-09-07 (bmsBootDeadline no longer re-armed on fresh→stale; not yet OTA-validated). Original: flat stuck at 0W 2026-07-06: bmsBootDeadline resets to 0 on every fresh→stale BMS transition (main.cpp:941-942), so a flapping BMS prevents the 12s fallback from ever expiring → sweep permanently blocked; fry escaped only because it rebooted twice (05:20, 07:48) which reset termCond
created: 2026-07-06T08:00:20.168Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_0c989dfbaffeu8t54SjhsuwLSw
---

**STATUS 2026-09-07: fix direction (1) applied in src/main.cpp (the deadline is armed once and never reset). Not yet validated on hardware.**

On 2026-07-06 flat was stuck at 0W for 2+ hours after sunrise (Vin=67.2V, PWM=0, DCM, st=↑MPPT,0) while fry produced ~150W normally. Charger termination had latched the previous afternoon at ~14:03 Jul 5 (cell voltage hit v_term ~3.46V/cell; logs show `no Vout authority (terminated): yield -> 26.200`). The `START blocked: Vin-Vout` phase cleared ~06:11 when Vin exceeded Vout, but the startup sweep at main.cpp:921-952 never fired because `full = bool(mppt.charger.termCond) && bmsFresh` stayed true.

**Why fry wasn't stuck:** fry rebooted twice this morning (05:20 and 07:48 — uptime counters 30s and 9s in the log), which reset `termCond` (charger.h:245 `termCond.reset()` in `loadParams`, only runs at boot). flat had ~21.4h uptime (N=34,300,000 at 445 sps) and never rebooted, so its latched termCond persisted. `termCond` has no runtime un-latch mechanism — once it latches true, only a reboot clears it.

The bmsBootDeadline (12s wait, main.cpp:930 `BMS_BOOT_WAIT_US = 12'000'000`) is supposed to be a fallback: if BMS is stale for 12s, sweep anyway. But main.cpp:941-942 resets it to 0 on every fresh→stale transition:
`if (bmsWasFresh && !bmsFresh) bmsBootDeadline = 0;`
If the BMS keeps flapping (fresh↔stale), the deadline never expires → `bmsWaitElapsed` stays false → sweep permanently blocked. This is worse than the 6-16 dawn delay (34-40 min, self-resolved on BMS reconnect — see [[project_shared_bus_termination_fix_deployed]]); on 7-6 there was no self-recovery after 2+ hours. fry charging normally confirms the pack isn't actually full. The reset-on-stale logic in the current code is a bug regardless: it converts a timed fallback into a permanent block under BMS oscillation. The premature termination itself is the EOC feedback death-spiral ([[project_eoc_feedback_death_spiral]], cv_float=3.35 too low).

**Why:** The reset was intended to prevent a brief BMS outage from allowing a sweep into a full pack, but it defeats the 12s fallback when BMS oscillates rather than going cleanly stale. Separately, termCond having no runtime un-latch means any premature termination locks the converter out until a reboot.
**How to apply:** When a converter with a BMS cell source is stuck at 0W in full sun with st=MPPT,0 and no `START blocked` message, check for BMS flapping + latched termCond. The bmsBootDeadline reset at main.cpp:941-942 is the likely mechanism. `restart` clears latched termCond as a workaround. Fix directions: (1) don't reset bmsBootDeadline to 0 on fresh→stale; let the 12s timer count down regardless of BMS state transitions. (2) Add a runtime un-latch for termCond (e.g. clear when cell voltage drops below cv_float for a sustained period, indicating the pack is no longer full).
