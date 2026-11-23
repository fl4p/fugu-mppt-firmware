---
name: balance-stale-pause
description: Balance-run stale-BMS pause (145adb55, 2026-11-23) — ordering trap with termCond.balance, the 180 s frozen-limit window, the 60 s in-band tolerance
metadata:
  type: project
---

Balance hold on stale BMS data pauses (termCond._staleT) instead of latching; latches 600 s into the pause (780 s after the last frame). Pinning treats the pause as terminated (nowTerm), so the pin goes bus -> Vbat_fallback and _cvIlim is dropped.

**Why:** fry+flat both latched on a ~3 min broker gap 2026-11-23 and lost the day's run.

**How to apply:**
- _updateTermination runs before _updateBalance, so termCond.balance still holds the previous tick's value on the first stale tick; during stale it is false (deferWhy "BMS stale"). Any stale-path decision about balance must be latched on that first tick.
- The resume frame must not evaluate hold exits: balance is still false then, and cv_hold_max_s=0 would time out at once. Resume shares the hold-entry branch for that reason.
- Pre-existing, unchanged: for the first 180 s of a gap the hold keeps pin=Vbat_max and a frozen _cvIlim (no ibat frames, no steps); BAL_GAP_T=60 s counts up to ~60 s of gap as in-band time.
