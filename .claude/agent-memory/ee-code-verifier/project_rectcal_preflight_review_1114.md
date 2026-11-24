---
name: rectcal-preflight-review-1114
description: Pre-flight review of rect-cal (knee fit, LS cap, fine pass) and measure-coil l0 at 72c449f (2026-11-14) -- applied offset lands late on replicas, no discard command, cap/CCM derivation, measure-coil traps
metadata:
  type: project
---

Review of /tmp/dawn 72c449f (branch flash-autosus), 2026-11-14.

- Forced-CCM boundary of a DCM LS window is HS-independent: LS_eff (HS-off to LS-off) <= (1-Vo/Vi)*T. Code cap
  0.9*(1-Vo/Vi)*pm leaves 0.1*(1-M)*T = 180-280 ct. Cap is computed once at Init (window means), not re-evaluated;
  sub-window Vin moves are only covered by protect().
- calHs still uses the PRE-ramp Vin (MPP): at full sun hs/M at the hold is ~0.75, not 0.55 (fry hold hs~1046, 1.7 A).
- Replica results: field-curve replay (fry, hs 1046) learned +19..+31, applied -1..+11 ct; sim rcal-sun learned +83,
  applied +58 vs truth 0. The margin max(20 ct, 0.3|l|) does not cover the ring/clamp delay -> applied is LATE vs the
  implementer's own crossing estimate (-35..-70). Harmless magnitude (~0.1-0.3 A peak reverse) but not "earlier".
- No `rect-cal discard`: a Done result is applied at once and saved at the first >=10 s converter-off with |P|<1 W
  (dusk, backoff, dc 0, ota download, measure-coil end). Discard = `restart` before that, or revert coil.conf after.
- flat history: +140 ct offset caused midday sweep loops ([[project_rectification]]) -> accept only
  applied <= 0 on flat. Backstop drops a positive offset to 0 on first collapse.
- measure-coil l0: band from the MPP Vin read right after the mode switch (M too high near Voc; Iout 2 A stop catches
  it); never disables first, so LS = whatever sync-rect/diode emulation state MPPT left (offset applies); manual mode
  runs no PD limiters; no abort command (restart only); `measure-coil ls` writes the LS comparator from core 0.
- RT check method: python lit.py-style scan of l32r literals + call targets in update()/rt()/backstop(); objdump shows
  bogus "call" targets (_coredump_iram_end+..., rom_rx_gain_force+...) that are mis-decoded literal pools.

Related: [[rectcal-signal-physics]], [[project_lsoff_minus_one_guard]].
