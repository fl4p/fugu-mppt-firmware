---
name: project-update-tail-reenables-bflow
description: shutdownDcdc() called mid-MpptController::update() is undone by the tail's bflow.enable/enableSyncRect; active() then goes false so nothing turns bflow off again
metadata:
  type: project
---

`MpptController::update()` (src/mppt.cpp) ends with `bflow.enable(aboveThres || boost || psu)` and
`converter.enableSyncRect(aboveThres)`, where aboveThres comes from the lagging EWM current. Any
`shutdownDcdc()` fired *inside* update() is followed by that tail unless the path returns early;
after the shutdown `active()` is false, loopRT stops calling update(), so bflow stays ON through the
backoff (battery->PV backfeed via the HS body diode at dusk). startSweep() does not touch bflow.

Paths found 2026-09-29: `sweep-no-mpp` (fixed by an early return after `_stopSweep`) and
`ctrl-nan` in the controller loop (NOT fixed at review time; in boost the tail enables bflow
unconditionally). Protect-trip shutdowns are safe: they happen before update(), whose top guard
(`disabled && !startCondition()`) drops bflow.

**How to apply:** when reviewing any new shutdown inside update(), check it returns before the
tail, or gate the tail on `!converter.disabled()`. See [[project_pd_control_td_and_dt_traps]] for
the stale-lastUs companion issue on the same early-return paths.
