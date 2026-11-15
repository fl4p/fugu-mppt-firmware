---
name: project_tracking_loss_budget_1114
description: Measured MPPT tracking-loss budget on fry/flat (2026-11-14) — biggest loss is sitting on a wrong local peak until the 30-min sweep, not dither or sweep cost
metadata:
  type: project
---

Measured from 48 Hz Influx telemetry (mppt: P, Ui, pwm_duty, mppt_state, cv_lim_idx), 11-08..11-14.

- Calm-window P&O loss (local P-V quadratic fit per 30 s, estimate): flat ~0.6 %, fry ~0.5 %; about half is a mean offset from the peak, half dither.
- Periodic sweep cost: 0.2-0.6 Wh each (11-15 s), ~0.25 % of a morning.
- Wrong operating point until the next sweep: in 13 of 49 steady sweeps over the week the sweep gained >5 % (fry up to +65 %, flat up to +40 %).
  - fry 11-12 09:28:30: duty ran 1680 -> 3780 in 60 s while power fell 290 -> 171 W, settled on the 30 V peak (193 W) for 21 min; the 09:50 sweep restored 344 W at 66 V. cv_lim_idx=1 (Vout limiter) touched intermittently during the run-away.
  - Cause (reproduced in host sim 11-14, scenario `shade-cv`, 69 % -> 99.7 % with the fix): src/mppt.cpp "recover from limit condition" sets controlValue>0 and `tracker.resetDirection(controlValue > 0)` forces direction=up-duty and restarts the tracker timer on every limiter release, overriding P&O reversals.
  - Second pattern: fry mornings ~08:45 sit on the 72 V peak while the 57 V peak is 17-21 % higher (moving shade); only a rescan finds it.

**Why:** daily flat/fry energy ratio swings +-20 %, useless as a tracking metric; these per-window and per-sweep statistics work on existing telemetry (scripts were /tmp/trk_q.py, not kept).

**How to apply:** before tuning P&O step sizes, fix/verify the limiter-release direction forcing in the host sim (test/host-stub/mppt-sim.cpp) and add a power-drop-triggered rescan. Related: [[project_recharge_after_full_periodic_sweep]], [[project_flat_mcpwm_sweep_loop_rect_offset]].

**11-14 11:20 UTC fix deployed:** flash-autosus 3e6cbb5 + 59df7b7 (NOT on main: main checkout has other sessions' edits in src/mppt.h and mppt-sim.cpp), image fry-brk1-487-g59df7b7a on fry and flat. A limit shorter than 250 ms that never passed its limit (D-only brake) leaves the tracker alone and its duty is given back; a real (P) limit keeps the old tracker reset + release step. ee-code-verifier review done, its 4 findings applied. Not yet field-validated: check the per-sweep gain statistic over the next days (expect no more Ui ~30 V stretches on fry).
Open side findings: flat terminated charge 11-14 10:01 in a cloud dip (termination line uses instantaneous ibat, cell voltage relaxes slower; also resets ahSinceFull); sim at 1470 sps with Vout cap <= 0.5 V above the battery loops in sweep-no-mpp (old and new code alike).

**11-14 11:55 UTC:** cloud-dip termination fixed in 9a8dee7 (flash-autosus): termination line placed with the decaying peak of ibat (tau 120 s, Li_ChgTerminationCondition::_iHold); host test charger-term-test.cpp has the 11-14 case (old code fails it). It latched twice on 11-14 (10:01 flat, 11:31 fry+flat) ~90 s into a dip from ~45 A. fry and flat run fry-brk1-488-g9a8dee74. Not field-validated; watch for "Termination latched" lines in cloud dips and that a real tail still terminates.
