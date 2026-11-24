---
name: project_charger_termination
description: Charger termination on the shared fry/flat bus — EOC feedback, float/yield pins, voutAuthority asymmetry, CV hold, top-balance run, holiday mode, recharge release, cv_float key
metadata:
  type: project
---

fry and flat charge one 8S LFP pack (bat_caravan) through one BMS; see [[project_charger_bms_feed]] for the feed, flat's Vout
sense error and stale-data handling. Termination logic lives in `src/charger.h` (`Li_ChgTerminationCondition`,
`_updatePackVoltagePinning`). Charger params load only at boot: `set-config charger.conf …` needs `restart` (cli.cpp loads
into a throwaway struct and only validates). Several set-config in one MQTT console session: only the first landed, send one
per connection.

## cv_float, not cv_min
2026-05-28, still true. The float key is **`cv_float`** (struct member is `cv_min`); a `cv_min` key is never read. Since
506c4d29 (11-23) `status` prints `cv_float=`. Incident: on-device cv_float=3.25 (below the LFP rest plateau ~3.32) plus a
hand-added cv_min=3.38 → Vout pinned to ~26 V below the pack → no charge. Fix `set-config charger.conf cv_float 3.38`,
`del-config charger.conf cv_min`, `restart`.
**Why:** key-name vs member mismatch is a silent landmine. **How to apply:** edit `cv_float`; when charging is stuck with
cells above float, suspect cv_float below the plateau. On-device values drift from repo configs (devices ran 3.35 while
config/fmetal said 3.37); read the device, not the repo. `limits.conf::duty_max` is likewise dead.

## Recharge-after-full (May 2026)
FIXED. Layer 1 (05-20): the 30-min periodic sweep ran into a full pack; gated on `termCond || mode==CV` (mppt.cpp).
Layer 2 (05-24, 08db36d): while terminated the pin tracked vbat_avg down; now holds absolute `Vbat_fallback`
(N×cv_float) with a 5 s glide. Failed experiment: PI on BMS ibat → ioutLim (stash WIP-PI-experiment-2026-05-24) tripped
the `iOutSmall && D*0.8 > vr` "sensor/HB fail" protection (mppt.h) at light-load DCM; any float control must coexist with
that protection — keep a tight VoutCTRL ceiling. Also: VFLOOR release needs 4 consecutive sub-threshold BMS frames; gating
the auto-restart sweep (main.cpp ~670) on `!termCond` was reverted (flat stayed disabled after every INA226 latency trip) —
don't reintroduce.

## Shared-bus yield and CV-floor latch (June)
2026-06-09..16, all deployed. Chain of fixes:
- 32540e31: EOC pin feedback gated on Vout authority + `releaseVoutPinning()` (old fw latched the pin at the floor once a
  sibling took the bus; fry drop-outs 06-10 were simply stale pre-fix fw — check the live version, `elf_archive list` was stale).
- 0bb5fb2: release goes to `Vbat_max`, not `Vbat_fallback` (fallback ≈ resting pack → CV-throttled to ~90 W after each sweep).
  BMS-stale path still targets fallback (conservative). Test `test_release_vout_pinning_goes_to_vbat_max`.
- 12d5876: terminated && !voutAuthority → yield LOW to `Vbat_fallback − vout_offset_max` (was climbing to Vbat_max and
  re-injecting 477 W, ~60 s limit cycle); EOC feedback gain ×n_cells (issue #56 dimensional bug). 744995f: bat_c optional.
- 7d62478: authority = `iout>0.5 A || termCond` (and duty > minAuthorityPwm).
- b310345: CV-floor stuck watchdog skipped while termCond (a full pack at 0 W is healthy).
- c6ac56b+babe25b: no startup sweep while termCond; at boot defer until `terminationDecided()` (cell voltage AND warm ibat),
  12 s fallback. NVS-persisting termination REJECTED (stale flag would refuse a depleted pack); persist Ah, not the flag.
- 179879d: `full = termCond && haveValidCellVoltage()`; 64-bit `wallClockUs/Ms` (micros() wrapped at 71.5 min).
- flat afternoon bursts 06-15 were config: flat cv_float 3.380 vs fry 3.350 + flat's low Vout reading → aligned to 3.350.
- Left open (cosmetic): atomic<float> pins, vestigial ioutLim, dead balancingMode branch, unused vbat arg.

## voutAuthority asymmetry
Confirmed 06-30, 07-07; root of "one charger at 0 W after termination". With authority the pin glides to Vbat_fallback
(26.8 V) and can still produce (flat made 400-530 W on 07-07 because it reads low, so 26.8 V set ≈ 27.1 V real); without
authority it sits at the 26.2 V yield floor, duty 1, 0 W. fry lost both times. Log tell: `no Vout authority (terminated):
yield X -> Y`. Partly addressed by aebc8e4 (terminated sibling joins at float, 10-31) and the BMS-shared deficit
([[project_charger_bms_feed]]); the join causes the accepted highD trip ([[project_backflow_wire_guard]]).

## Load-following plan
Superseded in part (09-07). `plans/load-following.md`: Phase 1 (terminated + discharging ibat<−0.1 A → pin at
Vbat_fallback, allow sweeps) still open; Phase 2 replaced by `charger.conf partial_charge` hold: ibat-frame-gated
integrator on vpack_pin (±20 mV/frame, 4 mV/A, 0.5 A deadband). partial_charge set 0 on fry+flat 11-18 (full charge at home).
`recharge_vfloor_band` configurable, default 0.05 V (OTA'd 06-28; code had drifted to 0.1).

## EOC feedback death spiral
FIXED in code 09-07, not explicitly field-validated (superseded in practice by the CV hold). Old EOC target
`min(cv_eoc, v_term(ibat))` with `v_term = cv_float + r·ibat`: pin cut → current drops → v_term falls → more cut →
latched at ~3.40 V/cell (07-02 ~84 % SoC, 07-03, 07-07; both units latch together via the shared BMS). Fix: target
`cv_eoc` while charging, `cv_float` once terminated; `termCond.update()` once per cell frame, VTERM_STREAK_REQ=2. Premature
termination also shifts the recharge band (DoD release is calibrated to true full). Cloud-dip latch (11-14) fixed 9a8dee7:
line uses a decaying ibat peak (tau 120 s, `_iHold`); test in charger-term-test.cpp. Not field-validated: watch "Termination latched" in cloud dips and that a real tail still terminates.

## Coulomb counter: no clamp
2026-05-25, rule. `CoulombCounter` (`src/etc/coulomb_counter.h`) integrates −ibat; `ahSinceFull` going negative from
sibling charging is correct. **Why:** clamping at 0 wipes discharge each sibling cycle → `recharge_dod` releases far too
early. **How to apply:** never clamp; per-charger baselines are now shared via the BMS charge topic ([[project_charger_bms_feed]]).

## CV hold before termination
Live since 11-20 (8cc90174, fleet 775), `cv_hold_max_s 5400` on both. Origin 11-19: latch on the top cell (3.507 V at 31 A)
because the line is flat at cv_eoc above 14 A, so batmon never got an "all cells ≥3.40 V in CV" anchor. Hold: constant
target cv_eoc (never v_term), K 0.5, ≤30 mV/frame, pin in [Vbat_fallback, Vbat_max], up-steps ≤ Vout+0.2 V, frozen
without authority or in PSU mode, `chargeHold()` blocks the periodic sweep. Exits: tail (ibat < tail_c_rate·Cbat for
tail_hold_s 180 within cv_hold_band 0.02), cv_hold_max_s timeout (still marks full), stale BMS (keeps the Ah ref).
cv_ceiling 3.55 explicit. Dashboard state `cvhold` (4ffa0444). Log: `cv hold ended: tail/timeout`.
OTA lesson (fry 11-20): a task-WDT self-restart in setup while PENDING_VERIFY silently rolled back and the retry masked
it — after an OTA read the AFTER table AND the log for a second "Rebooting".

## Top-balance run
Live since 11-23 (b4158335 + 75ff260f); fleet on 1009-g44e16128 since 11-24 15:30 UTC. Fab: top-balance about monthly. Holiday/partial-charge interplay host-tested only.
CV hold at cv_eoc with tail/timeout off; done after `balance_hold_s` in band (Fab set 14400 on 11-23) or BMS delta ≤
`balance_delta` 10 mV (`mqtt.conf cell_voltages_delta_topic` = `bat_caravan/cell_voltages/delta`). Deferred on holiday,
pack >35 °C, stale BMS/temp; session cap 2× hold, state in RTC (`s_chargerRtc`, pinned 0x50001D00 since 881). `balance
now` forces; `balance_interval 30` days on both.
- 11-23 forced runs hit the 3.55 V ceiling: flat's low Vout frame + pin hold. 5e346dec: hold drives an output-current limit
  from BMS frames, pin at Vbat_max; ceiling defers interval/4 (≥20 h). 1bb428fa: stale pause up to 600 s.
- Entry overshoot (fry entered at 3.529 V with full ilim while flat booted): cc30ca19 approach band cv_eoc−30 mV cuts ilim
  to tail current, K 1.5 down/0.5 up, boot idle 0.25 A near top until termination decided. c9a4be4b: approach cut never
  raises the limit. 783c87c5: BMS re-seed detector (200 A × max(dt,12 s) + 0.5 Ah, dt ≤120 s). 44e16128: BMS ref follows
  charge up while terminated (integrator does not).
- bat_c 280 → 245 on both (11-24). Open: re-seed during a reboot is undetected (needs last raw charge+time in RTC);
  first scheduled run ~12-23 (counter at 625/639 of 14400 s).
**How to apply:** check "balance done" and cell delta before/after; flat was once left at `dc 0` after a run (needs `mppt`).

## Holiday mode
Live on fry+flat since 11-18 (bd29156b, review fixes 64b8498a). Keys `holiday_charge 0.6 / holiday_ibat 2.3 /
holiday_band 0.1 / holiday_days`, console `holiday auto|on|off`, RTC_NOINIT state.
**Why mean discharge current:** daily %SoC is weather-dependent; mean discharge 1.4–2.0 A unattended vs home p25 2.9 A.
11-22 first window: device 1.85 A over 14.3 h, matches batmon, van was unattended → 2.3 A threshold right.
Open: window progress not RTC-saved (OTAs reset it), RTC survival on a real reset untested, local-midnight alignment of
the two units declined. Each unit detects independently.
