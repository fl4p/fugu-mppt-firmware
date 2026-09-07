---
name: EOC feedback death-spiral causes premature termination
description: "FIXED IN CODE 2026-09-07 (not yet OTA-validated): EOC feedback target was v_term(ibat) → positive feedback → premature termination at 3.40 V/cell on fry/flat (7-2, 7-3, 7-7). Now target = cv_eoc while charging, cv_min once terminated; line trigger debounced over 2 cell frames."
created: 2026-07-12T09:52:20.612Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_0aa42f5edffeTmhW6AQrHE5l1y
---

# EOC feedback death-spiral causes premature termination

**STATUS 2026-09-07: fixed in src/charger.h (uncommitted→committed same day): `v_eoc = terminated ? cv_min : cv_eoc`, termCond.update() gated to one call per BMS cell frame, VTERM_STREAK_REQ=2. Host-run of test/test_charger.cpp passes (`test_eoc_feedback_does_not_chase_termination_line`). NOT yet validated on fry/flat — watch for termination at cv_eoc with tail current after the next OTA.**

**Type:** project
**Description:** Charger EOC feedback loop (charger.h:330-356) creates a positive feedback: cvHigh→v_eoc triggers vpack_pin reduction→current drops→v_term collapses to cv_float→termination latches. Recurring on fry/flat with cv_float=3.35 on devices; confirmed 7-2, 7-3, 7-7. Fix direction: freeze v_term during EOC feedback or gate low-current termination.

EOC feedback death-spiral: the charger's EOC feedback loop (charger.h:330-356) and the current-dependent termination line (charger.h:148-150) form a positive feedback loop that causes premature termination at cell voltages barely above cv_float.

Mechanism: when cvHigh approaches v_eoc (= min(cv_eoc, termCond.v_term())), the EOC feedback reduces vpack_pin → output voltage drops → current drops → v_term drops (since v_term = cv_float + ibat × r, where r = (cv_eoc - cv_float)/(tail_c_rate × Cbat)) → more feedback → current crashes to ~0A → v_term floors at cv_float → cvHigh > v_term → termination latches (charger.h:164).

Code analysis (charger.h:150): `v_term = fminf(p.cv_min + fmaxf(0.f, vo), p.cv_eoc)`. The `fmaxf(0.f, vo)` ensures v_term can never fall *below* cv_float — the problem is v_term falling *toward* cv_float as current drops, not below it. The positive feedback is between the EOC loop driving current down and v_term tracking that reduced current.

Proposed fix direction (not yet implemented):
1. **Freeze v_term while EOC feedback is active** — don't let the termination threshold chase the feedback. When eocFeedback is true (charger.h:330), hold v_term at its pre-feedback value so the loop can regulate cvHigh→v_eoc without the moving goalpost triggering termination.
2. **Gate termination at low current** — when ibat is near zero and cvHigh is only marginally above cv_float, that's normal CV behavior, not end-of-charge. Don't latch termination in this regime.

Incident 2026-07-02 ~18:03: both fry and flat terminated charging at peak cell 3.4V. Devices have cv_float=3.35 (set 2026-06-15, see [[project_shared_bus_termination_fix_deployed]]). With cv_float=3.35, the termination line floors at 3.35V; any cell above 3.35V at low current triggers termination. 3.4V > 3.35V so the pack was considered "full" even though LFP isn't full until ~3.65V/cell. fry's Vout hit 26.976V (well above 26.8 target) → CV controller slammed duty → current < 0.05A threshold → termination. flat showed the death-spiral explicitly in logs: v_term dropped 3.370→3.350 as current fell, vpPin followed down 26.72→26.20.

Downstream confirmation 2026-07-03: the Jul-2 premature termination happened at ~84% SoC (vcell_max 3.40 @ +11.7A). Overnight the pack drained 84→69% (~41 Ah) but LFP plateau held vcell_max at 3.32-3.33 — 20-30 mV above the vfloor release (cv_float 3.35 − recharge_vfloor_band 0.05 = 3.30). Both chargers idled in full sun until 07:01 UTC when a −22.7A load spike sagged the plateau below 3.30 → synchronized release (shared BMS, 9 s apart). The DoD release (recharge_dod×Cbat: fry 42 Ah, flat 56 Ah) was 1 Ah short on fry — it's calibrated for termination at TRUE full, so premature termination shifts the whole recharge band down by the missing SoC. Net effect: terminate early AND restart late/randomly. bat_caravan data lives in open_pe @ influx.fabi.me:8086 (NOT ha_van@tm.fabi.me), see batmon() in /Users/fab/dev/pv/bat-impedance/datasets.py.

Recurrence 2026-07-07: **both fry AND flat terminated simultaneously** (fry 10:29:47, flat 10:29:45 — 2 s apart) because they share one BMS (bat_caravan) which reported the same cvHigh=3.377 / v_term=3.358. Both had been charging since ~05:00 (fry peak 333W, flat ~290W). **Only fry sat idle** at duty=1/Vin=72-75V (Voc) for 9 hours of peak sun (10:29–19:41). flat continued producing 400-530W all day because it retained voutAuthority (its vpPin stayed at 26.8V, and flat's −0.35V calibration offset meant 26.8V setpoint ≈ 27.1V real, above the bus — so MPPT wasn't CV-limited); see [[project_voutauthority_asymmetry_shared_bus]]. fry released at 19:40 when vcell_high dropped to 3.297V — by then sun was nearly gone (~12W recovered). This confirms the shared-BMS architecture makes the death-spiral systemic: one high-cell reading takes ALL chargers offline simultaneously, but the voutAuthority asymmetry means only one actually stops producing. Device still has cv_float=3.35 (release log confirmed `3.350 - 0.050` threshold); repo config/fmetal/conf/charger.conf says 3.37 (drifted).

**Why:** the EOC feedback loop is designed to regulate cvHigh→v_eoc, but at low current v_eoc collapses to cv_float, making the loop terminate rather than regulate. This is a design tension between longevity (low float) and charge completeness (cells reach higher voltage).
**How to apply:** when chargers terminate at cell voltages the user considers too low, check the on-device cv_float (not the repo config — they drift). The termination floor at low current = cv_float. To allow higher cell voltage before termination, raise cv_float on the device via `set-config charger.conf cv_float <val>` + `restart`. For a permanent fix, implement one of the two proposed fix directions above (freeze v_term during EOC feedback, or gate low-current termination). Related: [[project_cv_min_dead_conf_key]], [[project_shared_bus_termination_fix_deployed]].
