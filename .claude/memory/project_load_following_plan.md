---
name: Load-following plan for terminated chargers
description: Two-phase plan at plans/load-following.md to make terminated chargers supply house load from solar without charging the battery; Phase 1 (voutAuthority fix) and Phase 2 (OCV-based current tracking) not yet implemented
created: 2026-07-07T08:42:11.378Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_0c4446f77ffeyIL75KQ2dF5GYP
---

**STATUS 2026-09-07: superseded in part. `partial_charge` (charger.conf) implements load-following as the partial-charge hold: an ibat-frame-gated integrator on vpack_pin (±20 mV/frame, 4 mV/A, 0.5 A deadband) instead of the plan's proportional OCV law (which leaves a steady trickle when OCV ≠ Vbat_fallback). Phase 1 (terminated + discharging on a shared bus) is still open. See doc/Termination.md.**

Two-phase plan to fix terminated chargers wasting solar while battery discharges. Plan file: `plans/load-following.md`.

**Phase 1 — Fix voutAuthority asymmetry (~10 lines):** When terminated + battery discharging (ibat < -0.1A), pin no-authority converter to `Vbat_fallback` (26.8V) instead of floor (26.2V). Allow sweeps from duty=0 by making `batteryFull` false when ibat < -0.1A in mppt.cpp:70 and main.cpp:934. Gets both converters producing power symmetrically. Known limitation: still trickle-charges at float voltage.

**Phase 2 — OCV-based load-following:** Replace fixed `Vbat_fallback` target with `vpack_pin = Vbat_fallback - ibat × R_pack` (R_pack from termination line config). Converter supplies load (ibat→0) without charging. Add `ibat_t` to BatteryState for ibat-frame gating. Unify authority/non-authority paths. No additional EWMA (producer already smooths ibat). No new config keys.

**Already deployed:** `recharge_vfloor_band` made configurable (default 0.05V, was hardcoded 0.1V) and OTA'd to fry/flat 2026-06-28 to fix stuck termination where LFP's flat curve meant 3.27V release threshold was too deep. Doc (Termination.md:72) already said 0.05V; code had drifted to 0.1V.

**Why:** On 2026-06-28 both chargers stuck at 0W after termination latched — vcell 3.322V above 3.27V floor, DoD counter < 56Ah. On 2026-06-30 both terminated again; flat produced ~400W (float-limited), fry produced 0W (floor-pinned) — the voutAuthority race. Neither supplied the full load.

**How to apply:** Implement Phase 1 first (low risk, ~10 lines), OTA, verify both converters produce power when terminated+discharging. Then Phase 2 for OCV tracking. See [[project_voutauthority_asymmetry_shared_bus]] for the live observation that drove this.
