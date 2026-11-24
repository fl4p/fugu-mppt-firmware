---
name: project_charger_bms_feed
description: Shared BMS feed for fry/flat (bat_caravan MQTT topics/rates, lock-free BMS→charger, stale/flap handling, shared deficit) and flat's switching-dependent Vout sense error
metadata:
  type: project
---

fry, flat (and bench fugu-esp32s3-9804F49E139C) are parallel chargers on one 8S LFP bank, all on BMS `bat_caravan` via
MQTT. Termination logic: [[project_charger_termination]].

## Topics and rates
Observed 2026-06-28. `bat_caravan/soc/current` (ibat) ~every 4 s; `bat_caravan/cell_voltages/max` ~every 12 s;
`bat_caravan/soc/total_voltage` (0.1 V resolution, `mqtt.conf bat_voltage_topic`); `bat_caravan/mosfet_status/capacity_ah`
(`bat_charge_topic`); `bat_caravan/cell_voltages/delta` (balance). CoulombCounter `maxDt` 30 s.
**How to apply:** an ibat-driven loop must gate on its own ibat timestamp, not `vcell_high_t` (3× rate difference).
Influx: BMS pack voltage = measurement `V`, `entity_id='bat_caravan_voltage'` (current `A`/`bat_caravan_current`) in ha_van;
full batmon data in open_pe @ influx.fabi.me (`batmon()` helper, see CLAUDE.md "Device Data").

## Same-core lock-free flow
2026-05, current design. Producer (esp-mqtt task, `CONFIG_MQTT_USE_CORE_0`) and consumer (`charger.update()` from loopLF)
are both core 0 → only preemption, no SMP. portMUX removed (was on a startup LoadProhibited decode). ibat EWMA span 8
producer-side → one `std::atomic<float>` (NAN until 8 samples); CoulombCounter single-writer, `markFull()` sets an atomic
reset flag honoured on the next producer update (`ahSinceFull()` = 0 meanwhile).
**How to apply:** any BMS/MQTT→loop sharing = single-writer atomic snapshot, no portMUX.

## Shared deficit
Deployed 10-31 (dde86a7 + aebc8e4, fleet 395), VALIDATED 11-02/03: joint DoD release, joint re-latch, joint vfloor release
within 0.1–1 s. Before, each unit's own `ahSinceFull` drifted → flat released by DoD while fry sat TERMINATED at the 26.20 V
floor. Deficit refs still differed 11-24 (fry ~13.5 / flat ~17.8 Ah): an 11-23 BMS re-seed on pre-detector images; see
[[project_charger_termination]] Top-balance.

## flat Vout sense error
Measured 2026-10-28 (7 d of 1-min Influx vs BMS): idle both read +0.02/+0.04 V (gain fine); while flat converts it reads
−0.33…−0.46 V (step at 0.1 A, not ∝ I, CCM and DCM), −0.17 when fry also converts; fry reads correctly. Live 11-23 also
0.4–0.9 V low converting alone. Root cause (Fab, 11-10): flat lacks the RC filter cap on the Vout sense divider (fry has it;
~/dev/ee/hw/Fugu2/HW Stories/flat.md). Software ruled out: `ina22x_conv_time_us` 1100/2116 (11-12) and 1100 (11-24, now in
sensor.conf) moved the sawtooth frequency (0.26–0.67 → 1.30 Hz, so a sampling artefact) but not amplitude or offset;
reverted to 332. flat Vout sawtooth 0.4–1.5 V p-p vs fry ≤0.15.
**How to apply:** deployed `vout_rl=47e3` is nominal and right at idle; don't trim it or add droop. Consequence: flat
always wins Vout authority; Ro/Rc wire fits are blocked on flat ([[project_backflow_wire_guard]]). Details plans/vout-droop.md.

## Stale/flapping BMS
- Flap block (07-06), FIXED 09-07 in main.cpp: `bmsBootDeadline` was reset on every fresh→stale edge, so a flapping BMS
  blocked the startup sweep forever (flat 0 W 2+ h; latched termCond had no runtime un-latch). Now armed once. Not validated on hardware.
  Workaround on old fw: `restart`.
- Stale-cell ramp (11-12, dc8bb77, live): BMS cell data >180 s stale glided the cap to Vbat_fallback 26.8 V under a pack at
  27.3 V taking 1 kW (fry 0 W). Now, if last cell < cv_float+0.05 and not terminated: cap = min(prev pin, min(vout_avg,
  vout)+0.1), latched once per gap (keyed on vcell_high_ticks), ramping to fallback over 600 s. First version ratcheted.
  Gap cause: batmon on havan losing the ant24 BLE link (RSSI −96, 300 s watchdog exit); real fix = nimble proxy at .231.
- Boot: provisional 26.80 V cap until BMS cell data; cap lift starts a sweep at once ([[project_tracker_sweep]]).
- 11-23 ~3 min broker-side gap latched termination on both → 1bb428fa stale pause up to 600 s.
