---
name: project_charger_shared_bms_and_flat_vout_cal
description: "fry/flat share one BMS (bat_caravan); flat's Vout reads 0.3-0.45 V LOW only while converting (idle is fine) — root cause of fry-at-0W asymmetry (measured 2026-10-28)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 6fb9ccbe-34cd-4888-86ca-bce04f58f2b5
  modified: 2026-10-28T08:28:55.395Z
---

fry, flat (and fugu-esp32s3-9804F49E139C) are parallel chargers on one 8S LFP bank, all on BMS `bat_caravan` via MQTT.

**2026-10-28 (7 days of 1-min Influx, vs `V` entity_id=bat_caravan_voltage):** idle both read BMS +0.02/+0.04 V (static gain OK). While flat converts, flat reads **−0.33…−0.46 V** vs BMS (steps in at 0.1 A, not proportional to I, CCM and DCM alike; −0.17 when fry also converts); fry reads correctly while converting. fry sat at 0 A for 94–99% of flat's charging minutes on 5/7 days. So the shared-bus asymmetry is a **switching-dependent Vout sense error on flat** (INA226 VBUS divider path), not gain calibration — droop or vout_rl trim can't fix it. The old "flat reads ~0.35 V low at float" (2026-05) was probably this same artifact.

**How to apply:** see plans/vout-droop.md Step 0 results. BMS pack voltage is `"V"` measurement, `entity_id='bat_caravan_voltage'` (current: `"A"`, `bat_caravan_current`) in ha_van — batmon measurement only has dummies. `config/dl/` copies are stale (2025-12, old role keys); deployed sensor.conf unverified. Related: [[project_voutauthority_asymmetry_shared_bus]].

**2026-10-30 console (via havan):** deployed flat `vout_rl=47e3` (nominal; May trim NOT deployed) — correct at idle, don't trim. Both run g4fc1ddab (built 2026-06-18, 55 d uptime). Second cause found: each unit's own `ahSinceFull` drifts → flat released termination by DoD while fry stayed TERMINATED at the 26.20 V yield floor (0 W). Sense error makes flat always win authority. Details: plans/vout-droop.md "Console check".

**2026-10-31 deployed** dde86a7 (BMS-shared deficit via `mqtt.conf bat_charge_topic=bat_caravan/mosfet_status/capacity_ah`) + aebc8e4 (terminated sibling joins at float) to both by manual `ota` via havan, image fry-brk1-395-gaebc8e42, **LEDC gate build, IDF 6.0.3**; side-key migration applied first. Both rollback=VALID, confirmed valid at 22 s, both charging (~360 W + ~540 W). Not yet observed: a joint termination/release or a join event. Flat's Vout sense error still needs a scope check.

**2026-11-01 deployed** 3c7717f (INA226 missed-alert poll paced by time, EWMA_nPass NaN seed) to fry then flat via MQTT console (`--mqtt 100.64.0.10 --mqtt-port 1882 --name <host>`; telnet NAT ports stopped answering after reboot). fry: 0 ADC resets in 17 min MPPT, `peek` ADC_INA226::hasData()::numRecovered = numTimeouts = 48 (old build: each would have been a full ADC reset). Build LEDC with `idf.py -DSDKCONFIG=build/sdkconfig` — without -DSDKCONFIG a seeded build/sdkconfig is ignored and you get MCPWM (11% vs 14% free is the tell).

**2026-11-02/03 VALIDATED live:** joint DoD release 11-02 17:17:11 (both `ahSinceFull(42.02)>42.00`, 0.1 s apart), joint re-latch 11-03 13:50:12, joint vcell-floor release 11-03 20:07:46/47. Fix 1 works. Several stuck-watchdog CV-floor releases on 11-03 midday (Vout asymmetry, still open).
**2026-11-04 deployed** 84a48eb (+7dc0b11, #47 Ro/Rc/Rb telemetry) to both via MQTT console, `bat_voltage_topic=bat_caravan/soc/total_voltage` set on both (topic has 0.1 V resolution; broker 100.64.0.10:1882 needs etc/mqtt.env creds). `dc 0` at ~500 W trips "High-current through open backflow switch" (harmless shutdown).
