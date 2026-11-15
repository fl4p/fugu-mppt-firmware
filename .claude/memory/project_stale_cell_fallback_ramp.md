---
name: project_stale_cell_fallback_ramp
description: BMS cell-data gaps (~4-5/h) dropped the cap to 26.8 V under a charging pack; dc8bb77 caps at bus+0.1 V ramping to fallback over 10 min (OTA'd 11-12 13:41 as part of 08cf227)
metadata:
  type: project
---

2026-11-12 11:36 local: fry and flat lost the BMS cell voltage (>180 s stale) at the same moment. The cap glided to Vbat_fallback 26.8 V while the pack was at 27.3 V and taking ~1 kW, so fry went to 0 W for ~2.5 min (shared-bus asymmetry). batmon (influx.fabi.me/open_pe, device bat_caravan) shows the highest cell at 3.375 V and SoC 27%, with data present during the gap, so the MQTT cell topic is the suspect, not the BMS. Unverified.

**Fix dc8bb77 (integ-ota-1110; OTA'd to fry+flat 11-12 13:41-13:42 in 08cf227, rollback VALID):**
- Only if the last highest cell was < cv_float + 0.05 V and the charger is not terminated.
- The cap is min(prev pin, min(vout_avg, vout) + 0.1), latched once per gap (keyed on vcell_high_ticks, so stuck-watchdog releases can't ratchet it), ramping linearly to Vbat_fallback across the 600 s after expiry.
- Reviewed twice by ee-code-verifier. The first version ratcheted and was gated on cv_eoc, which is unsafe.

**How to apply:** check that "then to the fallback" log lines show no 0 W dips.

Root cause of the gaps: batmon on havan loses the ant24 BLE link on hci1 (RSSI -96), can't reconnect, and its 300 s publish watchdog exits. batmon-ha 31403a5 (scan adapter fix) was deployed by the batmon session; the real fix is the ESP32 nimble proxy at .231 (it panicked under batmon GATT on 11-12).
