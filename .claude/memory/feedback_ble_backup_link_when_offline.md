---
name: feedback_ble_backup_link_when_offline
description: When fry/flat go silent (no log/MQTT/telemetry), check them over BLE right away; weak link -> ESPHome proxy
metadata:
  type: feedback
---

When a converter goes silent on log, MQTT console and telemetry, connect over BLE within ~2 minutes instead of waiting (direct `fugu_console.py --ble flat`, one call with all commands); if the link is weak, go through the ESPHome nimble proxy (`--ble-proxy`, host in etc/nat.env::BLE_PROXY, tunnel via havan if no route). Procedure is in the project CLAUDE.md, section "A converter went silent".

**Why:** Fab, 2026-11-14: I watched flat stay offline for ~10 min and only checked BLE after he asked "flat advertises on ble?". flat was alive but out of heap (free 3.8 KB, min-ever 356 B), had no MQTT task, stale BMS data and sat at 1 W; a `restart` over BLE fixed it. He then said: "make it so you wont miss the ble backup link" and "if ble link is weak, try to connect through the nimble proxy".

**How to apply:** any time fry/flat stop answering. On 11-14 the proxy at 192.168.1.231:6053 was not reachable from the Mac (no route) nor via havan (EOF); direct BLE from the Mac worked at -87 dBm. Related: [[reference_esphome_ble_proxy]], [[project_wifi_reconnect_power_gated_daytime_stuck]].
