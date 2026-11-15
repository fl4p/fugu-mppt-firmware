---
name: project_fugu_ota_stop_conversion_high_power
description: OTAing fry/flat at full power reset them mid-download; FIXED in fw 116 (cmdOta self-quiesces)
metadata: 
  node_type: memory
  type: project
  originSessionId: b6a70e89-844c-4796-94b3-a108b61665f5
---

**FIXED in firmware 116 (gbd0569b9, deployed to fry+flat 2026-06-11):** `cmdOta` now folds in the
manual procedure automatically — latches `manualPwm`, ramps the converter to 0 and waits for it to
disable + de-energize, THEN halts the ADC and downloads (cli.cpp). So **`dc 0` is no longer needed
before `ota`** on fw ≥116. The note below is the pre-116 history / why. (Separately: the NAT-router
download path 192.168.1.231 can still drop a transfer — `tcp_read error: connection abort` — just
retry; fry weak-signal rssi -83 makes it flakier. On OTA *failure* in fw ≥116 the device resumes
MPPT itself; on pre-116 a failed `ota` leaves it stuck in your `dc 0` manual mode — send `mppt`.)

2026-06-11: OTAing fry at **full midday power (~280-300W)** reliably **reset it mid-download**
(reset at 87%, retry reset at 47%) → incomplete OTA slot discarded → stays on old image.
The download also CRAWLED (vs fast when idle).

**Why:** the OTA's flash erase/write disables the CPU cache and briefly halts the other core, so
each write stalls the RT loop; at full power the loop-latency / no-sample watchdog has no margin and
trips → reboot. Reset reason is **serial-only** (early-boot, pre-MQTT) so it's invisible in the
havan MQTT log — you just see uptime reset + old version.

**Fix (works, validated on both):** put the converter in manual mode with **`dc 0`** before the OTA.
This (a) drops output to ~0W so the RT loop is light, and (b) **disables the loop-latency watchdog**
— it's gated on `!manualPwm` in `main.cpp::lfWatchdog`. Download then runs at full speed and
completes. Manual mode is **RAM-only**, so the post-OTA reboot auto-resumes MPPT — no `mppt` needed
(only needed if the OTA *fails* without rebooting).

**Recipe (one device at a time):** `dc 0` → confirm `st= MANU,0` and ~0.0W (Vin rises to Voc) →
trigger `ota <url>` (persistent HTTP server from repo root + MQTT trigger,
[[project_flat_console_via_havan_mqtt_broker]]) → wait for Download 100% + reboot → verify
`App: ...115-g1c3613b8` banner + it's converting again. fry/flat lose ~5min of production each.
Alternative when not urgent: OTA at **dawn/dusk** (low power, light RT loop) — no manual stop needed.
Relates to [[project_ina226_timeout_loop_latency_shutdown]] (same RT-starvation watchdog).

**Superseded (11-12):** the app `ota` command now stops conversion itself and restores the mode on failure; see [[project_ota_via_mqtt_console]]. Manual `dc 0` is unnecessary and skips that restore.
