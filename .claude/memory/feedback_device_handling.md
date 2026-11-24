---
name: feedback_device_handling
description: Device handling rules — flash policy, locks before writes, read-only during diagnosis, ADC-error health check, BLE backup link, fugu_console not idf monitor, provision for config
metadata:
  type: feedback
---

## Flash policy
Bench (fbuck, fboost, mocks): flash without asking (2026-08-04). fry/flat OTAs: standing approval since 2026-11-18 ("yes all updates approved").
**Why:** per-update confirms slowed multi-session fleet pushes; rollback + boot watchdog cover a bad image.
**How to apply:** still `-n` first, scope with `--name`/`-m`, main-built image with VCONV off ([[project_vconv_builds]]), lock skill, then `bootinfo` VALID + log check; report to Fab. Behaviour-changing config (idle_cpu_mhz, charger limits) stays Fab's call. Bench: pass the port explicitly.

## Verify chip + firmware before flashing
2026-05-23: "device connected" was a fugu unit, not the expected NAT router.
**Why:** identical-looking ESP32-S3 units; wrong image wipes a live charger, wrong arch bricks.
**How to apply:** unnamed target → `esptool chip_id` + ~5 s of serial banner (`Project name:`); mismatch → stop and ask.

## Read-only during diagnosis
2026-08-10: `bf`/`dc` (no arg) sent to fbuck mid-test disabled backflow and shut PWM; Fab: "dont touch the device".
**Why:** "flash freely" does not mean "change converter state freely".
**How to apply:** when diagnosing, only status/sensor/get-config/rt-stats; ask before bf, dc, sweep, mppt, ±N, fan, sync, restart, ota — on every device.

## Check locks before writes
2026-11-22: piped `lock.sh acquire` through `tail`, missed BUSY, sent `crash abort` into another session's bench139c test. 2026-11-23: `del-config` on flat while another session held lock `flat`.
**Why:** other sessions park devices under locks; pipeline exit status hides BUSY.
**How to apply:** `~/bin/lock.sh list` before any state/config write to fry/flat (read-only fine); `lock.sh acquire X || { echo BUSY; exit 1; }` as its own step, re-acquire after any gap; if held, route through the holder.

## Console tests before flashing
2026-08-12.
**Why:** flashing is slow and risks the macOS USB-JTAG port dropping.
**How to apply:** first test the hypothesis on running firmware via console (dc, bf, sync, sensor, psu); flash only if code is needed.

## Config-only change: provision
2026-08-12.
**Why:** full flash rewrites bootloader/app/otadata needlessly.
**How to apply:** only `config/` changed → `./provision.sh <config>` (littlefs only).

## Never `idf.py monitor`
2026-05-29.
**Why:** Fab's instruction; it also needs a TTY and fails from the agent.
**How to apply:** all console I/O via `etc/fugu_console.py` (`-c`, `--stdin`), also for raw boot logs.

## "ADC error" = not healthy
**Why:** the no-sample watchdog self-heals, so climbing `num=`/sps looks fine while the ADC stalls >1 s and backs off 5 s repeatedly; I once called fry healthy that way.
**How to apply:** after any converter flash, grep ~30 s of log (fugu_console or havan `pv/fugu_console.log`) for `ADC error`, `no ADC samples`, `backoff`, `stopAndBackoff`, `Loop latency`; healthy = leaves `st=START`, stable sps, tracking, zero ADC errors. See [[project_adc_sampler]], [[project_ina226]].

## fry baseline lag
2026-11-16.
**Why:** fry's ~2.1 ms lag (2080-2190 µs) predates the 11-15 OTAs; says nothing about a new build.
**How to apply:** omit it unless clearly above baseline; flat's lag (<1 ms goal) still matters.

## BLE backup link when silent
2026-11-14: flat offline ~10 min before I checked BLE; it was out of heap (3.8 KB free, min 356 B), no MQTT task, 1 W; `restart` over BLE fixed it. Procedure: CLAUDE.md "A converter went silent".
**Why:** Fab: "make it so you wont miss the ble backup link"; weak link → nimble proxy.
**How to apply:** within ~2 min of silence connect over BLE. 2026-11-16 gotchas: "silent" may be only the havan log (MQTT down, InfluxDB still flowing) — compare last log line per device with the clock; `--ble flat` once hit a bench board, so check `hostname` and use `--ble --address` (flat CB3FFDD5-B0C8-6E28-A29B-9DCD2E4598CC, fry EF1A19C2-DC8F-D7C0-FF36-979D93BA34A8, macOS UUIDs); fry is -94 dBm from the Mac and the proxy gave EOF — telnet via `etc/nat-tunnel.sh up` worked. See [[reference_device_access]], [[project_wifi_reconnect]].
