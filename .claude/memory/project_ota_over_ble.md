---
name: OTA-over-BLE push (no WiFi)
description: OTA firmware push over BLE — protocol, partition budget, operational gotchas (parallel, stale-state, version-skip, GATT cache)
created: 2026-08-10T11:44:49.192Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_01483adf2ffecTSkJ33gwB78Ft
---

OTA-over-BLE push path (no WiFi), added 2026-05-20. Host pushes the image; device flashes via native
esp_ota (not esp_https_ota). Files: `src/etc/ota_ble.cpp/.h`, `etc/ota_ble.py` (bleak), wired into
`src/tele/console_ble.cpp` (FW char + tick) and `src/cli.cpp` (`ota-ble` command; the ESP_LOG tag is
"otab" but the CLI verb registered at cli.cpp:1550 is `ota-ble`, NOT `otab`).

Protocol contract (firmware ↔ python, keep in sync):
- New NUS char `6E400004-…` = FW data, WRITE_NR. Control via console `ota-ble begin <size> <sha256hex>` /
  `end` / `abort`. Status as ESP_LOGI lines mirrored to the client: `OTAB READY/CRED <G>/PROG <w>/<sz>/
  OK/FAIL`. `CRED <G>` = cumulative byte offset the host may stream up to (credit window = RING_CAP 32KB).
- Data plane: FW onWrite (host task) only copies into a PSRAM ring; `otaBleTick` (net loop, in
  `bleConsoleLoop`) drains to flash in 4KB slices + streaming SHA-256, then verifies vs host digest.
- Concurrency: ALL OTA state mutation runs on the net loop. Disconnect calls `otaBleRequestAbort()`
  (flag), consumed by tick — never free OTA state on the host task. Ring + active flag guarded by
  `ringMutex`; esp_ota_write done outside the lock.

Budget: WITH_BLE image is now **~8% free** in the 0x1c9000 (1871872 B) OTA slot. Size-check every WITH_BLE build. See [[project_ble_nus_console]], [[project_arduino_esp32_ble_version_cap]].

Verified end-to-end on hardware 2026-05-20. Operational gotchas:

- **Version-match skip reports success** (2026-08-10): `ota_ble.py` probes the device version via
  `uptime` before pushing. If device version == image version, it prints `☑️ skip: already at
  {ver} (use --force to push anyway)` and returns True → `✅ success` — WITHOUT pushing anything.
  After a rebuild with the same version string (e.g. dirty-tree `gXXXX-dirty`), the push is silently
  skipped. Always pass `-f`/`--force` after a rebuild, or watch for the `☑️ skip` line. Do NOT grep
  the output in a way that filters out `skip` — the agent did this and reported a successful OTA that
  was actually a no-op.
- **Never run two BLE OTAs in parallel from one host** (2026-08-10): launching fbuck + fboost
  simultaneously via `bash_background` stalled both at ~11% — the host has a single Bluetooth radio;
  two concurrent GATT connections split throughput to a crawl and both time out. Always OTA one device
  at a time, sequentially.
- **Failed/timed-out BLE OTA leaves the receiver active** (2026-08-10): a push that dies mid-transfer
  does NOT auto-abort on the device side. Retrying gets `OTAB FAIL already-active` / `timeout waiting
  for READY`. Recovery: connect via console (`fugu_console.py --ble --name <dev> -c "ota-ble abort"`)
  then retry the push. The CLI verb is `ota-ble` (NOT `otab` — that's only the log tag).
- **No PSRAM on the bench board** (Total PSRAM 0, ~62KB free internal heap, fragmented) → 32KB ring
  alloc failed (`OTAB FAIL no-mem`). RING_CAP is now 8KB; BLE throughput never bottlenecks on it.
- **macOS CoreBluetooth caches GATT** per (stable) ESP32 public address, so after adding the FW char
  the Mac kept serving the old RX/TX-only service (`BleCharacteristicNotFoundError 6e400004`). Bust
  it with `blueutil -p 0 && blueutil -p 1`. Any future GATT change on this device needs the same.
- Completion: device reboots right after queuing `OTAB OK`, so that notify usually never drains; the
  host (etc/ota_ble.py) waits for PROG==size, sends `end`, then treats the disconnect + a successful
  re-advertise as success.
- **bash timeout**: a 1.7MB BLE OTA at ~32KB/s takes ~57s clean, but BLE drops can stretch it to
  2-3 min. The `bash` tool default 120s timeout (and even 300s) can kill it mid-transfer. Use
  `bash_background` (no timeout) for BLE OTA pushes.
- **BLE scan misses device right after reboot** (2026-08-10): OTA succeeds and device reboots, but
  if you immediately retry another OTA (or the previous one failed), `BleakScanner` may not find the
  device for 10-20s while it re-initializes BLE. Wait ~15s after a reboot before scanning.
