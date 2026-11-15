---
name: project_arduino_esp32_ble_version_cap
description: arduino-esp32 capped <3.3.8 for the IDF 5.5 build only (IDF 6 uses >=3.3.12) — 3.3.8 BLE breaks against IDF 5.5.1 NimBLE
metadata: 
  node_type: memory
  type: project
  originSessionId: 9087f08e-09a6-4eab-acb5-bcd058f21995
---

`main/idf_component.yml` pins `espressif/arduino-esp32: '>=3.3.2,<3.3.8'` (was `^3.3.2`).

**Why:** arduino-esp32 **3.3.8** added `BLEDevice::getLocalIRK()` which unconditionally calls
`ble_gap_read_local_irk()` under `CONFIG_NIMBLE_ENABLED`. That NimBLE GAP API does not exist in
ESP-IDF 5.5.1's bundled NimBLE (only `ble_store_read_local_irk`), so a `WITH_BLE=1` build fails to
compile `BLEDevice.cpp`. 3.3.7 is the last release without the call. Only matters when BT is actually
enabled (BLE headers compile to nothing otherwise). See [[project_ble_nus_console]].

**How to apply:** keep the cap until IDF's NimBLE catches up; then bump and retest a `WITH_BLE=1`
build. If `idf.py reconfigure` resolves the new version in `dependencies.lock` but errors on a
missing `.component_hash`, `rm -rf managed_components/espressif__arduino-esp32` and reconfigure to
force a clean re-download. Note `dependencies.lock` and `sdkconfig` are untracked and drift; deleting
`sdkconfig` then `reconfigure` regenerates it for the default target `esp32` — always
`WITH_BLE=1 idf.py set-target esp32s3` afterward so `sdkconfig.ble` + the right target both apply.

Update 2026-10-09: the cap applies only when idf_version < 6.0; the IDF 6 build matches `>=3.3.12` (main/idf_component.yml).
