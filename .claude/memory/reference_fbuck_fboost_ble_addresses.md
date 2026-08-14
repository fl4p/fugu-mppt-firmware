---
name: fbuck/fboost device identities
description: fbuck/fboost BLE UUIDs, chip IDs, and how to connect to these bench devices
created: 2026-08-10T11:45:26.713Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: reference
  originSessionId: ses_012f7f1b0ffez2EOIVUBocIyKt
---

Bench device identifiers:

**fbuck**:
- BLE UUID (macOS CoreBluetooth): `C7513EAF-691F-DA42-411D-755D97AFD6BB` (advertised as `fugu-fbuck`)
- Chip ID / auto device ID: `fugu-esp32s3-344082188534` (from efuse MAC, see tele_core.cpp:39 getChipId)
- Hostname `fbuck` is stored in NVS, overrides the chip ID as the default device ID
- Config: `config/lab/fbuck_lab_bench/`

**fboost**:
- BLE UUID: `C839CEE8-3D60-F1B7-9278-2FF4F4870803` (advertised as `fugu-fboost`)

Connect with: `.venv/bin/python3 etc/fugu_console.py --ble --address <UUID> -c "status"`

These are bench devices (not live converters), safe to flash without asking. See [[feedback_confirm_before_flashing]].
