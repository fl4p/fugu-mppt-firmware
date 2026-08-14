---
name: project-rpi-ble-connect-scan-contention
description: rpi BLE connect timeouts were its shared WiFi/BT radio; fixed by a separate USB BT adapter
metadata: 
  node_type: memory
  type: project
  originSessionId: 7d8434b0-3357-4f2b-811a-a81c44633861
  modified: 2026-08-06T06:54:36.246Z
---

`fab@rpi.local` has **no ethernet uplink** — `wlan0` is the default route and carries Home
Assistant, ZeroTier, docker and every ssh session. Its on-board Cypress controller shares silicon
and antenna with wlan0, and BLE lost the coexistence arbitration.

Measured 2026-08-06 against fbuck (same peripheral, back to back, 4 s between connects):

| central / adapter | condition | ok | median |
|---|---|---|---|
| rpi on-board | baseline | 20/30 | 3.2 s |
| rpi on-board | `wlan0` saturated | 17/20 | 6.3 s |
| macOS | control | 30/30 | 0.7 s |
| **rpi USB adapter** | baseline | **30/30** | **0.4 s** |
| **rpi USB adapter** | `wlan0` saturated | **20/20** | **0.5 s** |

**Failure signature** (`btmon`): `LE Create Connection` → `LE Connection Complete: Success`
(optimistic — reported on *sending* CONNECT_IND) → ~75 ms later `0x3e Connection Failed to be
Established`. The ESP32 never logs `ble: client connected` on those, so the CONNECT_IND is not
received. **bluetoothd then waits ~5 s before retrying** — that fixed backoff, not radio time,
is what made a connect take 5–17 s or time out.

**Two hypotheses tested and REFUTED** (don't re-derive): the WITH_BLE_ADV broadcast racing the
connect via `teleAdvTick` (`adv_ms=0` was *worse*, 8/15 vs 10/15), and `bsync` holding the ESP32
WiFi in promiscuous `WIFI_PS_NONE` (off 33/45 vs on 18/30, p≈0.23).

**How to apply:** the adapter is `10:20:BA:05:4C:8E` (USB CDC-ACM, Espressif HCI controller). Its
kernel index is NOT stable — it appeared as hci1, then became hci0 within 20 min when the on-board
radio was disabled. **Select it by MAC, not by hciN.** `BleTransport(adapter=…)` /
`fugu_console.py --adapter` / `$BLE_ADAPTER` accept either and resolve a MAC over BlueZ D-Bus;
an unmatched MAC raises rather than falling back. `$BLE_ADAPTER` is not set persistently on the
rpi — it currently works because the dongle is the only adapter. See
[[project-fugu-py-shared-console]]; the rpi's `pwr-metering/ate/fugu/` is a divergent copy
patched separately.
