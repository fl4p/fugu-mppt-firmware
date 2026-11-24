---
name: BLE NUS console: TX pacing, GATT cache, glyph split
description: BLE NUS console: TX backpressure pattern, stale macOS GATT cache (ATT code 3), and the client-side UTF-8 glyph split
created: 2026-11-23T09:47:39.611Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: project
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
  modified: 2026-11-24T16:06:58.851Z
---

**What it is.** `BleConsoleService` (`src/console_ble.{h,cpp}`, NimBLE NUS) gives a wireless console
without Wi-Fi, for field provisioning and diagnostics — the ESP32-S3 is BLE-only, no classic SPP.
Implemented and hardware-verified 2026-05-20: boots, advertises, clean service start/stop, RT loop
unaffected. Gated behind compile-time `CONFIG_FUGU_WITH_BLE` (layers `sdkconfig.ble`); config in its
own `ble.conf` (`enabled`/`log_level`/security keys), **not** the old `board.conf` keys; registered
via `g_services.registerService()` with `requiresNetwork=false`. BT controller + NimBLE host are
pinned to core 0 (RT is core 1). Binary is 1,803,360 B — ~4% free in the 1.87 MB OTA slot, so always
check `build/fugu-firmware.bin` against 1,871,872 B. WebBLE (`etc/config-tool/conf-editor.html`) is a
first-class client, hence the default Just Works pairing (configurable none/justworks/passkey).
arduino-esp32 selects NimBLE via `CONFIG_NIMBLE_ENABLED`. Console: `service start ble`. Logging was
generalised to a callback array (`addLogCallback`/`removeLogCallback`). While building it, a real
`sdkconfig.defaults` bug surfaced: it set both `CONFIG_PARTITION_TABLE_CUSTOM` and `..._TWO_OTA=y`,
which drops the littlefs partition on fresh sdkconfigs — `TWO_OTA` was removed.

**TX backpressure is mandatory — never loop `notify()`.** NimBLE emits about one notification per
connection interval from a small mbuf pool, so a multi-line reply (`get-config sensor.conf`, ~15
lines) exhausts it and `ble_gatts_notify_custom` returns `BLE_HS_ENOMEM` (rc=6), **silently dropping
output** and truncating the client (only `[E] ...notify(): rc=6` spam is visible). The verified
pattern: `bleWrite` only appends to a mutex-guarded `txBuf` FIFO; `bleTxDrain()` — pumped every tick
from `bleConsoleLoop` via `tickAll()` on core 0 — sends chunks with true backpressure, and a
`TxCallbacks::onStatus` hook clears `lastNotifyOk` on the `ERROR_GATT` status `notify()` reports
synchronously, so the drain stops the instant one fails and retries that chunk next tick. No
`vTaskDelay` or guessed pacing. FIFO capped at 8 KB, cleared on disconnect/stop. Three more TX fixes
landed in the same pass (0 rc=6 over a full connect + command cycle): (1) a `TX_SETTLE_MS=500` settle
window before draining, because right after connect the central is still on its slow default interval
and draining the queued log backlog overflows the controller's ACL buffers; (2) `onConnect` requests
`requestConnParams(connId, 6, 12, 0, 400)` (7.5-15 ms instead of the central's ~30 ms+ default) plus
`BLEDevice::setMTU(247)` and draining at the *negotiated* MTU (`getPeerMTU()-3`, fallback 20), which
took a ~1.1 KB reply from ~58 notifications to ~5 (first byte ~25 ms, full dump ~150-200 ms); (3) the
`os_msys_num_free() < 4` pre-check is only a secondary guard — the connect burst bottleneck is the
controller ACL pool, not host msys. Separately, the REPL felt slow because `handleCommand` emitted no
completion marker, so clients waited out their full 4 s timeout: `loopConsole` (UART/telnet/BLE) now
writes `OK: <cmd>`/`ERR: <cmd>` after `handleCommand` returns, dropping commands from ~4080 ms to
~90-260 ms. MQTT calls `handleCommand` directly and gets no marker.

**Stale macOS GATT cache, not a firmware bug.** `--ble` to a bonded device failing at subscribe with
`CBATTErrorDomain Code=3 "Writing is not permitted"` means macOS wrote the *cached* CCCD handle: it
caches the attribute table per bonded peer, so after an OTA changes the GATT layout (the OTA-over-BLE
commit added a 4th characteristic, shifting handles) the old handle no longer maps to the writable TX
CCCD. Code 3 is distinct from 5/15 (auth/enc). The firmware NUS CCCD path is correct (NimBLE
auto-creates the CCCD for `PROPERTY_NOTIFY`; manual BLE2902 is Bluedroid-only), and a fresh unbonded
connect subscribes fine. Fix: unpair the device (`blueutil --unpair <mac>`), then reconnect — it
re-bonds and rebuilds GATT. Renaming does not help; the cache is keyed by bond/identity. Durable fix
still TODO: a GATT Service-Changed indication so bonded centrals self-invalidate after a BLE OTA.
Side finding: **fry is a weak-signal BLE peer from the Mac** (never seen in a 35 s cold scan, flat
~-92 dBm) — the bond is load-bearing for reconnecting to it.

**"Garbage bytes" are a client-side decode bug.** The `℃` in a status line turning into `` (exactly
3 replacement chars) after a high-volume command means the client decoded **each notification packet**
with a throwaway `new TextDecoder().decode(value)`: the 3-byte `e2 84 83` straddled a notification
boundary, so `e2` flushed alone and `84 83` became two lone continuations. Fixed with a persistent
streaming decoder per transport (`{stream:true}`, reset wherever the byte buffer resets), mirroring
the already-correct serial loop. Ruled out and byte-correct, do not re-investigate: the firmware's
formatting (both `℃` are identical `e2 84 83`), `bleTxDrain`'s chunking (it does split glyphs
byte-wise, which is harmless to clients that reassemble), and `etc/fugu/*` transports (they buffer raw
bytes and decode whole lines). An optional firmware defence — trimming chunk ends to a UTF-8 boundary
— was not applied. Latent bug found nearby (not this symptom): `logging.cpp` calls
`printf_old(entry.str)`, using an RT-enqueued line as a format string, so any `%` misformats; it
should be `printf_old("%s", entry.str)`.

**Bench/test caveats.** macOS keeps a stale bond after a reflash (CBError 14); `blueutil --unpair`
does not work for LE on recent macOS (classic only) — forget the device in System Settings, or set the
`ble.conf` security key to none and reboot to bypass bonding (2026-11-24: on fry it reported success
but the pairing stayed listed). Proxy path: the link is bonded+encrypted; the old "no replies" was the
client never writing the TX CCCD (aioesphomeapi start_notify doesn't, the proxy leaves it to the client);
fixed in fugu-py dc3b5ec. A peer without a CCCD subscription holds replies silently (NO_SUBSCRIBER treated
as backpressure in console_ble.cpp). Direct
bleak to fry is mbuf-fragile — a couple of reconnects can trip a GATT bring-up reset.
