*this document is an LLM generated placeholder*

# OTA radio release and buffer cuts (bench139c, 2026-11-23)

Measured on bench139c only. Nothing was sent to fry or flat.

## Changes

- **A, buffer cuts** (`sdkconfig.ble`, `sdkconfig.defaults.esp32s3`):
  - Wi-Fi `RX_BA_WIN` 6 → 4. This is the only Wi-Fi change with a runtime effect.
    `STATIC_RX_BUFFER_NUM` 8 → 4 in Kconfig changes nothing at runtime: arduino-esp32
    (`WiFiGeneric.cpp`, `useStaticBuffers()` false by default, nothing in `src/` calls it) passes
    `static_rx_buf_num=4` and `dynamic_rx_buf_num=32` to `esp_wifi_init()`, and the boot log confirms
    `Init static rx buffer num: 4` and `rx ba win: 6` on the pre-A image (815-g8c0f7e02). The Kconfig line only mirrors that 4, which keeps
    the BA window (4) within the static buffers. So main already ran a BA window of 6 over 4 buffers.
  - NimBLE `MSYS_2_BLOCK_COUNT` 24 → 12, `BT_CTRL_BLE_MAX_ACT` 6 → 3. The controller-activity peak is
    2: `MAX_CONNECTIONS=1`, no extended advertising, and one advert at a time (connectable while idle;
    the non-connectable `WITH_BLE_ADV` advert exists only during a connection). The third is a spare.
  - NimBLE `TRANSPORT_ACL_FROM_LL_COUNT` 24 → **34**. This is an increase, see the next section.
- **B, radio release** (`src/etc/ota.cpp`, `src/tele/console_ble.cpp`, `src/main.cpp`):
  - `ota <url>`: `doOta()` calls `bleConsoleRelease()` before the download. That function drains
    the TX FIFO, stops the telemetry stream and advertising, disconnects the client and waits up to
    5 s for the disconnect, then calls `BLEDevice::deinit(true)` (NimBLE stop/deinit, controller
    disable/deinit, controller memory released). The waiting matters because the Arduino `deinit()`
    deletes the server before the host stops, so a late disconnect event would hit freed memory. If
    the client does not go away within 5 s, BLE is kept and the OTA runs as before.
  - Once BLE is released, a failed download reboots (`systemRestart()`). The existing 5 min OTA
    watchdog already did so.
  - `bleConsoleBegin()` refuses after a release, and `bleConsoleEnd()` (`svc off ble`) now aborts an
    active BLE push. Before this, the push was left armed with the converter halted, because no
    `otaBleTick()` ran any more.
  - `ota-ble`: the esp-ota-ble `quiesce(true)` hook (push armed) calls `wifiRelease()`. That is the
    `wifi off` teardown (stop network services, `MDNS.end()`, `WiFi.disconnect(true)`), factored out
    of `networkLoopTick()`. It runs on the network loop, not inside a telnet input callback. Every
    failure path of esp-ota-ble ends in `quiesce(false)`, and with Wi-Fi released that sets a flag.
    The network loop then drains BLE TX and calls `systemRestart()`.
  - The bootloader path (`ota bootloader <url>`) is unchanged: it calls `httpOta()` directly, not
    `doOta()`.
  - Log lines: `ota: BLE released, free heap X -> Y` and `ota: WiFi released, free heap X -> Y`.
- **Review fixes to B** (second round, see "Refusals" below):
  - `ota <url>` refuses before the conversion stop while the running image is PENDING_VERIFY
    (`ota: ESP_ERR_OTA_ROLLBACK_INVALID_STATE, ...`) or Wi-Fi is not connected. `ota-ble begin` /
    `resume` refuse before the quiesce hook while PENDING_VERIFY
    (`OTAB FAIL esp_ota_begin ESP_ERR_OTA_ROLLBACK_INVALID_STATE`, the line `etc/ota_ble.py` retries
    on), without a connected BLE client (`OTAB FAIL no-ble-client`) or while a restart is blocked
    (`OTAB FAIL restart-blocked`). esp-ota-ble is unchanged.
  - `wifi on|off` is refused while `otaBleActive()`.
  - With `g_restartBlocked` (bootloader copy) neither radio is released. The reboot-due flag is
    consumed once; if `systemRestart()` still refuses, Wi-Fi is switched back on instead of spinning.
  - `bleConsoleRelease()` disconnects and waits before touching any state. On the 5 s fallback it only
    restores the server callbacks; console, log mirror, RX queue and telemetry stream are untouched.
  - New commit: the notify mbuf floors (console 8, Improv 8, telemetry 10 instead of 12) count only
    `msys_1`, the 12 × 256 B pool that notify/ATT/L2CAP allocate from. `os_msys_num_free()` sums both
    pools; there is no per-pool API, so `bleMsys1Free()` reads the first entry of NimBLE's size-sorted
    `g_msys_pool_list`.

### Incoming ACL pool: why 34, not 12

esp-ota-ble's credit window on a board without PSRAM is the 8 KB staging ring (`RING_CAP_MIN`). The
host can therefore have 8192 B in flight, which at 244 B per write is 34 single-PDU ACL packets.
That assumes DLE and 244 B writes that arrive as one PDU each; 182 B writes would need 45 buffers.
When the pool is empty, the controller's VHCI callback (`ble_hci_rx_acl`, IDF
`esp_nimble_hci.c`) retries with `vTaskDelay(1)` and drops the packet after 30 tries (30 ms). It
prints `ACL buf alloc failed N times` every 5 tries. A dropped write fails the SHA check, and the
OTA fails.

A first build with 12 buffers ran out of them, measured on the serial console:

| build | Wi-Fi during BLE OTA | pool-empty events (≥ 5 ms) | longest wait |
|---|---|---:|---:|
| A, ACL 12 | on | 0 and 2 (2 runs) | 5-9 ms |
| A+B, ACL 12 | released | 37 (1 run) | 15-19 ms |
| A+B, ACL 34 | released | 0 and 0 | none |

No packet was dropped and every image verified. Still, with Wi-Fi released the link is twice as
fast, and the margin to the 30 ms drop was down to half. ACL 34 costs about 6.5 KB of free heap
compared with 12: 41.9 KB vs 35.5 KB steady state, the same build otherwise.

## Setup

- Images: `sdkconfig.defaults;sdkconfig.fleet;sdkconfig.vconv_s3` (fleet options with the vconv
  plant, as in the heap report), IDF 6.0.3, esp-ota-ble from the local `../esp-ota-ble` checkout
  (7a137e2). Each image also carried a measurement patch that was not committed:
  `heap_caps_monitor_local_minimum_free_size_start()` plus a `MEAS ota start` line at the top of
  `cmdOta` and of `ota-ble begin`, and a `MEAS reboot local-min=… dma-local-min=…` line in
  `systemRestart()`. "Min during OTA" below is therefore the minimum free heap from the `ota` /
  `ota-ble begin` command to the reboot. It includes the conversion stop and the ring allocation,
  which come before the radio is released.
  - (1) main 2a115011, image 1 726 336 B
  - (2) A 3a96cc87, image 1 726 320 B
  - (3) A+B d534e791, image 1 730 768 B
- Services as on fry: mqtt, tele (binary wire), ftp, telnet, scope, ble on; lcd off. The only conf
  change needed was `tele.conf enabled=1 binary=1`, restored afterwards.
- Steady state: median of 10 `heap` readings over USB serial (on havan), about 100 s after boot, with
  no BLE or telnet client. The min-ever in that table includes the boot transients.
- Wi-Fi OTA: `etc/ota_mqtt.py` via the havan relay (`OTA_RELAY_PORT=9017`, since another session was
  updating flat), `--no-gw-boost`. **The route is not the fry/flat route**: bench139c sits on the
  192.168.1.x LAN (`ip` 192.168.1.125), not behind the NAT router, so the gateway was left alone.
  Downloads ran at 70-80 KB/s here, against 11-17 KB/s on fry/flat. Payload was always the same
  1 730 064 B image.
- BLE OTA: `etc/ota_ble.py --xform raw` direct from this Mac (native macOS sender, 244 B writes,
  2M PHY). Payload was always the same 1 708 496 B image (815-g8c0f7e02-dirty). Each push was a real
  write: before every BLE run the passive slot held a different build, and every run logged
  `OTAB SKIP kept=9 wrote=409 erases=409`. Transfer time is `OTAB READY` → `OTAB STAT` on the device
  clock. Host wall time includes connect, the info probe and the reconnect check after the reboot.
- Each measured OTA ran on the build under test; the restore OTAs in between are not counted.

## Steady state (no OTA)

| build | INTERNAL free | min-ever | largest block | DMA free | DMA min-ever |
|---|---:|---:|---:|---:|---:|
| (1) main | 31 760 / 31 656 | 26 244 / 25 812 | 21 504 | 23 996 / 23 892 | 18 480 / 18 048 |
| (2) A | 35 578 / 35 512 | 31 000 / 29 392 | 25 600 | 27 924 / 27 748 | 23 236 / 21 628 |
| (3) A+B | 35 436 / 35 456 | 29 504 / 29 340 | 25 600 | 27 672 / 27 692 | 21 740 / 21 576 |
| (2) A with ACL 12 | 41 984 / 42 040 | 35 468 / 34 824 | 31 744 | 34 220 / 34 416 | 27 704 / 27 060 |

Bytes, two sessions each (after separate reboots). A adds +3.8 KB of free heap with ACL 34 and
+10.3 KB with ACL 12. The +3.8 KB is NimBLE and controller memory: msys_2 −12 × 320 B ≈ 3.84 KB plus
controller instances −3 × 828 B ≈ 2.48 KB, less incoming ACL +10 × 284 B ≈ 2.84 KB. The Wi-Fi static
RX line contributes nothing (see Changes). B costs nothing at steady state. The absolute level is about 7 KB above the
24.9 KB of the heap report, because that image carried the heap-task-tracking overlay. The
differences between builds are what this comparison is for.

## Wi-Fi OTA (`ota <url>`)

| build | download | host wall | min during OTA | DMA min during OTA | free after BLE release |
|---|---:|---:|---:|---:|---|
| (1) main | 24.4 / 24.5 s | 81 / 88 s | 10 436 / 10 368 | 2 672 / 2 604 | n/a |
| (2) A | 24.5 / 24.8 s | 87 / 87 s | 15 772 / 14 392 | 8 008 / 6 628 | n/a |
| (3) A+B | 23.4 / 23.0 s | 83 / 84 s | 32 800 / 32 692 | 25 036 / 24 928 | 38 496 → 103 668, 38 504 → 103 700 |
| (2) A with ACL 12 | 24.9 / 24.5 s | 88 / 85 s | 20 784 / 21 648 | 13 020 / 13 884 | n/a |

- Main reaches 10.4 KB internal and **2.6 KB DMA** during a download. This is the same DMA floor as
  flat's 2.6 KB min-ever, so the OTA download is a plausible source of the live DMA minimum.
- With B the minimum is set before the download, during the conversion stop. Once BLE is released
  free heap is about 100 KB and does not come close to that minimum again.
- The BLE release frees 58.7-65.2 KB of the ~110 KB that BLE costs (82.6 KB heap + 27.3 KB static,
  heap report). Which part stays allocated (wrapper objects, bond store, static RAM that
  `btMemRelease` does not return) was not traced.
- Download time drops by about 5 % (23.0-23.4 s vs 24.4-24.8 s). That fits a download limited by
  the network path: BLE airtime was not the bottleneck on this LAN. Whether the radio release helps
  on the slower fry/flat NAT path was not measured.

## BLE OTA (`ota-ble`)

| build | transfer | throughput | host wall | min during OTA | DMA min during OTA | free at Wi-Fi release |
|---|---:|---:|---:|---:|---:|---|
| (1) main | 78.3 / 80.0 s | 21.3 / 20.9 KB/s | 94 / 96 s | 16 156 / 14 320 | 8 392 / 6 556 | n/a |
| (2) A | 64.6 / 61.2 s | 25.8 / 27.3 KB/s | 81 / 77 s | 18 404 / 17 880 | 10 640 / 10 116 | n/a |
| (3) A+B | 33.1 / 31.7 s | 50.3 / 52.7 KB/s | 50 / 48 s | 22 772 / 22 664 | 15 008 / 14 900 | 26 512 → 80 384, 26 124 → 80 344 |
| (2) A with ACL 12 | 65.2 / 64.3 s | 25.6 / 25.9 KB/s | 87 / 80 s | 24 624 / 24 440 | 16 860 / 16 676 | n/a |
| (3) A+B with ACL 12 | 34.4 s | 48.5 KB/s | 51 s | 27 408 | 19 644 | 32 652 → 86 864 |

(KB/s = 1024 B/s.)

- With Wi-Fi released, a full-write BLE push takes half the time (32 s instead of 61-65 s with A and
  78-80 s on main). Wi-Fi coexistence was taking that airtime.
- A alone was also faster than main (61-65 s vs 78-80 s, two runs each). The cause was not
  isolated. One candidate is the smaller Wi-Fi BA window changing coexistence timing (the static RX
  buffer count did not change at runtime).
- The minimum during a BLE OTA comes before the Wi-Fi release: the 8 KB ring and the 4 KB sector
  buffer are allocated first, and the release follows in `quiesce(true)`.
- Every run logs one `heap: alloc failed 4096 B caps 0x400` line. That is esp-ota-ble trying PSRAM
  first for the sector buffer, which then falls back to internal RAM. It is noise, not a fault.

## Failure paths (build 3, A+B)

Each case was run once. "Back" means: after the reboot, `bootinfo` showed the same image with
`rollback=VALID` and `reset reason: SW`, `svc list` showed mqtt connected and ble running, and both a
BLE console (`fugu_console.py --ble`) and an MQTT console answered `hostname`.

| transport | case | how | device | back |
|---|---|---|---|---|
| Wi-Fi | disconnect mid-transfer | server closes after 300 kB (17 %) | `update failed: ESP_FAIL at 17 %`, reboot | yes |
| Wi-Fi | stalled server | server sends 300 kB, then waits 120 s | hung in `esp_https_ota` until the server closed (123 s), then fail, reboot | yes |
| Wi-Fi | timeout | server sends 300 kB, then waits 420 s | `Ota timeout after 300.0 seconds` (OTA watchdog), reboot | yes |
| Wi-Fi | bad image | full image with 4 KB corrupted mid-image | `Image hash failed`, `ESP_ERR_OTA_VALIDATE_FAILED`, reboot | yes |
| Wi-Fi | sent from the BLE console | `fugu_console.py --ble -c "ota <url>"` | BLE link closed at release (client printed "ota dropped"), download completed, reboot into the new slot | yes |
| BLE | host killed mid-transfer | SIGKILL to `ota_ble.py` at 28 % | `client disconnected`, `OTAB FAIL aborted`, reboot | yes |
| BLE | `ota-ble abort` | serial console at 34 % | `OTAB FAIL aborted` (reached the host), reboot | yes |
| BLE | stall | SIGSTOP the host for 45 s at 33 % | `OTAB FAIL stalled` after 30 s, reboot | yes |
| BLE | bad image | ESP32-classic image (valid hash, wrong chip) | `OTAB FAIL esp_ota_end ESP_ERR_OTA_VALIDATE_FAILED`, reboot | yes |
| BLE | `svc off ble` mid-transfer | serial console at 27 % | `OTAB FAIL aborted`, reboot | Wi-Fi yes; BLE stays off because `svc off` persists `ble.conf enabled=0` (intended) |

A corrupted raw image cannot be pushed over BLE at all: `ota_ble.py` refuses it before `begin`
because the image hash does not verify. The ESP32-classic image was used for that reason.

## Refusals and msys_1 floors (second round)

Build: HEAD after the review fixes plus the measurement patch and two test-only lines: a counter of
floor hits printed as `MEAS msys1 min=… con-floor=… tele-floor=…`, and a fixed random static BLE
address. The address was needed because bench139c had lost its bond with this Mac (another session's
test build in between); macOS then fails every connect with `CBErrorDomain Code=14 "Peer removed
pairing information"`, also with `ble_security none`, and `blueutil --unpair` does not remove LE bonds.
Two variants (V3m, V3b) differing in one string. Defaults `sdkconfig.defaults;sdkconfig.fleet;sdkconfig.vconv_s3`
from a fresh sdkconfig.

| test | result |
|---|---|
| Wi-Fi OTA, then at 10 s uptime (PENDING_VERIFY) `ota <url>` on serial | `W main: ota: ESP_ERR_OTA_ROLLBACK_INVALID_STATE, running image not yet confirmed`, `ERR:`; no conversion stop, no BLE release, no reboot |
| same boot, `ota-ble begin <size> <sha>` on serial | `OTAB FAIL esp_ota_begin ESP_ERR_OTA_ROLLBACK_INVALID_STATE`, `ERR:` |
| same boot, `etc/ota_ble.py` push started at ~20 s | first `begin` refused with the same line at 20.3 s, no Wi-Fi release; host retried, image confirmed at 22.1 s, second `begin` armed, push OK (31 s) |
| `ota-ble begin` on serial, no BLE client, image VALID | `OTAB FAIL no-ble-client`, `ERR:` |
| `wifi off 1`, then `ota <url>` | `ota: WiFi not connected`, `ERR:`, BLE still running |
| `wifi on` and `wifi off 1` on serial during a BLE push | both `ERR: wifi: BLE OTA in progress, WiFi stays off until its reboot`; push OK |
| image never confirms (first test build lacked vconv, setup failed) | stayed PENDING_VERIFY; both refusals as above; a later `restart` rolled back to the previous slot as designed |

In every refusal case uptime kept counting and `bootinfo` showed the same slot and state afterwards.

mbuf floors, BLE console from this Mac with the telemetry stream on:

- Command loop (6 commands round robin, waiting for each reply), 60 s: 126/126 replies complete,
  telemetry 37 notifies, no floor hit. Repeated twice after a fresh boot: 42/42 and 42/42, ~180 B/s
  telemetry each.
- Burst (8 commands written at once, 10-15 times, `help` and `get-config`, ~5-8 k lines): the console
  floor engaged 16-28 times and the telemetry floor 14 times; `msys_1` free went down to 6 while the
  sum over both pools stayed at 18-24. The old floors (`os_msys_num_free() < 8` / `< 12`) would not
  have engaged at all. No `rc=6`, no assert, no reset; the link answered afterwards. Some
  `[log output dropped …]` markers appeared, from the half-FIFO cap of the log mirror under a burst
  of that size.
- Once, after the first burst variant, the telemetry stream delivered 0 bytes in two later sessions
  although `tele-ble 1` answered `on`; its floor counter stayed 0, so the floor was not what blocked
  it. It worked again after a reboot and did not recur in four later sessions, including a burst with
  26 KB of telemetry. Not investigated further.

Not bench-tested: the `g_restartBlocked` paths (they need a bootloader copy in progress) and the 5 s
disconnect fallback of `bleConsoleRelease()`.

## Third round (rebased onto main 815d5d67)

Changes: Improv Wi-Fi/scan RPCs are refused during a BLE push; a push leaves Wi-Fi alone when the user
already turned it off (no release, no reboot on failure) and a `wifi off <min>` timer waits for the
push to end; `restart-blocked` documented. On main, `ota`, `ota-ble` and `wifi` are not in the
read-only list, so an untrusted console (BLE with `ble_security none`, telnet without `trusted=1`)
cannot run them; the release and refusal code is unaffected. `bleConsoleRelease()` only runs inside
`doOta()`, where no console command can run until the reboot; `bleCommand()` (pair/bonds/unpair)
returns "service not running" once `bleInited` is false, and bonds stay in NVS.

Builds: fresh `sdkconfig.defaults;sdkconfig.fleet` (ACL 34, msys_2 12, MAX_ACT 3, BA window 4 in the
generated sdkconfig), classic ESP32, docs, host tests `con-guard-test` and `test_mask_secrets`.

Bench smoke on bench139c (new build + measurement patch, vconv defaults). bench139c's `board.conf`
still had `panel_sd`, which fails setup on main since the `bflow_*` rename, so `bflow_sd 40` was
added for the test and deleted again afterwards.

| test | result |
|---|---|
| Wi-Fi OTA to the new build | OK |
| at 10 s (PENDING_VERIFY): `ota <url>`, `ota-ble begin` | both refused with `ESP_ERR_OTA_ROLLBACK_INVALID_STATE`; image confirmed at 21.9 s |
| BLE client connected, `wifi off 5`, `ota-ble begin`, no data | no `WiFi released`; `OTAB FAIL stalled` at 30 s, then aborted; no reboot; Wi-Fi stayed off until `wifi on` |
| same with Wi-Fi on | `WiFi released 36 788 -> 84 372`, aborted, reboot, back VALID with mqtt and ble |
| restore: Wi-Fi OTA from the new build to 815 | `BLE released 46 540 -> 111 712`, OK, VALID |
| an image that never confirms (first attempt, `panel_sd`) | refused as above; `restart` rolled back |

Not done: a complete BLE OTA on the rebased build. With `justworks` this Mac fails every write with
`Insufficient Encryption` and does not pair (the stale bond for the public address; a random test
address did not pair either), and with `ble_security none` main's read-only gating refuses
`ota-ble`. The data path is unchanged from the second round, where full BLE pushes succeeded.

## Notes

- A stalled HTTP server keeps the download hung. Read timeouts do not end `esp_https_ota`; only a
  closed connection or the 5 min OTA watchdog does. This was already the case before B. With B the
  device has no BLE during that time, and its consoles are blocked anyway because the download runs
  on the network loop.
- After a failed BLE push the device reboots, so the receiver's resume point is lost. The host then
  sees `OTAB RESUME none` and starts again from 0.
- `svc on ble` after a boot with BLE disabled brought the stack up, but macOS rejected the CCCD write
  (`CBATTErrorDomain Code=3`). After a `restart` with `ble.conf enabled=1` it worked. This is
  unrelated to these changes and was not investigated.
- Opening `/dev/ttyACM0` on havan reset bench139c (`rst:0x15 USB_UART_CHIP_RESET`), even with
  DTR/RTS held low.
- **Existing build directories keep the old values.** An sdkconfig that already exists wins over the
  defaults, so `build/` and any other configured build dir still have the old buffer counts until its
  `sdkconfig` is deleted (configure prints a WARNING listing such keys). Fleet images must be built
  from a fresh sdkconfig; check `CONFIG_BT_NIMBLE_TRANSPORT_ACL_FROM_LL_COUNT=34` and
  `CONFIG_ESP_WIFI_RX_BA_WIN=4` in the build's sdkconfig before an OTA.
- Not tested: BLE OTA through the ESPHome proxy, a Wi-Fi OTA while a BLE client refuses to
  disconnect (the 5 s fallback that keeps BLE), and anything on fry/flat. The classic-ESP32 build
  compiles (not flashed).

## bench139c afterwards

Running fry-brk1-815-g8c0f7e02-dirty (the image it had before; now in `ota_1`, it was in `ota_0`),
`rollback=VALID`. `tele.conf` is back to `enabled=0 binary=0`, `ble.conf enabled=1`,
`ble_security=justworks` (set to `none` during the second round, restored). This Mac still holds a
stale bond for bench139c's public address, so a direct BLE connect from it fails with Code=14 until
the device is forgotten in System Settings. The services
are as before: mqtt, ftp, telnet, scope, ble running; tele and lcd stopped.
