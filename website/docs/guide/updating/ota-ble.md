---
title: "OTA over BLE"
sidebar_position: 2
---

*this document is an LLM generated placeholder*

# OTA over BLE (no Wi-Fi)

Update the firmware over Bluetooth Low Energy when there is no network. The host **pushes** the image
to the device over the existing BLE NUS link, and the device flashes it to the passive OTA partition
with the native `esp_ota` API and reboots. This complements the Wi-Fi path (`ota <url>`, see the
top-level docs / `ota.sh`), which *pulls* an image over HTTP and needs a network.

Only present in **`WITH_BLE`** firmware builds, and only usable while the `ble` service is running.

## Quick start

```bash
# 1. build a WITH_BLE image (ESP-IDF exported, see Getting started)
idf.py menuconfig (enable CONFIG_FUGU_WITH_BLE), then idf.py build

# 2. make sure the device advertises (ble service enabled). Over any console:
#      svc on ble           # or set ble.conf enabled=1 and reboot
#    The BLE advertised name is the device hostname, e.g. fugu-esp32s3-XXXXXXXXXXXX

# 3. push the image from a host with Bluetooth (bleak required: pip install bleak)
python -m etc.ota_ble build/fugu-firmware.bin fugu-esp32s3-XXXXXXXXXXXX
```

`etc/ota_ble.py` connects, streams the image, waits for the device to confirm it has the whole image,
finalizes, and confirms the device re-advertises after the reboot. Arguments:

```
python -m etc.ota_ble [path-to.bin] [device-name-or-address]
```

If the name/address is omitted it falls back to `$BLE_NAME`, then to the first peripheral advertising
the NUS service.

## GATT layout

The OTA data rides the same Nordic UART Service (NUS) the BLE console uses; one extra characteristic
is added for the firmware bytes:

| Role | UUID | Properties |
|------|------|------------|
| Service (NUS) | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | — |
| RX (host→device) | `6E400002-…` | write / write-no-response — console commands |
| TX (device→host) | `6E400003-…` | notify — console output + status (logs are mirrored here) |
| FW (host→device) | `6E400004-…` | write-no-response — raw firmware bytes |

The FW characteristic carries binary and bypasses the console line parser entirely. It requires the
same pairing as RX (encrypted under `ble_security=justworks`/`passkey`).

## Console commands

The control plane is plain console commands on RX (so it also works over UART/telnet/MQTT for
debugging, though the bulk data only flows over the BLE FW characteristic):

```
ota-ble begin <size> <sha256hex>   arm: validate size, halt the converter, erase the passive partition
ota-ble end                        finalize: drain, verify SHA-256, set boot partition, reboot
ota-ble abort                      cancel: esp_ota_abort, free staging, re-enable the converter
```

`<size>` is the image length in bytes; `<sha256hex>` is its lowercase SHA-256 (64 hex chars).

## Wire protocol

Status is reported as `OTAB …` log lines on the TX notify channel (the device mirrors its logs to the
connected client, so these arrive as ordinary notifications):

```
OTAB READY part=<label> size=<n>   armed; partition erased; ready to receive
OTAB CRED <G>                       credit: host may stream up to cumulative byte offset G
OTAB PROG <written>/<size>          progress (also emitted once when written == size)
OTAB OK rebooting                   verified + boot partition set; device reboots
OTAB FAIL <reason>                  rejected (bad-sha, no-partition, size…, no-mem, sha-mismatch,
                                    incomplete, esp_ota_*, aborted)
```

End-to-end sequence:

```
host  → RX : ota-ble begin <size> <sha>
device→ TX : OTAB READY …            (only after the erase completes)
device→ TX : OTAB CRED <G>
host  → FW : firmware bytes, in ATT-MTU-sized chunks, never exceeding the credit offset G
device→ TX : OTAB CRED <G> / OTAB PROG …   (as bytes are flushed to flash)
host       : (wait for OTAB PROG <size>/<size>)
host  → RX : ota-ble end
device→ TX : OTAB OK rebooting        (then reboots)  — or OTAB FAIL <reason>
```

## How it works (firmware)

`src/etc/ota_ble.cpp` (+ `.h`), wired into `src/console_ble.cpp` and `src/cli.cpp`.

- **Staging ring.** The FW characteristic's `onWrite` runs on the NimBLE host task and only *copies*
  bytes into a small ring buffer (`RING_CAP`, 8 KB). It never touches flash — a multi-millisecond
  flash stall on the host task would trip the BLE supervision timeout and drop the link.
- **Draining.** `otaBleTick()` runs on the network loop (core 0, from `bleConsoleLoop`). It pulls
  slices out of the ring and calls `esp_ota_write` + a streaming `mbedtls_sha256_update`, outside the
  ring lock so the host task never blocks on flash.
- **Flow control (credit window).** The ring capacity is the host's credit window. The device
  advertises a cumulative high-water offset `G = written + RING_CAP` via `OTAB CRED`; the host streams
  up to `G` and waits for a larger credit. Because BLE throughput (~tens of KB/s) is far below what an
  8 KB window sustains, the credit round-trip never bottlenecks. A drop (write-no-response can overflow
  controller buffers) is caught by the final SHA/length check, which forces a full retry.
- **Integrity.** Streaming SHA-256 compared against the host-supplied digest, **and** `esp_ota_end`'s
  built-in image validation. Only then is the boot partition switched.
- **Safety.** The converter is halted (`stopAndBackoff`, ADC halted) for the duration. All OTA state
  mutation happens on the network loop; a BLE disconnect requests an abort (`otaBleRequestAbort`) that
  the next tick performs, so a dropped link never leaves a half-written partition armed and always
  restores the converter.

## Notes & gotchas

- **Throughput / time.** ~32 KB/s in practice → roughly a minute for a ~1.8 MB image.
- **No PSRAM, tight heap.** The 8 KB ring is allocated from PSRAM if present, else internal heap. On a
  no-PSRAM board with a fragmented heap a larger ring fails to allocate (`OTAB FAIL no-mem`); 8 KB is
  deliberately small for this reason.
- **`begin` rejects oversized images.** `size` must be ≤ the passive partition size (the OTA slot is
  `0x1c9000` ≈ 1.78 MB; the `WITH_BLE` image is a tight fit). A too-large image fails fast.
- **macOS GATT cache.** CoreBluetooth caches a device's GATT by its (stable) BLE address. After a
  firmware change that alters the GATT — e.g. the first build that adds the FW characteristic — macOS
  keeps serving the stale service and bleak reports `Characteristic 6e400004-… was not found`. Flush
  it with `blueutil -p 0 && blueutil -p 1` (toggle Bluetooth). Any later GATT change needs the same.
- **Completion.** On success the device reboots immediately after queuing `OTAB OK`, so that
  notification usually never drains over the link. The host therefore waits for `OTAB PROG
  <size>/<size>` before sending `ota-ble end`, and does not rely on receiving `OTAB OK`. Over a direct link, success is
  confirmed by reading the new image's digest from the running slot after the reboot; a disconnect followed by
  re-advertising alone is only accepted on the ESPHome-proxy path. See [transport selection](ble-ota-transports.md).

See also: [Console](../../reference/console.md), [Services](../../reference/services.md), [Logging](../../development/debugging/logging.md).
