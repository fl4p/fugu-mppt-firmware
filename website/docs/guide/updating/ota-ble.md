---
title: "OTA over BLE"
sidebar_position: 2
---

# OTA over BLE (no Wi-Fi)

When there is no network, you can update the firmware over Bluetooth Low Energy. The host pushes the image
to the device over the existing BLE NUS link. The device flashes it to the passive OTA partition
with the native `esp_ota` API and reboots. The Wi-Fi path (`ota <url>`, see the
top-level docs / `ota.sh`) works the other way: the device pulls an image over HTTP, which needs a network.

OTA over BLE is present only in `CONFIG_FUGU_WITH_BLE` firmware builds, and you can use it only while the `ble` service is running.

## Quick start

The following commands build the image, make the device advertise, and push the image:

```bash
# 1. build (CONFIG_FUGU_WITH_BLE is on by default; ESP-IDF exported, see Getting started)
idf.py build

# 2. make sure the device advertises (ble service enabled). Over any console:
#      svc on ble           # or set ble.conf enabled=1 and reboot
#    The BLE advertised name is the device hostname, e.g. fugu-esp32s3-XXXXXXXXXXXX

# 3. push the image from a host with Bluetooth (bleak required: pip install bleak)
python -m etc.ota_ble build/fugu-firmware.bin fugu-esp32s3-XXXXXXXXXXXX
```

`etc/ota_ble.py` connects, streams the image, and waits for the device to confirm it has the whole image.
It then finalizes, confirms the device re-advertises after the reboot, and archives the build ELF for coredump
decoding. The script takes these arguments:

```
python -m etc.ota_ble [path-to.bin] [device-name-or-address]
```

If you omit the name or address, the script falls back to `$BLE_NAME`. Without that, it picks the single `fugu-*`
peripheral advertising NUS, and it refuses to guess when several are in range. It rejects a name that matches several devices
the same way.

## GATT

OTA uses the same Nordic UART Service as the BLE console. Commands go to RX, and status comes back on TX.
The firmware bytes go to the extra FW characteristic (`6E400004-…`, write-no-response), which requires the same
pairing as RX. See [GATT layout](../../reference/ble/gatt.md).

## Console commands

The host controls the update with plain console commands on RX. For debugging, these commands also work over UART, telnet, and MQTT,
but the bulk data flows only over the BLE FW characteristic. The device accepts these commands:

```
ota-ble begin <size> <sha256hex>   arm: validate size, halt the converter, erase the passive partition
ota-ble end                        finalize: drain, verify SHA-256, set boot partition, reboot
ota-ble abort                      cancel: esp_ota_abort, free staging, re-enable the converter
```

`<size>` is the image length in bytes; `<sha256hex>` is its lowercase SHA-256 (64 hex chars).

## Wire protocol

The host picks how the image travels with `--xform auto|raw|tamp|delta`. The default, `auto`, uses whatever the device offers.
This section describes the raw case.

The device reports status as `OTAB …` log lines on the TX notify channel. Because the device mirrors its logs to the
connected client, these lines arrive as ordinary notifications. The device sends these status lines:

```
OTAB READY part=<label> size=<n> … armed and ready to receive (erasure proceeds during the transfer)
OTAB CRED <G>                       credit: host may stream up to cumulative byte offset G
OTAB PROG <written>/<size>          progress (also emitted once when written == size)
OTAB OK rebooting                   verified + boot partition set; device reboots
OTAB FAIL <reason>                  rejected (bad-sha, no-partition, size…, no-mem, sha-mismatch,
                                    incomplete, esp_ota_*, aborted)
```

A complete push runs in this sequence:

```
host  → RX : ota-ble begin <size> <sha>
device→ TX : OTAB READY …            (armed; erasure proceeds during the transfer)
device→ TX : OTAB CRED <G>
host  → FW : firmware bytes, in ATT-MTU-sized chunks, never exceeding the credit offset G
device→ TX : OTAB CRED <G> / OTAB PROG …   (as bytes are flushed to flash)
host       : (wait for OTAB PROG <size>/<size>)
host  → RX : ota-ble end
device→ TX : OTAB OK rebooting        (then reboots)  — or OTAB FAIL <reason>
```

## How it works (firmware)

The firmware side is the [esp-ota-ble](https://github.com/fl4p/esp-ota-ble) component, wired in by
`src/tele/console_ble.cpp` and the `ota-ble` command in `src/cli.cpp`.

The component handles a push in these parts:

- **Staging ring.** The FW characteristic's `onWrite` runs on the NimBLE host task and only copies
  bytes into a ring buffer: 256 KB in PSRAM when available, else 8 KB (logged as `OTAB RING <n>`). It never touches flash, because a multi-millisecond
  flash stall on the host task would trip the BLE supervision timeout and drop the link.
- **Draining.** `otaBleTick()` runs on the network loop (core 0, from `bleConsoleLoop`). It pulls
  slices out of the ring and calls `esp_ota_write` + a streaming `mbedtls_sha256_update`, outside the
  ring lock so the host task never blocks on flash.
- **Flow control (credit window).** The ring capacity is the host's credit window. The device
  advertises a cumulative high-water offset `G` via `OTAB CRED`. The host streams up to `G` and waits
  for a larger credit. Write-no-response can overflow controller buffers and drop data. The final SHA/length check
  catches such a drop, which forces a full retry.
- **Integrity.** The device compares a streaming SHA-256 against the host-supplied digest and also runs `esp_ota_end`'s
  built-in image validation. Only then does it switch the boot partition.
- **Safety.** The converter stays halted (`stopAndBackoff`, ADC halted) for the duration. All OTA state
  mutation happens on the network loop. A BLE disconnect requests an abort (`otaBleRequestAbort`) that
  the next tick performs. After a dropped link, the device therefore never leaves a half-written partition armed, and it always
  restores the converter.

## Notes and limitations

Keep these points in mind when you push an image:

- **Throughput and time.** Throughput depends on the ring size and on how much flash the push really writes (sectors that
  already hold the right bytes are skipped). Bench pushes that rewrote the whole image measured ~13 kB/s with the
  256 KB PSRAM ring and 2–7 kB/s with the 8 KB ring.
- **No PSRAM, tight heap.** Without PSRAM the ring falls back to 8 KB of internal heap. If even that fails to
  allocate, `begin` fails with `OTAB FAIL no-mem`.
- **`begin` rejects oversized images.** `size` must be ≤ the passive partition size. The OTA slot is
  `0x1c9000` ≈ 1.78 MB, and the `CONFIG_FUGU_WITH_BLE` image is a tight fit. A too-large image fails fast.
- **macOS GATT cache.** CoreBluetooth caches a device's GATT by its (stable) BLE address. After a
  firmware change that alters the GATT (e.g. the first build that adds the FW characteristic), macOS
  keeps serving the stale service, and bleak reports `Characteristic 6e400004-… was not found`. To flush
  the cache, toggle Bluetooth with `blueutil -p 0 && blueutil -p 1`. Any later GATT change needs the same flush.
- **Completion.** On success the device reboots immediately after queuing `OTAB OK`, so that
  notification usually never drains over the link. The host therefore waits for `OTAB PROG
  <size>/<size>` before sending `ota-ble end`, and doesn't rely on receiving `OTAB OK`. Over a direct link, the host
  confirms success by reading the new image's digest from the running slot after the reboot. Only the ESPHome-proxy path accepts a disconnect followed by
  re-advertising alone as success. See [transport selection](ble-ota-transports.md).

See also: [Console](../../reference/console.md), [Services](../../reference/services.md), [Logging](../../development/debugging/logging.md).
