---
title: "OTA over BLE"
sidebar_position: 2
---

# OTA over BLE (no Wi-Fi)

When there is no network, you can update the firmware over Bluetooth Low Energy. The host pushes the image
to the device over the existing BLE NUS link. The device flashes it to the passive OTA partition
with the native `esp_ota` API and reboots. The [Wi-Fi path](ota-wifi.md) works the other way: the device pulls
an image over HTTP, which needs a network.

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
python -m etc.ota_ble [path-to.bin] [device-name-or-address] [options]
```

| Option | Effect |
|---|---|
| `bin` | Firmware image (default `build/fugu-firmware.bin`) |
| `name`, `-n NAME`, `--name NAME` | Device name or BLE address filter; the positional form wins (default `$BLE_NAME`) |
| `--address ADDR` | Target BLE address, directly or through `--ble-proxy` |
| `-f`, `--force` | Push even if the device already reports this version |
| `-y`, `--yes` | Skip the confirmation prompts (image without BLE, plant-simulation image) |
| `--allow-driver-change` | Push even if the image's gate driver differs from the device's, or either is unknown |
| `--ble-proxy HOST[:PORT]` | Reach the device through an ESPHome `bluetooth_proxy` (plaintext API); `--proxy-password` defaults to `$ESPHOME_API_PASSWORD` |
| `--xform auto\|raw\|tamp\|delta` | Wire payload, see [Wire protocol](#wire-protocol) (default `auto`) |
| `--base-dir DIR` | Extra directory to search for the image the device runs, as a delta base (repeatable) |

:::warning `-n` differs between the scripts
In `etc/ota_ble.py`, `-n` is the device name. In `etc/ota.py`, `-n` is a dry run.
:::

If you omit the name or address, the script falls back to `$BLE_NAME`. Without that, it picks the single `fugu-*`
peripheral advertising NUS, and it refuses to guess when several are in range. It rejects a name that matches several devices
the same way. The direct-link transport options are described in [transport selection](ble-ota-transports.md).

## GATT

OTA uses the same Nordic UART Service as the BLE console. Commands go to RX, and status comes back on TX.
The firmware bytes go to the extra FW characteristic (`6E400004-…`, write-no-response), which requires the same
pairing as RX. See [GATT layout](../../reference/ble/gatt.md).

## Console commands

The host controls the update with plain console commands on RX. For debugging, these commands also work over UART, telnet, and MQTT,
but the bulk data flows only over the BLE FW characteristic. The device accepts these commands:

```
ota-ble begin <size> <sha256hex>          arm: validate size, halt the converter, turn Wi-Fi off (if on), erase the passive partition
ota-ble resume <off> <size> <sha256hex>   arm at <off> instead of 0, continuing an interrupted raw push in the same boot
ota-ble end                               finalize: drain, verify SHA-256, set boot partition, reboot
ota-ble abort                             cancel: esp_ota_abort, free staging, reboot into the running image
```

`<size>` is the image length in bytes; `<sha256hex>` is its lowercase SHA-256 (64 hex chars). A `resume` must name
the offset the device advertises and the same size and digest; the SHA-256 still covers the whole image, so a
mismatched prefix ends in `OTAB FAIL sha-mismatch` with the boot partition unchanged. `etc/ota_ble.py` falls back to
`begin` when the device refuses a resume.

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

## Device behaviour

The receiver is the [esp-ota-ble](https://github.com/fl4p/esp-ota-ble) component; its repository describes the
staging ring, the credit window and the integrity checks. On the device, a push behaves as follows:

- **Converter.** Conversion and the ADC stay halted for the whole push.
- **Wi-Fi off.** Once the push is armed, the device stops its network services and turns Wi-Fi off to free
  memory and airtime. Wi-Fi comes back with the reboot that ends the push. `wifi on|off` and Improv Wi-Fi
  provisioning are refused during the push, and a `wifi off <minutes>` timer waits for its end.
- **Failure.** Any end other than success (abort, BLE disconnect, 30 s without data, a SHA or image check failure)
  reboots the device into the image it was running, with Wi-Fi and the converter back. An interrupted push then
  starts again from the beginning, because the resume point does not survive the reboot. When Wi-Fi was already
  off, or the build has no Wi-Fi support, nothing is released: a failed push restores the converter without a
  reboot.
- **Integrity.** The device checks the SHA-256 of the whole image and ESP-IDF's image validation before it switches
  the boot partition. A dropped or corrupted chunk fails the push; it never boots.
- **Refusals.** `begin` and `resume` are refused before the converter is touched while no BLE client is connected
  (`OTAB FAIL no-ble-client`), while the running image is still *pending verify*
  (`OTAB FAIL esp_ota_begin ESP_ERR_OTA_ROLLBACK_INVALID_STATE`; `etc/ota_ble.py` waits and retries), or while a
  bootloader update blocks restarts (`OTAB FAIL restart-blocked`). After an OTA, check that `bootinfo` shows
  `rollback=VALID` before pushing again; see [rollback](ota-wifi.md#rollback-and-boot-watchdog).

## Notes and limitations

Keep these points in mind when you push an image:

- **Throughput and time.** Push time depends mostly on the payload: the default `--xform auto` sends a delta
  against the running image or a tamp-compressed image when the device offers it, and only falls back to the raw
  image. It also depends on the staging ring (256 KB in PSRAM, else 8 KB of internal heap, logged as
  `OTAB RING <n>`) and on how much flash the push writes (sectors that already hold the right bytes are skipped).
  Raw pushes of a whole image run at about 13 kB/s with the 256 KB ring and 2–7 kB/s with the 8 KB ring.
- **No PSRAM, tight heap.** Without PSRAM the ring falls back to 8 KB of internal heap. If even that fails to
  allocate, `begin` fails with `OTAB FAIL no-mem`.
- **`begin` rejects oversized images.** `size` must be ≤ the passive partition size, 1 871 872 B
  (`0x1c9000`, ≈ 1.87 MB or 1.79 MiB) per slot in `partitions.csv`. A too-large image fails fast; see
  [Binary size](../../development/binary-size.md).
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
