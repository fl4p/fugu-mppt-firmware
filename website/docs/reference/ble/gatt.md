---
title: GATT layout
sidebar_position: 1
---

# GATT layout

`CONFIG_FUGU_WITH_BLE` builds expose one application service, the Nordic UART Service (NUS), while the `ble` service
is running (`svc on ble`, [`ble.conf`](../config/ble.md)). The console, OTA push, and telemetry stream all use it.
The following table lists the service and its characteristics:

| Role | UUID | Properties | Carries |
|------|------|------------|---------|
| Service (NUS) | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | — | — |
| RX (host→device) | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` | write, write-no-response | [Console](../console.md) input |
| TX (device→host) | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` | notify | Console output; the device's logs are mirrored here |
| FW (host→device) | `6E400004-B5A3-F393-E0A9-E50E24DCCA9E` | write-no-response | Raw firmware bytes of an [OTA push](../../guide/updating/ota-ble.md#wire-protocol) |
| TELE (device→host) | `6E400005-B5A3-F393-E0A9-E50E24DCCA9E` | notify | Binary telemetry stream, only in `CONFIG_FUGU_WITH_BLE_TELE` builds |

`CONFIG_FUGU_WITH_IMPROV` builds add the [Improv Wi-Fi](https://www.improv-wifi.com/ble/) service
(`00467768-6228-2272-4663-277478268000`) with its five standard characteristics. Its RPC command characteristic takes
write with response only, under the same security as RX. Capabilities are device info and network scan (`0x06`). See
[Wi-Fi setup](../../guide/connecting.md#improv-over-ble) for when it accepts credentials; a negative `wifi.conf::improv_offline_s` omits the service.

## Connection

The device applies these connection limits:

- One client at a time.
- The device requests an ATT MTU of 247. Notifications are cut at the negotiated MTU − 3 bytes (20 bytes before the
  MTU exchange), so a client reassembles TX and TELE as byte streams, not as one message per notification.
- Bonds are stored in NVS, up to 3.

## Console on RX / TX

RX takes the same line protocol as the serial console: a line ends at `\r` or `\n` and holds up to 127 characters.
A longer line is discarded. The device echoes input, and after each command writes `OK: <cmd>` or `ERR: <cmd>` to
TX. The command bytes may be split across several writes.

## Telemetry on TELE

The `set-time` and `tele-ble 1` console commands start the stream (see [Telemetry fields](../telemetry-fields.md#transports)
for the preconditions). Each record is `<0x7E><varint len><cid><payload>`, where `<cid><payload>` matches one UDP
telemetry datagram, tamp-compressed.

## Security

[`ble.conf::ble_security`](../config/ble.md) sets the requirement on the writable characteristics, RX and FW. TX and
TELE carry no security requirement of their own. Each value sets pairing and write access as follows:

| `ble_security` | Pairing | RX / FW writes |
|---|---|---|
| `justworks` (default) | Bonded, secure connections, no passkey | Encrypted link required |
| `passkey` | Bonded, secure connections, MITM protection with the static `ble_passkey` | Authenticated link required |
| any other value | None | Open |

With `justworks` or `passkey`, the device sends a security request on connect only to a bonded peer, which then
re-encrypts with its stored key. A peer without a bond can connect, discover services and read open characteristics
without pairing; its first write to RX, FW or the Improv command characteristic fails with "insufficient
authentication", and the client pairs and retries. TX carries no output until the link is encrypted. A peer without a bond can pair only while the
[pairing window](../config/ble.md#pairing-window) is open; otherwise the device answers its pairing request with
"command not supported".

## Advertising

The device advertises under its hostname (see [Connecting](../../guide/connecting.md#ble)) and answers scan
requests. It stops advertising while a client is connected.

| Packet | Content |
|---|---|
| Advertising data | Flags `0x06`, complete local name (`0x09`); a name over 26 characters is cut to 26 and sent as shortened name (`0x08`) |
| Scan response | Complete list of 128-bit service UUIDs (`0x07`): NUS |

Passive scanners, such as a Bluetooth proxy in passive mode, see only the advertising data, so they find the device
by name. Active scanners (Web Bluetooth, CoreBluetooth, Android) merge the scan response and can filter on the NUS
service UUID.

While Improv accepts credentials, the advertising data alternates every 0.5 s between two payloads, because the
Improv service UUID and its `0x4677` service data must share one advertisement and fill all 31 bytes:

| Payload | Content |
|---|---|
| Console | Flags, name, Improv service data `0x4677` if it still fits (names up to 16 characters) |
| Improv | Flags, Improv service UUID, service data `0x4677`: state, capabilities, 4 reserved bytes |

In `CONFIG_FUGU_WITH_BLE_ADV` builds the Improv payload takes every other telemetry slot while advertising is
connectable.

`CONFIG_FUGU_WITH_BLE_ADV` builds add a connectionless telemetry record to the advertising data (see
[BLE advertising record](../telemetry-fields.md#ble-advertising-record)). These builds keep advertising while a
client is connected, and the advertisements are then non-connectable.

Hosts cache the GATT table of a known device. After a firmware change that alters it, a host can keep using the old
table. See [Troubleshooting](../../guide/troubleshooting.md#ble) for this case.
