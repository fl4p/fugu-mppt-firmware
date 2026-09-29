---
title: GATT layout
sidebar_position: 1
---

# GATT layout

`CONFIG_FUGU_WITH_BLE` builds expose one application service, the Nordic UART Service (NUS), while the `ble` service
is running (`svc on ble`, [`ble.conf`](../config/ble.md)). The console, OTA push and telemetry stream all use it.

| Role | UUID | Properties | Carries |
|------|------|------------|---------|
| Service (NUS) | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | — | — |
| RX (host→device) | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` | write, write-no-response | [Console](../console.md) input |
| TX (device→host) | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` | notify | Console output; the device's logs are mirrored here |
| FW (host→device) | `6E400004-B5A3-F393-E0A9-E50E24DCCA9E` | write-no-response | Raw firmware bytes of an [OTA push](../../guide/updating/ota-ble.md#wire-protocol) |
| TELE (device→host) | `6E400005-B5A3-F393-E0A9-E50E24DCCA9E` | notify | Binary telemetry stream, only in `CONFIG_FUGU_WITH_BLE_TELE` builds |

## Connection

- One client at a time.
- The device requests an ATT MTU of 247. Notifications are cut at the negotiated MTU − 3 bytes (20 bytes before the
  MTU exchange), so a client reassembles TX and TELE as byte streams, not as one message per notification.
- Bonds are stored in NVS, up to 3.

## Console on RX / TX

RX takes the same line protocol as the serial console: a line ends at `\r` or `\n` and holds up to 127 characters;
a longer line is discarded. The device echoes input, and after each command writes `OK: <cmd>` or `ERR: <cmd>` to
TX. The command bytes may be split across several writes.

## Telemetry on TELE

The stream starts with the `set-time` and `tele-ble 1` console commands (see [Telemetry fields](../telemetry-fields.md#transports)
for the preconditions). Each record is `<0x7E><varint len><cid><payload>`, where `<cid><payload>` matches one UDP
telemetry datagram, tamp-compressed.

## Security

[`ble.conf::ble_security`](../config/ble.md) sets the requirement on the writable characteristics, RX and FW. TX and
TELE carry no security requirement of their own.

| `ble_security` | Pairing | RX / FW writes |
|---|---|---|
| `justworks` (default) | Bonded, secure connections, no passkey | Encrypted link required |
| `passkey` | Bonded, secure connections, MITM protection with the static `ble_passkey` | Authenticated link required |
| any other value | None | Open |

## Advertising

The device advertises under its hostname (see [Connecting](../../guide/connecting.md#ble)) with the NUS service
UUID in the advertising data, and answers scan requests. While a client is connected it stops advertising.

`CONFIG_FUGU_WITH_BLE_ADV` builds add a connectionless telemetry record to the advertising data, see
[BLE advertising record](../telemetry-fields.md#ble-advertising-record). These builds keep advertising while a
client is connected, then non-connectable.

Hosts cache the GATT table of a known device. After a firmware change that alters it, a host can keep using the old
table; see [Troubleshooting](../../guide/troubleshooting.md#ble).
