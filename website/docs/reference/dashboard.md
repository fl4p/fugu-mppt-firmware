---
title: Dashboard
sidebar_position: 2.5
---

# Dashboard

[`etc/config-tool/dashboard.html`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/etc/config-tool/dashboard.html)
is a single-page browser dashboard for one or more converters. It connects to them over Bluetooth Low Energy (BLE),
over an MQTT broker, or both, and shows one card per converter.

A hosted copy is at [/tools/dashboard.html](pathname:///tools/dashboard.html). The page is one HTML file plus the
telemetry decoder `teledec.js`, with no build step. The MQTT transport loads `mqtt.js` from a CDN, pinned to one
version and checked with a subresource integrity hash.

## What a card shows

Each converter gets a card, keyed by its hostname:

- **Live values**: output power, input and output voltage, output current, duty cycle and the `ntc_temp` temperature, from the telemetry
  stream. A card whose last data point is older than three times the usual update interval is dimmed.
- **Today's peak and energy**: the `P_max_today` and `E_today` telemetry fields.
- **Sweep curve**: the power-versus-input-voltage curve of the last MPPT sweep, read with `sweep-curve`, with the
  maximum power point and the live operating point marked. The page fetches the curve on connect and again after
  each sweep ends. A refresh button fetches it on demand.
- **Log tail**: the device console log, including the replies to commands sent from the page.
- **Console**: an input line that sends a command to the device over the active link. It is hidden until
  the **console** checkbox in the header is ticked; the page remembers the setting in this browser.
- **Link badges**: one for BLE and one for MQTT.
- **More info**: a collapsed box under the values with the stream rate and dropped bytes per link, the
  slower telemetry fields (`mcu_temp`, `lag`, `heap`, `heap_min`, lifetime `E`, the `Ro`/`Rc`/`Rb` resistances
  once known) and the MQTT lease state.

## Quick start

Web Bluetooth needs a Chromium browser (Chrome, Edge) and a secure context (`https://` or `localhost`). To run the
page from a checkout, serve the folder locally:

```bash
cd etc/config-tool
python3 -m http.server 8000
# open http://localhost:8000/dashboard.html
```

Then do one or both of the following:

- Enter a broker WebSocket URL, user and password, and click **Connect**. Every converter that logs to the broker
  appears as a card.
- Click **+ Add BLE device** and pick a converter in the browser's chooser. Repeat for each converter.

The page remembers the broker URL and user. Where the browser supports `navigator.bluetooth.getDevices()`, devices
allowed earlier reappear after a reload with a **Reconnect** button, so no new chooser click is needed.

## Transports

| Transport | Connection | Telemetry source | Requirements |
|-----------|------------|------------------|--------------|
| BLE | One NUS connection per device. The device advertises as `fugu-<hostname>`; each device needs one click in the chooser. | On connect, the page sends `set-time` (the device clock may be unset without Wi-Fi) and `tele-ble 1`, then decodes the stream from the telemetry characteristic. | Firmware built with BLE telemetry (`CONFIG_FUGU_WITH_BLE_TELE`), `ble` service running. See [Telemetry](../guide/telemetry/index.md). |
| MQTT | MQTT over WebSocket, `ws://` or `wss://`. The hosted `https` page can only use `wss://`. | Messages on `pv/tele/<hostname>`, either published by the device while a `tele-mqtt <seconds>` lease runs (build option `CONFIG_FUGU_WITH_TELE_MQTT`), or republished from the UDP telemetry wire by a host-side relay. | `mqtt.conf cmd_input=1` for commands. See [MQTT topics](mqtt.md#telemetry-stream). |

The page subscribes to `pv/log/#`, `pv/alert/#` and `pv/tele/#`, and sends commands on `pv/log/<hostname>/cmd`. When
a device has both links, BLE carries telemetry and commands, and the page does not lease the MQTT stream for it.

## Console commands used

| Command | When the page sends it |
|---------|------------------------|
| `set-time <epoch_ms>` | After a BLE connect, and over MQTT when a lease is refused because the clock is not set. |
| `tele-ble 1` | After a BLE connect, to start the BLE telemetry stream. |
| `tele-mqtt 120` | Over MQTT, about 5 s after a device first appears without relay telemetry. The page renews the lease every 60 s while the tab is open, and once more when the tab becomes visible again. |
| `sweep-curve` | On connect, after each sweep, and on the refresh button. Up to two retries, 4 s timeout each. |

See [Console commands](console.md) for the replies.

## Badge states

Hover a badge for its reason. The badge colour shows the state:

| Badge | State | Meaning |
|-------|-------|---------|
| BLE | on | Connected, telemetry stream running. |
| BLE | pending | Connecting, reconnecting, or enabling telemetry. |
| BLE | refused | Connected, but telemetry did not start: the firmware has no telemetry characteristic or no `tele-ble` command, or `tele-ble 1` returned an error such as an unset clock. |
| BLE | off | No BLE link, or disconnected. A **Reconnect** button appears on the card. |
| MQTT | on | Telemetry lease running (seconds left and dropped bytes in the tooltip), telemetry from a relay, a device without `tele-mqtt` (relay-fed), or log only. |
| MQTT | pending | Requesting a lease. |
| MQTT | refused | `tele-mqtt` returned an error (reason in the tooltip), or the device did not reply. The page retries a missing reply every 60 s. |
| MQTT | off | Broker not connected, or the device has not been seen on the broker. |

Without telemetry, a card shows the log and the sweep curve but no live values. The MQTT tooltip then tells why:

- "log only": the device logs to the broker, but no telemetry has arrived on `pv/tele/<hostname>` and no lease is
  requested yet, for example in the first seconds after the device appears.
- "firmware without tele-mqtt (relay-fed)": the device answered `tele-mqtt` with `Unknown command`. Live values
  appear only once a relay publishes its `pv/tele/<hostname>` topic.
- A `tele-mqtt` error (MQTT refused): the reason from the device, for example `low heap`, or UDP telemetry running the text wire (`tele.conf binary=0`). The page sets the clock itself when the reason is an unset clock.

## Security

:::caution
With **remember** ticked next to the password field, the broker password is stored in plain text in the browser's
`localStorage` for the page's origin. Unticked (the default), the password stays in memory for the session only, and
a stored password is removed. The URL and user are always stored.
:::

With `cmd_input=1`, anyone who can publish to the broker can control the converter, and the console line on each card
sends any command. Use broker authentication and ACLs, and give the dashboard's broker user only the topics it needs.
