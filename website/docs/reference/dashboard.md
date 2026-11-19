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
  stream. The header shows the age of the last data point ("now" up to 2 s). A card whose last data point is older than three times the usual update interval is dimmed.
- **Today's peak and energy**: the `P_max_today` and `E_today` telemetry fields.
- **Sweep curve**: the power-versus-input-voltage curve of the last MPPT sweep, read with `sweep-curve`, with the
  maximum power point and the live operating point marked. The page fetches the curve on connect and again after
  each sweep ends. A sweep that the device flagged because the sky changed during it
  ([sweep verdict](../internals/mppt-tracker.md#sweep-verdict)) is drawn dimmed and dashed, and the caption adds
  "sky changed during sweep". A refresh button fetches it on demand, and a **sweep** button starts a new global sweep. A dashed **live** curve is built from the telemetry
  stream in 0.25 V bins of input voltage: each sample sets the power of its own bin and of the existing bins within
  ±0.5 V. Bins stay for the session (at most 400; the least recently updated goes first), parts not updated for ten
  minutes are drawn faded, and the line breaks where neighbouring bins are more than 2 V apart. The power axis grows to the highest plotted value and does not shrink until a new sweep curve arrives or the refresh button is pressed, which start it over.
- **Input voltage, last 30 s**: a small chart under the sweep curve with the input voltage of the last 30 s of
  telemetry (the last sample of each 0.1 s), the newest sample at the right edge and the current value in its
  caption. The voltage axis spans the window's minimum to maximum plus 5 %. Hidden until telemetry arrives.
- **Log tail and console**: the device console log, including the replies to commands sent from the page,
  and an input line that sends a command to the device over the active link. Both are hidden until the
  **log & console** checkbox in the header is ticked; the page remembers the setting in this browser.
  The periodic `V=…` status lines and the `sc:` curve dumps are hidden unless the card's **status lines**
  and **sc: lines** checkboxes are ticked.
- **Live values from the status line**: the periodic `V=…` status line in the device log also updates power,
  voltages, current and mode, so a card stays current between telemetry points, for example while a device in
  its idle profile sends telemetry only every `tele.conf idle_interval_ms`.
- **Link badges**: one for BLE (only on cards with a BLE link) and one for MQTT. Alerts the device publishes
  under `pv/alert/<hostname>/<source>` appear as one badge per source, coloured by level, with the message
  and time as tooltip; an alert that arrives while the page is open is also written to the log.
- **More info**: a collapsed box under the values with the stream rate and dropped bytes per link, the
  slower telemetry fields (`mcu_temp`, `lag`, `heap`, `heap_min`, lifetime `E`, the `Ro`/`Rc`/`Rb` resistances
  once known), the MQTT lease state, and the telemetry decoder's error and resync counters.

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

The page remembers the broker URL, user and, unless **remember** is unticked, the password; with a user but no stored password it does not connect by itself and asks for the password instead. Where the browser supports `navigator.bluetooth.getDevices()`, devices
allowed earlier reappear after a reload. The page tries one connection to each `fugu-*` device without a click; when
that fails, or the browser requires a click, the card shows a **Reconnect** button instead.

The page also keeps a snapshot of up to 16 converters (the 16 seen most recently) in the browser's `localStorage`:
hostname, links seen, the last telemetry values, the last sweep curve and the BLE device name. After a reload these
cards appear at once, greyed, with the age of the values, until live data arrives; on connect the page asks each saved
host on the broker for its `hostname`. The **×** on a card removes the card and its snapshot. Log lines, live curve
bins and credentials are not part of the snapshot.

## Transports

| Transport | Connection | Telemetry source | Requirements |
|-----------|------------|------------------|--------------|
| BLE | One NUS connection per device. The device advertises as `fugu-<hostname>`; each device needs one click in the chooser. | On connect, the page sends `tele-ble 1` and decodes the stream from the telemetry characteristic. When the device refuses because its clock is not set (no Wi-Fi, no SNTP), the page sends `set-time` and tries once more. | Firmware built with BLE telemetry (`CONFIG_FUGU_WITH_BLE_TELE`), `ble` service running. See [Telemetry](../guide/telemetry/index.md). |
| MQTT | MQTT over WebSocket, `ws://` or `wss://`. The hosted `https` page can only use `wss://`. | Messages on `pv/tele/<hostname>`, either published by the device while a `tele-mqtt <seconds>` lease runs (build option `CONFIG_FUGU_WITH_TELE_MQTT`), or republished from the UDP telemetry wire by a host-side relay. | `mqtt.conf cmd_input=1` for commands. See [MQTT topics](mqtt.md#telemetry-stream). |

The page subscribes to `pv/log/#`, `pv/alert/#` and `pv/tele/#`, and sends commands on `pv/log/<hostname>/cmd`. When
a device has both links, BLE carries telemetry and commands, and the page does not lease the MQTT stream for it.

### Finding the broker

When no broker URL is stored and no MQTT client is connected, the page reads the broker from the first device that
connects over BLE:

1. After the BLE enable sequence, the page sends `get-config mqtt.conf` and takes the host from `broker_uri`, plus
   `username` and `password`.
2. It probes WebSocket listeners on that host, one at a time with a 3 s timeout each: `ws://<host>:1885`, `:9001`,
   `:8083`, `:8080`, and last the port from `broker_uri`. The first that accepts a connection (subprotocol `mqtt`)
   wins; the probe closes it at once.
3. On success the page fills in the URL, user and password fields and shows a banner with a **Connect** button. It
   does not connect by itself. The password stays in memory unless **remember** is ticked.

Without a listener on any of these ports, a banner says the broker has no WebSocket listener on the usual ports. An
`https` page cannot open `ws://` connections, so it probes `wss://<host>:8884`, `:8084` and `:443` instead and, when
none opens, says so.

### TLS

The hosted `https` copy of the page can only talk to a broker over `wss://`, and the browser must trust the broker's
certificate. A broker on a private network usually has no publicly issued certificate; a small private certificate
authority works: issue the broker a server certificate whose subject alternative names list the IP addresses and host
names clients use, give the broker the certificate and key (Mosquitto: `certfile`, `keyfile` on a `websockets`
listener), and install the CA certificate once on each phone or computer that should open the page. A certificate the
browser does not trust fails silently in the probe and shows as a connection error on Connect.

The fields can also be prefilled from the page URL: `dashboard.html?mqtt=ws://<host>:<port>&user=<name>`. The page
does not connect on its own then, and never reads a password from the URL.

## Console commands used

| Command | When the page sends it |
|---------|------------------------|
| `set-time <epoch_ms>` | Only when `tele-ble 1` or `tele-mqtt` is refused because the device clock is not set, followed by one retry of that command. |
| `hostname` | After a BLE connect, to name the card. The page waits up to 8 s and asks once more; without a reply it uses the advertised name minus `fugu-`. |
| `tele-ble 1` | After a BLE connect, to start the BLE telemetry stream. |
| `get-config mqtt.conf` | After the BLE enable sequence, while no broker URL is stored and no MQTT client is connected, to find the broker (see [Finding the broker](#finding-the-broker)). |
| `tele-mqtt 120` | Over MQTT, about 5 s after a device first appears, also when its telemetry already arrives (another client's lease can expire at any time). The page renews the lease every 60 s while the tab is open, on a broker reconnect, and once more when the tab becomes visible again. When telemetry had been arriving and stops for 15 s, the page requests the lease again, at most once per 15 s, while the lease is running, unanswered, or refused for a transient reason; not for relay-fed devices or after a permanent error such as `disabled`. Devices without the command are not asked again. |
| `sweep-curve` | On connect, after each sweep and its verdict, and on the refresh button. Up to two retries, 4 s timeout each. |
| `sweep` | On the **sweep** button next to the chart, which is disabled while the device reports a sweep in progress. |

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
| MQTT | refused | `tele-mqtt` returned an error (reason in the tooltip), or the device did not reply. The page retries every 60 s after a missing reply or a transient error (`mqtt not connected`, `clock not set`, `low heap`, `out of memory`), not after other errors such as `disabled`. |
| MQTT | off | Broker not connected, or the device has not been seen on the broker. |

Without telemetry, a card shows the log and the sweep curve but no live values. The MQTT tooltip then tells why:

- "log only": the device logs to the broker, but no telemetry has arrived on `pv/tele/<hostname>` and no lease is
  requested yet, for example in the first seconds after the device appears.
- "firmware without tele-mqtt (relay-fed)": the device answered `tele-mqtt` with `Unknown command` or `Command not
  found`. Live values appear only once a relay publishes its `pv/tele/<hostname>` topic.
- A `tele-mqtt` error (MQTT refused): the reason from the device, for example `low heap`, or UDP telemetry running the text wire (`tele.conf binary=0`). The page sets the clock itself when the reason is an unset clock.

## Security

:::caution
With **remember** ticked next to the password field (the default), the broker password is stored in plain text in the browser's
`localStorage` for the page's origin. Unticked (the default), the password stays in memory for the session only, and
a stored password is removed. The URL and user are always stored, as are the device snapshots (no credentials). The dashboard keeps its password under its own
keys, separate from the configuration editor's.

The hosted copy shares its `github.io` origin with every other project page on that host, and script on any of those
pages can read its `localStorage`. Do not tick **remember** on the hosted copy; serve the page yourself if the
password should be remembered.
:::

Broker discovery reads `mqtt.conf` over BLE, so the device sends its MQTT username and password over the BLE link,
and the reply also appears in the card's log. Anyone who can connect to the device's BLE console can read these
credentials the same way.

With `cmd_input=1`, anyone who can publish to the broker can control the converter, and the console line on each card
sends any command. Use broker authentication and ACLs, and give the dashboard's broker user only the topics it needs.
