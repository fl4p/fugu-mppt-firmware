---
title: Telemetry & Home Assistant
sidebar_position: 1
---

# Telemetry & Home Assistant

The firmware publishes live data four ways: MQTT with Home Assistant discovery, InfluxDB line protocol over UDP, a
BLE notify stream, and connectionless BLE advertisements. A build option adds a fifth, a binary telemetry stream over
MQTT.

## Quick start

To send data to InfluxDB over Wi-Fi, set the host and enable the `tele` service:

```
set-config tele.conf influxdb_host <influxdb-ip>
svc on tele
```

The service reads `influxdb_host` each time it starts; after changing the host of a running service, apply it with
`svc rs tele`.

To feed Home Assistant over MQTT, set the broker and its credentials, then restart the `mqtt` service:

```
set-config mqtt.conf broker_uri mqtt://<broker-ip>:1883
set-config mqtt.conf username <user>
set-config mqtt.conf password <pass>
svc rs mqtt
```

## Paths

| Path | Needs | Transport | Receiver |
|---|---|---|---|
| MQTT + Home Assistant | Wi-Fi, `mqtt.conf::broker_uri` | MQTT | Home Assistant MQTT discovery |
| InfluxDB | Wi-Fi, `tele.conf::influxdb_host`, `tele` service on | UDP to port 8086 | InfluxDB UDP listener, or `etc/influx_binary_proxy.py` |
| BLE stream | `CONFIG_FUGU_WITH_BLE_TELE`, `ble` service on, a connected client | notify characteristic (TELE) of the BLE console service | `etc/influx_binary_proxy.py --ble`, `etc/fugu_console.py --ble --tele` |
| BLE advertising | `CONFIG_FUGU_WITH_BLE_ADV`, `ble` service on | manufacturer data in advertisements | `etc/influx_binary_proxy.py --adv` |
| MQTT stream (build option) | `CONFIG_FUGU_WITH_TELE_MQTT` (off by default), Wi-Fi, a `tele-mqtt <0..3600>` lease | binary wire on `pv/tele/<hostname>` | [Dashboard](../../reference/dashboard.md), see [Telemetry stream](../../reference/mqtt.md#telemetry-stream) |

The BLE paths need no Wi-Fi. See [Build Options](../getting-started/build-options.md) for the Kconfig flags.

## MQTT and Home Assistant

When the MQTT service connects to the broker, the device announces a Power sensor through Home Assistant MQTT
discovery:

| Topic | Content |
|---|---|
| `homeassistant/sensor/<device-id>-power/config` | discovery payload (retained), `device_class: power`, unit W |
| `homeassistant/sensor/<device-id>-power/state` | converter power in W (physical current × voltage sensor), about every 3 s |

`<device-id>` is `fugu-<target>-<chip id>`. The device publishes no state during an MPPT sweep, and the sensor
expires in Home Assistant after 30 s without updates. The device also re-sends the retained discovery message every
1000 state updates.

The MQTT service also mirrors the console log to `pv/log/<hostname>` and subscribes to BMS topics. For details, see
[BMS integration](../charging/bms-integration.md) and [Connecting](../connecting.md#mqtt). All topics are listed in
[MQTT topics](../../reference/mqtt.md).

## InfluxDB over UDP

The `tele` service sends InfluxDB line protocol datagrams to `tele.conf::influxdb_host` on UDP port 8086. A
typical `tele.conf` looks like this:

```ini title="tele.conf"
enabled=1
influxdb_host=<influxdb-ip>   # an IP address, not a hostname
binary=0
```

- Measurement `mppt`, tag `device=<hostname>`, fields such as `Ui`, `Uo`, `I`, `P`, `E`, `E_today`, `pwm_duty`,
  `mppt_state`, `mcu_temp`, `ntc_temp`, and `lag`. The full list is in [Telemetry fields](../../reference/telemetry-fields.md).
- Points carry wall-clock timestamps, so the service starts sending after SNTP sets the clock.
- The service batches datagrams up to one TCP MSS. Small batches wait for more points.
- The service is off by default. `svc on tele` enables and persists it. See [Services](../../reference/services.md).

The receiver is an InfluxDB 1.x UDP input on port 8086, or any relay that accepts line protocol over UDP.

### Binary wire

`binary=1` replaces text line protocol with a symbol-table encoding (see [Telemetry fields](../../reference/telemetry-fields.md#transports)), always
tamp-compressed, several times smaller. It needs a build with `CONFIG_FUGU_WITH_TELE_BINARY` (`CONFIG_FUGU_WITH_BLE_TELE`
turns it on); other builds log a warning and keep the text wire. A plain InfluxDB can't read it, so put
`etc/influx_binary_proxy.py` in between. The proxy can print the decoded points, forward them to a UDP relay, or write them to InfluxDB over HTTP:

```bash
python3 etc/influx_binary_proxy.py --listen 0.0.0.0:8086                        # decode and print
python3 etc/influx_binary_proxy.py --listen 0.0.0.0:8086 --forward-udp <relay-host>:<port>
python3 etc/influx_binary_proxy.py --listen 0.0.0.0:8086 \
    --influx http://<influxdb-host>:8086 --db <db> --user <user> --password <pass>
```

## BLE telemetry stream

With `CONFIG_FUGU_WITH_BLE_TELE`, the same binary wire is streamed over a notify characteristic of the BLE console
service. After connecting, the client enables the stream with these commands:

```
set-time <epoch_ms>     # there is no NTP without Wi-Fi
tele-ble 1
```

The host tools send both commands themselves:

```bash
python3 etc/influx_binary_proxy.py --ble <device-name>          # decode + print, or forward with --influx / --forward-udp
python3 etc/influx_binary_proxy.py --ble-all                    # every Fugu device in range
python3 etc/fugu_console.py --ble --tele                        # console plus decoded points as "tele| …" lines
```

`tele-ble 1` is refused when the clock is not set, no client is connected, `tele.conf::ble=0`, or the UDP service is
running the text wire. In the last case, set `binary=1`, then run `svc rs tele`.

The stream stops on disconnect, and the next client must enable it again.

## BLE advertising

With `CONFIG_FUGU_WITH_BLE_ADV`, the device broadcasts a compact telemetry record in the manufacturer data of its
advertisements while the `ble` service runs. The record is a custom 17-byte little-endian layout under company ID
`0xFFFF`:

```
u8 magic=0xF7, u8 seq, f16 Ui Uo I P, i8 mcu_temp ntc_temp, u16 pwm_duty, u16 lag_us, u8 state
```

- `f16` is an IEEE half-precision float. `I` and `P` come from the physically sensed current. A missing temperature
  reads `-128`.
- The low nibble of `state` is the control mode (`0` none, `1` CV, `2` CC, `3` CP, `4` MPPT, `5` sweep), the high
  nibble the active CV limiter.
- Every 8th refresh carries the device name instead of the record. While Improv Wi-Fi provisioning is offered, every
  other refresh carries the Improv data.

Any number of observers can listen without connecting. The device keeps broadcasting while a client holds the console
connection. The broadcast is unencrypted and lossy, and observers stamp the time on receipt.

One key in `tele.conf` sets the refresh interval:

```ini title="tele.conf"
adv_ms=500      # refresh interval, min 100, 0 = off
```

`svc rs ble` re-reads the interval. To decode the broadcasts, run the proxy in advertising mode:

```bash
python3 etc/influx_binary_proxy.py --adv --verbose
```

:::note
The firmware does not send BTHome, so Home Assistant's BTHome integration does not decode this record. A design
for BTHome advertising is in
[plans/bthome-advertising.md](https://github.com/fl4p/fugu-mppt-firmware/blob/main/plans/bthome-advertising.md).
:::

## Common scenarios

| Goal | Setup |
|---|---|
| Power in a Home Assistant dashboard | MQTT broker in `mqtt.conf` |
| Long-term history over Wi-Fi | `influxdb_host` + `svc on tele`, text wire straight into InfluxDB |
| Low bandwidth over a weak link | `binary=1` + `influx_binary_proxy.py` near the database |
| No Wi-Fi at the site | BLE advertising + `influx_binary_proxy.py --adv` on a host within BLE range |
| Debug one device from a laptop | `fugu_console.py --ble --tele` |
| Live view in a browser, no server | [Dashboard](pathname:///tools/dashboard.html) over BLE or MQTT ([docs](../../reference/dashboard.md)) |

See also: [`tele.conf`](../../reference/config/tele.md), [`mqtt.conf`](../../reference/config/mqtt.md),
[Host tools](../../reference/host-tools.md).
