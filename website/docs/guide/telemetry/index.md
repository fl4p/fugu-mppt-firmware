---
title: Telemetry & Home Assistant
sidebar_position: 1
---

*this document is an LLM generated placeholder*

# Telemetry & Home Assistant

The firmware publishes live data four ways: MQTT with Home Assistant discovery, InfluxDB line protocol over UDP, a
BLE notify stream, and connectionless BLE advertisements.

## Quick start

InfluxDB over Wi-Fi:

```
set-config tele.conf influxdb_host <influxdb-ip>
svc on tele
restart
```

Changing `influxdb_host` takes effect after a reboot.

Home Assistant over MQTT:

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
| BLE stream | `CONFIG_FUGU_WITH_BLE_TELE`, `ble` service on, a connected client | NUS notify characteristic | `etc/influx_binary_proxy.py --ble`, `etc/fugu_console.py --ble --tele` |
| BLE advertising | `CONFIG_FUGU_WITH_BLE_ADV`, `ble` service on | manufacturer data in advertisements | `etc/influx_binary_proxy.py --adv` |

The BLE paths need no Wi-Fi. See [Build Options](../getting-started/build-options.md) for the Kconfig flags.

## MQTT and Home Assistant

When the MQTT service connects to the broker, the device announces a **Power** sensor through Home Assistant MQTT
discovery:

| Topic | Content |
|---|---|
| `homeassistant/sensor/<device-id>-power/config` | discovery payload (retained), `device_class: power`, unit W |
| `homeassistant/sensor/<device-id>-power/state` | converter power in W (physical current × voltage sensor), about every 3 s |

`<device-id>` is `fugu-<target>-<chip id>`. No state is published during an MPPT sweep, and the sensor expires in
Home Assistant after 30 s without updates. The retained discovery message is also re-sent every 1000 state updates.

The MQTT service also mirrors the console log to `pv/log/<hostname>` and subscribes to BMS topics, see
[BMS integration](../charging/bms-integration.md) and [Connecting](../connecting.md#mqtt). All topics are listed in
[MQTT topics](../../reference/mqtt.md).

## InfluxDB over UDP

The `tele` service sends InfluxDB line protocol datagrams to `tele.conf::influxdb_host` on UDP port 8086.

```ini title="tele.conf"
enabled=1
influxdb_host=<influxdb-ip>   # an IP address, not a hostname
binary=0
```

- Measurement `mppt`, tag `device=<hostname>`, fields such as `Ui`, `Uo`, `I`, `P`, `E`, `E_today`, `pwm_duty`,
  `mppt_state`, `mcu_temp`, `ntc_temp` and `lag`. The full list is in [Telemetry fields](../../reference/telemetry-fields.md).
- Points carry wall-clock timestamps, so sending starts after the clock is set by SNTP.
- Datagrams are batched up to one TCP MSS; small batches wait for more points.
- The service is **off by default**. `svc on tele` enables and persists it; see [Services](../../reference/services.md).

The receiver is an InfluxDB 1.x UDP input on port 8086, or any relay that accepts line protocol over UDP.

### Binary wire

`binary=1` replaces text line protocol with a symbol-table encoding (`src/tele/sym_line_protocol.h`), always
tamp-compressed, several times smaller. A plain InfluxDB cannot read it; put `etc/influx_binary_proxy.py` in between:

```bash
python3 etc/influx_binary_proxy.py --listen 0.0.0.0:8086                        # decode and print
python3 etc/influx_binary_proxy.py --listen 0.0.0.0:8086 --forward-udp <relay-host>:<port>
python3 etc/influx_binary_proxy.py --listen 0.0.0.0:8086 \
    --influx http://<influxdb-host>:8086 --db <db> --user <user> --password <pass>
```

## BLE telemetry stream

With `CONFIG_FUGU_WITH_BLE_TELE`, the same binary wire is streamed over a notify characteristic of the BLE console
service. The client enables it after connecting:

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
running the text wire (set `binary=1`, then `svc rs tele`). The stream stops on disconnect; the next client must
enable it again.

## BLE advertising

With `CONFIG_FUGU_WITH_BLE_ADV`, the device broadcasts a compact telemetry record in the **manufacturer data** of its
advertisements: a custom 17-byte record (company ID 0xFFFF, magic byte 0xF7) with Vin, Vout, current, power, MCU and
NTC temperature, duty, loop lag and MPPT state. Any number of observers can listen without connecting, and it keeps
broadcasting while a client holds the console connection. It is sent while the `ble` service runs.

```ini title="tele.conf"
adv_ms=500      # refresh interval, min 100, 0 = off
```

`svc rs ble` re-reads the interval. Decode with:

```bash
python3 etc/influx_binary_proxy.py --adv --verbose
```

The broadcast is unencrypted and lossy; observers stamp the time on receipt.

:::note
[BTHome advertising](bthome.md) is a design proposal and not implemented. The custom manufacturer-data record above
is what the firmware sends today.
:::

## Common scenarios

| Goal | Setup |
|---|---|
| Power in a Home Assistant dashboard | MQTT broker in `mqtt.conf` |
| Long-term history over Wi-Fi | `influxdb_host` + `svc on tele`, text wire straight into InfluxDB |
| Low bandwidth over a weak link | `binary=1` + `influx_binary_proxy.py` near the database |
| No Wi-Fi at the site | BLE advertising + `influx_binary_proxy.py --adv` on a host within BLE range |
| Debug one device from a laptop | `fugu_console.py --ble --tele` |

See also: [`tele.conf`](../../reference/config/tele.md), [`mqtt.conf`](../../reference/config/mqtt.md),
[Host tools](../../reference/host-tools.md).
