---
title: MQTT topics
sidebar_position: 5
---

# MQTT topics

This page lists the topics the firmware publishes and subscribes to when the `mqtt` service has a broker configured
in [`mqtt.conf`](config/mqtt.md).

## Quick start

A minimal `mqtt.conf` sets the broker, its credentials, and command input:

```ini title="/littlefs/conf/mqtt.conf"
broker_uri=mqtt://broker.local:1883
username=fugu
password=secret
cmd_input=1
```

With that file in place, you can follow the console log and send commands with the Mosquitto clients:

```bash
# follow the console log
mosquitto_sub -h broker.local -t 'pv/log/<hostname>'
# send a console command
mosquitto_pub -h broker.local -t 'pv/log/<hostname>/cmd' -m 'status'
```

The client connects asynchronously once Wi-Fi is up. Without `mqtt.conf`, or with an empty `broker_uri`, the
service stays idle. `svc restart mqtt` reloads the file and re-subscribes.

## Placeholders

The topic names on this page use these placeholders:

| Placeholder    | Value                                                                                           |
|----------------|-------------------------------------------------------------------------------------------------|
| `<hostname>`   | Hostname stored in NVS (`hostname` console command); defaults to `<device-id>`                  |
| `<device-id>`  | `fugu-<target>-<MAC>`, e.g. `fugu-esp32s3-` followed by the eFuse MAC in upper-case hex         |

## Topic overview

The firmware uses the following topics:

| Topic                                              | Dir.      | Payload                   | Condition                          |
|----------------------------------------------------|-----------|---------------------------|------------------------------------|
| `pv/log/<hostname>`                                | publish   | console / log lines       | always while connected             |
| `pv/log/<hostname>/cmd`                            | subscribe | console command           | `cmd_input=1`                      |
| `pv/dev/<hostname>`                                | publish   | `online` / `offline`, retained, QoS 1 | on connect; `offline` on a clean stop and as last will |
| `pv/alert/<hostname>/<source>`                     | publish   | alert JSON, retained, QoS 1 | on a state change of the source    |
| `pv/tele/<hostname>`                               | publish   | binary telemetry, QoS 0   | while a `tele-mqtt` lease runs     |
| `homeassistant/sensor/<device-id>-power/config`    | publish   | HA discovery JSON, retained | on connect, then every 1000 state updates |
| `homeassistant/sensor/<device-id>-power/state`     | publish   | power in W                | every 3 s, not during a sweep      |
| value of `cell_voltages_max_topic`                 | subscribe | highest cell voltage, V   | key set                            |
| value of `ibat_topic`                                | subscribe | battery current, A        | key set                            |
| value of `ibat_lim_topic`                          | subscribe | charge current limit, A   | key set                            |
| values of `bat_temp_topic` (up to 4)               | subscribe | pack temperature, °C      | key set                            |

All subscriptions use QoS 0. In the Home Assistant topics, `<device-id>` is lower-cased.

## Presence

`pv/dev/<hostname>` holds the device's presence as a retained message. The firmware publishes `online` after each
connect and `offline` before a clean disconnect (for example `svc off mqtt`). The same topic is registered as
the last will with payload `offline`, so the broker publishes it when the connection drops without a disconnect
(power loss, reset, network loss) once the keep-alive times out. A subscriber to `pv/dev/+` gets every known
device at once, including quiet ones.

## Alerts

Each alert source has its own retained topic `pv/alert/<hostname>/<source>`, so a subscriber sees the
current state of every source after its own restart, and one source clearing doesn't hide another.
The payload is

```json
{"lvl":"warn","msg":"Pw 2.6W dR 4.3mOhm (fast)","t":1794490551}
```

with `lvl` one of `ok`, `warn`, `derate`, `trip` and `unverified`, a short human-readable `msg`, and
`t` the device's Unix time (seconds since boot until the clock is set). `ok` clears the alert;
`unverified` means the source has no data to judge, which is not `ok`. The
firmware publishes on a change of the level and once after boot, logs the same line, and republishes
every source after an MQTT reconnect. A state change while MQTT is down is published once it is up.

| Source | Levels | Meaning |
|--------|--------|---------|
| `wire` | `ok`, `warn`, `derate`, `trip`, `unverified` | Output wire-fault protection, see [limits.conf](config/limits.md#output-wire-fault-protection) |

A Home Assistant automation on `pv/alert/+/+` can notify on any level other than `ok`.

## Console over MQTT

`pv/log/<hostname>` mirrors every console and log line (the same output as on UART and telnet) while the
client is connected. The firmware doesn't mirror lines that the MQTT client itself logs.

When `cmd_input=1`, `pv/log/<hostname>/cmd` accepts one console command per message. The command runs on the
network task, and the reply plus `OK: <cmd>` or `ERR: <cmd>` appear on the log topic. The command set is the same
as on the [serial console](console.md).

:::warning
With `cmd_input=1` anyone who can publish to the broker can control the converter (`dc`, `restart`, `ota`, …).
Use broker authentication and ACLs.
:::

`etc/fugu_console.py --mqtt` uses these two topics. See [Host tools](host-tools.md).

## Telemetry stream

With the build option `CONFIG_FUGU_WITH_TELE_MQTT`, the `tele-mqtt <seconds>` console command leases a
telemetry stream on `pv/tele/<hostname>` for up to 3600 s. Each message is `<cid><payload>`, byte-identical to one
UDP telemetry datagram of the binary wire: `cid` is `0` (uncompressed), and the payload is a sequence of
length-prefixed `sym_line_protocol` frames (symbol table and data points). The firmware batches points for up to
0.5 s or 1 KB per message. A repeated `tele-mqtt <seconds>` renews the lease to that many seconds from now, `tele-mqtt 0` ends it,
and the stream also ends when the `mqtt` service stops.

The lease needs a set clock, a connected broker, the binary wire if the UDP telemetry service is running
(`tele.conf binary=1`), and 16 KB of free heap. The stream switches the telemetry wire to binary while it runs.
A message starts with the full symbol table after the lease starts and after every reconnect, so a subscriber can
decode from its first message. `etc/fugu/teledec.py` (`decode_payload`) decodes a message.

A host-side relay that republishes the UDP telemetry wire may publish the same topic for devices built without
the option.

## Home Assistant discovery

The firmware announces one sensor through [MQTT discovery](https://www.home-assistant.io/integrations/sensor.mqtt/)
(`src/tele/home_assistant.cpp`). The config payload uses abbreviated keys, with `~` as the topic base:

| Key            | Value                                   |
|----------------|-----------------------------------------|
| `name`         | `Power`                                 |
| `uniq_id`      | `<device-id>-power`                     |
| `dev_cla`      | `power`                                 |
| `unit_of_meas` | `W`                                     |
| `stat_cla`     | `measurement`                           |
| `sug_dsp_prc`  | `1`                                     |
| `exp_aft`      | `30` (s)                                |
| `ic`           | `mdi:flash`                             |
| `stat_t`       | `~/state`                               |
| `dev`          | `{"name":"<hostname>","ids":["<device-id>"]}` |

The state payload is the smoothed converter power as plain text, with 3 decimals (0 decimals above 999 W).

The firmware publishes no state while a global sweep runs, so the entity can expire (`exp_aft`) during a sweep
longer than 30 s.

## BMS inputs

The charger subscribes to BMS values when the corresponding `mqtt.conf` key names a topic
(`src/charger.h`, `beginMqtt`). Each payload is a plain decimal number. The charger logs and ignores non-finite
values. The keys have these effects:

| Key                       | Unit | Effect                                                                                    |
|---------------------------|------|-------------------------------------------------------------------------------------------|
| `cell_voltages_max_topic` | V    | Highest cell voltage, used for termination instead of pack voltage / N. Expires after 180 s, then `Vbat_fallback` applies |
| `ibat_topic`              | A    | Battery current (`Ibat = Iout − Iload`), smoothed; used for termination and coulomb counting. Expires after 180 s |
| `ibat_lim_topic`          | A    | Replaces the charge current limit (`charger.conf::ibat_max`); negative values are ignored |
| `bat_temp_topic`          | °C   | Comma-separated list, up to 4 topics. Drives the `bat_temp_*` policy in `charger.conf`; each sensor expires after 1 h |

When `cell_voltages_max_topic` is set, a converter restarting after a stop waits up to 12 s for the BMS termination
decision before it sweeps. The first sweep after boot starts without this wait, under the
`charger.conf::vout_max_fallback` cap; when BMS data lifts the cap during a no-MPP backoff, a sweep starts at once. See
[MPP Tracker](../internals/mppt-tracker.md#when-the-tracker-does-not-sweep) and
[Charge Termination](../guide/charging/termination.md).

This example `mqtt.conf` subscribes to a BMS that publishes each value on its own topic:

```ini title="mqtt.conf with a BMS publishing per-value topics"
cell_voltages_max_topic=bms/cell_voltages/max
ibat_topic=bms/current
bat_temp_topic=bms/temperatures/1,bms/temperatures/2
```
