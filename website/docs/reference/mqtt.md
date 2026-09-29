---
title: MQTT topics
sidebar_position: 5
---

# MQTT topics

Topics the firmware publishes and subscribes to when the `mqtt` service is configured with a broker in
[`mqtt.conf`](config/mqtt.md).

## Quick start

```ini title="/littlefs/conf/mqtt.conf"
broker_uri=mqtt://broker.local:1883
username=fugu
password=secret
cmd_input=1
```

```bash
# follow the console log
mosquitto_sub -h broker.local -t 'pv/log/<hostname>'
# send a console command
mosquitto_pub -h broker.local -t 'pv/log/<hostname>/cmd' -m 'status'
```

The client connects asynchronously once Wi-Fi is up. Without `mqtt.conf` or with an empty `broker_uri` the
service stays idle. `svc restart mqtt` reloads the file and re-subscribes.

## Placeholders

| Placeholder    | Value                                                                                           |
|----------------|-------------------------------------------------------------------------------------------------|
| `<hostname>`   | Hostname stored in NVS (`hostname` console command); defaults to `<device-id>`                  |
| `<device-id>`  | `fugu-<target>-<MAC>`, e.g. `fugu-esp32s3-` followed by the eFuse MAC in upper-case hex         |

## Topic overview

| Topic                                              | Dir.      | Payload                   | Condition                          |
|----------------------------------------------------|-----------|---------------------------|------------------------------------|
| `pv/log/<hostname>`                                | publish   | console / log lines       | always while connected             |
| `pv/log/<hostname>/cmd`                            | subscribe | console command           | `cmd_input=1`                      |
| `homeassistant/sensor/<device-id>-power/config`    | publish   | HA discovery JSON, retained | on connect, then every 1000 state updates |
| `homeassistant/sensor/<device-id>-power/state`     | publish   | power in W                | every 3 s, not during a sweep      |
| value of `cell_voltages_max_topic`                 | subscribe | highest cell voltage, V   | key set                            |
| value of `ibat_topic`                                | subscribe | battery current, A        | key set                            |
| value of `ibat_lim_topic`                          | subscribe | charge current limit, A   | key set                            |
| values of `bat_temp_topic` (up to 4)               | subscribe | pack temperature, °C      | key set                            |

All subscriptions use QoS 0. In the Home Assistant topics `<device-id>` is lower-cased.

## Console over MQTT

**`pv/log/<hostname>`** mirrors every console and log line (the same output as on UART and telnet) while the
client is connected. Lines produced by the MQTT client itself are not mirrored.

**`pv/log/<hostname>/cmd`** accepts one console command per message when `cmd_input=1`. The command runs on the
network task, and the reply plus `OK: <cmd>` / `ERR: <cmd>` appear on the log topic. The command set is the
[serial console](console.md).

:::warning
With `cmd_input=1` anyone who can publish to the broker can control the converter (`dc`, `restart`, `ota`, …).
Use broker authentication and ACLs.
:::

`etc/fugu_console.py --mqtt` uses these two topics, see [Host tools](host-tools.md).

## Home Assistant discovery

The firmware announces one sensor through [MQTT discovery](https://www.home-assistant.io/integrations/sensor.mqtt/)
(`src/tele/home_assistant.cpp`). The config payload uses abbreviated keys with `~` as the topic base:

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

The state payload is the smoothed converter power as plain text, 3 decimals (0 decimals above 999 W). No state is
published while a global sweep runs, so the entity can expire (`exp_aft`) during a sweep longer than 30 s.

## BMS inputs

The charger subscribes to BMS values when the corresponding `mqtt.conf` key names a topic
(`src/charger.h`, `beginMqtt`). Each payload is a plain decimal number; non-finite values are logged and ignored.

| Key                       | Unit | Effect                                                                                    |
|---------------------------|------|-------------------------------------------------------------------------------------------|
| `cell_voltages_max_topic` | V    | Highest cell voltage, used for termination instead of pack voltage / N. Expires after 180 s, then `Vbat_fallback` applies |
| `ibat_topic`              | A    | Battery current (`Ibat = Iout − Iload`), smoothed; used for termination and coulomb counting. Expires after 180 s |
| `ibat_lim_topic`          | A    | Replaces the charge current limit (`charger.conf::ibat_max`); negative values are ignored |
| `bat_temp_topic`          | °C   | Comma-separated list, up to 4 topics. Drives the `bat_temp_*` policy in `charger.conf`; each sensor expires after 1 h |

When `cell_voltages_max_topic` is set, the converter waits briefly for the first BMS frame before its start-up
sweep, see [MPP Tracker](../internals/mppt-tracker.md#when-the-tracker-does-not-sweep) and
[Charge Termination](../guide/charging/termination.md).

```ini title="mqtt.conf with a BMS publishing per-value topics"
cell_voltages_max_topic=bms/cell_voltages/max
ibat_topic=bms/current
bat_temp_topic=bms/temperatures/1,bms/temperatures/2
```
