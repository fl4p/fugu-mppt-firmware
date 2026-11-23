---
title: mqtt.conf
sidebar_position: 8
---

# mqtt.conf

`mqtt.conf` sets the MQTT broker connection and the BMS topic subscriptions. It has the following keys.

| key                       | unit | type   | default | description                                             |
|---------------------------|------|--------|---------|---------------------------------------------------------|
| `broker_uri`              |      | string | `""`    | MQTT broker URI, e.g. `mqtt://host:port`; empty = no MQTT connection |
| `username`                |      | string | `""`    | Broker login username; empty = none                     |
| `password`                |      | string | `""`    | Broker login password; empty = none                     |
| `cell_voltages_max_topic` |      | string | `""`    | Topic for BMS highest cell voltage; empty = not subscribed |
| `ibat_topic`              |      | string | `""`    | Topic for battery current from BMS; empty = not subscribed |
| `bat_charge_topic`        |      | string | `""`    | Topic for the BMS remaining charge in Ah (batmon-ha: `<dev>/mosfet_status/capacity_ah`). While fresh (< 120 s), it replaces the integrated `ibat` for the deficit since full, so chargers sharing a BMS release termination together. Empty = not subscribed |
| `bat_voltage_topic`       |      | string | `""`    | Topic for the BMS pack voltage (batmon-ha: `<dev>/soc/total_voltage`). With `ibat_topic`, it feeds a fit of the output-to-pack voltage drop against output and battery current; telemetry `Rc`/`Rb`. Empty = not subscribed |
| `ibat_lim_topic`          |      | string | `""`    | Topic for battery charge current limit; empty = not subscribed |
| `bat_temp_topic`          |      | string | `""`    | Up to 4 comma-separated topics for pack temperatures from the BMS (batmon-ha: `<dev>/temperatures/1,<dev>/temperatures/2`); drives `bat_temp_*` in charger.conf. Empty = not subscribed |
| `cell_voltages_delta_topic` |    | string | `""`    | Topic for the BMS cell-voltage spread in volts (batmon-ha: `<dev>/cell_voltages/delta`); lets a [top-balancing](../../guide/charging/termination.md#top-balancing) run finish early (`balance_delta` in charger.conf). Empty = not subscribed |
| `cmd_input`               |      | bool   | 0       | Accept console commands over MQTT (`pv/log/<host>/cmd`) |
| `enabled`                 |      | bool   | 1       | Start this service at boot                              |
| `log_level`               |      | enum   | info    | Verbosity: `error`, `warn` or `info`                    |
