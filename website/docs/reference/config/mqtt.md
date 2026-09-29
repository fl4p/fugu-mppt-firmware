---
title: mqtt.conf
sidebar_position: 8
---

# mqtt.conf

Broker + BMS coupling.

| key                       | unit | type   | default | description                                             |
|---------------------------|------|--------|---------|---------------------------------------------------------|
| `broker_uri`              |      | string | —       | MQTT broker URI, e.g. `mqtt://host:port`                |
| `username`                |      | string | —       | Broker login username                                   |
| `password`                |      | string | —       | Broker login password                                   |
| `cell_voltages_max_topic` |      | string | —       | Topic for BMS highest cell voltage                      |
| `ibat_topic`              |      | string | —       | Topic for battery current from BMS                      |
| `ibat_lim_topic`          |      | string | —       | Topic for battery charge current limit                  |
| `bat_temp_topic`          |      | string | —       | Up to 4 comma-separated topics for pack temperatures from the BMS (batmon-ha: `<dev>/temperatures/1,<dev>/temperatures/2`); drives `bat_temp_*` in charger.conf |
| `cmd_input`               |      | bool   | 0       | Accept console commands over MQTT (`pv/log/<host>/cmd`) |
| `enabled`                 |      | bool   | 1       | Start this service at boot                              |
| `log_level`               |      | enum   | info    | Verbosity: `error`, `warn` or `info`                    |
