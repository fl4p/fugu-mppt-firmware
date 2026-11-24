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
| `bat_charge_topic`        |      | string | `""`    | Topic for the BMS remaining charge in Ah (batmon-ha: `<dev>/mosfet_status/capacity_ah`). While fresh (< 120 s), it replaces the integrated `ibat` for the deficit since full, so chargers sharing a BMS release termination together. A step larger than 200 A could cause within at most 120 s (counted as at least 12 s) (a BMS re-seed) is replaced by the `ibat` estimate. Empty = not subscribed |
| `bat_voltage_topic`       |      | string | `""`    | Topic for the BMS pack voltage (batmon-ha: `<dev>/soc/total_voltage`). With `ibat_topic`, it feeds a fit of the output-to-pack voltage drop against output and battery current; telemetry `Rc`/`Rb`. Empty = not subscribed |
| `ibat_lim_topic`          |      | string | `""`    | Topic for battery charge current limit; empty = not subscribed |
| `bat_temp_topic`          |      | string | `""`    | Up to 4 comma-separated topics for pack temperatures from the BMS (batmon-ha: `<dev>/temperatures/1,<dev>/temperatures/2`); drives `bat_temp_*` in charger.conf. Empty = not subscribed |
| `cell_voltages_delta_topic` |    | string | `""`    | Topic for the BMS cell-voltage spread in volts (batmon-ha: `<dev>/cell_voltages/delta`); lets a [top-balancing](../../guide/charging/termination.md#top-balancing) run finish early (`balance_delta` in charger.conf). Empty = not subscribed |
| `ca_file`                 |      | string | `""`    | CA certificate (PEM) that verifies an `mqtts://` broker: a file name in `/littlefs/conf` or an absolute path; ignored for `mqtt://`. Empty = the built-in CA bundle. Needs `CONFIG_FUGU_WITH_HTTPS` |
| `cmd_input`               |      | bool   | 0       | Accept console commands over MQTT (`pv/log/<host>/cmd`) |
| `enabled`                 |      | bool   | 1       | Start this service at boot                              |
| `log_level`               |      | enum   | info    | Verbosity: `error`, `warn` or `info`                    |

## TLS

With `mqtt://`, the broker password, the BMS data and the console (`cmd_input`) cross the network in plaintext. A
build with `CONFIG_FUGU_WITH_HTTPS` connects to `mqtts://host:port` instead. A broker with a certificate from a
private CA needs that CA on the device: copy the PEM file into the config partition (FTP, the
[HTTP endpoint](http.md) or the provisioning image) and name it in `ca_file`:

```
broker_uri=mqtts://<broker>:8883
ca_file=mqtt-ca.pem
```

The server certificate must name the address in `broker_uri`, as a DNS name or an IP address in its subject
alternative names. A missing or unreadable `ca_file` fails the service start rather than falling back to the CA
bundle.

`ca_file` applies to the broker only: an `https://` OTA download is always verified against the CA bundle.

A TLS connection needs considerably more heap than a plain one. See
[TLS heap](../../guide/getting-started/build-options.md#tls-heap) for measured figures and the
`CONFIG_MBEDTLS_DYNAMIC_BUFFER` option.
