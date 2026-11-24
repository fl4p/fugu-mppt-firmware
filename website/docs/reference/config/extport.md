---
title: extport.conf
sidebar_position: 14.5
---

# extport.conf

`extport.conf` configures the external comms port (service `extport`). The service exists only in builds with
`CONFIG_FUGU_WITH_EXTPORT=y` (off by default). `enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common
service keys. For details, see [Service Architecture](../services.md#per-service-conf-file).

| key         | unit | type | default | description |
|-------------|------|------|---------|-------------|
| `mode`      |      | enum | off     | `off` or `vedirect`. Any other value fails the service start |
| `tx_pin`    |      | int  | —       | GPIO of the port's TX line. Required; no pin is driven while it is unset |
| `rx_pin`    |      | int  | -1      | GPIO of the port's RX line, used as the wrong-adapter guard (below). -1 = no guard |
| `pid`       |      | int  | 0xA056  | Product ID sent in the `PID` field. The default is the SmartSolar MPPT 100/30 |
| `enabled`   |      | bool | 0       | Start this service at boot |
| `log_level` |      | enum | info    | Verbosity: `error`, `warn`, or `info` |

All keys are read when the service starts (`svc rs extport`).

## VE.Direct output

With `mode=vedirect` the port sends one VE.Direct text-mode block per second on UART1 at 19200 baud, 8N1, as
specified in Victron's VE.Direct Protocol 3.34. Each block ends in a `Checksum` field that makes the sum of all its
bytes 0 mod 256. HEX mode is not implemented.

| field | value |
|---|---|
| `PID` | `pid` key |
| `FW` | firmware version, e.g. `210` for 2.1.0 |
| `SER#` | `FG` plus the last four bytes of the MAC address |
| `V`, `I` | output (battery) voltage in mV and current in mA, EWM-filtered; `I` > 0 while charging |
| `VPV`, `PPV` | input (panel) voltage in mV and power in W |
| `CS` | 0 off, 3 bulk, 4 absorption (CV control or CV hold), 5 float (charge terminated), 252 manual duty (`dc`) |
| `MPPT` | 0 off, 1 voltage or current limited, 2 tracking (MPPT or sweep) |
| `OR` | off reason while the converter is stopped: `0x00000001` input too low, `0x00000010` protection (backoff, temperature, fault inhibits), else 0 |
| `ERR` | always 0 |
| `H19` | total yield, 0.01 kWh |
| `H20`, `H21` | yield today (0.01 kWh) and maximum power today (W) |
| `H22`, `H23` | the same for the most recent stored day |
| `HSDS` | number of stored days, modulo 365 |

No block is sent until the sensor filters hold finite values.

## Wrong-adapter guard

A VE.Direct peer either stays silent or sends HEX frames (`:`, hex digits, newline). With `rx_pin` set, the first
received byte outside that set turns TX off and leaves it off until `svc rs extport`. A CAN transceiver echoing TX
trips the guard on the first block. `svc list` shows `refused: rx 0x..` with the offending byte.

The guard cannot detect an adapter that stays silent. Connect only VE.Direct devices or cables to this port.
