---
title: Connecting
sidebar_position: 4
---

*this document is an LLM generated placeholder*

# Connecting

The device serves one text [console](../reference/console.md) over serial, telnet, BLE and MQTT; the host client
`etc/fugu_console.py` speaks all of them.

## Quick start

```bash
pip install pyserial bleak                         # + paho-mqtt, aioesphomeapi, zeroconf as needed
python3 etc/fugu_console.py                              # scan all transports, print how to connect
python3 etc/fugu_console.py -p /dev/cu.usbmodemXXXX      # interactive console over serial
python3 etc/fugu_console.py --ip <device-ip> -c status   # one command over telnet
```

## Transports

| Transport | Device side | Client flags |
|---|---|---|
| Serial | UART0 at 115200 baud; on ESP32-S3 also the USB-Serial-JTAG port | `-p PORT` (default `$ESPPORT`, else the first matching `/dev/cu.usbmodem*`, `/dev/ttyUSB*`, `/dev/ttyACM*`, …), `-b BAUD` |
| Telnet | Wi-Fi up, `telnet` service running, TCP port 23, one client at a time | `--ip HOST[:PORT]` |
| BLE | `CONFIG_FUGU_WITH_BLE` build and the `ble` service running (off by default) | `--ble [NAME]`, `--name`, `--address`, `--adapter hciN` (Linux) |
| BLE via ESPHome proxy | same as BLE, reached through an ESPHome `bluetooth_proxy` | `--ble-proxy HOST[:PORT]` (default port 6053), `--proxy-password` |
| MQTT | `mqtt.conf::broker_uri` set; `cmd_input=1` to accept commands | `--mqtt BROKER`, `--mqtt-port`, `--mqtt-user`, `--mqtt-pass`, `--mqtt-readonly`, `--name` |

Every command is answered with `OK: <cmd>` or `ERR: <cmd>` after its output, on every transport.

### Serial

The preferred transport: it works before Wi-Fi is configured and shows the complete boot log.

```bash
python3 etc/fugu_console.py -p /dev/cu.usbmodemXXXX
```

Any terminal program works as well (115200 8N1, commands terminated with `\n` or `\r`). Opening the port may reset
the board, depending on how its USB-UART bridge drives RTS/DTR.

### Telnet

After `wifi-add <ssid>:<password>` and `restart`, get the address with `ip` on the serial console, then:

```bash
python3 etc/fugu_console.py --ip <device-ip>
telnet <device-ip>                                 # any telnet client works
```

No password is required, and the server accepts one client at a time. Terminate commands with `\n`. The
`telnet` service is enabled by default; see [Services](../reference/services.md).

:::warning
Telnet is unauthenticated. Only enable it on networks you trust.
:::

### BLE

The BLE console (Nordic UART Service) works without Wi-Fi. The service is **off by default** because it exposes the
console. Enable it once over serial or telnet:

```
svc on ble
```

This persists `enabled=1` in `ble.conf`. The device advertises under its hostname (default
`fugu-<target>-<chip id>`, change it with `hostname <name>`), prefixed with `fugu-` if it does not already start
with it.

```bash
python3 etc/fugu_console.py --ble                        # first device whose name contains "fugu"
python3 etc/fugu_console.py --ble <name>                 # filter by advertised name
python3 etc/fugu_console.py --ble --address <ble-address>
```

Pairing is set by [`ble.conf::ble_security`](../reference/config/ble.md): `justworks` (default, encrypted and
bonded, no passkey) or `passkey` (bond + MITM with the static `ble_passkey`).
Firmware updates over BLE are covered in [OTA over BLE](updating/ota-ble.md).

### BLE through an ESPHome proxy

An ESP32 running ESPHome's `bluetooth_proxy` can bridge the BLE console to the network, which extends range and
avoids the host's Bluetooth stack. The client uses the plaintext ESPHome native API (no noise encryption).

```bash
python3 etc/fugu_console.py --ble-proxy <proxy-ip> --name <device-name>
python3 etc/fugu_console.py --ble-proxy <proxy-ip> --address <ble-address>
```

`--proxy-password` defaults to `$ESPHOME_API_PASSWORD`.

### MQTT

With a broker configured in [`mqtt.conf`](../reference/config/mqtt.md), the device mirrors all console output to
`pv/log/<hostname>`. With `cmd_input=1` it also executes commands published to `pv/log/<hostname>/cmd`.

```bash
python3 etc/fugu_console.py --mqtt <broker-ip> --mqtt-user <user> --mqtt-pass <pass> --name <hostname> -c status
python3 etc/fugu_console.py --mqtt <broker-ip> --mqtt-readonly    # passive log monitor, never publishes
```

`--name` is matched as a substring of the hostname in the log topics. `--mqtt-port`, `--mqtt-user` and `--mqtt-pass`
default to `$MQTT_PORT` (else 1883), `$MQTT_USER` and `$MQTT_PASS`; the client also reads these from `etc/mqtt.env`
if present, with shell variables taking precedence.

:::warning
Anyone who can publish to the command topic controls the converter, including the half-bridge. Leave `cmd_input`
off unless the broker requires authentication.
:::

## Client modes

| Invocation | Mode |
|---|---|
| no arguments | Scan serial ports, mDNS, BLE, an ESPHome proxy (`$BLE_PROXY`) and MQTT (`$MQTT_HOST`); print how to connect, without connecting |
| transport only | Interactive REPL |
| `-c CMD` (repeatable) | Run the commands over one connection, print the replies, exit |
| `--stdin` | Run newline-separated commands from stdin over one connection; blank lines and `#` comments skipped |
| piped stdin, no mode flag | Same as `--stdin` |
| `--coredump [info\|get\|erase]` | Inspect or pull the panic core dump, see [Debugging](../development/debugging/index.md#coredumps) |
| `--ble --tele` | REPL plus the decoded BLE telemetry stream, see [Telemetry](telemetry/index.md) |

With more than one command, each reply is headed by `=== <cmd> ===`:

```bash
python3 etc/fugu_console.py -p $ESPPORT <<'EOF'
status
sensor
get-config limits.conf
EOF
```

The client resolves `peek <symbol>` and `sym <pattern>` against the build ELF (`--elf`, default `$FUGU_ELF` or the
newest `build*/fugu-firmware.elf`), see [`peek`](../development/debugging/peek.md).

## Common scenarios

### Find a device

```bash
python3 etc/fugu_console.py
```

Busy serial ports show no hostname. Before sending state-changing commands, confirm which device you reached: the
telnet banner reads `Welcome to <hostname> (<ip>)`, and `ip` prints the address on any transport.

### Script a sequence

```bash
python3 etc/fugu_console.py --ip <device-ip> -c "svc list" -c "get-config tele.conf"
```

### Watch a device without touching it

```bash
python3 etc/fugu_console.py --mqtt <broker-ip> --mqtt-readonly --name <hostname>
```

See also: [Console commands](../reference/console.md), [Troubleshooting](troubleshooting.md).
