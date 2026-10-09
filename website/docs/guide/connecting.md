---
title: Connecting
sidebar_position: 4
---

# Connecting

The device serves one text [console](../reference/console.md) over serial, telnet, BLE, and MQTT. The host client
`etc/fugu_console.py` speaks all of them.

## Quick start

Install the client's dependencies, then find a device or connect to one:

```bash
pip install pyserial bleak                         # + paho-mqtt, aioesphomeapi, zeroconf as needed
python3 etc/fugu_console.py                              # scan all transports, print how to connect
python3 etc/fugu_console.py -p /dev/cu.usbmodemXXXX      # interactive console over serial
python3 etc/fugu_console.py --ip <device-ip> -c status   # one command over telnet
```

## Transports

Each transport has its own requirements on the device and its own client flags:

| Transport | Device side | Client flags |
|---|---|---|
| Serial | UART0 at 115200 baud; on ESP32-S3 also the USB-Serial-JTAG port | `-p PORT` (default `$ESPPORT`, else the first matching `/dev/cu.usbmodem*`, `/dev/ttyUSB*`, `/dev/ttyACM*`, …), `-b BAUD` |
| Telnet | Wi-Fi up, `telnet` service running, TCP port 23, one client at a time | `--ip HOST[:PORT]` |
| BLE | `CONFIG_FUGU_WITH_BLE` build and the `ble` service running (off by default) | `--ble [NAME]`, `--name`, `--address`, `--adapter hciN` (Linux) |
| BLE via ESPHome proxy | same as BLE, reached through an ESPHome `bluetooth_proxy` | `--ble-proxy HOST[:PORT]` (default port 6053), `--proxy-password` |
| MQTT | `mqtt.conf::broker_uri` set; `cmd_input=1` to accept commands | `--mqtt BROKER`, `--mqtt-port`, `--mqtt-user`, `--mqtt-pass`, `--mqtt-readonly`, `--name` |

On every transport, the device follows each command's output with `OK: <cmd>` or `ERR: <cmd>`.

### Serial

Serial is the preferred transport because it works before Wi-Fi is configured and shows the complete boot log. To open an interactive console over serial, run:

```bash
python3 etc/fugu_console.py -p /dev/cu.usbmodemXXXX
```

Any terminal program works as well (115200 8N1, commands terminated with `\n` or `\r`). Opening the port may reset
the board, depending on how its USB-UART bridge drives RTS/DTR.

### Telnet

Before you connect over telnet, add a network with `wifi-add <ssid>:<password>` and `restart`. Get the address with
`ip` on the serial console, then connect:

```bash
python3 etc/fugu_console.py --ip <device-ip>
telnet <device-ip>                                 # any telnet client works
```

The server accepts one client at a time and asks for no password. Terminate commands with `\n`. The
`telnet` service is enabled by default. See [Services](../reference/services.md).

:::warning
Telnet is unauthenticated. Only enable it on networks you trust.
:::

### BLE

The BLE console (Nordic UART Service) works without Wi-Fi. The service is off by default because it exposes the
console. Enable it once over serial or telnet:

```
svc on ble
```

This persists `enabled=1` in `ble.conf`. The device advertises under its hostname, which defaults to
`fugu-<target>-<chip id>` and changes with `hostname <name>`. If the hostname doesn't already start with `fugu-`, the
device adds that prefix.

Connect to the first matching device, or filter by name or address:

```bash
python3 etc/fugu_console.py --ble                        # first device whose name contains "fugu"
python3 etc/fugu_console.py --ble --name <name>          # filter by advertised name
python3 etc/fugu_console.py --ble --address <ble-address>
```

[`ble.conf::ble_security`](../reference/config/ble.md) sets the pairing mode: `justworks` (default, encrypted and
bonded, no passkey) or `passkey` (bond + MITM with the static `ble_passkey`).

For firmware updates over BLE, see [OTA over BLE](updating/ota-ble.md).

### BLE through an ESPHome proxy

An ESP32 running ESPHome's `bluetooth_proxy` can bridge the BLE console to the network, which extends range and
avoids the host's Bluetooth stack. The client uses the plaintext ESPHome native API (no noise encryption).

Address the device through the proxy by name or by BLE address:

```bash
python3 etc/fugu_console.py --ble-proxy <proxy-ip> --name <device-name>
python3 etc/fugu_console.py --ble-proxy <proxy-ip> --address <ble-address>
```

`--proxy-password` defaults to `$ESPHOME_API_PASSWORD`.

### MQTT

With a broker configured in [`mqtt.conf`](../reference/config/mqtt.md), the device mirrors all console output to
`pv/log/<hostname>`. With `cmd_input=1` it also executes commands published to `pv/log/<hostname>/cmd`.

Run a command over MQTT, or monitor the log without publishing:

```bash
python3 etc/fugu_console.py --mqtt <broker-ip> --mqtt-user <user> --mqtt-pass <pass> --name <hostname> -c status
python3 etc/fugu_console.py --mqtt <broker-ip> --mqtt-readonly    # passive log monitor, never publishes
```

The client matches `--name` as a substring of the hostname in the log topics. `--mqtt-port`, `--mqtt-user`, and `--mqtt-pass`
default to `$MQTT_PORT` (else 1883), `$MQTT_USER`, and `$MQTT_PASS`. The client also reads these from `etc/mqtt.env`
if present. Shell variables take precedence.

:::warning
Anyone who can publish to the command topic controls the converter, including the half-bridge. Leave `cmd_input`
off unless the broker requires authentication.
:::

## Client modes

The invocation selects the client mode:

| Invocation | Mode |
|---|---|
| no arguments | Scan serial ports, mDNS, BLE, an ESPHome proxy (`$BLE_PROXY`), and MQTT (`$MQTT_HOST`); print how to connect, without connecting |
| transport only | Interactive REPL |
| `-c CMD` (repeatable) | Run the commands over one connection, print the replies, exit |
| `--stdin` | Run newline-separated commands from stdin over one connection; blank lines and `#` comments skipped |
| piped stdin, no mode flag | Same as `--stdin` |
| `--coredump [info\|get\|erase]` | Inspect or pull the panic core dump, see [Debugging](../development/debugging/index.md#coredumps) |
| `--ble --tele` | REPL plus the decoded BLE telemetry stream, see [Telemetry](telemetry/index.md) |

With more than one command, the client heads each reply with `=== <cmd> ===`:

```bash
python3 etc/fugu_console.py -p $ESPPORT <<'EOF'
status
sensor
get-config limits.conf
EOF
```

The client resolves `peek <symbol>` and `sym <pattern>` against the build ELF (`--elf`, default `$FUGU_ELF` or the
newest `build*/fugu-firmware.elf`). See [`peek`](../development/debugging/peek.md).

## Common scenarios

### Find a device

Run the client without arguments to scan all transports:

```bash
python3 etc/fugu_console.py
```

Busy serial ports show no hostname. Before sending state-changing commands, confirm which device you reached: the
telnet banner reads `Welcome to <hostname> (<ip>)`, and `ip` prints the address on any transport.

### Script a sequence

Repeat `-c` to run several commands over one connection:

```bash
python3 etc/fugu_console.py --ip <device-ip> -c "svc list" -c "get-config tele.conf"
```

### Watch a device without touching it

A read-only MQTT client monitors the log and never publishes:

```bash
python3 etc/fugu_console.py --mqtt <broker-ip> --mqtt-readonly --name <hostname>
```

See also: [Console commands](../reference/console.md), [Troubleshooting](troubleshooting.md).
