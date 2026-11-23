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

## Wi-Fi setup

A device joins the networks listed in [`wifi.conf`](../reference/config/wifi.md). There are two ways to add one.

### Console

On any transport, add the network and restart:

```
wifi-add <ssid>:<password>
restart
```

### Improv over BLE

Builds with `CONFIG_FUGU_WITH_IMPROV` (on by default) offer [Improv Wi-Fi](https://www.improv-wifi.com/) next to the
BLE console, so a phone can send the credentials. It needs the `ble` service running (`svc on ble`, see
[BLE](#ble)). A negative `wifi.conf::improv_offline_s` (e.g. `-1`) turns Improv off: the device then creates no
Improv service and saves its heap. The setting is read when BLE starts, so restart after changing it.

Two clients support it:

- [improv-wifi.com](https://www.improv-wifi.com/) in Chrome on Android or desktop.
- Home Assistant's Improv via BLE integration, which discovers the device on its own.

The device stores the network with `wifi-add` semantics and joins it. If the join fails within about 25 s, the
client shows an error and you can retry.

Improv has no authentication of its own. The device offers it only while it is offline, and accepts credentials only
while the BLE [pairing window](../reference/config/ble.md#pairing-window) is open. The window takes the place of
the authorization button that Improv expects:

| Condition | Improv state |
|---|---|
| No network configured, or Wi-Fi down for `wifi.conf::improv_offline_s` (default 600 s), and the pairing window open | Authorized; accepts credentials |
| The same, with the pairing window closed | Authorization required; credentials refused |
| Wi-Fi down for a shorter time, or turned off with `wifi off` | Authorization required; credentials refused |
| Wi-Fi connected | Provisioned; credentials refused |

To authorize, power-cycle the device. The window opens at boot for `ble.conf::pair_window_s` (default 300 s), and
while it is open the device offers Improv once Wi-Fi has been down for 20 s, so it does not wait out
`improv_offline_s`. A device with no network configured is ready for setup right after power-up. `ble pair` from
another console opens the window as well, which is the only way with `pair_window_s=0`. With
`improv_offline_s=0`, Improv stays limited to devices with no network configured, also during the window. With
`ble_security=none` the window is not needed.

The device advertises Improv while it is offered, and for one minute after a successful setup. The
Improv command characteristic uses the same pairing mode as the console
([`ble.conf::ble_security`](../reference/config/ble.md)), so with `justworks` or `passkey` the phone pairs first.
The optional network scan is supported. Identify and hostname commands are not.

## Transports

Each transport has its own requirements on the device and its own client flags:

| Transport | Device side | Client flags |
|---|---|---|
| Serial | UART0 at 115200 baud; on ESP32-S3 also the USB-Serial-JTAG port | `-p PORT` (default `$ESPPORT`, else the first matching `/dev/cu.usbmodem*`, `/dev/ttyUSB*`, `/dev/ttyACM*`, …), `-b BAUD` |
| Telnet | `CONFIG_FUGU_WITH_TELNET` build, Wi-Fi up, `telnet` service running (off by default), TCP port 23, one client at a time | `--ip HOST[:PORT]` |
| TLS-PSK | `CONFIG_FUGU_WITH_TLS_CONSOLE` build, Wi-Fi up, `tlscon` service running with a key (off by default), TCP port 992, one client at a time | `--tls HOST[:PORT]`, `--psk-id`, `--psk` |
| BLE | `CONFIG_FUGU_WITH_BLE` build and the `ble` service running (off by default) | `--ble [NAME]`, `--name`, `--address`, `--adapter hciN` (Linux) |
| BLE via ESPHome proxy | same as BLE, reached through an ESPHome `bluetooth_proxy` | `--ble-proxy HOST[:PORT]` (default port 6053), `--proxy-password` |
| MQTT | `mqtt.conf::broker_uri` set; `cmd_input=1` to accept commands | `--mqtt BROKER`, `--mqtt-port`, `--mqtt-user`, `--mqtt-pass`, `--mqtt-readonly`, `--name` |

On every transport, the device follows each command's output with `OK: <cmd>` or `ERR: <cmd>`, with secret values
masked as `***` (see [Secrets](../reference/console.md#secrets)).

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
`telnet` service is off by default; enable it with `svc on telnet` over another console. See
[Services](../reference/services.md) and [`telnet.conf`](../reference/config/telnet.md).

:::warning
Telnet is unauthenticated and unencrypted. It runs only read-only commands such as `status`, `sensor` and
`get-config` unless `telnet.conf` sets `trusted=1`; then anyone on the network can run any command, including
`ota`. See [Trusted consoles](../reference/console.md#trusted-consoles).
:::

### TLS-PSK

The `tlscon` service serves the same console as telnet, encrypted with TLS 1.2 and authenticated by a pre-shared
key. Set a key over serial or BLE and start the service, as described in
[`tlscon.conf`](../reference/config/tlscon.md), then connect (Python 3.13 or later):

```bash
python3 etc/fugu_console.py --tls <device-ip> --psk-id <identity> --psk <hex key>
```

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

A client without a bond pairs on its first connection. That works only while the
[pairing window](../reference/config/ble.md#pairing-window) is open: for `pair_window_s` (default 300 s) after
power-up, or after `ble pair` on another console. Once bonded, the client connects at any time. This applies to every
client: the host running `fugu_console.py`, a browser using Web Bluetooth (the dashboard and the config editor pair
through the operating system), and a BLE proxy. If the host still holds a bond that the device has deleted (after
`ble unpair` or an NVS erase), remove the device in the host's Bluetooth settings and pair again inside a window.

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

The proxy, not the host, pairs with the device, so its first connection has to happen inside the
[pairing window](../reference/config/ble.md#pairing-window). Whether it keeps that bond across its own restarts
depends on the proxy firmware; one that does not needs a new window after each restart.

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
