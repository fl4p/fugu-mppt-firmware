---
title: Services
sidebar_position: 11
---

# Services

Everything outside the real-time control path runs on core 0: the consoles, Wi-Fi, MQTT, telemetry, file access,
the display and BLE. Most of these subsystems are wrapped as *services* with a common lifecycle, so they can be
started, stopped and inspected at runtime without touching the control loop on core 1.

This page covers how the core-0 side is organised. The [Service Architecture](../reference/services.md) reference
documents the `Service` API, the network watchdog and the per-service conf keys in detail.

## Core-0 loop

Arduino's `loop()` runs on core 0 (`CONFIG_ARDUINO_RUNNING_CORE=0`) and calls `loopNetwork_task()` once per
iteration. The function asserts that it runs on core 0 and does its work in a fixed order:

1. **Idle power profile** (`idleProfileTick`): runs first, so a converter start leaves the low-clock profile before
   any long job below.
2. **Serial consoles** (`loopUart`): reads and dispatches UART input, and USB input while a host is connected.
3. **RT log queue** (`flush_async_uart_log`): formats lines that core 1 queued and writes them to the UART, telnet,
   MQTT and BLE mirrors. At most 32 lines per call; the rest wait for the next iteration.
4. **Deferred calls** (`process_queued_tasks`): runs calls handed over from core 1 and commands queued by the
   telnet and MQTT consoles.
5. **Wi-Fi** (`networkLoopTick`): reconnect, the Wi-Fi-up and Wi-Fi-down edges, the network watchdog and modem
   power save. See [Wi-Fi](#wi-fi).
6. **Service ticks** (`g_services.tickAll()`): one `onTick()` per Running service, in registration order.
7. **Low-frequency work** (`loopLF`): every 3 s, the sampler and stuck watchdogs, low-frequency control, OTA
   image confirmation, energy metering, the status line and the LED. See [Protection](protection.md).
8. **Yield**: `vTaskDelay(1 ms)`, unless a scope client is attached; the scope pump then blocks for about one tick
   and serves as the yield.

Console commands from telnet and MQTT never run inside the network callback that received them. They are queued and
run in step 4, so a command that tears down its own transport (`svc off mqtt`, `wifi off`, `restart`) does not free
the connection it is executing on.

## Service model

A service (`src/service.h`) wraps one subsystem behind `start()`, `stop()`, `restart()` and a periodic `tick()`. The
concrete class implements `onStart()`, `onStop()` and optionally `onTick()`.

| State | Meaning |
|---|---|
| `Stopped` | Initial state, and the state after `stop()`. A stopped service does not tick. |
| `Running` | `onStart()` returned true. `tick()` calls `onTick()` on every loop iteration. |
| `Failed` | `onStart()` returned false or threw, or `onTick()` threw. `stop()` clears it to `Stopped`. |

Exceptions never leave a service: `start()` and `tick()` catch them and move the service to `Failed`, so a
misbehaving subsystem cannot take down the network loop.

Each service has its own conf file under `/littlefs/conf/`. Two keys there belong to the service layer:

- `enabled` (0/1): start the service at boot. The default comes from the service's constructor.
- `log_level` (`error`, `warn`, `info`): the ESP-IDF log level of the service's tag, which equals its name.

`ServiceManager g_services` is the registry. `setup()` registers every compiled-in service after the initial Wi-Fi
connect attempt, which reads both keys, and then starts the enabled ones. Registration order is tick order.

## Service list

The table lists the services in registration order. Each wrapper lives in its own header next to the module it
wraps; `src/main.cpp` only registers them.

| Name | Subsystem | Needs Wi-Fi | Enabled by default | Build option | Conf |
|---|---|---|---|---|---|
| `mqtt` | MQTT client: BMS data, console input, log mirror, Home Assistant | yes | yes | `FUGU_WITH_NETW` | [mqtt.conf](../reference/config/mqtt.md) |
| `tele` | InfluxDB telemetry over UDP | yes | no | `FUGU_WITH_NETW` | [tele.conf](../reference/config/tele.md) |
| `ftp` | FTP access to the conf partition | yes | yes | `FUGU_WITH_FTP` | [ftp.conf](../reference/config/ftp.md) |
| `telnet` | Telnet console and log sink, port 23 | yes | yes | `FUGU_WITH_NETW` | [telnet.conf](../reference/config/telnet.md) |
| `lcd` | I²C character display | no | no | always | [lcd.conf](../reference/config/lcd.md) |
| `scope` | Raw ADC stream over TCP, port 24 | yes | yes | `FUGU_WITH_SCOPE` | [scope.conf](../reference/config/scope.md) |
| `ble` | BLE console (NUS), BLE OTA, BLE telemetry | no | no | `FUGU_WITH_BLE` | [ble.conf](../reference/config/ble.md) |
| `bsync` | Switching-clock sync to Wi-Fi beacons | no | no | `FUGU_WITH_BSYNC` with MCPWM | [bsync.conf](../reference/config/bsync.md) |
| `extport` | External comms port: VE.Direct text output | no | no | `FUGU_WITH_EXTPORT` | [extport.conf](../reference/config/extport.md) |

`FUGU_WITH_FTP` and `FUGU_WITH_SCOPE` depend on `FUGU_WITH_NETW`. `bsync` sniffs beacons without associating, so it
does not wait for Wi-Fi. See [Build Options](../guide/getting-started/build-options.md) for the Kconfig switches.

### Per-service notes

- **mqtt**: `MqttService` (`src/tele/mqtt.*`) is the MQTT module itself. Its `preStart` hook, set in `setup()`, runs
  inside every `onStart()` and wires the charger's BMS subscriptions and the Home Assistant discovery callback. This
  keeps those dependencies out of the MQTT code, and `svc restart mqtt` re-subscribes. A second hook, `tickHook`,
  publishes the Home Assistant power value at most every 3 s. Without a `mqtt.conf` the service runs idle.
- **tele**: `onStart()` loads `tele.conf` and creates a flush task on core 0. `onTick()` only produces points; the
  flush task compresses and sends them. `onStop()` deletes the task.
- **telnet**: the connected client is also a log sink; `onStop()` drops the sink before the server stops.
- **lcd**: `onStart()` probes the display; `onTick()` refreshes Vin, Vout, Iin, Iout and temperature.
- **extport**: `onStart()` installs the UART1 driver on the configured pins; `onTick()` checks RX and sends one
  VE.Direct block per second.
- **scope**: the data channel is registered during sensor setup; the service only owns the TCP server and its
  pump. The sampling path reaches the scope through a plain pointer and does not depend on the service layer.

## Wi-Fi

Wi-Fi is a precondition for the network services, not a service itself. `setup()` starts a connect and waits up to
6 s for it before registering services. `startEnabledAtBoot()` then skips network services if Wi-Fi is not up yet, so they
stay `Stopped` rather than fail.

On every loop iteration `networkLoopTick()` handles the transitions:

| Event | Action |
|---|---|
| Wi-Fi comes up | `startEnabledNetworkServices()` starts every enabled network service that is not Running. |
| Wi-Fi up, every 30 s | The network watchdog restarts enabled network services that are `Failed` or `Stopped`. |
| `wifi off` | `stopNetworkServices()` runs while the netif is still valid, then Wi-Fi disconnects. |
| `wifi off <minutes>` expires | Wi-Fi reconnects; services follow on the up edge. |

`wifi off` only sets a flag; the teardown happens in the network loop on the next iteration. Tearing down from
inside the console callback that received the command would free sockets that the command's own reply still uses.
`svc off` persists `enabled = 0`, so the watchdog does not restart a service that was stopped on purpose.

Other Wi-Fi housekeeping in the same function:

- Reconnect attempts run only while the MCU die temperature is below 80 °C. An attempt blocks the network loop
  for up to 6 s. See [wifi.conf](../reference/config/wifi.md) for network selection.
- After 5 minutes without conversion (below 10 W), the modem switches to maximum power save; it returns to minimum
  power save as soon as conversion resumes.
- The network watchdog also re-initialises a disconnected MQTT client with growing gaps and reboots the device when
  MQTT stays down. See [Network watchdog](../reference/services.md#network-watchdog).

## BLE

The `ble` service runs the NimBLE NUS console. It is off by default because it exposes the console, and it does not
need Wi-Fi, so it remains reachable when Wi-Fi, MQTT or the heap are gone. The device advertises as
`fugu-<hostname>`; `ble_security` and `ble_passkey` in `ble.conf` select the pairing mode.

`onStop()` stops advertising and disconnects the client, but leaves the BLE stack initialised, because the Arduino
BLE wrapper cannot be re-initialised cleanly. The controller and the NimBLE host are pinned to core 0.

The service's `onTick()` drives all BLE traffic:

| Step | Function | Active when |
|---|---|---|
| Console input | `loopConsole` | A client writes to the RX characteristic |
| Console output | `bleTxDrain` | Output is queued; paced by NimBLE buffer availability |
| Firmware upload | `otaBleTick` | A [BLE OTA](../guide/updating/ota-ble.md) transfer is in progress |
| Telemetry stream | `teleBleTick` | `FUGU_WITH_BLE_TELE` build and `tele-ble 1` |
| Telemetry broadcast | `teleAdvTick` | `FUGU_WITH_BLE_ADV` build and `tele.conf::adv_ms` above 0 |

Both telemetry paths therefore stop with the `ble` service. The telemetry stream uses the binary wire on its own
notify characteristic; the broadcast puts a packed snapshot in the advertising data, readable without a connection.
See [GATT layout](../reference/ble/gatt.md).

## Console commands

The `svc` command (alias `service`) manages services from any console. See
[Service Commands](../reference/console.md#service-commands).

| Command | Effect |
|---|---|
| `svc` / `svc list` | Name, state, log level, enabled flag and a live detail (client address, `connected`) |
| `svc on <name>` | Persist `enabled = 1` and start |
| `svc off <name>` | Persist `enabled = 0` and stop |
| `svc restart <name>` / `svc rs <name>` | Stop, then start; re-reads the service's own conf keys. Refused while disabled |
| `svc log <name> <error\|warn\|info>` | Set and persist the log level |

`svc on` and `svc off` refuse and leave the service unchanged when the conf file cannot be written.

## Shutdown

`systemRestart()` disables the converter, half-closes the telnet connection and waits up to 2 s for the client to
close, then calls `g_services.stopAll()` while Wi-Fi is still up. Stopping MQTT before Wi-Fi drops ends its task
cleanly instead of letting it run into transport errors during the reset.
