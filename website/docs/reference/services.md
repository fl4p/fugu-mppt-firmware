---
title: "Service Architecture"
sidebar_position: 4
---

*this document is an LLM generated placeholder*

# Service Architecture

A small, systemd-lite service layer (`src/service.h`) for the firmware's optional, non-real-time
subsystems. A *service* wraps one building block (MQTT, telemetry/InfluxDB, FTP, telnet, LCD,
scope, BLE console, beacon sync) behind a uniform lifecycle and status, with its own runtime log level and a
persisted enable flag.

## Scope

- **In scope:** core-0 subsystems that can be brought up and torn down independently.
- **Out of scope:** the real-time control path on core 1 (ADC sampling, MPPT, protection, the
  synchronous-converter PWM). These are never stoppable and are not services.
- **Wi-Fi is a precondition, not a service.** Services that need the network report `Failed` until
  Wi-Fi is up, then self-heal (see *Wi-Fi dependency*).

## ServiceState

```
enum class ServiceState { Stopped, Running, Failed };
```

- `Stopped` — not running; the initial state and the state after `stop()`.
- `Running` — `onStart()` succeeded.
- `Failed`  — `onStart()` returned false or threw, or `onTick()` threw. `stop()` clears a `Failed`
  service back to `Stopped`.

## `Service` base class

Defined in `src/service.h`. Construction:

```
Service(const char *name, const char *confPath,
        bool requiresNetwork = false, bool enabledDefault = true)
```

`name` is both the human-facing identifier and the ESP_LOG tag. `confPath` is the service's own
conf file. `requiresNetwork` marks services that depend on Wi-Fi. `enabledDefault` is the fallback
enable state when the conf file (or its `enabled` key) is absent.

### Public lifecycle

| Method | Behaviour |
|---|---|
| `bool start()` | No-op (returns true) if already `Running`. Applies the log level, then calls `onStart()`. A false return or a thrown exception transitions to `Failed`. Never propagates an exception. |
| `void stop()` | No-op if already `Stopped`. Calls `onStop()` and sets `Stopped` (also clearing `Failed`). |
| `bool restart()` | `stop()` then `start()`. |
| `void tick()` | Calls `onTick()` only while `Running`; a thrown exception transitions to `Failed`. |
| `void setLogLevel(esp_log_level_t, bool persist = true)` | Applies the level to the ESP log tag and, if `persist`, writes `log_level` to the conf file. |
| `void setEnabledPersist(bool)` | Writes `enabled` to the conf file. Does not itself start/stop. |
| `void loadConf()` | Reads `enabled` and `log_level` from the conf file and applies the level. |

### Accessors

`name()`, `state()` (routes through `liveState()`), `logLevel()`, `enabled()`, `requiresNetwork()`.

### Hooks (overridden by concrete services)

- `virtual bool onStart() = 0;` — bring the subsystem up; return false (or throw) to fail.
- `virtual void onStop() = 0;` — tear it down.
- `virtual void onTick() {}` — periodic work, invoked each network-loop iteration while `Running`.
- `virtual ServiceState liveState() const;` — override when "running" depends on a live condition
  (default returns the stored state).

### Log levels

A service's `name()` is its ESP_LOG tag, so the level maps directly onto the framework's per-tag
filtering. Only `ESP_LOG_ERROR`, `ESP_LOG_WARN`, and `ESP_LOG_INFO` are usable — the build caps
the compiled-in maximum at INFO, so DEBUG/VERBOSE are unavailable. Helpers `levelToStr`,
`strToLevel`, and `stateStr` convert to/from the console strings `error|warn|info` and the state
names.

## `ServiceManager`

A registry holding `Service*` (global instance `g_services`, an inline variable in `src/service.h`).

- `registerService(Service*)` — appends and immediately calls the service's `loadConf()`.
- `findByName(const char*)` — case-insensitive lookup.
- `all()` — the registered services, for status listing.
- `startEnabledAtBoot()` — starts every service whose persisted `enabled` flag is set.
- `startEnabledNetworkServices()` — restarts enabled network services that are not `Running`;
  called on the Wi-Fi-up edge.
- `stopNetworkServices()` — stops the network services while the netif is still valid; called on the
  Wi-Fi-down edge, before `WiFi.disconnect()`.
- `tickAll()` — ticks every service.

## Threading and integration

All ticks run on **core 0**, from `loopNetwork_task` in `src/main.cpp`. Registration and
`startEnabledAtBoot()` happen in `setup()` after the filesystem is mounted, after the initial Wi-Fi
connect attempt, and after the scope data channel is set up.

Service wrapper objects are file-scope `inline` globals defined in their headers (not registered
from constructors — the filesystem is not mounted at static-init time). Registration order in
`setup()` determines tick order.

## Wi-Fi dependency

Network services (`requiresNetwork == true`) return false from `onStart()` while Wi-Fi is down, so
they report `Failed`. When Wi-Fi comes up, `loopNetwork_task` detects the rising edge and calls
`startEnabledNetworkServices()`, which (re)starts them. MDNS is set up before they start.

The **falling edge** is symmetric, and deliberately deferred: `wifi off` only sets `disableWifi` —
it may run from a console / telnet / MQTT input callback — and `loopNetwork_task` does the actual
teardown on the next tick (`stopNetworkServices()` then `WiFi.disconnect()`), *outside* any input
callback. Tearing down inline was a use-after-free: a `wifi off` arriving over telnet deinitialised
the netif under its own socket, and the command's confirmation log — mirrored to that telnet client —
then wrote to freed lwip pbufs.

## File layout

The base + registry live in `src/service.h`. Each concrete wrapper lives in its own header next to
the module it wraps, and defines an `inline <Name>Service <name>Service;` instance:

| Service | Header | Touches main.cpp globals? |
|---|---|---|
| `MqttService` | `src/tele/mqtt.h` / `mqtt.cpp` | the service *is* the MQTT module |
| `FtpService` | `src/tele/ftp_service.h` | no |
| `TelnetService` | `src/tele/telnet_service.h` | no |
| `TelemetryService` | `src/tele/telemetry_service.h` | `mppt` (extern) |
| `ScopeService` | `src/tele/scope_service.h` | owns the `Scope` object as a member; the global `scope` pointer aims at it |
| `LcdService` | `src/viz/lcd_service.h` | owns the `LCD` as a member; also `adcSampler`, `sensors`, `mppt` (extern) |
| `BleConsoleService` | `src/tele/console_ble_service.h` | no; whole header compiled only with `WITH_BLE` |
| `BeaconSyncService` | `src/sync/bsync.h` / `bsync.cpp` | the converter's MCPWM leg; compiled only with `WITH_BSYNC` + `WITH_MCPWM` |

Where practical a service *owns* the object it wraps rather than referencing a global: `LcdService::lcd`
is the display (mppt holds a reference to it and the boot splash draws on it via `lcdService.lcd`),
and `ScopeService::scopeObj` is the scope. The scope is also reached from the real-time ADC path
through the lightweight global pointer `scope` (declared in `scope.h`), which is constant-initialised
to point at `scopeService.scopeObj`; this keeps the RT/sampling code free of any dependency on the
service layer. FTP/telnet/telemetry have no such global — their underlying objects are translation-
unit statics in telemetry.cpp behind a free-function API that the services call.

`main.cpp` includes these headers and keeps only the registration block; it holds no service class
definitions.

## Concrete services

| Name | Conf | Network | `onStart` | `onStop` | `onTick` |
|---|---|---|---|---|---|
| `mqtt` | `mqtt.conf` | yes | read conf, run the `preStart` hook (charger BMS subscriptions + `onConnected`), then `init()` | `close()` | — (the MQTT client runs its own task) |
| `tele` | `tele.conf` | yes (disabled by default) | require Wi-Fi (UDP is connectionless), load the wire format, spawn the flush task | delete the flush task | produce InfluxDB points (the flush task sends them) |
| `ftp` | `ftp.conf` | yes | require Wi-Fi, then `ftpBegin()` | `ftpEnd()` | `ftpHandle()` |
| `telnet` | `telnet.conf` | yes | require Wi-Fi, then `setupTelnet()` | `telnetStop()` (drops the log sink) | `telnetLoop()` |
| `lcd` | `lcd.conf` | no (disabled by default) | `lcd.init()` if not already initialised | — | refresh the display |
| `scope` | `scope.conf` | yes | require Wi-Fi, then begin the TCP server | end the server | accept clients, then pump (the pump blocks ~1 tick and serves as the loop's yield) |
| `ble` | `ble.conf` | no (disabled by default) | begin NUS with `ble_security` / `ble_passkey` | end NUS | drive the RX queue |
| `bsync` | `bsync.conf` | no — sniffs beacons without association (disabled by default) | parse `bssid`/`channel`/loop gains, bring up the RX-only sniffer, start the period dither | restore the nominal period, stop the sniffer | 1 Hz servo; re-arms the sniffer radio if it went down |

`MqttService` is the MQTT module itself rather than a separate wrapper; a `preStart`
`std::function<void(const ConfFile&)>` set in `setup()` keeps the charger / Home-Assistant coupling
out of the MQTT translation unit and re-runs on every start (so `svc restart mqtt` re-subscribes
the BMS topics).

Notes specific to two services:

- **telnet** doubles as a log sink while a client is attached; `onStop()` drops that sink before the
  server is torn down, so logging never writes to a dead connection.
- **scope** has its object and data channel created in `setup()` (the channel must be registered
  during sensor setup); the service only owns the server's begin/end and the network pump.

## Per-service conf file

Each service owns a flat key=value conf file (parsed by `ConfFile`, `src/conf.h`) with two
service-layer keys alongside the subsystem's own configuration:

```
enabled   = 1        # 1 = start at boot, 0 = leave stopped
log_level = info     # error | warn | info
```

Defaults when the file or a key is absent: `enabled` follows the constructor's `enabledDefault`
(true for `mqtt`, `ftp`, `telnet`, `scope`; false for `tele`, `lcd`, `ble` and `bsync`), `log_level` is `info`.

`ConfFile::add` updates these keys in place and rewrites the whole file, preserving inline and
full-line comments and never growing the file with duplicate lines — appropriate for keys that are
rewritten repeatedly. (`ConfFile::addFast` is the append-only alternative for one-shot writes where
file growth is acceptable.)

The conf files ship from the board images under `config/*/conf/` and are flashed with the
firmware's littlefs partition.

## Console commands

Dispatched from `handleCommand()` (shared by UART, USB-CDC, telnet, and MQTT):

- `svc` or `svc list` — table of name, state, log level, and enabled flag (the `mqtt` row
  also indicates a live connection).
- `svc on <name>` — persist `enabled = 1` and start.
- `svc off <name>` — persist `enabled = 0` and stop.
- `svc restart <name>` (alias `svc rs <name>`) — stop then start.
- `svc log <name> <error|warn|info>` — set and persist the log level.
