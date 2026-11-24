---
title: "Service Architecture"
sidebar_position: 4
---

# Service Architecture

`src/service.h` is a small, systemd-lite service layer for the firmware's optional, non-real-time
subsystems. A *service* wraps one building block (MQTT, telemetry/InfluxDB, FTP, telnet, LCD,
scope, BLE console, beacon sync) behind a uniform lifecycle and status, with its own runtime log level and a
persisted enable flag.

## Scope

Services cover the core-0 subsystems that can be brought up and torn down independently. The
real-time control path on core 1 (ADC sampling, MPPT, protection, the synchronous-converter PWM)
can never be stopped and is not a service.

Wi-Fi is a precondition rather than a service. Services that need the network stay `Stopped` while
Wi-Fi is down (a manual `svc on` then reports `Failed`), then self-heal when it comes up. See
[Wi-Fi dependency](#wi-fi-dependency).

## ServiceState

A service is always in one of three states:

```
enum class ServiceState { Stopped, Running, Failed };
```

- `Stopped`: not running; the initial state and the state after `stop()`.
- `Running`: `onStart()` succeeded.
- `Failed`: `onStart()` returned false or threw, or `onTick()` threw. `stop()` clears a `Failed`
  service back to `Stopped`.

## `Service` base class

The base class is defined in `src/service.h`. Its constructor has this signature:

```
Service(const char *name, const char *confPath,
        bool requiresNetwork = false, bool enabledDefault = true)
```

`name` is both the human-facing identifier and the ESP_LOG tag. `confPath` is the service's own
conf file. `requiresNetwork` marks services that depend on Wi-Fi. `enabledDefault` is the fallback
enable state when the conf file (or its `enabled` key) is absent.

### Public lifecycle

The public methods drive the lifecycle and the persisted settings:

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

The class has these accessors: `name()`, `state()` (routes through `liveState()`), `logLevel()`, `enabled()`, `requiresNetwork()`.

### Hooks (overridden by concrete services)

Concrete services override these hooks:

- `virtual bool onStart() = 0;`: bring the subsystem up; return false (or throw) to fail.
- `virtual void onStop() = 0;`: tear it down.
- `virtual void onTick() {}`: periodic work, invoked each network-loop iteration while `Running`.
- `virtual ServiceState liveState() const;`: override when "running" depends on a live condition
  (default returns the stored state).

### Log levels

A service's `name()` is its ESP_LOG tag, so the level maps directly onto the framework's per-tag
filtering. Only `ESP_LOG_ERROR`, `ESP_LOG_WARN`, and `ESP_LOG_INFO` are usable. The build caps
the compiled-in maximum at INFO, so DEBUG/VERBOSE are unavailable. The helpers `levelToStr`,
`strToLevel`, and `stateStr` convert to/from the console strings `error|warn|info` and the state
names.

## `ServiceManager`

`ServiceManager` is a registry of `Service*` pointers. Its global instance, `g_services`, is an
inline variable in `src/service.h`. It provides these methods:

- `registerService(Service*)`: appends and immediately calls the service's `loadConf()`.
- `findByName(const char*)`: case-insensitive lookup.
- `all()`: the registered services, for status listing.
- `startEnabledAtBoot(bool networkUp)`: starts enabled services; network ones only if `networkUp`.
- `startEnabledNetworkServices()`: restarts enabled network services that are not `Running`;
  called on the Wi-Fi-up edge.
- `retryFailedNetworkServices()`: starts enabled network services that are `Failed` or `Stopped`
  again; called every 30 s while Wi-Fi is up (see [Network watchdog](#network-watchdog)).
- `stopNetworkServices()`: stops the network services while the netif is still valid; called on the
  Wi-Fi-down edge, before `WiFi.disconnect()`.
- `tickAll()`: ticks every service.

## Threading and integration

All ticks run on core 0, from `loopNetwork_task` in `src/main.cpp`. Registration and
`startEnabledAtBoot()` happen in `setup()` after the filesystem is mounted, after the initial Wi-Fi
connect attempt, and after the scope data channel is set up.

Service wrapper objects are file-scope `inline` globals defined in their headers. They aren't
registered from constructors, because the filesystem is not mounted at static-init time.
Registration order in `setup()` determines tick order.

## Wi-Fi dependency

While Wi-Fi is down, `startEnabledAtBoot()` skips network services (`requiresNetwork == true`),
so they stay `Stopped`. A manual start attempt without Wi-Fi (e.g. `svc on ftp`) returns false from
`onStart()` and reports `Failed`. When Wi-Fi comes up, `loopNetwork_task` detects the rising edge and calls
`startEnabledNetworkServices()`, which (re)starts them. MDNS is set up before they start.

The falling edge is handled the same way but deferred. `wifi off` may run from a console, telnet or
MQTT input callback, so it only sets `disableWifi`; `loopNetwork_task` does the actual teardown on the
next tick (`stopNetworkServices()` then `WiFi.disconnect()`), outside any input callback.

Tearing down inline was a use-after-free: a `wifi off` arriving over telnet deinitialised the netif under its own
socket, and the command's confirmation log, mirrored to that telnet client, then wrote to freed lwip
pbufs.

## Network watchdog

The network loop evaluates a watchdog once per second (`src/etc/net_guard.h`, wired in
`src/main.cpp`). It acts only while Wi-Fi is connected and not switched off with `wifi off`; a
Wi-Fi outage restarts all of its timers.

- **Failed services**: every 30 s, enabled network services in state `Failed` or `Stopped` are
  started again. A service whose start failed for lack of heap therefore recovers once memory is
  available, without waiting for the next Wi-Fi-up edge. `svc off` persists `enabled = 0`, so a
  service stopped on purpose stays stopped.
- **MQTT re-init**: when `mqtt` is `Running` with a broker configured but has not been connected
  for more than 120 s, the service is restarted (client destroyed and created again, subscriptions
  rebuilt). The gap to the next re-init doubles each time (120 s, 240 s, 480 s, ...) up to 30
  minutes, so re-inits happen at about 2, 6, 14 and 30 minutes of downtime, then every 30 minutes;
  a connect resets the sequence. The client
  itself retries the connection every 10 s in between. A `Failed` service is restarted by the 30 s
  retry instead. The warning log line carries the re-init count and the free heap. The restart runs
  from the network loop, never from the MQTT client's own task.
- **Reboot**: the device restarts through the normal restart path when MQTT (enabled, broker
  configured, `Running` or `Failed`) has not been connected for more than 15 minutes, or for more
  than 10 minutes while the free heap has stayed below 12 KB (or the largest free block below 9 KB)
  for at least a minute. The reboot is skipped within the first 5 minutes of uptime, during a flash
  write or a BLE firmware upload, and while restarts are blocked after a bootloader update. If the
  restart does not happen, the condition re-arms after another 10 or 15 minutes.
- **Reboot hold**: a guard reboot sets a flag in RTC memory, which survives software resets but not
  a power cycle. While the flag is set, the guard does not reboot again until the uptime exceeds
  6 hours; the first MQTT connect clears it. A broker that stays down for days therefore causes at
  most one guard reboot every 6 hours.

`bootinfo` prints a `netguard:` line with the current MQTT downtime, the re-init count since boot,
the guard reboots since power-on and whether the reboot hold is set.

`mqtt` reports `Failed` when its client cannot be created or its task cannot be started, instead
of showing `Running` without a client. Each `onStart()` releases a client left over from an
earlier attempt, because `start()` on a `Failed` service calls `onStart()` without `onStop()`. Every
teardown clears the connected flag after the client task has stopped: a connect that completes while
`esp_mqtt_client_stop()` waits still dispatches `MQTT_EVENT_CONNECTED`, and the task then exits
without a `MQTT_EVENT_DISCONNECTED`. A start that throws (for example from a subscription hook)
releases everything it allocated and leaves the service `Failed` and retryable. The connected
state, which `svc list` shows in its last column, requires a `Running` service with a live client.

### Heap diagnostics

These diagnostics are always on:

- A failed-allocation hook records the size, capabilities, heap function, task name, free heap
  and a short backtrace of the most recent failure. It neither allocates nor blocks; the
  low-frequency loop on core 0 prints the record (the first five, then at most one every 10 s,
  with the number of failures since the last report). Symbolicate the backtrace addresses
  against the build ELF.
- A warning when the free heap is below 8 KB, at most once a minute.
- Telemetry fields `heap` and `heap_min` (free and minimum-ever free heap in bytes) on every 40th
  point.

## File layout

The base class and the registry live in `src/service.h`. Each concrete wrapper lives in its own header next to
the module it wraps, and defines an `inline <Name>Service <name>Service;` instance:

| Service | Header | Touches main.cpp globals? |
|---|---|---|
| `MqttService` | `src/tele/mqtt.h` / `mqtt.cpp` | the service *is* the MQTT module |
| `FtpService` | `src/tele/ftp_service.h` | no |
| `HttpConfService` | `src/tele/http_conf_service.h` | no; compiled only with `WITH_HTTP_CONF` |
| `TelnetService` | `src/tele/telnet_service.h` | no; compiled only with `WITH_TELNET` |
| `TlsConsoleService` | `src/tele/tls_console.h` | no; compiled only with `WITH_TLS_CONSOLE` |
| `TelemetryService` | `src/tele/telemetry_service.h` | `mppt` (extern) |
| `ScopeService` | `src/tele/scope_service.h` | owns the `Scope` object as a member; the global `scope` pointer aims at it |
| `LcdService` | `src/viz/lcd_service.h` | owns the `LCD` as a member; also `adcSampler`, `sensors`, `mppt` (extern) |
| `BleConsoleService` | `src/tele/console_ble_service.h` | no; whole header compiled only with `WITH_BLE` |
| `BeaconSyncService` | `src/sync/bsync.h` / `bsync.cpp` | the converter's MCPWM leg; compiled only with `WITH_BSYNC` + `WITH_MCPWM` |

Where practical a service *owns* the object it wraps rather than referencing a global: `LcdService::lcd`
is the display (mppt holds a reference to it and the boot splash draws on it via `lcdService.lcd`),
and `ScopeService::scopeObj` is the scope.

The real-time ADC path also reaches the scope through the lightweight global pointer `scope`
(declared in `scope.h`), which is constant-initialised to point at `scopeService.scopeObj`. This
keeps the RT/sampling code free of any dependency on the service layer. FTP, telnet, and telemetry
have no such global. Their underlying objects are translation-unit statics in telemetry.cpp behind a
free-function API that the services call.

`main.cpp` includes these headers and keeps only the registration block; it holds no service class
definitions.

## Concrete services

The following table lists each service's conf file, network dependency, and hooks:

| Name | Conf | Network | `onStart` | `onStop` | `onTick` |
|---|---|---|---|---|---|
| `mqtt` | `mqtt.conf` | yes | release a leftover client, read conf, run the `preStart` hook (charger BMS subscriptions + `onConnected`), then `init()` (fails if the client cannot be created or started) | `close()` | — (the MQTT client runs its own task) |
| `tele` | `tele.conf` | yes (disabled by default) | require Wi-Fi (UDP is connectionless), load the wire format, spawn the flush task (fails if it cannot be created) | delete the flush task | produce InfluxDB points (the flush task sends them) |
| `ftp` | `ftp.conf` | yes | require Wi-Fi, resolve credentials, listen on port 21 | close the listener and sessions | poll sockets with a zero-timeout `select()`, one command and one chunk per session |
| `http` | `http.conf` | yes | require Wi-Fi, resolve credentials, listen on `port` | close the listener and clients | poll sockets with a zero-timeout `select()`, one chunk per connection |
| `telnet` | `telnet.conf` | yes (disabled by default) | require Wi-Fi, then `setupTelnet()` (fails if the server does not start) | `telnetStop()` (drops the log sink) | `telnetLoop()` |
| `tlscon` | `tlscon.conf` | yes (disabled by default) | require Wi-Fi, parse `psk_id`/`psk` (fails without a key), listen on `port` | close the session and the listener | accept one client, step the TLS handshake, read command lines, send the buffered log |
| `lcd` | `lcd.conf` | no (disabled by default) | `lcd.init()` if not already initialised | — | refresh the display |
| `scope` | `scope.conf` | yes | require Wi-Fi, then begin the TCP server | end the server | accept clients, then pump (the pump blocks ~1 tick and serves as the loop's yield) |
| `ble` | `ble.conf` | no (disabled by default) | begin NUS with `ble_security` / `ble_passkey` | end NUS | drive the RX queue |
| `bsync` | `bsync.conf` | no (sniffs beacons without association; disabled by default) | parse `bssid`/`channel`/loop gains, bring up the RX-only sniffer, start the period dither | restore the nominal period, stop the sniffer | 1 Hz servo; re-arms the sniffer radio if it went down |

`MqttService` is the MQTT module itself rather than a separate wrapper; a `preStart`
`std::function<void(const ConfFile&)>` set in `setup()` keeps the charger / Home-Assistant coupling
out of the MQTT translation unit. The hook re-runs on every start, so `svc restart mqtt`
re-subscribes the BMS topics.

Two other services have extra lifecycle details:

- `telnet` doubles as a log sink while a client is attached; `onStop()` drops that sink before the
  server is torn down, so logging never writes to a dead connection.
- `scope` has its object and data channel created in `setup()` (the channel must be registered
  during sensor setup); the service only owns the server's begin/end and the network pump.

## Per-service conf file

Each service owns a flat key=value conf file (parsed by `ConfFile`, `src/conf.h`) with two
service-layer keys alongside the subsystem's own configuration:

```
enabled   = 1        # 1 = start at boot, 0 = leave stopped
log_level = info     # error | warn | info
```

Defaults when the file or a key is absent: `enabled` follows the constructor's `enabledDefault`
(true for `mqtt`, `ftp`, `http`, and `scope`; false for `telnet`, `tlscon`, `tele`, `lcd`, `ble`, and `bsync`), and `log_level` is `info`.

`ConfFile::add` updates these keys in place and rewrites the whole file, preserving inline and
full-line comments and never growing the file with duplicate lines, which suits keys that are
rewritten repeatedly. (`ConfFile::addFast` is the append-only alternative for one-shot writes where
file growth is acceptable.)

The conf files ship from the board images under `config/*/conf/` and are flashed with the
firmware's littlefs partition.

## Console commands

`handleCommand()`, which UART, USB-CDC, telnet, and MQTT share, dispatches these commands:

- `svc` or `svc list`: table of name, state, log level, and enabled flag (the `mqtt` row
  also shows `connected` while it is `Running` with a live connection).
- `svc on <name>`: persist `enabled = 1` and start.
- `svc off <name>`: persist `enabled = 0` and stop.
- `svc restart <name>` (alias `svc rs <name>`): stop then start. Refused while the service is disabled; `svc on`
  enables and starts it.
- `svc log <name> <error|warn|info>`: set and persist the log level.
