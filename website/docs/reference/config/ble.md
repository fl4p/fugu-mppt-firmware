---
title: ble.conf
sidebar_position: 15
---

# ble.conf

`ble.conf` configures the BLE/NUS console, which is the service `ble` in `CONFIG_FUGU_WITH_BLE` builds. The service is off by default because it exposes the console.

`enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys. For how services use them, see [Service Architecture](../services.md#per-service-conf-file).

The file has the following keys:

| key            | unit | type   | default   | description                                                                 |
|----------------|------|--------|-----------|-----------------------------------------------------------------------------|
| `ble_security` |      | string | justworks | `justworks` (encrypted, no passkey), `passkey` (bond + MITM); any other value = open console |
| `ble_passkey`  |      | int    | 0         | Static passkey for `passkey` mode                                           |
| `idle_adv_ms`  | ms   | int    | 1000      | Connectable advertising interval while the idle power profile is on (see [`board.conf::idle_cpu_mhz`](board.md)), 20..10240. Applied while no client is connected, otherwise after it disconnects; the stack default (30–60 ms) returns when the profile ends. The non-connectable telemetry broadcast keeps its own timing, but in connectable mode the `tele.conf::adv_ms` payload rides this interval. 0 = unchanged |
| `enabled`      |      | bool   | 0         | Start this service at boot                                                  |
| `log_level` |      | enum   | info    | Verbosity: `error`, `warn` or `info` |
