---
title: ble.conf
sidebar_position: 15
---

# ble.conf

`ble.conf` configures the BLE/NUS console, which is the service `ble` in `CONFIG_FUGU_WITH_BLE` builds. The service is off by default because it exposes the console.

`enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys. For how services use them, see [Service Architecture](../services.md#per-service-conf-file).

`ble_security`, `ble_passkey` and `pair_window_s` are applied when the BLE stack first starts after boot; `svc rs ble`
only resumes advertising, so a change needs `restart`. `idle_adv_ms` is read at boot. The console masks
`ble_passkey` as `***` (see [Secrets](../console.md#secrets)).

| key             | unit | type   | default   | description |
|-----------------|------|--------|-----------|-------------|
| `ble_security`  |      | string | justworks | `justworks` (encrypted, no passkey) or `passkey` (bond + MITM); any other value, e.g. `none`, = no pairing and a read-only console (see [Trusted consoles](../console.md#trusted-consoles)). Also applies to the [Improv](../../guide/connecting.md#improv-over-ble) command characteristic |
| `ble_passkey`   |      | int    | 0         | Static passkey for `passkey` mode |
| `pair_window_s` | s    | int    | 300       | How long new devices can pair after boot, see [Pairing window](#pairing-window). 0 = only after `ble pair`; max 86400 |
| `idle_adv_ms`   | ms   | int    | 1000      | Connectable advertising interval while the idle power profile is on (see [`board.conf::idle_cpu_mhz`](board.md#idle-power-profile)), 20..10240; the stack default (30–60 ms) returns when the profile ends. In connectable mode the `tele.conf::adv_ms` payload rides this interval. 0 = unchanged |
| `enabled`       |      | bool   | 0         | Start this service at boot |
| `log_level`     |      | enum   | info      | Verbosity: `error`, `warn` or `info` |

## Pairing window

With `justworks` or `passkey`, a device that has no bond yet can pair only while the pairing window is open. Bonded
devices connect and use the console at any time. The window opens:

- For `pair_window_s` after every boot, so a power cycle stands for physical access.
- With the console command `ble pair [seconds]` from any console that already accepts commands (serial, telnet,
  MQTT, or a bonded BLE client). Without an argument it opens for `pair_window_s`, or 300 s if that is 0.

While the window is closed, the device rejects every pairing request before any keys are exchanged. This covers a
bonded device that tries to pair again, so a peer that impersonates a bonded address cannot replace that bond. A peer
without a bond stays connected without encryption: it cannot write commands and receives no console output. The
device logs a warning with the peer address, and an info line for each paired or bonded peer.

The device stores up to 3 bonds in NVS. They survive reboots and firmware updates, and `ble unpair` or erasing the NVS
partition removes them. When the table is full, a new pairing inside the window evicts the oldest bond. Outside the
window it is refused. Stored notification subscriptions have their own table; when it is full, the device drops a
stored subscription (another peer's first), never a bond. Clients subscribe again on every connection anyway. List and
remove bonds with `ble bonds` and `ble unpair <address|all>`, see
[Console commands](../console.md#ble-commands).

With `ble_security=none` the window does not apply: any device can connect and run read-only commands, and
[Improv](../../guide/connecting.md#improv-over-ble) accepts credentials whenever it is offered.

