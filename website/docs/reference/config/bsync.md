---
title: bsync.conf
sidebar_position: 16
---

# bsync.conf

Beacon PWM-clock sync (service `bsync`, `CONFIG_FUGU_WITH_BSYNC` builds; needs NETW + MCPWM). `enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys, see [Service Architecture](../services.md#per-service-conf-file).

Frequency/phase-locks the MCPWM switching clock of multiple converters to a shared timebase
recovered from sniffed 802.11 beacons (receive-only, no association/TX — usable while Wi-Fi is
"off" for precision measurements). All participating devices must point `bssid`/`channel` at the
*same* AP. See [Beacon Clock Sync](../../development/sync/beacon-sync.md).

| key         | unit | type   | default | description                                                                |
|-------------|------|--------|---------|----------------------------------------------------------------------------|
| `bssid`     |      | string | —       | Sync AP BSSID `aa:bb:cc:dd:ee:ff` (required; same on all devices)          |
| `channel`   |      | int    | 1       | Wi-Fi channel of the sync AP (used only while unassociated)                |
| `phase_us`  | µs   | float  | 0       | Per-device target phase offset on the shared grid (interleaving)           |
| `bw`        |      | float  | 1.0     | Loop-bandwidth scale (0.05–4): lower = less phase breathing, slower lock   |
| `kp`        |      | float  | 5e-6    | Servo P gain, period-ticks per phase-error-tick (×bw)                      |
| `ki`        | 1/s  | float  | 2.5e-7  | Servo I gain, steady-state crystal-offset trim (×bw²)                      |
| `hw_only`   |      | bool   | 0       | Accept only full-IE (hw TBTT) beacons; drops short injected frames         |
| `enabled`   |      | bool   | 0       | Start this service at boot                                                 |
| `log_level` |      | enum   | info    | Verbosity: `error`, `warn` or `info` |
