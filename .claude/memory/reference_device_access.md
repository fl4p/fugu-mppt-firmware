---
name: reference_device_access
description: Reaching fry/flat — NAT ports flip-flop (confirm hostname), MQTT console topics pv/log/<host>/cmd, BLE proxy MACs, havan over Tailscale, creds env files, patched mosquitto ACL image
metadata:
  type: reference
---

NAT table, `nat-tunnel.sh`, BLE backup link and `--mqtt` usage: see CLAUDE.md "Connecting to devices". Credentials: never in notes; source `etc/mqtt.env`, `etc/influx.env` (gitignored; `set -a; . etc/x.env; set +a`) or `.claude/memory/secrets.env`.

## NAT port mapping flip-flops
2026-05-22 `:232`->flat, 2026-06-07 `:232`->fry, and 2026-11-12 it flipped between consecutive connections. Never trust a table or a previous connection: check `hostname`/banner on every connection before driving or flashing.

## MQTT console
Device logs to `pv/log/<hostname>` and takes commands on `pv/log/<hostname>/cmd`. Broker per `mqtt.conf broker_uri` (fry/flat: 192.168.1.200:1882; brokers drift, read `config/dl/<dev>.*/conf/mqtt.conf`). From off-LAN: `ssh -L 11882:127.0.0.1:1882 havan-ts`. Message-framed, so long output (`coredump get`) arrives whole. When a link flaps, wait for a fresh `pv/log/<host>` line, then send. Pullers on havan: `/home/fab/miniconda3/bin/python3` (has paho).

## ESPHome BLE proxy
nimble-proxy at 192.168.1.231:6053 (plaintext API). fry MAC `70:04:1d:a6:ab:32`, flat `70:04:1d:a4:ea:36` (public). GATT runs on the proxy, so macOS GATT cache/bond trouble does not apply. ESPHome 2026.5 forwards only raw adverts (transport parses AD bytes). "error 534" on disconnect is normal; rapid reconnects can hang (transport retries, `connect_retries` 2). Validated 2026-05-28.

## havan off-site
`havan.local` does not resolve off-LAN; Tailscale 100.64.0.10 with `ssh -o HostKeyAlias=havan.local 100.64.0.10` (bare IP fails host-key check). From havan, .231 and .200 are reachable.

## Mosquitto ACL image
Since 2026-11-24 havan's mosquitto add-on runs locally built `local/app-mosquitto:7.1.3-pr6`, tagged as official 7.1.3, with `/share/mosquitto/acl.conf`; user `pv` may write only pv/#, homeassistant/#, lora-bridge/#.
- **Why:** stock add-on makes every login superuser, so the `pv` password (in every dashboard browser) could spoof the BMS topics the chargers read. Upstream PR #6 unmerged (issue #5).
- **How to apply:** before any mosquitto add-on update, check it contains PR #6 or the ACL silently stops; a new login needs a `user` block. Details/rollback: doc/lab/havan-mqtt-tls.md.

## Creds env files
`etc/influx.env` = InfluxDB `open_pe` at influx.fabi.me (batmon, bench data; NOT the `mppt` fleet data, see [[project_telemetry]]). `etc/mqtt.env` = broker 192.168.1.200:1882, user `pv`.
