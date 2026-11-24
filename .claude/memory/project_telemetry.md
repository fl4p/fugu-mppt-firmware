---
name: project_telemetry
description: Telemetry pipeline — in-house text/binary line protocol + tamp, havan relay to tm.fabi.me/ha_van `mppt`, BLE tele stream/adv, dashboard (havan http / GitHub Pages), query quirks
metadata:
  type: project
---

Influx query recipe and field list: see CLAUDE.md "Device Data".

## Where `mppt` lives
firmware -> UDP binary -> havan:8086 `influxdb-udp-relay` add-on -> tm.fabi.me:8086 db `ha_van` (creds `.claude/memory/secrets.env`). NOT influx.fabi.me/open_pe (batmon, bench `pwr_test`). tm.fabi.me resolves only via NAT64: use IPv4 135.181.43.162 over HTTP.
- havan console log is Europe/Lisbon time, Influx is UTC (summer: console = UTC+1).
- `mppt_state` stuck at 4 — don't infer idle from it. Idle (duty 0) `max(P)` shows 2-4 W spikes = Iin noise x Voc.
- Relay 0.15+ decodes binary and republishes datagrams to `pv/tele/<device>` (0.16 on havan since 11-18; local add-on at /var/lib/homeassistant/apps/local/influxdb-udp-relay; set options via Supervisor API from `hassio_cli` — /data/options.json edits are overwritten; `ha store reload` then `ha apps update`).

## Line protocol (text)
2026-05-21: vendored InfluxDB-Client-for-Arduino removed; header-only `src/tele/line_protocol.h` `LineProtocol` writes into one reserved string. Tags before fields; int fields get `i`; `setTimeMs()` uses a digit loop. Validate with `etc/influx_test.py`.
- Lesson from the old lib: `%llu` under newlib-nano printed literal `lu` as every timestamp ([[project_build_toolchain]]).

## Binary protocol + tamp
`src/tele/sym_line_protocol.h` (`BinaryLineProtocol` + `SymbolTable`): LEB128 SIDs, per-symbol datatype in table frames, LE raw values, varint ms ts; frames length-prefixed (1=data, 2=table). New symbol -> table resent on next 3 consecutive points, then 2x at 20 s. Runtime: `tele.conf binary` (0 text / 1 binary; fry+flat run 1 since 11-18) + `compressor` (none|tamp). tamp vendored at `components/tamp/`, wrapper `src/tele/compress.h`.
- Sizes on real data: text 275 B/pt, f32 binary 106, f16 69; per-batch tamp on changing f32 only ~50 %, so `Compressor::maxBatchRaw(mtu)` sizes batches (tamp ~1.5 x MTU raw). Per-datagram tamp alone is a net loss.
- Encode cost on RT core: text 1645 us/pt (nano `%f`) vs binary 264 us/pt; tamp 40-pt batch 22.6 ms on core 0.
- Gotcha: `addField(int32_t)` is ambiguous on xtensa (int32_t = long); use `addField(int)`.
- Receiver `etc/influx_binary_proxy.py` (also `--ble`, `--ble-all`, `--adv`); plain InfluxDB cannot ingest binary.

## BLE telemetry
2026-08-05, bench-validated on fbuck/fboost. `CONFIG_FUGU_WITH_BLE_TELE`: NUS char `6E400005`, records `0x7E + varint len + cid + payload`. Enable: `set-time <epoch_ms>` then `tele-ble 1` (refused while UDP tele runs text wire). Core split into `src/tele/tele_core.cpp` / `tele_ble.cpp`; buffers pre-reserved at enable.
- OOM landmines fixed: console txBuf compaction + reserve, `TaskQueue::add` drop instead of assert, log mirror capped at half txBuf.
- NimBLE `ble_att_cmd.c:91 assert(rc==0)` on mbuf shortage: IDF patched (Fab approved), patch `etc/patches/nimble-att-tx-no-panic.patch` — reapply after every IDF update.
- Adv broadcast `CONFIG_FUGU_WITH_BLE_ADV` (ea394e5): 17 B mfr record (0xFFFF, magic 0xF7), `tele.conf adv_ms`; never enable `CONFIG_BT_NIMBLE_EXT_ADV` (legacy adv then silently ENOTSUP). Address type must match BLEDevice's (b2d60f4). Fleet fragment is without BLE_ADV since 11-18.
- rpi bridge `fugu-ble-bridge.service` (`etc/deploy_ble_bridge_rpi.sh`, uses fl4p/bluek); see [[reference_bench_rigs]]. Open: no-WiFi boot mis-archives today as yesterday in DailyEnergyMeter until set-time.

## Dashboard
2026-11-17..19. `etc/config-tool/dashboard.html` + `teledec.js`, Web Bluetooth or MQTT-over-WS; published at `/tools/dashboard.html`. Firmware: `P_max_today` (PersistentState must stay 32 B), `sweep-curve`, optional `CONFIG_FUGU_WITH_TELE_MQTT` lease (`tele-mqtt <s>` -> `pv/tele/<host>`, default off). Charger state line 6d2f36e5.
- Fab does not want the private CA on his devices (11-19). Use GitHub Pages `…/tools/dashboard.html?mqtt=wss://192.168.1.200:8884&user=pv` after a one-time cert click-through on https://192.168.1.200:8884/ (Chrome also asks Local Network Access), or the no-TLS copy `http://192.168.1.200/local/fugu/dashboard.html?mqtt=ws://192.168.1.200:1885&user=pv` (files in `/var/lib/homeassistant/homeassistant/www/fugu/`; HA caches /local/ 31 days — hard reload). Tailscale: 100.64.0.10. TLS recipe: doc/lab/havan-mqtt-tls.md.
- Energy meter froze in float at 16 kWh (E_today 0 for weeks, history wrong); double since c9078ee4; `systemRestart()` force-commits stats (e31ab42d).
- macOS keeps stale GATT/names after a GATT change: `blueutil -p 0 && blueutil -p 1`.
