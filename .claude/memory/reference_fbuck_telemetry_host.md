---
name: fbuck/fboost telemetry via rpi.local BLE relay
description: fbuck/fboost rig telemetry path: BLE ADV → rpi.local (192.168.178.26) influx_binary_proxy → influxdb-udp-relay → influx.fabi.me:8086/open_pe; rpi.local must be up; query with etc/influx.env creds
created: 2026-08-10T18:57:02.654Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: reference
  originSessionId: ses_012e2f1d9ffek4jj6u5q6fm41U
---

fbuck/fboost rig telemetry goes via a **BLE relay on rpi.local** → InfluxDB at `influx.fabi.me:8086` (SSL, db `open_pe`, user `openpe` — same as `etc/influx.env`). NOT to tm.fabi.me/ha_van like fry/flat.

**Relay chain (all on rpi.local, 192.168.178.26):**
1. `/opt/fugu-ble-bridge/influx_binary_proxy.py --adv --forward-udp 127.0.0.1:8086` — decodes BLE ADV telemetry records from fbuck/fboost
2. `/home/fab/influxdb-udp-relay/main.py` (systemd service `influxdb-udp-relay`, listens :8086 UDP) — batches and forwards to `influx.fabi.me:8086` (SSL, db `open_pe`). Config: `/home/fab/influxdb-udp-relay/configuration.yaml`

**Network notes:**
- rpi.local resolves to TWO IPs: 192.168.178.35 (was down 2026-08-10) and 192.168.178.26 (alive). Use .26 or ssh `fab@192.168.178.26`.
- The firmware's `tele.conf` has `influxdb_host=192.168.178.180` (UDP), but .180 was unreachable — the BLE relay path is what actually gets data into the database.
- fboost has NO tele.conf — its telemetry is solely via the BLE relay path.

**Querying:**
```
curl -sG https://influx.fabi.me:8086/query --data-urlencode "u=openpe" --data-urlencode "p=0ffgrid" --data-urlencode "db=open_pe" --data-urlencode "q=SELECT * FROM mppt WHERE device='fbuck' ORDER BY time DESC LIMIT 5"
```
- Measurement `mppt`, tag `device=fbuck`/`fboost`. Also: `smart_shunt` (rig loggers: HP3458A, DMM6500, BLE_ESP32_INA228_2/3, BLE_TMP117/2), `logger_cfg`.
- fry/flat: firmware → havan:8086 (UDP relay) → tm.fabi.me:8086 / db `ha_van` (see [[reference_mppt_telemetry_influx]])
