*this document is an LLM generated placeholder*

# Sweep curve collector (havan)

`sweep_collector.py` stores every MPPT sweep curve into InfluxDB at full resolution. It listens on
`pv/log/+`; when a device logs `Stop sweep` or `sweep verified|flagged:` it sends the read-only
command `sweep-curve` to `pv/log/<host>/cmd` (8 s after `Stop sweep`, 2 s after the verdict line,
5 s timeout, 2 retries, again later while `sweeping=1`) and parses the `sc:` reply like
`etc/config-tool/teledec.js`. Replies triggered by anyone else (dashboard) are stored too. It sends
no other command.

Each curve goes first to a JSONL archive, `~/pv/sweeps/<UTC date>.jsonl` (one object per curve:
`device, recv_ts, t, n, mpp_V, mpp_P, sweeping, flag, land, points [[V,P],...], raw [sc: lines]`;
incomplete replies carry `complete: false, reason`), then to InfluxDB `ha_van`, measurement `sweep`:

- tags `device`, `src=fw`
- fields `i` (pair index in the reply), `V`, `P`; point 0 also `mpp_V`, `mpp_P`, `n` and, once the
  sweep has a verdict, `flag` (`""` clean, `light`) and `land` (W). No verdict: no `flag`/`land`.
- time = sweep end `t` (s) + `i` ms; receive time when `t=0`.

A curve already stored (same device and `t`, from the archive of the last 2 days) is skipped; if
it now carries a verdict that the stored one lacked, point 0 is rewritten (for `t=0` curves at the
first fetch's timestamp base, kept as `base_s` in the record).

The archive is the source of truth. A writer thread does the Influx writes, so an unreachable
Influx never delays a fetch: a failed write is retried every 60 s before anything newer, a 4xx
reply is logged and not retried. Each finished write appends `<record id> ok|rejected` to
`~/pv/sweeps/<UTC date>.written`; at start, archived records of the last 7 days without a mark are
queued again. Re-import by hand (idempotent, same timestamps; marks the records when the file is in
`--archive`):

```bash
cd ~/pv && ~/miniconda3/bin/python3 sweep_collector.py --env ~/.config/fugu-sweep-collector.env --replay sweeps/2026-11-27.jsonl
```

Only one collector may use an archive directory (`.lock`, flock). Test run beside the unit, no
commands sent and nothing written to Influx:

```bash
cd ~/pv && ~/miniconda3/bin/python3 sweep_collector.py --env fugu-tools/mqtt.env --archive /tmp/sweeps-test --no-send --no-influx --log -
```

## Install / update

Runs as the systemd unit `fugu-sweep-collector` in `~/pv` (like `fugu-mqtt-console-log`), MQTT
client id `fugu-sweep-collector-<hostname>-<random>`, log `~/pv/sweep_collector.log` (1 MB x 3). MQTT
credentials from `~/pv/fugu-tools/mqtt.env` (broker `MQTT_COLLECTOR_HOST`, default localhost);
Influx credentials from `~/.config/fugu-sweep-collector.env` (mode 0600, never commit):
`INFLUX_URL`, `INFLUX_DB`, `INFLUX_USER`, `INFLUX_PASS`.

```bash
scp etc/havan/sweep_collector.py havan-ts:pv/
scp etc/havan/fugu-sweep-collector.service havan-ts:/tmp/ && ssh havan-ts 'sudo mv /tmp/fugu-sweep-collector.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now fugu-sweep-collector && sudo systemctl restart fugu-sweep-collector'
```

Tests: `.venv/bin/python3 -m pytest etc/test_sweep_collector.py`.
