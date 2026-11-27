*this document is an LLM generated placeholder*

# Scheduled follow-ups (set 2026-11-27)

Paseo schedules (each starts a Claude agent in this repo, bypass mode, Mac must be on with Paseo running) and one
havan timer. Towards fry/flat they send read-only console commands only (`bootinfo`, `heap`, `tasks`, `wire`) and
change no settings. They commit nothing, but may write their results into plan files, `doc/research/` and memory
notes. `da22db9a` also removes the havan timer and `~/stack-reading/` (credentials copy) after the reading.
Paseo runs need the Mac awake; what Paseo does with a run missed during sleep is not checked.

| when (Europe/Lisbon) | id | what | plan |
|---|---|---|---|
| 11-28 10:47 | havan `fugu-stack-reading.timer` | `bootinfo`/`heap`/`tasks` of fry+flat to `~/stack-reading/` | [stack-reading-havan](stack-reading-havan-2026-11-27.md) |
| 11-28 11:07 | `da22db9a` | compare the day-long stack reading, update the proposal, clean up havan | [stack-trim-proposal](stack-trim-proposal-2026-11-27.md) |
| 11-30 15:13 | `3e687a03` | fry light-load efficiency before/after rect_offset −325 ns (InfluxDB) | [fry-rectoffset-efficiency-check](fry-rectoffset-efficiency-check-2026-11-26.md) |
| 12-01 17:37, 12-08 17:37 | `d9ab7fcb` | fry wire-guard baseline days, wire alerts, days above 27 A | [wire-guard-highcurrent-replay](wire-guard-highcurrent-replay-2026-11-24.md) |
| 12-01 18:23 | ~~`084b49f5`~~ | flat near-point probe, clean ≥300 W `scan near:` samples (log-based; PAUSED 11-27, superseded by `3bf63b73` below) | [scan-near-probe-data-check](scan-near-probe-data-check.md) |
| 11-30 18:23, then Mondays | `3bf63b73` | flat near-point probe from the durable archive `~/pv/near/*.jsonl` (collector `fugu-near-collector` on havan since 11-27), not the rotating log | [scan-near-probe-data-check](scan-near-probe-data-check.md) |

Already covered elsewhere: the daily peak-scan field check (Claude cron `f0984744`, another session, until 12-02).

Not scheduled (wait on hardware or a decision): bench-converter-work, hs-short-bench-tests, pv-shade-hw-test,
idle-power open items, flat's Vout filter cap (wire guard learning), BLE OTA test (bench139c unpair), dropping
`chargerRtcMigrate()`, the stack trim itself, `wire_protect=2` on fry.

Manage: `mcp__paseo__list_schedules` / `pause_schedule` / `delete_schedule`, or the Paseo app.
