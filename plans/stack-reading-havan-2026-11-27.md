*this document is an LLM generated placeholder*

# Scheduled stack reading on havan (2026-11-28)

Day-long `tasks` reading for [stack-trim-proposal-2026-11-27.md](stack-trim-proposal-2026-11-27.md), run on havan
so it needs no agent session or awake Mac.

## Setup

- Timer: systemd user unit files in `~/.config/systemd/user/` on havan (user `fab`, linger on), enabled:
  `fugu-stack-reading.timer`, `OnCalendar=2026-11-28 10:47:00` (Europe/Lisbon), `Persistent=true` (runs at the next
  boot if havan was down at that time; real unit files, so a reboot keeps them).
- Service: `fugu-stack-reading.service` (oneshot) runs `/usr/bin/python3 %h/stack-reading/read.py fry flat`.
- Script `~/stack-reading/read.py`: paho-mqtt (1.6.1 on havan), sends `bootinfo`, `heap`, `tasks` to
  `pv/log/<dev>/cmd` and collects `pv/log/<dev>` until the exact `OK: <cmd>`/`ERR: <cmd>` line (15 s timeout).
  The firmware splits log output across MQTT messages, so the script joins them and splits on newlines. `V=`
  status lines are dropped; other log lines arriving meanwhile stay in the block. Read-only commands only.
- Credentials: `~/stack-reading/mqtt.env` (copy of `etc/mqtt.env`, mode 600, directory 700).
- Output: `~/stack-reading/stack-<YYYYMMDD-HHMM>.txt`, one `=== <dev> <cmd> ===` block per command.

Tested 11-27 on fry (10:34, first version) and on flat (12:45, current version: all three replies, 17 task rows).

## Commands

```bash
ssh havan-ts 'systemctl --user list-timers fugu-stack-reading.timer --no-pager'    # pending?
ssh havan-ts 'ls -t ~/stack-reading/stack-*.txt | head -1 | xargs cat'              # result
ssh havan-ts 'systemctl --user disable --now fugu-stack-reading.timer'             # cancel
ssh havan-ts 'cd ~/stack-reading && python3 read.py fry flat'                       # run now
```

## After the run

Paseo schedule `da22db9a` ("fugu stack-trim: day-long reading analysis", one run, 2026-11-28 11:07 Europe/Lisbon,
expires 11-30) starts a Claude agent in this repo that does the steps below, copies the output to
`plans/stack-readings/` and removes the units and `~/stack-reading/` from havan in every case, also when the reading failed.

Compare the per-task minimum STKFREE_B with the "free left" column of the proposal, note the uptimes (another OTA in
between makes it a short reading) and add the result to the proposal. Afterwards remove `~/stack-reading/` on havan,
since it holds a credentials copy.
