*this document is an LLM generated placeholder*

# fugu_crash_watch

Follows the fleet console log (written by `mqtt_console.py`; lines without the `time device:` prefix continue the
previous line's MQTT payload) and reports per device:

- reboots (the esp_log ms counter in `I (N)` lines goes back by > 60 s): it then runs `bootinfo` (read-only) and
  reports a reset reason other than SW (POWERON/EXT as warning), or `reset-unknown` when bootinfo fails 3×
- `coredump: new dump` (then pulls it with `fugu_console.py --coredump get` after 60 s into `coredumps/<dev>-<UTC>/`)
- ADC stall, `Never got a sample`, `ina22x: N timeout!`, `Loop latency high`, abort/assert/pbuf lines
- no log line for `FCW_SILENCE_S` (default 300 s), and when lines come back

Events go to `events.jsonl` and, with `FCW_DSN` set, to GlitchTip (one issue per device + signature, 10 min dedup). GlitchTip
emails alerts only for projects on a team with members (the alerting projects use `mail-alerts`).
Decode a pulled dump on a machine with the ELF archive: `python3 etc/idf-devtools/elf_archive.py decode <coredump.bin>`.

Install (user unit, needs linger): copy `fugu_crash_watch.py`, `fugu_console.py`, `fugu/`, `protected-devices`,
`idf-devtools/peek_symbols.py` and an `mqtt.env` under `<dir>/etc/`, and `src/etc/con_guard.h` under
`<dir>/src/etc/` (without it the console's lock guard treats `bootinfo` as state-changing and refuses it), put `FCW_DSN=...` in `<dir>/env`, then a
`systemd --user` service running `python3 -u <dir>/etc/fugu_crash_watch.py` with `EnvironmentFile=<dir>/env`,
`Restart=always`.
Test the matcher: `FCW_DIR=/tmp/x FCW_CONSOLE=<stub> FCW_BOOT_DELAY_S=0.1 FCW_TEST_WAIT=2 python3 fugu_crash_watch.py --test < sample.log`.
