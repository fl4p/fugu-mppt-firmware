*this document is an LLM generated placeholder*

# fugu_crash_watch

Follows the fleet console log (written by `mqtt_console.py`) and reports per device:

- non-SW resets (`reset reason: PANIC|*WDT|BROWNOUT|...`, POWERON/EXT as warning)
- `coredump: new dump` (then pulls it with `fugu_console.py --coredump get` after 60 s into `coredumps/<dev>-<UTC>/`)
- ADC stall, `Never got a sample`, `ina22x: N timeout!`, `Loop latency high`, abort/assert/pbuf lines
- no log line for `FCW_SILENCE_S` (default 300 s), and when lines come back

Events go to `events.jsonl` and, with `FCW_DSN` set, to GlitchTip (one issue per device + signature, 10 min dedup).
Decode a pulled dump on a machine with the ELF archive: `python3 etc/idf-devtools/elf_archive.py decode <coredump.bin>`.

Install (user unit, needs linger): copy `fugu_crash_watch.py`, `fugu_console.py`, `fugu/`, `protected-devices`,
`idf-devtools/peek_symbols.py` and an `mqtt.env` under `<dir>/etc/`, put `FCW_DSN=...` in `<dir>/env`, then a
`systemd --user` service running `python3 -u <dir>/etc/fugu_crash_watch.py` with `EnvironmentFile=<dir>/env`.
Test the matcher: `FCW_DIR=/tmp/x python3 fugu_crash_watch.py --test < sample.log`.
