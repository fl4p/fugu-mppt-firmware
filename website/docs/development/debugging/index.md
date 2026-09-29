---
title: Debugging
sidebar_position: 4
---

# Debugging

The device prints every error and panic on the console. After a panic it stores a coredump in flash, which can be
pulled over any console transport and decoded against the exact build that crashed.

## Quick start

```bash
python3 etc/fugu_console.py -p $ESPPORT -c "coredump info"       # is there a dump?
python3 etc/fugu_console.py -p $ESPPORT --coredump get           # writes coredump.bin
python3 etc/idf-devtools/elf_archive.py decode coredump.bin
```

:::warning
Treat ADC failures in the log (`ADC stall`, `Never got a sample! Please check ADC`, INA22x timeouts, `Loop latency high`) as critical: the control loop is not getting samples.
:::

## Coredumps

`coredump [info|get|erase]` inspects, streams (base64) or clears the dump on the `coredump` partition. `get`
works over serial, telnet, MQTT and BLE, so a backtrace can be retrieved without a cable. Over BLE, output is
truncated at about 8 KB; use MQTT or telnet for full dumps.

`crash <null|abort|stack>` deliberately panics the device to test the path. Bench use only.

### ELF archive

`esp-coredump` only decodes a dump against the ELF of the exact build, matched by its SHA-256. The ELF archive
keeps those ELFs:

- `idf.py flash` / `app-flash` and `etc/ota.py` archive the ELF after each flash, one `zstd -19` file per build under
  the gitignored `elf-archive/`, with a flash log in `index.jsonl`.
- The 30 most recently flashed builds are kept (`ELF_ARCHIVE_KEEP`).
- The device name comes from `$FUGU_DEVICE`, else the serial port name; `./flash.sh <name>` sets it.

```bash
python3 etc/idf-devtools/elf_archive.py decode <core.bin>                  # match by SHA in the dump
python3 etc/idf-devtools/elf_archive.py decode --device <name> <core.bin>  # fallback: latest build for <name>
python3 etc/idf-devtools/elf_archive.py list                               # flash history
python3 etc/idf-devtools/elf_archive.py find --device <name> -o fw.elf     # extract an ELF
```

Builds flashed before the archive existed need their ELF passed to `esp-coredump` by hand.

## Runtime inspection

| Tool | What it shows |
|---|---|
| `status`, `sensor` | Charger state and sensor readings |
| `tasks` | FreeRTOS tasks with minimum free stack; watch for stack overflows |
| `rt-stats` | Per-task CPU load per core |
| `reset-lag` | Resets the max loop-latency statistic and prints `rtcount` section timings |
| [`peek`](peek.md) | Reads memory at an address; `fugu_console.py` resolves symbols |
| `rtcount(label)` | Macros in `src/etc/rt.h` that accumulate per-section timings |
| `sprofiler` | Sampling profiler, needs OpenOCD and `CONFIG_FUGU_WITH_SPROFILER` |
| `scope` | Streams raw ADC samples over TCP; view with `./etc/scope.py` |

See [Console](../../reference/console.md) for all commands and [Logging](logging.md) for how log output reaches each transport.
