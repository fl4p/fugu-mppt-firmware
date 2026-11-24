---
title: Debugging
sidebar_position: 4
---

# Debugging

The device prints every error and panic on the console. After a panic it stores a coredump in flash. You can pull
the dump over any console transport and decode it against the exact build that crashed.

## Quick start

These commands check for a coredump, download it, and decode it:

```bash
python3 etc/fugu_console.py -p $ESPPORT -c "coredump info"       # is there a dump?
python3 etc/fugu_console.py -p $ESPPORT --coredump get           # writes coredump.bin and decodes it
python3 etc/idf-devtools/elf_archive.py decode coredump.bin      # decode again later
```

`--coredump get` decodes through the ELF archive when no `--elf` is given, and falls back to
`build/fugu-firmware.elf` if the archive has no matching build.

:::warning
Treat ADC failures in the log (`ADC stall`, `Never got a sample! Please check ADC`, INA22x timeouts, `Loop latency high`) as critical: the control loop is not getting samples.
:::

## Coredumps

`coredump [info|get|erase]` inspects, streams (base64), or clears the dump on the `coredump` partition
(`CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH`, on in `sdkconfig.defaults`). Without an argument it runs `info`, which
prints the address, size, checksum state and the approximate crash time (`crashed=`, the first SNTP-synced boot
after the dump appeared; `0` if unknown). `get` prints the dump between `==COREDUMP-BEGIN size=<n>==` and
`==COREDUMP-END==` markers. It works over serial, telnet, MQTT, and BLE, so you can retrieve a backtrace without a
cable. Over BLE, `get` waits for the BLE transmit buffer to drain between lines; MQTT and telnet remain the more
reliable path for a full dump.

`crash <null|abort|stack>` deliberately panics the device to test the coredump path. Use it on the bench only.

### ELF archive

`esp-coredump` only decodes a dump against the ELF of the exact build, matched by its SHA-256. The ELF archive
keeps those ELFs:

- `idf.py flash` / `app-flash` and the OTA tools (`etc/ota.py`, `etc/ota_mqtt.py`, `etc/ota_ble.py`) archive the
  ELF after each flash, one `zstd -19` file per build under the gitignored `elf-archive/`, with a flash log in
  `index.jsonl`.
- The archive keeps the 30 most recently flashed builds (`ELF_ARCHIVE_KEEP`) plus the last 3 builds of every device
  (`ELF_ARCHIVE_KEEP_PER_DEVICE`).
- The device name comes from `$FUGU_DEVICE`, else the serial port name. `./flash.sh <name>` sets it.

`elf_archive.py` decodes dumps, lists the flash history, and extracts archived ELFs:

```bash
python3 etc/idf-devtools/elf_archive.py decode <core.bin>                  # match by SHA in the dump
python3 etc/idf-devtools/elf_archive.py decode --device <name> <core.bin>  # fallback: latest build for <name>
python3 etc/idf-devtools/elf_archive.py list                               # flash history
python3 etc/idf-devtools/elf_archive.py find --device <name> -o fw.elf     # extract an ELF
```

Builds flashed before the archive existed need their ELF passed to `esp-coredump` by hand.

## Runtime inspection

These console commands and firmware tools show the device's state while it runs:

| Tool | What it shows |
|---|---|
| `status`, `sensor` | Charger state and sensor readings |
| `tasks` | FreeRTOS tasks with minimum free stack; watch for stack overflows |
| `rt-stats` | Per-task CPU load per core over about 2 s, RT-task heap operations, the loop-rate watchdog threshold, and the longest loop gap during the command |
| `reset-lag [q]` | Resets the max loop-latency statistics (status `lag=`, telemetry `lag`) and prints the `rtcount` section timings; `q` skips the print |
| `log <tag> <level>` | Sets the ESP-IDF log level of a tag at runtime; see [Logging](logging.md#log-levels) |
| [`peek`](peek.md) | Reads memory at an address; `fugu_console.py` resolves symbols |
| [`rtcount(label)`](rtcount.md) | Function in `src/etc/rt.h` that accumulates per-section timings |
| [`rtprof`](profiling.md#rtprof) | Core-1 call-stack sampler, needs `CONFIG_FUGU_WITH_RTPROF` |
| `sprofiler` | Sampling profiler, needs OpenOCD, `CONFIG_FUGU_WITH_SPROFILER` and `pprof.conf` `sprofiler_hz` |
| `scope` | Streams raw ADC samples over TCP; view with `./etc/scope.py` |

See [Console](../../reference/console.md) for all commands and [Logging](logging.md) for how log output reaches each transport.
