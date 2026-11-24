---
title: Logging
sidebar_position: 2
description: "src/logging.cpp multiplexes firmware log output (ESP_LOGx, UART_LOG, printf_mux) to the UART/USB console, telnet, MQTT, BLE and the other console sinks."
---

# Logging

*Original hand-written note: [doc/Logging.md](https://github.com/fl4p/fugu-mppt-firmware/blob/main/doc/Logging.md).*

`src/logging.cpp` multiplexes firmware log output (`ESP_LOGx`, `UART_LOG`, `printf_mux`) to the
UART/USB console, telnet, MQTT, BLE, and the other console sinks.

## Pipeline

`enable_esp_log_to_telnet()` installs `vprintf_` as the sink for every `ESP_LOGx` (via
`esp_log_set_vprintf`). It saves the previous sink as `old_vprintf`, which is the libc `vprintf`
that writes to the UART/USB console. `vprintf_` routes each line by its caller:

- **Core 1 (RT core)**, when `deferLogs` is set or the caller can't yield (ISR / critical section):
  `vprintf_` captures the line unformatted into a ring (`enqueue_log`), and core 0 formats and
  writes it later (`flush_async_uart_log`, called from the network loop). The RT loop must never
  block in `uart_tx_char` (~5 ms for a 60-byte line at 115200 baud), and it never formats, allocates
  or writes the console itself. See [RT log ring](#rt-log-ring).
- **The ESP-IDF `wifi` task** (detected via `pcTaskGetName`, guarded by `xPortCanYield()`): the line
  goes straight to `old_vprintf` (UART only) and bypasses `vprintf_mux`. Its 3072 B stack can't
  absorb `vprintf_mux`'s 300 B `loc_buf` plus the mirror-sink frames during a connect/reconnect
  logging burst. That overflowed the stack and reboot-looped the device. See
  [Real-time latency](real-time-latency.md).
- **Everything else** (core 0 tasks, and core 1 before `deferLogs`): the line goes through
  `vprintf_mux` synchronously.

`UART_LOG()` and `printf_mux()` follow the same split: they defer on core 1 once `deferLogs` is set
and run synchronously elsewhere. `loggingEnableDefer()` sets `deferLogs` just before the RT loop
starts, so start-up messages on core 1 still print immediately.

## vprintf_mux — fan-out

`vprintf_mux` formats the line once into a stack buffer (`loc_buf[300]`, with a heap fallback for
longer lines). It then writes that one string to every active sink:

1. UART / USB-JTAG console (always, via `old_vprintf`)
2. telnet (`log_telnet`)
3. registered callbacks (`addLogCallback`): MQTT mirror, BLE NUS console, USB CDC console, TLS console
4. boot backlog (see below)

The line must be formatted only once. Log emission runs on the *caller's* task stack, and a
`va_list` can't be carried to another task, so the formatting `vsnprintf` must happen there.
Formatting twice (one pass for UART, another for the mirror) overflowed the ESP-IDF `wifi` task's
3072-byte stack during association and caused a boot loop.

Keep it to a single `vsnprintf`. The long-line heap refmt needs a `va_copy` taken *before* the
first pass, because the first pass spends `argptr`.

## RT log ring

Deferred lines go into `s_rtLog`, a fixed 16-slot lock-free multi-producer ring (`MpscRing`,
`src/etc/mpsc_ring.h`). It never allocates. `enqueue_log` is `IRAM_ATTR`, masks interrupts on its
core from claiming a slot to publishing it, and accepts producers on either core. The sole consumer
is the core-0 network loop.

On Xtensa, a slot stores the line unformatted: the format pointer, the `va_list` argument words and a
timestamp. Formatting (float formatting alone costs about 0.4 ms and runs from flash) happens on core 0.
`%s` arguments that do not point into flash are copied into the slot (80 bytes in total) while they
are still valid. During a flash operation the format string itself cannot be read, so such
arguments print as `?`.

When the ring is full, the producer drops the line. ESP_LOG info, debug and verbose lines are given
up first. The flush reports the total as `<n> RT log lines dropped (ring full)`.

`flush_async_uart_log` formats up to 32 lines per call into a static buffer and frees their slots,
then writes them to the sinks, so it can pet the task WDT between calls.

## Sinks (addLogCallback / removeLogCallback)

`logCbMux` guards a table of up to `kMaxLogCallbacks` (4) callbacks. A callback may itself log, so
the code snapshots the table under the lock and invokes the callbacks outside it, which makes it
re-entrancy safe. MQTT, BLE, USB CDC and the TLS console register their mirrors here when they come
up. A full table drops the new sink with `log callback table full, dropping`.

The MQTT mirror publishes every line to `pv/log/<hostname>` while the client is connected; console
commands arrive on `pv/log/<hostname>/cmd`. `mqttLogCallback` drops lines tagged `) mqtt:` to avoid a
publish→log→publish feedback loop, and drops lines logged from inside its own publish call.

## Boot backlog + replay

Early boot logs (`setup()`, WiFi bring-up) happen before any remote sink exists. MQTT can't connect
until WiFi is up, well after setup() logs. In builds with networking, `bootLogBegin()` at the top of
`setup()` allocates `s_bootLog` (8 KB), which captures formatted lines until a sink takes it.

The backlog captures esp_log lines from `enable_esp_log_to_telnet()` (after service registration)
on, and `UART_LOG` lines from the start of `setup()`. For earlier ESP_LOG output, use the serial
console. Lines tagged `) mqtt:` are not captured.

The backlog has these limits:

- Only a sink registered with `replayBootLog=true` takes it, and only the first one; in practice the
  MQTT mirror on its first connect. The console sinks (BLE, USB CDC, TLS console) register with
  `false` and leave it alone, because the replay would flood their transport. Telnet does not
  receive it.
- Capture stops when the buffer is full.
- Without a replay sink, the buffer is freed 30 minutes after boot.

A one-shot MQTT client (`fugu_console.py --mqtt … -c`) may need a longer read window before its
command's reply, because the replay arrives first.

## Retained lines

`logRetained()` logs a line to every sink and keeps it for 120 s (256 bytes in total). Every log
callback and telnet client that attaches within that window receives it. A console command that
runs longer than 5 s reports its `OK: <cmd>` / `ERR: <cmd>` marker this way, so a client that
reconnects after a dropped link still sees the result.

## Log levels

Two console commands set log levels at runtime:

- `log <tag> <error|warn|info|debug|verbose|none>` sets the ESP-IDF level of one tag (`*` = all
  tags). The level is not saved and resets on reboot.
- `svc log <name> <error|warn|info>` sets the level of a service's tag and saves it as
  `log_level` in that service's conf file. `svc list` shows the current level.

Debug and verbose lines print only if the build's `CONFIG_LOG_MAXIMUM_LEVEL` includes them.

## esp_log internals (reference)

These ESP-IDF and Arduino details affect where log output ends up:

- `esp_log_write` uses `s_log_print_func`, which defaults to `vprintf`. Override it with
  `esp_log_set_vprintf`.
- With `ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG` enabled, `vprintf` also writes to USB
  (`vfs_console.c` / `console_write()`).
- `ARDUHAL_ESP_LOG` redefines `ESP_LOGx` to Arduino's `log_x` macros (not used here).
- `ESP_CONSOLE_USB_CDC_SUPPORT_ETS_PRINTF` enables `esp_rom_printf` / `ESP_EARLY_LOG` via USB CDC.
