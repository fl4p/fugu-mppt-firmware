---
title: Logging
sidebar_position: 2
---

# Logging

`src/logging.cpp` multiplexes firmware log output (`ESP_LOGx`, `UART_LOG`, `printf_mux`) to the
UART/USB console, telnet, MQTT, and BLE.

## Pipeline

`enable_esp_log_to_telnet()` installs `vprintf_` as the sink for every `ESP_LOGx` (via
`esp_log_set_vprintf`). It saves the previous sink as `old_vprintf`, which is the libc `vprintf`
that writes to the UART/USB console. `vprintf_` routes each line by its caller:

- **Core 1 (RT core)**, when `deferLogs` is set or the caller can't yield (ISR / critical section):
  `vprintf_` pushes the line onto a queue (`enqueue_log`), and core 0 drains it later
  (`flush_async_uart_log`, called from the network loop). The RT loop must never block in
  `uart_tx_char` (~5 ms for a 60-byte line at 115200 baud). It formats into a heap buffer and
  queues it, but never writes the console itself.
- **The ESP-IDF `wifi` task** (detected via `pcTaskGetName`, guarded by `xPortCanYield()`): the line
  goes straight to `old_vprintf` (UART only) and bypasses `vprintf_mux`. Its 3072 B stack can't
  absorb `vprintf_mux`'s 300 B `loc_buf` plus the mirror-sink frames during a connect/reconnect
  logging burst. That overflowed the stack and reboot-looped the device. See
  [Real-time latency](real-time-latency.md).
- **Everything else** (core 0 tasks, and core 1 before `deferLogs`): the line goes through
  `vprintf_mux` synchronously.

`UART_LOG()` and `printf_mux()` follow the same split: they defer on core 1 and run synchronously
elsewhere.

## vprintf_mux — fan-out

`vprintf_mux` formats the line once into a stack buffer (`loc_buf[300]`, with a heap fallback for
longer lines). It then writes that one string to every active sink:

1. UART / USB-JTAG console (always, via `old_vprintf`)
2. telnet (`log_telnet`)
3. registered callbacks (`addLogCallback`): MQTT mirror, BLE NUS
4. boot backlog (see below)

The line must be formatted only once. Log emission runs on the *caller's* task stack, and a
`va_list` can't be carried to another task, so the formatting `vsnprintf` must happen there.
Formatting twice (one pass for UART, another for the mirror) overflowed the ESP-IDF `wifi` task's
3072-byte stack during association and caused a boot loop.

Keep it to a single `vsnprintf`. The long-line heap refmt needs a `va_copy` taken *before* the
first pass, because the first pass spends `argptr`.

## Async queue (core 1 → core 0)

`uart_async_log_queue` is a single-producer / single-consumer `ReaderWriterQueue`. The sole
producer is core 1 (`enqueue_log` asserts `xPortGetCoreID()==1`), and the sole consumer is the
core-0 network loop. To enqueue from other tasks, switch to an MPMC queue
(`moodycamel::ConcurrentQueue`) first.

`flush_async_uart_log` drains ≤32 entries per call so it can pet the task WDT. Otherwise a
saturated RT core could spam faster than the UART drains. On the RT side, lines beyond 200 queued
entries are dropped.

## Sinks (addLogCallback / removeLogCallback)

`logCbMux` guards a table of up to `kMaxLogCallbacks` (4) callbacks. A callback may itself log, so
the code snapshots the table under the lock and invokes the callbacks outside it, which makes it
re-entrancy safe. MQTT and BLE register their mirrors here when they come up. `mqttLogCallback`
drops lines tagged `) mqtt:` to avoid a publish→log→publish feedback loop.

## Boot backlog + replay

Early boot logs (`setup()`, WiFi bring-up) happen before any remote sink exists. MQTT can't connect
until WiFi is up, well after setup() logs. `s_bootLog` (8 KB) captures formatted lines until the
first sink attaches. `addLogCallback` then freezes the backlog and replays it in one shot to sinks
that ask for it.

The backlog captures esp_log lines from `enable_esp_log_to_telnet()` (after service registration)
on, and `UART_LOG` lines from the start of `setup()`. For earlier ESP_LOG output, use the serial
console.

Only the MQTT mirror receives the backlog, on each fresh attach. Telnet and BLE don't receive it.
The backlog freezes at the first sink attach, whichever sink that is. A one-shot MQTT client
(`fugu_console.py --mqtt … -c`) may therefore need a longer read window before its command's reply.

## esp_log internals (reference)

These ESP-IDF and Arduino details affect where log output ends up:

- `esp_log_write` uses `s_log_print_func`, which defaults to `vprintf`. Override it with
  `esp_log_set_vprintf`.
- With `ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG` enabled, `vprintf` also writes to USB
  (`vfs_console.c` / `console_write()`).
- `ARDUHAL_ESP_LOG` redefines `ESP_LOGx` to Arduino's `log_x` macros (not used here).
- `ESP_CONSOLE_USB_CDC_SUPPORT_ETS_PRINTF` enables `esp_rom_printf` / `ESP_EARLY_LOG` via USB CDC.
