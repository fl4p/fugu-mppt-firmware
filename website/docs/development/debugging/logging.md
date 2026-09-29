---
title: Logging
sidebar_position: 2
---

# Logging

How firmware log output (`ESP_LOGx`, `UART_LOG`, `printf_mux`) is multiplexed to the UART/USB
console, telnet, MQTT and BLE. Implemented in `src/logging.cpp`.

## Pipeline

`enable_esp_log_to_telnet()` installs `vprintf_` as the sink for every `ESP_LOGx` (via
`esp_log_set_vprintf`), saving the previous one as `old_vprintf` (the libc `vprintf` → UART/USB
console). From `vprintf_`:

- **Core 1 (RT core)**, when `deferLogs` is set or the caller can't yield (ISR / critical section):
  the line is pushed onto a queue (`enqueue_log`) and drained later on core 0
  (`flush_async_uart_log`, called from the network loop). The RT loop must never block in
  `uart_tx_char` (~5 ms for a 60-byte line at 115200 baud), so it never formats/writes the console
  itself.
- **The ESP-IDF `wifi` task** (detected via `pcTaskGetName`, guarded by `xPortCanYield()`): routed
  straight to `old_vprintf` (UART only), bypassing `vprintf_mux`. Its 3072 B stack can't absorb
  `vprintf_mux`'s 300 B `loc_buf` + mirror-sink frames during a connect/reconnect logging burst —
  that overflowed it and reboot-looped the device. See [Real-time latency](real-time-latency.md).
- **Everything else** (core 0 tasks; and core 1 before `deferLogs`): synchronous `vprintf_mux`.

`UART_LOG()` / `printf_mux()` follow the same core-1-defer / else-synchronous split.

## vprintf_mux — fan-out

Formats the line **once** into a stack buffer (`loc_buf[300]`, heap fallback for longer lines),
then writes that one string to every active sink:

1. UART / USB-JTAG console (always, via `old_vprintf`)
2. telnet (`log_telnet`)
3. registered callbacks (`addLogCallback`) — MQTT mirror, BLE NUS
4. boot backlog (see below)

**Format once — load-bearing.** Log emission runs on the *caller's* task stack, and a `va_list`
can't be carried to another task, so the formatting `vsnprintf` must happen there. Formatting twice
(one pass for UART, another for the mirror) overflowed the ESP-IDF `wifi` task's 3072-byte stack
during association → boot loop. Keep it to a single `vsnprintf`; the long-line heap refmt needs a
`va_copy` taken *before* the first pass (the first pass spends `argptr`).

## Async queue (core 1 → core 0)

`uart_async_log_queue` is a **single-producer / single-consumer** `ReaderWriterQueue`: the sole
producer is core 1 (`enqueue_log` asserts `xPortGetCoreID()==1`), the sole consumer is the core-0
network loop. Don't enqueue from other tasks without switching to an MPMC queue
(`moodycamel::ConcurrentQueue`). `flush_async_uart_log` drains ≤32 entries per call so it can pet
the task WDT (a saturated RT core could otherwise spam faster than UART drains); RT-side overflow
(>200 queued) is dropped.

## Sinks (addLogCallback / removeLogCallback)

Up to `kMaxLogCallbacks` (4) callbacks, guarded by `logCbMux`. A callback may itself log, so the
table is snapshotted under the lock and invoked outside it (re-entrancy safe). MQTT and BLE register
their mirrors here when they come up; `mqttLogCallback` drops lines tagged `) mqtt:` to avoid a
publish→log→publish feedback loop.

## Boot backlog + replay

Early boot logs (`setup()`, WiFi bring-up) happen before any remote sink exists — MQTT can't connect
until WiFi is up, well after setup() logs. `s_bootLog` (8 KB) captures formatted lines until the
first sink attaches; `addLogCallback` then replays the captured block to that sink in one shot and
freezes the backlog. That's how the boot sequence reaches MQTT/telnet after the fact. `vprintf_`
routes esp_log through this path from the very start of setup() so the capture sees the whole boot.

Note: a fresh console session therefore receives the entire boot backlog up front; a one-shot
client (`fugu_console.py -c`) may need a longer read window before its command's reply.

## esp_log internals (reference)

- `esp_log_write` uses `s_log_print_func`, which defaults to `vprintf`; override with
  `esp_log_set_vprintf`.
- With `ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG` enabled, `vprintf` also writes to USB
  (`vfs_console.c` / `console_write()`).
- `ARDUHAL_ESP_LOG` redefines `ESP_LOGx` to Arduino's `log_x` macros — not used here.
- `ESP_CONSOLE_USB_CDC_SUPPORT_ETS_PRINTF` enables `esp_rom_printf` / `ESP_EARLY_LOG` via USB CDC.
