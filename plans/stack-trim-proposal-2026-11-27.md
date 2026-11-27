*this document is an LLM generated placeholder*

# Task stack trim (proposal, not implemented)

Goal: return internal heap on the fleet build (ESP32-S3, BLE + Wi-Fi). flat's min-ever free heap reached 18.7 KB
on 11-24 (DMA 11.0 KB). Readings: `tasks` STKFREE_B (bytes never used since boot), fleet sdkconfig of 11-27.

## Readings

| task | configured | where set | free fry 11-24 | free flat 11-24 | free fry 11-27 | free flat 11-27 | min free | used max |
|---|---|---|---|---|---|---|---|---|
| loopRt | 8192 | `main.cpp` xTaskCreate | 6464 | 6308 | 6312 | 6312 | 6308 | 1884 |
| loopTask | 8192 | `CONFIG_ARDUINO_LOOP_STACK_SIZE` | 4504 | 4532 | 4524 | 4564 | 4504 | 3688 |
| mqtt_task | 8192 | `mqtt.cpp` task.stack_size | 5828 | 5200 | 5852 | 5772 | 5200 | 2992 |
| esp_timer | 3584 | `CONFIG_ESP_TIMER_TASK_STACK_SIZE` | 3040 | 2992 | 3040 | 3040 | 2992 | 592 |
| teleflush | 4096 | `telemetry_service.h` | 2792 | 2752 | 2680 | 2680 | 2680 | 1416 |
| sys_evt | 4096 | `CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE` | 3212 | 2700 | 3340 | 3328 | 2700 | 1396 |
| mdns | 4096 | `CONFIG_MDNS_TASK_STACK_SIZE` | 2324 | 2756 | 2328 | 1880 | 1880 | 2216 |
| Tmr Svc | 2048 | `CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH` | 1360 | 1360 | 1360 | 1360 | 1360 | 688 |
| tcpip | 3072 | `CONFIG_LWIP_TCPIP_TASK_STACK_SIZE` | 1680 | 1680 | 1728 | 1764 | 1680 | 1392 |
| nimble_host | 4096 | `CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE` | 1872 | 1056 | 1880 | 1748 | 1056 | 3040 |
| IDLE0/1 | 1536 | `CONFIG_FREERTOS_IDLE_TASK_STACKSIZE` | 704 | 704 | 704 | 704 | 704 | 832 |
| ipc0/1 | 1280 | `CONFIG_ESP_IPC_TASK_STACK_SIZE` | 368 | 352 | 368 | 360 | 352 | 928 |

Uptimes were short: 11-24 fry 21 min / flat 90 min, 11-27 fry 38 min / flat 3 min. A day-long reading is pending
(see Next steps). The values agree within ~150 B between devices and days, except flat's nimble_host and mdns.

## Proposed sizes

| task | now | proposed | saved | free left at worst reading | reason |
|---|---|---|---|---|---|
| loopRt | 8192 | 6144 | 2048 | ~4.2 KB | RT path is shallow; keep room for a caught exception's unwind and `stopAndBackoff` logging |
| loopTask | 8192 | 6656 | 1536 | ~3.0 KB | runs every console command; rare ones (`coredump get`, `ota`, conf writes) not yet measured |
| esp_timer | 3584 | 2560 | 1024 | ~2.0 KB | callbacks are small; the boot watchdog and Arduino timers log through vprintf_mux |
| teleflush | 4096 | 3072 | 1024 | ~1.6 KB | an unguarded bad_alloc in flushQueue panicked here before; keep ≥1.5 KB |
| Tmr Svc | 2048 | 1536 | 512 | ~0.8 KB | few software timers |
| **total** | | | **6144** | | |

Not touched:

- **mqtt_task** (5.2 KB free): `mqtt.cpp` notes that ~6 KB overflowed in the transport-error storm on Wi-Fi teardown.
  That case is not in these readings, so the 8 KB stays.
- **sys_evt**: overflowed before through vprintf_mux on the caller's stack (logging stack budget note).
- **nimble_host, tcpip, mdns, IDLE, ipc, wifi, btController, arduino_events**: low or variable margins, or vendor-set.

My earlier estimate of 10–12 KB assumed trimming mqtt_task and sys_evt too. With their history, ~6 KB is the
safe figure.

## Risks

- STKFREE_B is the count of bytes still holding the 0xa5 fill from the low end of the stack (`src/etc/perf.h`),
  in bytes. A large local buffer that is declared but only partly written counts its unwritten part as free, so
  it can understate use, most likely in loopTask (console commands with big buffers).
- High-water only covers the paths that ran. Overflow detection is the FreeRTOS canary
  (`CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY`), checked at context switch: it panics (coredump, reboot) rather
  than corrupting silently in most cases, but an overflow of loopRt stops the converter.
- loopRt is the critical one. A 2 KB cut still leaves 2× its measured use.

## Next steps

1. Day-long `tasks` reading on fry and flat (read-only) once both have ~24 h uptime. Scheduled on havan: systemd user
   timer `fugu-stack-reading.timer` (2026-11-28 10:47 local) runs `~/stack-reading/read.py fry flat` (paho, MQTT console,
   `bootinfo`/`heap`/`tasks`), output `~/stack-reading/stack-<date>.txt`.
2. On bench139c with the trimmed build and `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK=y` (catches an overflow at the
   write, not only at the next context switch): exercise the rare paths and read `tasks` after each: `coredump get`,
   `ota <url>` (download, then reboot), `wifi off`/`on` (transport-error storm), `measure-coil` start,
   a forced protection backoff, `heap`/`tasks`/`wire`/`status`, a BLE console session, Improv RPC.
3. If every task keeps ≥ the "free left" column above: fleet build, bench, then fry, then flat
   (`ota_mqtt.py -n` first), and a `tasks` + `heap` reading after a day.
