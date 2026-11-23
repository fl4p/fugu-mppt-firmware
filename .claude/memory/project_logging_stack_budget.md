---
name: Logging stack budget: sys_evt, wifi, ping task overflows
description: vprintf_mux runs on the caller's stack: sys_evt/wifi/ping task overflows and how to size or bypass them
created: 2026-11-23T09:48:04.107Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: project
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
---

Logging through `vprintf_mux` runs on the **caller's** stack (a 300 B `loc_buf` plus mirror/callback
frames), so any small task that emits a log can overflow. Three confirmed instances, all reboot
loops that look like something else:

**`sys_evt` task (2304 B).** fry reboot-looped on a WiFi flap with `LoadProhibited` in
`esp_event_loop_run` ← `esp_event_loop_run_task` (the sys_evt default-loop task), dereferencing a
corrupted SLIST node — a small-task stack overflow, not a true UAF (esp_event mutex-protects register
vs run). With `enable_esp_log_to_telnet()` active, an enabled-level log from a WiFi/IP event handler
ran `vprintf_mux` on the handler task's stack. Commit `6f02574` routes **wifi-task** logs to the light
default vprintf, but that bypass keys on the task name `wifi*` and does **not** cover `sys_evt`, which
is what actually crashed. Defensive fix added alongside: `CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE=4096`
in sdkconfig.defaults (+1.8 KB RAM).

**wifi task (3072 B), twice — before and after the fix.** Hooking `esp_log` to `vprintf_mux` from the
start of `setup()` bricks any device that joins WiFi: the connect burst logs heavily and blows the
3072 B wifi stack → `A stack overflow in task wifi has been detected` → reboot loop hung in `setup()`
before any service starts, so no remote recovery (serial reflash only). The mock-ADC bench never
joins a real AP, so it booted clean and hid this. Keep `enable_esp_log_to_telnet()` **late** (after
`registerServices()`), accepting that the boot-log capture then misses the `setup()` body. That is
only belt-and-suspenders: once the hook is active, **any later reconnect** logs on the wifi task via
the mux and overflows the same way (seen on a wiped bench S3 whose boot WPA handshake timed out,
`Reason 15 4WAY_HANDSHAKE_TIMEOUT` — the retry storm reboot-looped it until a clean handshake stuck;
very likely the mechanism behind the chronic reboot-on-AP-loss). Note there is **no**
`CONFIG_ESP_WIFI_TASK_STACK_SIZE` in IDF 5.5 — the 3072 B is internal. Fixed 2026-05-29 in
`logging.cpp` (`6f02574`): `vprintf_()` checks `pcTaskGetName(nullptr)` (guarded by `xPortCanYield()`
for ISR safety) and routes the `wifi` task to the light `old_vprintf` (UART only); verified over 3
WiFi off/on cycles plus a forced reconnect burst with monotonic uptime.

**lwip's internal ping task (default 2048+extra).** The `ping` console command's
`on_ping_success/timeout/end` callbacks run in lwip's ping task, not the console task, and they call
`UART_LOG`. It overflowed after the first reply ("stack overflow in task ping"); fix is
`pc.task_stack_size = 6144`.

**Rule:** when a callback or task you don't own logs through the mux, either size its stack ≥6 KB or
marshal the data out and log from the console/net task. Related: [[project_vprintf_mux]].

**Meta-lesson from the brick:** do not OTA a bleeding-edge build (untested on real converter hardware
plus WiFi) to a live converter. Use the build a sibling already runs, dry-run first, and confirm the
after-table.
