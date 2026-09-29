*this document is an LLM generated placeholder*

# Real-time latency: device incidents

Internal lab notes, not published. Extracted verbatim from the pre-scrub `doc/dev-notes/Real-Time Latency.md` lines 367–426 (commit 695feee), with top-level headings demoted one level; the generic lessons are in `website/docs/development/debugging/real-time-latency.md`.

## Console commands trip the loop-latency watchdog (fry, 2026-05-28)

After the boot deadlock and the `ESP_INTR_FLAG_IRAM` cache-error were fixed, fry booted and ran, but
cycled `Loop latency high (<200 Hz), shutdown!` → `stopAndBackoff(5s)` → re-sweep + recalibrate. This
is the same symptom long attributed to INA226 alert misses, but the field data points elsewhere.

Quantified from the MQTT log over a clean hour (no flash writes involved):

- ~7 shutdowns/hour. **No** cache panics, **no** reboots — the converter just keeps interrupting itself.
- sps median 511 (healthy ~90% of the time); ~10% of 1 s windows dip < 200.
- persistent max loop lag ~3 ms vs ~2 ms nominal at 511 sps — already near the edge.

**Root cause — 7/7 correlation: every shutdown fired ~1 s after a console command** (`hostname`, `ip`,
`uptime`, `getc`). The pure in-memory commands (`ip`/`uptime` read no flash) trip it just as the flash
one does, so it is **not** the flash read — handling a command on core0 stalls the RT loop on core1
below the 200 Hz floor. Same shared-lock contention as *Deferred logging still mallocs on the RT core*
above: the command's `received serial command` log + its response + the `OK:` marker run through the
console mux and allocate, holding the heap lock long enough that core1's RT path stalls for a whole
watchdog window. Net effect: a discovery / health poller that sends `ip`/`hostname`/`uptime` shuts the
converter down on **every** poll. fry is stable when it is not polled.

Fixes:

- **APPLIED:** the loop-latency watchdog now requires the low-sps condition to persist across 3
  consecutive windows before `stopAndBackoff`, so a one-off core0 stall (a poll) can't trip it
  (`lfWatchdog`, commit 97f66f2). Per-sample OV/OC protection is unaffected. This is a mitigation.
- Still open (removes the contention at the source): get the log-queue allocation off the RT path
  (preallocated ring, as in *Deferred logging* above).
- Still open: stop the poller from hitting these devices' console, or have lightweight commands
  (`ip`/`uptime`) avoid the heavy logging/alloc path.

## Boot-log backlog → MQTT, and the wifi-task stack trap

To make boot debuggable remotely, `logging.cpp` captures early log lines into an 8 KB buffer
(`s_bootLog`) and replays them to each sink as it attaches (`addLogCallback`) — so MQTT, which can't
connect until WiFi is up (well after `setup()`), still gets the boot sequence in `pv/log/<host>`. The
buffer freezes on first attach; `") mqtt:"`-tagged lines are skipped so the one-shot replay isn't
dropped by `mqttLogCallback`'s own filter.

**Trap that bricked flat (2026-05-28):** to capture the *`setup()` body* (not just post-setup), the
`esp_log → vprintf_` hook (`enable_esp_log_to_telnet`) was moved to the *start* of `setup()`. That
routes the **wifi task**'s connect-time logging burst through `vprintf_mux`, whose `loc_buf[300]`
stack buffer (plus `vsnprintf` + callback frames) overflows the wifi task's **3072-byte stack** →
`***ERROR*** A stack overflow in task wifi has been detected` → reboot loop, hung *before* any
service starts (no telnet / MQTT / BLE → serial reflash only). The mock-ADC bench never associates
with a real AP, so it booted clean and hid the bug; the live converter (flat) didn't.

**Fix (2026-05-29):** `vprintf_()` now detects the wifi task (`pcTaskGetName`, guarded by
`xPortCanYield()` so it's never called from an ISR) and routes it to the light default `old_vprintf`
(UART only), bypassing `vprintf_mux` entirely. The heavy 300 B-buffer + mirror path never runs on the
3072 B wifi task, so connect *and reconnect* bursts are safe. This also closed a worse case the
late-hook ordering missed: **post-setup reconnects** (AP loss / a slow WPA handshake) overflowed the
same way once the hook was active — the likely mechanism behind the chronic converter reboot-on-AP-loss.
Verified on hardware: 3× wifi off/on + a forced `init→auth→assoc→run` burst, no overflow, uptime
monotonic. (Note: there is **no `CONFIG_ESP_WIFI_TASK_STACK_SIZE`** in IDF 5.5 — the 3072 B is internal.)

Keeping `enable_esp_log_to_telnet()` **after** `registerServices()` is now belt-and-suspenders, not
load-bearing. Generally: any small-stack system task (wifi 3072 B) that logs through `vprintf_mux`
risks this — keep the heavy 300 B-buffer formatting path off those tasks.


## Moved from the published page (2026-09-29)

Verbatim from `website/docs/development/debugging/real-time-latency.md` at commit 8f90384 (page last changed in d4b6894), headings demoted one level. The generic lessons stay on the published page. The page's condensed "Console commands trip the loop-latency watchdog" and "Boot-log backlog" sections are not repeated here; their full narratives are above. Values below are as written at the time; the no-sample
timeout is 250 ms in the current code (`kNoDataTimeoutUs`, `src/adc/adc_esp32_cont.h`), not >1 s.

### Internal-ADC watchdog landmines (page lines 83-110)

**Landmine — a no-sample watchdog must not gate the read() that feeds it.** `ADC_ESP32_Cont` has a
no-sample watchdog (`isGood()` returns false when the DMA delivered nothing for >1 s) so a stalled
internal ADC halts the converter instead of running MPPT on a stale Vin. But `read()` is the *only*
place that drains the DMA ring **and** refreshes the watchdog's `lastDataUs_`. An early version of
`ADC_Sampler::_updateAdc` checked `isGood()` *before* `read()` and returned `AdcError` on a stale
flag — so a single transient >1 s gap (e.g. a WiFi-reconnect storm starving the RT loop) latched the
ADC dead forever: the gate blocked the only call that could clear it, and `resetPeripherals` couldn't
reliably break out. Fix: for the `StreamedCallback` backend, **drain `read()` first** (a live DMA
self-clears), then report `AdcError` from the watchdog afterwards. General rule: a liveness watchdog
must never sit in front of the operation that proves liveness.

**…but drain-first was necessary-not-sufficient — the real boot `ADC error` was `wait()` starving
`read()`.** The reorder above still left every device tripping `E (….) main: ADC error` a second or
two after boot. Root cause was *not* in the ADC code at all but in `TaskNotification::wait()`
(`src/etc/rt.h`): it returned `ulTaskNotifyTake(pdFALSE, …) == 1`, i.e. true only when *exactly one*
notification was pending. At boot `loopRT` arms the watchdog in `start()`, then sits in the
`delay(1000)` under `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS` before the drain loop spins up — so the
conv-done ISR piles up *thousands* of notifications. `wait()` then returns false (`count != 1`),
`hasData()` is false, `read()` is **never called**, `lastDataUs_` never refreshes, and the watchdog
trips on its first `isGood()` (instrumented: `reads=0 hits=0 stale≈1100ms`). The drain-first reorder
can't help when the thing gating the drain is `hasData()` itself. Fix: make `wait()` a proper
clear-on-exit binary semaphore — `ulTaskNotifyTake(pdTRUE, …) != 0` — so any pending count reads as
one wakeup and `read()` drains the whole ring. This also removes a latent steady-state bug (the old
`==1` dropped a sample whenever ≥2 frames queued between iterations) and matches the FreeRTOS
"as-binary-semaphore" pattern `TaskNotification` already cites. The pre-watchdog firmware
build hid this: the same burst just cost a few harmless spin iterations.
(Diagnostic gotcha: `%lld` in `ESP_LOG` corrupts args under newlib-nano — the first instrumentation
pass printed impossible values; use 32-bit `%ld`/`%lu` casts. See *Configuration* / newlib notes.)

### GPIO ISR install deadlock (page lines 187-195)

**Landmine — do not wrap `gpio_install_isr_service()` in `esp_ipc_call_blocking(RT_CORE, …)`.**
That function does its *own* internal `esp_ipc_call_blocking()` to the calling core (via
`gpio_isr_register` → `esp_intr_alloc` on the target core). Calling it from inside an IPC callback on
RT_CORE makes that core's single `ipc` worker wait on itself → **permanent deadlock in `setup()`**,
before `loopRT` even exists, so nothing reboots it. This was diagnosed via JTAG (loopTask blocked in
`esp_ipc_call_blocking`, `ipc1` blocked inside `gpio_install_isr_service`) and it silently bricked two
field units after an OTA. The correct way to run it on RT_CORE is a **short-lived task pinned to
RT_CORE** that calls `gpio_install_isr_service()` and notifies setup() when done — the `ipc` worker
stays free, the nested IPC completes, and the ISR lands on RT_CORE.

### IRAM-installed GPIO ISR cache panic (page lines 197-209)

**IRAM — do NOT install with `ESP_INTR_FLAG_IRAM`.** `attachInterrupt()` registers arduino-esp32's
`__onPinInterrupt` dispatcher, which lives in flash (not IRAM). An IRAM-installed service keeps firing
while the flash cache is disabled — i.e. during *any* flash write (coulomb/stats persist, config
save, OTA) — and then jumps into that cached dispatcher, panicking with `Cache disabled but cached
memory region accessed` (seen on a live converter the instant a flash op coincided with an INA226 alert; the
mock-ADC bench never hits it because it has no `attachInterrupt`). Install with flags `0` instead: the
alert is simply masked for the brief cache-off window. RT_CORE affinity comes from the installing
task, independent of the flag, so latency in normal (cache-enabled) operation is unchanged.

This couples to the loop-latency shutdowns seen on live converters: when INA226 alert edges are missed/late
the RT sampler starves and the latency watchdog trips `stopAndBackoff`. Lower, deterministic wake
latency (ISR local to RT_CORE) reduces that pressure — the watchdog itself is correct, the starvation
is the bug.

### Console `tasks` / `rt-stats` wedged the continuous-ADC DMA (2026-05-30)

`uxTaskGetSystemState()` (used by the `tasks` and `rt-stats` console commands) walks every TCB under
`taskENTER_CRITICAL(&xKernelLock)` — measured ~1.16 ms for 8 tasks, scaling ~linearly, so ~2 ms on a
networked converter. While core 0 holds that lock, our IRAM `conv_done` callback on core 1 spins in
`vTaskNotifyGiveFromISR()`, stalling the ADC driver ISR so it can't recycle DMA descriptors.

The IDF continuous-ADC driver keeps a *fixed* `INTERNAL_BUF_NUM = 5` frames of DMA descriptors
(independent of `max_store_buf_size`, which only sizes the software ring/pool). At the old
`conv_frame_size = 64 B` that's only 5 × ~192 µs ≈ **0.96 ms** of headroom — less than the critical
section — so the DMA ran dry and **halted**, recovering only via `resetPeripherals()` (stop+start).
This is pre-existing (a 05-28 build reboots on `rt-stats`); the 05-29 no-sample watchdog merely made it
visible. `flush_pool`/bigger `max_store_buf_size` do **not** help — the wedge is descriptor starvation,
not pool overflow.

Fix:
- **A** — `conv_frame_size` raised to 128 B (`ADC1_READ_LEN` 128→256), giving 5 × ~0.38 ms ≈ 1.9 ms of
  DMA headroom so the driver rides through the critical section. Cost: conv-done / OV-protection
  latency rises from ~192 µs to ~384 µs. (A busier converter whose critical section exceeds ~1.9 ms
  still wedges; the loopRT watchdog (B) then resets+recovers it without a converter backoff. Bump
  `ADC1_READ_LEN` to 384/512 for more headroom at the cost of more latency.)
- **B** — `loopRT` ADC watchdog unified + made transient-tolerant: a stall is reset promptly
  (~300 ms throttle) and the converter is stopped only if it persists > ~800 ms (genuine dead ADC),
  so a diagnostic-induced blip no longer trips a backoff on a live converter.
