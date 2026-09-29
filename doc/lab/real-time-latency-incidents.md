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

