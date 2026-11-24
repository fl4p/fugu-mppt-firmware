---
name: project_crash_rootcauses
description: Decoded fry/flat/bench crash root causes (May-Nov 2026) — bad_alloc aborts, wifi/mDNS/telnet UAFs, nested-IPC deadlock, tracker OOB, BLE controller assert, ConfFile miscompile
metadata:
  type: project
---

Common lesson: a stall/crash signature (no ADC samples, corruption, INT_WDT) says nothing about hardware until an
isolated test or a decoded coredump says so. Decode recipe: [[project_coredump]].

## May-29 fry/flat instability
2026-05-29, FIXED (`6f02574` wifi-task vprintf stack, `24e24cf` sys_evt stack, `14734cc` ADC deadlock; both on `g2e4ca6c`).
Three firmware bugs triggered by a Wi-Fi reconnect storm after a NAT-router reflash. fry's "dead ADC" was the
deadlock ([[project_adc_sampler]]), NOT damage from hot-removing the MCU board (coincidence; isolated self-test
passed). Coredump correctly showed wifi-task stack overflow (corrupted NVS-handle vtable). INA226 path unaffected.
Stack sizing details: [[project_logging_stack_budget]].

## fry "ADC error" reboot loop = bad_alloc
2026-05-29, FIXED `c295e93` (log `ADC error`+`stopAndBackoff` only when `!mppt.inBackoff()`) + `196c7d3`
(`enqueue_log` uses `new(std::nothrow)`, drops the line on OOM). Chain: ADC DMA stall → loopRT logged every tick →
`enqueue_log` `new[]` threw → `std::terminate`. flat survived same image because its DMA self-healed.
**Why:** any throwing `new` reachable from RT-loop logging is an abort landmine. **How to apply:** no throwing allocs on
the RT path; HEAP_POISONING / gpio "already installed" were red herrings. fry MAC 70:04:1d:a6:ab:30.

## teleflush bad_alloc panic (flat)
2026-06-04 PANIC after 56 h; FIXED in code (try/catch(std::bad_alloc) in `TelemetryService::flushQueue`, drops batch).
`batch += frame` → `operator new(4033)` threw with 41 KB free but no 4 KB contiguous block (poisoning + fragmentation,
heap churn during Wi-Fi disruption). Rule: every non-RT task that allocates must catch, not terminate the converter.

## `wifi off N` over telnet = pbuf UAF
2026-05-30, FIXED (build fry-brk1-69): `cmdWifi` only sets `g_app.disableWifi`; a Wi-Fi-down edge in
`networkLoopTick` runs `stopNetworkServices()` → `disconnect_wifi(true)` outside the input callback. Cause: teardown
ran inside `telnet.loop()`, then the log mirror wrote to the dead socket → `pbuf_free` through a freed pointer
(PC 0xfefefefe). Found by `fuzz_extreme`. Family: [[project_vprintf_mux]] — never destroy the transport a command
arrived on from inside its callback.

## mDNS UAF on Wi-Fi deinit
FIXED: `disconnect_wifi(bool off)` (telemetry.cpp) runs `MDNS.end()` before `WiFi.disconnect(off)`.
`WiFi.disconnect(true)` deinits the netif; mDNS group-leave then hits the freed netif on the tcpip thread →
LoadProhibited after the next "Connected to WiFi". Exposed by `wifi off <min>` / `wifi on` ([[project_wifi_reconnect]]).
**How to apply:** always go through `disconnect_wifi()`. Repro on a converter only at idle (`dc 0`); e2e
`etc/e2e-test/test_wifi_off_timeout.py`.

## gpio_install_isr_service nested-IPC deadlock
2026-05-28, FIXED (97f47a8 + follow-up). `gpio_install_isr_service()` does its own `esp_ipc_call_blocking` to the
calling core, so calling it inside `esp_ipc_call_blocking(RT_CORE, …)` deadlocks `ipc1` in setup() — bricked fry &
flat after OTA. **How to apply:** to install on a core, use a plain task pinned to it and join. Second bug: no
`ESP_INTR_FLAG_IRAM` — arduino `__onPinInterrupt` is in flash → "Cache disabled but cached memory region accessed"
during any flash write. Install with flags 0 while the dispatcher stays in flash; IRAM is now conditional on `FUGU_GPIO_ISR_IRAM` (src/main.cpp, see [[project_ina226]]). Affinity comes from the installing task.

## flat .bss corruption = Tracker table OOB
2026-06 (after MCPWM e6762ce). `pwmPowerTable`/`pwmTimeTable` sized [2048] in `src/tracker.h`, indexed by raw PWM count;
MCPWM pwmMax ~4103 → writes past the arrays into `limits.Vin_max`, `charger.termCond.p`, `bflow.panelEN` (values were
wallClockMs timestamps). Fixed 2026-06-08 by deleting the tables (write-only dead code; frees ~16 KB); flat recovered to ~465 W. Never-instantiated `AdaptiveEWMATable` had the same 2048 arrays — size before use. Scope buf[-1] and bootLogCapture fixes were red
herrings; found by bisecting with `elf-archive/index.jsonl`. See [[project_mcpwm]], [[project_tracker_sweep]].

## BLE conn-update controller assert = INT_WDT
2026-11-20 fry INT_WDT (770). btController ROM `r_assert_param` ← `r_lld_con_param_update`: peer conn update while a
local LL procedure (PHY + conn params fired back-to-back in console_ble.cpp) was open; Mac at -91 dBm after 5 failed
connects. FIXED c5854a6c (PHY, then interval, one at a time, after encryption), on fry+flat as 775; unvalidated (no
bench repro). **Why:** the BLE backup link can itself reboot a converter. **How to apply:** below ~-85 dBm use the
nimble proxy or wait, no back-to-back retries; INT_WDT after a BLE session → check this stack first.
[[feedback_device_handling]], [[project_ble_console]].

## In-mem ConfFile multi-pair miscompile
xtensa-esp-elf 14.2, -Os: `ConfFile{{…}}` with ≥2 pairs and any runtime `const char*`/`std::to_string` value throws
`std::length_error` at construction (boot loop). Single pair or all-literal lists work. Fix: build with literals, then
`ConfFile::set(k,v)` for runtime values.
