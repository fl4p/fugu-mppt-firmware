---
name: project_rt_loop_latency
description: RT loop gap budget — flash writes stalled 33-65 ms; auto-suspend + IRAM set brought flat <1 ms (goal closed 11-18); Wi-Fi connect costs ~3 ms
metadata:
  type: project
---

General flash-stall mechanics: Canonical: ~/dev/kb/esp32/realtime-and-power.md (Flash writes stall both cores).

## Flash write stall (pre-auto-suspend)
2026-11-10, fry ~70 W, measured with `reset-lag` + `set-config stallprobe.conf x N`: littlefs append +1 ms
(3 → ~4 ms peak), sector erase (~1 in 10 writes) 33-65 ms. No warnings (ADC reset needs >200 ms, latency watchdog
3 starved windows). Rule then: persist only when stopped/low power. Superseded by auto-suspend below.

## Flash auto-suspend + RT code in IRAM
GOAL CLOSED 2026-11-18 (Fab: "goal reached"): flat <1 ms loop gap for a full day (build 580 max 883 us; build 608 max
881 us through dusk/night/dawn). On main since 3743b96 (11-13); deployed fleet-wide.
What it took (lessons):
- Auto-suspend alone gave 55-202 ms: the continuous ADC DMA starved because its ISR was on core 0 (cache misses);
  moved ADC DMA + I2C interrupts to the RT core via `runOnCore()`.
- IRAM set from `CONFIG_FUGU_WITH_RTPROF` (`rtprof`, `rtprof all|clear`) → `etc/rtprof_map.py <log> <elf> --lf
  main/rt_iram.lf`, `--check` verifies placement (earlier generated fragments missed COMDAT inlines, aliases, libc; fixed in the mapper — remaining limit: whole-object entries). Code the sampler cannot see goes
  in `main/rt_iram_extra.lf` (critical sections like `_i2c_hal_set_bus_timing`, `__onPinInterrupt`, asm, vtables),
  found via IRAM→flash l32r edges. `CONFIG_FREERTOS_IN_IRAM`. `RT_COLD()` (util.h) moves cold paths out of loopRT.
- RT-core log lines are captured raw and formatted on core 0 (float formatting from flash cost 34-69 ms).
- Shutdown/trip path in IRAM; start/periodic sweep waits for a running flash op.
- Notch retune of all sensors in one pass cost 1.13-1.2 ms (soft-double libm): now one sensor per sample, float-only.
- `CONFIG_BT_CTRL_RUN_IN_FLASH_ONLY` REMOVED: BLE ISR from flash → INT_WDT during a littlefs erase.
- <1 ms absolute needs `sensor.conf ina22x_conv_time_us=332` (moved from board.conf; a differing/lone board.conf copy now throws in setupSensors) + `sensor.conf lv_v/lv_i/hv_i _filt_len` ×3 (flat; fry
  stays 1100 us / ~2 ms period, see [[feedback_device_handling]] on fry baseline lag).
- `i2c_hs_freq` (src/i2c_hs.h, direct I2C0): 380 kHz 138 us/read vs Wire 165 us; HS 2.2 MHz 73 us.
- Idle CPU profile at 80 MHz made flat exceed 1 ms at the exit (1025 us); fixed by fd6ad79a rate guard
  ([[project_heap_power_budget]]).
- Don't run `rt-stats` on a converting unit (2.63 ms gap once, cause unproven).
- Slow-erase WDT: FIXED 05c236b0 (fsAuto reader yields every 20 ms, stops after 2 s, erase at idle prio); a slow erase
  now yields auto-test `fail(slow)` → caches-off. On fry+flat as 782 (11-20). INT_WDT turns suspend off by design.
Open (11-13): one unexplained fry PANIC during an OTA erase (wifi ppTask wild call); raw Iout scope capture across a loaded stop (tail shape of the zero error) not done. Related:
[[project_rectification]], [[project_crash_rootcauses]].

## Wi-Fi connect during conversion
2026-05-30 bench (vconv ~815 W): `wifi on` mid-conversion → steady ~3 ms lag (baseline 0.7 ms), no trips; fry/flat in
production ~2.9-3.8 ms then. dry_int (real internal ADC): sampler held ~744 sps through wifi off/on cycles. Budget was
the 50 ms no-sample watchdog. Auto-reconnect stays power-gated (<10 W) because real-HW risk is INA226 alert misses
([[project_ina226]], [[project_wifi_reconnect]]); manual `wifi on` is not gated.
