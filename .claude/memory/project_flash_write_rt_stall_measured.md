---
name: project_flash_write_rt_stall_measured
description: "littlefs writes stall the RT loop on fry: ~1 ms per append, 33-65 ms when an erase happens (measured 2026-11-10)"
metadata:
  node_type: memory
  type: project
  originSessionId: 5705d5de-ac35-448c-b5a3-418631e9f63a
  modified: 2026-11-10T14:29:42.741Z
---

Measured on fry 2026-11-10 ~14:30 (about 70 W, MPPT, DCM/CCM) with `reset-lag` + `set-config stallprobe.conf x N` (a scratch file the firmware never reads; it was left on fry's littlefs, since there is no rm command):
- Plain littlefs append commit: peak loop lag rises from about 3 ms (baseline) to 3.9-4.3 ms.
- About once per 10 writes (metadata compaction, i.e. a sector erase): peak lag of 33-65 ms, 64.9 ms worst of 16 batches. No ADC-stall or latency warnings: the ADC reset needs >200 ms and the latency watchdog needs 3 starved windows.

**Why:** this confirms review finding #5 (core-0 flash writes stall core 1, CONFIG_SPI_FLASH_AUTO_SUSPEND off).
**How to apply:** persist anything written while converting (rect-cal results, stats) only when conversion is stopped or power is low, or after flash auto-suspend is proven on the chip (boot-time flash-probe work, 11-10). Related: [[project_flat_mcpwm_sweep_loop_rect_offset]].
