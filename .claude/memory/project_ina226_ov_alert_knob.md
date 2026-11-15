---
name: project-ina226-ov-alert-knob
description: "board.conf ina22x_ov (INA226 ALERT as latched bus-OV alert, ISR forces MCPWM gates off) on main bd1f472; IRAM GPIO ISR service behind Kconfig FUGU_GPIO_ISR_IRAM (default off, untested)"
metadata:
  node_type: memory
  type: project
  originSessionId: 14101fa7-908e-4f8d-a01e-f15fd5d26a56
  modified: 2026-11-14T13:53:43.306Z
---

On main (2026-11-14, not pushed): bd1f472 `board.conf ina22x_ov` (V after divider, default 0 = old conversion-ready
ALERT). When set: ALERT = BOL|LEN, ISR -> `g_hwOvIsrHook` forces MCPWM gates low + `g_hwOvTrip` -> `protect()` ->
`shutdownDcdc("hw-OV")`; samples found by polling CVRF (esp_timer wake 300 us early, busy-wait <=400 us).
Fry-tested: trip under switching ([hw-OV] mid-sweep), 20/20 forced OTAs clean with knob armed, 511-513 sps,
+~0.6 ms max loop gap, 1.38 MASK_EN reads/sample. Fry left with knob 0.

9053667 + 3815a3c: GPIO ISR service with ESP_INTR_FLAG_IRAM, only with Kconfig `FUGU_GPIO_ISR_IRAM` (default n,
S3, needs rt_iram_extra.lf). AdsAlertISR made IRAM_ATTR (ICACHE_RAM_ATTR expands to nothing in arduino-esp32 3.x).
NOT device-tested on the caches-off path and NOT reviewed (review was stopped when Fab closed the session).

d53f9cb (11-14, NOT device-tested): review fixes — lost arm trips + retries once per period (ISR/hasData keyed
on ovRaw_, not ovArmed_), mismatch no longer re-arms in the same poll, armOv restores CAL/conv and never clears
g_hwOvTrip, forceShutdown always_inline. Needs a fry run before relying on it.

**Why:** OV shutdown independent of RT-loop/CPU stalls; not faster than the RT check (same ~2 ms conversion).
**How to apply:** before enabling FUGU_GPIO_ISR_IRAM: review + forced caches-off test (patch
`fugu_flash_sus_set(false)` in flash_probe.cpp, constant OV alert, 20 set-config writes, no panic). Open: enable path
can clear the ISR force for one sample. elf_archive decode auto-matched the wrong ELF once (check sha).
See [[project-flash-autosuspend-live-result]].
