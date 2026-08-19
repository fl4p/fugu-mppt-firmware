---
name: pcnt-new-channel-forces-pullup
description: pcnt_new_channel() unconditionally enables the GPIO pull-UP and disables the pull-down — wrong bias for the wsync coupling network
metadata:
  type: project
---

`pcnt_new_channel()` calls `gpio_pullup_en()` + `gpio_pulldown_dis()` + `gpio_func_sel(GPIO)` +
`gpio_input_enable()` on `edge_gpio_num`, unconditionally
(`esp_driver_pcnt/src/pulse_cnt.c:826-830`, IDF 5.5, verified 2026-08-19).

**Why:** the wired-sync receiver bias assumes the internal **pull-down** (see
[[wsync-coupling-network-numbers]] — 0.64 V idle with R_PD 45k). A pull-up parks the AC-coupled
node near mid-rail, where threshold chatter becomes a sync-reload storm — exactly what the
comment at `src/pwm/mcpwm.h:271` describes for the legacy U0RXD pull-up.

**How to apply:** any PCNT channel on a biased/AC-coupled line must call
`gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY)` immediately AFTER `pcnt_new_channel()`. The
existing `src/buck.h:184-207` counter survives only by luck of ordering: `initSyncIn()` runs
after it and re-applies `GPIO_PULLDOWN_ONLY` (`src/pwm/mcpwm.h:271`). A standalone PCNT probe
has no such rescue. Related: [[wsync-usb-pin-plan]].
