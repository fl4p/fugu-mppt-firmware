---
title: tracker.conf
sidebar_position: 7
---

# tracker.conf

`tracker.conf` configures the MPPT tracker. It has the following keys.

| key                 | unit | type  | default | description                                                          |
|---------------------|------|-------|---------|----------------------------------------------------------------------|
| `target_duty_cycle` |      | float | 0.0     | Fixed duty cycle fraction (0–1 of pwmMaxDriver); if > 0 enters manual PWM mode with hard-fixed duty (no PD control, no MPPT tracking) |
| `sweep_speed`       |      | float | 4.0     | Global-sweep speed; scales both per-tick cap and limit-tracking gain (clamp ≥ 0.1) |
| `pv_substrings`     |      | byte  | 0       | Bypass-diode substrings in series (for example 2 modules × 3 diodes = 6, max. 6). Above 1 it enables the [partial peak scan](../../internals/mppt-tracker.md#partial-peak-scan); 0 = off |
| `scan_interval`     | s    | long  | 300     | Period of the partial peak scan; 0 = triggered scans only |
| `scan_trigger`      |      | byte  | 1       | 1 = also scan after a sustained change of Vin or power since the last scan or sweep |
| `dp_po`             |      | byte  | 0       | [Drift-compensated P&O](../../internals/mppt-tracker.md#drift-compensated-po): 1 = on in slow mode, 0 = off |
