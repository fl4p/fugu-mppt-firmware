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
| `sweep_pace`        |      | float | 1.0     | Largest [sweep](../../internals/mppt-tracker.md#global-sweep) speed-up where Vin moves slowly with the duty (near Voc, and towards the Vin floor), 1..8; 1 = off (a ramp linear in duty). Experimental: at low sample rates the filter lag can capture a lower MPP |
| `sweep_verify`      |      | float | 0.10    | [Sweep verdict](../../internals/mppt-tracker.md#sweep-verdict): flag a sweep when the settled power at its MPP duty differs from the captured MPP power by more than this fraction; a flagged sweep starts a peak scan or one re-sweep (at most one per 10 min). 0 = off |
| `pv_substrings`     |      | byte  | 0       | Learned bypass-diode substring count of the [partial peak scan](../../internals/mppt-tracker.md#substring-count) (2..6), written by the converter after a shaded sweep. 0 = learn again; a change takes effect at the next boot, and a count learned before then is not written over it |
| `scan_interval`     | s    | long  | 300     | Period of the [partial peak scan](../../internals/mppt-tracker.md#partial-peak-scan) (0..65535); 0 = triggered scans only |
| `scan_trigger`      |      | byte  | 1       | 1 = also scan after a sustained change of Vin or power since the last scan or sweep |
