---
title: tracker.conf
sidebar_position: 7
---

# tracker.conf

`tracker.conf` configures the MPPT tracker. It has the following keys.

| key                 | unit | type  | default | description                                                          |
|---------------------|------|-------|---------|----------------------------------------------------------------------|
| `target_duty_cycle` |      | float | absent  | Fixed duty cycle fraction (0–1 of pwmMaxDriver); if set, boots into manual PWM mode with hard-fixed duty (no PD control, no MPPT tracking); an explicit `0` boots manual at duty 0 (converter off). Absent: MPPT |
| `sweep_speed`       |      | float | 4.0     | Global-sweep speed; scales both per-tick cap and limit-tracking gain (clamp ≥ 0.1) |
| `sweep_pace`        |      | float | 4.0     | Largest [sweep](../../internals/mppt-tracker.md#global-sweep) speed-up where Vin moves slowly with the duty (near Voc, and towards the Vin floor), 1..8; 1 = off (a ramp linear in duty). Bounded further by the sensor filter delay: none at low sample rates with long filters |
| `sweep_verify`      |      | float | 0.10    | [Sweep verdict](../../internals/mppt-tracker.md#sweep-verdict): flag a sweep when a return-pass probe differs from the down-pass power at its maximum, or the settled landing power from the MPP it aimed at, by more than this fraction. A flagged sweep does not teach `pv_substrings` and can start a peak scan, or one re-sweep when it recorded one maximum and `pv_substrings` is 0. 0 = off |
| `pv_substrings`     |      | byte  | 0       | Learned bypass-diode substring count of the [partial peak scan](../../internals/mppt-tracker.md#substring-count) (2..6), written by the converter after a shaded sweep; 1 = single peak (two verified single-maximum sweeps in a row): no scan, no re-sweep. 0 = learn again; a change takes effect at the next boot, and a count learned before then is not written over it |
| `scan_interval`     | s    | long  | 300     | Period of the [partial peak scan](../../internals/mppt-tracker.md#partial-peak-scan) (0..65535); 0 = triggered scans only |
| `scan_trigger`      |      | byte  | 1       | 1 = also scan after a sustained change of Vin or power since the last scan or sweep |
