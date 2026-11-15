---
title: coil.conf
sidebar_position: 4
---

# coil.conf

`coil.conf` describes the inductor. The following table lists its keys.

| key           | unit  | type  | default | description                                                                                                                                                |
|---------------|-------|-------|---------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `L0`          | H     | float | —       | Coil inductance (for ripple-current computation; undershot 5% for DC bias)                                                                                 |
| `rect_offset_ns` | ns | float | 0       | DCM low-side turn-off delay (gate-drive/MOSFET dead-time comp; >0 = LS off later, toward the zero crossing). Stored as a time so it survives a PWM-resolution/`pwm_freq` change; converted to counts at boot via the PWM tick rate (`ns·1e-9·tick_rate`; the full timer period × fsw, or the MCPWM peripheral resolution; not `pwmMax`, which excludes dead-time counts). See `measure-coil ls apply` (on-device; `--apply` there writes nothing) or `etc/measure_coil.py --ls-sweep --apply` (host). Used only if `rect_offset_drv` names the active gate driver, else 0. Written by `rect-cal`. |
| `rect_offset_drv` | — | string | — | Gate driver `rect_offset_ns` was measured on (`mcpwm`, `ledc`). Absent or different from the active driver: the offset is ignored (0) and marked uncalibrated. Written together with `rect_offset_ns` by `rect-cal` and `measure-coil ls apply`. |
| `rect_offset_cal` | — | 0/1/2 | 1 | State of `rect_offset_ns`. 1: a measurement. 0: a runtime-backstop reduction; it is used, but the offset counts as uncalibrated, so `rect_cal_auto` runs a calibration. 2: a bound; `rect-cal` found the knee at or past the ideal window (expected above `V_i = 2·V_o`) and kept the shorter of 0 and the previous offset. Written by `rect-cal` (1 or 2), the backstop (0) and `measure-coil ls apply` (1). |
| `rect_cal_auto` | — | 0/1 | 1 | Run `rect-cal` automatically while the offset is uncalibrated (foreign/missing `rect_offset_drv`, or `rect_offset_cal=0`): at most once a day, not in the first 10 min after boot, at a steady DCM operating point above 5 W. A bound (`rect_offset_cal=2`) is retried only while `V_i` is below `2·V_o`, and not within a week of a bound result (counted since boot). See [Diode emulation](../../internals/diode-emulation.md#self-calibration-rect-cal). |
