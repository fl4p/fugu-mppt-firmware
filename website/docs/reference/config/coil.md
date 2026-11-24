---
title: coil.conf
sidebar_position: 4
---

# coil.conf

`coil.conf` describes the inductor and the low-side timing calibration. The firmware reads it at boot, so reboot
after `set-config` (see [When changes take effect](index.md#when-changes-take-effect)); `rect-cal` and
`measure-coil ls apply` write their result here. [Coil inductance](../../lab/coil-inductance.md) describes how to
measure `L0`. The following table lists its keys.

| key           | unit  | type  | default | description                                                                                                                                                |
|---------------|-------|-------|---------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `L0`          | H     | float | (required) | Coil inductance (for ripple-current computation; undershot 5 % for DC bias). `board.conf::pwm_freq` × 0.95 × `L0` must lie between 1 and 20, else setup fails. `rect-cal` locates the zero crossing from the measured current and `0.95·L0`; an error of ε in L0 moves its reference by ε/2 of the LS window                                                                                 |
| `rect_offset_ns` | ns | float | 0       | DCM low-side turn-off delay (compensates gate-drive and MOSFET delays; >0 = LS off later, toward the zero crossing). Used only if `rect_offset_drv` names the active gate driver, else 0; at most +125 ns. Written by `rect-cal`. See [Offset limits](#offset-limits) |
| `rect_offset_drv` | — | string | — | Gate driver `rect_offset_ns` was measured on (`mcpwm`, `ledc`). Absent or different from the active driver: the offset is ignored (0) and marked uncalibrated. Written together with `rect_offset_ns` by `rect-cal` and `measure-coil ls apply`. |
| `rect_offset_cal` | — | 0/1/2 | 1 | State of `rect_offset_ns`. 1: a measurement. 0: a runtime-backstop reduction; it is used, but the offset counts as uncalibrated, so `rect_cal_auto` runs a calibration. 2: a bound; `rect-cal` found the knee more than the margin plus 125 ns past the zero crossing, or any late knee above `V_i = 2·V_o`, and kept the shorter of the limit (+125 ns, above `V_i = 2·V_o` 0) and the previous offset. A shorter previous offset that was a measurement stays, with state 1. A stored or `measure-coil ls apply` value past +125 ns is used as +125 ns with this state. Written by `rect-cal` (1 or 2), the backstop (0) and `measure-coil ls apply` (1, or 2 when it clamped to +125 ns). `measure-coil ls apply` is a bench tool and does not apply the `V_i = 2·V_o` rule. |
| `rect_cal_auto` | — | 0/1 | 1 | Run `rect-cal` automatically while the offset is uncalibrated (`rect_offset_drv` missing or naming another driver, or `rect_offset_cal=0`): at most once a day, not in the first 10 min after boot, at a steady DCM operating point above 5 W. A bound (`rect_offset_cal=2`) is retried only while `V_i` is below `2·V_o`, and not within a week of a bound result; a reboot clears that wait. See [Rectifier timing calibration](../../internals/rect-cal.md#preconditions-and-automatic-runs). |

## Offset limits

`rect_offset_ns` is stored as a time, so it survives a change of PWM resolution or `pwm_freq`. At boot the firmware
converts it to counts with the PWM tick rate (`ns·1e-9·tick_rate`): the full timer period × fsw, or the MCPWM
peripheral resolution, not `pwmMax`, which excludes dead-time counts.

The firmware limits the offset in use:

- At most +125 ns is used, compared in counts at the active tick rate. A larger value counts as a bound
  (`rect_offset_cal=2`).
- A positive value is applied only where it leaves the LS off for `T/32` plus the reverse current's drain time
  before the next HS turn-on, and not above `M = 0.8`; see
  [Rectifier timing calibration](../../internals/rect-cal.md#late-results).

Besides `rect-cal`, two tools measure the offset: `measure-coil ls apply` (on-device; `--apply` there writes nothing)
and `etc/measure_coil.py --ls-sweep --apply` (host).
