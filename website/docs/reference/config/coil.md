---
title: coil.conf
sidebar_position: 4
---

# coil.conf

Inductor.

| key           | unit  | type  | default | description                                                                                                                                                |
|---------------|-------|-------|---------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `L0`          | H     | float | —       | Coil inductance (for ripple-current computation; undershot 5% for DC bias)                                                                                 |
| `rect_offset_ns` | ns | float | 0       | DCM low-side turn-off delay (gate-drive/MOSFET dead-time comp; >0 = LS off later, toward the zero crossing). Stored as a time so it survives a PWM-resolution/`pwm_freq` change; converted to counts at boot via the PWM tick rate (`ns·1e-9·tick_rate`; the full timer period × fsw, or the MCPWM peripheral resolution — not `pwmMax`, which excludes dead-time counts). See `measure-coil ls --apply`. |
