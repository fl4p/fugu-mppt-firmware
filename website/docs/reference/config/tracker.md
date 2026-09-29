---
title: tracker.conf
sidebar_position: 7
---

# tracker.conf

MPPT.

| key                 | unit | type  | default | description                                                          |
|---------------------|------|-------|---------|----------------------------------------------------------------------|
| `target_duty_cycle` |      | float | 0.0     | Fixed duty cycle fraction (0–1 of pwmMaxDriver); if > 0 enters manual PWM mode with hard-fixed duty (no PD control, no MPPT tracking) |
| `sweep_speed`       |      | float | 4.0     | Global-sweep speed; scales both per-tick cap and limit-tracking gain (clamp ≥ 0.1) |
