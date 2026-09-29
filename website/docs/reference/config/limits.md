---
title: limits.conf
sidebar_position: 3
---

# limits.conf

Protection cutouts.

| key                        | unit | type  | default | description                                       |
|----------------------------|------|-------|---------|---------------------------------------------------|
| `vin_max`                  | V    | float | —       | Maximum input voltage before protection cutout    |
| `vin_min`                  | V    | float | —       | Input-voltage regulation floor. In MPPT/PSU/PV modes the Vin controller reduces duty to keep Vin ≥ `vin_min`. Not a trip, and not applied in manual PWM. The hard supply undervoltage stop is fixed at ~9 V. Required (no usable default) |
| `vout_max`                 | V    | float | —       | Maximum output voltage before protection cutout   |
| `iin_max`                  | A    | float | —       | Maximum input current limit                       |
| `iout_max`                 | A    | float | —       | Maximum output current limit                      |
| `iout_short`               | A    | float | —       | Output short-circuit current threshold            |
| `p_max`                    | W    | float | —       | Maximum power (used for thermal derating)         |
| `temp_max`                 | °C   | float | 90      | Maximum temperature before shutdown               |
| `temp_derate`              | °C   | float | —       | Temperature where power derating begins           |
| `reverse_current_paranoia` |      | bool  | 1       | Enable aggressive reverse-current protection      |
