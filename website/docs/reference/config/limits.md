---
title: limits.conf
sidebar_position: 3
---

# limits.conf

Protection cutouts.

| key                        | unit | type  | default | description                                       |
|----------------------------|------|-------|---------|---------------------------------------------------|
| `vin_max`                  | V    | float | —       | Maximum input voltage before protection cutout    |
| `vin_min`                  | V    | float | —       | Minimum input voltage; below this converter stops |
| `vout_max`                 | V    | float | —       | Maximum output voltage before protection cutout   |
| `iin_max`                  | A    | float | —       | Maximum input current limit                       |
| `iout_max`                 | A    | float | —       | Maximum output current limit                      |
| `iout_short`               | A    | float | —       | Output short-circuit current threshold            |
| `p_max`                    | W    | float | —       | Maximum power (used for thermal derating)         |
| `temp_max`                 | °C   | float | 90      | Maximum temperature before shutdown               |
| `temp_derate`              | °C   | float | —       | Temperature where power derating begins           |
| `reverse_current_paranoia` |      | bool  | 1       | Enable aggressive reverse-current protection      |
