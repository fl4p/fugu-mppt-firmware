---
title: limits.conf
sidebar_position: 3
---

# limits.conf

`limits.conf` holds the protection cutouts. The following table lists its keys.

Voltage and current ratings of the hardware are named by board side (HV/LV). `converter.conf::topo`
maps them to the converter roles:

| side key   | buck (in = HV, out = LV) | boost (in = LV, out = HV) |
|------------|--------------------------|---------------------------|
| `hv_max`   | Vin max                  | Vout max                  |
| `lv_max`   | Vout max                 | Vin max                   |
| `hv_i_max` | Iin max                  | Iout max                  |
| `lv_i_max` | Iout max                 | Iin max                   |

A missing limit names the key and the role it plays, for example
`limits.conf: missing hv_max (Vin max under topo=buck)`. `vin_min`, `iout_short`, and `p_max` describe the
source, the output, and the power path, not a side, and keep their role names. The battery limit
`charger.conf::vout_max` is also unaffected: the battery is always on the output.

The firmware no longer reads the old role keys for the four side ratings. A file that still has one
fails setup at boot with its replacement, for example
`limits.conf: vout_max is no longer read; with topo=boost use hv_max`. See
[Migrating from role keys](sensor.md#migrating-from-role-keys).

| key                        | unit | type  | default | description                                       |
|----------------------------|------|-------|---------|---------------------------------------------------|
| `hv_max`                   | V    | float | —       | Maximum HV-side voltage before protection cutout       |
| `lv_max`                   | V    | float | —       | Maximum LV-side voltage before protection cutout       |
| `vin_min`                  | V    | float | —       | Input-voltage regulation floor. In MPPT/PSU/PV modes the Vin controller reduces duty to keep Vin ≥ `vin_min`. Not a trip, and not applied in manual PWM. The hard supply undervoltage stop is fixed at ~9 V. Required (no usable default) |
| `hv_i_max`                 | A    | float | —       | Maximum HV-side current                           |
| `lv_i_max`                 | A    | float | —       | Maximum LV-side current                           |
| `iout_short`               | A    | float | —       | Output short-circuit current threshold            |
| `p_max`                    | W    | float | —       | Maximum power; sets thermal derating and the sweep's minimum MPP power (0.2 %) |
| `temp_max`                 | °C   | float | 90      | Maximum temperature before shutdown               |
| `temp_derate`              | °C   | float | —       | Temperature where power derating begins           |
| `reverse_current_paranoia` |      | bool  | 1       | Enable aggressive reverse-current protection      |
