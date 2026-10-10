---
title: limits.conf
sidebar_position: 3
---

# limits.conf

`limits.conf` holds the protection cutouts. The following table lists its keys.

Voltage and current ratings of the hardware are named by board side (HV/LV). `converter.conf::topo`
maps them to the converter roles:

| side key   | buck (in = HV, out = LV) | boost (in = LV, out = HV) | legacy alias |
|------------|--------------------------|---------------------------|--------------|
| `hv_max`   | Vin max                  | Vout max                  | `vin_max` (buck), `vout_max` (boost) |
| `lv_max`   | Vout max                 | Vin max                   | `vout_max` (buck), `vin_max` (boost) |
| `hv_i_max` | Iin max                  | Iout max                  | `iin_max` (buck), `iout_max` (boost) |
| `lv_i_max` | Iout max                 | Iin max                   | `iout_max` (buck), `iin_max` (boost) |

The role keys `vin_max`, `vout_max`, `iin_max`, and `iout_max` still work as legacy aliases. A file
uses one form for all four: any side key next to any of these role keys fails at boot
(`limits.conf: mixes side key hv_max with role key vout_max …`), even when they name different
limits. A missing limit is reported under both names, for example
`limits.conf: missing hv_max (or vin_max under topo=buck)`. `vin_min`, `iout_short`, and `p_max` describe the
source, the output, and the power path, not a side, and keep their role names. The battery limit
`charger.conf::vout_max` is also unaffected: the battery is always on the output.

| key                        | unit | type  | default | description                                       |
|----------------------------|------|-------|---------|---------------------------------------------------|
| `hv_max`                   | V    | float | —       | Maximum HV-side voltage before protection cutout (legacy: `vin_max`/`vout_max`) |
| `lv_max`                   | V    | float | —       | Maximum LV-side voltage before protection cutout (legacy: `vout_max`/`vin_max`) |
| `vin_min`                  | V    | float | —       | Input-voltage regulation floor. In MPPT/PSU/PV modes the Vin controller reduces duty to keep Vin ≥ `vin_min`. Not a trip, and not applied in manual PWM. The hard supply undervoltage stop is fixed at ~9 V. Required (no usable default) |
| `hv_i_max`                 | A    | float | —       | Maximum HV-side current (legacy: `iin_max`/`iout_max`) |
| `lv_i_max`                 | A    | float | —       | Maximum LV-side current (legacy: `iout_max`/`iin_max`) |
| `iout_short`               | A    | float | —       | Output short-circuit current threshold            |
| `p_max`                    | W    | float | —       | Maximum power; sets thermal derating and the sweep's minimum MPP power (0.2 %) |
| `temp_max`                 | °C   | float | 90      | Maximum temperature before shutdown               |
| `temp_derate`              | °C   | float | —       | Temperature where power derating begins           |
| `reverse_current_paranoia` |      | bool  | 1       | Enable aggressive reverse-current protection      |
