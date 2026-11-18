---
title: charger.conf
sidebar_position: 6
---

# charger.conf

`charger.conf` sets the charge targets, current limits, temperature limits, and termination rules for the battery. The firmware reads these keys:

| key                 | unit | type  | default    | description                                                    |
|---------------------|------|-------|------------|----------------------------------------------------------------|
| `vout_max`          | V    | float | —          | Max pack/output charge-target voltage                          |
| `cv_eoc`            | V    | float | 3.5        | End-of-charge voltage of highest cell at tail current          |
| `cv_float`          | V    | float | 3.325      | Float cell voltage where termination line meets zero current   |
| `cv_ceiling`        | V    | float | cv_eoc+0.05 | Hard per-cell ceiling: latch termination if highest cell reaches it, regardless of current (backstops cv_eoc on imbalanced packs) |
| `vout_max_fallback` | V    | float | cells × `cv_float` | Max output voltage when BMS data is missing, also the float target after termination. Must be greater than `vout_offset_max`; otherwise setup fails and the converter stays off |
| `ibat_max`          | A    | float | 20         | Maximum battery charge current limit                           |
| `bat_c`             | Ah   | float | unset → termination line and EOC feedback disabled (see [LFP charging](../../guide/charging/lfp-charging.md)) | Effective battery pack capacity |
| `tail_c_rate`       | C    | float | 0.05       | End-of-charge tail current as fraction of capacity             |
| `recharge_dod`      |      | float | 0.20       | Depth-of-discharge since full to release termination           |
| `recharge_vfloor_band` | V | float | 0.05    | Cell-voltage drop below `cv_float` to release termination (fallback to DoD counter) |
| `vout_offset_max`   | V    | float | 0.6        | Worst-case Vout-sensor error tolerated during float: how far below the float floor the BMS-driven EOC loop may pull to stop charging a full pack when Vout reads high |
| `partial_charge`    |      | float | 0          | SoC fraction to stop at between full charges (Ah-counted from the last termination); the pack is held there by load-following. 0 = always charge to full. Needs `bat_c` and a BMS `ibat_topic`; after a software reset the hold continues from a live `bat_charge_topic`, else the charger charges to full first |
| `full_charge_interval` | d | float | 7          | With `partial_charge` or `holiday_charge`: charge to full (BMS balancing) at least this often. The age of the last full charge survives a software reset, not a power loss (unknown age: a full charge is due) |
| `holiday_charge`    |      | float | 0          | Holiday auto mode: SoC ceiling applied after `holiday_days` days in a row whose mean discharge current stays below `holiday_ibat` (the lower of this and `partial_charge` if both are set), e.g. 0.6. Ends when a day's mean reaches `holiday_ibat`. 0 = auto mode off. Needs `bat_c`, `holiday_ibat` and a BMS `ibat_topic`; console override `holiday on\|off\|auto` |
| `holiday_ibat`      | A    | float | (required) | Holiday auto mode: a 24 h window is quiet when the mean pack discharge current (over the time `ibat` < −0.2 A) stays below this. Set about 1.5 × the idle base-load discharge current; required when `holiday_charge` > 0 |
| `holiday_band`      |      | float | 0.1        | Holiday auto mode: SoC band below `holiday_charge` before the hold releases (instead of `recharge_dod`); with 0.6 the pack cycles about 50–60 %. Must be in (0, `holiday_charge`) |
| `holiday_days`      | d    | int   | 3          | Holiday auto mode: quiet 24 h windows in a row needed to apply `holiday_charge` (1..60) |
| `bat_temp_min`      | °C   | float | 0          | Pack current held at zero below this pack temperature (BMS `bat_temp_topic`, coldest sensor; loads still served); released 2 °C above |
| `bat_temp_derate`   | °C   | float | 45         | Pack current limit ramps down linearly from here (hottest sensor), regulated through the pack-voltage pin ... |
| `bat_temp_max`      | °C   | float | 55         | ... to zero here |
