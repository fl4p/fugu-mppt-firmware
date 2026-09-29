---
title: charger.conf
sidebar_position: 6
---

# charger.conf

Battery termination.

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
| `recharge_vfloor_band` | V | float | 0.05    | Cell-voltage drop below cv_min to release termination (fallback to DoD counter) |
| `vout_offset_max`   | V    | float | 0.6        | Worst-case Vout-sensor error tolerated during float: how far below the float floor the BMS-driven EOC loop may pull to stop charging a full pack when Vout reads high |
| `partial_charge`    |      | float | 0          | SoC fraction to stop at between full charges (Ah-counted from the last termination); the pack is held there by load-following. 0 = always charge to full. Needs `bat_c` and a BMS `ibat_topic` |
| `full_charge_interval` | d | float | 7          | With `partial_charge`: charge to full (BMS balancing) at least this often. Counted from the last termination since boot; a reboot charges to full first |
| `bat_temp_min`      | °C   | float | 0          | Pack current held at zero below this pack temperature (BMS `bat_temp_topic`, coldest sensor; loads still served); released 2 °C above |
| `bat_temp_derate`   | °C   | float | 45         | Pack current limit ramps down linearly from here (hottest sensor), regulated through the pack-voltage pin ... |
| `bat_temp_max`      | °C   | float | 55         | ... to zero here |
