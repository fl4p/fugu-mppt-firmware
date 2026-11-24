---
title: charger.conf
sidebar_position: 6
---

# charger.conf

`charger.conf` sets the charge targets, current limits, temperature limits, and termination rules for the battery. The firmware reads it at boot, so reboot after `set-config` (see [When changes take effect](index.md#when-changes-take-effect)). A value outside its range fails setup and the converter stays off; `conf-check` runs the same checks without a reboot.

| key                 | unit | type  | default    | description                                                    |
|---------------------|------|-------|------------|----------------------------------------------------------------|
| `vout_max`          | V    | float | (required) | Max pack/output charge-target voltage. The cell count is ⌊`vout_max` / `cv_eoc`⌋ and must be at least 1 |
| `cv_eoc`            | V    | float | 3.5        | End-of-charge voltage of highest cell at tail current; must be ≥ `cv_float` |
| `cv_float`          | V    | float | 3.325      | Float cell voltage where termination line meets zero current   |
| `cv_ceiling`        | V    | float | cv_eoc+0.05 | Hard per-cell ceiling: latch termination if highest cell reaches it, regardless of current (backstops cv_eoc on imbalanced packs). Must be ≥ `cv_eoc` |
| `vout_max_fallback` | V    | float | cells × `cv_float` | Max output voltage when BMS data is missing, also the float target after termination. Must be greater than `vout_offset_max` |
| `ibat_max`          | A    | float | 20         | Maximum battery charge current limit                           |
| `bat_c`             | Ah   | float | unset → termination line and EOC feedback disabled (see [LFP charging](../../guide/charging/lfp-charging.md)) | Effective battery pack capacity |
| `tail_c_rate`       | C    | float | 0.05       | End-of-charge tail current as fraction of capacity; must be > 0 |
| `cv_hold_max_s`     | s    | int   | 0          | [CV hold](../../guide/charging/termination.md#cv-hold): instead of terminating when the highest cell crosses the termination line, hold it at `cv_eoc` and terminate on the tail current, at most this long (then terminate anyway). 0 = off (terminate on the line). Range 0..86400, e.g. 5400 |
| `tail_hold_s`       | s    | int   | 180        | CV hold: the pack current must stay below `tail_c_rate` × `bat_c`, with the highest cell inside `cv_hold_band` of `cv_eoc`, this long without a break to terminate (1..7200) |
| `cv_hold_band`      | V    | float | 0.02       | CV hold and top balancing: cell band below `cv_eoc` that counts as "held"; a line crossing enters the hold only inside it. Must be > 0, and smaller than `cv_eoc` − `cv_float` when `cv_hold_max_s` or `balance_interval` is set |
| `recharge_dod`      |      | float | 0.20       | Depth-of-discharge since full to release termination, 0 ≤ x < 1; with `partial_charge` set it must be in (0, `partial_charge`) |
| `recharge_vfloor_band` | V | float | 0.05    | Cell-voltage drop below `cv_float` to release termination (fallback to DoD counter); must be ≥ 0 |
| `vout_offset_max`   | V    | float | 0.6        | Worst-case Vout-sensor error tolerated during float: how far below the float floor the BMS-driven EOC loop may pull to stop charging a full pack when Vout reads high. Must be ≥ 0 |
| `partial_charge`    |      | float | 0          | SoC fraction to stop at between full charges (Ah-counted from the last termination); the pack is held there by load-following. 0 = always charge to full, valid 0 ≤ x < 1. Needs `bat_c` (else disabled with a warning) and a BMS `ibat_topic`; after a software reset the hold continues from a live `bat_charge_topic`, else the charger charges to full first |
| `full_charge_interval` | d | float | 7          | With `partial_charge` or `holiday_charge`: charge to full (BMS balancing) at least this often, 0 < x < 365. The age of the last full charge survives a software reset, not a power loss (unknown age: a full charge is due) |
| `balance_interval`  | d    | float | 0          | [Top balancing](../../guide/charging/termination.md#top-balancing): hold the highest cell at `cv_eoc` for `balance_hold_s` at least this often, for an active cell balancer. 0 = off. Must be at least 0 and below 365. The age of the last run survives a software reset; after a power loss it counts from boot. Console `balance [now\|skip]` |
| `balance_hold_s`    | s    | int   | 7200       | Top balancing: time the highest cell must spend within `cv_hold_band` of `cv_eoc` to complete a run, summed over charge sessions (1..86400). A single session ends after 2 × this |
| `balance_delta`     | V    | float | 0.01       | Top balancing: the run also completes when the BMS cell-voltage spread (`mqtt.conf` `cell_voltages_delta_topic`) stays at or below this on 3 frames in a row while the cell is in band; must be ≥ 0 |
| `holiday_charge`    |      | float | 0          | Holiday auto mode: SoC ceiling applied after `holiday_days` days in a row whose mean discharge current stays below `holiday_ibat` (the lower of this and `partial_charge` if both are set), e.g. 0.6. Ends when a day's mean reaches `holiday_ibat`. 0 = auto mode off, valid 0 ≤ x < 1. Needs `bat_c` (else disabled with a warning), `holiday_ibat` and a BMS `ibat_topic`; console override `holiday on\|off\|auto` |
| `holiday_ibat`      | A    | float | (required with `holiday_charge`) | Holiday auto mode: a 24 h window is quiet when the mean pack discharge current (over the time `ibat` < −0.2 A) stays below this. Set about 1.5 × the idle base-load discharge current; must be > 0 |
| `holiday_band`      |      | float | 0.1        | Holiday auto mode: SoC band below `holiday_charge` before the hold releases (instead of `recharge_dod`); with 0.6 the pack cycles about 50–60 %. Must be in (0, `holiday_charge`) |
| `holiday_days`      | d    | int   | 3          | Holiday auto mode: quiet 24 h windows in a row needed to apply `holiday_charge` (1..60) |
| `bat_temp_min`      | °C   | float | 0          | Pack current held at zero below this pack temperature (BMS `bat_temp_topic`, coldest sensor; loads still served); released 2 °C above |
| `bat_temp_derate`   | °C   | float | 45         | Pack current limit ramps down linearly from here (hottest sensor), regulated through the pack-voltage pin ... |
| `bat_temp_max`      | °C   | float | 55         | ... to zero here. Needs `bat_temp_min` < `bat_temp_derate` < `bat_temp_max` |
