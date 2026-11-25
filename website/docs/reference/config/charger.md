---
title: charger.conf
sidebar_position: 6
---

# charger.conf

`charger.conf` sets the charge targets, current limits, temperature limits, and termination rules for the battery. The firmware reads it at boot, so reboot after `set-config` (see [When changes take effect](index.md#when-changes-take-effect)). A value outside its range fails setup and the converter stays off; `conf-check` runs the same checks without a reboot.

| key                 | unit | type  | default    | description                                                    |
|---------------------|------|-------|------------|----------------------------------------------------------------|
| `vout_max`          | V    | float or `auto` | (required) | Maximum charge-target voltage of the pack at the output. The cell count is ⌊`vout_max` / `cv_eoc`⌋ and must be at least 1. `auto` detects 4, 8 or 16 cells at boot, see [Pack detection](#pack-detection) |
| `cv_eoc`            | V    | float | 3.5        | End-of-charge voltage of highest cell at tail current; must be ≥ `cv_float` |
| `cv_float`          | V    | float | 3.325      | Float cell voltage where termination line meets zero current   |
| `cv_ceiling`        | V    | float | cv_eoc+0.05 | Hard per-cell ceiling: latch termination if highest cell reaches it, regardless of current (backstops cv_eoc on imbalanced packs). Must be ≥ `cv_eoc` |
| `vout_max_fallback` | V    | float | cells × `cv_float` | Max output voltage when BMS data is missing, also the float target after termination. Must be greater than `vout_offset_max`. Not allowed with `vout_max=auto` |
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

## Pack detection

With `vout_max=auto` the firmware takes the cell count from the battery voltage it measures while the converter is
off. It then sets `vout_max` to cells × `cv_eoc` and the fallback to
cells × `cv_float`, and runs as if both were configured. A missing `vout_max` still fails setup.

Setup fails unless all of these hold:

- The converter is a buck, and its Vout sensor sees the battery: the board has no backflow switch, or
  `board.conf::bflow_bat=1` declares the switch on the battery side. Without a battery, a boost's output sits at
  Vin less a diode drop, and a Vout sensor on the converter side of a backflow switch reads that node instead of the
  battery. Pack detection is therefore buck-only.
- `cv_eoc` lies between 2.43 and 4.85 V, which keeps the voltage windows below apart.
- `vout_max_fallback` is not set, because the fallback follows the detected count.

### Voltage windows

A pack of $n$ cells matches when $n \times 2.5\ \text{V} \le V_{out} \le n \times cv_{eoc} \times 1.03$. With
`cv_eoc` = 3.65 V:

| Cells | Vout window | `vout_max` | Fallback with `cv_float` = 3.35 V |
|---|---|---|---|
| 4 | 10.0–15.0 V | 14.6 V | 13.4 V |
| 8 | 20.0–30.1 V | 29.2 V | 26.8 V |
| 16 | 40.0–60.2 V | 58.4 V | 53.6 V |

### Readings

The firmware reads the filtered Vout every 3 s. Three readings in a row must fall in the same window and stay within
1 % of the first, which rejects output capacitors that are still discharging. A reading is discarded when:

- the converter is switching;
- Vin lies within max(1.5 V, 4 % of Vout) of Vout, where a conducting high-side FET could feed the output;
- cells × `cv_eoc` exceeds the output voltage limit (`limits.conf::lv_max`);
- fresh BMS data disagrees. The BMS reports no cell count, so the firmware checks
  $V/V_{high} - 0.5 \le n \le V/(V_{high} - \Delta) + 0.5$, with $V$ the BMS pack voltage (else Vout), $V_{high}$
  the highest cell and $\Delta$ the cell spread (else 0).

With a BMS cell-voltage topic configured, detection waits up to 30 s after boot for its first frame, then goes on
with Vout alone. The result appears once in the log, for example
`pack detected: 8 cells at 26.40 V -> vout_max 29.20 V, fallback 26.80 V`. While no pack is detected, the log names
the reason at most every 15 s, and every 60 s while it stays the same.

### Before detection

Until a pack is detected, the converter stays off: MPPT and PSU starts are blocked (`bat-detect`), manual PWM is
held off and the charger is idle. `vset` is refused and `status` shows `Vbat_max=nanV (auto, no pack yet)`.
A persistent output over-voltage clears `vout_max` as it does for a configured value, and detection then runs again.

### Pack change

After detection the firmware keeps checking Vout while the converter is off. When three readings in a row, 3 s
apart, fall outside the detected pack's band, it logs `pack left the 8-cell band (Vout 13.47 V, below its band),
detecting again`, clears `vout_max`, the fallback and the charge-voltage pin, and detection starts over. Until it
succeeds the converter stays off, as after boot.

For a pack of $n$ cells the band is $n \times \min(2.5\ \text{V}, \max(2\ \text{V}, cv_{eoc} \times 1.03 / 2 +
50\ \text{mV})) \le V_{out} < 2n \times 2.5\ \text{V}$, which holds the whole detection window. It is wider than the detection window, which prevents a change from flapping:

- A deep-discharged pack down to 2.0 V per cell (with `cv_eoc` up to 3.78 V), or a load sag shorter than the three readings, keeps the
  count.
- A 4, 8 or 16 cell pack of another count lies outside the band, empty or full, for any accepted `cv_eoc`. So does a
  removed pack once the output capacitors have discharged.
- Readings taken while the converter switches don't count and restart the three readings, so a sag under load never
  clears the count.
- A missing Vout reading counts as outside.

A pack below 2.0 V per cell (2.21 V with `cv_eoc` = 4.2 V, at most 2.5 V) counts as removed. It is not charged until it reads inside a
window again.

### Limits

- Only 4, 8 and 16 cells. A pack of 12 to 15 cells reads inside the 16-cell window, and one of 5 to 7 cells inside the
  8-cell window. Without a BMS that contradicts it, such a pack is charged as 16 or 8 cells and overcharged. Set
  `vout_max` explicitly for any other cell count.
- A non-lithium pack, such as a 12 V lead-acid battery, reads inside the 4-cell window and is charged with `cv_eoc`
  per cell.
