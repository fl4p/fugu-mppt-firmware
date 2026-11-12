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
| `wire_protect`             |      | int   | 1       | Output wire-fault protection: 0 off, 1 warn only, 2 warn, derate and trip. See [Output wire-fault protection](#output-wire-fault-protection) |
| `wire_p_warn`              | W    | float | 2       | Excess wiring power that raises a warning         |
| `wire_p_derate`            | W    | float | 4       | Excess wiring power that caps the output current at √(`wire_p_derate`/ΔR) |
| `wire_p_trip`              | W    | float | 8       | Excess wiring power that stops the converter until `wire-reset` or a reboot |
| `wire_dr_min`              | mΩ   | float | 4       | Minimum excess resistance ΔR for any level        |
| `wire_r_max`               | mΩ   | float | 0       | Absolute cap on the learned converter-to-pack resistance baseline; 0 = none |

## Output wire-fault protection

A loose or corroded connection between the converter and the battery heats up with the extra power
ΔR·I² it dissipates. The protection compares the output-path resistance against a baseline learned
on the board itself and grades the excess power:

| level  | condition                                                  | action |
|--------|------------------------------------------------------------|--------|
| warn   | excess power ≥ `wire_p_warn` and ΔR ≥ `wire_dr_min`         | log line, telemetry `wf`, MQTT alert |
| derate | excess power ≥ `wire_p_derate` (and ΔR ≥ `wire_dr_min`)     | output current capped at √(`wire_p_derate`/ΔR) |
| trip   | excess power ≥ `wire_p_trip`, or still ≥ 1.25 × `wire_p_derate` for 5 minutes while derated | converter stopped until `wire-reset` or reboot |

The ΔR floor keeps normal temperature drift of the copper (about 0.4 %/K) from raising a level at high
current. Three estimates feed the grading:

- **Excess drop per BMS frame.** The output-to-pack voltage drop minus what the baseline fit predicts
  for the present output and battery current, averaged over one minute while `Iout` ≥ 5 A. Needs the
  pack voltage and battery current over MQTT (`mqtt.conf` `bat_voltage_topic`, `ibat_topic`). A level
  needs 5 consecutive one-minute means above its threshold.
- **Slow fit.** The running converter-to-pack fit (telemetry `Rc`, `Rb`) evaluated at the present
  operating point against the baseline fit. Evaluated once a minute; a level needs 30 consecutive
  minutes above its threshold.
- **Sweep fit.** The output resistance of each qualifying sweep (telemetry `Ro`) against its own
  baseline. Needs no BMS data. A level needs 3 consecutive qualifying sweeps above its threshold.

The slow fit and the sweep fit are graded at the highest one-minute output current of the last one to
two charging hours. Any estimate can raise the level. A level drops only after every estimate that
still has data has stayed below 75 % of the thresholds for 30 minutes. A trip never drops by itself.
An estimate counts as having data until 15 (excess drop), 60 (slow fit) or 360 (sweep fit) charging
minutes pass without a new value; time without output current does not age it, so a night neither
releases nor invalidates the day's verdict.

The baseline is the median of the daily medians of the last 14 days that had data. A day ends at the
first hour without conversion at least 12 hours after the previous one, or after 36 hours. A day's
median enters the baseline only when it lies within `wire_dr_min` of the current baseline and, once
learning has ended, within half of `wire_dr_min` of the lowest baseline since then, so a slowly growing
fault isn't learned in. The first 7 days with data are a learning phase that only warns. The baseline
is kept in NVS and written at most once a day, only while the converter is stopped; reprovisioning the
config partition doesn't touch it. `wire-reset` clears the trip and the baseline and restarts
learning.

A rise of the shared bus-to-pack resistance (`Rb`) only warns: the chargers can't relieve that
section while a load discharges the pack through it. When the output and battery currents move
together (a single charger, steady loads), the converter can't tell a fault in the shared section
from one in its own wire, and the excess-drop and slow-fit estimates grade it as its own.

When no estimate has data (no BMS data and no qualifying sweep), telemetry `wf` reports -1 and the
level holds. After more than 3 charging days without any data the MQTT alert reports `unverified`.
