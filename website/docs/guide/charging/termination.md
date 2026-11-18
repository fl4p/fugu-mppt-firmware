---
title: "Charge Termination"
sidebar_position: 2
---


# Charge Termination

`Li_ChgTerminationCondition` in `src/charger.h` implements a charge-termination
line for LFP and other lithium chemistries, as described in
[Charging Marine Lithium Battery Banks](https://nordkyndesign.com/charging-marine-lithium-battery-banks/).

The model places the cell at `cv_min` (the "float" voltage) at zero current and
at `cv_eoc` (the absorption voltage) when the charging current equals the
tail-current threshold. Between those two operating points, a straight line in
the (current, voltage) plane defines the termination boundary. Above the line,
the cell is "still charging". Below it, the cell is "done".

The line can be restated as an apparent series resistance:

```
r = (cv_eoc - cv_min) / (tail_c_rate * Cbat)
```

With that resistance, the per-current termination voltage becomes:

```
v_term(ibat) = cv_min + ibat * r       (clamped at cv_eoc)
```

The charger declares termination when the highest reported cell voltage
crosses `v_term`. Like the ceiling, the line trigger needs two consecutive BMS
cell frames above the line (`termCond.update()` runs once per cell frame).

The current that places the line is the recent peak of the battery current, decaying with a time
constant of 2 minutes, not the instantaneous value. When the charge current drops suddenly, for example
under a passing cloud, the cell voltage needs minutes to relax, and an instantaneous line would
drop below it at once and end the charge early.

## The `tail_c_rate` parameter

`tail_c_rate` is the C-rate at which the cell is considered "fully charged",
that is, the tail current expressed as a fraction of capacity. You can set it in
`charger.conf:tail_c_rate`. It defaults to `0.05` (LFP).

The following table lists typical values per chemistry:

| Chemistry                            | `tail_c_rate` |
|--------------------------------------|---------------|
| LFP                                  | `0.05`        |
| Sanyo/Panasonic NCR18650GA (67 mA / 3500 mAh, [datasheet](https://www.orbtronic.com/content/Datasheet-specs-Sanyo-Panasonic-NCR18650GA-3500mah.pdf)) | `~0.02` |
| EVE INR18650                         | `0.033`       |

A larger `tail_c_rate` is safer. It produces a steeper termination line, so
termination triggers at a higher current, earlier in the constant-voltage
phase. The cell is left less fully charged but with more headroom against
over-charge.

A smaller `tail_c_rate` charges more fully with a tighter margin. The charger
holds CV down to a smaller tail current before it terminates. The cell ends up
closer to its true full-charge state, but the cell-voltage tolerance is
narrower.

For unknown cells, the default `0.05` is the conservative choice. Termination
may be slightly early for chemistries that prefer a smaller tail. The tail rate
leaves the voltage limit unchanged: for any `tail_c_rate`, the termination line
is capped at `cv_eoc`, and `cv_ceiling` is a current-independent per-cell
backstop above it.

## Recharge hysteresis (DoD-based release)

The charger releases termination after a set amount of net discharge, not on
cell voltage alone. LFP cells discharge nearly flat: a 1 % SoC drop from full
can take voltage from 3.45 V down to 3.35 V. Releasing termination on
`vcell_high < cv_min` alone therefore causes micro-cycling at the top of
charge. The charger oscillates between "terminated" and "charging" every few
minutes while the cells barely move.

To measure that discharge, the charger integrates `-ibat` over time (positive =
pack discharging) and tracks Ah-since-the-last-full event. Termination releases
when either of these conditions holds:

- Ah condition (primary): `ahSinceFull > recharge_dod * Cbat`. With the
  default `recharge_dod = 0.20` and a 280 Ah pack, this is ~56 Ah of net
  discharge before recharge is permitted.
- Voltage floor (fallback): `vcell_high < cv_min - recharge_vfloor_band`
  (band default 0.05 V) for 4 consecutive BMS cell frames. That is 3.275 V with
  the firmware defaults and 3.32 V with `cv_float=3.37`. This condition catches
  integrator drift, a wrong `Cbat`, or a missing BMS, any of which would
  otherwise leave the charger stuck terminated while the pack is genuinely
  empty.

With `mqtt.conf:bat_charge_topic` set, the charger takes the deficit from the
BMS remaining-charge counter instead, as long as that value is fresh (a frame
within the last 120 s). The charge is sampled on every cell frame, and the
reference is the sample on the frame that latched termination. Chargers that
share one BMS therefore compute the same deficit and release on the same cell
frame. With independent integrators, small differences let one charger release
first. Its charging then pulls the other's deficit back below the threshold, so
that one stays terminated until the next full charge. The release compares the
deepest deficit since the last full charge, so a sibling that starts charging
cannot undo a release that is already due. If the BMS charge goes stale, the
integrator takes over.

The charger re-zeroes the integrator on every termination event. The
integrator counts the "deficit since the last known full" and recalibrates
itself each cycle, so it does not have to be a precise long-term SoC gauge.

The counter and the termination latch are RAM-only. After a reboot, the
charger is not terminated, so it charges until termination latches again
(within two cell frames if the pack is still full). That termination re-zeroes
the counter, and from then on both release conditions apply as usual.

You can set `recharge_dod` in `charger.conf:recharge_dod`. A higher value
releases later: deeper discharges between full charges, fewer cycles, and more
time at mid-SoC. A lower value releases sooner: more frequent topping, more
cycle wear, and less time below 100 %. LFP off-grid systems commonly run
0.10–0.30.

## Parallel chargers on one pack

When several chargers feed one pack and the pack is terminated, usually only
one of them drives the bus. A converter drives the bus while it is enabled
and above its minimum duty. The others sit at minimum duty, and charging into
a full pack would only fight that converter. A charger that does not drive
the bus therefore pins its target at the float floor
(`vout_max_fallback - vout_offset_max`) and delivers no current.

While the BMS reports the pack discharging (pack current below −1 A), the load
is larger than what the driving charger supplies. The idle charger then joins
at the float target (`vout_max_fallback`) and helps cover the load. It returns
to the floor once the pack current rises above +0.5 A, or when the BMS current
goes stale. The join target is fixed, so a converter that is off overnight
does not build up a higher setpoint.

Set `bat_charge_topic` so that all chargers leave termination together (see
[Recharge hysteresis](#recharge-hysteresis-dod-based-release)).

## Related parameters

These parameters also shape termination:

- `cv_eoc`: absorption voltage (LFP: 3.65 V; firmware default 3.5 V)
- `cv_float` (loaded into `cv_min`): float voltage (LFP: 3.37 V; firmware
  default 3.325 V)
- `cv_ceiling`: hard per-cell ceiling, default `cv_eoc + 0.05`. It latches
  termination after 2 consecutive frames at or above it, regardless of current.
- `bat_c`: effective pack capacity in Ah. For parallel packs, use the summed
  Ah (e.g. 2P 280 Ah → 560). The model assumes `Cbat` is the parallel-effective
  capacity.
- `recharge_dod`: DoD threshold for releasing termination (see above).

:::warning Set `bat_c` on any battery system
If `bat_c` is missing, the termination line and the EOC feedback on the highest
cell are disabled. The output is held at the pack-level `vout_max_fallback`
(default `N_cells × cv_float`), open-loop on the Vout reading, like a stale BMS.
Only the hard `cv_ceiling` latch can still flag termination, and it does not
lower the output. The Ah recharge condition is disabled; only the voltage floor
releases termination. The pack floats near `cv_float` per cell and does not
reach full, and the highest cell is not individually limited.
:::

## Absorption target vs. termination line

The termination line only *decides* termination. It is not a setpoint.

The EOC feedback loop (`BatteryCharger::_updatePackVoltagePinning`) lowers the
pack-voltage pin whenever the highest cell is above its target. That target is
the fixed `cv_eoc` while charging and `cv_min` once terminated.

The target must not be the current-dependent `v_term`. Lowering the pin
reduces the current, which lowers `v_term`, which lowers the pin again, until
the current is zero and the line trips at `cv_min`. This is the premature
termination observed in the field in July 2026 at 3.40 V/cell.

## Partial-charge ceiling (`partial_charge`)

The charger can stop short of full to lower the average SoC, which is the
best-evidenced LFP lifetime lever (see
[LFP longevity](../../internals/lfp-longevity.md)). Recharge hysteresis alone
keeps the pack in the 80–100 % window.

The partial-charge ceiling works as follows:

- After a termination (the only point where the Ah counter is known to be at
  zero), charging stops once `ahSinceFull <= (1 - partial_charge) * Cbat`. The
  charger then holds the pack there by load-following: on every BMS `ibat`
  frame, the pin steps by up to 20 mV (4 mV/A, 0.2 A deadband) so the pack
  current goes to a small target and the converter covers the load only.
- The charger trims the target by the Ah error (±1 A at 2 Ah off the ceiling),
  so a BMS current offset inside the deadband cannot walk the SoC away over the
  week.
- The hold starts from the measured bus voltage and only steps while the
  converter drives the bus (not at night).
- The pin may fall to
  `n_cells * (cv_float - recharge_vfloor_band) - vout_offset_max` and rise to
  `Vbat_max`. If the load exceeds the PV, the pin sits at `Vbat_max` and the
  pack discharges normally.
- The hold releases when the deficit exceeds the ceiling by `recharge_dod`
  (the pack cycles between `partial_charge - recharge_dod` and
  `partial_charge`).
- Every `full_charge_interval` days, the charger drops the ceiling and charges
  the pack to full for BMS balancing.
- A software reset (restart, crash, OTA update) keeps the age of the last full
  charge and the BMS remaining charge at that point in RTC memory. With
  `mqtt.conf:bat_charge_topic` reporting, the hold continues after such a reset
  from the BMS charge. Otherwise, and after a power loss, the deficit counter
  starts unknown and the charger charges to full first.
- The hold needs live BMS data. If the cell-voltage or the `ibat` stream stops
  (180 s), the charger drops the hold and falls back to its ordinary behaviour.
  It re-engages the hold when the data returns.
- Unlike termination, the hold does not block the dawn start. The converter
  has to run to serve the loads, and the pin keeps the pack current at zero.

`status` reports `PARTIAL HOLD (load-following)`, the ceiling, and the age of
the last full charge. The periodic re-sweep and the stuck watchdog treat the
hold like termination.

The configuration must satisfy `0 < recharge_dod < partial_charge`. Otherwise,
the release band would reach below 0 % SoC. Without `bat_c`, the ceiling is
disabled.

### Holiday auto mode (`holiday_charge`)

A pack that is cycled every day benefits from charging to full; a pack that
barely moves for days, for example while the owner is away, only sits at the
top. With `holiday_charge` set, the charger detects such quiet periods and
applies a ceiling automatically:

- The charger measures the mean discharge current of the pack in consecutive
  24 h windows on its own uptime clock: the Ah discharged while the BMS `ibat`
  was below −0.2 A, divided by the time it was. Charging is ignored, so a full
  charge is not counted as activity, and neither is discharge caused by the
  hold itself while it drains the pack down to the ceiling. A window is quiet
  when this mean (taken over at least 2 h) stays below `holiday_ibat`.
- A window is only evaluated when live `ibat` data covered at least 80 % of it
  (a gap of more than 60 s between frames counts as not covered) and, if the
  pack discharged for 2 h or more, the `ibat` reading changed in at least 5 %
  of the frames (a stuck reading is not quiet). A covered window with almost no
  discharge counts as quiet. A window that fails these
  checks neither counts nor resets the streak. Before holiday mode is on, two
  such windows in a row reset the streak; once it is on, missing data never
  ends it (without fresh BMS data the hold is off anyway, and the next
  evaluated window decides).
- After `holiday_days` quiet windows in a row, the ceiling `holiday_charge`
  applies (the lower of `holiday_charge` and `partial_charge` if both are set).
  It works like the partial-charge ceiling above, including the hold and the
  periodic full charge every `full_charge_interval` days, but the hold
  releases `holiday_band` (instead of `recharge_dod`) below the ceiling.
- With `holiday_charge=0.6` and the default `holiday_band` of 0.1, the pack
  cycles between about 50 and 60 % SoC. That lies on the same calendar-aging
  plateau as lower storage levels and below the step near 70 % (see
  [LFP longevity](../../internals/lfp-longevity.md)), and it keeps several
  days of base-load reserve. The hold refills towards the ceiling at no more
  than 1 A, so a base load that discharges more overnight than that refills
  during the day slides the pack down to the release point, where normal
  charging takes it back up to the ceiling.
- Holiday mode ends when a window closes with a mean at or above
  `holiday_ibat`, or earlier, as soon as the running window has discharged for
  3 h at a mean above 1.2 × `holiday_ibat`. The pack then charges to full again
  (or to `partial_charge`).
- The holiday state, the quiet-day streak and the console override survive a
  software reset (RTC memory), not a power loss.

Set `holiday_ibat` to about 1.5 times the pack's idle discharge current (standby
consumers, BMS, inverter idle), measured from the BMS current on days when the
installation is unattended. Below the idle current holiday mode never engages;
far above it, normal use counts as quiet. There is no default: the mode
refuses to load without it.

The console command `holiday on|off|auto` overrides the detection: `on` applies
the holiday ceiling now, `off` disables it, and `auto` returns to the detection.
`status` shows the holiday state, the mode, the quiet-day streak, the
unevaluated windows and the discharge time and mean current of the running
window. The configuration must satisfy `0 < holiday_band < holiday_charge`;
without `bat_c` the auto mode is disabled.

## Pack temperature (`bat_temp_*`)

When `mqtt.conf:bat_temp_topic` is configured (up to four BMS sensors), the
charger limits the pack current by temperature:

- If the coldest sensor is below `bat_temp_min` (default 0 °C), the same
  load-follower as the partial hold holds the pack current at zero. Loads are
  still served from PV, and discharging a cold pack is fine. The cold hold has
  no voltage floor, so a cold pack may sit at any SoC. The hold releases 2 °C
  above `bat_temp_min`.
- If the hottest sensor is above `bat_temp_derate` (45 °C), the *pack* current
  limit `ibat_max` scales linearly to zero at `bat_temp_max` (55 °C), and the
  load-follower regulates the BMS pack current to that limit. Because it
  regulates the pack current and not this converter's output, it holds on a
  shared bus, where neither converter can tell the load from the sibling.
- Without a live `ibat` (no topic, or the stream stopped for 180 s), both
  fall back to an output-current limit: 0.25 A when cold, and the derated
  `ibat_max` when hot.

The charger drops a sensor whose reading is outside −40…100 °C. Each sensor
expires an hour after its last frame. With none left, the policy switches off
(as without a sensor). Without a topic, nothing changes.
