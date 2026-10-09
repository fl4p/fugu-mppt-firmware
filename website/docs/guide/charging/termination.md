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
  the pack to full for BMS balancing. A reboot also charges to full first,
  because the deficit counter starts unknown.
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
