*this document is an LLM generated placeholder*


# Charge Termination

`Li_ChgTerminationCondition` in `src/charger.h` implements a charge-termination
line for LFP and other lithium chemistries, as described in
[Charging Marine Lithium Battery Banks](https://nordkyndesign.com/charging-marine-lithium-battery-banks/).

The model: the cell sits at `cv_min` (the "float" voltage) at zero current and
at `cv_eoc` (the absorption voltage) when the charging current equals the
tail-current threshold. Between those two operating points, a straight line in
the (current, voltage) plane defines the termination boundary — anything above
the line is "still charging," anything below is "done."

Restated as an apparent series resistance:

```
r = (cv_eoc - cv_min) / (tail_c_rate * Cbat)
```

so the per-current termination voltage becomes

```
v_term(ibat) = cv_min + ibat * r       (clamped at cv_eoc)
```

The charger declares termination when the highest reported cell voltage
crosses `v_term`.

## The `tail_c_rate` parameter

`tail_c_rate` is the C-rate at which the cell is considered "fully charged" —
the tail current expressed as a fraction of capacity. It's configurable via
`charger.conf:tail_c_rate` and defaults to `0.05` (LFP).

| Chemistry                            | `tail_c_rate` |
|--------------------------------------|---------------|
| LFP                                  | `0.05`        |
| Sanyo/Panasonic NCR18650GA (67 mA / 3500 mAh, [datasheet](https://www.orbtronic.com/content/Datasheet-specs-Sanyo-Panasonic-NCR18650GA-3500mah.pdf)) | `~0.02` |
| EVE INR18650                         | `0.033`       |

**Higher = safer.** A larger `tail_c_rate` produces a steeper termination line,
so termination triggers at a higher current — i.e. *earlier* in the
constant-voltage phase, leaving the cell less fully charged but with more
headroom against over-charge.

**Lower = more thorough but tighter margin.** Smaller `tail_c_rate` means the
charger holds CV down to a smaller tail current before terminating; the cell
ends up closer to its true full-charge state but the cell-voltage tolerance is
narrower.

For unknown cells, the default `0.05` is the conservative choice — termination
may be slightly early for chemistries that prefer a smaller tail, but it will
never be premature on LFP and never over-charges.

## Recharge hysteresis (DoD-based release)

LFP cells discharge nearly flat: a 1 % SoC drop from full can take voltage from
3.45 V down to 3.35 V. Releasing termination on `vcell_high < cv_min` alone
therefore causes micro-cycling at the top of charge — the charger oscillates
between "terminated" and "charging" every few minutes while the cells barely
move.

To fix this, the charger integrates `-ibat` over time (positive = pack
discharging) and tracks Ah-since-the-last-full event. Termination releases
when *either* of:

- **Ah condition (primary)**: `ahSinceFull > recharge_dod * Cbat`. With the
  default `recharge_dod = 0.20` and a 280 Ah pack, this is ~56 Ah of net
  discharge before recharge is permitted.
- **Voltage floor (fallback)**: `vcell_high < cv_min - 0.05 V` (3.32 V for
  LFP). Catches integrator drift, wrong `Cbat`, missing BMS — anything that
  would otherwise leave the charger stuck terminated while the pack is
  genuinely empty. Reliability beats elegance here.

On every termination event the integrator is re-zeroed, so it doesn't have to
be a precise long-term SoC gauge — it's a "deficit since the last known full"
counter that self-recalibrates each cycle.

State is **RAM-only**. On reboot the counter starts at 0, and the next full
charge re-establishes the reference. The first post-boot cycle releases on the
voltage-floor fallback (same as the pre-Ah-counting behaviour).

`recharge_dod` is configurable via `charger.conf:recharge_dod`. Higher = release
later (deeper discharges between full charges = fewer cycles but more time at
mid-SoC). Lower = release sooner (more frequent topping = more cycle wear but
less time below 100 %). LFP off-grid systems commonly run 0.10–0.30.

## Related parameters

- `cv_eoc` — absorption voltage (LFP: 3.65 V)
- `cv_float` (loaded into `cv_min`) — float voltage (LFP: 3.37 V)
- `bat_c` — effective pack capacity in Ah. For parallel packs, use the summed
  Ah (e.g. 2P 280 Ah → 560). The model assumes `Cbat` is the parallel-effective
  capacity.
- `recharge_dod` — DoD threshold for releasing termination (see above).

If `bat_c` is missing, `r` is non-finite and the impedance-compensation branch
degrades to "absorption-only" — pack-voltage pinning still caps at `cv_eoc`
via `Vbat_fallback`, so the charger is safe but does not fully charge. The Ah
release condition is also disabled in that case; only the voltage floor remains.

## Absorption target vs. termination line

The EOC feedback loop (`BatteryCharger::_updatePackVoltagePinning`) lowers the
pack-voltage pin whenever the highest cell is above its target. That target is
the fixed `cv_eoc` while charging and `cv_min` once terminated. It must not be
the current-dependent `v_term`: lowering the pin reduces the current, which
lowers `v_term`, which lowers the pin again, until the current is zero and the
line trips at `cv_min` — the premature termination seen on fry/flat in July
2026 at 3.40 V/cell. The line only *decides* termination; it is not a setpoint.

The line trigger, like the ceiling, needs two consecutive BMS cell frames above
the line (`termCond.update()` runs once per cell frame).

## Partial-charge ceiling (`partial_charge`)

Recharge hysteresis alone keeps the pack in the 80–100 % window. To lower the
average SoC — the best-evidenced LFP lifetime lever, see
`LFP Longevity Research.md` — the charger can stop short of full:

- After a termination (the only point where the Ah counter is known to be at
  zero), charging stops once `ahSinceFull <= (1 - partial_charge) * Cbat` and
  the pack is **held** there by load-following: on every BMS `ibat` frame the
  pin steps by up to 20 mV (4 mV/A, 0.2 A deadband) so the pack current goes to
  a small target and the converter covers the load only. The target is trimmed
  by the Ah error (±1 A at 2 Ah off the ceiling) so a BMS current offset inside
  the deadband cannot walk the SoC away over the week. The hold starts from the
  measured bus voltage and only steps while the converter drives the bus (not
  at night). The pin may fall to
  `n_cells * (cv_float - recharge_vfloor_band) - vout_offset_max` and rise to
  `Vbat_max`; if the load exceeds the PV the pin sits at `Vbat_max` and the
  pack discharges normally.
- The hold releases when the deficit exceeds the ceiling by `recharge_dod`
  (pack cycles between `partial_charge - recharge_dod` and `partial_charge`).
- Every `full_charge_interval` days the ceiling is dropped and the pack charges
  to full for BMS balancing; a reboot also charges to full first, because the
  deficit counter starts unknown.
- The hold needs live BMS data: if the cell-voltage or the `ibat` stream stops
  (180 s) the hold is dropped and the charger falls back to its ordinary
  behaviour; it re-engages when the data returns.
- The hold does not block the dawn start (unlike termination): the converter
  has to run to serve the loads, and the pin keeps the pack current at zero.

`status` reports `PARTIAL HOLD (load-following)`, the ceiling and the age of the
last full charge. The periodic re-sweep and the stuck watchdog treat the hold
like termination. Config: `0 < recharge_dod < partial_charge`, else the release
band would reach below 0 % SoC; without `bat_c` the ceiling is disabled.

## Pack temperature (`bat_temp_*`)

With `mqtt.conf:bat_temp_topic` configured (up to four BMS sensors):

- coldest sensor below `bat_temp_min` (default 0 °C): the pack current is held
  at zero by the same load-follower as the partial hold (loads are still served
  from PV; discharging a cold pack is fine). The cold hold has no voltage floor,
  a cold pack may sit at any SoC. Released 2 °C above.
- hottest sensor above `bat_temp_derate` (45 °C): the *pack* current limit
  `ibat_max` scales linearly to zero at `bat_temp_max` (55 °C), and the
  load-follower regulates the BMS pack current to that limit. Regulating the
  pack current rather than this converter's output is what makes it hold on a
  shared bus, where neither converter can tell the load from the sibling.
- without a live `ibat` (no topic, or the stream stopped for 180 s) both fall
  back to an output-current limit: 0.25 A when cold, the derated `ibat_max`
  when hot.

Readings outside −40…100 °C drop that sensor. Each sensor expires an hour after
its last frame; with none left the policy switches off (as without a sensor).
Without a topic nothing changes.
