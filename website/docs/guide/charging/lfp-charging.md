---
title: "LFP Battery Charging"
sidebar_position: 1
---

*this document is an LLM generated placeholder*


# LFP Battery Charging

This document describes how the firmware charges a Lithium Iron Phosphate
(LFP / LiFePO4) battery, what each configurable parameter does, and how the
termination and recharge logic work. For a strictly developer-facing view of
the same algorithm, see [Termination.md](termination.md).

## Overview

The charger drives a synchronous DC/DC converter to push solar power into an
LFP battery pack. It is designed to work with a **Battery Management System
(BMS)** that publishes per-cell voltages and pack current over MQTT — the BMS
is the source of truth about how full the pack actually is, and the firmware
uses that data to decide when to taper charging, when to hold, and when to
back off.

If the BMS goes offline, the charger keeps running but clamps the pack
voltage at a conservative fallback level until BMS communication resumes.

### Charging phases

During a normal day the charger moves through these phases:

1. **MPPT** — the panel is producing and the pack is below the per-cell
   end-of-charge voltage. The charger pulls maximum power.
2. **Pack-voltage pinning (absorption)** — the highest cell has reached the
   end-of-charge voltage. The charger clamps the pack voltage so the highest
   cell stays at or below that threshold. Charging current naturally tapers
   as the cells fill up.
3. **Termination** — the pack is full (the highest cell has crossed the
   termination line, see below). The charger backs off.
4. **Recharge wait** — termination is held until the pack has discharged by
   a configurable amount. This avoids constant micro-cycling at the top of
   charge, which shortens cycle life on LFP cells.

The charger transitions MPPT → pinning → termination → recharge wait → MPPT
in a normal cycle.

## Configuration

All charging parameters live in `charger.conf` on the device's storage
partition. Edit them with `set-config charger.conf <key> <value>` and then
`restart`: the file is read at boot only. `vset <V>` / `iset <A>` change
`vout_max` / `ibat_max` immediately but are lost on reboot.

:::danger
`vout_max_fallback=0` does **not** disable the converter, and `ibat_max` limits
the converter output, not the battery current. A runtime current limit of 0 is
not a stop. See the table below.
:::

| Key                  | Unit      | Range                              | Description |
| -------------------- | --------- | ---------------------------------- | ----------- |
| `vout_max`           | V         | > 0                                | Nominal maximum pack voltage. Hard upper bound for the converter setpoint. Typical: ~14.6 V (4s LFP), ~29 V (8s), ~57 V (16s). |
| `cv_eoc`             | V         | ≥ `cv_float`                       | End-of-charge voltage **per cell** (absorption ceiling). The charger keeps the highest cell at or below this. Typical LFP: 3.55–3.65. |
| `cv_float`           | V         | 0 < `cv_float` ≤ `cv_eoc`          | Float voltage **per cell**, i.e. where the termination line crosses zero current. Typical LFP: 3.37. |
| `vout_max_fallback`  | V         | > 0                                | Pack-voltage limit used when BMS data is unavailable, and the float target after termination. Default `N_cells × cv_float`. Must be a positive voltage: `0` does **not** disable the converter. A non-positive pin is ignored and the limit returns to `vout_max`, both when the BMS goes stale and during terminated float. |
| `ibat_max`           | A         | > 0, capped at `limits.conf:iout_max` | Output-current limit of the converter (charge current plus any connected loads). The firmware does not compensate for loads, so with loads the battery receives less than this. Default 20 A. A runtime limit of 0 (`iset 0`, `ibat_lim_topic`) still allows 0.25 A; it is not a stop. |
| `bat_c`              | Ah        | > 0 (pack-effective)               | Pack capacity. For parallel packs use the summed Ah (e.g. 2P × 280 Ah → 560). Used by both the termination line and the recharge hysteresis. If unset, both features degrade — see below. |
| `tail_c_rate`        | unitless  | > 0                                | Ratio of "fully-charged" tail current to capacity. Default 0.05 (LFP). See the table further down for other chemistries. |
| `recharge_dod`       | fraction  | 0 < x < 1                          | Depth of discharge below the last full-charge event at which the charger is permitted to recharge. Default 0.20. LFP off-grid typical: 0.10–0.30. |

The BMS link itself is configured separately in `mqtt.conf`
(`cell_voltages_max_topic` for the highest cell voltage and `ibat_topic` for
pack current). Without those topics configured, the charger runs in
fallback mode.

## Termination logic

The pack is considered "full" when the highest cell sits at or above a
**current-dependent voltage threshold**. The threshold is a straight line in
the (current, cell-voltage) plane: at zero charging current it sits at the
float voltage `cv_float`; at the configured tail current it reaches the
end-of-charge voltage `cv_eoc`. Above the line means the cell is more
charged than the line predicts for its current — i.e. the pack is full.

For pack current $I_\text{bat}$ (positive when charging), per-cell float
voltage $V_\text{float}$ (`cv_float`), end-of-charge voltage $V_\text{EoC}$
(`cv_eoc`), tail-current ratio $r_\text{tail}$ (`tail_c_rate`), and
pack-effective capacity $C$ (`bat_c`):

$$
V_\text{term}(I_\text{bat}) = \min\!\left(\, V_\text{float} + I_\text{bat} \cdot \frac{V_\text{EoC} - V_\text{float}}{r_\text{tail} \cdot C},\; V_\text{EoC} \,\right)
$$

The clamp at $V_\text{EoC}$ keeps the threshold from rising above the
absorption ceiling at high charging currents. A "dumb" BMS may cut off at
$V_\text{EoC}$ regardless of current, and we want to stay below that line to
avoid voltage transients.

Termination latches when the **highest cell voltage** (reported by the BMS)
rises above $V_\text{term}$ on two consecutive BMS cell frames while the pack
is being charged.

<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 600 410" font-family="sans-serif" font-size="12">
  <text x="300" y="20" text-anchor="middle" font-size="14" font-weight="bold">LFP cell termination line</text>
  <text x="300" y="38" text-anchor="middle" font-size="11" fill="#666">cv_float = 3.37, cv_eoc = 3.65, tail_c_rate = 0.05</text>

  <!-- still-charging region (above line) -->
  <polygon points="80,350 80,50 500,50 500,100 164,350" fill="#c8e6c9" />

  <!-- plot area border -->
  <rect x="80" y="50" width="480" height="300" fill="none" stroke="#999" />

  <!-- axes -->
  <line x1="80" y1="350" x2="560" y2="350" stroke="black" stroke-width="1.5" />
  <line x1="80" y1="50" x2="80" y2="350" stroke="black" stroke-width="1.5" />

  <!-- x ticks -->
  <g text-anchor="middle">
    <line x1="80" y1="350" x2="80" y2="355" stroke="black" /><text x="80" y="370">3.30</text>
    <line x1="200" y1="350" x2="200" y2="355" stroke="black" /><text x="200" y="370">3.40</text>
    <line x1="320" y1="350" x2="320" y2="355" stroke="black" /><text x="320" y="370">3.50</text>
    <line x1="440" y1="350" x2="440" y2="355" stroke="black" /><text x="440" y="370">3.60</text>
    <line x1="560" y1="350" x2="560" y2="355" stroke="black" /><text x="560" y="370">3.70</text>
  </g>
  <text x="320" y="395" text-anchor="middle">Highest cell voltage (V)</text>

  <!-- y ticks -->
  <g text-anchor="end">
    <line x1="75" y1="350" x2="80" y2="350" stroke="black" /><text x="72" y="354">0.00</text>
    <line x1="75" y1="275" x2="80" y2="275" stroke="black" /><text x="72" y="279">0.015</text>
    <line x1="75" y1="200" x2="80" y2="200" stroke="black" /><text x="72" y="204">0.030</text>
    <line x1="75" y1="125" x2="80" y2="125" stroke="black" /><text x="72" y="129">0.045</text>
    <line x1="75" y1="50" x2="80" y2="50" stroke="black" /><text x="72" y="54">0.060</text>
  </g>
  <text x="20" y="200" text-anchor="middle" transform="rotate(-90 20 200)">Charge current (C-rate)</text>

  <!-- termination line -->
  <polyline points="80,350 164,350 500,100 500,50" fill="none" stroke="#1565c0" stroke-width="2.5" />

  <!-- labels -->
  <text x="170" y="130" fill="#2e7d32" font-weight="bold">still charging</text>
  <text x="430" y="320" fill="#888" font-style="italic">terminated</text>
</svg>

The line is the termination boundary. In the **shaded (green) region above
the line** the cell is still accepting meaningful charge — the cell voltage
is below the threshold for the current charging rate. **Below the line**
further charging at that current would only push voltage up without
storing useful energy, and termination latches. The full algorithm follows
[Charging Marine Lithium Battery Banks](https://nordkyndesign.com/charging-marine-lithium-battery-banks/),
which is the original reference for this approach.

## The `tail_c_rate` parameter

`tail_c_rate` is the C-rate at which a cell is considered "fully charged" —
the tail current expressed as a fraction of capacity. Geometrically, it
controls the steepness of the termination line: lower means a shallower line
(the cell must accept very little current before being considered full);
higher means a steeper line (more current is still permitted at the EoC
voltage).

| Chemistry                                       | `tail_c_rate` |
| ----------------------------------------------- | ------------- |
| LFP (LiFePO4)                                   | 0.05          |
| Sanyo/Panasonic NCR18650GA (67 mA on 3500 mAh)  | ~0.02         |
| EVE INR18650                                    | 0.033         |

**Higher = safer.** A larger `tail_c_rate` produces a steeper termination
line, so termination triggers at a higher current — earlier in the
constant-voltage phase, leaving the cell less than 100 % full but with more
headroom against over-charge.

**Lower = more thorough but tighter margin.** Smaller `tail_c_rate` means
the charger holds the EoC voltage down to a smaller tail current before
terminating; the cell ends up closer to true full charge, but the
voltage tolerance is narrower.

For unknown cells the default `0.05` is the conservative choice — termination
may be slightly early for chemistries that prefer a smaller tail, but it will
never be premature on LFP and never over-charges.

## Recharge hysteresis (DoD-based release)

LFP cells discharge nearly flat: a single percent of state-of-charge drop
from full can take cell voltage from 3.45 V down to 3.35 V. If termination
were released as soon as the highest cell fell below the float voltage, the
charger would oscillate between "terminated" and "charging" every few
minutes while the cells barely moved. That micro-cycling shortens cycle life
on LFP unnecessarily.

To fix this, the firmware integrates pack current over time and tracks
**Ampere-hours discharged since the last full-charge event**. Termination
releases (charging is permitted again) when *either* of:

- **Discharge condition (primary)** — the integrated discharge exceeds
  `recharge_dod × bat_c`. With the default `recharge_dod = 0.20` and a
  280 Ah pack, that's about 56 Ah of net discharge before recharging is
  permitted.
- **Voltage floor (fallback)** — the highest cell stays below
  `cv_float − recharge_vfloor_band` (band default 0.05 V) for 4 consecutive
  BMS cell frames: 3.275 V with the firmware defaults, 3.32 V with
  `cv_float=3.37`. This is well below normal
  LFP operating voltage and only trips when the pack is genuinely deep into
  discharge. It exists as a belt-and-suspenders catch for cases where the
  Ah counter cannot be trusted: wrong `bat_c`, BMS reporting bad current,
  long communication outages. Reliability beats elegance here.

On every termination event the Ah counter is **re-zeroed**, so it doesn't
need to be a precise long-term state-of-charge gauge — it's a "discharge
deficit since the last known full" counter that self-recalibrates each
cycle.

The counter and the termination latch are **RAM-only**. After a reboot the
charger is not terminated, so it charges until termination latches again
(within two cell frames if the pack is still full). That termination
re-zeroes the counter, and from then on both release conditions apply as
usual.

**Tuning `recharge_dod`.** Higher values mean deeper discharges between full
charges — fewer full cycles per year and more time at mid state-of-charge
(which LFP prefers). Lower values mean more frequent topping up — more
cycle wear, but the pack spends more time near 100 %. For LFP off-grid
systems, 0.10–0.30 is typical; 0.20 is a reasonable default.

## When `bat_c` is missing

:::warning
Set `bat_c` on any battery system. Without it the termination line and the
per-cell EOC feedback are disabled.
:::

If `bat_c` is not configured:

- The termination line and the EOC feedback on the highest cell are
  disabled. The output is held at the pack-level `vout_max_fallback`
  (default `N_cells × cv_float`), open-loop on the Vout reading, like a
  stale BMS.
- Only the hard `cv_ceiling` latch can still flag termination, and it does
  not lower the output.
- The Ah recharge condition is disabled; only the voltage floor releases
  termination.

The pack floats near `cv_float` per cell and does not reach full, and the
highest cell is not individually limited. The firmware logs a warning on
boot if `bat_c` is unset.

## When the BMS goes offline

If MQTT delivery of cell-voltage data stops for more than 3 minutes, the
BMS is considered offline. In that state:

- **Pack-voltage pinning is disabled** — there's no per-cell data to pin
  against. The pack-voltage limit drops to `vout_max_fallback` over a
  5-second linear glide, avoiding a sudden setpoint step on the converter.
- **The termination decision is frozen** — the latched state (terminated or
  not) is held until BMS data returns.
- **The Ah counter pauses** — current integration drops gaps longer than
  30 seconds rather than extrapolating across them.

When the BMS reconnects, normal operation resumes from the existing state.



## TODO mention here:
* charger robust to ADC gain error (using own Vout measurement to pin voltage, eliminates error)
* robust with multiple chargers without talking to each other