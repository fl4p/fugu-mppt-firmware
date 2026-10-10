---
title: First Power-Up
sidebar_position: 4
---

# First Power-Up

Bring a new board up in two stages: first on a current-limited bench supply, then on the real panel and battery.
Once the first-stage readings make sense, move on to the second stage.

To try the firmware on a board without a power stage, see [Mock ADC](../../development/testing.md#mock-adc).

:::danger High voltage and high current
A solar array delivers its short-circuit current into a fault, and a battery delivers far more. A firmware or
configuration error can put the full panel voltage on the battery terminals or short the half-bridge. Work with
fused wiring, keep a hand on a disconnect, and never leave a first bring-up unattended.
:::

## Checklist

The following table summarizes the configuration, power source, and checks for each stage.

| Stage | Config | Power | Verify |
|---|---|---|---|
| 1 | your board config | current-limited lab supply on the input | boot log clean, `sensor` readings match a multimeter, limits hold |
| 2 | your board config | panel + battery (+ BMS) | charging, termination, telemetry |

## Stage 1: bench supply

Provision your real board configuration (for Fugu2: `./provision.py fmetal`) and power the input from a lab supply.

### Before you switch on

Check the following items before you power the board:

- [ ] Review [`limits.conf`](../../reference/config/limits.md) with `cat conf/limits.conf`. Set `hv_max`,
  `lv_max`, `hv_i_max`, and `lv_i_max` to what your hardware and your battery tolerate, not to the example values.
- [ ] `charger.conf::vout_max` matches the battery you will connect later
  ([LFP charging](../charging/lfp-charging.md)).
- [ ] `coil.conf::L0` matches your inductor; diode emulation relies on it
  ([diode emulation](../../internals/diode-emulation.md)).
- [ ] Supply voltage above `limits.conf::vin_min` (10.5 V in the examples, protects the board supply) and below
  the input side's maximum (`hv_max` in a buck, `lv_max` in a boost).
- [ ] Supply current limit set low (a few hundred mA) for the first run.

### Readings

With only the input supply connected and the output open, check the following readings:

- [ ] No `E (…)` error lines in the boot log. `board.conf expects MCU …` means the config does not match the chip.
  See [ESP32 variants](../hardware/esp32-variants.md).
- [ ] `sensor`: Vin matches a multimeter within a few percent. If not, fix that side's divider (`hv_v_rh`/`hv_v_rl` in a buck) in
  [`sensor.conf`](../../reference/config/sensor.md).
- [ ] Current readings sit near zero. The sampler calibrates the zero-current offset at start.
- [ ] `status` shows the limits you configured.
- [ ] While idle the log names what blocks a start, for example `START blocked: Vin-Vout (Vin=12.4 Vout=12.9 …)`. A buck
  needs Vin above Vout + 1 V; a boost needs Vin below Vout + 1 V.

Then connect a load or a second supply/battery simulator on the output and let the converter start. Watch `status`
and the log. The firmware logs every protection trip with its reason.

:::warning Manual duty
`dc N` and `+N`/`-N` drive the half-bridge directly (protections stay active). Start with small values and small
steps. Large positive jumps can cause current transients that destroy the switches. `dc 0` always stops the
converter. See [Operating modes](../operating-modes.md).
:::

## Stage 2: panel and battery

Work through these steps in order:

- [ ] Connect the battery first, then the panel.
- [ ] Confirm Vout in `sensor` equals the battery voltage before the first sweep.
- [ ] Let the converter start on its own: it runs a global sweep, then tracks the MPP.
- [ ] If a BMS publishes cell voltages over MQTT, set up [BMS integration](../charging/bms-integration.md) and check
  that `status` shows a fresh `vcell_high`.
- [ ] Add [telemetry](../telemetry/index.md) to watch the first days of operation.

:::danger Output over-voltage
If the battery or load is removed during conversion, expect an over-voltage transient at the output (measured: 36 V
for 400 ms on a 28.5 V system). Add over-voltage protection (TVS, crowbar, second DC/DC) where connected devices
cannot tolerate that or a converter failure that puts the full panel voltage on the output.
:::

## Common scenarios

The following table lists common symptoms during bring-up and where to look for the cause.

| Symptom | Where to look |
|---|---|
| `Never got a sample! Please check ADC` | ADC backend, I²C pins, `ina22x_*`/`ads_alert` pins in `board.conf` |
| `Calibration failed, <sensor> …` | Sensor not at rest at boot, or wrong `_midpoint`/`_factor` |
| `START blocked: supply-UV` | The higher of Vin and Vout is below ~9.8 V, too low for the board supply |
| `START blocked: temp` | NTC or MCU temperature within 3 °C of `limits.conf::temp_max`, or MCU temperature unavailable; check `ntc_ch` |
| Protection trips at low power | Divider values or current `_factor` sign in `sensor.conf` |

For more symptoms, see [Troubleshooting](../troubleshooting.md).
