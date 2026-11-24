---
title: Sensors
sidebar_position: 2
description: "The firmware implements a sensor layer that abstracts asynchronous ADC reads and supports multiple ADCs at once."
---

# Sensors

*Original hand-written notes: [doc/Sensors.md](https://github.com/fl4p/fugu-mppt-firmware/blob/main/doc/Sensors.md) and the README section [Voltage & Current Sensors](https://github.com/fl4p/fugu-mppt-firmware#voltage--current-sensors-adc).*

The firmware implements a sensor layer that abstracts asynchronous ADC reads and supports multiple ADCs at once.
A sensor represents an ADC channel, which is usually one physical pin, or two pins for a differential channel.

## Required signals

The hardware must measure the following signals.

| Signal | Required | Notes |
|---|---|---|
| `Vout` | Yes | Battery voltage; should be precise and sampled fast to limit transients on load changes |
| `Vin` | Yes | May be coarse (an 8-bit ADC is enough if `Iout` is measured); needed for diode emulation in DCM, the Vin over-voltage trip and the `vin_min` floor; the board-supply under-voltage stop uses the higher of Vin and Vout |
| `Iin` or `Iout` | At least one | The missing side is computed by a `VirtualSensor` from the voltage ratio and a fixed 97 % efficiency |
| `ntc` | Optional | Heatsink temperature for derating |

A bidirectional current sensor allows boost operation (lower solar voltage to higher battery voltage).

## ADC backends

`AsyncADC<float>` (`src/adc/adc.h`) is the interface. "Asynchronous" means a conversion is requested and the code
continues while it runs, so a slow ADC does not stall other work.

The following backends implement this interface.

| Backend | Hardware |
|---|---|
| `ADC_ESP32_Cont` | ESP32 and ESP32-S3 internal ADC in continuous (DMA) mode (`esp32adc1`), see [Internal ADC](../guide/hardware/internal-adc.md) |
| `ADC_ADS` | ADS1015 / ADS1115 over I²C (`ads1015`, `ads1115`; `CONFIG_FUGU_WITH_ADS`) |
| `ADC_INA226` | INA226 current and voltage monitor over I²C (`ina226`; `CONFIG_FUGU_WITH_INA226`) |
| `ADC_Fake` | Sinusoidal mock, for tests and dev boards (`fake`) |
| `ADC_VConv` | Simulated converter, in the simulated-converter build (`vconv`), see [vconv.conf](../reference/config/vconv.md) |
| `ADC_Dummy` | Replays recorded samples, for unit tests only |

`sensor.conf` selects the backend per channel with `<chn>_adc`, using the names in parentheses, and the channel
with `<chn>_ch` (`255` = absent). A backend left out of the build fails setup.
`<chn>` is a board side (`hv_v`, `lv_v`, `hv_i`, `lv_i`) or `ntc`; `converter.conf::topo` maps the sides to Vin,
Vout, Iin and Iout. A channel without `<chn>_adc` uses the backend named by `adc`.
See the [configuration reference](../reference/config/sensor.md).

## Types

The sensor layer consists of the following types.

| Type | Role |
|---|---|
| `ADC_Sampler` | Schedules ADC reads round-robin, owns sensors and their calibration |
| `Sensor` | A physical sensor with running statistics (mean, variance) |
| `VirtualSensor` | A computed sensor, also with running statistics |
| `LinearTransform` | `y = (x − midpoint)·factor` scaling; derived from `*_rh`/`*_rl` for voltage dividers |
| `CalibrationConstraints` | Limits for the start-up calibration with the converter off: largest absolute reading, standard deviation, and whether the zero offset is calibrated (current sensors) |

:::warning
`vout` must remain the last sensor added in `setupSensors()`. It then has the lowest latency, which the
over-voltage protection relies on.
:::

[Signal filters](signal-filters.md) describes the filter chain: the inverter-ripple notch, comb or both, a moving
median (optionally with despike), and an EWMA.
