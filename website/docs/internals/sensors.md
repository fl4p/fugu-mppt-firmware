---
title: Sensors
sidebar_position: 4
---

# Sensors

The firmware implements a sensor layer that abstracts asynchronous ADC reads and supports multiple ADCs at once.
A sensor represents an ADC channel, which is usually one physical pin, or two pins for a differential channel.

## What the hardware must measure

| Signal | Required | Notes |
|---|---|---|
| `Vout` | Yes | Battery voltage; should be precise and sampled fast to limit transients on load changes |
| `Vin` | Yes | May be coarse (an 8-bit ADC can do if `Iout` is measured); needed for diode emulation in DCM and UV/OV shutdown |
| `Iin` or `Iout` | At least one | The missing side is computed by a `VirtualSensor` from the voltage ratio and `power_conversion_eff` |
| `ntc` | Optional | Heatsink temperature for derating |

A bidirectional current sensor allows boost operation (lower solar voltage to higher battery voltage).

## ADC backends

`AsyncADC<float>` (`src/adc/adc.h`) is the interface. "Asynchronous" means a conversion is requested and the code
continues while it runs, so a slow ADC does not stall other work.

| Backend | Hardware |
|---|---|
| `ADC_ESP32_Cont` | ESP32/ESP32-S3 internal ADC in continuous (DMA) mode, see [Internal ADC](../guide/hardware/internal-adc.md) |
| `ADC_ADS` | ADS1015 / ADS1115 over I²C |
| `ADC_INA226` | INA226 current/voltage monitor over I²C |
| `ADC_Fake` | Sinusoidal mock, for tests and dev boards |

`sensor.conf` selects the backend per channel with `<chn>_adc` and the channel with `<chn>_ch` (`255` = absent).
See the [configuration reference](../reference/config/sensor.md).

## Types

| Type | Role |
|---|---|
| `ADC_Sampler` | Schedules ADC reads round-robin, owns sensors and their calibration |
| `Sensor` | A physical sensor with running statistics (mean, variance) |
| `VirtualSensor` | A computed sensor, also with running statistics |
| `LinearTransform` | `y = (x − midpoint)·factor` scaling; derived from `*_rh`/`*_rl` for voltage dividers |
| `CalibrationConstraints` | Bounds (mean, standard deviation) a sensor must meet during zero-current calibration |

:::warning
`vout` must remain the last sensor added in `setupSensors()`. It then has the lowest latency, which the
over-voltage protection relies on.
:::

Filtering (notch, moving median, EWM) is described in [Signal Filters](signal-filters.md).
