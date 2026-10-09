---
title: Examples
sidebar_position: 19
---

# Examples

## Bare ESP32-S3 example

This minimal `sensor.conf` runs the firmware on an ESP32-S3 with no external ADC. The channel→GPIO comments are S3 mappings.

```
adc = esp32adc1
esp32adc1_sr = 83333  # continuous-ADC sample rate, required
esp32adc1_avg = 32    # samples averaged per reading (1–1023), required

vin_ch = 4          # ch4 = GPIO5
vin_rh = 200e3      # voltage divider, upper resistor
vin_rl = 1.5e3      # voltage divider, lower resistor

vout_ch = 5         # ch5 = GPIO6
vout_rh = 47e3      # voltage divider, upper resistor
vout_rl = 1e3       # voltage divider, lower resistor

iin_ch = 3          # ch3
iin_factor = 20
iin_midpoint = 0
iin_filt_len = 30

iout_filt_len = 30  # Iout is the virtual sensor here

expected_hz = 80
power_conversion_eff = 0.97

ignore_calibration_constraints = 1  # skip noise/range checks (NOT for production!)
```

A bare ESP32 setup is useful for testing things other than the ADC and PWM. With the ADC pins left
floating, the readings are garbage with a high stddev. That's why this example needs
`ignore_calibration_constraints = 1`.
