---
title: sensor.conf
sidebar_position: 2
---

# sensor.conf

`sensor.conf` sets the channel map, the divider ratios, and the calibration of the voltage and current sensors.

The following keys apply to all channels:

| key                              | unit | type   | default | description                                                    |
|----------------------------------|------|--------|---------|----------------------------------------------------------------|
| `adc`                            |      | string | —       | Default ADC backend for all channels                           |
| `expected_hz`                    | Hz   | uint16 | 0       | Minimum control-loop sample rate for the loop-rate watchdog, 0–65535 (0 = off). If the samples per second stay below it for three consecutive ~3 s windows (outside calibration and manual PWM), the converter logs `Loop latency high (…), shutdown!` and backs off. A value outside the range fails sensor setup at boot. Set it below the rate the ADC settings deliver, e.g. `3900` in `config/lab/dry_mock` |
| `power_conversion_eff`           |      | float  | 0.95    | Assumed converter efficiency for the virtual current sensor    |
| `ignore_calibration_constraints` |      | bool   | 0       | Bypass ADC calibration sanity constraints                      |
| `notch_adaptive`                 |      | bool   | 1       | Auto-tune the inverter-ripple notch to the tone measured on Vout (off = fixed at `notch_freq`) |
| `notch_freq`                     | Hz   | float  | 100     | Notch frequency when not adaptive (2×mains: 100 = 50 Hz, 120 = 60 Hz) |
| `notch_q`                        |      | float  | 20      | Notch quality factor (bandwidth ≈ `notch_freq`/`notch_q`)      |
| `despike`                        |      | float  | 0       | Glitch-safe median outlier threshold (running mean-deviation units): 0 = off (legacy unconditional median); ~8 enables (lower = clips more). Passes dense load pulses through (unbiased current) but still clips impulse glitches |
| `esp32adc1_sr`                   | Hz   | int    | —       | Internal ADC1 continuous-mode raw sample rate (required with `esp32adc1`) |
| `esp32adc1_avg`                  |      | int    | —       | Software average of N raw conversions per delivered sample (1–1023, required with `esp32adc1`) |
| `esp32adc1_inl`                  |      | bool   | 1       | Apply the built-in ESP32-S3 ADC1 INL correction (12 dB attenuation only; about ±0.2 V of S-curve on a 28:1 divider). It carries no gain or offset, so calibrate each board with `_gain`/`_offset` |

The following per-channel keys take the prefix `vin_`, `vout_`, `iin_`, `iout_`, or `ntc_`:

| suffix      | unit | type   | default     | description                                                                    |
|-------------|------|--------|-------------|--------------------------------------------------------------------------------|
| `_adc`      |      | string | value of `adc` | ADC backend for this channel                                                |
| `_ch`       |      | byte   | 255         | ADC channel index (255 = absent)                                               |
| `_rh`       | Ω    | float  | —           | Voltage divider upper (high-side) resistor (voltage channels)                  |
| `_rl`       | Ω    | float  | —           | Voltage divider lower resistor (voltage channels)                              |
| `_gain`     |      | float  | 1           | Per-board gain on top of the divider (voltage channels): V = gain · V_div + offset |
| `_offset`   | V    | float  | 0           | Per-board offset (voltage channels), see `_gain`                               |
| `_factor`   |      | float  | 1           | Linear scale factor raw ADC → physical, sign sets direction (current channels) |
| `_midpoint` |      | float  | 0           | Zero/offset midpoint subtracted before scaling (current channels)              |
| `_filt_len` |      | int    | 10          | Filter window length (samples). Currently ignored for `ntc`, which uses a fixed 50 |

For worked ACS712 and bare-ESP32 configs, see [Notes & examples](#notes--examples).


## Notes & examples

### Sensors

Configure the voltage and current sensors in `sensor.conf` to match your topology and chips.

There are four sensors: Vin, Vout, Iin, and Iout. A topology can have one or two current sensors.
With a single current sensor, the firmware computes the other from the voltage ratio and
`power_conversion_eff`. The tables above list all keys.

The following example mixes backends as the Fugu2 board image (`config/fmetal`) does. The channel
numbers belong to that board: Vin is on internal ADC1 channel 3, and Vout and Iout are on the INA226,
which only has channel 0 (bus voltage) and channel 1 (shunt current).

```
adc = ina226         # default ADC backend for all channels (ina226, ads1015, ads1115, esp32adc1)
esp32adc1_sr = 22000 # required when any channel uses esp32adc1
esp32adc1_avg = 32

vin_adc = esp32adc1  # per-channel backend override
vin_ch = 3           # Vin: ADC1 channel 3
vin_rh = 200e3       # voltage divider, upper (high-side) resistor
vin_rl = 7.5e3       # voltage divider, lower resistor

vout_ch = 0          # Vout: INA226 channel 0 (bus voltage)
vout_rh = 47e3       # voltage divider, upper resistor
vout_rl = 47e3       # voltage divider, lower resistor

iout_ch = 1          # Iout: INA226 channel 1 (shunt)
iout_factor = 1      # raw -> A scale (sign sets direction)
iout_midpoint = 0    # zero offset
iout_filt_len = 30   # filter window (samples)

#iin_ch = 255        # 255 = absent -> Iin becomes a virtual sensor
#iin_factor = -20.15 # sensitivity (A/V)
#iin_midpoint = 1.88 # zero offset (e.g. ACS712)
#iin_filt_len = 30

expected_hz = 80             # loop-rate watchdog lower bound (0 disables)
power_conversion_eff = 0.97  # assumed efficiency for the virtual current sensor
```

### ADC

Pick the ADC backend with `adc`, or per channel with `<chn>_adc`. The firmware implements these backends:

* `ina226`
* `ads1115`
* `ads1015`
* `esp32adc1`: internal continuous-mode ADC, no external chip (see [Internal ADC](../../guide/hardware/internal-adc.md))

This backend is planned:

* `ina228`

### Voltage sensors `vin`, `vout`

The firmware computes the gain of a voltage channel from the resistor values of the ADC input voltage
divider and the hardcoded ADC input impedance. Specify both resistors, as in this example:

```
vout_rh = 47e3    # upper resistor of voltage divider
vout_rl = 47e3    # lower resistor
```

### ACS712

The ACS712 sensitivity is 66mV/A. A 10k+3.3k voltage divider scales the output to match the ADC
voltage range. `iin_factor` encodes both the sensitivity and the divider.

Specify the ACS712 midpoint voltage with `iin_midpoint` (or `iout_midpoint`). This ACS712 has a
2.5V midpoint, scaled through the same 10k + 3.3k divider: `2.5V * 10k/(10k+3.3k)`. The following
example shows both keys:

```
iin_factor=-20.15  # sensitivity = -1/0.066 * (10k+3.3k)/10k
iin_midpoint=1.88  # midpoint    = 2.5V * 10k/(10k+3.3k)
```
