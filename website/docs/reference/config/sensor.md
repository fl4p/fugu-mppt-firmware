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
| `esp32adc1_inl`                  |      | bool   | 0       | Apply the built-in ESP32-S3 ADC1 INL correction (12 dB attenuation only). It removes a ±0.2 V S-curve on a 28:1 divider between ~0.57 and ~2.66 V at the pin and tapers to zero outside that range. It has no gain or offset of its own, so set it together with each voltage channel's `_gain`/`_offset`. It applies to every ADC1 channel, including NTC and current |

The following per-channel keys take a channel prefix. Name the voltage and current channels by the
board side they measure: `hv_v_`, `lv_v_`, `hv_i_`, `lv_i_`. The temperature channel is `ntc_`.

`converter.conf::topo` maps each side to its converter role, so changing the topology does not
require editing `sensor.conf`:

| side key prefix | buck (in = HV, out = LV) | boost (in = LV, out = HV) |
|-----------------|--------------------------|---------------------------|
| `hv_v_`         | Vin                      | Vout                      |
| `lv_v_`         | Vout                     | Vin                       |
| `hv_i_`         | Iin                      | Iout                      |
| `lv_i_`         | Iout                     | Iin                       |

An unknown `converter.conf::topo` fails sensor setup. The firmware no longer reads the old role
prefixes (`vin_`, `vout_`, `iin_`, `iout_`): a file that still has one fails sensor setup at boot.
See [Migrating from role keys](#migrating-from-role-keys).

The `vconv` and `fake` simulators' channels are fixed by role (`src/adc/vconv.h`: 0 = Vin, 1 = Vout,
2 = Iout, 4 = NTC; `src/adc/mock.h`), not by board side. Their profiles use the side key that plays
that role under the profile's `topo`, so a `topo` change there means moving the channel numbers too.

Current sign: a side current `_factor` is defined in the buck direction, positive when power flows
from HV to LV. In boost the firmware negates it, so Iin and Iout stay positive for forward power
flow. The same shunt therefore keeps the same `_factor` in both topologies.

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

There are four sensors: Vin, Vout, Iin, and Iout. Configure them by side (HV/LV); the topology
assigns the roles. A converter can have one or two current sensors.
With a single current sensor, the firmware computes the other from the voltage ratio and
`power_conversion_eff`. The tables above list all keys.

The following example mixes backends as the Fugu2 board image (`config/fmetal`) does. The channel
numbers belong to that board: the HV side voltage is on internal ADC1 channel 3, and the LV side
voltage and current are on the INA226, which only has channel 0 (bus voltage) and channel 1 (shunt
current). The HV side has no current sensor.

```
adc = ina226         # default ADC backend for all channels (ina226, ads1015, ads1115, esp32adc1)
esp32adc1_sr = 22000 # required when any channel uses esp32adc1
esp32adc1_avg = 32

hv_v_adc = esp32adc1 # per-channel backend override
hv_v_ch = 3          # HV voltage: ADC1 channel 3
hv_v_rh = 200e3      # voltage divider, upper (high-side) resistor
hv_v_rl = 7.5e3      # voltage divider, lower resistor

lv_v_ch = 0          # LV voltage: INA226 channel 0 (bus voltage)
lv_v_rh = 47e3       # voltage divider, upper resistor
lv_v_rl = 47e3       # voltage divider, lower resistor

lv_i_ch = 1          # LV current: INA226 channel 1 (shunt)
lv_i_factor = -1     # raw -> A scale, positive for HV -> LV power flow
lv_i_midpoint = 0    # zero offset
lv_i_filt_len = 30   # filter window (samples)

#hv_i_ch = 255       # 255 = absent -> a virtual sensor
#hv_i_factor = -20.15 # sensitivity (A/V)
#hv_i_midpoint = 1.88 # zero offset (e.g. ACS712)
#hv_i_filt_len = 30

expected_hz = 80             # loop-rate watchdog lower bound (0 disables)
power_conversion_eff = 0.97  # assumed efficiency for the virtual current sensor
```

As a buck this reads Vin from ADC1 and Vout and Iout from the INA226, with Iin virtual. With
`topo=boost` the same file reads Vin and Iin from the INA226 and Vout from ADC1, with Iout virtual.

### ADC

Pick the ADC backend with `adc`, or per channel with `<channel>_adc`. The firmware implements these backends:

* `ina226`
* `ads1115`
* `ads1015`
* `esp32adc1`: internal continuous-mode ADC, no external chip (see [Internal ADC](../../guide/hardware/internal-adc.md))

This backend is planned:

* `ina228`

### Voltage sensors `hv_v`, `lv_v`

The firmware computes the gain of a voltage channel from the resistor values of the ADC input voltage
divider and the hardcoded ADC input impedance. Specify both resistors, as in this example:

```
lv_v_rh = 47e3    # upper resistor of voltage divider
lv_v_rl = 47e3    # lower resistor
```

### ACS712

The ACS712 sensitivity is 66mV/A. A 10k+3.3k voltage divider scales the output to match the ADC
voltage range. `hv_i_factor` encodes both the sensitivity and the divider.

Specify the ACS712 midpoint voltage with `hv_i_midpoint` (or `lv_i_midpoint`). This ACS712 has a
2.5V midpoint, scaled through the same 10k + 3.3k divider: `2.5V * 10k/(10k+3.3k)`. The following
example shows both keys:

```
hv_i_factor=-20.15  # sensitivity = -1/0.066 * (10k+3.3k)/10k
hv_i_midpoint=1.88  # midpoint    = 2.5V * 10k/(10k+3.3k)
```

## Migrating from role keys

Firmware before the side keys read `vin_*`, `vout_*`, `iin_*`, `iout_*` here and `vin_max`,
`vout_max`, `iin_max`, `iout_max` in [limits.conf](limits.md). The current firmware reads only
side keys and stops sensor or limits setup on a leftover role key, naming its replacement:

```
sensor.conf: vout_ch is no longer read; with topo=boost use hv_v_ch
```

`etc/migrate_side_keys.py` converts both files under the board's `converter.conf::topo`
(no `topo` = buck). In a boost it also flips the sign of the current `_factor`, because side
factors are positive for HV → LV power. `vin_min`, `iout_short`, `p_max`, `ntc_*` and
`charger.conf::vout_max` keep their names.

- **Config directory** (profile or backup): rewrites `sensor.conf` and `limits.conf` in place,
  keeping comments and order. Already migrated files stay unchanged; a role key next to its own
  side key is refused. `--check` only reports.

  ```
  python3 etc/migrate_side_keys.py dir config/my_board
  ```

- **Live board**: migrate its configs *before* the OTA to the new firmware, otherwise sensor setup
  fails at boot. Capture `get-config converter.conf`, `get-config sensor.conf` and
  `get-config limits.conf` into a file, then print the console commands and run them over BLE or
  telnet:

  ```
  python3 etc/migrate_side_keys.py live board.log      # set-config ... / del-config ... lines
  python3 etc/migrate_side_keys.py live board.log --rollback
  ```

  The `set-config` lines come first, so an interrupted run leaves the role keys in place. The old
  firmware reads the configs only at boot, so OTA right after migrating: rebooting the old
  firmware on migrated configs leaves it without sensors. `--rollback` prints the inverse
  sequence. Without a captured `converter.conf`, pass `--topo buck|boost`.
