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
| `ignore_calibration_constraints` |      | bool   | 0       | Bypass ADC calibration sanity constraints                      |
| `notch_adaptive`                 |      | bool   | 1       | Auto-tune the inverter-ripple notch to the tone measured on Vout (off = fixed at `notch_freq`) |
| `notch_freq`                     | Hz   | float  | 100     | Notch frequency when not adaptive (2×mains: 100 = 50 Hz, 120 = 60 Hz) |
| `notch_q`                        |      | float  | 20      | Notch quality factor (bandwidth ≈ `notch_freq`/`notch_q`)      |
| `ripple_filter`                  |      | string | notch   | Inverter-ripple filter: `notch`, `comb` (moving average over one ripple period, also removes the harmonics, ~5 ms delay) or `both`. Tuned like the notch (`notch_adaptive`, `notch_freq`). See [signal filters](../../internals/signal-filters.md#ripple-comb) |
| `despike`                        |      | float  | 0       | Glitch-safe median outlier threshold (running mean-deviation units): 0 = off (legacy unconditional median); ~8 enables (lower = clips more). Passes dense load pulses through (unbiased current) but still clips impulse glitches. A sample is clipped only if it also deviates by more than 4 quantization steps (learned per sensor). The NTC always uses the plain median and is not counted. A warning is logged when clips exceed 0.5 % of the samples in a window |
| `esp32adc1_sr`                   | Hz   | int    | —       | Internal ADC1 continuous-mode raw sample rate (required with `esp32adc1`) |
| `esp32adc1_avg`                  |      | int    | —       | Software average of N raw conversions per delivered sample (1–1023, required with `esp32adc1`) |
| `esp32adc1_inl`                  |      | bool   | 0       | Apply the built-in ESP32-S3 ADC1 INL correction (12 dB attenuation only). It removes a ±0.2 V S-curve on a 28:1 divider between ~0.57 and ~2.66 V at the pin and tapers to zero outside that range. It has no gain or offset of its own, so set it together with `hv_v_gain`/`hv_v_offset` (and `lv_v_` if that side is on ADC1). It applies to every ADC1 channel, including NTC and current |

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


## INA226

The following keys set the INA226 measurement (`ina226` backend); its wiring (`ina22x_alert`,
`ina22x_addr`, `i2c_*`) is in [board.conf](board.md):

| key | unit | type | default | description |
|-----|------|------|---------|-------------|
| `ina22x_resistor` | Ω | float | — | INA226 current-sense shunt resistance, required with `ina226` (0 to 10 Ω, else sensor setup fails); the current range is the shunt full scale, 81.92 mV / `ina22x_resistor`, at the shunt ADC resolution (2.5 µV / `ina22x_resistor`) |
| `ina22x_conv_time_us` | µs | int | 1100 | INA226 per-conversion time (rounded up to nearest device step); lower = faster/noisier |
| `ina22x_ov` | V | float | 0 | INA226 bus over-voltage limit, in volts of the sensor on the INA226 bus channel (after its divider; the LV side, which is the battery only in buck topology). When set, ALERT is a latched over-voltage alert: its ISR forces the gates off (MCPWM gate driver only) and the converter backs off; conversions are then polled instead of signalled. Set it above the highest normal voltage. 0 = ALERT is the conversion-ready strobe |

## Notes & examples

### Sensors

Configure the voltage and current sensors in `sensor.conf` to match your topology and chips.

There are four sensors: Vin, Vout, Iin, and Iout. Configure them by side (HV/LV); the topology
assigns the roles. A converter can have one or two current sensors.
With a single current sensor, the firmware computes the other from the voltage ratio and a fixed
97 % conversion efficiency. The tables above list all keys.

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

## Moving the INA226 keys

Earlier firmware read `ina22x_resistor`, `ina22x_conv_time_us` and `ina22x_ov` from
[board.conf](board.md). The current firmware reads them only from `sensor.conf`. A copy left in
`board.conf` with the same value in `sensor.conf` only logs a warning. Any other copy in
`board.conf` stops sensor setup:

```
board.conf ina22x_resistor is no longer read, move it to sensor.conf
```

`etc/migrate_side_keys.py` moves them as well. In a config directory it cuts the lines, with
their comments, from `board.conf` and appends them to `sensor.conf`; a key that is in both files
with different values is refused. For a live board, add `get-config board.conf` to the capture.
The output has two steps:

1. Before the update: `set-config sensor.conf ...` for each key that `sensor.conf` lacks or holds
   with a different value. Earlier firmware reads these keys only from `board.conf` and ignores
   the `sensor.conf` copy.
2. After the update, once `bootinfo` shows the new image valid: `del-config board.conf ...`.

Between the two steps both files hold the same value, so either firmware boots. Deleting the
`board.conf` copy earlier would leave earlier firmware without its shunt value at the next reboot.
A key whose `sensor.conf` value differs from `board.conf` is printed as a warning; step 1 sets it
to the `board.conf` value, the one earlier firmware uses, so check which value is right first.
