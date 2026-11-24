---
title: sensor.conf
sidebar_position: 2
---

# sensor.conf

`sensor.conf` sets the channel map, the divider ratios, and the calibration of the voltage and current sensors.

The following keys apply to all channels:

| key                              | unit | type   | default | description                                                    |
|----------------------------------|------|--------|---------|----------------------------------------------------------------|
| `adc`                            |      | string | —       | Default ADC backend for all channels; required unless every present channel sets `<channel>_adc` |
| `ignore_calibration_constraints` |      | bool   | 0       | Skip the ADC range and noise checks of the boot calibration    |
| `notch_adaptive`                 |      | bool   | 1       | Auto-tune the inverter-ripple notch to the tone measured on Vout (80–140 Hz); 0 = fixed at `notch_freq` |
| `notch_freq`                     | Hz   | float  | 100     | Notch frequency when not adaptive (2×mains: 100 = 50 Hz, 120 = 60 Hz) |
| `notch_q`                        |      | float  | 20      | Notch quality factor (bandwidth ≈ `notch_freq`/`notch_q`)      |
| `notch_depth`                    | dB   | float  | -15     | Notch attenuation at its centre, ≤ 0; a positive value logs a warning and uses -15. See [choosing the filter](../../internals/signal-filters.md#choosing-the-filter) |
| `ripple_filter`                  |      | string | notch   | Inverter-ripple filter: `notch`, `comb` or `both`; an unknown value logs a warning and uses `notch`. See [ripple comb](../../internals/signal-filters.md#ripple-comb) |
| `despike`                        |      | float  | 0       | Outlier threshold of the decision-based median, in running mean-deviation units: 0 = plain median, about 8 enables it (lower clips more). See [despike](../../internals/signal-filters.md#despike) |
| `esp32adc1_sr`                   | Hz   | int    | —       | Internal ADC1 continuous-mode raw sample rate (required with `esp32adc1`) |
| `esp32adc1_avg`                  |      | int    | —       | Software average of N raw conversions per delivered sample, 1–1023 (required with `esp32adc1`) |
| `esp32adc1_inl`                  |      | bool   | 0       | Apply the built-in ESP32-S3 ADC1 INL correction (12 dB attenuation only, about −8 to +5 mV at the pin between 0.57 and 2.66 V, roughly ±0.2 V at the input of a 28:1 divider, tapering to zero outside) to every ADC1 channel, including NTC and current. It has no gain or offset of its own, so calibrate `_gain` and `_offset` with it on. See [Internal ADC](../../guide/hardware/internal-adc.md) |

The firmware reads `sensor.conf` once at boot. In `CONFIG_FUGU_WITH_VCONV` builds a
`sensor.vconv.conf`, if present, is read instead (see [vconv.conf](vconv.md)).

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

The NTC channel is read only from `sensor.conf`: a `board.conf::ntc_ch` left over from earlier
firmware fails setup when `sensor.conf` has no NTC channel.

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
| `_rh`       | Ω    | float  | —           | Voltage divider upper (high-side) resistor (voltage channels, required when `_ch` is set) |
| `_rl`       | Ω    | float  | —           | Voltage divider lower resistor (voltage channels, required when `_ch` is set); the ADC input impedance is taken in parallel |
| `_gain`     |      | float  | 1           | Per-board gain on top of the divider (voltage channels): V = gain · V_div + offset. Must be between 0.5 and 2 (exclusive), else sensor setup fails |
| `_offset`   | V    | float  | 0           | Per-board offset (voltage channels), see `_gain`. Must be within ±10 V (exclusive), else sensor setup fails |
| `_factor`   |      | float  | 1           | Linear scale factor raw ADC → physical, sign sets direction (current channels) |
| `_midpoint` |      | float  | 0           | Zero-point offset subtracted before scaling (current channels)                 |
| `_filt_len` |      | int    | 10          | Filter window length (samples). Ignored for `ntc`, which uses a fixed 50       |

For worked ACS712 and bare-ESP32 configs, see [Notes & examples](#notes--examples).


## INA226

The following keys set the INA226 measurement (`ina226` backend); its wiring (`ina226_alert`,
`ina226_addr`, `i2c_*`) is in [board.conf](board.md):

| key | unit | type | default | description |
|-----|------|------|---------|-------------|
| `ina226_resistor` | Ω | float | — | INA226 current-sense shunt resistance, required with `ina226` (0 to 10 Ω, else sensor setup fails); the current range is the shunt full scale, 81.92 mV / `ina226_resistor`, at the shunt ADC resolution (2.5 µV / `ina226_resistor`) |
| `ina226_conv_time_us` | µs | int | 1100 | INA226 per-conversion time, rounded up to the next device step (140, 204, 332, 588, 1100, 2116, 4156, 8244; above that 8244); a lower value is faster and noisier |
| `ina226_ov` | V | float | 0 | INA226 bus over-voltage limit, in volts of the voltage channel on INA226 channel 0 (after its divider, `_gain` and `_offset`). When set, ALERT is a latched over-voltage alert: its ISR forces the gates off (MCPWM gate driver only) and the converter backs off; conversions are then polled. Set it above the highest normal voltage. Without a voltage channel on INA226 channel 0 sensor setup fails. 0 = ALERT is the conversion-ready strobe |

## Notes & examples

### Sensors

There are four sensors: Vin, Vout, Iin, and Iout. Configure them by side (HV or LV); the topology
assigns the roles. A converter can have one or two current sensors; with none, setup fails.
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
* `fake`: sinusoidal mock for bench boards without a power stage (sample rate `board.conf::adc_freq`)
* `vconv`: the simulated plant of `CONFIG_FUGU_WITH_VCONV` builds (see [vconv.conf](vconv.md))

`ina226`, `ads1015` and `ads1115` exist only when compiled in (`CONFIG_FUGU_WITH_INA226`,
`CONFIG_FUGU_WITH_ADS`, both on by default); an unknown or disabled backend fails sensor setup.

An `ina228` backend is planned.

### Voltage sensors `hv_v`, `lv_v`

The firmware computes the gain of a voltage channel from the resistor values of the ADC input voltage
divider and the hardcoded ADC input impedance. Specify both resistors, as in this example:

```
lv_v_rh = 47e3    # upper resistor of voltage divider
lv_v_rl = 47e3    # lower resistor
```

### ACS712

The ACS712 sensitivity is 66 mV/A. A 10k + 3.3k voltage divider scales the output to match the ADC
voltage range. `hv_i_factor` encodes both the sensitivity and the divider.

Specify the ACS712 midpoint voltage with `hv_i_midpoint` (or `lv_i_midpoint`). This ACS712 has a
2.5 V midpoint, scaled through the same 10k + 3.3k divider: `2.5V * 10k/(10k+3.3k)`. The following
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

- Config directory (profile or backup): the script rewrites `sensor.conf` and `limits.conf` in place,
  keeping comments and order. Already migrated files stay unchanged; a role key next to its own
  side key is refused. `--check` only reports.

  ```
  python3 etc/migrate_side_keys.py dir config/my_board
  ```

- Live board: migrate its configs *before* the OTA to the new firmware, otherwise sensor setup
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

Earlier firmware read `ina226_resistor`, `ina226_conv_time_us` and `ina226_ov` from
[board.conf](board.md). The current firmware reads them only from `sensor.conf`. A copy left in
`board.conf` with the same value in `sensor.conf` only logs a warning. Any other copy in
`board.conf` stops sensor setup:

```
board.conf ina226_resistor is no longer read, move it to sensor.conf
```

The keys were also renamed from the `ina22x_*` spelling, see
[INA226 key prefix](board.md#ina226-key-prefix).

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
