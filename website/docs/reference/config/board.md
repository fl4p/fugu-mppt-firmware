---
title: board.conf
sidebar_position: 1
---

# board.conf

`board.conf` describes the board's pins, buses, and ADC and driver wiring. The firmware reads it once at boot. The table lists its keys.

The INA226 measurement settings (`ina226_resistor`, `ina226_conv_time_us`, `ina226_ov`) are in
[sensor.conf](sensor.md#ina226); see [Moving the INA226 keys](sensor.md#moving-the-ina226-keys).
The backflow switch pins were renamed; see [Backflow switch keys](#backflow-switch-keys).

| key                     | unit | type   | default | description                                    |
|-------------------------|------|--------|---------|------------------------------------------------|
| `mcu`                   |      | string | —       | MCU type, `esp32s3` or `esp32`. It must match the build target, else setup fails and the converter stays off |
| `skip_assert`           |      | bool   | 0       | Skip GPIO pull-resistor sanity checks at init  |
| `i2c_sda`               | GPIO | int    | 255     | I2C SDA pin; 255 = no I2C                      |
| `i2c_scl`               | GPIO | int    | —       | I2C SCL pin (required when `i2c_sda` is set)   |
| `i2c_freq`              | Hz   | int    | 100000  | I2C bus clock frequency                        |
| `i2c_hs_freq`           | Hz   | int    | 0       | INA226 sample-read clock (esp32s3); 0 = read through Wire at `i2c_freq`. Reads drive the I2C controller directly; above 400 kHz each read enters I2C high-speed mode (controller code at ≤380 kHz). Self-tested at boot, falls back to Wire |
| `i2c_port`              |      | int    | 0       | INA226 bus index. Only 0 is implemented; any other value fails INA226 init |
| `ina226_alert`          | GPIO | int    | —       | INA226 ALERT pin, required with the `ina226` backend (unset fails INA226 init) |
| `ina226_addr`           |      | int    | 0x40    | INA226 I2C address                             |
| `pwm_freq`              | Hz   | int    | —       | Converter PWM switching frequency at boot, 5–500 kHz (exclusive); 0.95 · `pwm_freq` · `coil.conf::L0` must lie between 1 and 20 Ω. The `pwm-freq` console verb changes it for the session without persisting |
| `pwm_driver_logic`      |      | enum   | —       | Gate driver logic: `HiLi` or `InEn`; any other value fails converter setup |
| `pwm_hi`                | GPIO | int    | —       | High-side gate driver pin (HiLi mode, required) |
| `pwm_li`                | GPIO | int    | —       | Low-side gate driver pin (HiLi mode, required)  |
| `pwm_sd`                | GPIO | int    | 255     | Gate driver shutdown (DIS) pin (HiLi mode); 255 = none |
| `pwm_in`                | GPIO | int    | —       | Gate driver IN pin (InEn mode, required)       |
| `pwm_en`                | GPIO | int    | —       | Gate driver EN pin (InEn mode, required)       |
| `boot_refresh_ns`       | ns   | float  | 2000    | Min *realized* LS on-time to refresh HS bootstrap cap; `pwm_deadtime_hl_ns` is reserved on top. Also sets the buck's highest Vout/Vin ratio in DCM ([LS minimum pulse](../../internals/diode-emulation.md#ls-minimum-pulse-near-unity-ratio)) |
| `pwm_deadtime_ns`       | ns   | float  | 0       | HiLi hardware dead-time (MCPWM), common value for both transitions; 0 = none, and 0 also means the console `dt` command cannot arm it later. Valid 0–10⁶ ns, else 0 with an error; each edge is capped at 1/32 of the period. Ignored with `InEn` |
| `pwm_deadtime_hl_ns`    | ns   | float  | `pwm_deadtime_ns` | HS->LS override (ctrl-off -> rect-on). Hardware RED delay on the rect rising edge; the *realized* gap is one tick less (the ctrl generator spends one tick claiming its dead-time path). 0 leaves the submodule bypassed for the whole boot. An out-of-range value (outside 0–10⁶ ns) falls back to `pwm_deadtime_ns` |
| `pwm_deadtime_lh_ns`    | ns   | float  | `pwm_deadtime_ns` | LS->HS override (rect-off -> ctrl-on at the period wrap). Reserved in software as `pwmMax = periodTicks - ticks`; the realized band is one tick *wider* (callers cap `cmpLS` at `pwmMax-1`) and it eats commandable duty span. An out-of-range value falls back to `pwm_deadtime_ns` |
| `pwm_fault_pin`         | GPIO | int    | 255     | GPIO for HW OST brake (MCPWM); 255 = disabled  |
| `pwm_fault_active_high` |      | bool   | 0       | 1 if fault asserts high, 0 if low              |
| `pwm_sync_pin`          | GPIO | int    | 255     | Wired-sync pulse pin (`CONFIG_FUGU_WITH_WSYNC`); output on the leader, input on a follower. Required when `converter.conf::sync_role` ≠ `none`. See [Wired clock sync](../../development/sync/wired-sync.md) |
| `bflow_en`              | GPIO | int    | 0       | Backflow switch enable pin, driven high to turn the switch on; 0 = no switch. Set either `bflow_en` or `bflow_sd`, not both |
| `bflow_sd`              | GPIO | int    | 0       | Backflow switch shutdown pin, driven high to turn the switch off; 0 = no switch |
| `bflow_bat`             |      | bool   | 0       | Declares that the backflow switch (`bflow_en` or `bflow_sd`) sits between the converter output and the battery, and that the output current sensor is on the battery side. Enables the idle [high-side short check](../../internals/protection.md#high-side-short-check), `selftest bflow` and its daily run ([backflow switch test](../../internals/protection.md#backflow-switch-test)). Ignored on a boost, without a backflow switch, or with a virtual output current sensor |
| `hs_clamp`              |      | bool   | 0       | On a latched high-side short, clamp the low-side FET on once Vin has stayed below 9 V for 60 s, through a 20 ms pulse-width ramp; until then all gates are off ([low-side clamp](../../internals/protection.md#low-side-clamp)). Needs `bflow_bat=1`, an output current sensor that reads reverse current, and a passed blocks phase of the backflow switch test within the last 36 h (not the toggle variant). Validate on a bench first: grounding the switch node can turn the backflow switch on, which the ramp is meant to avoid |
| `lv_pgood`              | GPIO | int    | 255     | LV power-good output; high disables the HV aux supply path (Fugu2: GPIO35, not usable on modules with octal PSRAM). LV is Vout on a buck, Vin on a boost. Released at once on a converter shutdown, ADC stall, stale or non-finite LV reading; 255 = none |
| `lv_pgood_v`            | V    | float  | 10.5    | LV release point: `lv_pgood` releases below it and asserts after LV held 0.5 V above it for 5 s. Values below 5 V or non-finite fall back to 10.5 |
| `led_WS2812`            | GPIO | int    | 255     | WS2812 status LED data pin; 255 = off          |
| `led_simple`            | GPIO | int    | 255     | Plain on/off status LED pin; 255 = off         |
| `fan_pwm`               | GPIO | int    | 255     | Cooling fan PWM pin; 255 = off                 |
| `ads_alert`             | GPIO | int    | —       | ADS1x15 ALERT/RDY pin                          |
| `adc_freq`              | Hz   | int    | 3000    | Tick rate of both mock ADC backends, `fake` and `vconv` (every channel is sampled each tick); `vconv.conf::adc_freq` overrides it in a vconv build |
| `idle_cpu_mhz`          | MHz  | int    | 80      | CPU clock of the [idle power profile](#idle-power-profile): 80, 160 or 240, any other value fails setup. 0, or the boot clock, keeps the boot clock |

## Idle power profile

The idle power profile starts after the converter and every maintenance task (PSU or manual duty,
sweep, rect-cal, coil measurement, OTA, bsync, USB drive) have been off for 5 min. It ends on the
first network-loop pass that sees one of them active. Besides the CPU clock it sets the BLE
advertising interval ([`ble.conf::idle_adv_ms`](ble.md)) and the telemetry interval
([`tele.conf::idle_interval_ms`](tele.md)).

- While the CPU runs at `idle_cpu_mhz` the converter does not start. A start request ends the
  profile, and the converter starts once the control loop has run at the boot clock for at least
  200 ms and 128 samples.
- On each entry the control-loop sample rate is measured at the boot clock and again a few seconds
  after the switch. Below 97 % of the boot-clock rate, the boot clock is restored and kept until
  the next reboot; the rest of the profile stays active.
- A value that needs a different PLL than the boot clock (240 MHz on the ESP32) is ignored with a
  warning.

## Backflow switch keys

Earlier firmware read the backflow switch pins as `panel_en` and `panel_sd`. The switch can sit on
the battery side, so the keys now share the `bflow_` prefix with `bflow_bat`:

| Old key    | New key    |
|------------|------------|
| `panel_en` | `bflow_en` |
| `panel_sd` | `bflow_sd` |

A `board.conf` that has an old key without its new one stops setup, and the converter stays off:

```
board.conf: panel_sd was renamed to bflow_sd
```

With both keys set, the firmware uses the new one and logs a warning that the old one is ignored.

`etc/migrate_side_keys.py dir <dir>` renames the keys in a config directory, keeping comments. For a
live board, capture `get-config board.conf` and run `etc/migrate_side_keys.py live board.log`. It
prints two steps:

1. `set-config board.conf bflow_sd ...` before the update. Earlier firmware does not read it.
2. `del-config board.conf panel_sd` after the update.

Between the two steps both keys are set, so either firmware finds its pin across the reboot.

## INA226 key prefix

The INA226 keys were spelled `ina22x_*`. They now use the part number, matching the `ina226` ADC
backend and the `ADC_INA226` driver:

| Old key               | New key                | File          |
|-----------------------|------------------------|---------------|
| `ina22x_alert`        | `ina226_alert`         | `board.conf`  |
| `ina22x_addr`         | `ina226_addr`          | `board.conf`  |
| `ina22x_resistor`     | `ina226_resistor`      | `sensor.conf` |
| `ina22x_conv_time_us` | `ina226_conv_time_us`  | `sensor.conf` |
| `ina22x_ov`           | `ina226_ov`            | `sensor.conf` |

An old key without its new one in either file stops setup, and the converter stays off:

```
ina22x_* was renamed to ina226_*: ina22x_alert is no longer read, use ina226_alert
```

With both set, the firmware uses the new one and logs a warning that the old one is ignored.

`etc/migrate_side_keys.py dir <dir>` renames the keys in a config directory, keeping comments. For a
live board, capture `get-config board.conf` and `get-config sensor.conf`, then run
`etc/migrate_side_keys.py live board.log`. It prints the same two steps as the backflow rename above,
and moves a `board.conf` measurement key to `sensor.conf` under its new name.
