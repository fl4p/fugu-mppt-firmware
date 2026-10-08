---
title: board.conf
sidebar_position: 1
---

# board.conf

Pins, buses, ADC/driver wiring.

| key                     | unit | type   | default | description                                    |
|-------------------------|------|--------|---------|------------------------------------------------|
| `mcu`                   |      | string | —       | MCU type, e.g. `esp32s3` or `esp32`            |
| `skip_assert`           |      | bool   | 0       | Skip GPIO pull-resistor sanity checks at init  |
| `i2c_sda`               | GPIO | int    | 255     | I2C SDA pin; 255 = no I2C                      |
| `i2c_scl`               | GPIO | int    | —       | I2C SCL pin (required when `i2c_sda` is set)   |
| `i2c_freq`              | Hz   | int    | 100000  | I2C bus clock frequency                        |
| `i2c_port`              |      | int    | 0       | INA226 bus index. Only 0 is implemented; any other value fails INA226 init |
| `ina22x_alert`          | GPIO | int    | —       | INA226 ALERT pin                               |
| `ina22x_addr`           |      | int    | 0x40    | INA226 I2C address                             |
| `ina22x_resistor`       | Ω    | float  | —       | INA226 current-sense shunt resistance          |
| `ina22x_range`          | A    | float  | 35      | INA226 max current range for PGA config        |
| `ina22x_conv_time_us`   | µs   | int    | 1100     | INA226 per-conversion time (rounded up to nearest device step); lower = faster/noisier |
| `pwm_freq`              | Hz   | int    | —       | Converter PWM switching frequency (**boot value**; the `pwm-freq` console verb changes it live for the session without persisting) |
| `pwm_driver_logic`      |      | enum   | —       | Gate driver logic: `HiLi` or `InEn`            |
| `pwm_hi`                | GPIO | int    | —       | High-side gate driver pin (HiLi mode)          |
| `pwm_li`                | GPIO | int    | —       | Low-side gate driver pin (HiLi mode)           |
| `pwm_sd`                | GPIO | int    | 255     | Gate driver shutdown/DIS pin (HiLi mode); 255 = none |
| `pwm_in`                | GPIO | int    | —       | Gate driver IN pin (InEn mode)                 |
| `pwm_en`                | GPIO | int    | —       | Gate driver EN pin (InEn mode)                 |
| `boot_refresh_ns`       | ns   | float  | 2000    | Min *realized* LS on-time to refresh HS bootstrap cap; `pwm_deadtime_hl_ns` is reserved on top |
| `pwm_deadtime_ns`       | ns   | float  | 0       | HiLi hardware dead-time (MCPWM), common value for both transitions; 0 = none, and 0 also means the console `dt` command cannot arm it later |
| `pwm_deadtime_hl_ns`    | ns   | float  | `pwm_deadtime_ns` | HS->LS override (ctrl-off -> rect-on). Hardware RED delay on the rect rising edge; the *realized* gap is one tick less (the ctrl generator spends one tick claiming its dead-time path). 0 leaves the submodule bypassed for the whole boot |
| `pwm_deadtime_lh_ns`    | ns   | float  | `pwm_deadtime_ns` | LS->HS override (rect-off -> ctrl-on at the period wrap). Reserved in software as `pwmMax = periodTicks - ticks`; the realized band is one tick *wider* (callers cap `cmpLS` at `pwmMax-1`) and it eats commandable duty span |
| `pwm_fault_pin`         | GPIO | int    | 255     | GPIO for HW OST brake (MCPWM); 255 = disabled  |
| `pwm_fault_active_high` |      | bool   | 0       | 1 if fault asserts high, 0 if low              |
| `pwm_sync_pin`          | GPIO | int    | —       | Wired-sync pulse pin (`WITH_WSYNC`, MCPWM); leader out / follower in. Required when `sync_role` ≠ none |
| `panel_en`              | GPIO | int    | 0       | Panel/input backflow enable switch pin; 0 = no switch |
| `panel_sd`              | GPIO | int    | 0       | Panel/input backflow shutdown switch pin; 0 = no switch |
| `lv_pgood`              | GPIO | int    | 255     | LV power-good output; high disables the HV aux supply path (Fugu2: GPIO35, not usable on modules with octal PSRAM). LV is Vout on a buck, Vin on a boost. Released at once on a converter shutdown, ADC stall, stale or non-finite LV reading; 255 = none |
| `lv_pgood_v`            | V    | float  | 10      | LV voltage that must hold for 5 s before `lv_pgood` asserts; releases 0.5 V below. Values below 5 V or non-finite fall back to 10 |
| `led_WS2812`            | GPIO | int    | 255     | WS2812 status LED data pin; 255 = off          |
| `led_simple`            | GPIO | int    | 255     | Plain on/off status LED pin; 255 = off         |
| `fan_pwm`               | GPIO | int    | 255     | Cooling fan PWM pin; 255 = off                 |
| `ads_alert`             | GPIO | int    | —       | ADS1x15 ALERT/RDY pin                          |
| `adc_fake_freq`         | Hz   | int    | 3000    | Mock ADC tick rate; every channel is sampled each tick |
