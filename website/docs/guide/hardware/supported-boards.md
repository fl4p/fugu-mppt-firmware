---
title: Supported Boards
sidebar_position: 1
---

*this document is an LLM generated placeholder*

# Supported Boards

The firmware has no compiled-in board definition. A board is supported when a folder under
[`config/`](https://github.com/fl4p/fugu-mppt-firmware/tree/main/config) describes its pins, sensors and limits.

## Quick start

```bash
export ESPPORT=/dev/cu.usbmodemXXXX
./provision.py fmetal          # Fugu2
```

See [Provisioning](../getting-started/provisioning.md) for the details.

## Board configurations

| Board | Config folder | MCU | Voltage sensing | Current sensing | Topology |
|---|---|---|---|---|---|
| [Fugu2](https://github.com/fl4p/Fugu2) | `config/fmetal` | ESP32-S3 | Vin: internal ADC1 (200k / 7.5k divider), Vout: INA226 | INA226, 0.5 mΩ shunt, output side | Buck |
| Original Fugu, ADS1015 | `config/fugu1/fugu1_esp32` | ESP32 | ADS1015 | ACS712-30 hall sensor, input side (Iout virtual) | Buck |
| Original Fugu, ADS1015, ESP32-S3 module | `config/fugu1/fugu1_esp32s3` | ESP32-S3 | ADS1015 | ACS712-30, input side | Buck |
| Original Fugu, internal ADC | `config/fugu1/fugu_int_adc` | ESP32 | internal ADC1 | ACS712-30 on internal ADC1, input side | Buck |
| Solar boost (Fugu2 pinout) | `config/solar-boost` | ESP32-S3 | Vin: INA226, Vout: internal ADC1 (200k / 7.5k) | INA226, 1 mΩ shunt, input side | Boost |
| 12 V power supply (Fugu2 pinout) | `config/psu_12v` | ESP32-S3 | Vin: internal ADC1, Vout: INA226 | INA226, 1 mΩ shunt, output side | Buck, forced PWM |
| Boost PSU example | `config/psu/boost80V` | ESP32-S3 | Vin: INA226, Vout: internal ADC1 | INA226, 0.5 mΩ shunt | Boost, `mode=psu` at 24 V |

A board with a single current sensor gets the other current from a virtual sensor, computed from the voltage ratio
and `sensor.conf::power_conversion_eff`.

Configurations for development without a power stage (`config/lab/dry_mock`, `dry_int`, `vconv_mock`,
`wokwi_mock`) are listed in [Provisioning](../getting-started/provisioning.md#board-configurations).

## Board notes

### Fugu2 (`fmetal`)

The reference hardware: dual parallel high-side switches, snubber, INA226 current sensor, `HiLi` gate driver logic
(`pwm_hi`, `pwm_li`, `pwm_sd`), WS2812 status LED and an input backflow switch on `panel_sd`.

`converter.conf` sets `pwm_driver=mcpwm`. The key is only consulted when the firmware is built with both
`CONFIG_FUGU_WITH_LEDC` and `CONFIG_FUGU_WITH_MCPWM`; with one driver compiled in, that one is used. See
[Build Options](../getting-started/build-options.md).

### Original Fugu (`fugu1`)

The [original design](https://www.instructables.com/DIY-1kW-MPPT-Solar-Charge-Controller/) uses `InEn` gate driver
logic (`pwm_in`, `pwm_en`), an ADS1015 and an ACS712-30 hall sensor on the input. Its noise performance is poor;
Fugu2 is recommended for new builds.

- `fugu1_esp32` and `fugu1_esp32s3` differ only in pin numbers (the S3 module remaps I²C, gate driver, LED, fan and
  NTC channel).
- `fugu_int_adc` replaces the ADS1015 with the ESP32's internal ADC. Wiring is in
  [Internal ADC](internal-adc.md). The folder only contains `board.conf`, `sensor.conf` and `limits.conf`; copy
  `charger.conf`, `coil.conf` and `converter.conf` from `fugu1_esp32` before provisioning.

:::note
The `fugu1` sensor files set `conversion_eff`, but the firmware reads `power_conversion_eff`. The built-in default of
0.95 applies until you rename the key.
:::

### Solar boost

Same pinout as Fugu2, with `converter.conf::topo=boost`. The INA226 moves to the low-voltage input side, the
internal ADC divider to the high-voltage output side. Limits are swapped accordingly (`vin_max=60`, `vout_max=85`).

### 12 V power supply

Same pinout as Fugu2, running as a regulated supply rather than a charger:

- `forced_pwm=1` for tight output regulation, `fpwm_gate=0`.
- Output voltage is `charger.conf::vout_max` (12 V); `limits.conf::vout_max=16` is the hard cutout.

:::danger
With `fpwm_gate=0`, never ramp the duty to 0 in forced PWM: a complementary low side at duty ~0 shorts the output
through the coil. Only use this configuration if the output is never pre-charged or stiff.
See [`converter.conf`](../../reference/config/converter.md#fpwm_gate-fpwm_gate_margin).
:::

## Adding your own board

1. Copy the closest folder, e.g. `cp -r config/fmetal config/myboard`.
2. Set `board.conf::mcu` to the chip you build for (`esp32s3` or `esp32`); a mismatch stops the converter at boot.
3. Adjust pins in [`board.conf`](../../reference/config/board.md), dividers and channels in
   [`sensor.conf`](../../reference/config/sensor.md), and hard limits in
   [`limits.conf`](../../reference/config/limits.md).
4. Bring it up with the [first power-up checklist](../getting-started/first-power-up.md).

:::tip
Folders under `config/lab/` may contain `mqtt.conf` or `wifi.conf` from the maintainer's lab. Remove or replace
those files in your copy.
:::
