---
title: Introduction
sidebar_position: 1
---

# Fugu MPPT Firmware

Firmware for ESP32 and ESP32-S3 based MPPT solar charge controllers and DC/DC converters. It started as a
re-write of [AngeloCasi/FUGU-ARDUINO-MPPT-FIRMWARE](https://github.com/AngeloCasi/FUGU-ARDUINO-MPPT-FIRMWARE)
and targets the [Fugu2](https://github.com/fl4p/Fugu2) hardware, while staying compatible with the
[original Fugu design](https://www.instructables.com/DIY-1kW-MPPT-Solar-Charge-Controller/).

Topology, pins, sensors, limits and charger parameters are **not compiled in**. They live in
[configuration files](../reference/config/index.md) on the device's flash file system, so one firmware image serves
many boards and can be updated over the air.

## Highlights

| Area | What you get |
|---|---|
| Charging | Proper Li-ion / LiFePO4 [charge termination](charging/termination.md), [BMS coupling](charging/bms-integration.md) over MQTT, for BMSes on BLE, CAN bus or RS485 through a bridge |
| Control | MPPT with periodic global sweep, five PD limiters (Vin, Iin, Vout, Iout, thermal), buck and boost operation, PSU mode |
| Power stage | Synchronous buck with sensor-less [diode emulation](../internals/diode-emulation.md), LEDC or [MCPWM](../internals/pwm-drivers.md) gate driver |
| Sensing | ADC abstraction for the [internal ADC](hardware/internal-adc.md), ADS1x15 and INA226, async sampling (< 900 µs in-out latency), [notch/median/IIR filters](../internals/signal-filters.md) |
| Protection | Fast over-voltage / over-current shutdown, temperature derating, latency watchdog |
| Connectivity | [Console](../reference/console.md) over UART, USB, telnet, MQTT and BLE; InfluxDB telemetry; Home Assistant; FTP |
| Updates | [OTA over Wi-Fi](updating/ota-wifi.md) with rollback, [OTA over BLE](updating/ota-ble.md) |
| Debugging | Coredumps with an ELF archive, real-time counters, sampling profiler, soft oscilloscope |

## Reference hardware

- [Fugu2](https://github.com/fl4p/Fugu2) (KiCad): dual parallel high-side switches, snubber, INA226 current sensor.
  This is the "standard" board (`config/fmetal`).
- [Original Fugu](https://www.instructables.com/DIY-1kW-MPPT-Solar-Charge-Controller/) (Proteus): ADS1015 or
  internal ADC (`config/fugu1`). Its noise performance is poor; Fugu2 is recommended.

## Where to go next

- **New here?** Follow [Getting Started](getting-started/index.mdx) to build, flash and provision a board.
- **Tuning a device?** See the [configuration reference](../reference/config/index.md) and [console commands](../reference/console.md).
- **Curious how it works?** Start with the [architecture overview](../internals/architecture.md).

:::danger Use at your own risk

If the battery or load is removed during conversion, expect an over-voltage transient at the output
(measured: 36 V for 400 ms on a 28.5 V system). In a software or hardware failure the charger may put the full
panel voltage on the battery terminals. Add over-voltage protection (TVS, crowbar, second DC/DC) where connected
devices cannot tolerate that.

:::
