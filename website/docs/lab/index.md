---
title: Lab overview
sidebar_position: 1
---

# Lab overview

Most firmware work needs no power stage, so pick the smallest setup that exercises what you are changing.

## Setups by requirement

The following table lists each setup with the hardware, build, and config profile it needs, and the work it suits.

| Setup | Hardware | Build | Config profile | Good for |
|---|---|---|---|---|
| No power stage | Any ESP32-S3 dev board | Default | `config/lab/dry_mock` | Console, services, networking, telemetry, config handling |
| Simulated converter | Any ESP32-S3 (or classic ESP32) board | `CONFIG_FUGU_WITH_VCONV=y` | `config/lab/vconv_mock` (`vconv_mock_esp32`) | Closed-loop control, MPPT, charger, PSU/PV-sim modes |
| Simulator, no board | None | Default | `config/lab/wokwi_mock` (`wokwi_mock_esp32`) | Boot, console and CI smoke tests in [Wokwi](../development/wokwi.md) |
| Single converter | A Fugu board, current-limited PSU on the input, e-load or battery emulator on the output | Default (or MCPWM) | Your board's profile, e.g. `config/fmetal` | Diode emulation, protection, efficiency, [automated bench tests](automated-bench-tests.md) |
| Two-converter loop | A boost and a buck board in a recirculating loop, one PSU | MCPWM | See [Power-loop rig](power-loop.md) | Waveforms, efficiency and sync work at power without a solar array |

[Lab config profiles](config-profiles.md) describes what each profile under `config/lab/` assumes.
[Testing](../development/testing.md) covers the unit and e2e suites that run on these setups.

## Quick start: no power stage

`dry_mock` selects the fake ADC (`sensor.conf::adc=fake`), which produces sinusoidal readings, so the control loop
runs without any analog front end. See [Mock ADC](../development/testing.md#mock-adc) for provisioning and the checks.

## Quick start: simulated converter

Build with `CONFIG_FUGU_WITH_VCONV=y` (see
[Build options](../guide/getting-started/build-options.md#control-loop-work-without-hardware) for the ESP32-S3
specifics), then provision `config/lab/vconv_mock`. The gate driver and ADC are replaced by a software plant
(`src/sim/vconv.*`) configured by `vconv.conf`: a panel curve (`isc`, `voc`, `pv_k`), a battery (`v_bat`, `r_bat`),
input/output capacitance, and per-channel noise.

## Half-bridge safety

:::danger Driving a real half-bridge
- Current-limit the supply. Bring up a new board, profile or firmware from a bench supply with a low current
  limit, not from a panel or battery.
- In the power-loop rig, the PSU limit caps the power put into the loop. It isn't a loop-current limit.
  Circulating current is set by the ratio mismatch and can reach tens of amps within that power
  budget. The limit does nothing against returned energy or discharge of the bus capacitance. The firmware's
  current-based trips are blind on that rig, but its voltage trips still act. Provide a separate way to break the
  loop (switch or fuse sized for the loop current), and discharge the bus before rewiring. See
  [Power-loop rig](power-loop.md#pitfalls).
- Mock profiles are logic-only. Use them with the power stage disconnected to check console, services, and
  synthetic telemetry. Never use them with a panel, supply, or battery attached. `dry_mock` uses the fake ADC, drives
  PWM on pins that aren't your board's, and sets zero dead-time. Before first power, provision the real board profile
  and compare `sensor` readings against a meter at a current-limited supply (see
  [First power-up](../guide/getting-started/first-power-up.md)).
- Never OTA a `VCONV` build to a real converter. The half-bridge is never switched, so the converter stops
  charging while looking healthy (real sensor readings, sweeps, repeated `Vr-sensor-fail`) and the loads drain the
  battery. See [Build options](../guide/getting-started/build-options.md).
- Stop conversion before updating at high power. An OTA reboots the device into the new image, so send `dc 0`
  first. With `etc/ota.py`, always dry-run (`-n`) and scope the target (`-m REGEX`) first.
- Rollback needs the serial-flashed bootloader. A board that has only ever been updated over the air keeps its
  old bootloader and will not revert from a bad image. See [OTA over Wi-Fi](../guide/updating/ota-wifi.md).
- Match the profile to the output. An open-output profile raises the output over-voltage threshold. Never keep
  it on a board with a battery attached (see [Lab config profiles](config-profiles.md#battery-vs-open-output)).
- Watch for fixed-duty profiles. `tracker.conf::target_duty_cycle > 0` boots straight into a hard-fixed duty with no
  PD control or MPPT tracking.
- If the battery or load is removed during conversion, expect an over-voltage transient.
:::
