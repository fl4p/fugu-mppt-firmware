---
title: Lab config profiles
sidebar_position: 2
---

# Lab config profiles

`config/lab/` holds littlefs configuration images for bench and simulation setups. Each folder has a `conf/`
directory with the [configuration files](../reference/config/index.md) that `./provision.py` writes to the device.

:::warning Examples, not templates
These profiles describe the maintainer's bench hardware. Check pins, divider ratios, shunt values, and limits
against your board before use.

The network settings (`wifi.conf`, `mqtt.conf`, `tele.conf`) are lab-specific. After provisioning, set your own SSID,
broker, and telemetry host with `set-config`, and do not reuse the values in the repository. `wifi.conf` is gitignored
and absent on a fresh clone.
:::

## Profiles

The following table lists each profile with its purpose, the hardware it assumes, and its notable keys.

| Folder | Purpose | Hardware assumptions | Notable keys |
|---|---|---|---|
| `dry_mock` | Firmware without a power stage | Any ESP32-S3 board | `sensor.conf::adc=fake` (sinusoidal mock), `board.conf::adc_fake_freq`, `skip_assert=1`, BLE console `ble_security=justworks` |
| `dry_int` | Internal ADC, gate driver not connected | ESP32-S3 (Waveshare ESP32-S3-Zero LED pin), integrated half-bridge driver | `adc=esp32adc1`, `pwm_driver_logic=InEn`, `skip_assert=1`, `pprof.conf::sprofiler_hz` |
| `vconv_mock` | Simulated converter on ESP32-S3 | Any ESP32-S3 board, `CONFIG_FUGU_WITH_VCONV=y` build | `sensor.conf::adc=vconv` (identity transform, the plant returns physical units), `converter.conf::mode=psu`, `psu_vout=28`, `vconv.conf` plant parameters incl. inverter-ripple shapes (`vbat_ac_*`) |
| `vconv_mock_esp32` | Simulated converter on classic ESP32 | Classic ESP32 board, VCONV build | Same plant as `vconv_mock`, classic-ESP32 pins, MPPT mode (no `mode=psu`) |
| `wokwi_mock` | Wokwi simulator | Simulated ESP32-S3 | `adc=fake`, all network services enabled |
| `wokwi_mock_esp32` | Wokwi simulator | Simulated classic ESP32 | As `wokwi_mock`, classic-ESP32 pins |
| `f2_test` | Fugu2-style buck on a bench supply | Fugu2 pinout, INA226 on the output, internal ADC for Vin/NTC | `forced_pwm=1`, `fpwm_gate=0`, `limits.conf::vin_min=72`, `tracker.conf::target_duty_cycle=0.425` |
| `buck_bench` | Bench buck board, **battery / battery-emulator output** | Fugu2 pinout, INA226 (1.5 mΩ shunt) for Vout/Iout, internal ADC for Vin, 80 µH coil | `pwm_deadtime_ns=200`, `sync_role=follower`, `charger.conf::vout_max=29`, `target_duty_cycle=0.37` |
| `buck_bench_open_output` | Same bench buck board, **open output** (switch-node sweeps) | As `buck_bench`, nothing connected to the output | Identical to `buck_bench` except `charger.conf::vout_max=60` |
| `buck_bench_no_ina226` | Bench buck board with the INA226 absent | Internal ADC for Vin/NTC, fake ADC for Vout/Iout | Per-channel `lv_v_adc=fake`, `lv_i_adc=fake` |
| `boost_bench` | Bench boost board (input of a power loop) | Fugu2 pinout, INA226 on the low-voltage input, internal ADC for Vout | `topo=boost`, `forced_pwm=1`, `sync_role=leader`, `sync_phase_deg=180`, `charger.conf::vout_max=75`, `notch_freq=0`, `target_duty_cycle=0.65` |
| `fmetal_boost` | The Fugu2 board (`config/fmetal` wiring) run as a boost on the bench | Fugu2, same HV/LV sensors as `fmetal`; whether `panel_sd` is on the LV side and shorted is unverified | `topo=boost`, `forced_pwm=1`, `vin_min=8`, `charger.conf::vout_max=80`, `target_duty_cycle=0.1` |
| `boost_pv` | Bench boost board as a solar-array simulator | As `boost_bench` | `converter.conf::mode=pv`, `pv_isc`, `pv_voc`, `pv_k`; no `target_duty_cycle` (a fixed duty would override `mode=pv`) |

The bench profiles (`buck_bench*`, `boost_*`, `fmetal_boost`, `f2_test`) set `limits.conf::reverse_current_paranoia=0`, which changes
several protection thresholds. See [limits.conf](../reference/config/limits.md). Several also set
`tracker.conf::target_duty_cycle`, which boots into a hard-fixed duty (manual PWM, no MPPT).

## Battery vs open output

The open-output case is a separate, complete profile rather than an override of the battery profile.

With the output open, Vout floats toward Vin, so the 29 V battery reference in `charger.conf::vout_max` would stop a
switch-node sweep above ~30 V. Raising it in the battery profile would also raise the protection threshold the next
time a battery is attached. The two profiles differ in `vout_max` and in what they expect on the output:

| Profile | `charger.conf::vout_max` | Use with |
|---|:---:|---|
| `config/lab/buck_bench` | 29 V | Battery or battery emulator on the output |
| `config/lab/buck_bench_open_output` | 60 V | Nothing on the output |

`test/host_py/test_bench_config_profiles.py` guards this split. The battery profile must keep 29 V, and the open-output
profile must contain the same files with identical content except `charger.conf`. Edit both profiles together.

## Common scenarios

### Provision a profile

Before provisioning, copy the profile, fix what it lacks, and remove the lab's network settings. For example, with
`dry_mock`:

```bash
cp -r config/lab/dry_mock /tmp/mock
cp config/fmetal/conf/charger.conf /tmp/mock/conf/   # dry_mock has no charger.conf
rm -f /tmp/mock/conf/mqtt.conf                        # lab broker settings, not yours
./provision.py /tmp/mock
```

Then set your own network settings from the console. Use `wifi-add <ssid>:<password>` for Wi-Fi, and
`set-config mqtt.conf <key> <value>` / `set-config tele.conf <key> <value>` for the broker and telemetry host. See
[Provisioning](../guide/getting-started/provisioning.md).

### Derive a profile for your board

Copy the closest profile to a new folder. Before you enable the power stage, check these against your hardware:
`board.conf` pins, `sensor.conf` divider/shunt values, `coil.conf::L0` ([measure it](coil-inductance.md)), and
`limits.conf`.
