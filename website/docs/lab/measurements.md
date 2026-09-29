---
title: Measurements
sidebar_position: 5
---

*this document is an LLM generated placeholder*

# Measurements

Procedures and host tools for measuring what the firmware cannot see on its own: gate timing, switch-node and coil
waveforms, inductance, ring frequency and ADC noise.

| Quantity | Method | Tool |
|---|---|---|
| Gate timing, dead-time, fault brake | External scope on HS/LS gates | `etc/mcpwm_gate_verify.py` |
| Gate driver, no external instruments | Internal GPIO loopback into MCPWM capture | [`doc/pwm-test-spec1.md`](../../../doc/pwm-test-spec1.md), `test/test_pwm.cpp` |
| Duty cycle at the pin | External scope, auto-stepped | `etc/pico_pwm_duty.py` |
| Coil ripple current | Current clamp on the coil | `etc/pico_capture.py` |
| Coil inductance `L0` | DCM transfer relation from on-board sensors | [Coil inductance](coil-inductance.md) |
| DCM ring frequency | Switch-node probe | [DCM coil ringing](../internals/dcm-ringing.md) |
| ADC noise, ripple, filter behaviour | Raw sample stream over TCP | `./etc/scope.py` (adcscope) |

## Scope capture

The capture scripts in `etc/` drive a legacy PicoScope 2000 (ps2000 driver) headlessly through `ctypes`, as one
example of a scriptable scope. The driver libraries ship with the PicoScope application and are x86_64-only; on Apple
Silicon the scripts re-exec themselves under Rosetta. Any scope works for the manual procedures below.

```bash
# coil ripple current, clamp on channel A (mV/A of your clamp), window sized to ~4 switching periods
python3 etc/pico_capture.py --range 200mv --mv-per-a 100 --fsw 39000
```

`pico_capture.py` also has `--ch-b` (voltage across the coil on channel B), `--didt` (single-shot `L = V/(di/dt)`)
and `--monitor` (free-running min/max/pk-pk). The ps2000 buffer holds about 3968 samples per block.

:::warning Probing the switch node
Switch-node and high-side gate signals are referenced to a node that swings between ground and Vin. Use a
differential probe or an isolated scope for them; a ground clip on the switch node shorts the half-bridge through
the scope ground.
:::

## Gate-driver verification

Two distinct procedures:

- **Without instruments** — [`doc/pwm-test-spec1.md`](../../../doc/pwm-test-spec1.md) validates the MCPWM driver
  (`src/pwm/mcpwm.h`) using the internal GPIO-matrix loopback into the MCPWM capture channels (12.5 ns resolution).
  Runs as part of the on-target Unity suite in an MCPWM build, see [Testing](../development/testing.md).
- **With a scope** — `python3 etc/mcpwm_gate_verify.py` drives the device over the console and captures both gates
  (channel A = `board.conf::pwm_hi`, channel B = `pwm_li`), then prints PASS/FAIL per assertion and exits with the
  number of failed rows. A free GPIO wired to `board.conf::pwm_fault_pin` exercises the fault brake; skip that with
  `--skip-fault`.

:::danger
The verifier drives the gates up to near-full duty. Its hostname allow-list does not prove the power stage is
disconnected: an unnamed real converter reports the default `fugu-esp32s3-…` hostname and passes. Disconnect panel
and battery first.
:::

```bash
python3 etc/mcpwm_gate_verify.py --serial $ESPPORT
```

`etc/pico_pwm_duty.py` pairs with `test_mcpwm_endpoint_duty_scope` in `test/test_pwm.cpp`: the device dwells on a
list of duties and prints sync markers, the script captures mid-dwell and prints a configured-vs-measured table.

## Coil inductance

`coil.conf::L0` drives the CCM/DCM decision in diode emulation, so measure it per board. Both a host script
(`etc/measure_coil.py`) and an on-device command (`measure-coil`, `CONFIG_FUGU_WITH_MEASURE_COIL=y`) derive `L` from
`Vin`, `Vout` and `Iout` without a current probe. See [Coil inductance](coil-inductance.md).

## DCM ring frequency

After the coil current reaches zero in DCM, the coil rings with the switch-node capacitance. The frequency depends on
the FETs and layout of each board, so measure it on the switch node rather than reusing another board's value. See
[DCM coil ringing](../internals/dcm-ringing.md) for the model and what the ring period means for low-side timing.

## ADC noise

The `scope` service streams raw ADC samples over TCP. The host client is the `etc/adcscope` submodule, launched
through a wrapper that registers device discovery:

```bash
git submodule update --init etc/adcscope
./etc/scope.py                      # discover via mDNS, pick in the UI
./etc/scope.py --ip <device-ip>     # connect directly
```

The service is on by default and needs Wi-Fi; toggle it with `svc on|off scope` or `scope.conf::enabled`. Captures are saved per channel and can be
reopened offline with `--load`. The wire format is `etc/adcscope/PROTOCOL.md`; filter design notes are in
[Signal filters](../internals/signal-filters.md).

## Efficiency and thermal soak 🆕

:::note Method outline
This section outlines what to measure; there is no validated procedure or tooling for it in the repository yet.
:::

- **Efficiency:** measure input and output power with external meters (4-wire where currents are high), not with the
  board's own sensors, which carry calibration error and the `power_conversion_eff` assumption for virtual channels.
  Sweep load at several input voltages; record duty, conduction mode (CCM/DCM) and switching frequency per point.
- **Thermal soak:** run at a fixed operating point until temperatures settle. Log `ntc_temp` and the MCU temperature
  from telemetry alongside FET and coil temperatures from external probes, and note where `limits.conf::temp_derate`
  starts reducing power.
- **Record the setup:** firmware version, config profile, coil, dead-time and PWM driver, so points stay comparable.
