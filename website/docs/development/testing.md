---
title: Testing
sidebar_position: 6
---

# Testing

The firmware has four test layers. The following table lists them from fastest to most hardware-dependent.

| Layer | Where | Needs |
|---|---|---|
| Host unit tests | `test/host-stub/*-test.cpp` | A C++ compiler |
| Host Python tests | `test/host_py/` | Python |
| On-target unit tests | `test/test_*.cpp` (Unity) | An ESP32-S3 board with no powered stage (drives gate pins) |
| End-to-end tests | `etc/e2e-test/` | A device over serial/telnet; some clusters need a power stage |

For power-stage tests with a programmable supply and load, see [Automated Bench Tests](../lab/automated-bench-tests.md).

## Host unit tests

Each `test/host-stub/*-test.cpp` is self-contained. Shims for the ESP-IDF and Arduino headers live in `test/host-stub/`.
The following command builds and runs one test:

```bash
clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src \
    -o /tmp/service-test test/host-stub/service-test.cpp && /tmp/service-test
```

## On-target unit tests

`RUN_TESTS=1` builds the Unity suite by swapping `src/main.cpp` for `test/main.cpp`.

:::danger Bare board only
Run the suite on a dev board or on a Fugu board with the power stage unpowered (no PV, no battery). The suite
drives GPIO 1, 2, 4-9, and 21 as outputs. GPIO 21 is the high-side gate input on Fugu2 boards. The ISR tests pulse
it for about 1 µs and leave it LOW, but it still switches the gate. `idf.py flash` also overwrites the littlefs
config with `config/lab/dry_mock`. If the littlefs config must be kept, use `app-flash`.
:::

To build and flash the suite and open the console, run:

```bash
RUN_TESTS=1 idf.py -B build-tests build
RUN_TESTS=1 idf.py -B build-tests -p $ESPPORT flash
python3 etc/fugu_console.py -p $ESPPORT      # then press RESET/EN to see the Unity output
```

The suite runs once from `setup()`, so press the board's RESET/EN button after the console is open. As an
alternative, `idf.py -p $ESPPORT flash monitor` resets the board and attaches in one step.

The PWM/MCPWM tests only run in an MCPWM build (`CONFIG_FUGU_WITH_MCPWM=y`). The software suite passes without it.

`MAIN_SRC=<file>` replaces the application sources with one file that provides `setup()`/`loop()`, for example
`MAIN_SRC=../test/main_ads_rate.cpp idf.py build`. The `test_*.cpp` files run under `test/main.cpp` via
`RUN_TESTS=1`. They aren't entry points.

## End-to-end tests

`etc/e2e-test/run_e2e.py` groups the `test_*.py` scripts into clusters by the setup they need. It skips the
scripts whose prerequisites are missing and exits 1 on any failure. A run where nothing passed or failed
(everything skipped) exits 2 and doesn't count as a pass.

The following table lists the clusters, their setup, and whether they're safe to run on a live converter.

| Cluster | Setup | Safe on live converters |
|---|---|:---:|
| `console` | Any device, console only | Mostly: it runs `tasks`/`rt-stats`, which can starve the continuous-ADC DMA on a busy configuration |
| `mock` | A mock-ADC build over serial | n/a |
| `destructive` | Bench unit only: deliberately panics, reboots, fuzzes | **No** |
| `power` | Real converter with coil; drives the half-bridge | **No** |
| `wifi` | Controllable access point / router rig | No |

The following commands list the tests and run clusters over serial or telnet:

```bash
python etc/e2e-test/run_e2e.py --list
python etc/e2e-test/run_e2e.py --cluster console --serial <serial-port>
python etc/e2e-test/run_e2e.py --cluster console --telnet <device-ip>:23 --mqtt-host <broker-ip>
python etc/e2e-test/run_e2e.py --cluster destructive --serial <serial-port> --with-fuzz
```

Tests that need extra setup read it from flags or from these environment variables: `$MQTT_HOST`,
`$RESTART_URL`, `$E2E_SSID`, `$E2E_OTHER_SSID`, `$E2E_PSK`, `$E2E_ROUTER`, `$E2E_ROUTER_WAN_IP`.

## Without hardware

Three setups run the firmware without a power stage:

- `config/lab/dry_mock` runs the firmware on any ESP32-S3 dev board with a sinusoidal mock ADC. See
  [Mock ADC](#mock-adc).
- `CONFIG_FUGU_WITH_VCONV=y` with `config/lab/vconv_mock` closes the loop around a simulated converter. See
  [Build Options](../guide/getting-started/build-options.md#control-loop-work-without-hardware).
- `config/lab/wokwi_mock` runs in the [Wokwi](https://wokwi.com) simulator.

### Mock ADC

Use `config/lab/dry_mock` to check the firmware, console, Wi-Fi, and services on a new chip or build. It replaces
all sensors with a fake ADC that produces sinusoidal readings, so the control loop runs without a power stage.

To provision a copy of the mock configuration and open the console, run:

```bash
cp -r config/lab/dry_mock /tmp/mock
cp config/fmetal/conf/charger.conf /tmp/mock/conf/   # dry_mock has no charger.conf
rm -f /tmp/mock/conf/mqtt.conf                        # lab broker settings, not yours
export ESPPORT=/dev/cu.usbmodemXXXX
./provision.py /tmp/mock
python3 etc/fugu_console.py -p $ESPPORT
```

The copy adds `charger.conf` because the firmware requires `charger.conf::vout_max`. Without it, setup logs
`error during sensor/converter/tracker setup: vout_max must be a positive finite voltage …` and the control loop
does not start.

In the console, run these commands:

```
uptime
sensor
status
wifi-add <ssid>:<password>
restart
```

Then check the following:

- [ ] No `E (…)` error lines in the boot log. `board.conf expects MCU …` means the config does not match the chip.
  See [ESP32 variants](../guide/hardware/esp32-variants.md).
- [ ] `sensor` lists `vin`, `vout`, and `iout` with changing values.
- [ ] `ip` shows an address after the restart, and `svc list` shows the network services you expect.

:::danger
Never provision a mock configuration on a board that is wired to a panel, supply, or battery. The protections act
on the fake readings, not on the real voltages. `dry_mock` also sets zero dead-time, and its half-bridge pins may not
match your board's.
:::
