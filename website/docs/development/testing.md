---
title: Testing
sidebar_position: 6
---

# Testing

The firmware has these test layers. The following table lists them from fastest to most hardware-dependent.

| Layer | Where | Needs |
|---|---|---|
| Host unit tests | `test/host-stub/*-test.cpp` | A C++ compiler (clang++) |
| Host closed-loop simulation | `test/host-stub/mppt-sim.cpp` (`mppt-sim.sh`) | A C++ compiler |
| Host Python tests | `test/host_py/test_*.py`, `etc/test_*.py` | Python (`.venv`) |
| QEMU tests | `test/unity_qemu.py`, `test/rt_task_qemu.py`, `test/bl_ota_qemu.py` | ESP-IDF with QEMU |
| On-target unit tests | `test/test_*.cpp` (Unity) | An ESP32-S3 board with no powered stage (drives gate pins) |
| End-to-end tests | `etc/e2e-test/` | A device over serial/telnet; some clusters need a power stage |

For power-stage tests with a programmable supply and load, see [Automated Bench Tests](../lab/automated-bench-tests.md).

## Host unit tests

Each `test/host-stub/*-test.cpp` builds into its own executable. `test/host-stub/host-tests.sh` builds and runs all
of them except `scope-test` (an endless streaming prototype), `converter-test` (does not build) and
`conf-hash-test` (a helper that `test/host_py/test_conf_hash.py` builds), in parallel (`-j N`, default 4) or the ones named, and exits non-zero if any fails; `--list` prints the names.
Known failures are reported as `XFAIL` with their reason. The header comment of each test gives its compile command. Shims for the ESP-IDF and Arduino headers live in `test/host-stub/`
(`esp/`, `freertos/`, `mqtt-stub/`, and the `arduino-shim` submodule). Some tests also compile firmware sources, for
example `src/sim/vconv.cpp` or `components/virtual-fat12/src/vfat.cpp`.

The `vfat-core-test` entry runs the `virtual-fat12` package's own core suite
(`components/virtual-fat12/test/test_vfat.cpp`) alongside the firmware-side `vfat-test`, so the standard host run
keeps the FAT12 core coverage without a second copy of it in `test/host-stub/`. The following command builds and
runs one test:

```bash
clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src \
    -o /tmp/service-test test/host-stub/service-test.cpp && /tmp/service-test
```

Tests that link more of the firmware have a wrapper script in `test/host-stub/` that builds and runs them:
`calib-noise-test.sh`, `izero-test.sh`, `settle-test.sh` and `vconv-spice-check.sh`.

The `Host tests` GitHub workflow runs `host-tests.sh` and the closed-loop scenarios on Linux for every push to `main`
and every pull request that changes `src/`, `test/host-stub/`, `config/` or `components/virtual-fat12/`.

### Closed-loop simulation

`test/host-stub/mppt-sim.sh [scenario ...]` builds `mppt-sim.cpp` with the real `src/mppt.cpp`, sensor setup and the
virtual converter plant (`src/sim/vconv.cpp`), then runs the named scenarios (default: all, `--list` prints them). The
exit status reports whether any scenario failed. `minpulse-sim.sh` runs closed-loop checks on top of it.

## Host Python tests

The Python tests need no device. Most run directly, for example
`.venv/bin/python3 test/host_py/test_conf_hash.py -v` or `.venv/bin/python3 etc/test_ota_common.py`; all of them run
under `.venv/bin/python3 -m pytest <file>`, which the files without a `__main__` block need.
`test/host_py/` covers the config, coredump and migration helpers; `etc/test_*.py` cover the console client and the
OTA tools. Besides pytest they need only `rich` and `pyserial`, and the `etc/fugu` and `etc/idf-devtools` submodules.
The `Python tests` GitHub workflow runs both directories on Linux for every push to `main` and every pull request that
changes `etc/`, `test/host_py/`, `config/` or `src/`.

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
RUN_TESTS=1 idf.py -B build/tests build
RUN_TESTS=1 idf.py -B build/tests -p $ESPPORT flash
python3 etc/fugu_console.py -p $ESPPORT      # then press RESET/EN to see the Unity output
```

The suite runs once from `setup()`, so press the board's RESET/EN button after the console is open. As an
alternative, `idf.py -p $ESPPORT flash monitor` resets the board and attaches in one step.

The PWM/MCPWM tests only run in an MCPWM build (`CONFIG_FUGU_WITH_MCPWM=y`). The software suite passes without it.

`TEST_ADC_HW=1` together with `RUN_TESTS=1` builds a suite that runs only the internal-ADC and INA226 liveness tests
(`test/test_adc_hw.cpp`) and skips the tests that drive gate pins.

Without a board, `python3 test/unity_qemu.py` (after `. ./idf-export.sh`) builds the suite in `build/qemu`, runs it in
QEMU (esp32s3), and exits non-zero on a failure or crash. QEMU's flash has no QIO mode, so the script sets DIO in that
build directory's sdkconfig only. The tests that need real pin edges (GPIO interrupts, MCPWM output and capture)
detect QEMU by its blank factory MAC and report as Ignored.

`MAIN_SRC=<file>` replaces the application sources with one file that provides `setup()`/`loop()`, for example
`MAIN_SRC=../test/main_ads_rate.cpp idf.py -B build/ads-rate build`. The `test_*.cpp` files run under
`test/main.cpp` via `RUN_TESTS=1`. They aren't entry points.

| Entry point | Purpose |
|---|---|
| `test/main_ads_rate.cpp` | ADS1x15 sample-rate check on a board |
| `test/main_rt_task.cpp` | RT task start-up check, run in QEMU by `test/rt_task_qemu.py <build-dir>` |
| `test/main_bl_ota.cpp` | Bootloader OTA refusal cases and install, run in QEMU by `test/bl_ota_qemu.py <build-dir>` |

The two QEMU scripts expect a build in its own directory with `CONFIG_ESPTOOLPY_FLASHMODE_DIO=y` in that build's
sdkconfig; the command is in each script's header.

## End-to-end tests

`etc/e2e-test/run_e2e.py` groups the `test_*.py` scripts into clusters by the setup they need. It skips the
scripts whose prerequisites are missing and exits 1 on any failure. A run where nothing passed or failed
(everything skipped) exits 2 and doesn't count as a pass.

The following table lists the clusters, their setup, and whether they're safe to run on a live converter.

| Cluster | Setup | Safe on live converters |
|---|---|:---:|
| `console` | Any device, console only | Mostly: it runs `tasks`/`rt-stats`; check their `loop gap during command` line |
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

Four setups run the control code without a power stage:

- `config/lab/dry_mock` runs the firmware on any ESP32-S3 dev board with a sinusoidal mock ADC. See
  [Mock ADC](#mock-adc).
- `CONFIG_FUGU_WITH_VCONV=y` with `config/lab/vconv_mock` closes the loop around a simulated converter. See
  [Build Options](../guide/getting-started/build-options.md#control-loop-work-without-hardware).
- `config/lab/wokwi_mock` runs in the [Wokwi](https://wokwi.com) simulator.
- The host simulator `test/host-stub/mppt-sim.sh` runs the control path against the same plant in simulated time,
  without a board. See [Simulation](simulation.md#host-simulator).

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
