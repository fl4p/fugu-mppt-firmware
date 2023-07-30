# FUGU-ARDUINO-MPPT-FIRMWARE

This is a complete re-write of the
original [FUGU-ARDUINO-MPPT-FIRMWARE](https://github.com/AngeloCasi/FUGU-ARDUINO-MPPT-FIRMWARE) by AngeloCasi.
It is compatible with
the [original hardware design](https://www.instructables.com/DIY-1kW-MPPT-Solar-Charge-Controller/) you can find on
Instructables.

The charger uses a simple CC (constant current) and CV (constant voltage) approach.
This is common for Lithium-Batteries (e.g. LiFePo4).

* Compatible with ESP32 and ESP32-S3
* Async ADC sampling
* Zero-current calibration
* Can use ESP32/ESP32-S3's internal ADC1 instead of the external ADS1x15, see [Internal ADC](doc/Internal%20ADC.md)
* PID control for voltage and current regulation
* Periodic MPPT global scan
* Sophisticated Diode Emulation for low-side switch
* Battery voltage detection
* Fast protection shutdown in over-voltage and over-current conditions
* PWM Fan Control and temperature power limiting / derating
* Telemetry to InfluxDB over UDP

The firmware sends real-time data to InfluxDB server using UDP line protocol.

# Building

You can build with PlatformIO or ESP-IDF using Arduino as component build toolchain.

Here's how to build using ESP-IDF:

```
git clone --recursive https://github.com/fl4p/fugu-mppt-firmware
cd fugu-mppt-firmware
```

Depending on your target chip, either rename `sdkconfig.esp32` or `sdkconfig.esp32s3` to `sdkconfig`.
Then build

```
idf.py build
```

## Configuring Build

* Set environment variable `RUN_TESTS=1` to run unit-tests
* Set the solar-input voltage divider resistor with `FUGU_HV_DIV`. Original board design uses 5.1k: `FUGU_HV_DIV=5.1`.
  Defaults to 4.7.
* `FUGU_BAT_V`: hard-code the battery voltage. If not set the program tries to detect bat voltage from a multiple of
  14.6V.

# Control Loop

The control loop reads Vout, Vin, Iin and adjust the PWM duty cycle of the buck DC-DC converter for MPPT and output
regulation.  
Besides the MPP tracker, the control loop contains 5 PD control units (PID without the integral component):

- `VinCTRL` (solar voltage), keeps solar voltage above 10.5V to prevent board supply UV
- `IinCTRL` (solar current), limits input current to protect hardware
- `VoutCTRL` (bat voltage), regulates the output voltage when battery is full or disconnected
- `IoutCTRL` (charge current), controls the charge current
- `PowerCTRL` (thermal derating), limits conversion power to prevent excess temperatures

The `VoutCTRL` is the fastest controller. Keeping the output voltage in-range with varying load is most crucial to
prevent damage from transient over-voltage. Because this is powered by solar, reacting on short-circuits is not
important, so `IinCTRL` (and `IoutCTRL`) can be slow.

In each loop iteration we update all controllers and pick the one with the minimum response value. If it is positive, we
can proceed with the MPPT. Otherwise, we halt MPPT and decrease the duty cycle proportionally to the control value.

The control loop has an update rate of about 160 Hz or 260 Hz without telemetry.

# MPPT Algorithm

The tracking consists of 3 phases:

1. Global scan
2. Fast tracking (observe & perturb)
3. Slow tracking (observe & perturb)

The controller starts with a global scan, at a duty cycle of 0 and linearly increases it while capturing the
maximum power point (MPP) until one of these conditions are met:
input under-voltage, output over-voltage, over-current, 100% duty cycle.

It then sets the duty cycle to the captured MPP and goes into fast tracking mode to track the MPP locally.

After a while it switches to slow tracking mode, trying to reduce the mean tracking error.
When it detects a mayor change in power conditions (e.g. clouds, partial shading), it'll switch back to fast tracking,
to quickly adapt to the new condition.

A global scan is triggered every 30 minutes to prevent getting stuck in a local maximum. This can happen with partially
shaded solar strings. A scan lasts about 20 to 60 seconds.

# Synchronous Buck Diode Emulation

We can leave the LS switch off and the coil discharge current will flow through the LS MOSFET´s body diode. The buck
converter then operates in non-synchronous mode. This decreases conversion efficiency but prevents the buck converter
from becoming a boost converter. Voltage boosting causes reverse current flow from battery to solar and can cause excess
voltage at the solar input, eventually destroying the LS switch and even the board. Timing the LS can be tricky.

The firmware implements a synchronous buck converter. It uses the Vout/Vin voltage ratio to estimate the slope of the
coil current and adjusts the switching time of the Low-Side (LS) MOSFET so that the current never crosses zero.
It handles both Continuous Conduction Mode (CCM) and Discontinuous Conduction Mode (DCM).
This approach allows arbitrary buck duty cycles, without trouble.

For additional safety the low-side duty cycle is slowly faded to its maximum value. As soon as we detect reverse
current (which might also be noise), we decrease the LS switch duty cycle and slowly recover.

# Not implemented / TODO

* More precise PWM
* LCD Buttons
* Web Interface
* OTA Updates
* WiFi Network managing, WiFi power saving / power off
* Bluetooth communication
* Serial / Modbus interface
* Acid Lead, AGM charging algorithm

# Using this Firmware

I am currently using this firmware on a couple of Fugu Devices in a real-world application. Each device is connected to
2s 410WP solar panels, charging an 24V LiFePo4 battery. They produce more than 4 kWh on sunny days.

I'd consider the current state of this software as usable. However, a lot of things (WiFi, charging parameters) are
hard-coded.

If the battery or load is removed during power conversion expect an over-voltage transient at the output.
With a battery voltage of 28.5V, I measured 36V for 400ms.

Keep in mind that in case of a failure (software or hardware), the charger might permanently output the max solar
voltage at the battery terminal, potentially destroying any connected device. Add over-voltage protection (another
DC/DC, varistor, TVS, crowbar circuit) if necessary.

Use it at your own risk.

# Contribution

We need contributors for Hardware Design and Software. Open an issue or pull request or drop me an email (you find my
address in my
github profile) if you want to contribute or just share your experience.

# Resources

* [TI Power Topologies Handbook](https://www.ti.com/seclit/ug/slyu036/slyu036.pdf#page=18) (timings CCM, DCM, forced PWM)

* [Power conversion efficiency measurement](https://github.com/fl4p/fugu-mppt-doc/blob/master/Power%20Measurements.md#findings)
