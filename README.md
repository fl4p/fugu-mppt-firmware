# FUGU-ARDUINO-MPPT-FIRMWARE

This is a complete re-write of the
original [FUGU-ARDUINO-MPPT-FIRMWARE](https://github.com/AngeloCasi/FUGU-ARDUINO-MPPT-FIRMWARE) by AngeloCasi.
It is compatible with
the [original hardware design](https://www.instructables.com/DIY-1kW-MPPT-Solar-Charge-Controller/) you can find on
Instructables.

The charger uses a simple CC (constant current) and CV (constant voltage) approach.
This is common for Lithium-Batteries (e.g. LiFePo4).

* Tested with ESP32 and ESP32-S3
* Async ADC sampling for low latency control loop
* Automatic zero-current calibration
* Can use ESP32/ESP32-S3's internal ADC1 instead of the external ADS1x15 or INA226,
  see [Internal ADC](doc/Internal%20ADC.md)
* PID control for precise voltage and current regulation
* Periodic MPPT global search
* Sophisticated Diode Emulation for low-side switch
* Anti back-flow (back-feed, ideal diode)
* Battery voltage detection
* Fast protection shutdown in over-voltage and over-current conditions
* PWM Fan Control and temperature power limiting, linear de-rating
* Telemetry to InfluxDB over UDP
* LCD (hd44780) and WS2812B LED Indicator
* [Serial UART console](doc/Serial%20Console.md) to interact with the charger
* Unit tests

The firmware sends real-time data to InfluxDB server using UDP line protocol.

The aim of this program is to provide a flexible MPPT and DC/DC solution that you can use with various hardware
topologies (e.g. buck & boost, location of current sensor). I tried to structure components in classes so they
reflect the physical and logical building-blocks of a MPPT solar charger. Feel free to use parts of the code

# Building

You can build with PlatformIO or ESP-IDF toolchain using Arduino as a component.

Here's how to build using ESP-IDF:

```
git clone --recursive https://github.com/fl4p/fugu-mppt-firmware
cd fugu-mppt-firmware
idf.py set-target esp32s3 # (or esp32)
idf.py build
```

## Configuring Build

* Set environment variable `RUN_TESTS=1` to run unit-tests
* Set the solar-input voltage divider resistor with `FUGU_HV_DIV`. Original board design uses 5.1k: `FUGU_HV_DIV=5.1`.
  Defaults to 4.7.
* `FUGU_BAT_V`: hard-code the battery voltage. If not set the program tries to detect bat voltage from a multiple of
  14.6V.
* `USE_INTERNAL_ADC` enable fallback to internal ADC

# Getting started
Once you've built and flashed the firmware on the device, use the serial console to connect the chip to your
wifi network: `wifi-add <ssid> <secret>`.
You can then upload HW configuration files to configure IO pins, ADC and converter topology.
This is currently WIP. I had a hard disk failure and develpment is currently halted.
You can start at an older commit, without the runtime configuration (HW topology hard coded)
* [tag/idf-working](https://github.com/fl4p/fugu-mppt-firmware/releases/tag/idf-working) (2023-09-09) last known working revision using esp-idf toolchain
* tag/pio-last: old revision before adding esp-idf.
These versions work with the original Fugu design. Mind the voltage divider values.

# Control Loop

The control loop reads Vout, Vin, Iin and adjusts the PWM duty cycle of the buck DC-DC converter for MPPT and output
regulation. The control loop updates once a Vout reading is available. This ensures low latency for output voltage
control, which is important (see below).

Besides the MPP tracker, the control loop contains 5 PD control units (PID without the integral component):

- `VinCTRL` (solar voltage), keeps solar voltage above 10.5V to prevent board supply UV
- `IinCTRL` (solar current), limits input current to protect hardware
- `VoutCTRL` (bat voltage), regulates the output voltage when battery is full or disconnected
- `IoutCTRL` (charge current), controls the charge current
- `PowerCTRL` (thermal derating), limits conversion power to prevent excess temperatures

The `VoutCTRL` is the fastest controller. Keeping the output voltage in-range with varying load is most crucial to
prevent damage from transient over-voltage. Because a solar panel is very similar to a constant current
source, `IinCTRL` and `IoutCTRL` can be slower.

`IoutCTRL` and `PowerCTRL` both have variable set-points, provided by charging algorithm and temperature feedback,
respectively.

In each loop iteration we update all controllers and pick the one with the lowest response value. If it is positive, we
can proceed with the MPPT. Otherwise, we halt MPPT and decrease the duty cycle proportionally to the control value.

The control loop has an update rate of about 160 Hz or 260 Hz without telemetry.

# Voltage & Current Sensors, ADC

The firmware tries to be as hardware independent as possible by using layers of abstraction (HAL), so you can easily
adopt it
with your ADC model and topology. Implementations exist for the ADS1x15, INA226, esp32_adc.

The hardware should always sense `Vin` and `Vout`. `Vin` is not crucial and can
be coarse (8-bit ADC might be ok if there is a current sensor at `Iout`), it is needed for diode emulation, under- and over-voltage shutdown. Since `Vout` is our
battery voltage it should be more precise.

The current sensor can be either at the input (`Iin`, solar) or output (`Iout`, battery) or both. If there's only one current sensor we
can infer the other current using the voltage ratio and efficiency of the converter.
The code represents this with a `VirtualSensor`.

If the current sensor is bi-directional, the converter can operate in boost mode, boosting lower solar voltage to a
higher battery voltage. This is not yet implemented.

Here are some relevant types:

* `LinearTransform`: Simple 1-dimensional linear transform (Y = a*X + b) to scale voltage readings and zero-offsetting.
* `ADC_Sampler`: Schedules ADC reads, manages sensors and their calibration
* `CalibrationConstraints`: value constraints a sensor must meet during calibration (average, stddev).
* `Sensor`: Represents a physical sensor with running statistics (average, variance)
* `VirtualSensor`: A sensor with computed values. Also comes with running stats.
* `AsyncADC`: Abstract interface for asynchronous (non-blocking) ADC implementation

Asynchronous here means that we request a sample from the ADC and continue code execution while the conversion is
happening. This improves average CPU utilization and other things can run smoothly even with a slow ADC.

# MPPT Algorithm

The tracking consists of 3 phases:

1. Global scan (aka search, sweep)
2. Fast tracking (observe & perturb)
3. Slow tracking (observe & perturb)

The controller starts with a global scan, at a duty cycle of 0 and linearly increases it while capturing the
maximum power point (MPP) until one of these conditions are met:
input under-voltage, output over-voltage, over-current, max duty cycle.

It then sets the duty cycle to the captured MPP and goes into fast tracking mode to follow the MPP locally.

After a while it switches to slow tracking mode, trying to reduce the mean tracking error.
When it detects a mayor change in power conditions (e.g. clouds, partial shading), it'll switch back to fast tracking,
to quickly adapt to the new condition.

A global scan is triggered every 30 minutes to prevent getting stuck in a local maximum. This can happen with partially
shaded solar strings. A scan lasts about 20 to 60 seconds, depending on the loop update rate. Scanning too often or slow scanning ca significantly less reduce overall efficiency.

# Synchronous Buck and Diode Emulation

We can leave the Low-Side (LS, aka *sync-FET*, *synchronous rectifier*) switch off and the coil discharge current will
flow through the LS MOSFET´s body diode.
The buck converter then operates in non-synchronous mode. This decreases conversion efficiency but prevents the buck
converter from becoming a boost converter. Voltage boosting causes reverse current flow from battery to solar and can
cause excess voltage at the solar input, eventually destroying the LS switch and even the board. Timing the LS can be
tricky.

The firmware implements a synchronous buck converter. It uses the Vout/Vin voltage ratio to estimate the slope of the
coil current and adjusts the switching time of the LS MOSFET so that the current never crosses zero.
It handles both Continuous Conduction Mode (CCM) and Discontinuous Conduction Mode (DCM). The LS FET stays off during
low-power conversion (apart from a minimum on-time to keep the charge pump for the HS gate driver active).
This approach allows arbitrary buck duty cycles, without trouble.

For additional safety the low-side duty cycle is slowly faded to its maximum value. As soon as we detect reverse
current (which might also be noise), we decrease the LS switch duty cycle and slowly recover.

# Not implemented / TODO

* make buck signal pins configurable
* learn buck start duty cycle
* 2nd and more (interleaved) channels
* PWM fade!
* More precise PWM
* LCD Buttons
* Web Interface
* OTA Updates
* WiFi Network managing, WiFi power saving / power off
* Bluetooth communication
* Serial / Modbus interface
* Acid Lead, AGM charging
  algorithm ([1](https://github.com/RedCommissary/mppt-charger-firmware/blob/c0dba8700d3baff1d13a86c43f5ce570b15e01af/source/application/inc/Battery.hpp#L27))
* Boost converter
* Detect burned HS and short LS, and back-flow? (implement self-tests)
* low current, low voltage drop -> disable bf (might sense phantom current due to temperature drift)
* Calibration
    * find pwm min duty (vout > 0 or iout > 0)

## Issues

* There is a design issue with `IoutCTRL` and `PowerCTRL`. In an over-load situation, the controllers will
  decrease duty-cycle, which can increase solar voltage. The converter increases power until it runs into the hard
  limits, shuts down and recovers.

# Current sensing

- ina226
- increase conversion time: reduce random noise, more aliasing
- averaging: eliminate aliasing of I and U readings due to sampling the i2c registers
- consider current sense input filter cut-off frequency and control loop update rate when tuning these values
- Voltage refresh rate should be <10ms, so max averaging is 64 and 140us conversion time (64 * 140us = 9ms)
    - note that we can use the ina226 OV alert feature and shutdown from an ISR

# Using this Firmware

I am currently using this firmware on a couple of Fugu Devices in a real-world application. Each device is connected to
2s 410WP solar panels, charging an 24V LiFePo4 battery. They produce more than 4 kWh on sunny days.

I'd consider the current state of this software as usable. However, a lot of things (WiFi, charging parameters) are
hard-coded. ADC filtering and control loop speed depend on the quality of measurements (noise, outliers) and need to be adjusted manually.

The original Fugu HW design has some flaws (hall sensor placement after input caps, hall sensor too close to coil,
sense wires layout).

Interference increases with power, so we must slow down the control loop to ensure a steady output. Otherwise the
converter might unexpectedly shutdown, wasting solar energy. A slow control loop however causes higher voltage
transients
during load changes (e.g. BMS cut-off) which can be dangerous for devices.

If the battery or load is removed during power conversion expect an over-voltage transient at the output.
With a battery voltage of 28.5V, I measured 36V for 400ms.

Keep in mind that in case of a failure (software or hardware), the charger might permanently output the max solar
voltage at the battery terminal, potentially destroying any connected device. Add over-voltage protection (another
DC/DC, Varistor, TVS, crowbar circuit) if necessary.

Use it at your own risk.

# Contribution

We need contributors for Hardware Design and Software. Open an issue or pull request or drop me an email (you find my
address in my
github profile) if you want to contribute or just share your experience.

# Resources

* [TI Power Topologies Handbook](https://www.ti.com/seclit/ug/slyu036/slyu036.pdf#page=18) (timings CCM, DCM, forced
  PWM)

* [Power conversion efficiency measurement](https://github.com/fl4p/fugu-mppt-doc/blob/master/Power%20Measurements.md#findings)

* [Renesas: Can you explain diode emulation and why it is used?](https://en-support.renesas.com/knowledgeBase/4967491)
