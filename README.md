# FUGU-ARDUINO-MPPT-FIRMWARE

This is a complete re-write of the
original [FUGU-ARDUINO-MPPT-FIRMWARE](https://github.com/AngeloCasi/FUGU-ARDUINO-MPPT-FIRMWARE) by AngeloCasi.
It is compatible with
the [original hardware design](https://www.instructables.com/DIY-1kW-MPPT-Solar-Charge-Controller/) you can find on
Instructables.

The charger uses a simple CC (constant current) and CV (constant voltage) approach.
This is common for Lithium-Batteries (e.g. LiFePo4).

Highlights:

* Tested with ESP32 and ESP32-S3
* Async ADC sampling for low latency control loop (<900µs in-out latency)
* Automatic zero-current calibration
* ADC abstraction layer with implementations for ESP32(S3) [Internal ADC](doc/Internal%20ADC.md), ADS1x15 and
  INA226/INA228
* PID control for precise voltage and current regulation
* Periodic MPPT global search
* Sophisticated Diode Emulation for low-side switch
* Battery voltage detection
* Fast protection shutdown in over-voltage and over-current conditions
* PWM Fan Control and temperature power de-rating
* Telemetry to InfluxDB over UDP
* LCD (hd44780) and WS2812B LED Indicator
* [Serial UART console](doc/Serial%20Console.md) and telnet to interact with the charger
* Unit tests

The firmware sends real-time data to InfluxDB server using UDP line protocol.

The aim of this program is to provide a flexible MPPT and DC/DC converter solution that you can use with various
hardware topologies (e.g. buck & boost, location of current sensor).
You can configure pins, limits, converter topology and sensors through config files, without the need to rebuild the
firmware.
Access files through FTP or USB Mass Storage Class (MSC, ESP32-S3).
I tried to structure components in classes, so they reflect the physical and logical building-blocks of a MPPT solar
charger.
See [Voltage & Current Sensors, ADC](#Voltage Current Sensors (ADC))
Feel free to use parts of the code.

# Reference Hardware

* [Fugu2](https://cadlab.io/project/27217/master/files) (KiCad)
    * Dual parallel HS switches
    * Snubber circuit for reduced EMI
    * INA226 current sensor
* [Original Fugu](https://www.instructables.com/DIY-1kW-MPPT-Solar-Charge-Controller/) (Proteus)

# Getting Started

You can build with ESP-IDF toolchain using Arduino as a component.

If you want to use PlatformIO, checkout `tag/pio-last`. The PIO build branch is currently not maintained.
This version works with the original Fugu design. Mind the voltage divider values. ADS1015 and ACS712-30 hall.

## Building with ESP-IDF

Follow [Espressif's Get Started guide](https://docs.espressif.com/projects/esp-idf/en/v5.1.4/esp32/get-started/index.html)
to
install ESP-IDF v5.1.4 . The firmware depends on `arduino-esp32` so we can use Arduino libraries.
Since v3.x `arduino-esp32` is compatible with `esp-idf v5.1` (before we had to use `esp-idf v4.4`).
To install `esp-idf v5.1.4` you can follow these commands (make sure you have all the prerequisites from
the Espressif guide mentioned before and you might have to downgrade python to 3.9 if running into issues
like [this](https://github.com/espressif/esp-idf/issues/12322), note that esp-idf will create a new python virtual
environment with your system's default python version `python --version`):

```
git clone -b v5.1.4 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.1
cd esp-idf-v5.1
./install.sh esp32s3
 . ./export.sh
```

Build the MPPT firmware:

```
git clone --recursive https://github.com/fl4p/fugu-mppt-firmware
cd fugu-mppt-firmware
idf.py set-target esp32s3 # (or esp32)
idf.py build
idf.py flash
```

## Configuring Build

* Set environment variable `RUN_TESTS=1` to run unit-tests
* `FUGU_BAT_V`: hard-code the battery voltage. If not set the program tries to detect bat voltage from a multiple of
  14.6V.

## Board Configuration

IO pins mappings, sensor and system (I2C, WiFi, etc.) config values are stored in `.conf` files on the `littlefs`
partition.
This enables easy OTA updates of the firmware across various hardware configurations. And you can easily alter the
configuration by flashing a new `littlefs` image or by editing the files over FTP. Some crucial parameters are still
hard-coded, making them configurable is WIP.

You find existing board configuration in the folder [`provisioning/`](provisioning/):

* `fmetal`: [Fugu2 board](https://github.com/fl4p/Fugu2)
* `fugu`: original fugu design
* `dry`: useful for testing with ESP32 dev boards, uses fake ADC

Chose the board and flash these config files (set `PROV` to the name of the folder under `provisioning/`):

```
BOARD=dry
littlefs-python create provisioning/$PROV $PROV.bin -v --fs-size=0x20000 --name-max=64 --block-size=4096
parttool.py --port /dev/cu.usb* write_partition --partition-name littlefs --input $PROV.bin
```

Alternatively, in `CMakeLists.txt`, add `FLASH_IN_PROJECT` argument for `littlefs_create_partition_image()`. Then the
config files will be flashed with the next `idf.py flash`:

```
littlefs_create_partition_image(littlefs provisioning/fmetal
  FLASH_IN_PROJECT
)
```

Once you've built and flashed the firmware on the device, use the serial console to connect the chip to your
Wi-Fi network:

```
idf.py monitor
> wifi-add <ssid>:<password>
> restart
```

If Wi-Fi connection is successful you will be able to connect with telnet and FTP.
You can send the same commands over telnet as over the [Serial console](doc/Serial%20Console.md).

Use FTP to upload HW configuration files to configure IO pins, ADC and converter topology.
Note that FTP server is unstable. It seems to work well with the Filezilla client.
FTP settings: 1 simultaneous connection, disable passive mode.

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

# Voltage & Current Sensors (ADC)

The firmware tries to be as hardware independent as possible by using layers of abstraction (HAL), so you can easily
adopt it
with your ADC model and topology. Implementations exist for the ADS1x15, INA226, esp32_adc.

The hardware should always sense `Vin` and `Vout`. `Vin` is not crucial and can
be coarse (8-bit ADC might be ok if there is a current sensor at `Iout`), it is needed for diode emulation, under- and
over-voltage shutdown. Since `Vout` is our
battery voltage it should be more precise.
To reduce voltage transients during load change a high sampling rate is prefered.

The current sensor can be either at the input (`Iin`, solar) or output (`Iout`, battery) or both. If there's only one
current sensor we
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
shaded solar strings. A scan lasts about 20 to 60 seconds, depending on the loop update rate. Scanning too often or slow
scanning ca significantly less reduce overall efficiency.

# Synchronous Buck and Diode Emulation

We can leave the Low-Side (LS, aka *sync-FET*, *synchronous rectifier*) switch off and the coil discharge current will
flow through the LS MOSFET´s body diode.
The buck converter then operates in non-synchronous mode. This decreases conversion efficiency but prevents the buck
converter from becoming a boost converter. Voltage boosting causes reverse current flow from battery to solar and can
cause excess voltage at the solar input, eventually destroying the LS switch and even the board. So we must take timing
the LS switch very carefully.

The firmware implements a synchronous buck converter. It uses the Vout/Vin voltage ratio to estimate the slope of the
coil current and adjusts the switching time of the LS MOSFET so that the current never crosses zero.
It handles both Continuous Conduction Mode (CCM) and Discontinuous Conduction Mode (DCM). The LS FET stays off during
low-power conversion (apart from a minimum on-time to keep the charge pump for the HS gate driver active).
This approach allows arbitrary buck duty cycles, without trouble.

For additional safety the low-side duty cycle is slowly faded to its maximum value. As soon as we detect reverse
current (which might also be noise), we decrease the LS switch duty cycle and slowly recover.

# Not implemented / TODO

* usb console CDC multiplex
* make buck signal pins configurable
* learn buck start duty cycle, buck self test * Calibration
    * find pwm min duty (vout > 0 or iout > 0)
* 2nd and more (interleaved) channels
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

## Issues

* There is a design issue with `IoutCTRL` and `PowerCTRL`. In an over-load situation, the controllers will
  decrease duty-cycle, which can increase solar voltage, thus increase conversion power. In this case the converter will
  increases power until it runs into the hard
  limits, shuts down and recovers. Because this is a transient situation, it should not cause any damage to hardware.

# Using this Firmware

I am currently using this firmware on a couple of Fugu Devices in a real-world application. Each device is connected to
2s 410WP solar panels, charging an 24V LiFePo4 battery. They produce more than 4 kWh on sunny days.

I'd consider the current state of this software as usable. However, a lot of things (WiFi, charging parameters) are
hard-coded. ADC filtering and control loop speed depend on the quality of measurements (noise, outliers) and need to be
adjusted manually.

The original Fugu HW design has some flaws (hall sensor placement after input caps, hall sensor too close to coil,
sense wires layout). Using the CSD19505 at the HS is not a good idea: it is a MOSFET designed for rectificiation and has
a large Qrr (body diode reverse recovery charge) which will cause a lot of ringing noise.

Interference increases with power, so we can slow down the control loop to ensure a steady output. Otherwise the
converter might repeatedly shutdown, wasting solar energy. A slow control loop however causes higher voltage
transients during load changes (e.g. BMS cut-off) which can be dangerous for devices.

If the battery or load is removed during power conversion expect an over-voltage transient at the output.
With a battery voltage of 28.5V, I measured 36V for 400ms.

Keep in mind that in case of a failure (software or hardware), the charger might permanently output the max solar
voltage at the battery terminal, potentially destroying any connected device. Add over-voltage protection (another
DC/DC, Varistor, TVS, crowbar circuit) if necessary.

Use it at your own risk.

# Contribution

We need contributors for Hardware Design and Software. Open an issue or pull request or drop me an email (you find my
address in my github profile) if you want to contribute or just share your experience.

# Resources

* [TI Power Topologies Handbook](https://www.ti.com/seclit/ug/slyu036/slyu036.pdf#page=18) (timings CCM, DCM, forced
  PWM)

* [Power conversion efficiency measurement](https://github.com/fl4p/fugu-mppt-doc/blob/master/Power%20Measurements.md#findings)

* [Renesas: Can you explain diode emulation and why it is used?](https://en-support.renesas.com/knowledgeBase/4967491)

* [fetlib - find the right MOSFET](https://github.com/fl4p/fetlib)
