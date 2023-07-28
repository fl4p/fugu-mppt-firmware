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
* PID voltage and current control
* Periodic MPPT global scan
* Sophisticated Diode Emulation for low-side switch
* Battery voltage detection
* Fast protection shutdown in over-voltage and over-current conditions
* PWM Fan Control and temperature power limiting / derating
* Telemetry to InfluxDB over UDP

The firmware sends real-time data to InfluxDB server using UDP line protocol.

# Not implemented / TODO

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
voltage at the battery terminal, potentially destroying any connected device. Add over-voltage protection (another DC/DC, varistor, TVS, crowbar circuit) if necessary.

Use it at your own risk.

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

## ENV Variables
* Set `RUN_TESTS=1` to run unit-tests
* Set the solar-input voltage divider resistor with `FUGU_HV_DIV`. Original board design uses 5.1k: `FUGU_HV_DIV=5.1`. Defaults to 4.7.
* `FUGU_BAT_V`: hard-code the battery voltage. If not set the program tries to detect bat voltage from a multiple of 14.6V.

# Contribution

We need contributors for Hardware Design and Software. Open an issue or pull request or drop me an email (you find my
address in my
github profile) if you want to contribute or just share your experience.


# Resources
* [Power conversion efficiency measurement](https://github.com/fl4p/fugu-mppt-doc/blob/master/Power%20Measurements.md#findings)
