# Sensors

You can configure the voltage and current sensors through the file `conf/sensors` according to your topology and chips.

There are 4 sensors (Vin, Vout, Iin, Iout). Your hardware topology can have one or two current sensors.

In case of one sensor, the other current is computed using the voltage ratio and conversion efficiency.

```
adc =   ina226    # choose the ADC chip (ina226,ads1015,ads1115,esp32adc1)

vin_ch = 2        # ADC channel for V_in
vin_rh = 200e3    # upper resistor of voltage div.
vin_rl = 7.5e3    # lower resistor
vin_calib = 1.0   # calibration factor

vout_ch = 0       # V_out ADC channel
vout_rh = 47e3    # upper resistor of voltage div.
vout_rl = 47e3    # lower resistor
vout_calib = 1.0  # calibration factor

iout_ch=1           # I_in ADC channel
iout_filt_len=30    #
iout_factor=1       #
iout_midpoint=0     #

#iin_ch=NA          # virtual sensor
#iin_filt_len=30    #
#iin_factor=-20.15  # set sensivity (A/V)
#iin_midpoint=1.88  # set zero offset (for ACS712)

expected_hz=80      # loop latency watchdog (set 0 to disable)
conversion_eff=0.97 # estimated power efficency for virtual sensor computation

```

# ADC

Choose a ADC chip. Currently implemented

* ina226
* ads1115
* ads1015

TODO:

* TODO: ina228
* TODO: esp32adc1

# Voltage sensors `vin`, `vout`

Specify the resistor values of the ADC input voltage divider network.
The firmware uses (hardcoded) ADC input impedance and resistor values to compute the gain.

```
vout_rh = 47e3    # upper resistor of voltage divider
vout_rl = 47e3    # lower resistor
```

# Converter.conf


### `forced_pwm`
With the default value `0`, the converter will run in DCM during light load conditions.
Notice that the controller uses inductivity (from `coil.conf`), input and output voltages to decided if the converter
operates in DCM or CCM.

If set to `1`, diode emulation is disabled and the converter will always run in CCM, which is called forced PWM mode and has these characteristics:

* less output noise because inductor is never "free-wheeling".
  See [here](https://www.nisshinbo-microdevices.co.jp/en/faq/083.html) for wave forms
* much better output regulation during load changes, useful for PSU
* less conversion efficiency due to reverse coil current. energy is charged back and forth between output and input.
* a buck converter in FPWM can easily boost voltage from output to input

Forced PWM is useful if you want to use the converter as power supply.


# ACS712

The ACS712 sensitivity is 66mV/A. Output is scaled with a 10k+3.3k voltage divider to match the ADC voltage range.
This is encoded into `iin_factor`.

Specify the ACS712 midpoint voltage with `iin_midpoint` (or `iout_midpoint`).
Here the ACS712 has a MP of 2.5V, which is scaled through a voltage divider (10k + 3.3k).
`2.5V * 10k/(10k+3.3k)`

```
iin_factor=-20.15  # sensivity = -1/0.066 * (10k+3.3k)/10k
iin_midpoint=1.88  # midpoint  = 2.5V * 10k/(10k+3.3k)
```

# Bare ESP32 example

Here's a minimal dummy configuration:

`/sensor`

```
adc=esp32adc1


vin_ch = 4        # ch4 GPIO5
vin_rh = 200e3    # upper resistor of voltage div.
vin_rl = 1.5e3    # lower resistor

vout_ch = 5	      # ch5 GPIO6
vout_rh = 47e3    # upper resistor of voltage div.
vout_rl = 1e3     # lower resistor

iin_ch = 3        # ch3
iin_factor = 20
iin_midpoint = 0
iin_filt_len = 30

iout_filt_len=30

expected_hz=80
conversion_eff=0.97 # TODO does this belong here?

ignore_calibration_constraints=1 # skip noise and range check (not for production!)
```

Use this configuration to run the firmware on an ESP32 without any external ADC. This is useful for testing other things
than the ADC and PWM.

If you leave the ADC pins floating, the values will be garbage with a high stddev. That's why we
set `ignore_calibration_constraints=1` here.

