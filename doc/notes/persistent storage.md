To reduce fragmentation of firmware binaries (having a different binary for each hardware configuration) and to enable
streamlined OTA updates, we avoid hard-coding GPIO pin numbers, ADC channel numbers, calibration factors and hardware
limits (voltage & current).
We make these adjustable through configuration files

Partitions:

* data: stores metering, daily statistics
* conf: stores config files (board hw and charger?, wifi)
*

# Things that need to be stored

* daily rolling stats
* calibration data
    * Vin, Vout, I factors
    * NTC temp sensor
* hardware config
    * unique id or name
    * ADC config (ADS, INA226)
    * max input voltage, max out, max input current, max output current, max power
    * max temp
    * pin-out override
        * NTC adc channel
        * SD/EN,IN,LIN,BFlow
    * backflow switch position (pv or bat)
    * current sensor positions (pv or bat)
    * current sensor bidirectional (y or n)
    * max loop lag
* Charger Config
    * battery voltage
    * battery chemistry
    * max current (not the hw limit)
    * topping mode
        * enable/disable
        * voltage, max current
    * linear time voltage ramp
        * increase max voltage linearly after reaching a threshold
* wifi networks

# Format

## Existing options

* Arduino `Preferences` libary uses a littlefs and a single file for each key. With many keys we'll have many files.
  Storage density is a low? (1 file = 2 words: payload and meta)
* JSON is too noisy and complex and doesn't support comments
* YAML has a nice syntax, but is also complex

So we use our own format similar to env files, which looks like this:

```
foo=bar
a=1
#a=2
b=2.6 # inline comment
```

# Samples

```

ssid=mentha
pw=secret

#ssid=df

iin_calibration=1.0345336
vout_calibration=2.3434
vin_calibration=5.1234
```

hw.conf

```
vin_max=80
vout_max=80
iout_max=32
```

pin.conf

```
buck_IN=21 # half bridge driver IN
pwm_IN=21
buck_EN=14

bf_EN=0
#bf_SD=0 # inverted bf output

adc_NTC=7


i2c_sda=2
i2c_scl=21
i2c_freq=800000
```

sensors

```
adc=ina226

vin_ch = 2        #2=chAux, esp32 adc
vin_rh = 200e3    # upper resistor of voltage div.
vin_rl = 7.5e3    # lower resistor
vin_calib = 1.0 # calibration factor

vout_ch=0
vout_rh = 47e3    # upper resistor of voltage div.
vout_rl = 47e3    # lower resistor
vout_calib = 1.0 # calibration factor

iout_ch=1
iin_filt_len=30

#iin_ch=NA # virtual sensor

expected_hz=80
conversion_eff=0.97 # TODO does this belong here?

```

eps32 adc1

```
adc=esp32adc1

#    ADC_Vin = ADC1_CHANNEL_4, // GPIO5
#    ADC_Vout = ADC1_CHANNEL_5, // GPIO6
#    ADC_Iin = ADC1_CHANNEL_3,

vin_ch = 4        #2=chAux, esp32 adc
vin_rh = 200e3    # upper resistor of voltage div.
vin_rl = 7.5e3    # lower resistor

vout_ch = 5
vout_rh = 47e3    # upper resistor of voltage div.
vout_rl = 47e3    # lower resistor

iin_ch=3
iin_filt_len=30

expected_hz=80
conversion_eff=0.97 # TODO does this belong here?

```

```
adc=ads1015

vin_ch = 3
vin_rh =     # upper resistor of voltage div.
vin_rl =     # lower resistor
vin_calib = 1.0 # calibration factor

vout_ch = 1
vout_rh = 47e3    # upper resistor of voltage div.
vout_rl = 2e3    # lower resistor
vout_calib = 1.0 # calibration factor

#iout_ch=NA

iin_ch = 2
iin_filt_len=8
iin_factor=-20.1515 #(-acs712_30_sensitivity * (10+3.3)/10); acs712_30_sensitivity=1./.066;
iin_midpoint=1.8797 # (2.5 * 10. / (10 + 3.3))

expected_hz=30
conversion_eff=0.97

```