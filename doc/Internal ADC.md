# Poor ADC

ESP32's internal ADC has non-linearity issues and is quite noisy.

The linearity can be fixed with calibration and we keep a margin to the ADC voltage range. (e.g. at 6dB PGA attenuation
150 mV ~ 1750 mV,
see [suggested range](https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#_CPPv425adc1_config_channel_atten14adc1_channel_t11adc_atten_t))
For simple curve fit see [here](https://github.com/espressif/esp-idf/issues/164#issuecomment-318861287).
This can be improved with a look-up-table (LUT).

We can reduce noise with a sliding mean filter, which reduces update rates.
The ESP32 ADC being much faster than the ADS1015, the reduced update rate is still sufficient for an MPPT application.

# Wiring

To use the internal ADC1 of the ESP32/ESP32-S3, just omit ADS1015 or ADS1115 (U10) and the ALERT 10k pull-up resistor
(R31).

The firmware will automatically switch to internal ADC1 if it doesn't find the ADS1015 or ADS1115 on the I2C bus.

Remove the ADS chip and the pull-up resistor next to it. Then use thin cables or isolated copper wire to
connect the analog signals directly with the ESP32 chip.

## ESP32-S3-WROOM-1

```
A2 (solar current) ---> GPIO4 (ADC1_CHANNEL_3)
A3 (solar voltage) ---> GPIO5 (ADC1_CHANNEL_4)
A4 (bat voltage)   ---> GPIO6 (ADC1_CHANNEL_5) (orignally ADC-ALERT)
```

## ESP32-WROOM-32

```
A2 (solar current) ---> GPIO36 / SENSOR_VP (ADC1_CHANNEL_0)
A3 (solar voltage) ---> GPIO34 (ADC1_CHANNEL_6) (orignally ADC-ALERT)
A4 (bat voltage)   ---> GPIO39 / SENSOR_VN (ADC1_CHANNEL_3) 
```
