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

# Sampling Rate

We configure ESP32's ADC unit 1 in continuous mode with a sampling rate of 80 kHz (`adc_continuous_config`).
This means that the ADC does 80k conversions per second. We apply averaging of 32 samples to reduce the quite noisy
readings. With 4 channels this gives 625 samples per second per channel (80000 / 3 / 32).

On the ESP32-S3 we can achieve this theoretic rate, the ESP32 appears to be a bit slower (511 / 625 @ 80 kHz, 639 /
781 @ 100khz ) and always at ~80% of the calculated rate. This ratio is constant across tested sampling rates 80k, 100k,
125k, 128k, 156.25k, 160k, 200k, 250k, 312.5k, 320k and 400kHz.

At 640 kHz the ESP32 works at 70% of expected sampling rate and with significant variance. This might be due to a slow
control loop.
