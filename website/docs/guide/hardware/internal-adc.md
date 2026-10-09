---
title: "Internal ADC"
sidebar_position: 3
---

# Internal ADC

## Linearity and noise

The ESP32 internal ADC has a poor reputation: it is non-linear and known to be noisy.

Calibration fixes the linearity, as long as the input stays within the voltage range Espressif suggests
(e.g. at 6dB PGA attenuation 150 mV ~ 1750 mV,
see [suggested range](https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#_CPPv425adc1_config_channel_atten14adc1_channel_t11adc_atten_t)).
For a simple curve fit, see [this ESP-IDF issue comment](https://github.com/espressif/esp-idf/issues/164#issuecomment-318861287).
A look-up table (LUT) improves on this.

Much of the noise comes from how measurements are taken. A simple approach is a periodic timer that takes a
single-shot measurement with the Arduino API `analogRead()`, expecting 12-bit precision. This ignores conversion time.
The shorter the conversion, the higher the noise, and `analogRead()` doesn't tell the ADC how long to measure.

Timed until return, `analogRead` takes only a few tens of microseconds. It calls `adc_oneshot_read` and
`adc_oneshot_hal_convert`. The ADC characteristics in the ESP32-S3 data sheet show a 100ksps sampling rate with the DIG
controller (2Msps on the ESP32), so the measurement itself likely takes under a microsecond, which explains the noise.

Longer conversion gives better results, and a periodic single-shot measurement leaves the ADC mostly idle. A
measurement that takes 100µs, taken every 100ms, gives an effective ADC duty cycle of 0.1% (1/1000), so the ADC is idle
99.9% of the time.

To use the ADC fully, the firmware runs continuous measurements and averages the samples down to the
desired sampling rate. Averaging the raw adc int16 readings gives the best performance.
`sensor.conf::esp32adc1_sr` sets the continuous rate, and IDF caps it at `SOC_ADC_SAMPLE_FREQ_THRES_HIGH`
(83.3 kHz on the ESP32-S3). At that maximum, 32-sample averaging works well. With 3 channels this gives a
per-channel rate of ~ 868 sps (=83333 / 32 / 3). [Sampling rate](#sampling-rate) gives the per-channel rates for
other channel counts and with an NTC channel.

## Wiring

*This applies to the original fugu PCB design with either ESP32 or ESP32-S3*

To use the internal ADC1 of the ESP32/ESP32-S3, omit or remove the ADS1015 or ADS1115 (U10) and the ALERT 10k pull-up
resistor (R31) next to it. Then use thin cables or isolated copper wire to connect the analog signals directly with
the ESP32 chip.

Select the internal ADC explicitly in `sensor.conf` (`adc = esp32adc1`, see the
[bare ESP32 example](../../reference/config/examples.md)). If the configured ADC fails to initialise, sensor setup
fails. There is no automatic fallback.

The following connections apply to each module.

### ESP32-S3-WROOM-1

```
A2 (solar current) ---> GPIO4 (ADC1_CHANNEL_3)
A3 (solar voltage) ---> GPIO5 (ADC1_CHANNEL_4)
A4 (bat voltage)   ---> GPIO6 (ADC1_CHANNEL_5) (orignally ADC-ALERT)
```

### ESP32-WROOM-32

```
A2 (solar current) ---> GPIO36 / SENSOR_VP (ADC1_CHANNEL_0)
A3 (solar voltage) ---> GPIO34 (ADC1_CHANNEL_6) (orignally ADC-ALERT)
A4 (bat voltage)   ---> GPIO39 / SENSOR_VN (ADC1_CHANNEL_3) 
```

## Sampling rate

The ESP32-S3 max ADC sampling rate according to
the [datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf#page=65) is 100
kHz, but esp-idf allows only 83.3 kHz (`SOC_ADC_SAMPLE_FREQ_THRES_HIGH` in `soc_caps.h`).
The ESP32 can do up to 2 MHz.

ADC1 runs in continuous mode (`adc_continuous_config`). The sampling rate and averaging are set by
`sensor.conf::esp32adc1_sr` and `esp32adc1_avg`. The shipped profiles mostly use 22000 Hz and 32, and some lab profiles
use 83333 Hz. The rate is shared by all channels in the conversion pattern, and averaging divides it further.

### Pattern and per-channel rate

Without an NTC channel on ADC1, each channel appears once per pattern. At 83333 Hz with averaging 32, three
channels give ~868 samples/s per channel and four give ~651.

The single-shot API can't be used while the ADC is in continuous mode, so a low-rate channel such as an NTC has to
join the continuous pattern. With an NTC channel on ADC1, the other channels are added a second time to the pattern
(if it fits). With *n* channels in total, each non-NTC channel gets
2 · SR / ((2n − 1) · avg) and the NTC channel SR / ((2n − 1) · avg). The pattern is logged at boot
(`pattern[i] = {...}`).

For example, with `ch3` as the NTC channel, the pattern is `ch0,ch1,ch2,ch3,ch0,ch1,ch2`. Putting all four channels
once in the pattern would reduce the bandwidth of all channels to 651sps (83333/4/32). The repeated pattern increases
the sampling rate of `ch0`,`ch1` and `ch2` to 744sps (83333/4/32*8/7). `ch3` is sampled with 372sps (83333/7/32).

### Measured rates

The ESP32-S3 reaches the theoretical rate. The ESP32 is a bit slower (511 / 625 @ 80 kHz, 639 /
781 @ 100 kHz) and always runs at ~80% of the calculated rate. This ratio is constant across the tested sampling
rates 80k, 100k, 125k, 128k, 156.25k, 160k, 200k, 250k, 312.5k, 320k and 400kHz.

At 640 kHz the ESP32 works at 70% of the expected sampling rate and with significant variance. This might be due to a
slow control loop.

### Latency

`conv_frame_size` sets the latency.
With frame size 128 bytes, 4 bytes/sample and SR=83.3kHz, latency is
expected to be 384µs (128 / 4 / 83333).

### Sources

The following links cover the ADC sampling rates:

* https://esp32.com/viewtopic.php?t=29554


* https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf#page=44
    * dig controller 2M SPS

* esp32s3
    * https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf#page=65: 100 kHz
    * according to `soc_caps.h`: 83333 Hz

The ESP32 appears to be the only one with 2Msps.

## Other notes

These noise observations come from two boards:

* Waveshare ESP32-S3 Mini (ESP32-S3-Zero) has no floating pin noise
* ESP32-S3-WROOM DevBoard has peak noise level of 100~120

The following commented-out defines list sampling-rate and averaging options:

```
//#define ADC1_SR 400000 // sampling rate (105k max, see https://www.esp32.com/viewtopic.php?t=1215)
// 50k, 64k, 80k, 100k, 125k, 128k, 156.25k, 160k, 200k, 250k, 312.5k, 320k, 400k, 500k, 625k, 640k, 800k
// https://www.wolframalpha.com/input?i=factor+%5B%2F%2Fmath%3A80000000%2F%2F%5D
//#define ADC1_AVG 64 // num averaging samples, max 256
```
