---
title: "Internal ADC"
sidebar_position: 3
---

# Internal ADC

## Poor ADC ?

The ESP32 internal ADC has a poor reputation.
It suffers from non-linearity and is known to be noisy. But is it really that bad?

The linearity can be fixed with calibration and we keep a margin to the ADC voltage range, like Espressif suggests
(e.g. at 6dB PGA attenuation 150 mV ~ 1750 mV,
see [suggested range](https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#_CPPv425adc1_config_channel_atten14adc1_channel_t11adc_atten_t))
For simple curve fit see [here](https://github.com/espressif/esp-idf/issues/164#issuecomment-318861287).
This can be improved with a look-up-table (LUT).

About the noise issue, we should take a closer look how we take measurements.
A simple and naive way is to set up a periodic timer and take a single-shot measurement with Arduino API `analogRead()`,
expecting a 12-bit precision. But what about conversion time? The shorter the AD conversion time, the higher the noise.
We didn't tell the ADC for how long to measure. If we measure the time `analogRead` takes until return, its only a
couple 10 microseconds. Digging through the code, we find that it calls `adc_oneshot_read`, `adc_oneshot_hal_convert`.
The ADC characteristics in the ESP32-S3 data sheet show 100ksps (for the ESP32 even 2Msps) sampling rate with the DIG
controller. So we can assume that
measurements are taken with sub-µs time, which explains the noise issue quite well.

The more time the ADC spends for conversion, the better the results get. Let's assume a measurement that takes 100µs and
we take it every 100ms. This will give us an effective ADC duty cycle of 0.1% (1/1000). So 99.9% of the time the ADC
doesn't
do anything.

To get most of the ADC performance we start a continuous measurements at maximum sampling
rate (`SOC_ADC_SAMPLE_FREQ_THRES_HIGH`) and average the samples so we get the
desired sampling rate. Averaging should be done with the raw adc int16 readings for best performance.

For the ESP32-S3, a good configuration appears to be 83.3kHz SR and 32 samples averaging. With 3 channesl this gives an
per-channel rate of ~ 868 sps (=83333 / 32 / 3).

We cannot use the single-shot measurement API when the ADC is in continuous mode. So when adding another channel that
has low requirements (e.g. an NTC), we need to add it the continous reading pattern, which will reduce bandwidth of
all channels to 651sps (83333/4/32). We can improve this by duplicating the pattern without the NTC channel.

For example `ch0,ch1,ch2,ch3,ch0,ch1,ch2`, where `ch3` is the NTC channel.
This increases sampling rate of `ch0`,`ch1` and `ch2` to 744sps (83333/4/32*8/7). `ch3` is sampled with 372sps (
83333/7/32).

## Wiring

*This applies to the original fugu PCB design with either ESP32 or ESP32-S3*

To use the internal ADC1 of the ESP32/ESP32-S3, just omit ADS1015 or ADS1115 (U10) and the ALERT 10k pull-up resistor
(R31).

Select the internal ADC explicitly in `sensor.conf` (`adc = esp32adc1`, see the
[bare ESP32 example](../../reference/config/examples.md)). There is no automatic fallback: if the
configured ADC fails to initialise, sensor setup fails.

Remove the ADS chip and the pull-up resistor next to it. Then use thin cables or isolated copper wire to
connect the analog signals directly with the ESP32 chip.

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

## Sampling Rate

ESP32-S3 max ADC sampling rate according to
the [datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf#page=65) is 100
kHz, however only 83.3 kHz is possible with
esp-idf (`SOC_ADC_SAMPLE_FREQ_THRES_HIGH` in `soc_caps.h`).
ESP32 can do up to 2 MHz.

ADC1 runs in continuous mode (`adc_continuous_config`). The sampling rate and averaging are set by
`sensor.conf::esp32adc1_sr` and `esp32adc1_avg`; the shipped profiles mostly use 22000 Hz and 32, some lab profiles
83333 Hz. The rate is shared by all channels in the conversion pattern, and averaging divides it further.

Without an NTC channel on ADC1, each channel appears once per pattern: at 83333 Hz with averaging 32, three
channels give ~868 samples/s per channel and four give ~651. With an NTC channel on ADC1, the other channels are
added a second time to the pattern (if it fits), so with *n* channels in total each non-NTC channel gets
2 · SR / ((2n − 1) · avg) and the NTC channel SR / ((2n − 1) · avg). The pattern is logged at boot
(`pattern[i] = {...}`).

On the ESP32-S3 we can achieve this theoretic rate, the ESP32 appears to be a bit slower (511 / 625 @ 80 kHz, 639 /
781 @ 100khz ) and always at ~80% of the calculated rate. This ratio is constant across tested sampling rates 80k, 100k,
125k, 128k, 156.25k, 160k, 200k, 250k, 312.5k, 320k and 400kHz.

At 640 kHz the ESP32 works at 70% of expected sampling rate and with significant variance. This might be due to a slow
control loop.

Latency can be configured through `conv_frame_size`.
with frame size 128 bytes, 4 bytes/sample, SR=83.3kHz, latency is
expected to be 384µs (128 / 4 / 83333).

* https://esp32.com/viewtopic.php?t=29554


* https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf#page=44
    * dig controller 2M SPS

* esp32s3
    * no info in
      datasheet? https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf#page=38
    * according to `scop_caps.h`: 83333 Hz

looks like esp32 is the only one with 2Msps

## Other Notes

* Waveshare ESP32-S3 Mini (ESP32-S3-Zero) does have no floating pin noise
* ESP32-S3-WROOM DevBoard has peak noise level of 100~120


```
//#define ADC1_SR 400000 // sampling rate (105k max, see https://www.esp32.com/viewtopic.php?t=1215)
// 50k, 64k, 80k, 100k, 125k, 128k, 156.25k, 160k, 200k, 250k, 312.5k, 320k, 400k, 500k, 625k, 640k, 800k
// https://www.wolframalpha.com/input?i=factor+%5B%2F%2Fmath%3A80000000%2F%2F%5D
//#define ADC1_AVG 64 // num averaging samples, max 256
```