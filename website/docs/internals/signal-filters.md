---
title: "Signal Filters"
sidebar_position: 5
---

# Signal Filters

## Noise sources

Noise can degrade tracking performance significantly.

The ADC itself is the most obvious noise source. You can easily reduce this noise by increasing the ADC conversion time
and sample averaging.

The loads connected to the battery are another significant noise source. The higher the impedance of the battery and
wiring, the greater the noise. To reduce noise coupling from the loads to the charger, connect the MPPT charger and the
loads separately, as close as possible to the battery terminals. Keep in mind that high-frequency noise can propagate
more easily due to wire inductance, even with very thick cables.

Laptops can have complex noise spectra that are difficult to describe. A sliding median filter and an averaging filter
work well here.

A 50 Hz inverter puts a 100 Hz ac component on the input, because the input sees `abs(sin(a*t))`. A notch filter can
remove this quite well. The [inverter-ripple notch](#inverter-ripple-notch) section describes the firmware's notch.

See also: [Krampmeier/uncertainty](https://github.com/Krampmeier/uncertainty).

## Median filter

A median filter removes spikes. It can filter load bursts.

## Residual noise

IIR filters have a lower memory footprint and run faster than FIR filters. Their disadvantage is a nonlinear phase
response. The firmware measures only the dc amplitude and doesn't use phase information, so IIR filters are a good
choice. For background, see [FIR and IIR filters (NI DIAdem)](https://www.ni.com/docs/de-DE/bundle/diadem/page/genmaths/genmaths/calc_filterfir_iir.htm).

The following techniques address residual noise:

* use IIR filters
* use multipass filters
* adapt the window length to a given target noise

The following plots show the charger output current, sampled by the INA226 ADC (t_conv=1140µs, avg=1). The charger was
connected to an LFP battery and a cheap Chinese inverter that produces heavy burst noise. The sinusoidal inverter input
adds a slight 100 Hz ac component, double the frequency of the 50Hz 220V output. The useful signal has a triangular
waveform, which comes from the MPPT perturbation.

The first plot shows the noisy signal (blue), a moving average with N=80 (orange), and a 2-pass moving average with N=40:

![Noisy output current with moving averages](img/noise1.webp)

The 2-pass filter has much better noise rejection:

![2-pass moving average](img/noise_2p.webp)

The next plot shows a single IIR filter:

![IIR filter](img/noiseIir.webp)

The last plot shows a 2-pass IIR filter:

![2-pass IIR filter](img/noiseIIR2.webp)

## Inverter-ripple notch

An inverter on the DC bus draws ripple at twice the mains frequency (100 Hz for 50 Hz, 120 Hz for 60 Hz), which
corrupts the MPPT power estimate. Each physical sensor whose own sample rate allows it (f0 < 0.45·fs) gets an IIR
notch (biquad) at that frequency. Slower channels get none, and the notch settings have no effect on them.

By default, the notch tunes itself (`sensor.conf::notch_adaptive=1`):

- A ripple detector watches `Vout` and scans 80–140 Hz over blocks of about 0.75 s.
- An estimate is used only if its peak-to-mean ratio (SNR) is at least 15.
- The notch moves halfway toward each accepted estimate, and only when that step exceeds 0.3 Hz, so one noisy
  block can't pull it away.
- Detection and retuning run on the RT core, so filter coefficients never change under a running filter.

With `notch_adaptive=0`, the notch stays at `notch_freq` (default 100 Hz). `notch_q` sets the bandwidth. See
[`sensor.conf`](../reference/config/sensor.md).

Background: [notch filters (video)](https://www.youtube.com/watch?v=tpAA5eUb6eo).
