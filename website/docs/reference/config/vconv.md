---
title: vconv.conf
sidebar_position: 18
---

# vconv.conf

`vconv.conf` configures the virtual converter plant, which replaces the power stage and ADC with a simulated PV source, battery, and passives. It applies to `CONFIG_FUGU_WITH_VCONV` builds with `sensor.conf` `adc = vconv`. `src/adc/sensor_setup.cpp` and `src/adc/vconv.h` read it. For an example, see [`config/lab/vconv_mock`](../../../../config/lab/vconv_mock/conf/vconv.conf).

The simulation takes the inductor from `coil.conf::L0` and the topology from `converter.conf::topo` (`buck`/`boost`). This file doesn't set either.

The file accepts the following keys.

| key             | unit | type  | default | description                                                                 |
|-----------------|------|-------|---------|-----------------------------------------------------------------------------|
| `isc`           | A    | float | 8.0     | PV short-circuit current                                                    |
| `voc`           | V    | float | 40.0    | PV open-circuit voltage                                                     |
| `pv_k`          |      | float | 0.8     | V_mpp / Voc                                                                 |
| `v_bat`         | V    | float | 28.0    | Battery sink voltage (`v_bat=0`, `r_bat=1e9` ≈ open output)                 |
| `r_bat`         | Ω    | float | 0.05    | Battery series resistance                                                   |
| `vbat_ac_amp`   | V    | float | 0.0     | Ripple amplitude (peak) on V_bat; 0 = off                                   |
| `vbat_ac_freq`  | Hz   | float | 100     | Ripple frequency                                                            |
| `vbat_ac_shape` |      | int   | 0       | 0 = sine (inverter), 1 = full-wave \|sin\| (rectifier load), 2 = spiky pulse |
| `c_in`          | F    | float | 470e-6  | Input capacitance                                                           |
| `c_out`         | F    | float | 470e-6  | Output capacitance                                                          |
| `adc_freq`      | Hz   | int   | 3000    | Sampling tick (falls back to `board.conf::adc_fake_freq`, then 3000)        |
| `noise_vin`     | V    | float | 0       | Gaussian ADC noise σ on Vin; 0 = deterministic                              |
| `noise_vout`    | V    | float | 0       | Gaussian ADC noise σ on Vout                                                |
| `noise_iout`    | A    | float | 0       | Gaussian ADC noise σ on Iout                                                |
| `noise_ntc`     |      | float | 0       | Gaussian ADC noise σ on the NTC channel                                     |
| `ntc_v`         |      | float | 0.9     | Constant NTC channel value (25 °C equivalent)                               |
