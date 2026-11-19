*this document is an LLM generated placeholder*

## S1 standard set: harvest % of GMPP (TRACKER line), 5 noise cases (noise-free, Gaussian 1-2, replay 0/8 s)
| scenario | board | LEDC mean | MCPWM mean | MCPWM-LEDC mean (se) | worst..best | MCPWM offset 0 - LEDC | trips LEDC / MCPWM | sweep-no-mpp LEDC / MCPWM | onset s LEDC / MCPWM | flag |
|---|---|---|---|---|---|---|---|---|---|---|
| wx-dawn | fry | 97.50 | 97.87 | +0.38 (0.32) | -0.2..+1.6 | +0.29 (0.23) | 0 / 0  | 88 / 89 | 0 / 0 |  |
| wx-dawn | flat | 98.21 | 98.25 | +0.04 (0.16) | -0.4..+0.5 |  | 0 / 0  | 89 / 91 | 0 / 0 |  |
| wx-dusk | fry | 98.52 | 97.84 | -0.68 (0.47) | -2.5..+0.0 | -0.24 (0.23) | 0 / 0  | 55 / 55 | 0 / 0 |  |
| wx-dusk | flat | 98.36 | 98.47 | +0.11 (0.32) | -0.7..+1.0 |  | 0 / 0  | 55 / 55 | 0 / 0 |  |
| wx-cloud | fry | 98.70 | 98.72 | +0.03 (0.03) | -0.1..+0.1 | -0.01 (0.04) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-cloud | flat | 98.71 | 98.78 | +0.07 (0.02) | +0.0..+0.1 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-cloud-cv | fry | 91.91 | 91.92 | +0.00 (0.03) | -0.1..+0.1 | -0.02 (0.07) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-cloud-cv | flat | 88.33 | 88.33 | -0.00 (0.05) | -0.1..+0.1 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-low | fry | 97.24 | 97.22 | -0.02 (0.34) | -0.8..+0.9 | -0.29 (0.67) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-low | flat | 97.95 | 97.55 | -0.40 (0.35) | -1.7..+0.3 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-low-slow | fry | 98.40 | 98.21 | -0.19 (0.18) | -0.8..+0.1 | -0.04 (0.21) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-low-slow | flat | 98.61 | 98.43 | -0.17 (0.14) | -0.5..+0.1 |  | 0 / 0  | 0 / 2 | 0 / 0 |  |
| wx-mid | fry | 99.74 | 99.74 | +0.00 (0.01) | -0.0..+0.0 | +0.01 (0.02) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-mid | flat | 99.77 | 99.75 | -0.02 (0.01) | -0.1..+0.0 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-calfail | fry | 97.05 | 96.23 | -0.82 (0.51) | -2.5..+0.5 | -0.31 (0.25) | 10 / 13 calib-fail | 0 / 0 | 0 / 0 |  |
| wx-calfail | flat | 98.57 | 98.35 | -0.22 (0.24) | -0.9..+0.5 |  | 6 / 6 calib-fail | 0 / 0 | 0 / 0 |  |
| wx-sweep-fall | fry | 93.29 | 93.37 | +0.08 (0.02) | +0.0..+0.1 | +0.07 (0.02) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-fall | flat | 93.46 | 93.54 | +0.08 (0.01) | +0.1..+0.1 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-rise | fry | 94.37 | 94.44 | +0.06 (0.03) | +0.0..+0.2 | +0.05 (0.03) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-rise | flat | 94.46 | 94.46 | +0.01 (0.02) | -0.1..+0.1 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-shade | fry | 98.42 | 98.44 | +0.02 (0.02) | -0.0..+0.1 | +0.03 (0.02) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-shade | flat | 98.38 | 98.41 | +0.03 (0.02) | -0.0..+0.1 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-gust | fry | 94.64 | 94.67 | +0.03 (0.01) | +0.0..+0.1 | +0.01 (0.02) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-gust | flat | 94.72 | 94.77 | +0.05 (0.02) | +0.0..+0.1 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-learn | fry | 97.22 | 97.29 | +0.07 (0.00) | +0.1..+0.1 | +0.07 (0.00) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-learn | flat | 97.21 | 97.34 | +0.13 (0.07) | +0.0..+0.4 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-learn-off | fry | 99.30 | 99.33 | +0.03 (0.01) | +0.0..+0.0 | +0.03 (0.00) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-learn-off | flat | 99.27 | 99.38 | +0.11 (0.06) | +0.0..+0.4 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-scan | fry | 99.30 | 99.31 | +0.01 (0.01) | -0.0..+0.0 | +0.02 (0.01) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sweep-scan | flat | 99.26 | 99.28 | +0.02 (0.01) | -0.0..+0.0 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade2 | fry | 99.46 | 99.48 | +0.02 (0.00) | +0.0..+0.0 | +0.03 (0.01) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade2 | flat | 99.50 | 99.51 | +0.01 (0.01) | -0.0..+0.0 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade3 | fry | 99.41 | 99.44 | +0.03 (0.01) | +0.0..+0.0 | +0.03 (0.01) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade3 | flat | 99.44 | 99.46 | +0.02 (0.01) | -0.0..+0.0 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade2-move | fry | 96.27 | 96.35 | +0.07 (0.02) | +0.0..+0.1 | +0.07 (0.02) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade2-move | flat | 96.32 | 96.36 | +0.04 (0.02) | -0.0..+0.1 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade3-move | fry | 75.72 | 75.71 | -0.01 (0.01) | -0.0..+0.0 | +0.01 (0.01) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade3-move | flat | 75.74 | 75.71 | -0.03 (0.01) | -0.1..+0.0 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade-walk | fry | 86.60 | 86.62 | +0.02 (0.01) | +0.0..+0.0 | +0.02 (0.00) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade-walk | flat | 86.60 | 86.62 | +0.02 (0.00) | +0.0..+0.0 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade-cloud | fry | 97.03 | 96.97 | -0.06 (0.11) | -0.4..+0.2 | -0.09 (0.12) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade-cloud | flat | 96.97 | 96.92 | -0.05 (0.12) | -0.4..+0.3 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade-learn | fry | 99.48 | 99.45 | -0.03 (0.03) | -0.1..+0.0 | -0.01 (0.01) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-shade-learn | flat | 99.44 | 99.52 | +0.07 (0.02) | +0.0..+0.1 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-mp-ramp | fry | 113.40 | 112.24 | -1.16 (1.33) | -6.4..+0.9 | -0.78 (1.11) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-mp-ramp | flat | 113.81 | 113.94 | +0.14 (0.15) | -0.1..+0.7 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-mp-psu | fry | 2.16 | 2.04 | -0.11 (0.09) | -0.5..+0.0 | -0.14 (0.08) | 3 / 3 minpulse-M | 42 / 42 | 0 / 0 |  |
| wx-mp-psu | flat | 0.59 | 0.59 | +0.00 (0.00) | +0.0..+0.0 |  | 1 / 1 minpulse-M | 71 / 71 | 0 / 0 |  |
| wx-mp-psu-nosink | fry | 2.17 | 2.20 | +0.03 (0.03) | -0.0..+0.1 | +0.01 (0.04) | 2 / 2 minpulse-M | 97 / 96 | 0 / 0 |  |
| wx-mp-psu-nosink | flat | 0.62 | 0.62 | +0.00 (0.00) | +0.0..+0.0 |  | 0 / 0  | 174 / 174 | 0 / 0 |  |
| wx-mp-manual | fry | 1.79 | 1.77 | -0.02 (0.00) | -0.0..-0.0 | -0.02 (0.00) | 8 / 9 minpulse-M | 0 / 0 | 0 / 0 |  |
| wx-mp-manual | flat | 2.95 | 2.95 | -0.00 (0.00) | -0.0..+0.0 |  | 10 / 10 minpulse-M | 0 / 0 | 0 / 0 |  |
| wx-mp-ccm | fry | 97.42 | 97.54 | +0.12 (0.06) | -0.0..+0.2 | +0.16 (0.05) | 5 / 5 minpulse-M | 0 / 0 | 0 / 0 | BEYOND |
| wx-mp-ccm | flat | 96.09 | 96.19 | +0.10 (0.08) | -0.1..+0.2 |  | 5 / 5 minpulse-M | 0 / 0 | 0 / 0 |  |
| wx-sun-fry | - | 99.88 | 99.89 | +0.01 (0.01) | -0.0..+0.0 | +0.01 (0.01) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-wob-fry | - | 99.58 | 99.77 | +0.19 (0.09) | -0.1..+0.4 | +0.09 (0.08) | 0 / 0  | 0 / 0 | 0 / 0 | BEYOND |
| wx-dusk-land-fry | - | 96.87 | 96.16 | -0.71 (0.68) | -2.5..+0.8 | -0.58 (0.75) | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-sun-flat | - | 99.82 | 99.80 | -0.02 (0.02) | -0.1..+0.0 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |
| wx-wob-flat | - | 99.86 | 99.82 | -0.04 (0.05) | -0.2..+0.1 |  | 0 / 0  | 0 / 0 | 0 / 0 |  |

## S2 dusk trap: wx-dusk SIM_VBAT=28, 4 boards x 4 readings x 9 noise cases (noise-free, Gaussian 1-4, replay 0/4/8/12)
| build | timing | n | mean harvest | trapped (< 96 %) | worst | trapped cells |
|---|---|---|---|---|---|---|
| pretrap | ledc | 144 | 97.98 | 3 | 85.0 | fry-hq/0.975:1/meas12:85.4 fry-hq/1:1.02/meas0:85.0 fry-hq/1:1.02/meas4:90.6 |
| pretrap | mcpwm | 144 | 98.13 | 0 | 96.5 |  |
| pretrap | mcpwm0 | 72 | 97.88 | 2 | 92.2 | fry/0.975:1/meas12:92.2 fry/1:1.02/meas12:94.0 |
| main | ledc | 144 | 98.25 | 0 | 96.9 |  |
| main | mcpwm | 144 | 98.13 | 0 | 96.5 |  |
| main | mcpwm0 | 72 | 98.09 | 0 | 97.0 |  |
| nodrain | ledc | 80 | 98.39 | 1 | 95.7 | fry/1:1.02/gauss1:95.7 |
| nodrain | mcpwm | 80 | 98.12 | 0 | 96.5 |  |
| nodrain | mcpwm0 | 40 | 98.23 | 1 | 95.7 | fry/1:1.02/clean0:95.7 |

Fix effect (main - pretrap), paired, per timing and board:
| timing | board | n | diff mean (se) | worst..best | < 96 % pretrap -> main |
|---|---|---|---|---|---|
| ledc | fry | 36 | +0.09 (0.05) | -0.8..+1.0 | 0 -> 0 |
| ledc | fry-hq | 36 | +0.99 (0.55) | -1.0..+13.5 | 3 -> 0 |
| ledc | flat | 36 | +0.00 (0.00) | +0.0..+0.0 | 0 -> 0 |
| ledc | flat-hq | 36 | +0.01 (0.01) | -0.3..+0.2 | 0 -> 0 |
| ledc | all | 144 | +0.27 (0.14) | -1.0..+13.5 | 3 -> 0 |
| mcpwm | fry | 36 | -0.00 (0.00) | -0.1..+0.0 | 0 -> 0 |
| mcpwm | fry-hq | 36 | +0.00 (0.00) | -0.1..+0.1 | 0 -> 0 |
| mcpwm | flat | 36 | +0.00 (0.00) | +0.0..+0.0 | 0 -> 0 |
| mcpwm | flat-hq | 36 | -0.01 (0.01) | -0.2..+0.1 | 0 -> 0 |
| mcpwm | all | 144 | -0.00 (0.00) | -0.2..+0.1 | 0 -> 0 |
| mcpwm0 | fry | 36 | +0.35 (0.21) | -0.9..+5.9 | 2 -> 0 |
| mcpwm0 | fry-hq | 36 | +0.06 (0.07) | -1.4..+0.9 | 0 -> 0 |
| mcpwm0 | all | 72 | +0.20 (0.11) | -1.4..+5.9 | 2 -> 0 |

Drain limit effect (main - nodrain), wx-dusk v28, 4 boards x 4 readings x 5 noise cases; wx-low fry/flat x 5:
| timing | set | n | diff mean (se) | worst..best | trips main / nodrain | min Iout main / nodrain |
|---|---|---|---|---|---|---|
| ledc | dusk exact | 20 | +0.00 (0.09) | -0.8..+1.0 | 0 / 0 | -0.165 / -0.165 |
| ledc | dusk Vout 0.975 | 20 | -0.55 (0.13) | -1.9..+0.5 | 0 / 14 | -0.179 / -0.181 |
| ledc | dusk fry* misread | 30 | -0.30 (0.17) | -1.9..+2.5 | 0 / 29 | -0.149 / -0.148 |
| ledc | dusk all | 80 | -0.15 (0.07) | -1.9..+2.5 | 0 / 29 | -0.179 / -0.181 |
| ledc | wx-low | 10 | -0.01 (0.01) | -0.0..+0.0 | | |
| mcpwm | dusk exact | 20 | +0.06 (0.12) | -0.5..+2.0 | 0 / 0 | -0.162 / -0.161 |
| mcpwm | dusk Vout 0.975 | 20 | -0.03 (0.07) | -0.9..+0.5 | 0 / 0 | -0.182 / -0.181 |
| mcpwm | dusk fry* misread | 30 | -0.12 (0.08) | -1.0..+0.5 | 0 / 0 | -0.148 / -0.148 |
| mcpwm | dusk all | 80 | -0.07 (0.05) | -1.0..+2.0 | 0 / 0 | -0.182 / -0.181 |
| mcpwm | wx-low | 10 | -0.01 (0.01) | -0.0..+0.0 | | |

## S3 min-pulse guard: main vs noguard, replay noise seeds 1-4 (SIM_NOISE_OFS 37 s steps), fry and flat
| scenario | timing | harvest noguard / guard | revI trips noguard / guard | on above onset [s] noguard / guard | minpulse-M stops | starts noguard / guard | Iout min noguard / guard |
|---|---|---|---|---|---|---|---|
| wx-dawn | ledc | 98.12 / 98.08 | 0 / 0 | 57 / 0 | 0 | 156 / 191 | -0.250 / -0.171 |
| wx-dawn | mcpwm | 98.29 / 98.20 | 0 / 0 | 56 / 0 | 0 | 159 / 194 | -0.263 / -0.165 |
| wx-dusk | ledc | 98.61 / 98.63 | 0 / 0 | 148 / 0 | 0 | 96 / 96 | -0.214 / -0.160 |
| wx-dusk | mcpwm | 98.27 / 98.29 | 0 / 0 | 148 / 0 | 0 | 96 / 96 | -0.242 / -0.150 |
| wx-low | ledc | 97.70 / 97.78 | 0 / 0 | 6 / 0 | 0 | 32 / 32 | -0.071 / -0.071 |
| wx-low | mcpwm | 97.65 / 97.31 | 0 / 0 | 5 / 0 | 0 | 32 / 31 | -0.071 / -0.071 |
| wx-mp-psu | ledc | 2.96 / 1.65 | 172 / 0 | 815 / 0 | 4 | 279 / 76 | -7.134 / -0.238 |
| wx-mp-psu | mcpwm | 2.96 / 1.64 | 157 / 0 | 834 / 0 | 4 | 265 / 74 | -7.056 / -0.241 |
| wx-mp-psu-nosink | ledc | 3.22 / 1.77 | 0 / 0 | 0 / 0 | 3 | 168 / 172 | -0.108 / -0.161 |
| wx-mp-psu-nosink | mcpwm | 3.23 / 1.70 | 0 / 0 | 0 / 0 | 1 | 168 / 170 | -0.108 / -0.160 |
| wx-mp-ramp | ledc | 116.43 / 113.60 | 0 / 0 | 0 / 0 | 0 | 8 / 8 | 0.000 / 0.000 |
| wx-mp-ramp | mcpwm | 115.67 / 113.66 | 0 / 0 | 0 / 0 | 0 | 8 / 8 | 0.000 / 0.000 |
| wx-mp-manual | ledc | 3.01 / 2.37 | 548 / 0 | 430 / 0 | 16 | 556 / 24 | -8.846 / -0.210 |
| wx-mp-manual | mcpwm | 3.00 / 2.36 | 548 / 0 | 422 / 0 | 16 | 556 / 24 | -8.897 / -0.209 |
| wx-mp-ccm | ledc | 96.51 / 96.46 | 72 / 0 | 107 / 0 | 8 | 72 / 8 | -4.885 / 0.000 |
| wx-mp-ccm | mcpwm | 96.60 / 96.52 | 72 / 0 | 108 / 0 | 8 | 72 / 8 | -4.482 / 0.000 |

## S5 peak scan: pv_substrings=6 minus default, 6 noise cases (Gaussian 1-3, replay 0/8/16)
| scenario | board | LEDC: base / scan - base (se) | MCPWM: base / scan - base (se) | scans/run LEDC / MCPWM |
|---|---|---|---|---|
| wx-cloud | fry | 98.75 / +0.20 (0.04) | 98.72 / +0.20 (0.05) | 5.8 / 5.8 |
| wx-cloud | flat | 98.75 / +0.23 (0.03) | 98.80 / +0.20 (0.04) | 6.0 / 5.8 |
| wx-shade-cloud | fry | 96.99 / -0.14 (0.08) | 96.83 / +0.01 (0.13) | 9.8 / 10.5 |
| wx-shade-cloud | flat | 96.90 / -0.07 (0.13) | 96.92 / -0.07 (0.06) | 10.5 / 10.7 |
| wx-shade-walk | fry | 86.60 / +10.27 (0.11) | 86.62 / +10.46 (0.07) | 8.0 / 7.2 |
| wx-shade-walk | flat | 86.61 / +10.35 (0.12) | 86.63 / +10.47 (0.06) | 7.8 / 7.2 |
| wx-shade2-move | fry | 96.28 / +2.78 (0.00) | 96.35 / +2.77 (0.01) | 5.0 / 5.0 |
| wx-shade2-move | flat | 96.33 / +2.79 (0.00) | 96.36 / +2.78 (0.00) | 5.0 / 5.0 |
| wx-shade3-move | fry | 75.73 / +23.61 (0.01) | 75.73 / +23.63 (0.01) | 7.0 / 7.0 |
| wx-shade3-move | flat | 75.74 / +23.61 (0.01) | 75.72 / +23.64 (0.01) | 7.0 / 7.0 |
| wx-sweep-shade | fry | 98.39 / +0.88 (0.00) | 98.43 / +0.87 (0.01) | 1.0 / 1.0 |
| wx-sweep-shade | flat | 98.39 / +0.87 (0.01) | 98.43 / +0.86 (0.01) | 1.0 / 1.0 |
| wx-shade2 | fry | 99.47 / +0.00 (0.00) | 99.48 / +0.00 (0.00) | 1.0 / 1.0 |
| wx-shade2 | flat | 99.50 / +0.00 (0.00) | 99.51 / +0.00 (0.00) | 1.0 / 1.0 |
| wx-shade3 | fry | 99.42 / +0.00 (0.00) | 99.45 / -0.01 (0.01) | 1.0 / 1.0 |
| wx-shade3 | flat | 99.45 / +0.00 (0.00) | 99.47 / +0.00 (0.00) | 1.0 / 1.0 |

## S6 settle window in slow mode, slow power filter (filter length 60, 200 Hz): main - oldsettle (51b3fa8 rule)
| scenario | board | noise | LEDC: old / main - old | MCPWM: old / main - old |
|---|---|---|---|---|
| wx-wob-fry | - | noise-free | 95.00 / +4.65 (0.00, +4.7..+4.7) | 95.20 / +4.62 (0.00, +4.6..+4.6) |
| wx-wob-fry | - | replay 0-12 | 99.64 / -0.02 (0.09, -0.2..+0.2) | 99.15 / +0.11 (0.39, -0.9..+0.9) |
| wx-wob-flat | - | noise-free | 98.43 / +1.02 (0.00, +1.0..+1.0) | 98.41 / +1.25 (0.00, +1.2..+1.2) |
| wx-wob-flat | - | replay 0-12 | 99.58 / +0.30 (0.07, +0.2..+0.5) | 99.55 / +0.24 (0.18, -0.3..+0.5) |
| wx-dusk | fry | noise-free | 96.70 / -0.18 (0.00, -0.2..-0.2) | 96.87 / +0.30 (0.00, +0.3..+0.3) |
| wx-dusk | fry | replay 0-12 | 96.91 / +0.94 (0.43, -0.0..+2.0) | 97.75 / +0.08 (0.23, -0.5..+0.6) |
| wx-dusk | flat | noise-free | 96.02 / +0.09 (0.00, +0.1..+0.1) | 96.79 / -0.84 (0.00, -0.8..-0.8) |
| wx-dusk | flat | replay 0-12 | 96.64 / +0.16 (0.40, -0.5..+1.3) | 96.91 / -0.06 (0.14, -0.5..+0.2) |

## S2b wx-dusk v28: timing - LEDC, paired
| build | board | timing | readings | diff |
|---|---|---|---|---|
| main | fry | mcpwm | exact | n=9 mean -0.36 se 0.33 [-2.5..+1.1] |
| main | fry | mcpwm | misread | n=27 mean -0.23 se 0.07 [-1.0..+0.5] |
| main | fry | mcpwm | all | n=36 mean -0.26 se 0.10 [-2.5..+1.1] |
| main | fry | mcpwm0 | exact | n=9 mean -0.28 se 0.17 [-0.7..+0.6] |
| main | fry | mcpwm0 | misread | n=27 mean -0.20 se 0.09 [-1.7..+0.7] |
| main | fry | mcpwm0 | all | n=36 mean -0.22 se 0.08 [-1.7..+0.7] |
| main | fry-hq | mcpwm | exact | n=9 mean -0.09 se 0.11 [-0.6..+0.3] |
| main | fry-hq | mcpwm | misread | n=27 mean -0.19 se 0.10 [-1.8..+0.6] |
| main | fry-hq | mcpwm | all | n=36 mean -0.16 se 0.08 [-1.8..+0.6] |
| main | fry-hq | mcpwm0 | exact | n=9 mean -0.14 se 0.14 [-1.1..+0.4] |
| main | fry-hq | mcpwm0 | misread | n=27 mean -0.10 se 0.10 [-1.0..+1.0] |
| main | fry-hq | mcpwm0 | all | n=36 mean -0.11 se 0.08 [-1.1..+1.0] |
| main | flat | mcpwm | exact | n=9 mean +0.10 se 0.19 [-0.7..+1.0] |
| main | flat | mcpwm | misread | n=27 mean -0.04 se 0.10 [-0.9..+0.9] |
| main | flat | mcpwm | all | n=36 mean -0.00 se 0.09 [-0.9..+1.0] |
| main | flat-hq | mcpwm | exact | n=9 mean -0.16 se 0.10 [-0.7..+0.2] |
| main | flat-hq | mcpwm | misread | n=27 mean -0.06 se 0.06 [-0.7..+0.6] |
| main | flat-hq | mcpwm | all | n=36 mean -0.09 se 0.05 [-0.7..+0.6] |
| pretrap | fry | mcpwm | exact | n=9 mean -0.36 se 0.33 [-2.5..+1.1] |
| pretrap | fry | mcpwm | misread | n=27 mean -0.10 se 0.08 [-0.7..+0.9] |
| pretrap | fry | mcpwm | all | n=36 mean -0.16 se 0.10 [-2.5..+1.1] |
| pretrap | fry | mcpwm0 | exact | n=9 mean -0.28 se 0.17 [-0.7..+0.6] |
| pretrap | fry | mcpwm0 | misread | n=27 mean -0.53 se 0.27 [-6.2..+0.9] |
| pretrap | fry | mcpwm0 | all | n=36 mean -0.47 se 0.21 [-6.2..+0.9] |
| pretrap | fry-hq | mcpwm | exact | n=9 mean -0.09 se 0.11 [-0.6..+0.3] |
| pretrap | fry-hq | mcpwm | misread | n=27 mean +1.13 se 0.74 [-0.9..+13.5] |
| pretrap | fry-hq | mcpwm | all | n=36 mean +0.83 se 0.56 [-0.9..+13.5] |
| pretrap | fry-hq | mcpwm0 | exact | n=9 mean -0.11 se 0.15 [-1.1..+0.4] |
| pretrap | fry-hq | mcpwm0 | misread | n=27 mean +1.13 se 0.73 [-1.5..+13.4] |
| pretrap | fry-hq | mcpwm0 | all | n=36 mean +0.82 se 0.55 [-1.5..+13.4] |
| pretrap | flat | mcpwm | exact | n=9 mean +0.10 se 0.19 [-0.7..+1.0] |
| pretrap | flat | mcpwm | misread | n=27 mean -0.04 se 0.10 [-0.9..+0.9] |
| pretrap | flat | mcpwm | all | n=36 mean -0.00 se 0.09 [-0.9..+1.0] |
| pretrap | flat-hq | mcpwm | exact | n=9 mean -0.10 se 0.10 [-0.7..+0.2] |
| pretrap | flat-hq | mcpwm | misread | n=27 mean -0.06 se 0.06 [-0.7..+0.6] |
| pretrap | flat-hq | mcpwm | all | n=36 mean -0.07 se 0.05 [-0.7..+0.6] |

## drain per board (main - nodrain)
ledc fry n=20 mean -0.03 se 0.19 [-1.4..+2.5] {'sweep-no-mpp': 176, 'minpulse-M': 15}
ledc fry-hq n=20 mean -0.39 se 0.18 [-1.9..+1.7] {'sweep-no-mpp': 196, 'minpulse-M': 14}
ledc flat n=20 mean -0.12 se 0.09 [-0.8..+0.7] {'sweep-no-mpp': 224}
ledc flat-hq n=20 mean -0.06 se 0.08 [-1.1..+0.4] {'sweep-no-mpp': 220}
mcpwm fry n=20 mean -0.09 se 0.09 [-1.0..+0.5] {'sweep-no-mpp': 220}
mcpwm fry-hq n=20 mean -0.08 se 0.08 [-0.9..+0.4] {'sweep-no-mpp': 219}
mcpwm flat n=20 mean -0.01 se 0.14 [-0.8..+2.0] {'sweep-no-mpp': 224}
mcpwm flat-hq n=20 mean -0.08 se 0.06 [-0.5..+0.3] {'sweep-no-mpp': 225}

## drain at MCPWM offset 0 (main - nodrain)
fry exact n=5 mean +0.23 se 0.22 [-0.3..+0.8] nodrain {'sweep-no-mpp': 55} main {'sweep-no-mpp': 55}
fry misread n=15 mean -0.12 se 0.27 [-2.0..+2.4] nodrain {'sweep-no-mpp': 136, 'minpulse-M': 10} main {'sweep-no-mpp': 165}
fry-hq exact n=5 mean +0.22 se 0.10 [-0.1..+0.5] nodrain {'sweep-no-mpp': 54} main {'sweep-no-mpp': 55}
fry-hq misread n=15 mean -0.22 se 0.18 [-1.2..+1.5] nodrain {'sweep-no-mpp': 157, 'minpulse-M': 9} main {'sweep-no-mpp': 165}
