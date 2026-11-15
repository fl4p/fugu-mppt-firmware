---
name: dcm-switch-node-ring
description: DCM Iout ripple vs duty is the L-C_node ring (not SR-timing notch); field f/C/Q per board, period law, mean biases, estimator
metadata:
  type: project
---

DCM sweeps show +-10..25 % Iout ripple periodic in duty count: N = Vout/(f_r*t_tick*Vin), coordinate is the
dead time t_d = T(1 - D*Vin/Vout). Fitted 2026-11-14 from field global sweeps: fry 248-257 kHz, Q~25,
C~4.9 nF (L 80 uH); flat 324-330 kHz, Q~15, C~4.7 nF (L 51 uH). Old doc blamed the SR reverse-current notch:
wrong, DCM LS turns off ~2 % early (rectCtrlRatio with 0.99/1.01 margins).

Mean biases on L from the DCM relation (inferred, not scoped): node charge -C*Vout*fsw (~-5 mA, L high);
HS turn-off transition delivers 1/2 C Vin^2 per cycle to output = +C*Vin^2*fsw/(2*Vout) (~+17 mA at 70 V,
L LOW 1.5-6 %, NOT in vconv plant); gate delay delta: dL/L = -2*delta/(D*T).

Estimator: fit 1/L_i = c0 + 2 harmonics in f*t_d, scan f; src/selftest/coil_fit.h == etc/measure_coil.py fit_l0.

**How to apply:** any L0/DCM-current work: expect phase-dependent Iout; never trust a median over < 1 ring
period. Settling test = scope node + coil current at fixed duty stepping HS through one period N.
See [[vconv-reverse-pump-energy-leak]].
