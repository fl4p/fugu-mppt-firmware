---
name: project_coil_and_ringing
description: Coil L per board (fry ~80 µH KS184, flat ~51 µH KS130), on-device measure-coil, DCM ring per-board (flu 320 kHz) and why LS-pulse damping fails, OV overshoot ~V0+I·√(L/C)
metadata:
  type: project
---

## fry vs flat coil inductance
2026-05-22, resolved. DCM sweeps: flat 50.9 µH, fry 79.8 µH. Different coils, not a fry INA226 error (that earlier claim is withdrawn). flat: 2× KS130-060A, ~20-21 t, nameplate 48.8-53.8 µH (validates the DCM method). fry: 2× KS184-125A; the documented 56 µH (10 t) is suspect, true L ≈ 80 µH. BMS-shunt regression: g_fry ≈ 1.0 ±0.1, g_flat ≈ 1.06-1.07 (flat under-reports ~6-7 %).
- 2026-11-15 (newer): fry 76.0 µH (4 runs within 0.35 µH; = coil.conf 80e-6 × 0.95), flat ~51 µH (coil.conf still 56e-6). A flat 59.5 µH outlier was a bad Iout zero (+0.12 A) read 0.1 s after a stop from 22 A.
- `measure-coil bias` (DC-bias curve, branch coil-lowfreq 6324ab1) is parked until a bench converter exists; plans/bench-converter-work.md.
- **How to apply:** coil.conf L0 = each board's measured L (flat ≈ 51e-6, fry 80e-6, which the 5 % DC-bias undershoot turns into 76 µH). The ±10-16 % L wiggle is SR-timing reverse current, not saturation. doc/Coil Inductance Measurement.md §7.

## On-device measure-coil
`measure-coil l0|ls [steps|hs] [dwell_ms] [apply]`, `src/measure_coil.{h,cpp}` (core-0 task, in MAIN_SRC only), DCM relation `L=(Vin-Vout)·Vin·D²/(2·Vout·fsw·Iout)`. Validated on flat 2026-05-22: 50.34 µH vs host `measure_coil.py` 49.69 µH. Mock-ADC bench results (fugu139C) are artifacts; plain dwell (3000 ms) + 5-read median is enough. E2E parity test `etc/e2e-test/test_measure_coil.py`.
- BF-closed L0 sweep (de63e98a) HW-validated on flat 2026-11-24 (reverse_current_paranoia=1 temporarily): no highD-bflowOpen, 49.74 µH ring fit, 35 DCM pts.
- 11-24 d8a2d358 (untested on HW): in the fleet build (+20.7 KB, ~90 KB left in the OTA slot); starts only from `dc 0`, ends there, `mppt` resumes; mode-changing commands refused while running.

## DCM ring
Per-board: f_r is set by C_sw, i.e. how many FETs are paralleled. **flu measured 320 kHz**, C_sw ~3.0 nF, T_ring/4 ~780 ns (2026-09-06, `pv/ee/plans/RESULTS-flu-csw-ring-20260906.md`). fry/flat: ~250 kHz fry / ~325 kHz flat, C ~5 nF (11-14/15 inference from coil-measure oscillation and the rect-cal knee, not a direct scope measurement; supersedes the old unverified 1.3-2 MHz estimate); fbuck unmeasured (old 3.34 nF retracted). Never time anything off a constant ring period. Canonical: website/docs/internals/dcm-ringing.mdx.
- **2026-11-18 (ideal-model analysis): LS-pulse active damping cannot work.** The ring circles V_out with amplitude V_out; pinning the node to 0 V never shrinks it, a T_ring/4 pulse grows it (27 -> 39-51 V), a valley pulse holds or grows it; rect_offset reverse current also enlarges it. MCPWM has no spare comparator anyway (2 per operator, both used).
- What helps: RC snubber at the SW node (size from measured f_r and C_ext halving it, R = √(L_par/C_par), C 3-5× C_par, loss ½CV²f), higher R_g, parallel Schottky, smaller commutation loop, or avoiding DCM (forced CCM `sync forced`/`forced_pwm=1`, burst mode). Valley switching only cuts HS turn-on loss. Sources: ADI "The Unseen Ring", Specter Engineering "Switch Node Ringing", TI slyt465.

## Open-battery overshoot vs L
2026-11-18. Battery-open Vout peak ≈ V0 + I·√(L/C): 34.7-41.4 V for L 28-100 µH, Cout 470/560 µF (8S LFP, ~28 A, Vin 64 V); 200 Hz vs 5 kHz trip ≤2.5 V apart. Earlier 55-66 V were vconv forward-Euler artifacts, fixed fb64bb1a, d590a26d (Fugu2#36 has a correction).
- **Why:** a faster HW OV trip cannot beat the coil-energy peak; only absorbing ~10-40 mJ or more Cout helps. ADC-monitor prototype on hold.
- **How to apply:** distrust light-load/open-output vconv results before fb64bb1a. Open review items: boost lossless, lK_ only on slopes, float sun clock. See [[project_vconv_builds]].
