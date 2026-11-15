---
name: project-out-impedance-exogenous-bias
description: OutputImpedance (src/out_impedance.h, issue #47) dV/dI regression is biased negative by bus disturbances and toward 0 by Iout noise; reviewed 2026-10-08
metadata:
  type: project
---

Review of the telemetry-only Ro estimator (2026-10-08, uncommitted at the time).

- Under fixed duty a buck fed by PV has bus-side small-signal conductance dIout/dVout ≈ g_pv/D² (g_pv = dIpv/dVin < 0),
  ~ -1.35 S at 27 V/76 V/13 A. Exogenous bus steps (inverter load, other converter) therefore produce dV/dI ≈ -0.7 Ω
  samples. Measured in /tmp vconv experiment: 0.5 V bus steps at 2%/100 ms took a 20 mΩ estimate to ~4 mΩ; without P&O to -14 mΩ.
- Iout noise passing the |dI| gate attenuates toward 0 (0.6 A/sample noise, no excitation -> ~0.1 mΩ with 3000 "steps") — reads as a healthy wire.
- Fix direction: instrumental variable on own duty change, R = Σ dV·dD / Σ dI·dD, restricted to tracker (non-limiting) steps.
- One NaN Vout block poisons sxy permanently; a gap without reset (manual mode skips update()) injects one huge bogus step.

**Why:** shared battery bus (fry+flat) and inverter loads make exogenous dV the norm, not the exception.
**How to apply:** any dV/dI-from-operating-data estimator on this bus needs an exogenous instrument, not just a |dI| gate.
