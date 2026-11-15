---
name: project-ro-tracker-step-not-viable
description: "Issue #47 field result (2026-11-08) — BMS wire fit Rc works on fry (~15 mΩ, stable); tracker-step Ro estimator is not viable (SE ~200 mΩ, flat biased)"
metadata:
  node_type: memory
  type: project
  originSessionId: e30d4973-fdd8-4b7f-a285-59288d26932b
  modified: 2026-11-12T13:19:47.547Z
---

84a48eb OTA'd to fry/flat 2026-11-04. Field result after 4 days:
- `Rc`/`Rb` (BMS wire fit): fry publishes from 11-06, Rc 14.1–15.2 mΩ, Rb 1.3–1.9 mΩ, stable across days. flat stays blocked (switching Vout error), as intended.
- `Ro` (IV fit on tracker duty steps): fry never publishes (corr² median 0.007, gate 0.25); flat publishes garbage (−1080…+1560 mΩ) because its Iout barely responds to duty (−0.2 mA/count) while its Vout error tracks duty (1.3 mV/count).
- An offline replay of 3 h midday telemetry gives a typical SE of 226 mΩ (fry) and 1160 mΩ (flat) with a 256-step window. MPP tracker steps carry almost no ΔI, so no gate rescues it.

**Why:** a 1-count step at the MPP moves Iout by only ~8 mA → ΔV ~0.1 mV, far below noise and duty-correlated sensing artifacts.
**Replaced 2026-11-09 by 5edefd4:** Ro is now a per-sweep Vout-vs-Iout line fit (span ≥3 A, residual ≤40 mV, SE ≤3 mΩ). Replay: fry 20/31 sweeps, ~23.7 mΩ; flat 0/41. Deployed via integ-ota 3e0539c (11-11). FIELD-VALIDATED 11-12: fry 7/7 sweeps ≥3 A ok, 21.2–23.95 mΩ, SE 0.45–1.1, res 9–27 mV; flat 0/11 (res 109–295 mV). Per-sweep log line "Ro fit ok/rejected" in havan pv/fugu_console.log.
**flat root cause (Fab, 2026-11-10):** flat lacks the filter cap on the Vout sense divider (fry has it); recorded in ~/dev/ee/hw/Fugu2/HW Stories/flat.md. Once fitted, Ro/Rc should publish on flat without firmware changes. Software fix ruled out 11-12: ina22x_conv_time_us 1100→2116 on flat changed neither the sweep residual (227 mV) nor the offset (~−0.3 V) → clipping, not aliasing; settings restored. Note 8244 µs would trip the loop-rate watchdog (expected_hz=200).
**How to apply:** don't revive the tracker-step estimator; either drop Ro or design deliberate excitation (needs Fab's OK on live converters). The wire check for #47 should build on Rc. Related: [[project-charger-shared-bms-and-flat-vout-cal]].
