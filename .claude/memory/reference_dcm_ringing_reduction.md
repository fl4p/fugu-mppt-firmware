---
name: DCM coil ringing reduction techniques and sources
description: Practical techniques to reduce inductor/switch-node ringing in a buck converter in DCM, with key source URLs and RC snubber design procedure
created: 2026-08-11T10:01:27.372Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: reference
  originSessionId: ses_00fbc8735ffeVJ7MPeNM59Pwo3
---

Research gathered 2026-08-11 on reducing coil (inductor) ringing in a DC-DC buck converter in DCM. Directly relevant to this project's synchronous buck with diode emulation (src/buck.h). Folded into doc/DCM Ringing.md (companion to doc/Diode Emulation.md).

## Mechanism
In DCM, when inductor current reaches zero, the LS rectifier turns off. The residual energy in the inductor bounces between L and the parasitic capacitance at the switch node (MOSFET C_oss + diode capacitance), forming an LC tank that rings at a frequency much higher than f_sw. The ringing couples EMI to the output and stresses the FET.

## Reduction techniques (passive → active → firmware)

1. **RC snubber across switch node to GND** (most common): Series R+C from SW node to GND, placed as close to the LS FET/diode as possible.
   - Design procedure (from ADI + Specter Engineering + SE post):
     a. Measure ringing frequency f_r on scope (bandwidth limit OFF, short ground spring).
     b. Add capacitance C_ext from SW to GND until f_r drops to ~half. Then C_parasitic = C_ext / 3.
     c. Calculate L_parasitic = 1 / (4π²·f_r²·C_parasitic).
     d. R_snubber = sqrt(L_parasitic / C_parasitic) (characteristic impedance).
     e. C_snubber = 3–5× C_parasitic (larger C = more damping but more loss: P = ½·C·V²·f_sw).
   - Practical note from SE post (scanny): calculations get you close; solder in a pot and tune R for best waveform. τ(RC) should be >3× ringing rise time. Final values for an LM2576 buck: 1nF + 470Ω.
   - For nonsynchronous bucks, the datasheet may say ringing is harmless (voltage never exceeds switching voltage). It's mainly an EMI concern.

2. **Active damping via LS FET** (firmware, specific to synchronous buck): Keep LS FET on briefly after ZCD to clamp SW node to ground, dissipating residual energy in R_ds(on). Most promising no-HW-change approach for this project — extends existing minLS/boot_refresh_ns mechanism in buck.h. Also: briefly re-turn LS FET on for a few ns at zero-crossing (active clamp, US8933635B2 patent).

3. **Gate resistance**: Increase R_g to slow dv/dt and di/dt, reducing excitation of the parasitic LC tank. Trade-off: higher switching loss. Near-linear overshoot reduction vs R_g (Specter Engineering: 1–20 Ω sweep).

4. **Schottky diode in parallel with LS FET body diode**: Near-zero reverse recovery → reduces the current step that excites the RLC tank. Simulation showed ~90V overshoot reduction (Specter Engineering).

5. **Layout optimization**: Minimize the high-di/dt commutation loop area (input cap → HS FET → LS FET/diode). ~10 nH per 25 mm of trace. Planar/laminated busbars or adjacent PCB power layers.

6. **Forced CCM** (firmware): Keep LS FET on at zero current (negative inductor current) or add minimum preload to stay in CCM. Eliminates DCM ringing but circulating current loss at light load.

7. **Pulse skipping / burst mode** (firmware): Skip PWM pulses at light load; fire CCM bursts when V_out droops. Avoids DCM ringing entirely. Trade-off: burst-frequency output ripple.

## Key sources
- Analog Devices: "The Unseen Ring: Taming Parasitics in Buck Converters Using a Snubber" (2026-03-16) — https://www.analog.com/en/resources/technical-articles/the-unseen-ring.html — full snubber calc + LTspice method
- Specter Engineering: "Switch Node Ringing" (2019-10-04) — https://www.specterengineering.com/blog/2019/9/26/switch-node-ringing — first-principles RLC analysis, gate-R sweep, Schottky parallel, snubber
- SE: "Snubbing DCM (nonsynchronous) Buck converter" — https://electronics.stackexchange.com/questions/246301/ — practical tuning outcome (1nF + 470Ω on LM2576), "snubbing improves waveforms, not makes them perfect"
- TI: "Controlling switch-node ringing in synchronous buck converters" — https://www.ti.com/lit/pdf/slyt465
- ResearchGate: "Impact of inductor current ringing in DCM on output voltage of DC-DC buck power converters" (2017) — https://www.researchgate.net/publication/317525001
- US Patent 8933635B2: active clamp for LED driver DCM ringing — https://patents.google.com/patent/US8933635B2/en
