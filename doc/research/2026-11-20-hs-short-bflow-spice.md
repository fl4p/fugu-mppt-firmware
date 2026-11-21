*this document is an LLM generated placeholder*

# HS short, LS clamp and `selftest bflow`: SPICE study of the Fugu2 power stage (2026-11-20, revised 2026-11-21)

Decks, scripts, CSVs and plots: `/Users/fab/dev/pv/ee/dcdc-tools/examples/fugu2-hs-short/` (README
there; one command per question, `run_all.sh` for everything). Every number below comes from models
built for this study (ngspice 46, cross-checked in LTspice 17.2.4 for Q1). No existing bench-validated
deck covers this circuit. The validated inputs are the coil inductance points, the vendor FET models
and the FastHenry extraction of the PCB copper. Nothing was measured on hardware.

This version replaces the 2026-11-20 record. Two audits found errors in the deck and in several
claims (section "Audit"); every table here was re-run on the corrected deck. The superseded outputs
are in `out/pre-audit/` and `out/pcb9ac7e69/`.

## Summary

- **Q1, hard LS clamp after an HS short: not safe on the current Fugu2, on 8s or 16s.**
  - With the "+12V" rail at its real 9.34 V, the clamp peak is set by how far the LS gate rises
    and by the LS channel model, not by the loop alone. At 27 V the first pulse is 1694 / 1609 /
    1184 A for the KP × 1 / 0.9 / 0.6 bracket (966-1768 A over all corners), ΔTj 62-147 K. At
    55 V it is 1849 / 1714 / 1269 A with ΔTj 520-815 K. ID,pulse is 812 A.
  - The Cin dump drives BflowS to about -15 V and turns the BF FETs on transiently (Vgs 3.1 V):
    a 116 A (27 V) / 188 A (55 V) battery discharge pulse. At 55 V the BF also avalanches (Vds
    61.4 V on a 60 V part).
  - While the LS is held on, the gate network biases the BF at Vgs 1.63 V (1.49-1.77 V over the
    rail tolerance). For a typical part that is nA-µA. For a minimum-Vth part (1.5 V) it is mA to
    1.5 A, and for a hot minimum part up to tens of amperes (EKV estimate, unverified).
  - A duty ramp of 100 ms keeps every 27 V pulse at or below 799 A (KP × 1), 769 A (KP × 0.6),
    658 A on flat (IPP019, 764 A rating), ΔTj 29-40 K. That is a 1.6 % margin to ID,pulse in the
    nominal corner, so it is not a safe margin. At 55 V every ramp leaves 66-187 pulses above
    812 A, ΔTj 140-304 K and LS Vds up to 94 V.
  - A switched BflowG-BflowS pull-down removes the static bias (end Vgs 0.04 V) and the 27 V
    transient turn-on. It does not stop the 55 V BF avalanche (188 A).
- **Q5, Vin-adaptive LS pulsing: feasible on 8s at 400 kHz or more, not safe on 16s.**
  - A pulse can be held near a target peak (319-383 A for a 400 A target) because the peak is
    gate-limited, nearly independent of Vin. But one pulse moves only about 55-62 µC (215 ns), so
    the drain needs Q_pulse · f > Isc = 12 A: 39, 100 and 200 kHz never collapse Vin.
  - At 400 kHz, 27 V: Vin falls below 2 V and the LS is latched on after 2.2-9.8 ms, LS ΔTj
    43-77 K, peaks ≤ 478 A, no LS avalanche (Vds ≤ 44 V). The width ramp (400 → 600 A over 3 ms)
    shortens the slow corners to 4.5-6.3 ms. A die resistance of 100 mΩ, or a slow gate (8 Ω
    driver) without the ramp, stalls the drain.
  - At 55 V: 5.2-35 ms and ΔTj 146-253 K in every corner (as-built caps: 3.6-6.4 ms, ΔTj
    124-245 K). That is not survivable for repeated use; the 16s verdict is unsafe.
  - End state is the same as the hard clamp: Vin about 0.13 V, LS held on, BF Vgs 1.63 V.
- **Q6, BF moved between the coil and C10 (C10 on +Bat+).**
  - The C10 dump through the coil disappears (coil min about 0 A instead of -528 A), and the
    battery discharge pulse after an HS short with it (≤ 1 A).
  - But BflowS has no capacitor, so every BF turn-off with current, every LS pulse after a short
    and the `selftest bflow` blocking phase ring BflowS to -20 to -39 V and avalanche the BF
    (61 V, 0.1-0.3 mJ per event, far below EAS 455 mJ but every switching period). It needs a
    BflowS clamp; an added BuckGND → BflowS Schottky (hypothetical, not fitted) holds it at
    -0.3 to -1.1 V.
  - In the blocking phase of `selftest bflow` the battery current reads +3.1 to +3.9 A (forward,
    DCM through the body diodes) instead of 0, so the test's pass criterion would need changing.
  - Normal operation (65 / 76 V → 27 V, 21-25 A) is the same: BF loss 0.28 → 0.29 W, the same
    BflowS and BT+ ripple.
- **Q2, `selftest bflow` (current topology) works.**
  - BF off: reverse battery current 0.000 A over the whole ramp (to Vout - D·Vin = +2.8 V).
  - BF on: the battery current crosses 0 at Vout - D·Vin = +0.24..+0.48 V (mostly dead time) and
    is -2.1 to -4.0 A at +1.0 V. Slope 0.08-0.13 A per count.
  - With the real 9.34 V rail the BF gate settles at 7.5-12.8 V (below 10 V for D above about
    0.6), reaching 4 V within 4.4 ms. The 150 ms wait is enough.
  - The CCM boundary is at most 4.5 A, so the 5.4 A test current is in CCM.
- **Q3, HS short while converting.** The first LS pulse after a hard short carries 1.3-1.9 kA
  and 0.8-1.1 J (flat: 2.5 kA, 0.19 J, Vds 86 V on its 80 V LS). The battery then discharges at
  -500 to -600 A. The proposed ratio check never fires: the battery current reverses within
  0.2-1.6 ms. A reverse-current check (below about -5 A for 1 ms) would fire, but no firmware
  check acts before the first pulse.
- **Q4, idle HS-short check works for hard shorts above about 5 % irradiance** (i ≥ 0.46 A,
  margin ≥ 1.04 V). Below g 0.03 the current is under the 0.3 A threshold; a 0.5 Ω short is missed
  from g 0.3 upwards.
- **Q7, LS model tiers.** The vendor tiers (Infineon L0, L1, L3) give clamp peaks of 1.6-2.0 kA;
  the dcdc-tools curve and recon tiers saturate at about 890 A but take twice the energy (0.56 J,
  ΔTj 154 K at 27 V). No tier is anchored above ID,pulse. Every verdict holds in every tier.
- **Q8, hard clamp on the as-built input caps (2 × ZLH 470 µF, 0.94 mF).** The peak falls from
  1694 A (schematic 3 mF) to 958-1195 A at 27 V, still 1.2-1.6 × ID,pulse in every corner (ESR max
  included); at 55 V it stays at 1.2-2.3 kA.
  - 27 V: the caps hold 0.37 J (Cin) + 0.18 / 0.39 J (C10 fry / flat); the LS takes 0.04-0.13 J,
    ΔTj 12-41 K, Tj ≤ 101 °C from a 60 °C case. The input-cap energy does not destroy the LS
    thermally; the binding limit is the peak current above ID,pulse at Vds 1.7-5.2 V for 13-100 µs.
  - 55 V: 1.5 J + 0.74 / 1.57 J; the LS takes 0.21-0.67 J, ΔTj 67-261 K, Tj above 175 °C in most
    corners, and the peak is 2-4 × the 10 µs SOA reading.
- **Q9, hard clamp at low Vin (night).** The LS stays within its ratings up to 12 V (≤ 0.65 ×
  ID,pulse, ΔTj ≤ 4 K). The BF is the limit: the clamp pulls BflowS to -0.88 × Vin, the BF at its
  static gate bias turns on and the battery discharges through it in linear mode (28-38 V Vds).
  With no BF conduction at datasheet-min Vth, the clamp is safe up to **Vin 5 V** on fry as built
  and flat (3 V on fry with a 1000 µF C10). A minimum-Vth part running warm conducts from 2-3 V
  (from 1 V with the 10.11 V rail, at 6-28 mJ); from 5 V the BF pulses are outside its SOA.
- **D16 is not fitted** on any build and is not in any model. The schematic's D16 (ST15100S) sits
  BuckGND → SW, as the LS rectifier, not as a BflowS clamp.

## Audit (2026-11-21)

Two audits of the 2026-11-20 version (an input audit and a results audit) and an independent review
of the corrected draft found errors in the deck and in several claims. All tables in this record
were re-run on the corrected deck.

### Deck corrections

| item | 2026-11-20 | now | why |
|---|---|---|---|
| "+12V" rail (LS drive VDDA, VDDB via R19/D10) | 12 V | 9.34 V, corners 8.59 / 10.11 V | the net is the AP3012 boost, R37 33k / R38 5.1k |
| VDDB load | none, C19 started at 11 V | 4.7 kΩ to SW (about 2 mA, UCC21330 quiescent), C19 IC 9.0 V | the 1.5-2.5 mA quiescent current sets the static VDDB |
| LS channel | KP 551.4 only | KP × 1 / 0.9 / 0.6 bracket, Rds tempco (Rd +0.9 %/K, Rs +0.3 %/K); other model tiers in Q7 | ngspice and LTspice evaluate the vendor Level-3 card 1.5× apart above ID,pulse; KP × 0.6 reproduces the LTspice run |
| Cin ESR | 17 mΩ (datasheet max) per cap | 0.65 × 17 mΩ typical; max as a sensitivity | Chemi-Con KZN models put R at 0.6-0.78 × Zmax(100 kHz); the dump peak is ESR-set |
| C10 ESR | 17 mΩ | 0.65 × 17 mΩ typical; max (17 mΩ) and cold (× 3) as sensitivities; fry's ZLH 470 µF at 0.65 × 33 mΩ | same reason; a lower ESR damps the C10 ring less |
| board copper | 3.2 nH lumped loop, no R | FastHenry extraction of the current PCB working tree (section "PCB re-extraction") | the copper was assumed |
| HS short | 5 mΩ + 9 nH ∥ 2 Ω, loop L swept × 0.5 / × 2 | die R (1-100 mΩ, 5 mΩ nominal) + TO-220 package of the shorted HS (Ld 2.5 nH + Ls 1.8 nH, 0.5 mΩ) + extracted HS port and position term | the 9 nH and the × 0.5 / × 2 sweep were made up; the package values are the Infineon `_L0` TO-220 leads (there is no vendor model for IPA050N10NM5S) |
| D16 (ST15100S) | fitted, treated as a BuckGND → BflowS clamp | removed from every model | not fitted on any build. Its schematic position is anode BuckGND, cathode SW: the LS rectifier, not a BflowS clamp |
| BF body diode BV | 65 V | 61 V | V(BR)DSS is about 61-62 V typical at 25 °C |
| BF Vth corner | DVTH -0.8 as "low Vth" | DVTH -0.5 = datasheet min 1.5 V; -0.8 = a min part about 60 K hot | Vth(1 mA) typ about 2.0 V, -5 mV/K |
| as-built boards | not modelled | flat and fry variants (Setup) | |
| solver guard | end-time check only | also fails on "Timestep too small" / "simulation(s) aborted" | lite mode linearizes onto the full tstop grid, so an aborted run passed the end-time check. Found during the pulsing runs; every lite result here was produced with the guard |
| pulse-law clock (Q5) | | sample clock period = integer multiple of the pulse period, offset by half a period; latched hold | coincident breakpoints of two incommensurate pulse trains aborted ngspice at 21 ms; the first Q5 numbers came from such aborted runs and were discarded |
| L3 self-heating start (Q7, Q8) | | `.ic` sets the L3 junction and Cauer nodes to the case temperature | the Cth capacitors go to ground, so under `uic` the junction started at 0 °C, not at Tcase. Found during Q8; the Q7 and Q8 L3 rows were re-run |
| input caps | schematic 3 × EKZN 1000 µF on every board | as built: 2 × ZLH 470 µF on fry and flat (Q8, Q9); the schematic 3 mF stays in the main Q1-Q7 tables, as-built rows are marked | HW Stories |

### Refuted claims

| 2026-11-20 claim | corrected |
|---|---|
| Hard clamp peaks 2.0 kA (27 V) / 3.1 kA (55 V), loop-limited | 1.7 kA / 1.85 kA at KP × 1, set by the gate rail and the channel model (Q1, Q7) |
| A 100 ms ramp keeps every 27 V peak at 786-807 A, "might be survivable" | 769-799 A on the schematic board (ΔTj 29-31 K); 1.6 % margin in the nominal corner, not a safe margin |
| At 55 V every ramp leaves 195-219 pulses above 812 A | 195-219 was the 100 ms ramp only; now 66 / 118 / 187 pulses for 10 / 30 / 100 ms |
| Static BF bias 2.14 V | 1.63 V (1.49-1.77 V over the rail tolerance) |
| BF at the low Vth corner conducts 20-26 A continuously | that was the 12 V rail with DVTH -0.8; see the leak table in Q1 |
| A Schottky BuckGND → BflowS (ST15100S = D16) limits BflowS | D16 is BuckGND → SW and not fitted. An added BflowS Schottky is a hypothetical part; it would carry 225-494 A, above the ST15100S 220 A surge rating |
| Q2: Ibat "at D·Vin = Vbat - 1.5 V" | the ramps ended at Vout - D·Vin = +0.5 to +0.9 V (Vin rises 2-3 V as the reverse current pumps Cin); the re-run ramps go to Vbat - 3 V and report Ibat at +1.0 V |
| The BF pump charges while SW is low | it charges while SW is high (VB = Vin + 9.1 V feeds D8/R21) |
| sw_phases `mode_untestable` = off interval below about 15 µs | it is the 300 ns min_dwell against off spans below 15 µs (D > 0.415) |
| sw_phases DCM(0.654) is a soft-commutation misclassification | it is an sw_phases bug: `subintervals` accepted a 5 ns "on" span (an edge ring). Reported, not fixed here |
| BflowS minimum to 0.01 V | ±0.5 V numerical scatter between runs with the same physics |
| BF avalanche at 65 V | about 61 V |
| Q4 margins | rest on a generic body-diode model of the BF FETs (not fitted to the TPH1R306PL diode curve) |

## Setup

### Circuit and component values

Extracted from the KiCad project `~/dev/ee/hw/Fugu2` (schematic at HEAD 13574db plus the working
tree), netlist exported with kicad-cli. All builds share one PCB, so the copper extraction applies
to every board.

| part | value | net / role | source |
|---|---|---|---|
| Q1, Q3 (HS) | IPA050N10NM5S | Solar+ → SW | schematic; generic Level 1 (KP 400, VTO 3, Rd 3.6 mΩ), used only before the short in Q2/Q3/Q6 |
| shorted HS | die R 5 mΩ (1-100 mΩ) + TO-220 leads Ld 2.5 nH + Ls 1.8 nH, 0.5 mΩ + HS port (Q1 4.71 nH, Q3 5.92 nH, mutual 4.51 nH) | Solar+ → SW | die R: brief; leads: Infineon `_L0` TO-220; port: extraction |
| Q2 (LS) | IPP022N12NM6, leads inside the model (Lg 6 nH, Ld 2.5 nH, Ls 1.8 nH) | SW → BuckGND | schematic; vendor `_L0` channel (tiers in Q7) |
| LS gate | R10 4.7 Ω + driver 1 Ω (slow-gate corner 8 Ω), R8 10k; drive from the "+12V" rail | | schematic |
| "+12V" rail | 9.34 V (8.59 / 10.11 V) | AP3012 boost, R37 33k / R38 5.1k | schematic + AP3012 Vref tolerance |
| HS gate | R4/R6 4.7 Ω on, R5/R7 2.2 Ω + D1 MSS1P4 off, R9 10k | | schematic |
| dead time | R1 20k → about 185 ns (UCC21330 formula); MCPWM `pwm_deadtime_ns` 0 | | schematic + datasheet formula |
| Cin bulk, as built | 2 × Rubycon ZLH 470 µF/100 V (0.94 mF) on fry, flat, ftall and fheat2; |Z| max 33 mΩ each at 100 kHz, ESR modelled 0.65 × 33 mΩ typical / 33 mΩ max, ESL 7.5 nH lumped; ripple rating 2.4 A rms | Solar+ - BuckGND | HW Stories (fisi.md, flat.md, ftall.md, fheat2.md); ZLH datasheet |
| Cin bulk, schematic | C11, C12, C34 3 × 1000 µF/100 V EKZN, ESR 0.65 × 17 mΩ each (max 17 mΩ), ESL 6.7 nH lumped; used for the "schematic board" rows | Solar+ - BuckGND | schematic; EKZN datasheet |
| Cin bulk port | nearest bulk port 11.21 nH / 3.42 mΩ (both variants) | | extraction |
| Cin MLCC | C16, C27, C9 1 µF; C17, C18 220 nF; C21, C22 10 µF; lumped 12 µF (assumed, DC-bias derated), ESL 0.5 nH, 3 mΩ; flat: no 10 µF parts, 3.44 µF; loop copper 3.29 nH / 3.13 mΩ | | schematic; flat.md; extraction |
| snubber | R11 7.5 Ω + C8 2.2 nF | SW - BuckGND | schematic |
| D16 | ST15100S, **not fitted on any build**, not modelled | BuckGND → SW (LS rectifier position) | schematic; Fab |
| D9 | ST40100C, DNP | | schematic |
| coil | 50 µH: 2 × KS130-060A, N = 20, DCR 4.75 mΩ; 80 µH: 2 × KS184-125A, N = 12, DCR 2.43 mΩ; DCR plus saturation only | SW → BflowS | HW Stories + coil.conf backups |
| C10 | schematic and flat (assumed): 1000 µF/100 V EKZN, ESR 0.65 × 17 mΩ, 20 nH; max and cold (× 3) as sensitivities. fry: 1 × ZLH 470 µF, ESR 0.65 × 33 mΩ | BflowS - BuckGND | schematic; fisi.md ("out: 1x") |
| BF FETs | Q5, Q6, Q7 TPH1R306PL (60 V) | S = BflowS, D = Bat+ | schematic |
| BF gate | C20 10 nF and DZ2 13 V BflowG-BflowS; R24 1M; R22 47k BflowG → Q4 drain; Q4 BSS123; R23 10k | | schematic |
| BF charge pump | VDDB (D10 from "+12V" via R19 22 Ω, C6 100 nF + C19 10 µF to SW, 4.7 kΩ quiescent load) → D8 → R21 200k → BflowG | | schematic + UCC21330 datasheet |
| F1 | 40 A fuse, 1.5 mΩ, no melting | Bat+ → BT+ | schematic; R assumed |
| R26 | 0.5 mΩ | BuckGND → GND | schematic |
| battery | ideal source + 25 mΩ (27 V) / 40 mΩ (55 V) + 1 µH | BT+ - GND | assumed |
| panel | Isc = 12·g A, diode string to Voc ≈ 75 V, 0.3 Ω, 5 µH cable, F3 3 mΩ | | brief + assumed |

The coil model has DCR and saturation but no core loss and no AC winding resistance. Both would
damp the coil-C10 ring, so leaving them out overstates the ring amplitude (conservative for the
BflowS and BF numbers).

### As-built boards

| board | part | value used | source |
|---|---|---|---|
| fry | HS | 2 × IPP040N08NF2S, rg_ext 4.7 Ω, gate diode MSS1P4 (not modelled: the short replaces the HS) | `/Users/fab/dev/pv/ee/dcdc-tools/loss/examples/fisi.yaml` |
| fry | LS | IPP022N12NM6 | fisi.yaml |
| fry | Cin | 2 × ZLH 470 µF, ESR 0.65 × 33 mΩ each, ESL 7.5 nH lumped; MLCCs as the schematic | `~/dev/ee/hw/Fugu2/HW Stories/fisi.md` |
| fry | C10 | 1 × ZLH 470 µF, ESR 0.65 × 33 mΩ | fisi.md |
| fry | snubber | fisi.md: "no snubber (?)"; modelled with the schematic R11/C8 (unverified) | fisi.md |
| fry | board | Fugu HW 2.2 | fisi.md |
| fry | coil | 2 × KS184-125A, N = 12 (80 µH) | fisi.md, coil.conf backup |
| fry | D16 | fisi.md lists an ST15100S on the low side; per Fab no build has D16 fitted. **Conflict, not resolved** | fisi.md vs Fab |
| flat | LS | IPP019N08NF2S (80 V, ID,pulse 764 A) | `~/dev/ee/hw/Fugu2/HW Stories/flat.md` |
| flat | HS | 2 × CSD (not modelled) | flat.md |
| flat | Cin | 2 × ZLH 470 µF, no 10 µF MLCCs (3 × 1 µF + 2 × 220 nF assumed, 3.44 µF) | flat.md |
| flat | coil | 2 × KS130-060A (50 µH) | flat.md |
| flat | LS gate | 4.7 Ω, no gate discharge network (modelled with the schematic network); "ls rise time 262 ns, fall 176 ns" noted on the bench | flat.md |
| flat | C10, BF FETs, BF gate network | **not documented; schematic values assumed** | assumption |
| both | PCB copper | the common extraction | one PCB for all builds |

### Sensor filters (Sensors.kicad_sch)

- Vin: R18 200k / R20 7.5k, C15 10 nF, τ = 72 µs.
- Vout: R52 470k / R53 11k, C37 100 nF, τ = 1.07 ms.
- Current: R26 0.5 mΩ → U6 AS333 inverting stage, R40 1k / R41 43k, non-inverting input biased by
  R42 226k / R39 1k from 3.3 V.
  - Output 0.64 V at 0 A, 21.5 mV/A; it hits the 0 V rail at about -29.8 A.
  - C32 100 pF across R41 gives τ 4.3 µs, then R51 1k / C36 100 nF gives τ 100 µs.
- R26 carries the battery-loop current only. Shoot-through (Cin → HS → LS → BuckGND → Cin) does not
  cross it.

### Models

- **LS IPP022N12NM6 (default tier `vendor_l0`).** The vendor `IPP022N12NM6_L0` Level-3 channel
  (KP 551.4, VTO 4.5) with Rd/Rs and body diode. Its Cgd network does not converge in ngspice, so
  Cgd is fixed at 0.5 nF. `KPF` scales KP (bracket × 1 / 0.9 / 0.6), `TJ` sets the Rds tempco.
  Other tiers in Q7.
- **LS ΔTj.** Vendor Cauer ladder (Zthtype 1, max), case held at 25 °C. ΣRth = 0.38 K/W.
- **BF TPH1R306PL.** Square law ID = 155.3·(VGS - 2.613)² per die, hand-traced from Fig. 8.3 of
  the datasheet (dsdig refused: no temperature labels bound). `DVTH` shifts VTO; body diode BV 61 V,
  Cgs 6 nF, Cgd fixed 0.48 nF. No subthreshold region, so static leakage uses the EKV estimate in
  `metrics.bf_leak_ekv` (Q1).
- **TPH1R306PL SOA** (Fig. 8.16, by hand): about 1-1.5 A (1 ms) and 5 A (100 µs) at 30-40 V Vds.
  EAS 455 mJ, IAS 50 A (datasheet). Used to judge linear-mode and avalanche BF events.
- **Coil.** Current-state integrator with incremental L(i), KDM sendust %µ laws, H[Oe] =
  0.4π·N·I/le_cm, air-core floor. Flat model 29.2 µH at 25.6 A against 29.5 µH from a bench ripple
  capture; fry model about 9 % high against its capture.
- **Q4 gate.** A 6 Ω switch driven by a PWL. The pull-down variant is a switch BflowG-BflowS with
  the same control.
- **Solver.** gear, reltol 1e-3, gmin 1e-9, rshunt 1e8. `run()` raises on an early stop and on
  "Timestep too small" / "simulation(s) aborted" in the log.

### PCB re-extraction

The 2026-11-20 copper came from an extraction of board revision 9ac7e69 (PCB sha 0b21d522). Since
then C21, C22, C16, C17, C11 and C12 have moved and the zones changed (current working tree, PCB
sha 8c735191). The extractor (`dcdc-tools/parasitics`, per-device FET ports, no leads) was re-run on
the working tree. The zone fills were checked as current with a kicad-cli refill of a copy.

| quantity | 9ac7e69 | working tree (used) |
|---|---|---|
| MLCC loop L / R | 3.20 nH / 3.14 mΩ | 3.29 nH / 3.13 mΩ |
| nearest bulk port | C11, 7.72 nH / 2.03 mΩ | C12, 11.21 nH / 3.42 mΩ |
| HS port L, Q1 / Q3 | 3.42 / 5.55 nH | 4.71 / 5.92 nH |
| HS mutual | 3.73 nH | 4.51 nH |
| HS position term Q1 / Q3 | 0.06 / 2.2 nH | 0.02 / 1.23 nH |
| LS conduction R | 0.91 mΩ | 2.30 mΩ |

C34 is not bonded to the extracted copper and was left out as a port, as before. Package leads are
added in the deck (above). The old copper runs as corner `pcb9ac7e69`: +3 % LS peak at 27 V
(1747 A), +15 K ΔTj. Files: `parasitics-current/`.

### Firmware facts used

- `pwm_freq` 39 kHz, 4103 counts per period; MCPWM timer 160 MHz (6.25 ns per count).
- `computeDCM` (src/buck.h): DCM when ir > 2.0·i (enter) or 1.8·i (exit), or i < 0.1 A.
- `iout_short` = 6 A. coil.conf L0 is 56 µH on flat and 40 µH on fry.

## Q1: hard LS clamp after an HS short

Initial state: converter idle, BF commanded off, panel at full sun charging the battery through the
HS short, the coil and the BF body diodes (about 11-12 A). At t = 15 ms the LS gate steps to the
"+12V" rail and holds. Source: `out/q1v2_all.csv` and `out/q1v2_hw.csv` (`q1_v2.py`); plots
`plots/q1v2_*.png`.

### Hard clamp

Schematic board, 50 µH, 5 mΩ die, typical ESR, rail 9.34 V unless noted.

| case | LS peak A | LS E J | LS ΔTj K | LS Vds max V | coil min A | BflowS min V | BF Vgs max V | BF Vds max V | Ibat discharge peak A |
|---|---|---|---|---|---|---|---|---|---|
| 27 V, KP × 1 | 1694 | 0.23 | 62 | 28 | -528 | -15.5 | 3.10 | 42.4 | 116 |
| 27 V, KP × 0.9 | 1609 | 0.25 | 67 | 28 | -527 | -15.4 | 3.10 | 42.4 | 115 |
| 27 V, KP × 0.6 | 1184 | 0.39 | 119 | 28 | -524 | -15.3 | 3.09 | 42.3 | 114 |
| 27 V, 80 µH | 1692 | 0.22 | 61 | 28 | -484 | -17.9 | 3.19 | 44.8 | 162 |
| 27 V, Cin ESR max | 1549 | 0.21 | 49 | 28 | -527 | -15.4 | 3.10 | 42.4 | 115 |
| 27 V, rail 8.59 V | 1417 | 0.30 | 85 | 28 | -527 | -15.4 | 3.09 | 42.4 | 114 |
| 27 V, rail 10.11 V | 1737 | 0.22 | 57 | 28 | -529 | -15.5 | 3.10 | 42.4 | 118 |
| 27 V, Tj 125 °C | 1591 | 0.33 | 82 | 28 | -515 | -15.1 | 3.09 | 42.0 | 110 |
| 27 V, die 1 mΩ | 1768 | 0.37 | 130 | 28 | -532 | -15.5 | 3.10 | 42.5 | 117 |
| 27 V, die 20 mΩ | 950 | 0.15 | 26 | 28 | -521 | -15.3 | 3.09 | 42.2 | 113 |
| 27 V, die 100 mΩ | 691 | 0.09 | 18 | 28 | -523 | -15.2 | 3.09 | 42.2 | 113 |
| 27 V, short in Q3 | 1691 | 0.23 | 62 | 28 | -528 | -15.5 | 3.10 | 42.4 | 116 |
| 27 V, copper 9ac7e69 | 1747 | 0.26 | 77 | 28 | -529 | -15.5 | 3.10 | 42.5 | 116 |
| 27 V, C10 ESR max | 1693 | 0.23 | 62 | 28 | -471 | -13.7 | 3.04 | 40.7 | 90 |
| 27 V, C10 ESR cold | 1693 | 0.22 | 62 | 28 | -359 | -10.9 | 2.93 | 37.9 | 51 |
| 27 V, KP × 0.6, rail 8.59 V | 966 | 0.51 | 147 | 28 | -520 | -15.3 | 3.09 | 42.2 | 111 |
| 55 V, KP × 1 | 1849 | 2.17 | 611 | 57 | -1187 | -16.6 | 2.99 | 61.4 | 188 |
| 55 V, KP × 0.9 | 1714 | 2.37 | 632 | 57 | -1174 | -16.3 | 2.98 | 61.4 | 183 |
| 55 V, KP × 0.6 | 1269 | 3.56 | 664 | 57 | -962 | -11.0 | 2.93 | 61.3 | 98 |
| 55 V, Cin ESR max | 1835 | 1.88 | 518 | 57 | -1184 | -16.5 | 2.99 | 61.4 | 186 |
| 55 V, rail 10.11 V | 2193 | 1.75 | 533 | 57 | -1199 | -16.9 | 3.00 | 61.4 | 194 |
| 55 V, die 1 mΩ | 1877 | 2.87 | 815 | 57 | -1198 | -17.0 | 2.99 | 61.4 | 194 |
| 55 V, die 100 mΩ | 1583 | 0.40 | 99 | 57 | -1204 | -16.7 | 2.99 | 61.4 | 190 |
| 55 V, C10 ESR cold | 1849 | 2.06 | 611 | 57 | -829 | -7.8 | 2.93 | 61.2 | 51 |
| flat, 27 V, KP × 1 | 1165 | 0.07 | 20 | 29 | -535 | -15.6 | 3.10 | 42.6 | 118 |
| flat, 27 V, Cin ESR max | 969 | 0.06 | 15 | 29 | -535 | -15.6 | 3.10 | 42.6 | 118 |
| fry, 27 V, KP × 1 | 1195 | 0.05 | 17 | 28 | -169 | -20.6 | 3.07 | 47.4 | 105 |
| fry, 27 V, Cin ESR max | 1009 | 0.05 | 12 | 28 | -169 | -20.6 | 3.07 | 47.4 | 105 |

All rows, including KP × 0.9 / 0.6 for every corner, DVTH -0.5 / -0.8 and 80 µH at 55 V, are in
the CSV.

- **Every clamp exceeds ID,pulse** (812 A; flat 764 A) except the 100 mΩ die at 27 V.
- **The peak is set by the gate and the channel model.** The rail moves it by -16 / +3 % (27 V)
  and the KP bracket by -30 %. 27 V and 55 V give similar peaks, because the channel saturates at
  the Vgs the gate reaches during the dump. The loop copper matters less: the old 9ac7e69 copper
  changes it by 3 %. At 55 V the LS takes 1.8-4.1 J and ΔTj 520-815 K: destroyed.
- **As built, the dump is smaller** (flat 1165 A, fry 1195 A at 27 V, ΔTj 12-20 K). The 2 × ZLH
  470 µF Cin has three times the ESR of 3 × EKZN 1000 µF. It still exceeds ID,pulse.
- **The C10 ring** is the coil-current reversal: C10, charged to Vbat, discharges through the coil
  and the clamped LS (coil min -528 A at 27 V, -1187 A at 55 V). It pulls BflowS to -11 to -21 V
  and turns the BF on for a moment. A higher C10 ESR damps it (cold: -359 A, 51 A discharge).
  fry's 470 µF C10 rings less (-169 A) but swings BflowS further (-20.6 V).
- **BF avalanche at 55 V**: BF Vds 61.4 V on every 55 V row except the cold C10.
- **The shorted-HS position** (Q1 or Q3) and the die R between 1 and 5 mΩ change the peak by under
  5 %. 20 mΩ and above halve it.

### Static bias after the clamp

While the LS is held on, D10 keeps VDDB at the rail, and D8/R21 200k feed R22 47k ∥ R24 1M:
BflowG settles at 1.63 V (1.49 / 1.77 V at the rail corners) with BflowS near 0 V. The square-law
SPICE model gives 0 A at that Vgs; it has no subthreshold region. The EKV interpolation of the same
fit (`metrics.bf_leak_ekv`, 3 dies, slope factor n not on the datasheet, bracketed 1.5-3) gives:

| Vth(1 mA) V | n | Vgs 1.49 V | Vgs 1.63 V | Vgs 1.77 V |
|---|---|---|---|---|
| 2.5 (max) | 1.5 / 3 | < 1 µA | < 1 µA | < 1 µA |
| 2.0 (typ) | 1.5 | 6 nA | 0.2 µA | 8 µA |
| 2.0 (typ) | 3 | 4 µA | 26 µA | 0.16 mA |
| 1.5 (min) | 1.5 | 2.3 mA | 75 mA | 1.5 A |
| 1.5 (min) | 3 | 2.6 mA | 16 mA | 91 mA |
| 1.2 (min, about 60 K hot) | 1.5 | 2.2 A | 14 A | 44 A |
| 1.2 (min, about 60 K hot) | 3 | 0.12 A | 0.6 A | 2.8 A |

A typical part leaks µA. A minimum-Vth part at the high rail can carry about 1.5 A at 25-30 V Vds,
which is 40 W across three dies; that heats it, lowers Vth and raises the current. The bias is
therefore a hazard at the low-Vth corner, not for a typical part. The switched pull-down removes it
(end Vgs 0.04 V).

### Duty ramps

The LS duty ramps from 0.5 % to 100 % at 39 kHz over 10, 30 or 100 ms, then holds.

| case | LS peak A | pulses > ID,pulse | LS E J | LS ΔTj K | LS Vds max V | coil min A | BflowS min V | BF Vgs max V | Ibat discharge A |
|---|---|---|---|---|---|---|---|---|---|
| 27 V, 10 ms, KP × 1 / × 0.6 | 1157 / 1076 | 35 / 29 | 0.53 / 0.57 | 45 / 49 | 50 | -30 | -0.6 | 1.64 | 0 |
| 27 V, 30 ms, KP × 1 / × 0.6 | 810 / 803 | 0 | 0.69 / 0.73 | 36 / 38 | 48 / 47 | -13 | -1.0 / -0.7 | 1.63 | 0 |
| 27 V, 100 ms, KP × 1 / × 0.6 | 799 / 769 | 0 | 1.13 / 1.19 | 29 / 31 | 43 | -5 | -0.9 | 1.63 | 0 |
| 27 V, 100 ms, Cin ESR max | 796 | 0 | 1.09 | 28 | 43 | -5 | -1.0 | 1.63 | 0 |
| flat, 27 V, 100 ms | 658 (rating 764) | 0 | 0.77 | 40 | 60 | -6 | -1.2 | 1.63 | 0 |
| 55 V, 10 ms, KP × 1 / × 0.6 | 1691 / 1231 | 66 / 68 | 2.70 / 3.61 | 245 / 304 | 94 / 86 | -56 | -0.6 | 1.64 | 0 |
| 55 V, 30 ms, KP × 1 / × 0.6 | 1408 / 1173 | 118 / 116 | 3.01 / 3.70 | 169 / 207 | 88 / 82 | -26 | -0.8 | 1.63 | 0 |
| 55 V, 100 ms, KP × 1 / × 0.6 | 1241 / 1061 | 187 | 4.12 / 4.77 | 140 / 155 | 77 / 74 | -11 | -1.0 | 1.63 | 0 |

- Ramps of 10 ms or longer avoid the transient BF turn-on (Vgs ≤ 1.64 V, no battery current) and
  keep BflowS above -1.2 V. They do not remove the static bias.
- A ramp moves the LS stress rather than removing it: every on-pulse while Cin is charged is a
  shoot-through.
- At 27 V the 30 and 100 ms ramps stay at or below 810 A and 799 A in the nominal corner. That is a
  0.2-1.6 % margin to 812 A, and the rail-high corner (not run for the ramps) raised the hard-clamp
  peak by 2.5 %. It is not a safe margin.
- At 55 V the ramps leave 66-187 pulses above 812 A, ΔTj 140-304 K and Vds up to 94 V on the
  120 V part. Destructive.

### Hardware variants

DVTH -0.8 (worst BF corner), KP × 1, 50 µH. "Added Schottky" is a hypothetical BuckGND → BflowS
Schottky (ST15100S model), not fitted on any build. "bfcoil" is the BF moved between the coil and
C10 (Q6). Source `out/q1v2_hw.csv` (its re-run replaces the four bfcoil + added Schottky rows that aborted in `out/q1v2_all.csv`).

| variant | V | LS peak A | coil min A | BflowS min V | BF Vgs max V | BF Vds max V | Ibat discharge A | end BF Vgs V | Schottky peak A |
|---|---|---|---|---|---|---|---|---|---|
| none (DVTH -0.8) | 27 | 1694 | -528 | -14.4 | 2.34 | 41.1 | 136 | 1.63 | |
| switched pull-down | 27 | 1694 | -528 | -15.6 | 0.49 | 42.6 | 0 | 0.04 | |
| added Schottky | 27 | 1693 | -528 | -3.8 | 1.94 | 30.8 | 7 | 1.63 | 225 |
| pull-down + added Schottky | 27 | 1693 | -528 | -3.8 | 0.38 | 30.8 | 0 | 0.04 | 225 |
| bfcoil | 27 | 1694 | 0 | -20.2 | 1.80 | 47.3 | 1 | 1.63 | |
| bfcoil + added Schottky | 27 | 1688 | 0 | -0.4 | 1.63 | 27.7 | 1 | 1.63 | < 1 |
| none (DVTH -0.8) | 55 | 1849 | -1187 | -16.6 | 2.40 | 61.3 | 188 | 1.63 | |
| switched pull-down | 55 | 1849 | -1186 | -16.6 | 1.00 | 61.4 | 188 | 0.04 | |
| added Schottky | 55 | 1849 | -1187 | -7.7 | 2.04 | 61.2 | 26 | 1.63 | 494 |
| pull-down + added Schottky | 55 | 1849 | -1187 | -7.7 | 0.89 | 61.2 | 20 | 0.04 | 494 |
| bfcoil | 55 | 1849 | 0 | 0.0 | 1.84 | 55.0 | 0 | 1.63 | |
| bfcoil, DVTH 0 | 55 | 1849 | 0 | -5.8 | 2.43 | 61.1 | 0 | 1.63 | |
| bfcoil + added Schottky | 55 | 1849 | 0 | -0.2 | 1.84 | 55.2 | 0 | 1.63 | < 1 |

- **Switched pull-down** (BflowG-BflowS, active while Q4 is on): removes the static bias and the
  27 V transient turn-on. At 55 V the BF still avalanches (Vds 61.4 V) and 188 A flows.
- **Added Schottky**: holds BflowS at -3.8 / -7.7 V but carries 225 / 494 A, above the ST15100S
  220 A non-repetitive surge rating; it would need a larger part.
- **Both**: battery discharge 0 A (27 V) / 20 A (55 V).
- **bfcoil**: the coil no longer dumps C10 (coil min about 0 A, battery discharge ≤ 1 A), but
  BflowS has no capacitor and rings to -20 V (27 V); at 55 V the BF reaches 61 V at DVTH 0. With
  an added Schottky BflowS stays at -0.2 to -0.4 V and the Schottky carries under 1 A, because the
  coil current is small in this topology.
- No hardware variant changes the LS stress.

### LTspice cross-check

Schematic board, 50 µH, KP × 1 deck translated to LTspice (`out/ltspice_xcheck.txt`,
`ltspice_xcheck.py`).

| quantity | ngspice 27 V | LTspice 27 V | ngspice KP × 0.6, 27 V | ngspice 55 V | LTspice 55 V | ngspice KP × 0.6, 55 V |
|---|---|---|---|---|---|---|
| LS peak A | 1694 | 1213 | 1184 | 1849 | 1293 | 1269 |
| LS energy J | 0.23 | 0.45 | 0.39 | 2.17 | 3.51 | 3.56 |
| LS ΔTj K | 62 | 118 | 119 | 611 | 669 | 664 |
| coil min A | -528 | -503 | -524 | -1187 | -979 | -962 |
| BflowS min V | -15.5 | -14.7 | -15.3 | -16.6 | -11.0 | -11.0 |
| BF Vgs max V | 3.10 | 3.10 | 3.09 | 2.99 | 2.94 | 2.93 |
| Ibat discharge peak A | 116 | 103 | 114 | 188 | 97 | 98 |
| end BF Vgs V | 1.64 | 1.61 | 1.63 | 1.63 | 1.61 | 1.63 |

- LTspice evaluates the same vendor Level-3 card with an earlier saturation: its peak is 30 % lower
  and its energy 1.6-2× higher. ngspice with KP × 0.6 reproduces LTspice within 3 % on the LS
  numbers, which is why the bracket goes down to × 0.6.
- The BF, BflowS and battery numbers agree within 10 % at 27 V. At 55 V LTspice gives a smaller C10
  ring, again matching ngspice KP × 0.6.
- Both simulators give the same verdict: the LS is overstressed at 27 V and destroyed at 55 V.

### Verdict

- **No firmware sequence makes the hard clamp safe** on the current hardware. At 27 V every clamp
  or ramp shorter than 30 ms exceeds ID,pulse; the 30-100 ms ramps sit within 2 % of it. At 55 V
  every sequence destroys the LS.
- **Hardware needed** for any clamp: a switched BflowS-referenced pull-down, and a way to keep the
  LS out of the Cin dump. Q5 is the firmware alternative for 8s.

## Q2: `selftest bflow`

Forced PWM, 39 kHz, 185 ns hardware dead time, 9.34 V rail.

- (a)/(b): D starts just above the operating point and ramps linearly over 40 ms down to
  D·Vin = Vbat - 3 V. The window averages are 0.5 ms. As the reverse current pumps Cin, Vin rises
  by 0.4-0.9 V, so the ramps reach Vout - D·Vin = +2.7 to +2.8 V with the BF off and +0.8 to +1.3 V
  with it on.
- The BF-on cases start after 25 ms of gate charge-up.
- sw_phases classified 13 sampled periods per ramp: all CCM, none unverified.

Sources: `out/q2ab_ramps.csv`, `out/q2g_gate_charge.csv`, `out/q2c_*.csv`, `out/q2_summary.txt`;
plots `plots/q2_*.png`.

### (a) BF commanded off

| Vin / Vbat | L µH | Ibat A | BflowS - D·Vin V | coil max / min A | C10 peak A | BF Vgs max V |
|---|---|---|---|---|---|---|
| 65 / 27 | 50 | 0.000 | +0.40 | 7.94 / -4.38 | 4.42 | -0.50 |
| 65 / 27 | 80 | 0.000 | +0.39 | 6.61 / -2.78 | 2.95 | -0.50 |
| 40 / 27 | 50 | 0.000 | +0.23 | 4.31 / -2.71 | 2.81 | -0.51 |
| 40 / 27 | 80 | 0.000 | +0.26 | 3.34 / -1.70 | 1.90 | -0.51 |
| 76 / 55 | 50 | 0.000 | +0.42 | 6.08 / -4.51 | 4.61 | -0.54 |
| 76 / 55 | 80 | 0.000 | +0.46 | 4.50 / -2.86 | 3.07 | -0.54 |

Reverse current is blocked over the whole ramp (up to Vout - D·Vin = +2.8 V). BflowS follows the
switch-node average, 0.23-0.46 V above D·Vin, and the coil ripple circulates through C10.

### (b) BF commanded on

| case | I = 0 at Vout - D·Vin V | I = -0.5 A at V | slope A/V | A per count | Ibat at +1.0 V A | BF Vgs V |
|---|---|---|---|---|---|---|
| 65/27 V, 50 µH | +0.378 | +0.427 | 5.61 | 0.090 | -4.01 | 12.70 |
| 65/27 V, 80 µH | +0.391 | +0.473 | 6.67 | 0.107 | -3.23 | 12.69 |
| 40/27 V, 50 µH | +0.237 | +0.293 | 9.70 | 0.097 | not reached (+0.77 V max) | 9.19 |
| 40/27 V, 80 µH | +0.258 | +0.360 | 12.59 | 0.125 | not reached (+0.77 V max) | 9.19 |
| 76/55 V, 50 µH | +0.412 | +0.490 | 5.23 | 0.098 | -3.31 | 11.49 |
| 76/55 V, 80 µH | +0.483 | +0.582 | 6.33 | 0.118 | -2.14 | 11.47 |
| 65/27 V, 50 µH, Rbat 10 mΩ | +0.387 | +0.426 | 6.03 | 0.097 | -4.00 | 12.70 |
| 65/27 V, 50 µH, Rbat 50 mΩ | +0.383 | +0.429 | 5.07 | 0.081 | -4.01 | 12.70 |

- **Offset.** The zero crossing sits at Vout - D·Vin = +0.24..+0.48 V. Most of it is the 185 ns dead
  time: with current near zero the switch node commutates during the dead time and shifts the
  effective duty by about dt/T = 0.7 %, 0.3-0.5 V at 40-76 V Vin. The rest is conduction drop.
- **Slope.** 5-13 A/V, 0.08-0.13 A per count (one count is Vin/4103, 10-18.5 mV). From I = 0 to
  -0.5 A takes 4-8 counts. Rbat 10-50 mΩ changes the slope by under 20 %.
- **Pass/fail.** At Vout - D·Vin = +1.0 V the reverse current is -2.1 to -4.0 A with the BF on and
  exactly 0 with it off. A threshold near -0.5 A, taken after the measured zero crossing, separates
  the two with a wide margin. Firmware should reference the offset to the measured crossing, not to
  D·Vin = Vbat.
- **Settling (derived, not simulated as a step).** τ = L/R_eff ≈ 50 µH / 0.17 Ω ≈ 0.3 ms (80 µH:
  about 0.5 ms). A dwell of at least 2 ms per count step is enough.

### (g) Gate charge-up from Q4 off (BF commanded on) at near-zero current

| Vin / Vbat | D | rail | t(Vgs 4 V) ms | t(Vgs 8 V) ms | t(Vgs 10 V) ms | final Vgs V |
|---|---|---|---|---|---|---|
| 35 / 27 | 0.79 | 9.34 | 3.9 | 23.6 | not reached | 8.10 |
| 35 / 27 | 0.79 | 8.59 | 4.4 | not reached | not reached | 7.53 |
| 35 / 27 | 0.79 | 10.11 | 3.5 | 13.9 | not reached | 8.70 |
| 40 / 27 | 0.69 | 9.34 | 3.5 | 12.8 | not reached | 9.00 |
| 50 / 27 | 0.55 | 9.34 | 3.1 | 8.8 | 17.8 | 10.63 |
| 65 / 27 | 0.43 | 9.34 | 2.7 | 6.9 | 10.8 | 12.61 |
| 65 / 27 | 0.43 | 8.59 | 2.9 | 7.5 | 12.1 | 12.04 |
| 65 / 27 | 0.43 | 10.11 | 2.5 | 6.4 | 9.7 | 12.78 |
| 76 / 27 | 0.36 | 9.34 | 2.5 | 6.2 | 9.2 | 12.79 |
| 67 / 55 | 0.83 | 9.34 | 3.6 | 13.2 | not reached | 8.88 |
| 67 / 55 | 0.83 | 8.59 | 3.9 | 18.7 | not reached | 8.31 |
| 67 / 55 | 0.83 | 10.11 | 3.2 | 10.6 | not reached | 9.47 |
| 70 / 55 | 0.80 | 9.34 | 3.2 | 10.4 | not reached | 9.65 |
| 76 / 55 | 0.73 | 9.34 | 2.8 | 7.7 | 13.6 | 11.21 |

- The pump (VB = Vin + VDDB through D8/R21 while SW is high, against R24 1M) settles lower at high
  D; the mechanism was not isolated. With the real 9.34 V rail the BF gate no longer reaches 10 V
  above D ≈ 0.6; at D 0.79 and the low rail it settles at 7.5 V.
- Vgs 4 V is reached within 4.4 ms and the final value within about 25 ms in every case, so the
  150 ms wait is enough. 7.5 V is enough for the 5.4 A selftest current (the TPH1R306PL is specified
  at Vgs 4.5 V), but it is below the 10 V the 2026-11-20 record assumed.
- DVTH +0.3 changes nothing; the pump does not depend on Vth.
- The fixed Cgd (0.48 nF) understates the Miller charge at Vds ≈ 0; the real charge-up is somewhat
  slower.

### (c) CCM boundary

Analytic boundary: I_b = Vo·(1 - D)/(2·L·f), Vo = Vbat + 0.7 V (`out/q2c_ccm_boundary_analytic.csv`).

| L µH | Vbat V | Vin 40 | 50 | 60 | 65 | 70 | 76 |
|---|---|---|---|---|---|---|---|
| 50 | 27 | 2.18 | 3.17 | 3.82 | 4.08 | 4.29 | 4.51 |
| 80 | 27 | 1.37 | 1.98 | 2.39 | 2.55 | 2.68 | 2.82 |
| 50 | 55 | | | | | 2.92 | 3.81 |
| 80 | 55 | | | | | 1.82 | 2.38 |

- The worst case is 4.51 A (50 µH, 27 V, 76 V in), so the selftest's 5.4 A is in CCM with 0.9 A
  margin.
- Simulation (BF off, duty ramped through the boundary, `out/q2c_ccm_sim.csv`): the coil current
  minimum reaches zero at 4.5-4.7 A (76/27 V, 50 µH), 2.8-2.9 A (80 µH), 3.1-3.3 A (50/27 V) and
  3.8-4.3 A (76/55 V), matching the analytic values.
- sw_phases switches from DCM to CCM at 3.9-4.0 A (76/27 V, 50 µH) and 2.3-2.4 A (80 µH), about
  15 % below the coil-current boundary. Its DCM verdicts (dcm_frac 0.21-0.24, 9 + 7 windows) sit
  where the coil current already dips below zero and the edges commutate softly. Unverified windows
  (`no_subintervals`): 16, 7 and 10 of 80 for 76/27 V 50 µH, 76/27 V 80 µH and 76/55 V; 0 for
  50/27 V.
- The 6 A trip acts on the 100 µs-filtered sense, which attenuates the 39 kHz ripple about 24×.
- The firmware's own boundary uses coil.conf L0 (flat 56 µH against 50.9 µH measured), which puts
  it about 10 % low, the safe direction for a CCM claim.

## Q3: HS short during conversion

Initial state: 65 V in, 27.6 V battery, 23.6 A, D = 0.435 fixed (open loop: no MPPT or protection
controller). The BF is on and the LS keeps switching. The short appears at t = 15 ms on a period
boundary. Sensor traces pass through the board RC filters; the current reading clips at -29.8 A.
Source: `out/q3_conv_short.csv` (`q3_conv_short.py`), plots `plots/q3_*.png`.

### Per-LS-pulse shoot-through (pulse width 14.1 µs)

| case | pulse 1: peak A / E mJ / Vin V / LS Vds max V | pulse 2 | pulse 3 | pulse 5 |
|---|---|---|---|---|
| healthy | 32 / 0.05 / 64 / 80 | same | same | same |
| die 1 mΩ | 1905 / 1085 / 63 / 94 | 1872 / 847 / 55 / 85 | 1838 / 617 / 46 / 75 | 1770 / 215 / 29 / 58 |
| die 5 mΩ | 1877 / 895 / 63 / 93 | 1845 / 672 / 55 / 84 | 1812 / 461 / 46 / 75 | 1737 / 114 / 30 / 56 |
| die 5 mΩ, 80 µH | 1877 / 894 / 63 / 93 | 1846 / 676 / 55 / 84 | 1814 / 470 / 47 / 76 | 1743 / 119 / 30 / 57 |
| die 5 mΩ, KP × 0.6 | 1286 / 765 / 63 / 84 | 1270 / 646 / 57 / 78 | 1252 / 521 / 51 / 70 | 1215 / 297 / 37 / 57 |
| die 5 mΩ, flat | 2455 / 192 / 63 / 86 | 1360 / 36 / 32 / 51 | 698 / 9 / 16 / 27 | 222 / 1 / 5 / 13 |
| die 20 mΩ | 1778 / 277 / 63 / 92 | 1737 / 138 / 56 / 82 | 1577 / 86 / 48 / 71 | 1205 / 43 / 35 / 53 |
| 100 mΩ | 592 / 12 / 63 / 73 | 567 / 11 / 61 / 70 | 543 / 10 / 58 / 67 | 500 / 8 / 53 / 62 |
| 0.5 Ω | 107 / 0.47 / 63 / 66 | 105 | 104 | 102 |
| 2 Ω | 14 / 0.016 / 64 / 74 | 14 | 14 | 14 |

- The first pulse after a hard short carries 1.3-1.9 kA and 0.8-1.1 J in 14 µs on the schematic
  board, far outside the LS ratings, with LS Vds up to 94 V on the 120 V part.
- flat's IPP019N08NF2S takes a higher first peak (2455 A) but less energy (0.19 J), because its
  smaller ZLH Cin collapses faster. Its Vds reaches 86 V on an 80 V part.
- The healthy LS Vds peaks at 80 V at 64 V in (switch-node overshoot of the healthy converter).

### Sensor view after the short

| case | t | Vin V (72 µs) | Vout V (1.07 ms) | I through R26 A (true) | Vout/Vin |
|---|---|---|---|---|---|
| 5 mΩ | before | 64.1 | 27.6 | +23.6 | 0.430 |
| | 0.1 ms | 47.5 | 27.7 | +34.6 | 0.582 |
| | 0.3 ms | 10.1 | 27.5 | +5.1 | 2.72 |
| | 1 ms | 10.0 | 20.5 | -568 | 2.04 |
| | 10-30 ms | 9.9-10.0 | 12.8-12.9 | -564 | 1.28-1.30 |
| 20 mΩ | 1 ms | 15.2 | 21.2 | -508 | 1.39 |
| | 10-30 ms | 15.2 | 14.3-14.4 | -503 | 0.94-0.95 |
| 100 mΩ | 0.3 ms | 44.2 | 27.5 | +13.3 | 0.623 |
| | 1 ms | 30.9 | 24.2 | -329 | 0.783 |
| | 10-30 ms | 31.6 | 19.0 | -320 | 0.60 |
| 0.5 Ω | 1 ms | 48.5 | 27.2 | -23 | 0.561 |
| | 10-30 ms | 51.2 | 24.6 | -97 | 0.480 |
| 2 Ω | 1 ms | 59.6 | 27.5 | +12.1 | 0.461 |
| | 10-30 ms | 59.0 | 26.7 | -10.6 | 0.453 |
| 5 mΩ, PWM stop + BF off at 1 ms | 3 ms | 27.1 | 26.1 | +1.8 | 0.963 |
| | 30 ms | 28.1 | 27.3 | +11.9 | 0.973 |

- **Shoot-through does not cross R26.** The sensed current is the battery loop. After a hard short
  the battery discharges through the BF (on), the coil and the switching LS, which also conducts
  back through the HS short: -500 to -600 A at a 25 mΩ battery resistance. F1 (40 A) would open
  within milliseconds; fuse melting is not modelled.
- **Proposed check** (CCM, I > 1 A, Vout/Vin > D + 0.15 = 0.585, sustained 50-100 ms):

  | case | ratio > 0.585 after ms | I < 1 A after ms | longest time both hold ms |
  |---|---|---|---|
  | 1 mΩ | 0.10 | 0.32 | 0.22 |
  | 5 mΩ | 0.10 | 0.31 | 0.21 |
  | 5 mΩ, 80 µH | 0.10 | 0.35 | 0.24 |
  | 5 mΩ, KP × 0.6 | 0.12 | 0.38 | 0.25 |
  | 5 mΩ, flat | 0.05 | 0.18 | 0.13 |
  | 20 mΩ | 0.12 | 0.28 | 0.17 |
  | 100 mΩ | 0.26 | 0.39 | 0.14 |
  | 0.5 Ω | never (max 0.56) | 0.73 | 0 |
  | 2 Ω | never (max 0.46) | 1.59 | 0 |

  The check never fires while converting. Its "I > 1 A" condition fails within 1.6 ms because the
  battery current reverses, and the LS has already taken 0.2-1.1 J in the first pulse, 11 µs after
  the short.
- **False trips.** In CCM with a stiff battery a healthy converter sits at M ≤ D (0.430 against
  0.435). DCM raises M above D, but M - D > 0.15 needs coil.conf L0 ≥ 1.6·L_true; flat's 56/50.9 =
  1.10 is well inside. Above D 0.85 the check can neither fire nor trip.
- **A criterion that would fire.** While converting with the BF on: battery current below about
  -5 A (reverse) for about 1 ms, or Vin below Vout + 0.5 V while D < 0.85. All short cases cross
  -5 A within 1.6 ms (the 2 Ω case reaches -10.6 A); the sensor's -29.8 A clip is below that. The
  action is PWM stop plus BF off: the "stop at 1 ms" run returns to +11.9 A (panel through the short
  and the BF body diodes), which the Q4 idle check then sees. Not checked against PSU / PV-sim mode
  or the `selftest bflow` reverse phase, which use reverse current on purpose.
- **Neither check protects the LS from the first pulses.** Only a gate-driver desaturation or
  overcurrent shutdown acts within the 11 µs before the first LS pulse.

## Q4: idle HS-short detection

Converter idle: LS off, BF commanded off. The panel charges the battery through the shorted HS,
the coil and the BF body diodes. Readings are steady-state means after 50 ms. Criterion: i > 0.3 A
and -1.5 V < Vin - Vout < 1.5 V + 0.05 Ω·i. Source: `out/q4_idle.csv` (`q4_idle.py`), 50 µH unless
noted. The 2026-11-20 numbers change by under 0.02 V on the corrected deck.

| g | Vbat | short | i A | Vin - Vout V | detects | margin V |
|---|---|---|---|---|---|---|
| 0.01 | 27 | 5 mΩ | 0.051 | 0.41 | no (i) | 1.09 |
| 0.01 | 55 | 5 mΩ | 0.000 | 0.06 | no (i) | 1.45 |
| 0.02 | 27 | 5 mΩ | 0.171 | 0.45 | no (i) | 1.06 |
| 0.03 | 27 | 5 mΩ | 0.291 | 0.46 | no (i) | 1.05 |
| 0.03 | 55 | 5 mΩ | 0.218 | 0.45 | no (i) | 1.06 |
| 0.05 | 27 | 5 mΩ | 0.530 | 0.48 | yes | 1.04 |
| 0.05 | 55 | 5 mΩ | 0.456 | 0.48 | yes | 1.05 |
| 0.1 | 27 | 5 mΩ | 1.13 | 0.51 | yes | 1.04 |
| 0.3 | 27 | 5 mΩ | 3.53 | 0.58 | yes | 1.09 |
| 0.6 | 27 | 5 mΩ | 7.12 | 0.66 | yes | 1.20 |
| 1.0 | 27 | 5 mΩ | 11.92 | 0.75 | yes | 1.34 |
| 1.0 | 55 | 5 mΩ | 11.65 | 0.75 | yes | 1.34 |
| 0.1 | 27 | 0.1 Ω | 1.13 | 0.62 | yes | 0.93 |
| 1.0 | 27 | 0.1 Ω | 11.92 | 1.88 | yes | 0.21 |
| 1.0 | 55 | 0.1 Ω | 11.58 | 1.85 | yes | 0.23 |
| 0.1 | 27 | 0.5 Ω | 1.13 | 1.07 | yes | 0.48 |
| 0.3 | 27 | 0.5 Ω | 3.52 | 2.33 | no (window) | -0.65 |
| 1.0 | 27 | 0.5 Ω | 11.91 | 6.65 | no (window) | -4.55 |
| 0.3 | 27 | 5 mΩ, 80 µH | 3.53 | 0.58 | yes | 1.10 |
| 1.0 | 27 | 5 mΩ, 80 µH | 11.92 | 0.72 | yes | 1.37 |

- **Hard short.** Vin - Vout is 0.4-0.75 V: the BF body-diode drop plus I·R. That leaves at least
  1.04 V of margin to the window.
- **Current threshold.** 0.3 A sets the low-light limit at g ≈ 0.03-0.05. 0.3 A is 6.5 mV at the U6
  output, near the ESP32 ADC offset and noise; the firmware needs a zero-calibrated reading.
- **Partial shorts.** 0.1 Ω is detected at full sun with 0.21 V margin. 0.5 Ω is detected only
  below g ≈ 0.1.
- **False trips.** With a healthy HS and the converter idle there is no current path, so i = 0. A
  sensor offset above 0.3 A is the only false-trip route (not simulated).
- The BF body diode is a generic model, not fitted to the TPH1R306PL diode curve, so the Vin - Vout
  values carry that uncertainty.

## Q5: Vin-adaptive LS pulsing after an HS short

Instead of one hard clamp, the LS is pulsed so that every pulse stays below a target peak. Vin is
drained pulse by pulse against the panel, and the LS is held on once Vin is low. Source:
`out/q5_all.csv` (`q5_pulse.py`), plots `plots/q5_<case>.png`; single-pulse calibration
`out/q5_calibration.txt` (`q5_calibrate.py`).

### Law

Vin is read through the board's 72 µs sensor RC and sampled at the 39 kHz control rate (every N
pulse periods). Each LS pulse gets

    t_on = max(t_g(I_tgt), L_min · I_tgt / Vin)

and the LS is held on (latched) once the sampled Vin < V_hold = 2 V.

- t_g is the gate-limited width that gives I_tgt in the fastest corner (KP × 1, rail 10.11 V, 55 V):
  170 / 215 / 315 ns for 300 / 400 / 600 A. The pulse peak is set by how far the LS gate rises in
  t_on, not by Vin.
- L_min = 12 nH is the lowest commutation inductance: MLCC loop copper 3.29 nH, MLCC ESL 0.5 nH,
  HS and LS TO-220 leads 4.3 nH each. The L·I/V term only matters below about 20 V, where it
  lengthens the pulse as Vin falls.
- Variant "ramp": t_g rises from 215 ns (400 A) to 315 ns (600 A) over 3 ms. In the fast corner
  that caps the peak near 600 A; in slow corners it raises the drain until Vin falls.

### Single-pulse calibration

One or two LS pulses into a charged Cin (schematic board, 50 µH), per width and corner. Source
`out/q5_calibration.txt`. Peak A / charge per pulse µC / energy per pulse mJ:

| width ns | 27 V nominal | 27 V rail 10.11 V | 27 V slow gate (8 Ω) | 55 V nominal | 55 V rail 10.11 V | 55 V KP × 0.6 |
|---|---|---|---|---|---|---|
| 150 | 203 / 24 / 0.62 | 239 / 31 / 0.78 | 97 / 10 / 0.25 | 219 / 28 / 1.48 | 262 / 36 / 1.90 | 202 / 25 / 1.31 |
| 215 | 320 / 55 / 1.32 | 365 / 67 / 1.56 | 206 / 32 / 0.80 | 341 / 62 / 3.18 | 404 / 78 / 3.96 | 311 / 54 / 2.80 |
| 250 | 378 / 76 / 1.74 | 426 / 91 / 2.02 | 264 / 48 / 1.17 | 402 / 84 / 4.26 | 475 / 106 / 5.23 | 365 / 73 / 3.73 |
| 315 | 479 / 120 / 2.55 | 529 / 140 / 2.85 | 365 / 85 / 1.94 | 508 / 132 / 6.44 | 597 / 164 / 7.77 | 455 / 113 / 5.60 |
| 350 | 529 / 147 / 2.97 | 579 / 169 / 3.27 | 417 / 108 / 2.37 | 561 / 161 / 7.68 | 659 / 199 / 9.19 | 499 / 137 / 6.67 |

- The peak depends on the width and the gate drive, and only by 5-10 % on Vin (27 against 55 V).
- At the 400 A width (215 ns) one pulse moves 55-62 µC at the nominal rail: 22-25 A average at
  400 kHz against Isc 12 A, 11-12 A at 200 kHz (break-even, no collapse) and 2.2-2.4 A at 39 kHz.
  With the slow gate it is 32-37 µC, 13-15 A at 400 kHz: barely above Isc, which is why that corner
  stalls without the width ramp.

### Pulse rate

The panel refills Cin at up to Isc = 12 A, so the pulse rate times the charge per pulse must exceed
12 A with margin.

| rate | 27 V, 400 A target | 55 V, 400 A target |
|---|---|---|
| 39 kHz | no collapse, Vin stays 28.0 V | no collapse, 56.1 V |
| 100 kHz | no collapse, 28.0 V | no collapse, 56.0 V |
| 200 kHz | no collapse, 28.0 V after 30 ms | no collapse, 50.8 V after 45 ms |
| 400 kHz | hold after 6.5 ms | hold after 16.5 ms |
| 800 kHz | hold after 2.3 ms | hold after 5.5 ms |

- **39 kHz is not enough**, nor are 100 or 200 kHz. The break-even lies between 200 and 400 kHz for
  a 400 A target. A 300 A target stalls even at 400 kHz (Vin 23 V after 30 ms at 27 V).
- A burst at a high rate inside each 39 kHz control period works the same way: what counts is the
  average drain, Q_pulse · f_pulse > Isc.
- 400 kHz is 400 counts of the 160 MHz MCPWM timer; t_on 215-315 ns is 34-50 counts. That is within
  the timer's resolution but below the firmware's usual minimum LS pulse, which would have to be
  bypassed in this mode.
- **Cumulative LS loss.** At 400 kHz the LS takes 2.27 J over 6.5 ms at 27 V (about 350 W, 0.9 mJ
  per pulse) and 10.9 J over 16.5 ms at 55 V (about 660 W, 1.7 mJ per pulse). The ΔTj below is
  this heating on the Cauer ladder over a fixed 25 °C case.

### Results

400 kHz, 50 µH, schematic board, rail 9.34 V, KP × 1 unless noted. "no" = no hold within the run
(30 ms at 27 V, 45 ms at 55 V).

| case | hold ms | LS peak A | LS Vds max V | LS ΔTj K | LS E J | short E J | BflowS min V | BF Vgs max V | coil min A |
|---|---|---|---|---|---|---|---|---|---|
| 27 V, 300 A | no | 240 | 43 | 72 | 8.10 | 0.34 | 21.9 | -0.51 | -4.0 |
| 27 V, 400 A | 6.5 | 319 | 43 | 62 | 2.27 | 0.22 | -2.7 | 1.88 | -13.9 |
| 27 V, 600 A | 2.3 | 478 | 41 | 77 | 1.29 | 0.22 | -2.0 | 1.81 | -28.9 |
| 27 V, 400 A, 800 kHz | 2.3 | 324 | 44 | 95 | 1.68 | 0.16 | -3.0 | 1.92 | -27.5 |
| 27 V, 400 A, KP × 0.6 | 7.6 | 306 | 44 | 62 | 2.53 | 0.22 | -2.5 | 1.85 | -12.5 |
| 27 V, 400 A, rail 10.11 V | 5.5 | 370 | 43 | 64 | 2.03 | 0.21 | -3.1 | 2.06 | -16.0 |
| 27 V, 400 A, rail 8.59 V | 9.8 | 300 | 43 | 62 | 3.09 | 0.24 | -2.5 | 1.73 | -12.6 |
| 27 V, 400 A, slow gate | no | 205 | 41 | 77 | 8.50 | 0.30 | 23.3 | -0.52 | -3.7 |
| 27 V, 400 A, die 1 mΩ | 6.1 | 327 | 44 | 67 | 2.32 | 0.06 | -2.5 | 1.86 | -12.7 |
| 27 V, 400 A, die 100 mΩ | no | 206 | 39 | 31 | 3.50 | 5.16 | 21.0 | -0.51 | -3.7 |
| 27 V, 400 A, short in Q3 | 7.4 | 315 | 46 | 60 | 2.45 | 0.22 | -2.3 | 1.84 | -11.9 |
| 27 V, 400 A, 80 µH | 6.4 | 319 | 43 | 62 | 2.22 | 0.22 | -2.7 | 2.07 | -11.2 |
| 27 V, 400 A, g 0.3 | 4.0 | 319 | 42 | 53 | 1.30 | 0.12 | -3.5 | 1.98 | -18.4 |
| flat, 27 V, 400 A | 3.1 | 358 | 37 | 70 | 0.86 | 0.11 | -1.2 | 1.73 | -20.4 |
| fry, 27 V, 400 A | 2.5 | 319 | 43 | 45 | 0.79 | 0.09 | -6.4 | 2.33 | -18.1 |
| 27 V, ramp 400 → 600 A | 4.9 | 367 | 43 | 70 | 1.87 | 0.21 | -2.2 | 1.82 | -11.9 |
| 27 V, ramp, slow gate | 6.3 | 330 | 41 | 72 | 2.23 | 0.21 | -2.5 | 1.86 | -12.5 |
| 27 V, ramp, rail 10.11 V | 4.5 | 383 | 43 | 69 | 1.78 | 0.21 | -2.9 | 2.05 | -15.9 |
| 27 V, ramp, rail 8.59 V | 5.6 | 356 | 43 | 71 | 2.04 | 0.21 | -3.0 | 1.78 | -15.4 |
| 27 V, ramp, KP × 0.6 | 5.2 | 358 | 44 | 70 | 1.95 | 0.21 | -2.6 | 1.86 | -13.1 |
| 27 V, ramp, die 100 mΩ | no (Vin 3.06 V) | 215 | 39 | 22 | 1.45 | 4.43 | 0.5 | 0.17 | -4.5 |
| flat, 27 V, ramp | 2.9 | 360 | 37 | 67 | 0.80 | 0.11 | -0.6 | 1.67 | -22.8 |
| fry, 27 V, ramp | 2.2 | 329 | 43 | 43 | 0.73 | 0.09 | -3.0 | 1.89 | -14.1 |
| 55 V, 300 A | no | 257 | 71 | 153 | 24.9 | 0.61 | 31.4 | -0.53 | -4.8 |
| 55 V, 400 A | 16.5 | 340 | 70 | 166 | 10.9 | 0.51 | -2.6 | 1.87 | -13.4 |
| 55 V, 600 A | 5.2 | 507 | 68 | 243 | 6.56 | 0.49 | -4.3 | 2.08 | -27.4 |
| 55 V, 400 A, 800 kHz | 5.5 | 343 | 72 | 253 | 7.54 | 0.35 | -4.4 | 2.08 | -25.5 |
| 55 V, 400 A, KP × 0.6 | 21.2 | 311 | 71 | 155 | 12.8 | 0.54 | -2.5 | 1.86 | -13.0 |
| 55 V, 400 A, rail 10.11 V | 11.9 | 403 | 70 | 187 | 8.99 | 0.50 | -2.5 | 2.00 | -14.5 |
| 55 V, 400 A, rail 8.59 V | 29.7 | 299 | 70 | 146 | 16.2 | 0.60 | -2.6 | 1.73 | -13.2 |
| 55 V, 400 A, slow gate | no | 226 | 68 | 156 | 25.3 | 0.53 | 33.7 | -0.53 | -4.7 |
| 55 V, 400 A, die 100 mΩ | no | 291 | 65 | 92 | 9.77 | 10.7 | 21.0 | -0.51 | -7.4 |
| 55 V, ramp 400 → 600 A | 7.7 | 487 | 70 | 237 | 7.46 | 0.48 | -2.8 | 1.89 | -14.5 |
| 55 V, ramp, slow gate | 11.0 | 380 | 68 | 212 | 8.87 | 0.45 | -2.6 | 1.87 | -13.4 |
| 55 V, ramp, rail 10.11 V | 6.6 | 568 | 70 | 249 | 6.92 | 0.50 | -2.8 | 2.03 | -17.2 |
| 55 V, ramp, rail 8.59 V | 9.7 | 408 | 70 | 218 | 8.33 | 0.46 | -2.5 | 1.73 | -12.9 |
| 55 V, ramp, die 100 mΩ | 35.3 | 363 | 65 | 99 | 4.34 | 8.59 | -0.4 | 1.66 | -8.7 |
| flat, 55 V, 400 A | 6.4 | 385 | 65 | 220 | 4.12 | 0.24 | -1.4 | 1.75 | -26.9 |
| flat, 55 V, ramp | 4.2 | 475 | 65 | 245 | 3.29 | 0.24 | -1.5 | 1.75 | -30.4 |
| fry, 55 V, 400 A | 6.0 | 340 | 70 | 124 | 3.82 | 0.20 | -3.7 | 1.96 | -14.5 |
| fry, 55 V, ramp | 3.6 | 428 | 70 | 145 | 2.93 | 0.19 | -6.1 | 2.28 | -18.1 |

The as-built 55 V rows are in `out/q5_asbuilt55.csv`. All 60 schematic and 27 V cases, including the C10 / Cin ESR, copper, DVTH and topology variants (all within 1 ms and
3 K of their base case), are in the CSV and in `tables.py q5`.

### Robustness

- **Peak.** No case exceeds its target by more than the rail tolerance: 400 A targets give
  299-403 A, the ramp 330-568 A, the 600 A target 478-507 A. flat's IPP019 reaches 358-360 A
  against its 764 A rating.
- **LS avalanche.** None (avalanche energy 0 in every case). LS Vds peaks at 37-46 V (27 V) and
  65-73 V (55 V) on a 120 V part (flat: 80 V part, 37 V).
- **Gate-speed corners stall the fixed-width law.** With an 8 Ω driver the 215 ns pulse reaches only
  205-226 A, too little charge to beat 12 A. The width ramp recovers it (6.3 ms at 27 V). Because
  t_g is not measured on the board, the ramp is required, not optional.
- **A resistive short stalls it.** With a 100 mΩ die the shorted HS limits each pulse (206-215 A)
  and Vin settles at 3-23 V. The fault then dissipates in the HS die (4.4-10.7 J), and the LS is
  never latched. The firmware needs a time-out that falls back to stopping the converter.
- **BF.** During the drain BF Vgs reaches 1.7-2.3 V (transient), BflowS stays at -0.6 to -6.4 V;
  the coil swings to -11 to -29 A. No battery discharge pulse of the Q1 kind.

### End state

After the hold Vin is about 0.13 V and the LS is on. The BF gate settles at 1.63 V (1.50-1.78 V over
the rail tolerance), as after the hard clamp; with the switched pull-down it is 0.04 V. The coil
rings with C10 at -12 to -29 A and decays; BflowS swings to -2 to -4 V.

### Verdict

- **8s (27 V): feasible only with the width ramp and a pulse rate of at least 400 kHz.** Hold
  after 2.2-6.3 ms, peaks ≤ 383 A (flat 360 A against 764 A), LS ΔTj 43-72 K over a fixed 25 °C
  case, no LS avalanche. It needs a time-out for resistive shorts and still leaves the static BF
  bias unless the pull-down is fitted.
- **16s (55 V): unsafe.** On the schematic board every corner takes 5-35 ms with LS ΔTj
  146-253 K (2.4-25 J). The as-built 0.94 mF Cin drains faster (3.6-6.4 ms, 2.9-4.1 J) but still
  gives ΔTj 124-145 K on fry and 220-245 K on flat's IPP019 (higher Zth), which a TO-220 will
  survive once at best. fry's ramp case also brings the BF to 61 V (0.65 A avalanche).
- **Can a Vin-adaptive scheme drain Cin against Isc within the peak target?** Only if the pulse
  rate is at least about 400 kHz for a 400 A target. At the 39 kHz control rate it cannot: one
  pulse per period moves about 55-62 µC at the 400 A width, 2.2-2.4 A on average against 12 A.

## Q6: BF between the coil and C10

The BF FETs move from the C10 node to between the coil and C10, so C10 sits on +Bat+ and BflowS is
the coil end with no capacitor. Compared with the current topology ("sch") and the current topology
with the switched pull-down. No BflowS clamp is fitted in the "bfcoil" variant; "bfcoil + added
Schottky" is a hypothetical BuckGND → BflowS Schottky. Source: `out/q6_*.csv` (`q6_bfpos.py`) and
the Q1/Q5 topology rows; plots `plots/q6_*.png`.

### (a) HS short

| case | LS peak A | coil min A | BflowS min V | BF Vgs max V | BF Vds max V | Ibat discharge A |
|---|---|---|---|---|---|---|
| sch, hard clamp, 27 V | 1694 | -528 | -15.5 | 3.10 | 42.4 | 116 |
| sch + pull-down, 27 V | 1694 | -528 | -15.6 | 0.49 | 42.6 | 0 |
| bfcoil, 27 V | 1694 | 0 | -20.4 | 1.81 | 47.5 | 1 |
| sch, hard clamp, 55 V | 1849 | -1187 | -16.6 | 2.99 | 61.4 | 188 |
| sch + pull-down, 55 V | 1849 | -1186 | -16.6 | 1.00 | 61.4 | 188 |
| bfcoil, 55 V | 1849 | 0 | -5.8 | 2.43 | 61.1 | 0 |
| sch, pulsing 400 A, 27 / 55 V | 319 / 340 | -13.9 / -13.4 | -2.7 / -2.6 | 1.88 / 1.87 | | 0 |
| bfcoil, pulsing, 27 / 55 V | 319 / 340 | -0.2 | -23.7 / -7.3 | 1.78 / 1.90 | | 0 |
| bfcoil + added Schottky, pulsing | 319 / 340 | -0.1 | -0.3 / -0.4 | 1.63 | | 0 |

Hard clamp with bfcoil + added Schottky: BflowS -0.4 / -0.2 V, Schottky peak under 1 A, BF Vds
27.7 / 55.2 V, no battery discharge (Q1 hardware table).

The LS stress does not depend on the BF position. Moving the BF removes the C10 discharge through the
coil and the battery pulse, at 55 V too. Without a clamp BflowS still swings to -6 to -24 V, and
at 55 V the BF reaches its 61 V avalanche.

### (b) BF turned off under reverse current

The BF is commanded off while the battery current is -5 or -20 A (65 → 27 V).

| case | BF current before A | BflowS min V | BF Vds max V | BF energy mJ | BF avalanche mJ | BF current < 0.1 A after ms | Schottky peak A |
|---|---|---|---|---|---|---|---|
| sch, -5 A | 1.0 | +23.7 | 0.4 | 0 | 0 | 0.42 | |
| sch, -20 A | 18.9 | +10.7 | 4.3 | 0.3 | 0 | 0.43 | |
| sch + pull-down, -5 A | 1.0 | +23.8 | 1.5 | 0 | 0 | 0.04 | |
| sch + pull-down, -20 A | 18.9 | +10.2 | 7.3 | 0.2 | 0 | 0.05 | |
| bfcoil, -5 A | 1.3 | -38.8 | 61.1 | 17.5 | 0.12 | 0.006 | |
| bfcoil, -20 A | 18.7 | -34.6 | 61.0 | 23.4 | 0.32 | 0.50 | |
| bfcoil + added Schottky, -5 A | 1.5 | -1.1 | 27.7 | 15.9 | 0 | 0.006 | 0.69 |
| bfcoil + added Schottky, -20 A | 18.8 | -1.1 | 27.7 | 21.3 | 0 | 0.50 | 0.66 |

- In the current topology C10 absorbs the coil current, so the BF turns off softly (Vds ≤ 7 V).
- In bfcoil the coil current has nowhere to go but the BF. The R22 gate discharge (τ about 1.3 ms)
  lets the BF commutate -20 A in linear mode over 0.15-0.5 ms (21-23 mJ, about 47 W per die for
  about 150 µs, inside the SOA reading). Then BflowS rings to -35 V and the BF avalanches at 61 V:
  0.1-0.3 mJ per event against EAS 455 mJ, IAS 50 A. While the LS keeps switching this repeats
  every period (BF Vgs pulses about 2.5 V).
- The firmware must stop switching when it turns the BF off in the bfcoil topology, or BflowS needs
  a clamp.

### (c) Normal buck operation

Forced PWM at 39 kHz, about 25 A into a 27 V battery. Means over t > 30 ms; ripple over the last
four switching periods.

Stiffer panel source (Isc 30 A, 0.1 Ω, Voc Vin + 1.5 V) so that it does not limit the 25 A point;
one secant step on the duty. 76 V settled at 21.5 A (the step overshot); all positions run the same
duty, so the comparison holds. BF loss is I²rms · RDS(on) max (1.3 mΩ, 25 °C) shared by three dies.
Ripple is shown after a 0.5 µs moving average (the raw node voltages carry ns edge spikes from the
BuckGND bounce, listed separately) and after a one-switching-period average. Source: `out/q6_c.csv`.

| BF position | Vin V | duty | Ibat A | BF rms A (3 dies) | BF loss W (3 dies) | BflowS ripple V pp (0.5 µs avg) / edge spikes V pp | BT+ ripple V pp (0.5 µs avg) / period avg mV pp | coil ripple A pp |
|---|---|---|---|---|---|---|---|---|
| schematic (C10 on BflowS) | 65 | 0.439 | 25.5 | 25.5 | 0.28 | 0.23 / 12.9 | 0.23 / 3.1 | 14.0 |
| schematic + pull-down | 65 | 0.439 | 25.5 | 25.5 | 0.28 | 0.25 / 12.9 | 0.25 / 2.5 | 14.0 |
| BF between coil and C10 | 65 | 0.439 | 25.5 | 25.8 | 0.29 | 0.24 / 13.1 | 0.24 / 2.3 | 14.0 |
| bfcoil + added Schottky (hypothetical) | 65 | 0.439 | 25.5 | 25.8 | 0.29 | 0.22 / 6.1 | 0.21 / 0.7 | 14.0 |
| schematic (C10 on BflowS) | 76 | 0.367 | 21.5 | 21.5 | 0.20 | 0.27 / 17.6 | 0.27 / 3.6 | 14.0 |
| schematic + pull-down | 76 | 0.367 | 21.5 | 21.5 | 0.20 | 0.27 / 17.5 | 0.27 / 3.7 | 14.0 |
| BF between coil and C10 | 76 | 0.367 | 21.5 | 21.9 | 0.21 | 0.27 / 17.3 | 0.27 / 3.2 | 14.0 |
| bfcoil + added Schottky (hypothetical) | 76 | 0.367 | 21.5 | 21.9 | 0.21 | 0.22 / 6.9 | 0.22 / 0.9 | 14.0 |

- In bfcoil the BF carries the coil current with its ripple, so its rms current rises by 1-2 %
  (25.5 → 25.8 A) and its loss by about 0.01 W. That is negligible.
- BflowS and BT+ ripple are the same in all positions (0.2-0.3 V pp after 0.5 µs averaging): with
  the BF on, its 0.4 mΩ (3 dies) ties BflowS to C10 in either position.
- The ns edge spikes (13-18 V pp, all nodes against GND) come from the BuckGND bounce and do not
  depend on the BF position. The hypothetical Schottky halves them on BflowS and BT+.
- The BF does not switch in normal operation, so there is no BF avalanche in bfcoil here.

### (d) `selftest bflow`, both phases (65 → 27 V, 50 µH)

| case | Ibat with BF off (blocks) | I = 0 with BF on at Vout - D·Vin | BflowS min V | Schottky peak A |
|---|---|---|---|---|
| sch | 0.000 A for Vout - D·Vin > +0.3 V | +0.38 V | +22 (no swing) | |
| bfcoil | +3.1 to +3.9 A (forward) over the whole ramp | +0.39 V | -38.6 | |
| bfcoil + added Schottky | +3.1 to +3.9 A (forward) | +0.39 V | -1.1 | 0.64 |

- The BF-on phase is the same in both topologies.
- In bfcoil the blocking phase does not read zero. Without C10 at BflowS the coil current cannot
  reverse; it flows forward through the body diodes in DCM pulses whenever BflowS rings above Vbat.
  The average is +3 to +4 A, and BflowS rings to -39 V, avalanching the BF every period. A
  `selftest bflow` on bfcoil hardware would need a different pass criterion (no reverse current,
  rather than near-zero current) and a BflowS clamp.

### Comparison

| | current topology | + switched pull-down | BF between coil and C10 |
|---|---|---|---|
| HS short, hard clamp: battery pulse | 116 / 188 A | 0 / 188 A | ≤ 1 A |
| HS short: BflowS | -15 V | -15 V | -6 to -24 V, no clamp |
| HS short: static BF bias | 1.63 V | 0.04 V | 1.63 V |
| BF turn-off under reverse current | soft, Vds ≤ 4 V | soft, 0.05 ms | -35 V ring, 61 V avalanche every period |
| normal operation, 21-25 A | BF 0.28 W, BT+ 0.23 V pp | same | BF 0.29 W, same ripple |
| `selftest bflow` blocking phase | 0 A | 0 A | +3 to +4 A, BF avalanche |
| needs a BflowS clamp | no | no | yes |

### Layout notes

- bfcoil puts C10 on +Bat+, after the BF. The BF source then sees the coil's switching ripple
  directly. The BF loop current (coil → BF → C10 → BuckGND) must be kept short, because there is
  no capacitor to absorb the edge on the BF side.
- A BflowS clamp in bfcoil would sit BuckGND → BflowS next to the BF sources. Its repetitive duty
  in the selftest is small (0.6-0.7 A peak), but after an HS short with a hard clamp the coil
  reversal is several hundred amperes (Q1: 225-494 A for the schematic position).
- The pull-down fix is a BflowS-referenced switch across C20; it needs a level shift from Q4's
  ground-referenced drive.

### Verdict

- **Moving the BF between the coil and C10 removes the HS-short battery pulse** (116 / 188 A → ≤ 1 A)
  and the C10 dump through the coil, at 55 V too. Normal operation does not change.
- **It needs a BflowS clamp.** Without one, every BF turn-off with current, every LS pulse after a
  short and the `selftest bflow` blocking phase ring BflowS to -20 to -39 V and avalanche the BF
  (0.1-0.3 mJ per event, every period while switching). An added BuckGND → BflowS Schottky
  (hypothetical part) holds BflowS at -0.3 to -1.1 V and carries under 1 A in every case simulated
  here, including the hard clamp.
- **It needs firmware changes.** Stop switching before or when the BF is turned off, and change the
  `selftest bflow` pass criterion to "no reverse current" (the blocking phase reads +3 to +4 A).
- **It does not change the LS stress** (Q1 and Q5 peaks identical). The switched BflowG-BflowS
  pull-down on the current topology removes the static bias and the 27 V transient turn-on with a
  smaller change, but not the 55 V BF avalanche. Both are BF-side fixes only.

## Q7: LS model tiers

The headline Q1 clamp (schematic board, 50 µH, 5 mΩ die, typical ESR, 9.34 V rail) and the Q5
pulsing case (400 A target, 400 kHz) were re-run with every LS model available for IPP022N12NM6,
and the flat clamp with two tiers of IPP019N08NF2S. Source: `out/q7_tiers.csv` (`q7_tiers.py`).
IPP040N08NF2S (fry's HS) is not used: the shorted HS is a resistance, not a FET model.

| tier | what it is |
|---|---|
| `vendor_l0` | reduced Infineon `_L0` Level-3 card, fixed Cgd, `KPF` bracket (the study default) |
| `vendor_l1` | full Infineon `_L1` (channel with Vth / gfs / Rds temperature terms at Tj = TEMP, nonlinear caps) |
| `vendor_l3` | full Infineon thermal model: self-heats on its own Zth ladder (Zthtype 1), Tcase 25 °C |
| `curve_lm` | dcdc-tools/loss curve tier: transfer law ID = 12.0·(VGS - 3.06)^2.35 with a 2.2 mΩ channel R, datasheet Coss/Crss curves, LM body diode, 25 °C |
| `recon` | dcdc-tools/loss reconstructed gate-charge tier: same channel law, fixed caps, 25 °C |

### Results

| tier | clamp 27 V: peak A / E J / ΔTj K | clamp 55 V | pulsing 27 V: hold ms / peak A / E J / ΔTj K | pulsing 55 V |
|---|---|---|---|---|
| vendor_l0, KP × 1 | 1694 / 0.23 / 62 | 1849 / 2.17 / 611 | 6.5 / 319 / 2.27 / 62 | 16.5 / 340 / 10.9 / 166 |
| vendor_l0, KP × 0.9 | 1609 / 0.25 / 67 | 1714 / 2.37 / 632 | | |
| vendor_l0, KP × 0.6 | 1184 / 0.39 / 119 | 1269 / 3.56 / 664 | 7.6 / 306 / 2.53 / 62 | 21.2 / 311 / 12.8 / 155 |
| vendor_l1 | 1624 / 0.27 / 75 | 1993 / 2.15 / 583 | 7.5 / 310 / 2.50 / 62 | 19.2 / 356 / 11.9 / 160 |
| vendor_l3 | 1596 / 0.32 / 88 (own Tj 101 °C) | 1683 / 4.32 / 654 (own Tj 633 °C) | 7.0 / 311 / 2.40 / 63 (own Tj 85 °C) | 15.9 / 366 / 10.6 / 170 (own Tj 190 °C) |
| curve_lm | 888 / 0.56 / 154 | 891 / 4.38 / 684 | 11.2 / 303 / 3.46 / 64 | 38.2 / 303 / 19.9 / 145 |
| recon | 886 / 0.56 / 154 | 894 / 4.38 / 684 | 9.5 / 308 / 3.02 / 63 | 34.1 / 309 / 18.5 / 146 |
| flat (IPP019), vendor_l0 | 1165 / 0.07 / 20 | | | |
| flat (IPP019), vendor_l1 | 1163 / 0.07 / 21 | | | |
| flat (IPP019), curve_lm | 1115 / 0.08 / 27 | | | |

ΔTj is the max-Zth Cauer ladder over a fixed 25 °C case for every tier; "own Tj" is the L3 model's
self-heated junction node (Tcase 25 °C). Pulsing LS Vds peaks at 43-45 V (27 V) and 70-74 V (55 V)
for L0 / curve / recon and at 55 V / 91-92 V for L1 / L3; no tier avalanches the 120 V part.

### What the tiers say

- **The clamp peak splits into two groups.** The vendor tiers (L0, L1, L3) give 1.6-2.0 kA; the
  dcdc-tools tiers (curve, recon) saturate at 886-894 A at both voltages. The KP bracket (× 0.6:
  1184-1269 A) sits between them. All exceed ID,pulse 812 A.
- **The energy and ΔTj move the other way.** A channel that saturates earlier holds a higher Vds
  during the dump: curve / recon take 0.56 J and ΔTj 154 K at 27 V against 0.23-0.32 J and 62-88 K
  for the vendor tiers. At 55 V every tier gives 2.2-4.4 J and ΔTj 583-684 K (the L3 junction node
  reaches 633 °C): destroyed in all of them.
- **On flat the tiers agree** (1115-1165 A, 0.07-0.08 J, 20-27 K): with the ZLH Cin the dump is
  ESR-limited rather than channel-limited.
- **The pulsing verdict holds in every tier.** Peaks stay at 303-366 A for a 400 A target. At 27 V
  the hold comes after 6.5-11.2 ms with ΔTj 62-64 K. At 55 V the slower-saturating curve / recon
  channels need 34-38 ms and 18-20 J, ΔTj 145-146 K; the vendor tiers need 16-21 ms, ΔTj 155-170 K (L3 junction node 190 °C).
  Unsafe at 55 V in every tier.

### Anchoring and extrapolation

No tier is anchored by measurement above ID,pulse (812 A; flat 764 A).

- **Datasheet range.** The IPP022N12NM6 typical transfer chart (Diagram 7, 25 °C) ends at about
  860 A at VGS 7.5 V; the output chart (Diagram 5) reaches 1000 A at VGS 8-10 V, still in the
  linear region at VDS 2.4-3.2 V. The IPP019N08NF2S charts end at 800 A. (Chart readings by eye,
  not digitized.)
- **Vendor L0 / L1 / L3.** Infineon's channel equations are fitted to the characterisation behind
  those charts; their fitting range is not published. At the 9.34 V drive they extrapolate the
  Level-3 / S5 channel law to 1.6-2.0 kA. That is consistent with the typical charts (VGS 8 V
  already exceeds 1000 A without saturating), but unverified above the chart range.
- **curve / recon.** The channel is anchored at Vth, at the datasheet Miller plateau at the
  gate-charge test current (50 A for IPP022, 100 A for IPP019) and at gfs_min at 100 A. The 2.35
  exponent is the value that keeps the tangent above gfs_min (a minimum floor, not a typical fit).
  Above 100 A it is a power-law extrapolation; it saturates at about 890 A at 9.34 V and about
  1170 A at 10.11 V. At VGS 7.5 V it gives about 400 A against the 860 A of the typical transfer
  chart, so for a typical part it saturates too early: it brackets the low-gain end.
- **KP × 0.6** reproduces LTspice's evaluation of the same L0 card and sits between the groups.

The tier spread (886-1993 A, 0.23-0.56 J at 27 V) is wider than the KP bracket and is the main
model uncertainty of Q1. It does not change any verdict.

## Q8: hard LS clamp on the as-built input caps

Hard clamp as in Q1, but on the input caps the boards actually carry: 2 × Rubycon ZLH 470 µF/100 V
(0.94 mF) instead of the schematic's 3 × EKZN 1000 µF. ZLH 100 V 470 µF 16 × 31.5 (datasheet
`~/dev/pv/ee/datasheets/rubycon/ZLH.pdf`): |Z| max 33 mΩ at 20 °C / 100 kHz (0.12 Ω at -10 °C),
rated ripple 2.4 A rms at 105 °C / 100 kHz. The datasheet gives impedance, not ESR, so the model uses
0.65 × 33 mΩ = 21.5 mΩ per cap (10.7 mΩ for the pair) as typical and 33 mΩ (16.5 mΩ) as max. flat
has no 10 µF input MLCCs (3.44 µF left); fry keeps the schematic MLCCs. C10 is fry's ZLH 470 µF
and the schematic 1000 µF EKZN on flat (assumed). 55 V is a hypothetical 16s battery on these boards.
Source: `out/q8_ng.csv`, `out/q8_lt.csv` (`q8_asbuilt.py`).

The "60 °C case" runs set the LS Rds tempco to Tj 60 °C (L0) or hold the L3 model's case at 60 °C;
the ΔTj is the max-Zth Cauer ladder over that fixed case, and "Tj peak" = case + ΔTj. "Vds > 1 V"
is the time the LS conducts (> 1 A) outside the fully-on region; "Cin E" and "C10 E" are
½·C·(V0² - Vend²) of each capacitor bank.

| board | V | model | LS peak A | Vds at peak V | peak power kW | Vds > 1 V µs | Cin E J | C10 E J | LS E J | ΔTj K / Tj peak °C, 25 °C case | ΔTj K / Tj peak °C, 60 °C case |
|---|---|---|---|---|---|---|---|---|---|---|---|
| fry | 27 | ngspice L0, KP × 1, ESR typ | 1195 | 2.4 | 4.8 | 19 | 0.37 | 0.18 | 0.048 | 17 / 42 | 19 / 79 |
| fry | 27 | ngspice L0, KP × 0.6, ESR typ | 1114 | 3.3 | 7.0 | 23 | 0.37 | 0.18 | 0.060 | 22 / 47 | 24 / 84 |
| fry | 27 | ngspice L0, KP × 1, ESR max | 1009 | 2.2 | 4.1 | 17 | 0.37 | 0.18 | 0.040 | 12 / 37 | 14 / 74 |
| fry | 27 | ngspice L0, KP × 0.6, ESR max | 978 | 2.5 | 4.8 | 23 | 0.37 | 0.18 | 0.048 | 15 / 40 | 16 / 76 |
| fry | 27 | LTspice L0, ESR typ | 1086 | 5.2 | 6.7 | 35 | 0.37 | 0.18 | 0.091 | 30 / 55 | 32 / 92 |
| fry | 27 | ngspice L3 (self-heating), ESR typ | 1171 | 3.1 | 5.1 | 22 | 0.37 | 0.18 | 0.054 | 20 / 45 (own node 41) | 23 / 83 (own node 79) |
| fry | 55 | ngspice L0, KP × 1, ESR typ | 1781 | 16.1 | 40.2 | 69 | 1.50 | 0.74 | 0.329 | 140 / 165 | 150 / 210 |
| fry | 55 | ngspice L0, KP × 0.6, ESR typ | 1237 | 34.2 | 43.0 | 83 | 1.50 | 0.74 | 0.609 | 258 / 283 | 261 / 321 |
| fry | 55 | ngspice L0, KP × 1, ESR max | 1736 | 4.4 | 22.1 | 71 | 1.50 | 0.74 | 0.210 | 67 / 92 | 75 / 135 |
| fry | 55 | ngspice L0, KP × 0.6, ESR max | 1216 | 25.7 | 32.0 | 86 | 1.50 | 0.74 | 0.439 | 178 / 203 | 183 / 243 |
| fry | 55 | LTspice L0, ESR typ | 1262 | 33.5 | 43.1 | 120 | 1.50 | 0.74 | 0.653 | 254 / 279 | 258 / 318 |
| fry | 55 | ngspice L3 (self-heating), ESR typ | 1711 | 23.6 | 40.4 | 93 | 1.50 | 0.74 | 0.529 | 206 / 231 (own node 218) | 226 / 286 (own node 272) |
| flat | 27 | ngspice L0, KP × 1, ESR typ | 1165 | 2.2 | 4.9 | 16 | 0.37 | 0.39 | 0.064 | 20 / 45 | 23 / 83 |
| flat | 27 | ngspice L0, KP × 0.6, ESR typ | 1147 | 2.6 | 5.2 | 39 | 0.37 | 0.39 | 0.073 | 24 / 49 | 26 / 86 |
| flat | 27 | ngspice L0, KP × 1, ESR max | 969 | 1.7 | 3.9 | 13 | 0.37 | 0.39 | 0.058 | 15 / 40 | 17 / 77 |
| flat | 27 | ngspice L0, KP × 0.6, ESR max | 958 | 2.1 | 4.1 | 36 | 0.37 | 0.39 | 0.065 | 17 / 42 | 19 / 79 |
| flat | 27 | LTspice L0, ESR typ | 1076 | 4.2 | 5.8 | 100 | 0.37 | 0.39 | 0.128 | 39 / 64 | 41 / 101 |
| flat | 27 | ngspice L3 (self-heating), ESR typ | 1161 | 2.4 | 4.8 | 17 | 0.37 | 0.39 | 0.066 | 22 / 47 (own node 41) | 25 / 85 (own node 79) |
| flat | 55 | ngspice L0, KP × 1, ESR typ | 2259 | 4.5 | 29.6 | 102 | 1.49 | 1.57 | 0.292 | 97 / 122 | 107 / 167 |
| flat | 55 | ngspice L0, KP × 0.6, ESR typ | 1683 | 16.3 | 41.9 | 112 | 1.49 | 1.57 | 0.478 | 225 / 250 | 233 / 293 |
| flat | 55 | ngspice L0, KP × 1, ESR max | 1889 | 3.7 | 19.3 | 103 | 1.49 | 1.57 | 0.261 | 83 / 108 | 93 / 153 |
| flat | 55 | ngspice L0, KP × 0.6, ESR max | 1653 | 7.6 | 28.0 | 114 | 1.49 | 1.57 | 0.347 | 104 / 129 | 114 / 174 |
| flat | 55 | LTspice L0, ESR typ | 1600 | 18.9 | 42.3 | 149 | 1.49 | 1.57 | 0.674 | 253 / 278 | 260 / 320 |
| flat | 55 | ngspice L3 (self-heating), ESR typ | 2195 | 6.7 | 26.5 | 118 | 1.49 | 1.57 | 0.402 | 135 / 160 (own node 137) | 163 / 223 (own node 200) |
| schematic 3 × EKZN 1000 µF | 27 | ngspice L0, KP × 1, ESR typ | 1694 | 4.1 | 10.7 | 92 | 1.18 | 0.39 | 0.227 | 62 / 87 |  |
| schematic 3 × EKZN 1000 µF | 55 | ngspice L0, KP × 1, ESR typ | 1849 | 32.0 | 73.5 | 155 | 4.76 | 1.57 | 2.165 | 611 / 636 |  |

"own node" is the L3 model's self-heating junction node, which uses the vendor's Zth network; the
Tj column uses the max-Zth ladder of the other rows on that run's power.

### Before and after the input-cap change

| | 27 V peak A | 27 V LS E J / ΔTj K | 55 V peak A | 55 V LS E J / ΔTj K |
|---|---|---|---|---|
| schematic 3 × EKZN 1000 µF (KP × 1, ESR typ) | 1694 | 0.23 / 62 | 1849 | 2.17 / 611 |
| fry as built (KP × 1 / 0.6, ESR typ / max) | 978-1195 | 0.04-0.06 / 12-22 | 1216-1781 | 0.21-0.61 / 67-258 |
| flat as built | 958-1165 | 0.06-0.07 / 15-24 | 1653-2259 | 0.26-0.48 / 83-225 |

The as-built peak falls from 1694 A to 958-1195 A at 27 V but stays above ID,pulse (812 A fry,
764 A flat) in every corner, ESR max included. At 55 V it stays at 1.2-2.3 kA.

### Is the input-cap energy enough to destroy the LS?

- **27 V: no, not thermally.** The caps hold 0.37 J (Cin) plus 0.18 J (fry C10) or 0.39 J (flat
  C10). Most of it goes into the capacitor ESR, the shorted die and the copper; the LS takes
  0.04-0.08 J (LTspice 0.09-0.13 J). ΔTj is 12-26 K (LTspice 30-41 K), so Tj peaks at 37-64 °C from
  a 25 °C case and 74-101 °C from a 60 °C case, far below 175 °C.
- **27 V: the binding limit is the peak current.** The LS carries 958-1195 A (1.2-1.6 × ID,pulse)
  for about 13-40 µs out of the fully-on region (LTspice up to 100 µs), at Vds 1.7-3.9 V at the peak
  (LTspice 4.2-5.2 V) and 3.9-7 kW peak power. In that Vds range the 10 µs SOA line is the current
  cap (below). The datasheet does not say what happens above ID,pulse for a few µs at low Vds
  (bond wires, current sharing in the die); it is outside the rating, not a thermal failure.
- **55 V: yes in most corners.** The caps hold 1.5 J (Cin) plus 0.74 / 1.57 J (C10). The LS takes
  0.21-0.67 J with ΔTj 67-261 K. Tj passes 175 °C in the KP × 0.6 rows (except flat with max ESR),
  both LTspice rows and fry's L3 row (203-321 °C; L3 own node 218 °C from a 25 °C case). From a
  60 °C case it also passes for fry KP × 1 with typical ESR (210 °C) and flat L3 (own node 200 °C).
  It stays below 175 °C for KP × 1 with max ESR on both boards, flat KP × 1 typical ESR (122-167 °C)
  and flat L3 from a 25 °C case (137 °C). The peak current (1.2-2.3 kA) at
  Vds 4-34 V is 2-4 × the 10 µs SOA.
- The pre-run scaling estimate (20-35 K at 27 V, 85-135 K at 55 V) matches at 27 V; at 55 V the
  simulated range is wider (67-261 K) because a channel that saturates earlier (KP × 0.6, LTspice)
  holds a higher Vds during the dump.

### SOA readings (chart readings by eye, not digitized)

Diagram 3 of each datasheet (TC = 25 °C, single pulse), the 10 µs line; reading uncertainty about
±15 %.

| part | below the cap | cap (ID,pulse) | above the cap |
|---|---|---|---|
| IPP022N12NM6 (fry LS) | RDS(on) limit line: about 490 A at 2.2 V, 730 A at 3.3 V | 812 A from about 3.6 V to 25 V | about 690 A at 28 V, 580 A at 34 V, 340 A at 57 V, 190 A at 100 V |
| IPP019N08NF2S (flat LS) | RDS(on) limit line: about 470 A at 1.7 V, 710 A at 2.6 V | 764 A from about 2.8 V to 20 V | about 560 A at 28 V, 300 A at 58 V, 180 A at 80 V |

At the simulated Vds at the peak:

| case | Vds at peak V | 10 µs SOA reading A | simulated peak A |
|---|---|---|---|
| fry 27 V, ngspice | 2.2-3.3 | 490-730 (RDS(on) line) | 978-1195 |
| fry 27 V, LTspice | 5.2 | 812 (cap) | 1086 |
| fry 55 V | 4.4-34 | 812 (cap) down to about 580 | 1216-1781 |
| flat 27 V, ngspice | 1.7-2.6 | 470-710 (RDS(on) line) | 958-1165 |
| flat 27 V, LTspice | 4.2 | 764 (cap) | 1076 |
| flat 55 V | 3.7-19 | 764 (cap) | 1600-2259 |

The RDS(on) segment of the SOA is drawn for the maximum RDS(on) at Tj 175 °C; the model conducts
more at the same Vds because it runs at typical RDS(on) and a cool die. That segment is a
conduction bound, not a stress limit, so the relevant comparison is the 812 / 764 A cap.

## Q9: hard LS clamp at low Vin (night)

Hard LS clamp after an HS short with the input side at low voltage: Cin, the shorted HS, the coil and
C10 start at Vin = 1, 2, 3, 5, 8 and 12 V, the panel delivers no current, the battery sits at 27 V on
Bat+ (25 mΩ) and the BF is commanded off, so its gate settles at the static clamp bias (Vgs 1.65 V;
1.79 V with the 10.11 V rail). Same corrected deck as Q1 and Q8: 9.34 V gate drive, extracted
3.2 nH loop with branch R, typical ESR, as-built input caps, no D16. Boards: fry as built (C10 ZLH
470 µF), fry with the schematic C10 1000 µF, and flat (C10 1000 µF assumed). Clamp at 2 ms, 25 ms
after it. Corners: KP × 1 / 0.6, ESR max, the 10.11 V rail, and the BF threshold shifted by
DVTH = -0.5 V (datasheet minimum Vth) and -0.8 V (a minimum part about 60 K warm). Source:
`out/q9_night.csv`, `out/q9_night_dvth.csv` (`q9_night.py base|dvth`); plots `plots/q9_*_Vin5_kp1.0.png`,
`plots/q9_*_Vin12_kp1.0.png`.

### LS

KP × 1, typical ESR. ID,pulse is 812 A (fry, IPP022N12NM6) and 764 A (flat, IPP019N08NF2S).

| board | Vin V | LS peak A (× ID,pulse) | LS I²t A²s | LS ΔTj K | BflowS min V | BF Vgs max V | coil min A |
|---|---|---|---|---|---|---|---|
| fry (C10 ZLH 470 µF) | 1 | 43 (0.05) | 0.03 | 0.0 | -0.9 | 1.73 | -2 |
| fry (C10 ZLH 470 µF) | 2 | 87 (0.11) | 0.13 | 0.1 | -1.8 | 1.80 | -5 |
| fry (C10 ZLH 470 µF) | 3 | 130 (0.16) | 0.28 | 0.2 | -2.7 | 1.88 | -7 |
| fry (C10 ZLH 470 µF) | 5 | 216 (0.27) | 0.79 | 0.5 | -4.5 | 2.08 | -12 |
| fry (C10 ZLH 470 µF) | 8 | 345 (0.42) | 2.02 | 1.3 | -7.2 | 2.46 | -21 |
| fry (C10 ZLH 470 µF) | 12 | 517 (0.64) | 3.99 | 2.9 | -10.6 | 2.84 | -35 |
| fry with C10 1000 µF | 1 | 43 (0.05) | 0.05 | 0.0 | -0.9 | 1.80 | -3 |
| fry with C10 1000 µF | 2 | 87 (0.11) | 0.22 | 0.1 | -1.8 | 1.95 | -7 |
| fry with C10 1000 µF | 3 | 130 (0.16) | 0.49 | 0.2 | -2.7 | 2.09 | -11 |
| fry with C10 1000 µF | 5 | 216 (0.27) | 1.36 | 0.5 | -4.5 | 2.46 | -19 |
| fry with C10 1000 µF | 8 | 346 (0.43) | 2.67 | 1.3 | -7.2 | 2.84 | -34 |
| fry with C10 1000 µF | 12 | 517 (0.64) | 4.66 | 2.9 | -10.6 | 2.99 | -65 |
| flat (C10 1000 µF assumed) | 1 | 42 (0.05) | 0.05 | 0.0 | -0.9 | 1.73 | -4 |
| flat (C10 1000 µF assumed) | 2 | 83 (0.11) | 0.20 | 0.1 | -1.8 | 1.81 | -9 |
| flat (C10 1000 µF assumed) | 3 | 125 (0.16) | 0.46 | 0.2 | -2.6 | 1.90 | -13 |
| flat (C10 1000 µF assumed) | 5 | 208 (0.27) | 1.28 | 0.6 | -4.3 | 2.10 | -24 |
| flat (C10 1000 µF assumed) | 8 | 333 (0.44) | 3.27 | 1.6 | -6.8 | 2.45 | -46 |
| flat (C10 1000 µF assumed) | 12 | 500 (0.65) | 6.32 | 3.6 | -9.8 | 2.86 | -93 |

- The LS stays inside its ratings over the whole range: the peak is at most 0.65 × ID,pulse at 12 V
  and ΔTj at most 4 K. KP × 0.6 and max ESR lower the peak (510 / 436 A at 12 V on fry).
- Neither datasheet gives an I²t rating; the column is for comparison between cases. The ΔTj from
  the Zth ladder is the thermal check.
- The LS is not what limits the clamp at night. The peak scales with Vin (about 43 A/V); extrapolated,
  the LS reaches ID,pulse at about 19 V (fry) and 18 V (flat) on this deck (not simulated).

### BF

The clamp pulls BflowS to about -0.88 × Vin (-0.9 V at 1 V, -10.6 V at 12 V) while the BF gate
stays near its static bias, so the BF Vgs rises from 1.65 V to 1.7-3.0 V. Once that exceeds Vth, the
battery discharges through the BF into the clamp, with the BF at 28-38 V Vds: a linear-mode pulse.
Cells give the BF peak current (all three dies) / the charge taken from the battery.

| board | Vin V | Vth nominal | Vth nominal, rail 10.11 V | DVTH -0.5 (datasheet min) | DVTH -0.8 (min, hot) | DVTH -0.8, rail 10.11 V |
|---|---|---|---|---|---|---|
| fry (C10 470 µF) | 1 | 0 | 0 | 0 | 0 | 0.5 / 0.22 |
| fry (C10 470 µF) | 2 | 0 | 0 | 0 | 0 | 2.1 / 0.66 |
| fry (C10 470 µF) | 3 | 0 | 0 | 0 | 1.0 / 0.20 | 3.8 / 1.13 |
| fry (C10 470 µF) | 5 | 0 | 0 | 0 | 6.4 / 1.38 | 9.4 / 2.28 |
| fry (C10 470 µF) | 8 | 0 | 0 | 9.7 / 1.77 | 19.7 / 3.56 | 21.4 / 4.44 |
| fry (C10 470 µF) | 12 | 25.2 / 2.39 | 27.2 / 2.76 | - | - | - |
| fry, C10 1000 µF | 1 | 0 | 0 | 0 | 0 | 2.3 / 1.04 |
| fry, C10 1000 µF | 2 | 0 | 0 | 0 | 3.2 / 0.95 | 6.9 / 2.40 |
| fry, C10 1000 µF | 3 | 0 | 0 | 0 | 8.6 / 2.25 | 11.9 / 3.72 |
| fry, C10 1000 µF | 5 | 0 | 0 | 10.0 / 2.75 | 20.2 / 5.77 | 23.0 / 7.38 |
| fry, C10 1000 µF | 8 | 25.7 / 3.57 | 29.0 / 4.54 | 43.8 / 7.48 | 51.2 / 9.84 | 53.1 / 11.77 |
| fry, C10 1000 µF | 12 | 69.7 / 9.87 | 71.9 / 10.23 | - | - | - |
| flat | 1 | 0 | 0 | 0 | 0 | 0.9 / 0.40 |
| flat | 2 | 0 | 0 | 0 | 0 | 2.9 / 1.38 |
| flat | 3 | 0 | 0 | 0 | 2.0 / 0.39 | 5.1 / 2.38 |
| flat | 5 | 0 | 0 | 0 | 11.3 / 2.72 | 15.3 / 4.82 |
| flat | 8 | 0 | 0 | 17.7 / 3.42 | 33.7 / 7.54 | 36.6 / 9.57 |
| flat | 12 | 31.3 / 3.40 | 35.1 / 4.30 | - | - | - |

- KP × 0.6 and max ESR do not change the BF current (fry 12 V: 25.1 / 25.2 A, flat 31.0 / 31.3 A).
- The pulses at nominal and datasheet-min Vth last 0.6-2.7 ms. The DVTH -0.8 V pulses have a long
  low-current tail (BF conducting for 16-24 ms), which carries most of the charge.
- BF energy per event: 0.08-0.30 J at 12 V, 0.05-0.22 J at 8 V (DVTH -0.5 V), 0.006-0.07 J at
  1-3 V (DVTH -0.8 V corners).
- The battery charge (0.2-12 mC) is irrelevant for the battery; the BF stress is the issue.

**Against the BF ratings.** The TPH1R306PL SOA reading (Fig. 8.16, by hand) gives about 1-1.5 A at
1 ms and 5 A at 100 µs per die at 30-40 V. Three dies in linear mode do not share evenly (one Vth
sets the current), so a single die can take most of it. Read against one die:
- 25-72 A for 0.6-2.2 ms (12 V; 8 V on fry with C10 1000 µF) is 15-50 × the 1 ms line: outside the
  SOA.
- 10-18 A for about 2 ms (8 V at DVTH -0.5 V; 5 V on fry with 1000 µF) is also outside, and so is
  6-53 A with a 17-24 ms tail (DVTH -0.8 V, 5-8 V).
- 1-2 A peaks with a 20 ms tail (3 V, DVTH -0.8 V; 1 V with the 10.11 V rail) sit near the 1 ms line
  at the peak and carry 6-28 mJ. They need the 10 ms line, which the chart does not resolve well; on
  energy they are well below the 1 ms line's about 40 mJ per die.

### Highest safe Vin

| case | highest Vin with no BF conduction | first Vin with BF current |
|---|---|---|
| nominal Vth, 9.34 or 10.11 V rail | fry (470 µF) and flat: 8 V; fry 1000 µF: 5 V | fry 470 µF: 12 V (25 A); flat: 12 V (31 A); fry 1000 µF: 8 V (26 A) |
| datasheet-min Vth (DVTH -0.5 V) | fry (470 µF) and flat: 5 V; fry 1000 µF: 3 V | 8 V (10-18 A); fry 1000 µF 5 V (10 A) |
| min Vth, warm (DVTH -0.8 V) | fry (470 µF) and flat: 2 V; fry 1000 µF: 1 V | 3 V (1-2 A); fry 1000 µF 2 V (3 A) |
| min Vth, warm, 10.11 V rail | none | 1 V (0.5-2.3 A, 6-28 mJ) |

- **LS:** within ratings up to 12 V with at least 35 % current margin; on its own it would allow
  about 18 V.
- **BF:** the limit. With margin (no conduction at datasheet-min Vth): **Vin ≤ 5 V** on fry as built
  and on flat, **≤ 3 V** on fry with a 1000 µF C10. A part whose Vth is at the minimum and warm
  starts to conduct at 3 V (2 V with a 1000 µF C10), with small energy at 3 V on fry as built and
  flat (6-11 mJ) and SOA violations from 5 V.
- The BF only conducts because its "off" gate sits at the static clamp bias (1.65-1.79 V). A gate
  held at 0 V relative to BflowS during the clamp would remove this path; then the LS sets the limit.
- In the dark a panel sits near 0 V and Cin discharges through it and the dividers, so a clamp then
  is safe. At
  dusk or dawn, Vin of 5-12 V with little current is the case that drives the BF out of its SOA.

## sw_phases notes

- `mode_untestable` is the 300 ns min_dwell against off spans below 15 µs (D > 0.415). For those
  windows the mode rests on the coil current, which is model-internal.
- `subintervals` accepts a 5 ns "on" span (an edge ring); that produced the DCM(0.654) verdict in
  the 2026-11-20 Q2 run. Reported here, not fixed.

## Limitations

- The LS ΔTj uses the max-Zth Cauer ladder over a fixed case (25 °C; Q8 also 60 °C), with no
  repeated-pulse case heating. At kA levels it shows the trend; it is not a failure prediction.
- The LS channel is used far above its 812 A pulse rating. No model tier is anchored above
  ID,pulse (Q7); the KP bracket and the tiers span the uncertainty.
- The HS model is a generic Level 1 (it matters only before the short). The short is a die
  resistance plus package; the real failure mode of a shorted die is not modelled.
- The coil saturation law is extrapolated below about 25 % µ; checked against bench ripple only up
  to 30 A. No core loss or AC winding resistance (overstates the ring).
- The BF square-law model has no subthreshold region; the EKV leak estimate has an assumed slope
  factor. No thermal feedback in linear mode.
- Cgd is fixed on the LS and BF.
- ESR typical factor 0.65 is a Chemi-Con model ratio, not a measurement of these caps. ESL, battery
  R/L, F1 resistance and panel cable values are assumptions. Fuse melting is not modelled.
- As-built boards: the PCB is common, but flat's C10, BF FETs and BF gate network are not documented
  and use the schematic values. Whether fry has an ST15100S fitted is in conflict (fisi.md vs Fab).
- Q3 runs at a fixed duty with no controller.
- Q5 assumes the pulse widths are realised as commanded (34-50 MCPWM counts); the firmware's minimum
  pulse and dead-time logic are not modelled.
- Offload: pgx93 and vup have no ngspice and `offload run` only runs sha-stamped mppt-sim builds, so
  every run was local with at most 4 workers.

## Unverified

- LS ΔTj and energy above ID,pulse (model tiers differ, Q7).
- The 27 V 30-100 ms ramp margin (0.2-1.6 % to ID,pulse; rail-high corner not run for ramps).
- BF static leakage at 1.49-1.77 V (EKV estimate, slope factor assumed, no thermal feedback).
- BF avalanche at 55 V and in bfcoil (BV 61 V body diode; EAS rating not exercised in the model).
- TPH1R306PL transfer fit and SOA readings (hand digitization).
- Coil saturation below 25 % µ.
- The added BflowS Schottky at hundreds of amperes (outside its model's validated range).
- Q5 pulse widths and rate on real MCPWM hardware; the LS gate rise time t_g on the board.
- sw_phases `mode_untestable` windows and the CCM boundary per sw_phases.
- Sensor offset and ADC noise at the 0.3 A Q4 threshold; the AS333 clip at -29.8 A.
- The UCC21330 dead time of 185 ns (formula).
- flat's C10 and BF network; fry's D16 / ST15100S.
- ZLH ESR (0.65 × |Z| max as typical; the datasheet gives impedance only).
- Q9 BF verdict: the BF Vth of the fitted parts, current sharing between the three dies in linear
  mode, and the TPH1R306PL SOA beyond the 1 ms line (the 10 ms line was not read).
- Q6c at 76 V ran at 21.5 A instead of 25 A (duty step overshot); all positions at the same duty.

## Open hardware measurements

These decide the verdicts above:

1. **"+12V" rail on fry and flat** under load (sets the LS gate drive, the clamp peak, the pulse
   peak and the static BF bias).
2. **LS gate rise** (t_g for 215-315 ns pulses) and the commutation loop inductance on the board,
   for example with a single LS pulse into a charged Cin at low voltage.
3. **BF Vth** of the fitted TPH1R306PL parts (sets the static leak at Vgs 1.5-1.8 V and the highest
   Vin at which a hard clamp leaves the BF off, Q9: 5 V at datasheet-min Vth, 2-3 V for a warm
   minimum part).
4. **As-built C10 and BF network** on flat (C10 part and ESR, BF FETs, R21/R22/R24/C20/DZ2), and
   whether D16 / an ST15100S is fitted on fry.
