*this document is an LLM generated placeholder*

# LibreSolar MPPT 2420 HC: HS-short fuse-blow crowbar, SPICE study (2026-11-21)

Study folder (decks, scripts, plots, CSVs): `/Users/fab/dev/pv/ee/dcdc-tools/examples/libresolar-2420hc-fuse-blow/`
(README there has one command per question). Not committed. No device was contacted.

Questions:

1. HC: when the LS is turned on into a shorted HS, what does the input-cap dump do to the LS, how fast does F1
   clear from the battery, and which comes first: fuse clearing or the LS exceeding its ratings?
2. HC: what happens after F1 opens (steady dissipation, spikes vs the LS Vds rating)?
3. HC: sensitivities (die-short R, battery R and L, fuse tolerance, KP, gate rail, ESR typ/max/cold, case
   temperature, coil model, supply model).
4. Fugu2: would LS on plus BF on clear the 40 A F1 at 27 V and 55 V, and does a fuse-blow scheme change the
   Cin-dump failure?

## Summary of findings

- **The firmware crowbar never fires (code reading, not simulated).** `Dcdc::fuse_destruction()` sets duty 0, which
  writes CCR1 = 0; `half_bridge_start()` only sets MOE when `TIM1->CCR1 != 0`. CCR1 reads back the preload value 0,
  so MOE stays 0 and both gates stay off. Details in "Firmware" below.
- **If it did fire (simulated), the LS exceeds its ratings long before F1 clears.**
  - 12 V system: the LS takes a 0.59 kA Cin dump (2.3x ID,pulse 256 A, only 4 K rise), then carries about 305 A
    from the battery. It reaches Tj 175 C 2.5 ms after turn-on. F1 (25 A ATOF) melts at 14.1 ms and clears at
    about 14.8 ms. Time ratio 5.6.
  - 24 V system: the dump is 1.13 kA (4.4x ID,pulse, 81 K rise). Tj 175 C is reached at 0.34 ms with 595 A; F1
    melts at 5.4 ms. Time ratio 15.7.
  - Battery R 10-50 mOhm, fuse tolerance 0.7-1.5x, KP 1/0.9/0.6, ESR typ/max/cold and the gate-rail corners do
    not reverse the order (Q3 table). Extrapolated Tj at the melt instant is a rise of 540 K (12 V) and 1500 K
    (24 V): the real LS fails before the fuse melts. Whether it then fails short (and still blows F1) or open is
    outside the model.
- **With the real supply chain at 12 V, the result depends on the assumed MCU brown-out threshold.** The crowbar
  pulls VBUS to about 4 V. With the assumed 3.8 V MCU-off level the drive holds until after the melt; with 4.3 V,
  12.0 V battery EMF or 50 mOhm battery R, the MCU browns out first (1.9-2.5 ms), the LS turns off at 205-306 A and
  F1 does not melt.
- **After F1 opens (Q2):** with the LS held on, the panel stays shorted (11 A, negligible dissipation). Once the
  LS drive dies, the panel lifts HV+, SW and VBUS to Voc (74.4 V): within the LS's 100 V rating but 2.1x the 35 V
  rating of the output caps C5/C27, with no battery left to clamp. LS turn-off spikes are 22-30 V. VBUS
  swings to -5.5 V (-33 to -43 V with a non-saturating coil) as the L1 current decays after the fuse opens.
- **Fugu2 (Q4):** LS-only does nothing to F1, because the BF blocks battery current (F1 I²t about 5 A²s). With BF
  and LS on at 27 V the 40 A fuse (modelled as ATOF 0287040, 3300 A²s) melts at 9.6-12.8 ms (5.2-6.8 ms with BF
  on 20 ms early), but the LS reaches Tj 175 C first in every as-built and schematic case (0.8-8.3 ms), except at
  50 mOhm battery R. At 55 V the Cin dump alone takes the LS past Tjmax within about 10 µs on the schematic and
  fry builds, the BF avalanches (Vds 61 V on a 60 V part), and a 32 V-rated blade fuse is outside its DC interrupt
  rating. **The fuse-blow scheme does not address the Cin-dump failure:** the dump does its damage within
  microseconds, before any fuse I²t accumulates (3 A²s of 3300 when the LS passes its limit at 55 V).

## Firmware

Source: github.com/LibreSolar/charge-controller-firmware, main, downloaded 2026-11-21 (untrusted, only read).
`app/src/half_bridge.c` sha256 1f99374c…a6bc701, `app/src/dcdc.cpp` sha256 9e4f22ba…5814d82.

- Detection, `Dcdc::check_hs_mosfet_short()` (dcdc.cpp:186): half bridge off, `inductor_current > 0.5 A` and
  `lvb->voltage_filtered > lvb->sink_control_voltage()`, for more than 10 s, sets `ERR_DCDC_HS_MOSFET_SHORT`.
- `main.cpp:111-114`: with `CONFIG_HS_MOSFET_FAIL_SAFE_PROTECTION` (Kconfig default y, help text: "trip the fuse
  by switching the low-side MOSFET continuously on"), `dcdc.fuse_destruction()` runs every second.
- `Dcdc::fuse_destruction()` (dcdc.cpp:363): after more than 20 calls it does `half_bridge_stop()`,
  `half_bridge_init(50, 0, 0, 0.98)`, `half_bridge_set_duty_cycle(0)`, `half_bridge_start()`, on every later call.
- `half_bridge_set_ccr()` writes `TIM1->CCR1 = clamp_ccr(ccr)`; with `tim_ccr_min = 0` that is 0.
- `half_bridge_start()` (half_bridge.c:188-195): `if (TIM1->CCR1 != 0U) TIM1->BDTR |= TIM_BDTR_MOE;`.
- The timer runs with OC1PE = 1 (half_bridge.c:149). RM0440 Rev 9 §29.6.16 (TIMx_CCR1): with preload enabled the
  register reads the preload value. It is 0, so MOE is never set; both outputs stay inactive. The crowbar does not
  happen, and charging through the shorted HS continues uncontrolled.
- If the guard were absent: RM0440 PWM mode 1, "If the compare value is zero then tim_ocxref is held at 0", so
  OC1 = 0 and OC1N = 1 continuously: the LS would be on without dead-time events. Each later 1 s call does
  stop/init/start, which would drop the LS for a few µs every second (not simulated).

## Board model (HC rev 0.2.3)

Source: github.com/LibreSolar/mppt-2420-hc main, `kicad/mppt-2420-hc.kicad_pcb` sha256 9cc787c4…1b28b. Nets read
from pad nets with `tools/pcb_pads.py` (the legacy `.sch` did not export a netlist). BOM values from the repo BOM.

| part | value used | provenance |
|---|---|---|
| Q1 HS, Q2 LS, Q3 (HV- leg) | IPA045N10N3G (TO-220 FullPAK, 100 V) | PCB/BOM |
| Q3 gate | from LS_DRV via D7, R56 1M pulldown; modelled as a 5 mOhm switch at Vgs > 6 V plus body diode | PCB; switch is an assumption |
| L1 | 53 µH, 2x MS-130060-2 sendust 60µ, AL 61, 21 turns, 40x AWG27 | schematic note |
| R6 | 2 mOhm shunt SW side of VBUS | PCB/BOM |
| C1, C2 | 2x 560 µF 100 V EKZN101ELL561MM25S | BOM (PCB value field says 390 µF; `cbulk390` case) |
| C3, C4 | 1 µF + 100 nF MLCC, modelled as 0.8 µF | BOM; DC-bias derating assumed |
| C17, C6 snubber | 10 nF HV+/HV-; 2.2 nF + R4‖R12 = 2.35 Ohm | BOM |
| RV2 | MOV-10D820K across J1 (82 V at 1 mA, 135 V at 5 A) | BOM; power-law fit |
| C5, C27 | 2x 680 µF 35 V EEU-FR1V681 on VBUS | BOM |
| F1 / XF1 | Littelfuse ATOF 0287025 (25 A) between VBUS and BAT+ | BOM |
| battery switch | none: battery to F1 to VBUS to R6 to L1 to SW | PCB nets |
| LM5107 | R2 3.3 Ohm LS gate, D15 turn-off bypass; driver R 2.5 Ohm | PCB; driver R assumed |
| +12V | T10 BC846 follower on D5 BZX84C12 (11.4/12.0/12.7 V) = 11.35 V nominal | PCB, BZX84C12 tolerance |
| SUPPLY_INPUT | VBUS via D10 1N4148, or HV+ via Q4 BUK98180 follower with D9 47 V zener | PCB |

### Values and their provenance

| quantity | value | source | status |
|---|---|---|---|
| IPA045N10N3G ID,pulse | 256 A (TC 25 C) | datasheet Rev 2.4, 2016-01-22, fetlib `IPA045N10N3_G.pdf` sha256 91c1f53b… | table value |
| RthJC, V(BR)DSS, EAS | 3.8 K/W; 100 V; 540 mJ | same | table values |
| Zth ladder | vendor Cauer (Zthtype 1 = max) from `IPA045N10N3` thermal subcircuit, sums to 3.80 K/W | Infineon OptiMOS3 100 V LTspice lib | vendor model; visually consistent with Diagram 4 (visual check only) |
| LS channel | Infineon `IPA045N10N3_L0`: KP 266.6, VTO 3.63, Rs 0.748 mOhm, Rd 2.61 mOhm TC 8m/K, Lg 4 n, Ld 2.5 n, Ls 1.8 nH; fixed Cgd 1 nF | vendor model, reduced | reduction as in the Fugu2 study |
| KP anchor | dsdig transfer, p.6 Diagram 7, 25 C: 72.2/146.4/190.6 A at Vgs 4.5/5.0/5.25 V. ngspice at Vds 10 V: KPF 0.9 gives 79.6/153.1/194.7 A (+10/+5/+2 %); KPF 1: 87/168/213 A; KPF 0.6: 55/107/136 A | dsdig commit bc9d3ac | **unverified** (status overlay-review-required; axes grid-verified) |
| Idsat at Vgs 11.35 V | 1.65 / 1.52 / 1.08 kA at KPF 1 / 0.9 / 0.6 | Level-3 extrapolation | extrapolated far above ID,pulse |
| HS short | die R 5 mOhm (swept 1-100 mOhm) + package Ld 2.5 n + Ls 1.8 nH, 0.75 mOhm | vendor _L0 leads; die R assumed | assumption |
| F1 0287025 | cold R 2.52 mOhm typ; melting I²t 1000 A²s typ ("from breaking capacity tests, melting time before arcing"); interrupt rating 1000 A at 32 V DC; 350 % 0.08-0.5 s, 600 % 0.1 s max | Littelfuse ATOF datasheet rev 02/04/2025, `sources/littelfuse_ATOF_287_datasheet_rev2025-02-04.pdf` sha256 4640faa7… | table values; the time-current rows agree with adiabatic I²t |
| F1 model | I²t integrator; R = Rcold·(1 + kh·I²t/I²t_melt), kh 1 (0-3); after melt an exponentially growing arc R, tau 100 µs (20-500 µs); clear = melt + 7 tau | assumption | the arc model is not validated |
| C1/C2 ESR | typ 0.68 x 27 mOhm = 18.4 mOhm; max 27 mOhm; cold (-10 C) 0.68 x 81 mOhm = 55 mOhm; ESL 8 nH | KZN datasheet: Zmax 0.027 Ohm at 100 kHz 20 C, 0.081 Ohm at -10 C; typ/max ratio and ESL from the Chemi-Con ladder of the 390 µF 16x25 sibling (R1 19 mOhm vs Zmax 28 mOhm) | max and cold from the datasheet; typ ratio borrowed from the sibling |
| C5/C27 ESR | typ 0.65 x 20 mOhm each; max 20 mOhm; cold 3x typ | FR-A sheet ABA0000C1259 (1-Sep-25), EEUFR1V681 10x20, Z 0.020 Ohm, 2180 mA; `sources/panasonic_FR-A_ABA0000C1259.pdf` sha256 47cf1846… | Z from the datasheet; 0.65 typ ratio and the cold factor 3 are assumptions (FR-A cold Z not read) |
| MLCC | 0.8 µF, 5 mOhm, 0.6 nH | assumption | DC-bias derating and ESR/ESL not sourced |
| L1 DCR | 6.6 mOhm (bracket 5-8 mOhm) | computed: 21 turns of 40x AWG27 (3.96 mm²), about 1.49 m per turn set on assumed core dims OD 33.0 / ID 19.9 / HT 2x 10.67 mm (Magnetics 130 size class), copper at 20 C | **derived**: the core size is an assumption (no core dimensions in the HC repo); not measured |
| L1 saturation | KS60 law a 3.57e-4, c 1.748, le 8.125 cm, N 21, floor L0/µ | KDM KS130-060A fit from the Fugu2 study, used by analogy (same OD and permeability class) | **assumption**, extrapolated below 25 % µ |
| L1 model scope | DCR plus saturation only: no core loss, no AC resistance | | see bias note below |
| Battery | LFP 4s/8s at 3.3 V/cell: 13.2 / 26.4 V; R 25 mOhm (10-50); cable L 2 µH (1-5); C8 10 nF | assumption | |
| Panel | Isc 11 A, Voc 75 V (40 V and 12 A cases), cable 5 µH | assumption | |
| Rail (board) | C_sup 5 µF, P_mcu 0.16 W, MCU-off 3.8 V (4.3 V case), C_p12 3.5 µF, LM5107 IDD 0.4 mA, UVLO falling 6.4 V | LM5107 SNVS333F (UVLO 6.0/6.9/7.4 V rising, 0.5 V hysteresis; IDD 0.3-0.6 mA; LI pulldown), `sources/ti_lm5107_SNVS333F.pdf` sha256 e9af4244…; the rest **assumed** | |
| LS case | Tcase 40 C (25 and 70 C cases); Tjmax 175 C, i.e. 135 K allowed rise | assumption; Tjmax from datasheet | |

**Coil-model bias.** Without core loss and AC resistance the modelled L1 path is slightly less lossy than the real
coil. During the ms-scale crowbar the current is nearly DC, so the missing AC/core terms are small. The bias is
towards a slightly higher battery current: F1 melts a little sooner and the LS carries a little more, so both the
fuse time and the LS stress lean pessimistic for the LS and optimistic for the fuse, by an amount below the DCR
5-8 mOhm bracket (that bracket moves the 12 V time ratio 6.1 to 5.3). After the fuse opens, real core loss would
damp the VBUS ring, so the modelled -5.5 V undershoot is an upper bound in magnitude for the saturating coil.

### Board copper (copper-only re-extraction)

Config `parasitics/examples/mppt-2420-hc-perDev-noLeads.yaml` (pitch 2 and 1 mm, lead_mm 0, per-device, Cin
C1-C4, lf 70 kHz, matrix). The pitch-1 run (8403 nodes, about 24 G of FastHenry work) was stopped by the 2 h background
time limit without output (not restarted); the decks use the pitch-2 twin `mppt-2420-hc-perDev-noLeads-p2.yaml`, output
`parasitics/out/mppt-2420-hc-perDev-noLeads-p2/` (extractor commit 92207b79; the `parasitics` working tree has
someone else's uncommitted edit of `examples/mppt-2420-hc.yaml`, not used here). The first launch was stopped by
the 30-min background time limit and restarted.

| quantity | value |
|---|---|
| L_loop, 4 caps in parallel (7 MHz plateau) | 3.04 nH |
| single nearest cap (C3) | 3.20 nH |
| per-cap port L at 7 MHz: C3 / C4 / C1 / C2 | 3.20 / 3.40 / 4.76 / 4.74 nH |
| per-cap port L and R at 70 kHz: C3 / C4 / C1 / C2 | 3.86 nH 1.75 mOhm / 4.16 nH 1.90 mOhm / 5.54 nH 2.11 mOhm / 5.56 nH 2.45 mOhm |
| R_loop (7 MHz plateau) / at 70 MHz | 3.92 mOhm / 18.08 mOhm |
| shared + branch reduction at 70 kHz (used in the deck) | shared 3.49 nH, 1.30 mOhm; branches C3 0.37 nH 0.45 mOhm, C4 0.67 nH 0.59 mOhm, C1 2.04 nH 0.81 mOhm, C2 2.06 nH 1.15 mOhm |
| CSI_hs / CSI_ls | 0.50 / 0.40 nH |
| gate loops HS / LS | 6.37 / 4.31 nH |
| r_hs, r_ls, r_loop_cond | not produced by this config (`cond_ref` None) |
| warnings | skin ladder fits R(f) only to 35.7 % (tol 10 %); at pitch 1 one pad-land terminal fell back to point-style pad nodes |

TO-220 leads are added in the deck: HS short 2.5 + 1.8 nH and 0.75 mOhm, LS inside the vendor subcircuit (Ld 2.5,
Ls 1.8, Lg 4 nH; Rs 0.748 mOhm), all from Infineon `IPA045N10N3_L0`.

The earlier HC runs used placeholder copper and had an initial-condition error (battery-inductor sign, missing ICs
on C8/C17/C6, no IC on the shared and HS inductors) that gave a 1 kA spike at t = 0. Both are fixed; all HC tables
below are from the rerun (`out/q1_base.csv`, `out/q1_sens.csv`; the old CSVs are in `out/prev_placeholder_copper/`).
The dump peak moved from 614 to 588 A (12 V) and 1178 to 1129 A (24 V); the timing results moved by under 2 %.

## Method

Each case starts from the pre-fault steady state: converter off, panel current (11 A) flowing through the shorted
HS, L1, R6 and F1 into the battery. The LS turns on at t_clamp = 1 ms through the LM5107 path. Times below are
from LS turn-on. The LS junction rise comes from the vendor Cauer ladder driven by the LS dissipation, case held
at Tcase; Rd follows Tj (electro-thermal, one-way per step; `noET` uses static 25 C). "Margin" is
t_melt / t_Tjmax: above 1 the LS reaches 175 C before F1 melts.

Rails: `ideal` holds LO at 11.35 V for ever (the firmware's intent). `board` models SUPPLY_INPUT and +12V hold-up,
and latches LO off when the MCU browns out (SUPPLY_INPUT below MCU-off) or VDD falls below the LM5107 UVLO.

ngspice 46, tmax 50 ns, reltol 1e-3. Convergence (`out/conv_check.txt`): tmax 20 ns and reltol 1e-4 change the
results by under 0.02 %. `sw-phases` was not used: there is no PWM switching in any case (the LS is held on), so
it has nothing to classify. `~/bin/offload` could not run these jobs (pgx93/vup have neither ngspice nor
FastHenry, and `offload run` only accepts binaries built with `offload build-sim`); all runs were local, at most 4
workers.

## Q1: Cin dump and the race between F1 and the LS (HC)

Base matrix, battery R 25 mOhm, Tcase 40 C, typical ESR, KP bracket (`out/q1_base.csv`).

| case | dump LS peak (A) | dump I²t LS (A²s) | dump E LS / HS (mJ) | dump ΔTj (K) | I_bat pk (A) | LS 175 C at (ms) | F1 I²t then (A²s) | I_LS then (A) | F1 melt (ms) | F1 clear (ms) | margin | LS rise at melt (K) | kill (ms) | I_LS at kill (A) | kill Vds pk (V) | LS Vds after (V) | VBUS min / max (V) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| V12_ideal_kp1.0 | 590 | 5.3 | 24 / 30 | 4.3 | 312 | 2.53 | 202 | 305 | 14.0 | 14.7 | 5.5 | 536 | – | – | – | 3.8 | -5.5 / 13.5 |
| V12_ideal_kp0.9 | 588 | 5.3 | 24 / 30 | 4.3 | 311 | 2.50 | 199 | 305 | 14.1 | 14.8 | 5.6 | 542 | – | – | – | 3.8 | -5.5 / 13.5 |
| V12_ideal_kp0.6 | 577 | 5.2 | 26 / 29 | 4.6 | 308 | 2.33 | 180 | 303 | 14.5 | 15.2 | 6.2 | 571 | – | – | – | 4.0 | -5.4 / 13.5 |
| V12_board_kp1.0 | 590 | 5.3 | 24 / 30 | 4.3 | 312 | 2.53 | 202 | 305 | 14.1 | 14.8 | 5.6 | 541 | 14.4 | 170 | 22.2 | 74.4 | 2.1 / 74.4 |
| V12_board_kp0.9 | 588 | 5.3 | 24 / 30 | 4.3 | 311 | 2.49 | 198 | 304 | 14.2 | 14.9 | 5.7 | 547 | 14.4 | 167 | 21.6 | 74.4 | 2.0 / 74.4 |
| V12_board_kp0.6 | 577 | 5.2 | 26 / 29 | 4.6 | 308 | 2.32 | 179 | 302 | 14.6 | 15.3 | 6.3 | 579 | 14.9 | 152 | 20.9 | 74.4 | 1.3 / 74.4 |
| V24_ideal_kp1.0 | 1135 | 38.7 | 212 / 108 | 80.4 | 590 | 0.34 | 49 | 597 | 5.3 | 6.0 | 15.5 | 1499 | – | – | – | 12.7 | -4.7 / 26.7 |
| V24_ideal_kp0.9 | 1129 | 38.5 | 215 / 107 | 81.4 | 589 | 0.34 | 48 | 595 | 5.4 | 6.1 | 15.7 | 1508 | – | – | – | 12.7 | -4.7 / 26.7 |
| V24_ideal_kp0.6 | 1068 | 37.1 | 231 / 104 | 86.9 | 581 | 0.33 | 43 | 579 | 5.6 | 6.3 | 16.8 | 1558 | – | – | – | 13.0 | -4.6 / 26.7 |
| V24_board_kp1.0 | 1135 | 38.7 | 212 / 108 | 80.4 | 590 | 0.34 | 49 | 597 | 5.3 | 6.0 | 15.5 | 1499 | 9.3 | 17 | 6.9 | 74.4 | -4.7 / 74.4 |
| V24_board_kp0.9 | 1129 | 38.5 | 215 / 107 | 81.4 | 589 | 0.34 | 48 | 595 | 5.4 | 6.1 | 15.7 | 1508 | 9.3 | 17 | 7.0 | 74.4 | -4.7 / 74.4 |
| V24_board_kp0.6 | 1068 | 37.1 | 231 / 104 | 86.9 | 581 | 0.33 | 43 | 579 | 5.6 | 6.3 | 16.8 | 1558 | 9.3 | 18 | 7.1 | 74.4 | -4.6 / 74.4 |

- Cin dump: 0.59 kA at 12 V, 1.13 kA at 24 V, against ID,pulse 256 A. The equivalent width is about 15 µs
  (I²t / Ipk²). At 12 V the dump heats the die by only 4.3 K; at 24 V it uses 81 K of the 135 K budget.
- The LS SOA chart (datasheet Diagram 3) has no dsdig class; the SOA comparison rests on ID,pulse and the
  vendor Zth ladder only (unverified for single pulses above ID,pulse).
- Crowbar: about 311 A (12 V) and 589 A (24 V) from the battery, limited by battery R, F1, R6, L1 DCR and the LS.
- F1 I²t when the LS reaches 175 C: 199 of 1000 A²s (12 V) and 48 A²s (24 V).
- Plots: `plots/q1_V12_ideal_kp0.9.png`, `q1_V24_ideal_kp0.9.png`, `q1_V12_board_kp0.9.png`,
  `q1_V24_board_kp0.9.png`.

## Q2: after F1 opens (HC)

- LS still driven (`ideal`): the panel stays shorted through HS and LS at Isc; LS dissipation is about 11² x Rd,
  around 1 W. LS Vds after the arc is at most 3.8 V (12 V) and 12.7 V (24 V).
- LS drive lost (`board`): at 12 V the drive dies 0.3 ms after the melt with 167 A still in the LS (L1 current);
  the turn-off spike is 22 V. At 24 V it dies at 9.3 ms (17 A, 7 V). Then the panel lifts HV+, SW and VBUS to
  Voc, 74.4 V. That is below the LS's 100 V V(BR)DSS, but 2.1x the 35 V rating of C5/C27, and anything on VBUS
  (the load output via Q7) sees panel voltage. The MOV RV2 (82 V) does not conduct at 74 V.
- Early brown-out (12 V board cases rbat50, mcuoff4.3, vbatlow): the LS turns off at 205-306 A before any melt;
  the turn-off spike is 23-30 V and the battery stays connected, so charging through the shorted HS resumes.
- BAT+ peak at the fuse opening: 13.5 V (12 V system, tau 100 µs), 18.9 V with a fast arc (tau 20 µs); 34.8 V on
  the 24 V system with tau 20 µs.
- VBUS undershoot: -5.5 V (12 V) and -4.7 V (24 V) as L1 drains C5/C27 through the LS; -8.6 V with a fast arc;
  -0.9 V with tau 500 µs; -33 V and -43 V with a linear (non-saturating) coil. C5/C27 are polarised.

## Q3: sensitivities (HC)

`out/q1_sens.csv`, KPF 0.9, deviations from the base one at a time. The four rail-lo/hi rows were rerun after a
fix (the first sweep changed only Vz, which the `ideal` rail ignores); the rest are from the full sweep.

Case keys: rbat10/50 battery R; lbat1/5 cable L (µH); rdie1m/20m/100m HS die-short R; i2t0.7/1.5 melting I²t factor; kh0/3 pre-melt R rise; tau20u/500u arc time constant; esrmax/esrcold C1/C2 and C5/C27 ESR (see the provenance table); dcr5m/8m L1 DCR; rail-lo/hi D5 corners (Vz 11.4/12.7 V, LO 10.75/12.05 V); tcase25/70; noET static 25 C Rd; voc40, isc12 panel; lin-coil no saturation; cbulk390 2x 390 µF (PCB value field); mcuoff4.3, mcu0.3W, csup3u rail model; vbatlow 12.0/24.0 V EMF; nohvsup no Q4/D9 HV+ supply path; nofuse F1 solid.

| system | case | dump LS peak (A) | dump ΔTj (K) | I_bat pk (A) | LS 175 C at (ms) | F1 melt (ms) | margin | kill (ms) | I_LS at kill (A) | kill Vds pk (V) | VBUS min / max (V) | BAT+ max (V) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 12 V | ideal base | 588 | 4.3 | 311 | 2.50 | 14.1 | 5.6 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | rbat10 | 581 | 4.2 | 463 | 0.83 | 8.5 | 10.3 | – | – | – | -5.2 / 13.3 | 13.6 |
| 12 V | rbat50 | 599 | 4.5 | 207 | 10.41 | 29.8 | 2.9 | – | – | – | -5.5 / 13.8 | 13.7 |
| 12 V | lbat1 | 588 | 4.4 | 314 | 2.46 | 14.1 | 5.7 | – | – | – | -5.4 / 13.5 | 13.5 |
| 12 V | lbat5 | 588 | 4.3 | 308 | 2.60 | 14.2 | 5.4 | – | – | – | -5.7 / 13.5 | 14.3 |
| 12 V | rdie1m | 700 | 5.2 | 311 | 2.49 | 14.1 | 5.7 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | rdie20m | 363 | 3.0 | 312 | 2.51 | 14.1 | 5.6 | – | – | – | -5.4 / 13.5 | 13.5 |
| 12 V | rdie100m | 323 | 2.3 | 311 | 2.52 | 14.1 | 5.6 | – | – | – | -5.4 / 13.5 | 13.5 |
| 12 V | i2t0.7 | 588 | 4.3 | 311 | 2.52 | 9.3 | 3.7 | – | – | – | -5.7 / 13.5 | 13.5 |
| 12 V | i2t1.5 | 588 | 4.3 | 311 | 2.48 | 23.9 | 9.6 | – | – | – | -5.0 / 13.5 | 13.5 |
| 12 V | kh0 | 588 | 4.3 | 312 | 2.44 | 13.5 | 5.5 | – | – | – | -5.4 / 13.5 | 13.5 |
| 12 V | kh3 | 588 | 4.3 | 311 | 2.62 | 15.4 | 5.9 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | tau20u | 588 | 4.3 | 311 | 2.50 | 14.1 | 5.6 | – | – | – | -8.6 / 13.5 | 18.9 |
| 12 V | tau500u | 588 | 4.3 | 311 | 2.50 | 14.1 | 5.6 | – | – | – | -0.9 / 13.5 | 13.5 |
| 12 V | esrmax | 500 | 3.8 | 311 | 2.50 | 14.1 | 5.6 | – | – | – | -5.4 / 13.5 | 13.5 |
| 12 V | esrcold | 334 | 2.8 | 311 | 2.52 | 14.1 | 5.6 | – | – | – | -5.2 / 13.5 | 13.5 |
| 12 V | dcr5m | 587 | 4.3 | 323 | 2.20 | 13.4 | 6.1 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | rail-lo | 586 | 4.4 | 311 | 2.47 | 14.2 | 5.7 | – | – | – | -5.4 / 13.5 | 13.5 |
| 12 V | rail-hi | 589 | 4.3 | 312 | 2.53 | 14.0 | 5.6 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | tcase25 | 595 | 4.1 | 314 | 3.09 | 13.7 | 4.4 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | tcase70 | 573 | 4.8 | 306 | 1.55 | 14.9 | 9.6 | – | – | – | -5.3 / 13.5 | 13.5 |
| 12 V | noET | 598 | 3.9 | 319 | 4.75 | 10.7 | 2.3 | – | – | – | -6.1 / 13.5 | 13.5 |
| 12 V | voc40 | 588 | 4.3 | 311 | 2.50 | 14.1 | 5.6 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | isc12 | 589 | 4.3 | 311 | 2.47 | 14.2 | 5.7 | – | – | – | -5.4 / 13.5 | 13.5 |
| 12 V | dcr8m | 588 | 4.4 | 302 | 2.78 | 14.8 | 5.3 | – | – | – | -5.4 / 13.5 | 13.5 |
| 12 V | lin-coil | 587 | 3.9 | 290 | 4.19 | 15.2 | 3.6 | – | – | – | -32.9 / 25.9 | 13.5 |
| 12 V | cbulk390 | 567 | 3.2 | 311 | 2.51 | 14.1 | 5.6 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | mcuoff4.3 | 588 | 4.3 | 311 | 2.50 | 14.1 | 5.6 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | mcu0.3W | 588 | 4.3 | 311 | 2.50 | 14.1 | 5.6 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | csup3u | 588 | 4.3 | 311 | 2.50 | 14.1 | 5.6 | – | – | – | -5.5 / 13.5 | 13.5 |
| 12 V | nofuse | 588 | 4.3 | 312 | 2.44 | – | – | – | – | – | 3.2 / 13.5 | 13.5 |
| 12 V | board base | 588 | 4.3 | 311 | 2.49 | 14.2 | 5.7 | 14.4 | 167 | 21.6 | 2.0 / 74.4 | 13.5 |
| 12 V | board rbat10 | 581 | 4.2 | 463 | 0.83 | 8.5 | 10.3 | 9.4 | 5 | 3.6 | -5.2 / 74.4 | 13.6 |
| 12 V | board rbat50 | 599 | 4.5 | 207 | – | – | – | 2.4 | 205 | 23.2 | 1.8 / 15.6 | 15.4 |
| 12 V | board mcuoff4.3 | 588 | 4.3 | 311 | – | – | – | 2.3 | 306 | 29.9 | 3.2 / 16.1 | 15.8 |
| 12 V | board mcu0.3W | 588 | 4.3 | 311 | 2.49 | 14.2 | 5.7 | 14.4 | 211 | 25.2 | 3.2 / 74.4 | 13.5 |
| 12 V | board csup3u | 588 | 4.3 | 311 | 2.49 | 14.2 | 5.7 | 14.4 | 206 | 25.0 | 3.2 / 74.4 | 13.5 |
| 12 V | board tau500u | 588 | 4.3 | 311 | 2.49 | 14.2 | 5.7 | 15.0 | 192 | 23.9 | 3.2 / 74.4 | 14.0 |
| 12 V | board vbatlow | 535 | 3.5 | 283 | – | – | – | 1.9 | 285 | 28.4 | 3.2 / 14.8 | 14.6 |
| 12 V | board nohvsup | 588 | 4.3 | 311 | 2.49 | 14.2 | 5.7 | 14.4 | 167 | 22.0 | 2.0 / 74.4 | 13.5 |
| 24 V | ideal base | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | rbat10 | 1123 | 85.3 | 819 | 0.29 | 4.4 | 15.3 | – | – | – | -3.9 / 26.5 | 26.8 |
| 24 V | rbat50 | 1140 | 77.3 | 407 | 0.83 | 8.7 | 10.6 | – | – | – | -5.3 / 27.0 | 26.9 |
| 24 V | lbat1 | 1129 | 96.0 | 619 | 0.30 | 5.4 | 18.2 | – | – | – | -4.6 / 26.7 | 26.7 |
| 24 V | lbat5 | 1129 | 66.9 | 575 | 0.45 | 5.4 | 11.9 | – | – | – | -4.7 / 26.7 | 27.9 |
| 24 V | rdie1m | 1320 | 87.0 | 587 | 0.34 | 5.4 | 16.1 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | rdie20m | 711 | 74.7 | 591 | 0.35 | 5.3 | 15.3 | – | – | – | -4.6 / 26.7 | 26.7 |
| 24 V | rdie100m | 646 | 82.9 | 585 | 0.34 | 5.4 | 16.0 | – | – | – | -4.5 / 26.7 | 26.7 |
| 24 V | i2t0.7 | 1129 | 81.4 | 588 | 0.34 | 3.2 | 9.5 | – | – | – | -5.3 / 26.7 | 26.7 |
| 24 V | i2t1.5 | 1129 | 81.4 | 589 | 0.34 | 10.6 | 31.1 | – | – | – | -3.3 / 26.7 | 26.7 |
| 24 V | kh0 | 1129 | 81.4 | 591 | 0.34 | 5.2 | 15.3 | – | – | – | -4.6 / 26.7 | 26.7 |
| 24 V | kh3 | 1129 | 81.4 | 585 | 0.34 | 5.7 | 16.6 | – | – | – | -4.8 / 26.7 | 26.7 |
| 24 V | tau20u | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | – | – | – | -5.7 / 26.7 | 34.8 |
| 24 V | tau500u | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | – | – | – | -0.6 / 26.7 | 26.7 |
| 24 V | esrmax | 969 | 74.5 | 589 | 0.35 | 5.4 | 15.4 | – | – | – | -4.6 / 26.7 | 26.7 |
| 24 V | esrcold | 654 | 65.3 | 591 | 0.35 | 5.4 | 15.2 | – | – | – | -4.3 / 26.7 | 26.7 |
| 24 V | dcr5m | 1128 | 86.6 | 608 | 0.33 | 5.2 | 15.9 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | rail-lo | 1121 | 82.3 | 588 | 0.34 | 5.4 | 15.9 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | rail-hi | 1135 | 80.5 | 590 | 0.34 | 5.3 | 15.6 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | tcase25 | 1143 | 77.6 | 595 | 0.37 | 5.2 | 14.0 | – | – | – | -4.8 / 26.7 | 26.7 |
| 24 V | tcase70 | 1101 | 88.3 | 578 | 0.29 | 5.7 | 19.5 | – | – | – | -4.5 / 26.7 | 26.7 |
| 24 V | noET | 1165 | 63.5 | 636 | 0.49 | 2.8 | 5.8 | – | – | – | -6.3 / 26.7 | 26.7 |
| 24 V | voc40 | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | isc12 | 1130 | 81.2 | 589 | 0.34 | 5.4 | 15.8 | – | – | – | -4.6 / 26.7 | 26.7 |
| 24 V | dcr8m | 1130 | 77.2 | 573 | 0.35 | 5.5 | 15.6 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | lin-coil | 1129 | 17.8 | 497 | 1.49 | 6.0 | 4.0 | – | – | – | -42.7 / 29.9 | 26.7 |
| 24 V | cbulk390 | 1092 | 77.0 | 589 | 0.35 | 5.4 | 15.5 | – | – | – | -4.8 / 26.7 | 26.7 |
| 24 V | mcuoff4.3 | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | mcu0.3W | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | csup3u | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | – | – | – | -4.7 / 26.7 | 26.7 |
| 24 V | nofuse | 1129 | 81.4 | 591 | 0.34 | – | – | – | – | – | 4.5 / 26.7 | 26.7 |
| 24 V | board base | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | 9.3 | 17 | 7.0 | -4.7 / 74.4 | 26.7 |
| 24 V | board rbat10 | 1123 | 85.3 | 819 | 0.29 | 4.4 | 15.3 | 10.4 | 11 | 5.7 | -3.9 / 74.4 | 26.8 |
| 24 V | board rbat50 | 1140 | 77.3 | 407 | 0.83 | 8.7 | 10.6 | 9.9 | -13 | -0.1 | -5.3 / 74.4 | 26.9 |
| 24 V | board mcuoff4.3 | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | 9.2 | 18 | 7.3 | -4.7 / 74.4 | 26.7 |
| 24 V | board mcu0.3W | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | 7.2 | 20 | 7.5 | -4.7 / 74.4 | 26.7 |
| 24 V | board csup3u | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | 7.4 | 27 | 8.9 | -4.7 / 74.4 | 26.7 |
| 24 V | board tau500u | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | 9.3 | 12 | 6.0 | -0.6 / 74.4 | 26.7 |
| 24 V | board vbatlow | 1034 | 61.4 | 543 | 0.41 | 6.0 | 14.4 | 8.5 | 8 | 4.9 | -4.9 / 74.4 | 24.3 |
| 24 V | board nohvsup | 1129 | 81.4 | 589 | 0.34 | 5.4 | 15.7 | 9.3 | 17 | 7.0 | -4.7 / 74.4 | 26.7 |

## Q4: Fugu2 comparison

Deck: `../fugu2-hs-short/fugu2.py` (post-audit defaults, not edited) with F1 replaced by the same fuse model.

| item | Fugu2 | HC |
|---|---|---|
| battery switch | BF: 3x TPH1R306PL (60 V; IDP 500 A for 100 µs; Rth(ch-c) 0.88 K/W; EAS 455 mJ at IAS 50 A; datasheet Rev 4.0 2019-10-18, fetlib sha256 2776eea3…) | none |
| F1 | 40 A ATO blade footprint, no MPN; modelled as ATOF 0287040: 1.44 mOhm cold, 3300 A²s melting (ATOF datasheet). Rated 32 V DC | ATOF 0287025, 25 A, 1000 A²s |
| LS | schematic and fry: IPP022N12NM6 (ID,pulse 812 A); flat: IPP019N08NF2S (764 A) | IPA045N10N3G (256 A) |
| HS | fry: 2x IPP040N08NF2S; flat: 2x CSD (as written on the board); shorted in the deck, so only the package matters | IPA045N10N3G |
| Cin | schematic 3x 1000 µF; flat and fry 2x Rubycon ZLH 470 µF 100 V (33 mOhm max each, typ 0.65x); fry plus C10 470 µF | 2x 560 µF |
| coil | 50 µH (2x KS130-060A); fry 80 µH (2x KS184) | 53 µH |
| copper | schematic: current PCB extraction; flat and fry: the shared as-built PCB (`fugu2.PCB_9AC7E69`, from `parasitics/out/fugu2-perDeev-noLeads`) | see above |
| D16 | not fitted on any build (deck default) | |
| gate rail | 9.34 V ideal (Fugu2 study assumption); flat's 3.3-to-12 V boost supply is not modelled separately | 11.35 V |

Modes, LS on at t = 25 ms: `ls_only` (BF stays off), `ls+bf` (BF commanded on with the LS; its gate charges from
the bootstrap through R21 200k, so it passes through linear mode), `bf_first` (BF on 20 ms earlier). Battery R
25 mOhm (40 mOhm at 55 V). The Fugu2 deck has **no electro-thermal coupling**: Rd is static at 25 C (`tj125`
case: 125 C). Tj values are rises over a fixed case; the 175 C threshold is taken as a 135 K rise from 40 C.

`out/q4_fugu2.csv`. LS on at 25 ms; times from LS turn-on.

| case | LS peak / ID,pulse (A) | LS rise max (K) | LS 175 C at (ms) | F1 I²t then (A²s) | F1 melt (ms) | BF peak, per die (A) | BF Vds pk (V) | BF Vgs max (V) | E_BF (J) | E_LS (J) |
|---|---|---|---|---|---|---|---|---|---|---|
| schematic_V27_L50_ls_only_kp1.0 | 1689 / 812 | 62 | – | – | – | 115, 38 | 42.4 | 3.10 | 0.5 | 0.2 |
| schematic_V27_L50_ls+bf_kp1.0 | 1689 / 812 | 167 | 8.33 | 1566 | 12.4 | 655, 218 | 42.6 | 7.47 | 37.6 | 7.3 |
| schematic_V27_L50_ls+bf_kp0.6 | 1184 / 812 | 193 | 7.19 | 1072 | 12.6 | 652, 217 | 42.4 | 7.47 | 37.9 | 8.9 |
| schematic_V27_L50_bf_first_kp1.0 | 1663 / 812 | 187 | 1.35 | 701 | 6.3 | 777, 259 | 3.4 | 7.47 | 7.9 | 7.4 |
| schematic_V27_L50_bf_first_kp0.6 | 1183 / 812 | 219 | 0.89 | 414 | 6.8 | 767, 256 | 4.5 | 7.47 | 11.1 | 9.1 |
| schematic_V55_L50_ls_only_kp1.0 | 1849 / 812 | 617 | 0.00 | 3 | – | 188, 63 | 61.4 | 2.98 | 1.9 | 2.2 |
| schematic_V55_L50_ls+bf_kp1.0 | 1849 / 812 | 617 | 0.00 | 3 | 12.2 | 648, 216 | 61.4 | 7.47 | 162.6 | 9.3 |
| schematic_V55_L50_ls+bf_kp0.6_retry | 1264 / 812 | 670 | 0.00 | 3 | 12.6 | 621, 207 | 61.3 | 7.47 | 166.4 | 12.0 |
| schematic_V55_L50_bf_first_kp1.0 | 1847 / 812 | 602 | 0.00 | 3 | 3.4 | 1146, 382 | 11.1 | 7.47 | 20.2 | 9.7 |
| schematic_V55_L50_bf_first_kp0.6 | 1263 / 812 | 657 | 0.00 | 3 | 3.6 | 1074, 358 | 11.9 | 7.47 | 22.1 | 14.4 |
| flat_V27_L50_ls_only_kp1.0 | 1243 / 764 | 22 | – | – | – | 118, 39 | 42.6 | 3.10 | 0.5 | 0.1 |
| flat_V27_L50_ls+bf_kp1.0 | 1243 / 764 | 247 | 6.58 | 872 | 12.2 | 670, 223 | 42.8 | 7.47 | 36.8 | 6.3 |
| flat_V27_L50_ls+bf_kp0.6 | 1220 / 764 | 274 | 6.19 | 718 | 12.4 | 655, 218 | 42.7 | 7.47 | 37.6 | 7.2 |
| flat_V27_L50_bf_first_kp1.0 | 1218 / 764 | 280 | 1.00 | 503 | 6.0 | 784, 261 | 2.6 | 7.47 | 6.0 | 6.3 |
| flat_V27_L50_bf_first_kp0.6 | 1197 / 764 | 307 | 0.83 | 396 | 6.3 | 777, 259 | 3.4 | 7.47 | 7.8 | 7.2 |
| flat_V55_L50_ls_only_kp1.0_retry | 2376 / 764 | 112 | – | – | – | 210, 70 | 61.4 | 3.01 | 2.1 | 0.3 |
| flat_V55_L50_ls+bf_kp1.0_retry | 2376 / 764 | 244 | 6.17 | 938 | 11.9 | 667, 222 | 61.4 | 7.47 | 158.8 | 6.5 |
| flat_V55_L50_ls+bf_kp0.6 | 1691 / 764 | 270 | 0.00 | 3 | 12.2 | 648, 216 | 61.4 | 7.47 | 162.5 | 7.6 |
| flat_V55_L50_bf_first_kp1.0 | 2358 / 764 | 423 | 0.25 | 179 | 3.3 | 1171, 390 | 10.8 | 7.47 | 19.4 | 6.7 |
| flat_V55_L50_bf_first_kp0.6 | 1689 / 764 | 498 | 0.00 | 3 | 3.4 | 1160, 387 | 11.0 | 7.47 | 19.9 | 8.1 |
| fry_V27_L50_ls_only_kp1.0 | 1255 / 812 | 19 | – | – | – | 105, 35 | 47.4 | 3.07 | 0.4 | 0.1 |
| fry_V27_L50_ls+bf_kp1.0 | 1255 / 812 | 230 | 6.08 | 1003 | 9.6 | 808, 269 | 48.7 | 7.47 | 20.4 | 7.3 |
| fry_V27_L50_ls+bf_kp0.6 | 1139 / 812 | 278 | 5.71 | 780 | 9.8 | 797, 266 | 48.7 | 7.47 | 20.1 | 8.9 |
| fry_V27_L50_bf_first_kp1.0 | 1231 / 812 | 233 | 1.22 | 672 | 5.2 | 832, 277 | 0.8 | 7.47 | 3.1 | 7.3 |
| fry_V27_L50_bf_first_kp0.6 | 1131 / 812 | 281 | 0.93 | 459 | 5.4 | 819, 273 | 0.8 | 7.47 | 3.2 | 9.1 |
| fry_V55_L50_ls_only_kp1.0_retry | 1788 / 812 | 175 | 0.01 | 3 | – | 170, 57 | 61.4 | 2.97 | 2.5 | 0.4 |
| fry_V55_L50_ls+bf_kp1.0_retry | 1788 / 812 | 258 | 0.01 | 3 | 9.0 | 887, 296 | 61.4 | 7.47 | 109.7 | 7.7 |
| fry_V55_L50_ls+bf_kp0.6 | 1233 / 812 | 320 | 0.00 | 3 | 9.1 | 879, 293 | 61.4 | 7.47 | 109.8 | 9.7 |
| fry_V55_L50_bf_first_kp1.0 | 1786 / 812 | 351 | 0.01 | 3 | 3.0 | 1146, 382 | 6.7 | 7.47 | 14.3 | 8.1 |
| fry_V55_L50_bf_first_kp0.6 | 1232 / 812 | 431 | 0.00 | 3 | 3.2 | 1126, 375 | 9.5 | 7.47 | 19.1 | 10.9 |
| schematic_V27_L80_ls+bf_kp1.0 | 1687 / 812 | 231 | 6.06 | 996 | 9.6 | 808, 269 | 46.0 | 7.68 | 20.6 | 7.5 |
| schematic_V27_L50_ls+bf_kp1.0_rbat10 | 1681 / 812 | 173 | 6.83 | 1270 | 11.2 | 692, 231 | 42.6 | 7.47 | 80.4 | 7.3 |
| schematic_V27_L50_ls+bf_kp1.0_rbat50 | 1701 / 812 | 97 | – | – | 20.5 | 450, 150 | 42.7 | 7.47 | 10.2 | 7.3 |
| schematic_V27_L50_ls+bf_kp1.0_tj125 | 1588 / 812 | 238 | 6.29 | 706 | 12.8 | 646, 215 | 42.1 | 7.46 | 37.3 | 11.2 |
| schematic_V27_L50_ls+bf_kp1.0_rbat50_tj125 | 1598 / 812 | 146 | 16.99 | 2511 | 21.1 | 442, 147 | 42.2 | 7.46 | 10.3 | 11.2 |
| schematic_V27_L50_ls+bf_kp1.0_esrmax | 1543 / 812 | 167 | 8.34 | 1570 | 12.4 | 655, 218 | 42.6 | 7.47 | 37.6 | 7.3 |

- **LS only:** F1 sees about 5 A²s; the BF body diodes block the battery. Nothing blows. The Cin dump still puts
  1.2-1.7 kA (27 V) and 1.8-2.4 kA (55 V) through the LS, 1.5-3x its ID,pulse.
- **LS + BF at 27 V:** F1 melts at 9.6-12.8 ms; the LS reaches 175 C at 5.7-8.3 ms (schematic 7.2-8.3,
  flat 6.2-6.6, fry 5.7-6.1 ms over the KP bracket). The BF dissipates 20-38 J in linear mode while its gate charges (Vgs reaches only 7.5 V; per die
  about 220-270 A, 7-13 J). With a 10 mOhm battery the BF takes 80 J.
- **BF first at 27 V:** F1 melts at 5.2-6.8 ms, the LS reaches 175 C at 0.8-1.35 ms. BF energy 3-11 J.
- **Battery R 50 mOhm:** the LS stays below 175 C with static 25 C Rd (97 K rise) and F1 melts at 20.5 ms; with
  Rd at 125 C the LS reaches 175 C at 17.0 ms against a melt at 21.1 ms.
- **55 V:** with the LS alone (no battery current) the Cin dump takes the LS to a 617 K rise on the schematic
  (3x 1000 µF) and 175 K on fry, past the 135 K budget within 10 µs. Flat (2x 470 µF, IPP019) reaches 112 K at
  KPF 1 and passes 135 K at KPF 0.6. With the
  BF commanded on, its Vds reaches 61 V on a 60 V part (avalanche in the model) and it dissipates 110-166 J.
  F1 melts at 3-12.6 ms, but at 55 V a 32 V-rated blade fuse is outside its interrupt rating, so clearing is not
  assured.
- Five 55 V cases first stopped on "timestep too small"; they passed on one retry with tmax 20 ns and reltol 3e-3
  (marked `_retry`).
- `fugu2.py` writes the hard-clamp PWL with `%.6g`, which rounds t_clamp + 20 ns onto t_clamp (non-increasing
  PWL); `q4_fugu2.py` rewrites that line with `%.9g`. `fugu2.py` itself was not changed.

**Does a fuse-blow scheme change the Cin-dump failure?** No. The dump peaks within microseconds (HC: about
15 µs equivalent width), while F1 needs
milliseconds; F1 I²t at the moment the dump takes the LS past its limit (55 V) is about 3 A²s. A fuse-blow sequence
can only act on the battery-fed phase, and even there the LS reaches its limit before a 40 A F1 melts at 27 V.

## Limitations and unverified items

- Tj above 175 C is an extrapolation of the vendor ladder and Rd tempco; the real device fails somewhere above
  Tjmax, and how (short, open, bond-wire fusing) is not modelled.
- KP anchor: the dsdig transfer reading is unverified (overlay-review-required); the Level-3 channel is
  extrapolated to 1-2 kA. The KP bracket 1/0.9/0.6 stands in for an LTspice cross-check with the full vendor
  model, which was not run.
- LS SOA (Diagram 3) was not digitized: no dsdig class. Only ID,pulse and the ladder are used.
- L1 saturation law is borrowed from a different core (KS130-060A) and extrapolated below 25 % µ; L1 DCR is
  computed from assumed core dimensions.
- The fuse arc model (kh, tau_arc, clear = melt + 7 tau) is an assumption; the melt I²t is a typical value.
- Battery, panel, MLCC, Cout typ/cold factors and the whole board-rail model (C_sup, P_mcu, MCU-off, C_p12) are
  assumptions. The 12 V board-rail result flips with the MCU-off threshold.
- Q3 of the HC is a switch; Q3's own dissipation is not tracked.
- Fugu2: static Rd (no electro-thermal), BF Tj and BF linear-mode SOA not evaluated, flat's gate rail taken as
  9.34 V, flat/fry parts the stories do not give keep schematic values.
- Copper: pitch-2 extraction only (the pitch-1 run hit the 2 h limit without output); branch mutuals enter only through the
  shared + branch reduction; skin-ladder fit warning (35.7 %).
- The firmware finding is from reading the code and RM0440; it was not run on hardware.

## Source access log

| source | URL / location | rung | result |
|---|---|---|---|
| Littelfuse ATOF datasheet | littelfuse.com/assetdocs/littelfuse-datasheet-287-atof?assetguid=43dcdce8-… | 1 (UA only): 403; Playwright API request: 403; 2 (full Chrome header set) | 200, 412292 B, %PDF-1.7 |
| Panasonic FR-A | industrial.panasonic.com/cdbs/www-data/pdf/RDF0000/ABA0000C1259.pdf | 1-2: curl tarpitted (0 B); 4: headless page.goto gives the viewer shell; in-page fetch from industrial.panasonic.com/ww/ | 855742 B, %PDF (C1209 fetched first was the FC series) |
| TI LM5107 SNVS333F | ti.com | 2 (`tools/chrome_curl.sh`) | PDF |
| IPA045N10N3G, TPH1R306PL | fetlib `/Users/fab/dev/pv/pwr-mosfet-lib/datasheets/` | local | PDFs |
| Chemi-Con KZN | earlier in the session (Zmax at 20 C and -10 C, 390 µF ladder) | | values as listed |
| HC hardware, firmware | GitHub archives in /tmp/ls2420, /tmp/lsfw | 1 | untrusted, parsed only |

Access recipe recorded in `~/dev/kb/access/panasonic-and-littelfuse-pdfs-need-browser-tls-or-full-chrome-headers.md`.
