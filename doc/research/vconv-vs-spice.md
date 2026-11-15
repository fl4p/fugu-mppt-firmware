*this document is an LLM generated placeholder*

# vconv host simulator vs SPICE half-bridge bench (2026-11-15)

Cross-validation of the firmware plant model **A** (`src/sim/vconv.cpp/.h`, `config/lab/vconv_mock/conf/vconv.conf`,
`test/host-stub/mppt-sim.*`) against **B**, an LTspice transient bench built from the dcdc-tools loss deck
(`~/dev/pv/ee/dcdc-tools/loss`). Nothing was flashed, commanded, pushed or committed; no firmware source was edited.
All netlists, scripts, raw results and caches are under `/tmp/vconv-spice/` (file list at the end); they are not in
the repository. The result rows that `test/host-stub/vconv-spice-check.cpp` uses are copied to
`doc/research/vconv-spice-data/` (CSV, with the `extract.py` that produced them).

## 1. Setup

### Model A (as driven here)

`/tmp/vconv-spice/a/a_harness.cpp` links the real `src/sim/vconv.cpp`, holds Vin/Vout stiff (set every cycle),
pwmMax 4103 at 38996 Hz (fry/flat MCPWM), runs 400 cycles at a fixed HS/LS count and averages the second half.
Variants:

| variant | L | node C | Q | diode Vf | note |
|---|--:|--:|--:|--:|---|
| A-noring-vf0 | per board | 0 | - | 0 | what the wx-dawn / wx-low tracker retunes used (`vconv_mock`: ring_c=0) |
| A-noring-vf07 | per board | 0 | - | 0.7 | |
| A-ring-fry | 76 µH | 5.17 nF (254 kHz) | 25 | 0.7 | mppt-sim fry board |
| A-ring-flat | 51 µH | 4.67 nF (326 kHz) | 15 | 0.7 | mppt-sim flat board |
| A-ring-sun | 80 µH | 5 nF | 40 | 0.7 | rcal-sun scenario |

### Model B

- Deck: dcdc-tools `loss.py` `_base.cir` (datasheet-curve FET models with LM body diodes, gate drive, loop
  parasitics from the Fugu2 extraction fixture). fry = `examples/fisi.yaml` (HS 2x IPP040N08NF2S, LS IPP022N12NM6);
  flat = same HS (proxy, see section 9), LS IPP019N08NF2S.
- `/tmp/vconv-spice/bench.py` rewrites the deck into a fixed-HS/LS bench: ideal stiff Vin and Vout sources, firmware gate
  timing at 160 MHz ticks with the UCC21330B 185 ns rising-edge dead time (RDT 20k), coil with DCR (fry 2 mΩ, flat 5 mΩ,
  estimates), `Rser=0` on every inductor, input bank lumped (940 µF/16.5 mΩ/2.5 nH + 4.81 µF/4 mΩ/0.8 nH; within 0.01 %
  of the full nine-branch Cin network on a check point). Averages over the last 2 of 5-6 periods.
- **B-rp**: B plus `Rcore` from switch node to output, Rp = Q·ω0·L with the A board Q and f (fry 25/254 kHz,
  flat 15/326 kHz). dcdc-tools has no core loss; B alone rings with Q ≈ 300-1700. B-rp is a damping proxy, so its Q
  is **not** an independent check of A's Q.
- CCM points (P2-P4) use dcdc-tools `loss.py` unchanged (`/tmp/vconv-spice/exp_ccm.sh`), plus DCR·I²rms for the coil.
- Tool: LTspice 17.2.4 (`LTspice -b`). ngspice 46 and Xyce are installed but not used.

### Firmware LS rule used in all DCM sweeps

DCM LS = HS·(1/M′−1), M′ = M/0.980198, minimum 320 counts (boot_refresh_ns 2000), capped at pwmMax−HS−1
(`/tmp/vconv-spice/exp_dawn.py::fw_ls`).

## 2. P1, dawn (Vin 30-55 V, 0.5-30 W, DCM, LS at its minimum pulse)

Per-point extract (stiff Vin, Vout 26.5 V). Full table for all eight sets (fry/flat x 30/35/45/55 V, 1/2/5/10/20 W):
`/tmp/vconv-spice/results/dawn_table.md`.

| set | HS | LS | model | Pin W | Pout W | loss W | η % | Pout vs B | I(LS off) A |
|---|--:|--:|---|--:|--:|--:|--:|--:|--:|
| fry-45 | 386 | 320 | A-noring | 1.139 | 1.139 | 0.000 | 100.0 | +1355 % | -0.105 (analytic) |
| | | | A-ring-fry | 1.015 | 0.771 | 0.244 | 76.0 | +885 % | |
| | | | B | 0.318 | 0.078 | 0.240 | 24.6 | - | -0.191 |
| | | | B-rp | 0.731 | 0.444 | 0.287 | 60.8 | +467 % | -0.125 |
| fry-45 | 791 | 526 | A-noring | 4.959 | 4.959 | 0 | 100.0 | +22.4 % | |
| | | | A-ring-fry | 4.734 | 4.493 | 0.241 | 94.9 | +10.9 % | |
| | | | B | 4.107 | 4.051 | 0.056 | 98.6 | - | +0.069 |
| | | | B-rp | 4.485 | 4.337 | 0.148 | 96.7 | +7.1 % | +0.084 |
| fry-45 | 1601 | 1064 | A-noring | 20.32 | 20.32 | 0 | 100.0 | -9.4 % | |
| | | | A-ring-fry | 19.25 | 19.00 | 0.251 | 98.7 | -15.2 % | |
| | | | B | 22.43 | 22.42 | 0.017 | 99.9 | - | +0.266 |
| | | | B-rp | 22.53 | 22.37 | 0.155 | 99.3 | -0.2 % | +0.268 |
| flat-45 | 425 | 320 | A-noring | 2.232 | 2.232 | 0 | 100.0 | -29.3 % | -0.076 (analytic) |
| | | | A-ring-flat | 2.556 | 2.291 | 0.265 | 89.6 | -27.4 % | |
| | | | B | 3.279 | 3.157 | 0.122 | 96.3 | - | +0.219 |
| | | | B-rp | 2.344 | 2.132 | 0.212 | 90.9 | -32.5 % | +0.019 |
| fry-30 (P4-like) | 3182 | 349 | A-noring | 10.12 | 10.12 | 0 | 100.0 | +90.5 % | |
| | | | A-ring-fry | 8.224 | 8.114 | 0.110 | 98.7 | +52.7 % | |
| | | | B | 5.372 | 5.314 | 0.059 | 98.9 | - | +0.030 |

Sweep statistics (`/tmp/vconv-spice/results/dawn_summary.json`; ripple = Pout residual against a smooth log-log fit
over HS for Pout > 1 W; neg slope = share of 45-count HS steps where Pout falls although HS rises):

| set | model | η 1-3 W | η 8-30 W | ripple amp | ripple period (counts) | neg-slope share | min-pulse region up to |
|---|---|--:|--:|--:|--:|--:|--:|
| fry-45 | A-noring | 1.000 | 1.000 | 0.3 % | - | 0.00 | 1.8 W |
| | A-ring-fry | 0.842 | 0.984 | 14 % | 386 | 0.00 | 1.2 W |
| | B | 0.779 | 0.997 | 29 % | 465 | 0.22 | 1.2 W |
| | B-rp | 0.838 | 0.989 | 22 % | 465 | 0.03 | 1.3 W |
| fry-35 | A-noring | 1.000 | 1.000 | 0.9 % | - | 0.00 | 3.1 W |
| | A-ring-fry | 0.915 | 0.989 | 12 % | 495 | 0.10 | 2.7 W |
| | B | 0.910 | 0.998 | 21 % | 645 | 0.28 | 3.6 W |
| | B-rp | 0.900 | 0.993 | 17 % | 630 | 0.17 | 3.4 W |
| fry-30 | A-noring | 1.000 | 1.000 | 0.5 % | - | 0.00 | 8.5 W |
| | A-ring-fry | 0.930 | 0.987 | 9 % | 666 | 0.33 | 8.4 W |
| | B | 0.940 | - | 20 % | 900 | 0.51 | 5.8 W |
| flat-45 | A-noring | 1.000 | 1.000 | 0.9 % | - | 0.00 | 2.7 W |
| | A-ring-flat | 0.865 | 0.984 | 7 % | 292 | 0.00 | 2.8 W |
| | B | 0.866 | 0.997 | 26 % | 330 | 0.23 | 3.8 W |
| | B-rp | 0.870 | 0.983 | 10 % | 330 | 0.00 | 3.1 W |
| flat-30 | A-noring | 1.000 | 1.000 | 0.9 % | - | 0.00 | 13.2 W |
| | A-ring-flat | 0.929 | 0.991 | 6 % | 555 | 0.19 | 13.1 W |
| | B | 0.955 | 0.998 | 14 % | 756 | 0.36 | 11.9 W |
| fry-55 / flat-55 | A-noring | 1.000 | 1.000 | <0.5 % | - | 0.00 | 1.1 / 1.8 W |
| | A-ring | 0.78 / 0.76 | 0.97 | 17 / 7 % | 296 / 240 | 0.00 | |
| | B | 0.87 / 0.97 | 0.99 | 34 / 29 % | 345 / 244 | 0.04 / 0.25 | |

Findings:

- Pointwise Pout at a given HS differs between any two models by ±10-30 % at 5-20 W and by up to 10x below 1 W. In B,
  B-rp and A-ring the coil current left over from the free ring at HS turn-on is ±0.1-0.3 A, so the delivered charge per
  cycle depends on ring phase. A-noring has no such term. This is the main error at P1, and it is not a loss error.
- **Spurious dP/dHS.** A-noring gives a monotone Pout(HS) (neg-slope share 0). B gives Pout ripple of 14-34 % with a
  period of 240-900 HS counts and negative steps on 4-51 % of 45-count moves. B-rp gives 10-22 % and 0-17 %, and A-ring
  6-17 % and 0-33 %. The A-ring ripple has 10-25 % shorter periods than B for fry, which follows from its higher ring
  frequency (254 vs ~233 kHz). With the stiff-source P&O estimate x = π·a·HS/(2·c·Λ) (PV curvature c from the k=0.78
  model), the trap half-width is 0.06-0.16 of Vmp for A-ring and 0.10-0.25 for B/B-rp. Treat this estimate as an order
  of magnitude only; the A-noring estimate is 0.
- **Efficiency.** A-noring is 100 % everywhere. Below 3 W, A-ring (76-93 %) brackets B (78-97 %) and B-rp (84-90 %).
  At 8-30 W, A-ring (97.4-99.1 %) loses more than B (99.3-99.8 %) and about the same as B-rp (98.3-99.3 %).
- The minimum-pulse region (LS pinned at 320) extends to 8.5-13 W at Vin 30 V, about 3 W at 35 V and 1-3 W at 45-55 V.
  The models agree to within the ring-phase scatter.

## 3. P2, about 100 W at the DCM/CCM boundary

HS swept from 0.55 to 1.0 of the boundary duty with the firmware DCM LS rule (14 points each). The full rows are in
`/tmp/vconv-spice/results/p2.json` and `p2_rp.json`, and the CCM side at 3.8 A is in the loss.py table below.

| set | model | Pout error vs B-rp (mean / rms / max) | Iin rms error | η range |
|---|---|--:|--:|--:|
| flat 60 V → 26.5 V, 31-98 W | A-noring | 0.0 / 4.9 / 8.1 % | 5.1 % | 100 % |
| | A-ring-flat | -0.5 / 1.4 / 3.2 % | 1.7 % | 98.5-99.6 % |
| | B (undamped) | - | - | 99.2-99.8 % |
| | B-rp | - | - | 98.4-99.2 % |
| fry 75 V → 26.5 V, 24-76 W | A-noring | 0.0 / 6.4 / 10.7 % | 6.4 % | 100 % |
| | A-ring-fry | +4.8 / 7.7 / 12.5 % | 8.1 % | 96.3-99.0 % |
| | B (undamped) | - | - | 97.8-99.8 % |
| | B-rp | - | - | 97.2-99.2 % |

- In B and B-rp, Pout(HS) is a staircase: plateaus of 3-4 HS steps, then jumps of 10-20 W. A-noring passes through the
  middle of it, so its mean error is 0 and its rms error is 5-6 %. A-ring-flat, with its ring tuned to 326 kHz, follows
  the B-rp staircase to within 1.4 % rms. A-ring-fry is tuned to 254 kHz while B runs at 233 kHz; its staircase is out
  of phase, which gives 7.7 % rms.
- **Boundary.** The analytic boundary (D = M, idle time to zero) is 98.6 W for flat at 60 V and 72.9 W for fry at 75 V.
  At D = M, B gives 97.6 W and 75.6 W, which agrees within the ±0.25 A ring residual (±7 % of Iob). At most points B
  turns the LS off at +0.0 to +0.5 A, so the WCE-scaled LS rule truncates the current and the LS body diode finishes
  the freewheel. Model A cannot show this.

## 4. P3/P4, CCM (loss.py, synchronous, 185 ns dead time)

`/tmp/vconv-spice/results/ccm_table.md`; budgets in `/tmp/vconv-spice/ccm/<tag>/loss_budget.json`. A in CCM is
lossless (Iin = M·Iout). B's η excludes coil core loss.

| point | Pout W | B loss W (FET+Cin+gate / DCR) | η B % | η A % | Iin error A vs B | largest B terms |
|---|--:|--:|--:|--:|--:|---|
| flat P2 60 V 3.8 A | 101 | 0.99 / 0.10 | 98.94 | 100 | -1.06 % | avalanche 0.36, Coss 0.26, Cin 0.15 |
| fry P2 75 V 3.8 A | 101 | 1.24 / 0.03 | 98.75 | 100 | -1.25 % | Coss 0.48, avalanche 0.37, Cin 0.12 |
| flat P3 60 V 11.3 A | 299 | 2.72 / 0.66 | 98.88 | 100 | -1.12 % | avalanche 0.83, Cin 0.65, Coss 0.27 |
| fry P3 60 V 11.3 A | 299 | 2.42 / 0.26 | 99.11 | 100 | -0.89 % | Cin 0.62, avalanche 0.36, Coss 0.33 |
| flat P3 60 V 22.6 A | 599 | 7.30 / 2.58 | 98.38 | 100 | -1.62 % | Cin 2.37, avalanche 1.74, cond. LS 0.76 |
| fry P3 60 V 22.6 A | 599 | 6.63 / 1.03 | 98.74 | 100 | -1.26 % | Cin 2.34, cond. LS 0.87, avalanche 0.77 |
| flat P4 30 V 3.0 A | 80 | 0.34 / 0.05 | 99.52 | 100 | -0.48 % | Coss 0.10, gate 0.09, Qrr 0.04 |
| fry P4 30 V 3.0 A | 80 | 0.35 / 0.02 | 99.54 | 100 | -0.46 % | Coss 0.10, gate 0.08, Qrr 0.05 |
| flat P4 30 V 11.3 A | 299 | 1.43 / 0.64 | 99.31 | 100 | -0.69 % | cond. HS 0.30, avalanche 0.27, Cin 0.26 |
| fry P4 30 V 11.3 A | 299 | 1.32 / 0.26 | 99.48 | 100 | -0.52 % | cond. HS 0.31, Cin 0.28, Qrr 0.22 |

- In CCM, A's error is the missing 0.5-1.6 % loss. It moves Iin and Pin, not the tracker's Pout(Vin) shape.
- The "avalanche" term is how loss.py books the HS turn-off overshoot. B shows the node at 84-97 V for 60-75 V input,
  which A does not model. I have not checked whether that booking is correct.
- **P4, DCM side (fry-30 / flat-30).** With Vin < 2·Vout, the free ring reaches Vin+Vf on every cycle. The reverse
  current then decays only at (Vin−Vout)/L (3.5 V across L), which takes about 4-5 µs. B therefore carries a negative
  current into the next HS pulse and delivers far less per HS count near the boundary: 5.3 W in B against 10.1 W in
  A-noring at fry HS 3182, +90 %. A-ring reproduces the mechanism, but at the wrong ring frequency it still reads +53 %.

## 5. DCM ring: frequency, damping, clamps

| quantity | A-ring-fry | B fry | A-ring-flat | B flat | flu bench (measured) / B flu |
|---|--:|--:|--:|--:|--:|
| free ring f | 254 kHz (const) | 232-235 kHz at 72-78 V; about 200 kHz at 45 V (clamped every cycle) | 326 kHz | 314 kHz (B), 327-345 kHz (B-rp) | - / 226 kHz free |
| rise-transit f | same C | - | same C | - | 320 kHz (2.98 nF) / 316-336 kHz |
| equivalent node C | 5.17 nF | about 6.1 nF at 76 µH (5.8 nF at 80 µH) | 4.67 nF | 4.8-5.0 nF | 2.98 nF (transit) vs 3.9 nF (free) in B |
| Q | 25 | 290-1700 (no core loss); B-rp 28 by construction | 15 | 325; B-rp 17.8 | - |
| low clamp | −0.7 V (LS diode, Vf) | −1.1 to −1.3 V (LS body diode, fast edge) | same | −1.26 V | - |
| high clamp | Vin+0.7 | Vin+0.6..0.7 (flu B 71.77 V at 71.1 V) | same | same | 72.84 V measured |
| HS turn-off overshoot | none | 84-97 V at 60-75 V input | none | 65 V at 60 V | - |
| ring peak from LS-diode state at 78.6 V | reaches clamp | 58-60 V (not clamped; needs 0.5 A reverse to clamp) | | | |

The node capacitance is strongly nonlinear (Coss). On flu, B's rise transit runs at 316-336 kHz while its free ring
runs at 226 kHz, so a single linear C cannot fit both the transit and the ring. The rise transit sets the zero-crossing
shift and the ring sets the ring-phase staircase.

## 6. LS current at turn-off (DCM, firmware LS rule)

| point | A (ideal ramps, analytic) | B | B-rp |
|---|--:|--:|--:|
| fry-45 HS 386 LS 320 | -0.105 A | -0.191 | -0.125 |
| fry-45 HS 521 LS 346 | +0.037 | +0.288 | +0.187 |
| flat-45 HS 335 LS 320 | -0.280 | -0.312 | -0.210 |
| flat-45 HS 425 LS 320 | -0.076 | +0.219 | +0.019 |
| fry-30 HS 1427 LS 320 | -0.272 | -0.156 | - |
| flat-30 HS 1014 LS 320 | -0.604 | -0.462 | - |
| fry-55 HS 318 LS 329 | +0.027 | +0.147 | - |
| sun-fry-78 HS 998, LS swept | zero crossing at 1907 | zero crossing at 1983 (+76) | 1997 (+90) |
| cal-flat-66 HS 923, LS swept | 1333 | 1390 (+57) | 1371 (+38) |

Absolute errors are 0.03-0.3 A, and the sign flips with ring phase. The zero crossing in B sits 40-90 counts later
than ideal because of the HS-on residual current, the 185 ns rising-edge dead time and the gate delays. A-ring produces
a similar +34 to +89 shift (section 8).

## 7. Claim (a): reverse current returned to the input

> "energy returned to input via the HS body diode, only conduction and diode losses lost"

Measured return fraction ret = dPin/dPout over LS from zc+20 to zc+400 counts:

| set | A-noring | A-ring (own board) | B | B-rp |
|---|--:|--:|--:|--:|
| sun-fry-78 (HS 998) | 1.000 | 0.998 | 0.956 | 0.949 |
| sun-fry-78 (HS 800) | 1.000 | 0.999 | 0.952 | 0.938 |
| sun-fry-72 | 1.000 | 1.037 (ring phase) | 0.976 | 0.983 |
| cal-flat-66 | 1.000 | 0.979 | 0.984 | 0.982 |
| low-fry-45 | 1.000 | 0.968 | 0.866 | 0.886 |
| low-fry-35 / -30 | 1.000 | 0.98 | 0.974 / 0.982 | 0.972 / 0.979 |
| low-flat-45 / -30 | 1.000 | 0.977 / 0.978 | 0.976 / 0.979 | 0.968 / 0.974 |

Going from the zero crossing to LS = 320 adds 0.01-0.18 W of loss in B (sun-fry-78: 0.137 → 0.313 W; low-fry-45:
0.205 → 0.230 W while Pout drops from +0.21 to −0.25 W). A Schottky across the fry LS (generic 15 A/100 V
model, `/tmp/vconv-spice/exp_sch.py`) changes the zero crossing by −13 to −21 counts, the return fraction by under
0.01 and dawn loss by +0.01 W.

**Verdict: largely correct, slightly optimistic.** The reverse current does return to the input. B returns 95-98 % of
it at 66-78 V and 87-98 % at dawn power, so 2-13 % is lost, against 0 % in A-noring and 0-4 % in A-ring. The dominant
dawn loss is something else: a fixed per-cycle HS-on hard-switching and ring loss of 0.1-0.3 W that does not depend on
how long the LS stays on. A-noring has none of it. A-ring has 0.24-0.30 W at 45 V, close to B/B-rp. At 66-78 V it has
0.6-1.0 W, which is 0.3-0.8 W too high because a linear 5 nF overstates the node energy C·V² at high voltage.

## 8. Claim (b): the ring clamp pins the rect-cal peak at 1030-1040 counts

> "the ring clamp pins the measured rect-cal peak at about 1030-1040 counts regardless of HS"

The rect-cal knee was fitted with the firmware `kneeFit` and the RectCal dip cut (`/tmp/vconv-spice/fit/knee.cpp`),
in counts of 4103 (1030-1040 of 2047 ≈ 2064-2084 of 4103):

| set | ideal | A-ring-sun knee (−ideal / −zc) | A-ring-fry | B | B-rp |
|---|--:|--:|--:|--:|--:|
| sun-fry-78 HS 998 | 1907 | 2026 (+119 / +30) | 2047 (+140 / +60) | 1981 (+74 / −2) | 1995 (+88 / −2) |
| sun-fry-78 HS 800 | 1529 | 1607 (+78 / +5) | 1633 (+104 / +35) | 1568 (+39 / +21) | 1531 (+2 / −46) |
| cal-flat-66 HS 923 | 1333 | - | A-ring-flat 1390 (+56 / +22) | 1406 (+73 / +16) | 1453 (+120 / +83) |
| sun-fry-72 HS 998 | 1663 | 1568 (−95) | 1564 (−99) | 1983 (+320) | 1943 (+280) |

**Verdict: not supported as stated.**

- The knee is not pinned. A 20 % HS change moves it by 419 counts in A-ring-sun, 413 in B and 464 in B-rp, which is
  1:1 with the ideal value. A ±3 % HS change, as in the parent's soft-source runs, moves the ideal value by about 60
  counts of 4103, roughly one rect-cal grid step (3.75 % of ideal). At that resolution a constant peak is expected
  whatever the mechanism.
- A late knee does exist. B puts it +39 to +74 counts after the ideal, against A-ring's +78 to +140.
- The cause in B is the coil current left over from the ring at HS-on, plus the dead time and gate delays, which move
  the zero crossing. The clamp is not the cause: B's knee−zc is about 0 (−2 to +21).
- A-ring adds a further +30 to +60 counts of "clamp lag" that B does not show. The linear-C ring at 76-80 µH and
  5 nF reaches the clamp from the LS-diode state, while B's nonlinear Coss ring peaks at 58-60 V and stays below
  Vin = 78.6 V.
- At Vin 72 V (Vin < 2.7·Vout) both models give ring-phase artefacts with opposite signs (A −95, B +300). The fit is
  not trustworthy there in either model.

## 9. Verdict per modelling assumption in A

| assumption | verdict | impact estimate |
|---|---|---|
| Cycle-averaged ideal ramps, phases HS → LS → off | OK for CCM and mean DCM transfer | P2 mean error 0 % (rms 5-6 % from ring phase) |
| Lossless switches: no Rds, DCR, dead time or gate delay | over-simplified | CCM loss 0.5-1.6 % of Pin (1-10 W); zero crossing 40-90 counts late in B; Iin −0.5 to −1.6 % |
| No-ring branch: reverse current returned instantly, no Vf (`ring_c=0`, the vconv_mock default used by the tracker retunes) | **wrong for dawn tuning** | misses the 0.1-0.3 W fixed per-cycle loss (η 100 % vs 78-97 % at 1-3 W), the ±0.1-0.3 A ring residual at HS-on, the 10-34 % Pout ripple over HS and the negative dP/dHS steps (0 % vs 4-51 %); +90 % Pout near Vin ≈ Vout |
| Return of reverse energy via the HS diode (claim a) | OK, slightly optimistic | 2-13 % of the reverse energy is lost in B vs 0 % in A |
| Ring as linear C, constant Q | over-simplified | fry ring 254 vs about 233 kHz in B gives staircase phase errors (P2 rms 7.7 % vs 1.4 % for flat, which is matched); hard-switch loss at 66-78 V 0.3-0.8 W too high; spurious clamp lag +30 to +60 counts |
| Q values (25/15/40) | not checkable with B | dcdc-tools has no core loss (B Q 300-1700). A low Q matters: B-rp vs B halves the P1 ripple |
| Clamps at −(Vf) and Vin+Vf | OK for the high clamp; low clamp 0.4-0.6 V too shallow | B shows −1.1 to −1.3 V on the LS diode edge. Small energy impact |
| HS turn-on charges the node from the input, C·(Vin−Vout−x) | OK in mechanism, wrong in size at high Vin | linear-C energy overestimates Coss energy |
| Constant L (no DC bias) | not tested (B uses constant L too) | - |
| Stiff Vin and Vout in this harness | test choice, not A's own | mppt-sim uses a PV source; the rcal-sun soft source was not reproduced |

## 10. Ranked fixes for A

1. **Run tracker and dawn tuning with the ring on.** Use the board ring by default in `vconv_mock`, or at least in
   wx-dawn and wx-low: `ring_c` > 0, Vf 0.7. Re-check the low-light retune against it. A-noring is blind to the
   spurious P&O slope, which is the effect the retune is supposed to handle. (`vconv.conf` change only.)
2. **Re-fit the ring frequency per board.** fry about 233 kHz (≈ 6 nF at 76-80 µH), flat about 315-330 kHz. This is
   the parameter that sets the Pout(HS) staircase phase: flat, which is matched, reaches 1.4 % rms and fry, which is
   not, 7.7 %.
3. **Separate the transit C from the ring C, or use a voltage-dependent C.** A small effective C for the HS-on
   energy and the rise transit, and a larger free-ring C. This removes the 0.3-0.8 W overstatement at 66-78 V and the
   spurious clamp lag in the rect-cal knee.
4. **Add a fixed per-cycle switching loss and conduction resistance.** E_on ≈ 2.5-7 µJ per cycle at 30-55 V, plus
   R_hs, R_ls and DCR. This brings CCM η to 98.4-99.5 % and dawn η into line, and makes Iin/Pin usable for input-side
   checks.
5. **Model the dead time and gate delay.** 185 ns rising-edge dead time plus driver delay, about 30-40 counts. This
   moves the DCM zero crossing and the rect-cal ideal by the same amount as in B.
6. **Put Vf (and about 2 % loss) on the no-ring HS-diode return path**, for the cases where the ring stays off.

## 11. Parameters that could not be matched

- flat HS part unknown ("2p CSD"); 2x IPP040N08NF2S used as a proxy.
- The ST15100S on fry has no vendor model; a generic 15 A/100 V Schottky was used for the sensitivity run only.
- Coil core loss and Q: none in dcdc-tools; B-rp uses A's own Q, so damping is not independently validated. DCR
  values are estimates (fry 2 mΩ, flat 5 mΩ).
- The Fugu2 parasitics fixture is stale: it pins pcb_sha256 32906787c3c5 and the current Fugu2.kicad_pcb hashes to
  b70bbf09c11c. Runs used `--allow-stale-parasitics`.
- Tj fixed at 40 °C (DCM benches) and 50/80/100 °C (CCM); constant L, no DC bias in both models.
- A is run at pwmMax 4103 (fry/flat MCPWM); the vconv_mock retunes use 2047.
- The rcal-sun soft PV source (Isc 5.6 A, Voc 79.6 V) was not reproduced; Vin is stiff here.
- dcdc-tools decks leave LTspice's default 1 mΩ Rser on `Ldrv`/`Lscs`/`Lg` (only the inductors in the bench rewrite
  get `Rser=0`). Possible small conduction-loss inflation; not verified.

## 12. Tooling notes

- The documented dcdc-tools venv (`~/dev/pv/ee/venv`) does not exist, and `~/dev/pv/venv` lacks `yaml`. Used
  `~/dev/venvs/fetlib` with `~/dev/pv/ee/sw-phases` on `PYTHONPATH`.
- `loss.py --body-diode lm` on the CLI did not override the YAML `tt`; per-board YAML copies with `lm` were used
  (`/tmp/vconv-spice/fry.yaml`, `flat.yaml`).
- No SPICE tool was missing. LTspice 17.2.4 ran all decks.

## 13. Files (`/tmp/vconv-spice/`)

- Bench: `bench.py` (deck rewrite, run cache `runs/`, analysis), `loss-fry-lm/`, `loss-flat-lm/`, `loss-flu/` (base
  decks), `fry.yaml`, `flat.yaml`.
- Model A: `a/a_harness.cpp`, `a_run.py`, `exp_a.py` → `results/a_all.json`.
- Experiments: `exp_ls.py` (LS sweeps), `exp_dawn.py` (P1), `exp_p2.py`, `exp_p2_rp.py` (P2), `exp_ccm.sh` → `ccm/`
  (P2-P4 CCM), `exp_sch.py` (Schottky), `run_all.sh`.
- Analysis: `analyze_ls.py`, `analyze_knee.py` + `fit/knee.cpp`, `analyze_dawn.py`, `tables.py`, `plot_ls.py`,
  `plotwave.py`.
- Results: `results/ls_sweeps_b.json`, `ls_summary.json`, `knee_summary.json`, `ls_sweeps.png`, `dawn_b.json`,
  `dawn_summary.json`, `dawn_table.md`, `p2.json`, `p2_rp.json`, `ccm_table.md`, `schottky.json`, `flu_ring.json`.
