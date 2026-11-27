*this document is an LLM generated placeholder*

<!-- Codex xhigh review of measure-coil at 4f335207, 2026-11-27; claims verified by Claude where noted in the session -->
# ACCESS

```text
$ git -C /Users/fab/dev/pv/fugu-mppt-firmware log --oneline -1 4f335207
4f335207 mppt-sim.sh: create OUT's directory before linking
```

Playwright navigated `about:blank` and read its DOM, then fetched
[Sendust](https://en.wikipedia.org/wiki/Sendust). Live `<title>`: **Sendust - Wikipedia**;
heading: Sendust. No challenge, no fabchrome use. This is an access check, not
the evidence for the electrical model.

# FINDINGS

Review of **4f335207**, completed 2026-11-27. Tags: **(a)** arithmetic error; **(b)** assumption that holds only under stated conditions; **(c)** design defect; **(d)** advice/experiment. Firmware `file:line` references below are at that commit; copies are under `pinned/`. The saved report is [FINDINGS.md](/tmp/codex-review-coilfit/FINDINGS.md). Log references are to `mc-log-2026-11-24-27.txt`. The precharge variant is assessed separately. No firmware, configuration, or field device was changed.

**Recommendation: keep these measurements diagnostic; do not automatically persist the fitted correction yet.** The ideal boost mapping, offset sign, and iteration survive. The 400 ns test is a conditional model check, not a calibration certificate. There are reproducible paths that persist an unevaluated or substantially wrong result. The data support a repeatable direction-dependent measurement difference, but do not identify its physical cause.

## 1. Model, sign, timing, and uncertainty

**(b) The ideal mapping is correct.** Let `H` be the high-side rail voltage, `V` the low-side rail voltage, `A` the positive coil ramp voltage, `B` the magnitude of its discharge voltage, `τ=D/f`, and `J` the **average coil current**. For a constant inductance starting at zero:

```
Ipk = A τ/L
Q = Ipk (τ + L Ipk/B)/2
J = f Q = A(A+B)D²/(2 B f L).

buck:  A=H−V, B=V,   J=Iout
boost: A=V,   B=H−V, J=Iin
```

Thus boost gives `L=V H D²/[2(H−V) f Iin]`. Substituting `(vin',vout')=(H,H−V)` into the buck formula gives exactly this result, its idle interval, and its peak-current relation. **Do not complement boost D or reverse the offset sign.** `coil_fit.h:36–45`; `measure_coil.cpp:239–250`. The mapping requires true average coil current and the actual coil-terminal voltages; it does not magically map sensor errors or nonideal components.

**(b) Fixed effective on-time error.** If actual time is `τ+δ`,

```
Lraw/Ltrue = [D/(D+fδ)]² ≈ 1 − 2fδ/D
1/Lraw ≈ (1/Ltrue)(1 + 2fδ/D).
```

Positive δ makes uncorrected L low, especially at low duty, and produces an apparent **positive** L-versus-current slope. Negative δ does the reverse. This sign is the same for buck HS drive and boost LS drive. `coil_fit.h:162–174,261–272,324–335` correctly adds the fitted residual to the accumulated offset and reevaluates the duties. Scratch ideal-data probes recover −300, 0 and +200 ns in both directions; corrected L is within 0.02% of the planted 48.6 µH. The −201…−353 ns boost estimates therefore cannot, taken literally as the sole error, explain a *low* uncorrected boost L: applying them would lower it further.

**(b) Dead time is not generally missing control on-time.** For this MCPWM implementation the control pulse is `Hcount/periodCounts()`, with a possible +1 timer-tick extension. The control generator remains the control generator in either topology; the named `hl` delay postpones the **rectifier** edge. The other band reserves wrap space. Subtracting both dead times from D would introduce an error. `buck.h:1686–1705`; `pwm/mcpwm.h:202–213`. The measurement uses the full period and realized frequency (`measure_coil.cpp:175–177,239`). At a 160 MHz timer, one tick is 6.25 ns, far short of 200–350 ns.

The following first-order expressions distinguish mechanisms. They assume constant L, positive forward current, ideal switching outside the stated perturbation, and small errors. They are derivations, not measured device corrections:

| Perturbation | Relative error in the nominal inversion, `(Lhat−L)/L` | Direction reversal |
|---|---|---|
| Effective control pulse longer by δ | `−2δ/τ` | No; control FET changes from HS to LS |
| Rectifier delayed by td after control turn-off, diode adds Vf during td | `+2 Vf td/[(A+B)τ]`, for `td ≪ L Ipk/B` | No; diode voltage polarity mirrors, lost forward charge does not |
| Entire discharge through body diode | `+[A/(A+B)] Vf/B` | No; magnitude changes |
| Rectifier turns off early at residual current `r Ipk`, diode finishes | `+[A/(A+B)] [Vf/(B+Vf)] r²` | No |
| Rectifier remains on ε after zero, then reverse current resets through opposite diode | `+(B/A)(ε/τ)²` to leading order, neglecting diode drops | No; this is quadratic at the ideal zero, not a first-order fixed δ |
| Ring leaves signed initial coil current i0 at control turn-on | `−2 i0/Ipk`, plus any additional idle-interval charge term `−f Qidle/J` | Phase-dependent; no universal sign reversal |

For the delayed-rectifier row, the lost charge is approximately `Vf Ipk td/B`; dividing by ideal `Q=Ipk τ(A+B)/(2B)` yields the expression. For early turn-off only the remaining triangular tail changes. For overrun, reverse current `Bε/L` removes two negative triangular areas, totaling `Bε²(1+B/A)/(2L)`. Do not add these formulas twice for the same interval. If LS turns on **after** natural zero, or a minimum pulse itself drives negative current, use the actual negative-current waveform, not the positive-current dead-time formula. A physical LS edge is rectification in buck and excitation in boost; specify its role before assigning a sign.

**(c) The real `l0` rectifier regime is not the one assumed by the narrative and most tests.** Starting the task stops conversion (`measure_coil.cpp:396`; `mppt.h:1471`), and disable clears sync (`buck.h:1519`). `l0` sets `mcBfZero` but never sets `mcSyncRect`; only `sweepLs` does (`measure_coil.cpp:211,309`). The RT loop enables sync on BF closure only when that flag is true (`mppt.cpp:503–513`). With reverse-current paranoia enabled, buck therefore uses **pwmRectMin**, normally a 2 µs bootstrap pulse plus its reserved dead time, followed by a substantial diode tail; boost's rectifier minimum is zero (`buck.h:1328–1335,1459–1461`). With paranoia disabled, startup enables sync (`mppt.cpp:532–535`), so the experiment also depends on an unreported configuration setting.

Paranoia defaults on (`mppt.h:67`). The claim that normal `l0` always turns LS off only about 2% early with a <0.1% diode-tail error does not follow (`website/docs/lab/coil-inductance.md:437–440`). The test plant explicitly sets nearly full synchronous discharge (`test/host-stub/coil-fit-test.cpp:32–35`). This is a confirmed **code/documentation/test precondition mismatch**, not proof of each historical board's gate waveform: the supplied log lacks sync/paranoia state. A body-diode-only illustration at H=70 V, V=26.5 V, Vf=0.7 V gives about +1.64% buck bias versus +0.61% boost bias. The 2 µs buck pulse reduces its tail contribution. That can contribute to buck reading higher, but does not by itself establish fry's full gap.

**(b,d) DCM ring symmetry is conditional.** With signed current positive in the power-flow direction, the same `−2i0/Ipk` result applies after mirroring. Physical current sign, equilibrium node voltage, and rail clamping change. In ideal buck the node rings about V after starting near zero; if H>2V it does not reach the opposite rail on a lossless first swing. In boost it starts near H and rings about V; for H>2V its first downward swing can hit the low rail and be diode-clamped. Thus similar 250/320 kHz fitted frequencies do not imply the same waveform or zero mean. Nonlinear Coss and diode clamps make the inferred `C=1/(4π²fr²L)` an effective model value. The harmonic fit removes an oscillating component; it does not measure or remove every cycle-average charge offset (`coil_fit.h:245–258`). Ring phase can move L either way. There is no justified universal “boost timing flips sign” correction.

**(b,c) Sigma is conditional, not a bound on all physical error.** The least-squares diagonal, units, division by residual degrees of freedom, uniform-bound `/√3`, and root-sum-square combination are internally sensible. The ±0.5%/A slope is a **uniform assumed bound**, not an experimentally established Gaussian sigma (`coil_fit.h:31,265–272`). For a general ratio `δres=10⁻⁶ β/c0`, full variance is

```
(10⁻⁶/c0)² [Var(β) + (β/c0)² Var(c0) − 2(β/c0) Cov(β,c0)].
```

The implementation retains the first term. Near a converged residual β≈0 this is the correct first-order limit; away from convergence it is incomplete. There are at most four refits and no explicit failure for nonconvergence (`coil_fit.h:330–335`). More consequentially, ring frequency/model selection is held fixed; the correlation inflation comes from the separate slope fit; voltage calibration, diode-regime error, switching-dependent sensor zero, and unknown temperature are not covered. The propagated current-zero and slope responses also condition on the fitted phase/current axes. “2 sigma” must not be presented as a calibrated 95% hardware guarantee.

## 2. What the log says, and a concrete persistence criterion

**Verified latest values:** the question's latest-pair L and δ numbers match the log (`236–277,287–325`). Two relevant omitted runs are flat buck on 11-26 11:11 (**48.72 → 53.40 µH**, +215 ns, applied), and flat boost at 23:18 (**49.01 µH**, −261 ns, FAIL). The plotting script excludes both. “Applied” on the offset line means the correction was used for the reported result; it does **not** establish a flash write. No `applied coil.conf L0=` line appears in this extract.

The current rule is stricter than just 400 ns: it also requires `σδ≤80 ns` and corrected duty span ≥2 for correction, but statuses 1–3 can persist the uncorrected value (`coil_fit.h:315–342`). The guard does not require δ to be reproducible across frequency.

| 11-27 board | δ difference | 2× combined noise sigma | Raw L spread | Corrected L spread |
|---|---:|---:|---:|---:|
| fry, 39/21.5 kHz | 97 ns | 104 ns | 1.10% | 2.23% |
| flat, 39/21.5 kHz | 184 ns | 94 ns | 3.28% | 1.99% |

Spreads are `2|L1−L2|/(L1+L2)`. Including both systematic sigmas as independent would give 172 ns for flat's 2-sigma δ difference, still below 184 ns; shared systematic terms actually require covariance, not blind quadrature. This is a warning about repeatability, not a decisive probability calculation. Fry's +67 ns is not significantly nonzero. Flat's +203 ns meets 400 ns because `203+2 hypot(29,57)=331 ns`; that says nothing about agreement with +19 ns. The rounded fry boost numbers give 399.7 ns although the firmware says FAIL: unrounded values control the boundary. This is not evidence of an arithmetic bug.

**(d) Proposed commissioning gate, explicitly conservative:**

1. Until calibration and the actual rectifier/ring regime are bounded, report `UNVERIFIED` and refuse automatic `apply` in **both** directions. Preserve raw and corrected candidates.
2. A candidate run must have ≥24 retained finite points, raw **and** corrected duty span ≥2, finite uncertainties, <1 ns final iteration change, no trip/cancellation/stale input, and a per-row DCM margin using the row's voltages and corrected duty. Retain the existing 400 ns and 80 ns tests. Median fallback, unresolved model, or an unevaluated condition is diagnostic-only.
3. Require a same-board, same-direction pair within 15 minutes, frequency ratio ≥1.5, unchanged calibration/drive settings, and repeatable thermal/source conditions. For this constant-L correction, require **both raw and corrected aggregate L spreads ≤2%**, and `|δ1−δ2|≤100 ns` **and** `≤2 hypot(noise1,noise2)`. These are proposed engineering acceptance limits to validate, not manufacturer limits or universal statistical thresholds. If current bands differ enough for real bias to matter, reject this scalar-L test and repeat over matched current ranges; do not waive it by assuming zero bias.
4. For eventual persistence, compare fits over a common current interval, then explicitly derive the quantity the firmware wants (`L0` versus finite-current effective L). Check subset/up-down predictions and use a joint model with a common coil curve, direction-specific sensor/drive terms, and shared physical-time offsets. Do not force the coil slope to zero independently in every run. Record a sensor/config/code fingerprint and validation state beside any saved value.

The summary-level cross-frequency check is intentionally a refusal screen, not proof of accuracy. It would refuse both fry 11-27 runs on corrected-L disagreement; flat fails on raw-L and δ disagreement, and its 21.5 kHz run also has only 20 points. Older summaries lack the offset/uncertainty information to pass this rule. A 2.5× duty requirement would be a poor drop-in suggestion: the present current floor at 20% of maximum already caps an ideal constant-voltage quadratic sweep at about `√5=2.24×` after filtering (`measure_coil.cpp:258–263`). Wider coverage needs a different floor justified by current SNR, or more excitation conditions.

The table below classifies **every started L0 sweep** in the supplied log. `U` means refuse persistence because required evidence is missing; `F` means a known gate fails; `A` means startup aborted; `N` means no result is present. Earlier firmware results cannot be retrospectively refitted from this extract: it contains the `measure-coil` summary lines, not the numerical sweep rows. All entries would remain diagnostic under the commissioning gate.

| # / log line | Start (November) | Board / direction / kHz | Raw → reported L, µH | Logged δ/status | Proposed result |
|---|---|---|---|---|---|
| 1 / 8 | 24 11:52 | flat / buck / 39.0 | 49.74 → 49.74 | not estimated in log | U: missing timing evidence |
| 2 / 15 | 24 12:19 | flat / buck / 39.0 | 48.35 → 48.35 | not estimated in log | U: missing timing evidence |
| 3 / 26 | 24 12:42 | flat / buck / 39.0 | 49.50 → 49.50 | not estimated in log | U: missing timing evidence |
| 4 / 40 | 24 13:33 | flat / buck / 39.0 | 49.50 → 49.50 | not estimated in log | U: missing timing evidence |
| 5 / 49 | 24 13:45 | flat / buck / 39.0 | 49.49 → 49.49 | not estimated in log | U: missing timing evidence |
| 6 / 57 | 24 13:55 | fry / buck / 39.0 | 76.18 → 76.18 | not estimated in log | U: missing timing evidence |
| 7 / 60 | 24 13:56 | flat / buck / 39.0 | 49.84 → 49.84 | not estimated in log | U: missing timing evidence |
| 8 / 73 | 24 16:13 | flat / buck / 39.0 | 49.67 → 49.67 | not estimated in log | U: missing timing evidence |
| 9 / 81 | 24 16:14 | flat / buck / 39.0 | 49.20 → 49.20 | not estimated in log | U: missing timing evidence |
| 10 / 88 | 24 21:41 | fry / boost / 39.0 | — | — | A: precharge |
| 11 / 93 | 24 21:42 | fry / boost / 39.0 | 74.69 → 74.69 | not estimated in log | U: missing timing evidence |
| 12 / 100 | 24 21:44 | fry / boost / 21.5 | 73.76 → 73.76 | not estimated in log | U: missing timing evidence |
| 13 / 108 | 24 21:51 | flat / boost / 39.0 | 48.83 → 48.83 | not estimated in log | U: missing timing evidence |
| 14 / 115 | 24 21:53 | flat / boost / 21.5 | 48.60 → 48.60 | not estimated in log | U: missing timing evidence |
| 15 / 123 | 24 23:47 | fry / boost / 39.0 | 74.56 → 74.56 | not estimated in log | U: missing timing evidence |
| 16 / 130 | 24 23:56 | flat / boost / 39.0 | 48.90 → 48.90 | not estimated in log | U: missing timing evidence |
| 17 / 137 | 25 08:14 | fry / buck / 39.0 | 76.47 → 76.47 | not estimated in log | U: missing timing evidence |
| 18 / 144 | 25 08:22 | flat / buck / 39.0 | 49.64 → 49.64 | not estimated in log | U: missing timing evidence |
| 19 / 151 | 25 08:23 | flat / buck / 30.0 | 51.52 → 51.52 | not estimated in log | U: missing timing evidence |
| 20 / 159 | 25 08:24 | flat / buck / 22.0 | 49.57 → 49.57 | not estimated in log | U: missing timing evidence |
| 21 / 167 | 25 08:25 | fry / buck / 39.0 | 76.75 → 76.75 | not estimated in log | U: missing timing evidence |
| 22 / 174 | 25 08:27 | fry / buck / 21.5 | 77.46 → 77.46 | not estimated in log | U: missing timing evidence |
| 23 / 176 | 25 08:27 | flat / buck / 39.0 | 49.49 → 49.49 | not estimated in log | U: missing timing evidence |
| 24 / 191 | 26 11:11 | flat / buck / 39.0 | 48.72 → 53.40 | +215 ns / applied | U: no frequency pair |
| 25 / 218 | 26 22:13 | flat / boost / 39.0 | — | — | N: no result |
| 26 / 224 | 26 22:16 | flat / boost / 39.0 | — | — | N: no result |
| 27 / 228 | 26 22:19 | fry / boost / 39.0 | — | — | A: precharge |
| 28 / 232 | 26 22:19 | fry / boost / 39.0 | — | — | A: precharge |
| 29 / 236 | 26 22:20 | fry / boost / 39.0 | 74.59 → 74.59 | -201 ns / FAIL | F: existing timing guard |
| 30 / 244 | 26 22:22 | fry / boost / 21.5 | 74.15 → 74.15 | -305 ns / FAIL | F: existing timing guard |
| 31 / 253 | 26 22:37 | flat / boost / 39.0 | 48.89 → 48.89 | -329 ns / FAIL | F: existing timing guard |
| 32 / 261 | 26 22:39 | flat / boost / 21.5 | 48.65 → 48.65 | -353 ns / FAIL | F: existing timing guard |
| 33 / 270 | 26 22:40 | flat / boost / 20.0 | 48.40 → 48.40 | -340 ns / FAIL | F: existing timing guard |
| 34 / 279 | 26 23:18 | flat / boost / 39.0 | 49.01 → 49.01 | -261 ns / FAIL | F: existing timing guard |
| 35 / 287 | 27 10:07 | fry / buck / 39.0 | 77.13 → 79.05 | +67 ns / applied | F: corrected pair spread >2% |
| 36 / 297 | 27 10:08 | fry / buck / 21.5 | 77.98 → 77.31 | -30 ns / applied | F: corrected pair spread >2% |
| 37 / 308 | 27 10:10 | flat / buck / 39.0 | 49.95 → 50.35 | +19 ns / applied | F: raw pair/δ; pair rejected |
| 38 / 318 | 27 10:11 | flat / buck / 21.5 | 48.34 → 51.36 | +203 ns / applied | F: raw pair/δ; 20 points |

Additionally: the two LS sweeps on 11-26 at 11:44 and 11:49 report a failed peak fit and a raw argmax (`201–207`); they are not L0 measurements. Four precondition refusals at 22:11:13/22:11:54/22:12:02 on flat and 22:14:27 on fry fail `Vin>Vout+1` before a sweep (`208–222`). Other received-command-only entries, including fry's 11-25 `f=15000`, provide no result and authorize no inferred calibration.

## 3. Sensor sensitivities and the direction gap

**(b) Voltage errors reverse their roles.** Let `eH,eV,eJ` be reading-minus-true errors and `gH=eH/H`, etc. Hold the physical operating point fixed:

```
buck:
 ΔL/L = (1/H + 1/(H−V)) eH − (1/V + 1/(H−V)) eV − eJ/J
      = [(2H−V)/(H−V)] gH − [H/(H−V)] gV − gJ

boost:
 ΔL/L = (1/H − 1/(H−V)) eH + (1/V + 1/(H−V)) eV − eJ/J
      = −[V/(H−V)] gH + [H/(H−V)] gV − gJ.
```

Here buck H/V are Vin/Vout and boost H/V are Vout/Vin. J is buck Iout or boost Iin; the other current reading does not directly enter the formula. For offset and gain together, substitute `eX=gX X+oX`. These follow directly from `coil_fit.h:39–45,172`.

At H=70 V, V=26.5 V, the voltage gain coefficients are **buck (+2.609, −1.609)**, **boost (−0.609, +1.609)**. Therefore a common HV gain error +1.24%, or LV gain error −1.24%, can produce approximately a 4% buck/boost difference even if the same physical channels are reused. A shared LV reading error around −0.33 V can do the same. If only boost's LV measurement is wrong, about −0.66 V is needed for −4%. At fry's roughly 78/27 V, the corresponding coefficients are (+2.529,−1.529) and (−0.529,+1.529). These are illustrative operating points; the boost header voltage is **before precharge**, not the sweep's HV voltage.

A boost coil-current reading +4% high makes L about 4% low. A residual +20 mA at 0.5 A does likewise, but fades to 0.4% at 5 A. A constant positive current offset makes L rise with current; a negative residual offset makes it fall. Thus the negative boost slopes by themselves do not support a simple positive current offset as the sole explanation of both slope and mean. Off-state zero correction does not remove a switching/common-mode-dependent error.

**(b) A resistive BF input drop has the wrong direct sign for low boost L.** If the boost voltage sensor reads pack V but the coil actually sees `V−Δ`, then

```
Lhat/L ≈ 1 + [1/V + 1/(H−V)] Δ.
```

It raises inferred L. In buck, a pack-side sensor below the coil terminal by Δ also raises L with the same coefficient. Extra winding/series resistance likewise reduces current relative to the ideal source-voltage calculation and normally raises L. A half-on BF can still alter the waveform, cause intermittent conduction, or draw pump current included in the sensor; those are different mechanisms. A low-reading coil-side voltage sensor or an incorrectly assigned sensing node can have the opposite effect. `bf 1` and successful precharge establish a request/voltage-follow condition, not a milliohm measurement.

**Ranked causal assessment (inference, not a diagnosis):**

1. **Differential voltage calibration/sense-node error**, then direction-dependent current gain/common-mode error. They naturally produce persistent direction offsets with little frequency dependence, and have enough sensitivity at sub-volt or few-percent errors. Verify which physical channel and transform each topology actually uses before claiming two independent current sensors. Agreement with the 11-25 host figures, 76.78/49.25 µH, confirms execution repeatability but shares the converter's sensors and inversion; it is not independent absolute metrology (`doc/research/2026-11-25-coil-bf-open-vs-closed.md:154,165,271`).
2. **Actual rectifier regime and ring/transition charge**, including bootstrap-related effective gate timing. These have a confirmed code/test mismatch and can give duty-dependent biases. The larger buck diode-tail bias has the right direction; asymmetric boost rail clamping is a reason to test the ring model. Pure fixed δ alone is weakened by the observed negative boost δ and by lack of simple frequency scaling; offsets fitted from slope are not measured gate delays.
3. **Current zero/nonlinearity under switching.** The zeros vary substantially across sessions; the subtraction is helpful but an on-state error is not bounded by two off-state endpoints. Stable-gain error is more compatible with a stable mean gap than a single additive offset, whose relative effect should shrink with current.
4. **Core DC bias and different sampled current ranges.** Real bias can explain a modest lower boost mean, especially flat at 6–13 A, and some negative slopes. It cannot establish the whole same-current direction gap. See the quantitative prediction below.
5. **Temperature/history.** Night/day is confounded with direction, including copper, core, diodes and sensors. There are no coil-temperature measurements here, so neither size nor sign is established. Controller/board temperatures are only proxies.
6. **Pure BF channel drop as the cause of low boost L.** Startup trouble is real, but its simple steady loss term raises boost L. Keep the gate-pump/waveform hypothesis separate from that sign-incompatible explanation.

For the latest runs, fry buck exceeds boost by 3.35% at 39 kHz and 5.04% at 21.5 kHz, using pair means as denominator. Flat is +2.14% at 39 kHz but **−0.64%** at 21.5 kHz uncorrected. Thus “flat boost is always 1–3% low” does not describe every run. Different current bands and the anomalous low-frequency buck result matter.

## 4. What can be learned about the core curve

**(b) Compare the right inductance.** If incremental `l(i)=dλ/di` is nonlinear but reversible and rate-independent, define

```
λ(P) = ∫₀ᴾ l(i) di
E(P) = ∫₀ᴾ i l(i) di
τ = λ/A;      J = f (1/A + 1/B) E
L_DCM(P) = λ²/(2E)
Ipk_reported = 2E/λ, not exactly the true peak P.
```

This derivation also survives in the existing documentation (`website/docs/lab/coil-inductance.md:611–618`). For `l=L0(1+c i)`, the DCM result is `L0(1+c P/3)` to first order; simply reading the incremental vendor slope at the logged peak would overstate the effect.

Use the **supplied printed coefficients**, not the unvalidated digitized traces: 125u `(b,c)=(1.34e−3,1.780)` and 60u `(3.57e−4,1.748)`, with `l/L0=1/[1+b Hfield^c]` and `Hfield=0.4π N i/le_cm`. The field is per core; stacking two cores doubles AL but does not double this magnetic path length. Conditional geometry: fry N=12 and le=10.740 cm (supplied `README.md:123–126`); flat N=20 and le=8.125 cm (read-only local `pwr-mosfet-lib/maglib/cores.py:294–298`, **not reverified against the vendor sheet in this review**). Turns remain unknown. No interface dimensions or turn counts were measured or inferred as fact.

For small bias, writing `q=b(0.4πN/le)^c`,

```
l/L0 ≈ 1 − q P^c
L_DCM/L0 ≈ 1 − 2q P^c/[(c+1)(c+2)].
```

Numerical quadrature of the full law, transformed onto the reported-current axis and fitted over each band's range, gives:

| Board / run | Reported peak range A | Predicted DCM slope %/A | Observed uncorrected %/A |
|---|---:|---:|---:|
| fry boost 11-26, 39 kHz | 2.2–5.5 | −0.232 | −1.06 ±0.33 noise |
| fry boost 11-26, 21.5 kHz | 3.3–7.4 | −0.296 | −0.81 ±0.15 |
| fry buck 11-27, 39 kHz | 2.3–5.0 | −0.224 | +0.56 ±0.50 |
| fry buck 11-27, 21.5 kHz | 2.9–6.4 | −0.268 | −0.28 ±0.18 |
| flat boost 11-26, 39 kHz | 3.6–7.9 | −0.310 | −0.98 ±0.28 |
| flat boost 11-26, 21.5 kHz | 5.7–12.9 | −0.423 | −0.46 ±0.11 |
| flat boost 11-26, 20 kHz | 6.0–13.2 | −0.431 | −0.45 ±0.14 |
| flat buck 11-27, 39 kHz | 2.6–5.7 | −0.247 | +0.18 ±0.41 |
| flat buck 11-27, 21.5 kHz | 4.0–7.8 | −0.316 | +1.16 ±0.20 |

Across all logged numeric runs the conditional predicted slopes are roughly −0.18…−0.33%/A for fry and −0.22…−0.43%/A for flat. Flat's low-frequency boost agreement and fry's low-frequency buck agreement are **consistent with** the law. They do not validate it: direction-dependent slope error has not been independently bounded. Using only the code's reported `±(2 noise + offset-bound)` envelopes, the latest fry and boost-flat predictions fit; flat buck 21.5 kHz does not (its reported interval is +0.20…+2.13%/A). Flat's +215 ns run and its 11-24 13:56 run also exclude the predicted negative slope under that incomplete envelope. Correcting each curve by assuming zero slope cannot then be used as evidence that the core has zero slope.

For flat, predicted `L_DCM/L0` goes from about 0.990 to 0.959 over 5.7–12.9 A; a few-percent finite-current reduction is plausible. For fry, 2.2–5.5 A gives about 0.998 to 0.990. Its much larger direction offset is not explained by that small same-band core reduction alone. Vendor small-signal DC-bias data are also not a guaranteed large-excursion hysteretic trajectory; the integral prediction is conditional on that equivalence.

To resolve the curve, first bound sensor/drive errors with matched-current, multi-frequency, both-direction data and controlled thermal history. Then fit a shared nonlinear λ(i) model without imposing a zero slope. A direct resolution of incremental L ultimately needs calibrated coil voltage and time-resolved current (`l(i)=vL/(di/dt)`), or a small-signal inductance measurement versus DC bias, plus actual turn counts. The two converters and averaged console telemetry can falsify nuisance models and improve consistency; they cannot alone supply an independent instantaneous current waveform or an absolute calibration reference for every sensor.

## 5. Additional persistence defects and guard review

**(c) P1 — Less evaluable data can change refusal into persistence.** `fit` returns a median below 16 points (`coil_fit.h:182–185`); `dlStatus` maps a nonfinite/out-of-bound estimate to status 1 when `n<16` (`317`), and `persistL` allows statuses 1–3 (`340–342`). Reproduced with true L=48.6 µH, +600 ns, 80/26.6 V: n=16 refuses; n=15 permits **36.032 µH**, with δ and its errors NaN. n=3 also permits it. This is a monotonicity and unevaluable-input failure, not a hypothetical concern. A narrow-band +200 ns example also falls back and permits 43.059 µH. Missing evidence must produce `UNVERIFIED`, not a writable fallback.

**(c) P1 — A protection abort can still leave a writable partial sweep.** `measure_coil.cpp:238` breaks after trip/disable, then proceeds through zero reading, fitting and return (`255–293`). Neither `s_tripped` nor `cancelled()` is checked at the persistence decision. If enough earlier points exist and the off-state zero is stable, `apply` can write them (`431–435`). The code has no explicit “partial after fault” validity state. Normal planned stops at current/DCM bounds need not be treated like faults; retain their distinction. This is a verified control-flow path; no field flash write after such a fault was observed in the supplied summaries.

**(c) P1 — Boost does not require a physical coil-current sensor.** The sensor check requires physical Iout only for buck (`measure_coil.cpp:404–407`), while `coilI()` selects Iin in boost (`40`). A valid configuration can have physical Iout but virtual Iin computed from output power and assumed efficiency (`adc/sensor_setup.cpp:298–308`; `mppt.h:374–384`). That substitutes an efficiency-model current for the independent average coil current, biases L, and also changes the meaning of the boost input-current trip. Refuse virtual coil-current inputs or implement and validate an explicitly separate output-current method. This is a configuration-dependent defect; it is not evidence that fry or flat used virtual Iin.

**(c) P2 — DCM validity is partly circular and not checked after correction.** The accepted-row test uses `r.dcm` (`measure_coil.cpp:244`), whose predictor depends on the configured inductance (`buck.h:1576–1590`). The buck band is chosen from initial voltages, and boost's extra margin is checked before the next row (`measure_coil.cpp:183–184,229–245`). `fit` checks positive effective duty but not `D_eff < vout'/vin'` (`coil_fit.h:163–174`). An incorrect old L, voltage movement, or positive timing correction can therefore leave an inadmissible row in a plausible fit. Add a fresh per-row geometric DCM margin, including after offset correction, independent of the old L. The summary extract cannot establish that this occurred on these boards.

**(c) P2 — A no-ring accepted result can have no L uncertainty.** `se/sys` remain NaN unless `r.nh` is nonzero (`coil_fit.h:275–279`), while the timing fit can pass and `persistL` ignores those fields. The ideal no-ring probes demonstrate status 0 and finite persisted L with NaN L errors. A genuine low-ring case should compute valid no-ring uncertainty, rather than be required to invent a ring. A +4% current-gain probe passes with δ≈0 and L=46.731 µH instead of 48.6: a useful known-bad example of what the δ guard cannot detect even with many points.

**(b,d) Finite-current L is not automatically L0.** The estimator returns a mean effective L over its sampled band (`coil_fit.h:15,227`), but stores it under `L0` (`measure_coil.cpp:292,434–435`). A scalar controller may intentionally use an effective value, but that choice and its operating band need explicit provenance; applying the controller's additional DC-bias factor does not recover a measured zero-current intercept. The simulator's slope test uses a per-cycle peak-dependent constant L (`sim/vconv.h:243`), not the integral vendor constitutive law. It validates estimator behavior under that model, not physical core-curve recovery.

**(d) Precharge variant:** `measure_coil-precharge-3c87a847.cpp:195–213` ramps the target to `pwmCtrlMax/5`; for boost's 0.9-period maximum that is about 18% of the available PWM span, slightly less of the full period when wrap dead time exists. It pauses target increments when Vout rose >0.25 V over the last interval and retains the three-second failure path. That addresses the observed fixed-320-count startup failure (`log:228–234`), but it is not HW-tested here and is not a resistance/calibration check. `Vout>Vin−3` allows a large drop and can be satisfied by residual capacitor charge. The subsequent zero/read/restart can change pump state again. Record actual reached duty, verify sustained conduction during the accepted rows, and retain protection abort as a veto; do not interpret ramp success as proof that the boost input path is ideal.

**Guard checklist applied to the proposed change:**

- Unevaluable: explicit `UNVERIFIED` for missing errors/rows/physical sensor/model validation; nothing persisted.
- Monotonicity: reject low-point and bad-offset tails independently; the reproduced 16→15 transition must no longer enable a write.
- Preconditions: log actual sync state, physical sensor identities, row freshness, realized period and DCM margin. Exercise the actual firmware path, not just the fit helper.
- Source of truth/provenance: fingerprint fit code and acquisition code, calibration transforms, topology, drive configuration and raw rows. Keep raw, corrected, assumed slope and saved quantity distinct.
- Persistence: a failed or aborted acquisition cannot set a verified state; absence of a cache today does not remove the requirement for any future cached validation.
- Known bad: include the +600 ns low-n probe, virtual-input configuration, post-row protection abort, and +4% gain probe. A small residual alone is not success.
- Fix versus mute: disabling automatic persistence closes the unsafe write path but does not repair metrology; correction requires the discriminating experiments below.
- Runtime: no production guard was changed or accepted. The scratch summary-screen prototype (`guard_screen_bench.cpp`) refuses the low-point example and both 11-27 pairs; its measured timing is in `guard-screen-output.txt`: 1e6 pair screens 7.842ms accepted=117185; no sampling/fitting/MCU cost included. This times only the additional summary predicates, excluding acquisition, fitting, raw-row checks and final ESP32 integration; measure those on the MCU before accepting a firmware implementation.

# EXPERIMENTS (ranked)

1. **Read the measurement contract and compare pack voltage — cheapest, no calibration writes.** On each board record `sensor` (physical/virtual and transforms), `sensor avg`, `pwm-dump`, `pwm-freq`, `dt`, `sync`, and `get-config limits.conf reverse_current_paranoia`; capture the status stream and BMS pack voltage at the same timestamps. Compare BMS pack V with buck Vout and boost Vin, first idle and then at a few low-current points. Compare the two converters against the same pack when they really share it. A repeatable −0.3 V class LV error could explain much of a 4% direction difference; an error increasing with current favors path drop or common-mode behavior. BMS voltage provides an independent check, with its own accuracy and pack/cable-node differences recorded; it does not calibrate the HV divider or current gain by itself. Commands are defined at `cli.cpp:185–199,293–351,354–398,1772–1800,1906`.

2. **Interleave frequencies and dwell, saving the full numerical rows.** From stopped manual mode run, without `apply`, `dc 0`, then `measure-coil l0 64 2000 f=39000`, `... f=21500`, and repeat 39000; add 30000 as a third independent frequency if useful. The accepted upper frequency is the configured nominal, so use that nominal instead of 39000 if lower (`cli.cpp:2446–2449`). Wait for each run to finish. Repeat in each direction under its valid source conditions. Repeat selected runs at 4000 ms dwell. Flat's +19/+203 ns split should reproduce at the same frequency if it is a fixed drive effect; movement with dwell/order favors settling/zero/thermal effects. Refit a common Ipk band (approximately fry 3.3–5 A, flat 4–5.5 A where available) using each row's actual voltages. Constant time error scales with `f/D=1/τ`, current offset with `1/J`, and additive per-cycle charge with `f/J`. A same-Ipk frequency test alone can hold τ nearly fixed, so also vary pulse width/current or voltage ratio: do not claim it independently separates all three.

3. **A/B the actual buck rectifier, then dead time — diagnose the code/test mismatch.** `sync` can be queried during `measure-coil`, but changes are refused; setting `sync on` before `l0` does not survive its stop/restart. Use controlled manual points instead: `pwm-freq <f>`, `dc <count>`, let forward current settle, `bf 1`, then compare `sync off` and `sync on` with `sensor avg` at the same count. Choose counts from `D=count/periodCounts` in the already observed DCM band, e.g. buck D≈0.12–0.22 after verifying `D<0.8 V/H`; keep the normal protections active. Compute nominal L offline and record actual LS count. A current increase/L decrease with full diode emulation quantifies the diode-tail bias. Use `measure-coil ls <hs> 3000` only as a diagnostic: its peak is ring-modulated and the two logged fits failed. For the dead-time A/B, first read `dt`, increase only the control-off→rect-on gap by 100 ns within allowed limits, repeat, then restore the recorded values while stopped. The prediction is a small positive L change if positive-current diode duration grows, not automatically `+2td/τ`. Never infer physical HS/LS roles from generator names alone. End manual tests with `dc 0`; restore frequency/settings explicitly and use `mppt` only when returning to the intended operating mode. `cli.cpp:201–219,223–235,251–285,289–351`.

4. **Separate current zero and gain using the two converters.** Repeat off-state reads before/after and between low/high manual points, including immediately after a warmer run. Compare up/down order and zero interpolation against time, not merely row index. If both share a battery with usable BMS current, hold other loads steady and change only one converter at a time: the incremental BMS pack current tests that converter's physical low-side current; alternate boards to control background drift. Where BMS current is unavailable or too coarse, console data alone cannot establish absolute current gain. A residual proportional to `1/J` points to additive current error; a multiplicative shift stable across J points to gain. The existing host script supports `--bidir --lo ... --hi ... --dwell ...` for buck; its fit remains uncorrected and is not independent of the sensors.

5. **Test BF pump/history without treating precharge as calibration.** At night repeat cold-start and recently-warmed low-power boost sequences, logging actual reached duty, Vin/Vout, coil current and any trip. Use the proposed ramp only after a separate controlled hardware qualification; do not deploy it as part of this review. Hold a few already successful boost points long enough to see whether the inferred L changes as the gate pump settles, and repeat at 39/21.5 kHz. A pure series drop should raise L; a low-L change with pump behavior suggests sensing, auxiliary-current inclusion, or non-DCM waveform behavior. A successful `bf 1` readback is a logic state. The current console has no independent VGS/RDS(on) observation, so a clean no-effect result cannot prove a fully enhanced BF. The existing buck BF-open/closed host evidence already supports a diode-drop effect in that direction; repeating a high-current BF-open test is unnecessary.

6. **Temperature/core separation with no new instrument.** Alternate short low-power repeats after a long idle and after a repeatable normal charging interval; log available board/MOSFET/MCU temperatures as proxies and both boards as controls. Compare only common current bands and calibrated voltages. A reproducible history effect within the same direction raises temperature/pump/zero above topology as the explanation. Do not assign a Sendust temperature coefficient from these logs. After those nuisance effects are bounded, use joint multi-frequency fits to test the conditional slopes above. This can support or reject the vendor-law hypothesis; unknown turns and lack of time-resolved current remain the limits.

# SURVIVED

- **No confirmed arithmetic sign or boost-mapping error.** Ideal DCM inversion, `asBuck`, control-duty sign, and fixed-offset iteration survived derivation and independent analytic probes. Full-period duty and realized frequency are already used correctly.
- **The guard's large-offset/noise precedence works at adequate point count.** The pinned host suite passes its existing 64 offset/noise-tail cases. It does not cover the reproduced low-point escape. Do not discard the useful 400 ns refusal; make unevaluable states non-writable and add independent validation.
- **Zero handling and output provenance are substantially present.** Zero interpolation uses original sweep-row positions; >20 mA endpoint movement or an unread final zero vetoes persistence (`coil_fit.h:81–84`; `measure_coil.cpp:255–257,285–291`). Corrected and uncorrected L/bins are labeled separately; a numeric plausibility bound rejects NaN/infinite/out-of-range saved L, and flash writes wait until the converter is off (`coil_fit.h:340–342`; `measure_coil.cpp:431–435`). “Applied” versus “saved” still needs careful interpretation.
- **Concurrency checks are present for PWM timing changes.** Measurement startup rejects pending frequency/dead-time requests, and the console refuses those mutations during measurement (`measure_coil.cpp:443–448`; `cli.cpp:298–301,366–368`). I found no basis to allege that the logged frequency pair used the wrong timer denominator.
- **Validation performed:** the pinned C++ default host suite **PASS** (106.18 s wall time, single process); the initial deliberately reduced `4 4 4` ensemble failed statistical coverage thresholds, so it was not treated as a valid coverage result. Python/C++ parity: **parity: 110/110 datasets agree (L 0.1 %, f 0.2 %; L(Ipk) split on 99); accuracy over 40 48-step sweeps: median of points rms 0.8 % (worst 2.4 %), ring fit rms 0.11 % (worst 0.2 %); stated error: actual/stated rms 0.85 over 57 sweeps with zero, on-time (30-200 ns) and gain errors drawn within the stated bounds (no node charge); PASS**. Independent header probes and full-log parsing are saved in `review-probe-output.txt`, `runs.json`, and `core-prediction-output.txt`. No long simulator parameter sweep or hardware test was run. The synthetic simulator validates its own plant assumptions; it does not close the physical error budget.
- **Evidence limits remain explicit:** the vendor coefficients are supplied inputs; KS130 geometry is a local library value not independently checked here; real turns, coil temperature, actual gate waveforms, physical sensor gains, and switching-state zero are unmeasured. The 11-25 host runs share the converter metrology. `src/selftest/backflow.h` contains only a placeholder, not an alternative conducting-state validation path.
