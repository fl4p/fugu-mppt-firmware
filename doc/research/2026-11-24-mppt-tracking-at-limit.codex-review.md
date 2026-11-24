*this document is an LLM generated placeholder*
# ACCESS

All three required sources were reached during this review on 2026-11-24. No token appeared in a tool response. Credentials were loaded without printing them; no account-state changes or device commands were issued. Fab's browser was not used.

1. **Live InfluxDB**, database `ha_van`, query `SELECT last(P) FROM mppt WHERE device='flat'`:

   ```json
   {"time":"2026-11-24T14:25:56.584Z","last":92.2518}
   ```

2. **Live SSH**, `ssh havan-ts 'grep -ac "scan near:" ~/pv/fugu_console.log; tail -1 ~/pv/fugu_console.log'`, first and last output lines:

   ```text
   2
   2026-11-24 14:25:56,798      fry: V=65.2/26.95 I=1.05/ 2.47A  68.6W 27℃37℃ 512sps  0㎅/s DCM(H|L|Lm)=1696|2274|2274 st=⇣MPPT,1 lag=3315㎲ N=376585 rssi=-46
   ```

   The count 2 is for the current file alone, not its rotations.

3. **Playwright**, requested URL `https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/`, live DOM title:

   > ESP-IDF Programming Guide - ESP32-S3 - — ESP-IDF Programming Guide v6.1.1 documentation

   The DOM also contained the ESP-IDF Programming Guide heading and 34 links; this was the requested content, with no challenge.

# Review result and provenance

**The decision to stop tracker work is not established by this note.** The evidence supports small losses for several examined mechanisms and useful local peak checks at moderate power. It does not establish that flat's bright-condition offset is a sweep artefact, or a general ≤0.5% ceiling on further tracking gains. This is a failure of the conclusion's validation, not proof that 22 Wh/day is recoverable or that a firmware change should be deployed.

The reviewed note is commit `eecdb90f8c57960db01b03b2c9e05dc9af492e25`. Code and committed study scripts were read from `origin/main`, pinned at `3d02495c6b43a746d81a7943f3e3cdf0514eac0f`. All `src/...` line citations below refer to that pinned revision. The main checkout's uncommitted files were not used as code evidence, and no file in that repository was modified. Scratch artifacts are confined to this review's cwd.

Primary event evidence was fetched with `grep -aHE` over the live console log and rotations `.1` through `.10`, selecting near probes, scan summaries, sweeps, backoffs and blocked starts. The captured events span 2026-11-13 through 2026-11-24 14:13:51. Near-probe records occur only on November 24: 30 flat and 31 fry completed pairs in this snapshot. There is no additional day of near-probe evidence in those rotations.

Every near line was paired with the next same-device `scan: P` line within two seconds, preserving both exact source lines in [near.csv](evidence/data/near.csv). The ≥100 W rule uses `max(p0,p1)`. This reproduces the table exactly:

| Device | Scans | Both <1 | +2 V >1 | +2 V >1.01 | −2 V >1.01 | Median + / − |
|---|---:|---:|---:|---:|---:|---:|
| flat | 14 | 11 | 3 | 1 | 0 | 0.986 / 0.9615 |
| fry | 19 | 12 | 1 | 0 | 2 | 0.972 / 0.993 |

Fresh raw Influx queries were made for all 33 qualifying scans, surrounding dither windows, the 11-20 afternoon peak-swap episode, and selected complete-day aggregates. Exact credential-free queries accompany each JSON response in [evidence/data](evidence/data). The 30-day analysis was recalculated with the pinned scripts against the earlier cached bins/curves, whose hashes are recorded; it was not represented as a new full 30-day raw-data download. [sha256.json](evidence/sha256.json) fingerprints these inputs and the analysis code.

# Quantitative detection check

The ordering is **−2 V, +2 V, first reference p0, distant probes, final reference p1** (`src/peak_scan.h:227-291`). The log divides both near powers by `max(p0,p1)`. Thus the references bracket the distant probes but **do not bracket the near probes**. Rising irradiance can suppress both near ratios; falling irradiance can inflate them. Taking a maximum also biases a noisy denominator upward. The 15% endpoint-change test controls movement to a distant peak, not the validity of near ratios, and it admits all the ≥100 W examples here.

The observed absolute reference change `|p1-p0|/p0` is:

| Device, ≥100 W | Median | p90 | p95 | Maximum |
|---|---:|---:|---:|---:|
| flat, n=14 | 0.528% | 1.472% | 3.506% | 7.214% |
| fry, n=19 | 0.669% | 4.029% | 4.212% | 4.718% |

These are empirical reference variations, containing light changes and measurement effects. They are **not** calibrated independent measurement-error confidence bounds; two later references cannot establish the irradiance at the earlier probes.

For an explicit sensitivity calculation, take a local quadratic `P(V)/Pm = 1 − k(V−Vm)^2`, an operating point `x` volts below Vm, and a 2 V positive step. Its true relative gain is

`g = 4k(x−1)/(1−kx²)`.

Even with perfect measurements, a 2 V step overshoots enough that offsets below 1 V need not yield a positive gain. With an unfavorable multiplicative reference error `q`, merely exceeding a logged ratio of 1 requires `g > q`; exceeding 1.01 requires `g > 1.01(1+q)−1`.

The fresh flat trace at 10:43:19.834–10:43:28.834 has 441 raw samples, a 5th–95th percentile voltage span of 2.251 V, and a local fit `k ≈ 0.00218 V⁻²` after fitting quadratic time drift simultaneously with quadratic voltage dependence. Residual scatter is 0.54% of power. This is a short, model-dependent fit, **not an independently validated curvature calibration over a 5 V range**. Low-power flat windows have much steeper fitted curvature, around 0.012 V⁻²; the measurement sensitivity is not constant with power. An attempted older bright-data fit did not meet the declared 1.5 V excitation requirement and was rejected, rather than used to claim a verified threshold.

Using the illustrative rounded broad-peak value `k=0.0022 V⁻²` gives:

| Offset below Vmpp | Actual power deficit in this model | True +2 V gain |
|---|---:|---:|
| 1.5 V | 0.50% | 0.44% |
| 3 V | 1.98% | 1.80% |
| 5 V | 5.50% | 3.72% |

At the observed flat p90 reference variation of 1.472%, the threshold for a ratio above 1 is **2.65 V / 1.54% power deficit** in this model. At the observed p95 variation of 3.506%, it becomes **4.78 V / 5.03% power deficit**. The 1.01 criterion is stricter. These are conditional sensitivity calculations, not a claim of 90% or 95% detection probability. The data do not establish a reliable, universal exclusion threshold for a 1.5–5 V offset.

A concrete failure case uses the measured 7.214% reference rise and a 5 V offset on that broad-peak model: the logged ratios would be **0.881 at −2 V and 0.967 at +2 V**, despite a 5.5% power deficit. Both sides below 1 therefore cannot certify a peak.

# Settling, steering, and other loss checks

The scanner continually steers duty toward `D*sqrt(Vin_fast/Vtarget)`, with per-tick slew limits (`src/peak_scan.h:231-239`, `src/mppt.cpp:363-369`). It starts the settle clock within 1% of target, or after a 1.5 s travel timeout, then waits for the power-filter settling estimate plus a 100 ms mean. `src/mppt.h:394-402` defines this as the estimated 99% settling of the physical power sensors, not an explicit capacitor-energy test. A travel timeout could still yield an invalid diagnostic in principle; none was demonstrated in these near-probe traces.

All 66 target plateaus from the 33 qualifying scans were found in raw telemetry. The final 100 ms estimates were within **0.058 V** of the rounded nominal targets. Flat plateaus lasted 0.189–0.550 s; fry 0.188–0.232 s. Assuming the documented as-built input capacitance of 0.94 mF (`doc/research/2026-11-20-hs-short-bflow-spice.md:153`), estimated residual `η C V dV/dt` magnitudes in those final windows were at most **0.107% of P for flat and 0.031% for fry**. Telemetry-window alignment and the component value remain assumptions; this is not an instrumented firmware measurement of the exact accumulator window.

For scale, a complete 2 V transition at 60 V moves approximately 0.113 J in a 0.94 mF capacitor, or about 1.1 W output-referred if all of it contaminated a 100 ms mean. The −2 V to +2 V transition moves about twice that energy. Those full-transition figures must not be substituted for the measured, substantially settled plateaus. **A large capacitor-settling error was not found in the inspected bright probes.**

Other checks:

- Fresh Influx integration reproduces **8.280 Wh** for fry 11-20 15:44:06–15:49:17 and **4.596 Wh** for 16:14:19–16:19:29. Multiplying by the earlier report's proposed alternative-peak gain ranges yields 0.132–0.745 Wh and 0.138–0.276 Wh. The additional two windows give about 8.90 and 5.33 Wh; the original fourth-row 5.7 Wh is consistent with including the extra 19 seconds before 16:14:19. The aggregate rounds to approximately **0.3–1.8 Wh**. The energy is measured; the competing-peak gain envelope remains an estimate.
- The 11-20 high-V flank interpretation is broadly supported: fresh fits find peaks reaching about **74.7 V**, and the accepted-window mean operating voltage is about **0.3 V below** the fitted optimum. Exact counts depend on filters and endpoint conventions; I did not independently reproduce the stated **42 / 0.23 V** combination. This supports rejecting the earlier inference that 73–75 V was automatically the wrong flank, not a universal zero-loss claim.
- The low-light script explicitly examines sweep and scan state, with a 20 s landing allowance. Re-summing its saved accepted intervals gives **3.11 Wh/day fry and 3.51 Wh/day flat** over 11-14..23. Re-running that method against fresh 1 s data on 11-20 gives **4.08 and 7.64 Wh**, respectively. These are deficits against an interpolated baseline, with negative deficits clipped; they include possible weather/landing errors and are not measured recoverable savings. The averages are not a per-day upper bound.
- Shared-bus idle time was **not overlooked**: `offtime.py:25-43` identifies own P<0.3 W while the other converter is producing and estimates availability from an own/other power ratio. Lines 54 onward classify recent high output voltage and limiting as charger-related. This is a proxy, not proof that the pack could absorb all estimated missing energy.
- Battery/output-limited bins are explicitly excluded from the 30-day comparison (`analyze.py:37-40,86`). In live code, an active limiter can prevent or abort a scan (`src/mppt.cpp:360-378`). The review therefore does not close losses during or around those interactions. Absolute conversion efficiency also cannot be determined from virtual Iin: `src/adc/sensor_setup.h:15` sets 0.97, while `src/mppt.h:363-370,410-413` uses physical output current and voltage for P. Local output-power comparisons are still meaningful.

# FINDINGS

1. **[c — wrong conclusion] “Both ratios <1” does not establish “at peak,” and the note has no validated exclusion threshold.**

   Evidence: note lines 21, 25–27 and 34; `src/peak_scan.h:251-292`; the sensitivity calculation above. In the exact live log pair:

   ```text
   2026-11-24 11:15:19,336     flat: I (2878230) mppt: scan near: v0 55.1 V, -2.0V x0.873, +2.0V x0.936
   2026-11-24 11:15:19,486     flat: I (2878230) mppt: scan: P 291.1->312.1 W at 55.1V, 1 probes, best x0.756 at 43.1V
   ```

   The positive probe is `0.936*312.1/291.1 = 1.00352` relative to the first reference, yet reads 0.936 against the later one. Raw telemetry confirms rising power at essentially fixed reference voltage. This demonstrates denominator suppression; it does not by itself locate the true Vmpp. On a broad peak consistent with an observed local fit, 1.5 V produces only a 0.44% signal and roughly 5 V can remain invisible during the observed rise.

   **The note should say:** “Most sampled near probes did not demonstrate a gain under this conservative, temporally unbracketed measurement. This does not establish that the operating point is at Vmpp or exclude the proposed offset.” A trustworthy exclusion test needs actual probe voltages and timestamps, bracketing/repeated references, order reversal, and a demonstrated detection threshold. No firmware defect is established merely by this log-only diagnostic being unsuitable for the inference.

2. **[c — wrong conclusion] The sample barely tests the bright flat regime carrying the alleged loss, and scan eligibility creates a blind spot.**

   Evidence: all 14 qualifying flat pairs have median reference power only **127.25 W**. None starts at ≥300 W. Only the rising 291.1→312.1 W example reaches that band at the final reference. In the study, the >300 W band supplies **975.53 / 1328.18 = 73.45%** of the analysed flat energy per represented day. Even on November 24 before 14:25, live data show **286.43 Wh and 2218 seconds above 300 W**—there was substantial bright operation outside this near-probe sample.

   `src/peak_scan.h:195-225` requires learned substrings and a reference, then constructs distant candidates. If pruning leaves no distant candidate, it returns **before the near probes**. Strong full-string operation can therefore omit the diagnostic precisely because there is no plausible alternative substring peak, even though a local offset would still matter. Limiters, sweeps, and PSU/full-pack states exclude further intervals. The trigger path requires five seconds of steadiness after a sustained change, but **periodic/forced scans do not share that steadiness requirement**; it would be incorrect to describe all scans as steady-light samples.

   **The note should say:** “This single-day sample supports moderate-power behavior. It does not test bright flat operation sufficiently to dismiss its historical offset.” A representative local test must be able to run when the distant-candidate list is empty and must cover the energy-dominant bands.

3. **[c — wrong conclusion] The sweep-artefact explanation is plausible in magnitude but not demonstrated, and its cited calibration does not establish the required voltage correction.**

   Evidence: note lines 7–10 and 41–42; the 30-day study lines 219–240; `curves.py:33-38`, `analyze.py:108-117`; [bands.py](evidence/prior-scripts/bands.py), lines 9–18.

   The “~2%” observation is actually a **+0.02159 ratio discrepancy** at 42 far probes, all **9.8–16 V below** the operating point: predicted mean 0.82292 versus measured 0.80133. It is not a direct measurement that Pmax, near-peak shape, or Vmpp is 2% too high. Recomputing the same comparisons, translating the curves **+2 V** reduces the mean discrepancy to +0.00232; translating them **−2 V** increases it to +0.05828. Thus the calibration does not support moving an allegedly excessive sweep Vmpp downward toward flat's lower operating voltage. It could reflect shape error, dynamics, timing, or a different voltage-dependent effect.

   The band script uses all available ages for the band energy and `age<=10 min` for its efficiency, without the main report's 1-minute lower bound or explicit sweep exclusion. That difference does **not** explain away the result: excluding sweep bins leaves **21.40 Wh/day**; requiring age 1–10 minutes and excluding sweeps gives **22.07 Wh/day**.

   A uniform multiplicative sweep-power error cancels in `s(V)=P(V)/Pmax`; a voltage-dependent distortion does not. The per-band result recalculates to **η=0.9779746, 975.53 Wh/day harvested, 21.97 Wh/day inferred loss**. If the inferred available power itself were exactly 2% too high, a hypothetical correction would leave **2.41 Wh/day** in this band. A suitable **2.252%** bias could erase the whole estimate. That is good magnitude agreement, but no such near-peak correction has been identified and validated. The residual 2.41 is a sensitivity example, not established recoverable energy.

   **The note should say:** “The earlier headroom estimate is uncertain and could largely be a sweep-shape artefact. The available near/far checks do not yet separate that artefact from real tracking loss.” Neither the original 22 Wh/day estimate nor its dismissal should be presented as settled.

4. **[a — counting/threshold error] The prose changes the threshold, and the peak-scan coverage label loses a day.**

   Evidence: note line 34 says +2 V exceeded 1 once. It exceeded 1 **three** times: **08:49:11 x1.009, 10:04:37 x1.002, 10:43:18 x1.024**. Only the last exceeds 1.01. The table is correct; the inference sentence is not. Note line 18 also labels the earlier 121-scan sample “11-20/21.” The >50 W re-count is **11 on 11-19, 60 on 11-20, 50 on 11-21**, totaling 121. The two named days alone contain 110 qualifying scans; there are 111 without the >50 W restriction.

   **The note should say:** “Three positive-side ratios exceeded 1; one exceeded 1.01,” and “121 scans above 50 W on 11-19..21.” These corrections do not solve the diagnostic's larger validity problem.

# SURVIVED

- **The near-probe table's counts, strict thresholds, and rounded medians are correct.** No numerical revision to that table is needed for the original snapshot.
- **[b — unstated assumption supported in these samples] The ±2 V commands reached their intended voltages and substantially settled.** The raw-data checks do not support blaming these results on a large duty-steering or capacitor-discharge error.
- **The small 11-20 missed-peak estimate survives as a bounded estimate for its examined windows.** Fresh measured energies support its arithmetic; the assumed alternative-peak gains should remain labeled estimates.
- **The high-V episode does not establish a tracking fault.** Fresh fits support a moving high-voltage optimum and a small average local offset, with the exact 42-window/0.23 V summary not independently reproduced.
- **Sweep/scan costs and shared-bus idle episodes were investigated.** They cannot be claimed as wholly unexamined omissions. Their proxy estimates do not close the complete energy budget or show that every deficit is recoverable.
- **The fixed-voltage comparison survives with its original uncertainty:** approximately +1.5±1% for fry and parity within about ±1.5% for flat in this autumn dataset. The note's rounded summary is consistent with that study, but does not independently establish absolute tracking efficiency.
- **The conversion-efficiency caveat and separate treatment of full-pack protection events are appropriate.** Neither the virtual input current nor an other-converter proxy establishes available battery-absorbable energy.
- **A sweep artefact large enough to explain much of the 2.5% headroom remains plausible.** The claim that it has already been proven—and therefore that further tracker work should stop—does not survive.
