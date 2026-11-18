*this document is an LLM generated placeholder*

# Independent review (Codex, xhigh), 2026-11-18

Final block of the review log, verbatim; findings verified and applied in ../2026-11-18-filter-chain-noise-study.md.

## ACCESS

- Live Influx ping: `204`.
- Playwright page title: `Median filter - Wikipedia`.

No repository files modified. New tests ran on this Mac; staged vup results were analysed separately.

**Recommendation:** retain `670a3ef9` and provisionally keep despike=8. Fix calibration retry scheduling. **Do not land `rev_k` yet.** The study supports rejecting this particular prototype, but does not establish that filtering is generally exhausted.

## FINDINGS

1. **High — (c) Persistent calibration failure still causes repeated sweeps. The new gate introduces a credible additional trigger.**

   At 506 samples/s, a 28 V bus with ±1.3 V, 100 Hz ripple **passes the old gate and fails the new one**, reporting `sd=0.7566 V > 0.7 V`. Notch and despike settings make no difference: calibration uses the raw transformed `last` values.

   Failure leaves `timeLastCalibration` unchanged, while periodic sweep scheduling uses that timestamp. The full simulator reproduced sweeps at **1800.0, 1809.6, 1819.1, … seconds**, with repeated Vout calibration failures and no protection trips. Thus the near-zero-current trigger was fixed, but the scheduling defect remains.

   This demonstrates reachability, not that either live converter currently experiences this ripple. Disabling despike would not fix this case. Separate retry timing from successful-calibration age.

   Sources: [sampling.h:551](src/adc/sampling.h:551), [sampling.h:650](src/adc/sampling.h:650), [mppt.cpp:97](src/mppt.cpp:97). [Reproduction log](/private/tmp/codex-review-1e-filter/sim-loop.txt).

2. **High, before deployment — (c) The `rev_k` port introduces an unguarded flash function into the RT path.**

   The port calls `expf()` every applicable tracker decision, without a flash-operation guard or corresponding IRAM placement. The current target map places `expf` at `0x420dc28c` and its implementation at `0x420dd7c0`, inside `.flash.text`.

   This creates a potential flash-operation stall in the control path. I did not reproduce a target-device stall; the port needs placement verification and timing validation before deployment.

   Sources: [port tracker.h:155](.claude/worktrees/agent-a1533e95825db54ec/src/tracker.h:155), [firmware map:103626](build/fugu-firmware.map:103626).

3. **Medium — (c) The dusk profiles reproduce their construction, but fry’s current polarity is wrong and the profiles contain plant motion.**

   Scope transmits `abs(raw)/3`; the builder restores a positive magnitude without recovering polarity or the sensor transform. With fry’s approximately **−0.545 A calibration offset**, its selected low-current captures run on the opposite branch: increasing scope current means decreasing calibrated current. Telemetry confirms this: the first capture fits approximately `I = 0.542 − 1.012 × scope_current`, correlation **−0.989**. The replay therefore reverses current’s relationship to Vin/Vout. RMS alone cannot reveal this error.

   `fry-dusk014` also contains substantial coherent plant motion: its 0.3–2 Hz current/Vin signals have approximately **0.92 absolute correlation at 350 ms lag**, and **0.98 coherence near 0.625 Hz**. This supports the contamination concern, though it does not independently prove which disturbance caused that motion.

   Exclude it from primary **sensor-floor estimation** unless that component is removed; retain it as a labelled stress case. Replaying recorded plant motion on top of a simulated tracker does not constitute independent sensor noise.

   Sources: [ina226.h:660](src/adc/ina226.h:660), [noise_profile.py:25](test/host-stub/noise_profile.py:25), [noise_profile.py:68](test/host-stub/noise_profile.py:68). [Correlation results](/private/tmp/codex-review-1e-filter/dither.txt).

4. **Medium — (c) `rev_k`’s covariance correction does not describe the actual filter.**

   The formula is correct for AR(1), but two-pass EWMA followed by period averaging is not AR(1). Even with white input, before adding median nonlinearity, my span-20 calculation at 450 Hz gives adjacent correlation **0.644**, versus the port’s **0.233**.

   Its inferred marginal sigma is approximately **0.56× actual at 450 Hz**, **0.60× at 506 Hz**, and **0.86× at 1470 Hz**. Consequently, k does not have the claimed consistent noise-relative meaning across sample rates. Empirical harvest gains would remain measurable, but the mathematical calibration of the threshold is wrong.

   Sources: [port tracker.h:70](.claude/worktrees/agent-a1533e95825db54ec/src/tracker.h:70), [port tracker.h:155](.claude/worktrees/agent-a1533e95825db54ec/src/tracker.h:155). [Calculation results](/private/tmp/codex-review-1e-filter/compare_chain.txt).

5. **Medium — (c) The study’s causal conclusions exceed what its comparisons establish.**

   The Vin regression assumes clock alignment from labelled sample rates and uses instantaneous linear/quadratic predictors. As a sensitivity test, aligning channel endpoints changes two inverter captures from **22.6% to 70.0%** and **69.7% to 97.6% explained variance**. Endpoint alignment is not recovered timing, but this instability prevents low R² from establishing “mostly not tracker dither.”

   The box experiment also changes the comparison: the offline equal-latency box is approximately **100–167 ms**, whereas the simulator prototype uses approximately **30 ms** in fast mode and removes most settling allowance. Correlated EWMA means do reduce adjacent-difference noise, but that mechanism was not isolated. Furthermore, `_lastPower` can remain held across multiple periods; the decision is not always an adjacent-period difference.

   Sources: [lowband_vin.py:14](doc/research/filter-chain/lowband_vin.py:14), [latency_study.py:29](doc/research/filter-chain/latency_study.py:29), [pwr_box.patch:127](doc/research/filter-chain/pwr_box.patch:127), [tracker.h:117](src/tracker.h:117).

6. **Medium — (c) Parallel result writes corrupt record association, and the comparison script accepts it silently.**

   Workers append multiline records concurrently. The parser associates every subsequent metric with the latest header and accepts whichever harvest field appears first.

   In `dusk-main.txt`, another run’s TRACKER line makes the parser assign **96.4%** to a baseline whose SUMMARY says **92.9%**. `fx-ds0.txt` also contains interleaved same-scenario records, leaving one parsed run empty. The aggregate then silently excludes missing harvests.

   **The document’s dusk harvest table nevertheless matches the reconstructed SUMMARY results.** This is a tooling defect, not evidence that those published harvest numbers are wrong. Write separate worker files and validate records before combining them.

   Sources: [trk-noise-sweep.sh:15](test/host-stub/trk-noise-sweep.sh:15), [trk-sweep-diff.py:10](test/host-stub/trk-sweep-diff.py:10), [interleaved evidence:24](/private/tmp/codex-review-1e-filter/dusk-main.txt:24).

7. **Lower severity — (a) Several quantitative statements need correction or consistent exclusions.**

   - **Despike +0.31 pp includes the +14 pp outlier.** Excluding that run gives **+0.153 pp**, not +0.31.
   - “Trip counts equal” is false: dusk totals are **292 baseline**, versus **292 / 294 / 291** for k=1.4/2.0/2.5. Equal totals also conceal changed individual runs.
   - The committed flat profile’s 30 ms mean RMS is **3.78 mA**, rather than 4.4 mA.
   - “At most ~10%” offline noise reduction misses `30w-hotplate-on-off`: the supplied script reproduces **0.69×**, a 31% reduction. That does not establish a harvest gain.

   Sources: [study:65](doc/research/2026-11-18-filter-chain-noise-study.md:65), [study:109](doc/research/2026-11-18-filter-chain-noise-study.md:109), [study:128](doc/research/2026-11-18-filter-chain-noise-study.md:128), [study:144](doc/research/2026-11-18-filter-chain-noise-study.md:144).

Recomputed harvest deltas, **mean / worst, percentage points**, from the staged SUMMARY fields:

| k | All dusk profiles | Excluding fry-dusk014 | Old profiles |
|---|---:|---:|---:|
| 1.4 | −0.56 / −5.40 | −0.50 / −5.00 | +0.26 / −0.60 |
| 2.0 | −0.47 / −8.70 | +0.01 / −2.20 | +0.25 / −1.80 |
| 2.5 | −0.21 / −7.80 | +0.10 / −1.00 | +0.11 / −5.20 |

Excluding `fry-dusk014` therefore changes the mean-based conclusion. However, three offsets of one recording at one gain do not establish a reliable improvement. The new dusk baseline itself varies by **up to 3.5 pp across offsets**, exceeding the older quoted 1.0 pp maximum. **Withholding `rev_k` remains justified; declaring that no k can outperform the fixed rule does not.** [Full tables](/private/tmp/codex-review-1e-filter/tables.txt).

## SURVIVED

- **The calibration estimator is mathematically appropriate for independent white samples.** Its squared value estimates variance; for correlated samples it measures `σ²(1−ρ₁)`. Thresholds now have sensor units: Vin/Vout in volts, currents in amperes, and **NTC in ADC volts**, despite its `'C'` metadata. The production-path host test passed **32/32**; the same harness with the old sampling header passed only **8/28**.

- **The suspected calibration bookkeeping problems did not reproduce.** Notch/despike cannot alter this raw-sample gate. Reset plus the 100-sample warm-up removes the sweep-boundary/initial-NaN concern. Normal calibration paths finish well before the uint16 counter cap. [sampling.h:546](src/adc/sampling.h:546).

- **Despike does not bypass direct OV/OC protection.** Raw `last` and the independently maintained median remain available to those checks. EWMA-dependent protection and regulation can change. Scale-zero warm-up passes the first deviation, and a clipped extreme outlier raises the next threshold: k=8 is neither an eight-sigma guarantee nor a robust MAD estimator. [sampling.h:137](src/adc/sampling.h:137), [mppt.h:710](src/mppt.h:710).

- **RT logging is deferred**, including float formatting. Warnings are limited by 2048 sampler updates, not elapsed seconds. A host benchmark measured roughly 8.0–8.2 ns/sample without despike versus 8.5–9.2 with it; **ESP32 timing remains unverified**. [logging.cpp:287](src/logging.cpp:287).

- **Profile construction and absolute residual magnitudes reproduce:** all three `.fnz` files rebuilt byte-for-byte; RMS values are **8.07, 10.90, 10.97 mA**. `SIM_NOISE_EXP=0` correctly makes current-noise amplitude independent of simulated current, including converter-off periods. This validates implementation, not a universal physical noise floor.

- **(b) Filtering power instead of filtering I and V separately happens to be a reasonable noise approximation in the six checked captures:** period-noise differences were about **0–1.6%**. It is not exact, particularly for bias. The staged box prototype’s **−0.257 pp** overall harvest result also reproduces.
