*this document is an LLM generated placeholder*

# Review: issue #37, 100/120 Hz rejection

## ACCESS

Requested first command: `git -C /Users/fab/dev/pv/fugu-mppt-firmware/.claude/worktrees/agent-abf531ce698932f59 log --stat a00e9093..4ba10efb` — completed.

`git rev-parse 4ba10efb`:

```text
4ba10efba6dabf570f9b6e7ba9ce2c863c7f602f
```

Requested `curl -s https://api.github.com/repos/fl4p/fugu-mppt-firmware/issues/37 | head -c 300` returned:

```text
{
  "url": "https://api.github.com/repos/fl4p/fugu-mppt-firmware/issues/37",
  "repository_url": "https://api.github.com/repos/fl4p/fugu-mppt-firmware",
  "labels_url": "https://api.github.com/repos/fl4p/fugu-mppt-firmware/issues/37/labels{/name}",
  "comments_url": "https://api.github.com/repos/fl4
```

The complete issue was also read: “100 / 120 Hz rejection”; it asks about choosing averaging/conversion time for inverter rejection. Scratch Playwright fetched [TI INA226](https://www.ti.com/product/INA226), title `INA226 data sheet, product information and support | TI.com`. No access check was blocked. No fabchrome, tokens, device, physical serial port, or MQTT broker was used.

Scope: the four requested commits, compared with a00e9093. CLAUDE.md was read, including RT/task, IRAM, conf-key and documentation requirements. Worktree HEAD was the requested commit. Its pre-existing dirty files were only `.claude/agent-memory/ee-code-verifier/MEMORY.md` and `.claude/agent-memory/ee-code-verifier/project_ripple_comb_vs_notch.md`; they were not touched or used as review evidence. No reviewed-repository file was modified. Git archives, builds, numerical experiments, and logs are under `/tmp/codex-review-fugu37`.

Severity: P2 = actionable functional defect; P3 = lower-priority documentation or contract issue. Tags: (a) arithmetic claim/error, (b) assumption/scope qualification, (c) design defect. There is no demonstrated new P0/P1 defect. The host default is unchanged; the principal functional finding affects opt-in comb/both.

## FINDINGS

### 1. P2 (c): the controller's delay and settling estimates omit the comb

Changed location: `src/adc/sampling.h:150-153`. A comb is inserted ahead of median/EWM, but `src/mppt.h:385-402` still computes delay and settling from only median-5 plus two-pass EWM. `sweepFiltDelay()` supplies the sweep speed bound (`src/mppt.cpp:254-255`); `powerSettleMs()` supplies tracker reversal suppression (`src/tracker.h:124`) and admission of peak-scan measurements (`src/peak_scan.h:234-235`). The omission therefore reaches actual control decisions, not just telemetry.

Reproduction uses the actual RippleComb, RunningMedian5, EWM and NotchFilter implementations, with settled zero history and a unit step. `filt_len=3` is accepted without clamping by `src/adc/sensor_setup.cpp:230` and occurs in existing configurations.

| fs | EWM span | Reported settling | Notch output at deadline | Comb output at deadline | Both output at deadline | Comb reaches 99% |
|---:|---:|---:|---:|---:|---:|---:|
| 450 | 3 | 26.578 ms | 0.99176 | 0.96962 | 0.96929 | 28.889 ms |
| 1470 | 3 | 8.136 ms | 0.99878 | 0.51087 | 0.50024 | 14.966 ms |
| 1470 | 20 | 46.531 ms | 0.98996 | 0.98181 | 0.98165 | 51.020 ms |

The comb also adds exactly 5 ms of DC mean delay at 100 Hz. With span 20 at 1470 sps, the sweep calculation still uses 14.286 ms instead of 19.286 ms. Whether the resulting pace error takes effect depends on the existing clamps; the missing delay itself does not.

Account for the active comb in both timing models, including its retuned period. This reproduction establishes incorrect timing estimates and premature eligibility for measurements; it does not establish a hardware instability or prove that every tracking decision hits the bad window. Evidence: `settling.cpp`, `settling.log`.

### 2. P3 (a): fractional-comb descriptions promise exact properties that the taps do not have

Locations: `src/math/ripple_comb.h:5-10`, `website/docs/internals/signal-filters.md:98-109`, and the `ripple_filter` help in `etc/config-tool/conf-editor.html:775`.

- The fractional FIR has **finite rejection**, not an exact zero at f or every sub-Nyquist harmonic. At fs=450, f=100, the stated taps give -46.5015 dB at f and -28.2859 dB at 2f. The useful finite attenuation table is substantially correct; the word “zero” is not.
- L/2 is the **DC mean delay**, not constant group delay. With L=4.5, delay is 2.25 samples at DC but 2.62889 samples at 90 Hz. Integer-L taps are symmetric; fractional taps generally are not.
- Preserving history prevents a reset transient, but does not make every retune step-free. With the same populated ring and next input, tuning L=4.5 to L=5 changes the next output from 0.0277778 to 0.1000000 in the pulse-history reproduction. This corresponds to 100 to 90 Hz at 450 sps, within a permitted half-step toward an 80 Hz estimate. No indexing error is involved.
- The implemented bound is `2 <= L < 23` for CAP=24, not L<=22. L=22.99999 fits safely. For adaptive 80–140 Hz operation the admissible rate interval is `[280,1840)` sps. This is a documentation mismatch, not an out-of-bounds defect.
- Full step completion is at sample index ceil(L), measured from the first changed sample at index zero; it can exceed one physical period by less than one sample. At 450 sps, L=4.5 completes at 11.111 ms rather than 10 ms.

Related numerical overstatement: `signal-filters.md:73-74,118` says the notch's off-centre response is independent of rate/depth. At 101 Hz, Q=20, depth=-15 dB, the exact biquad gives -5.737 dB at 450 sps and -7.663 dB at 1470 sps. Depth=0 gives an all-pass response. Qualify that row as an approximate result at a specified sample rate/depth.

Use “deep attenuation,” “DC/low-frequency delay,” “history-preserving retune,” and the actual bounds. DC unity remains mathematically correct for the FIR itself; it does not guarantee an unbiased mean after a nonlinear median. The documented spike-smearing tradeoff is real. Evidence: `math_check.py`, `math-results.txt`, `comb_audit.cpp`, `comb-audit.log`.

### 3. P3 (b): “no heap on RT” holds for the steady loop, not for all work performed by its task

Location: `src/adc/sampling.h:216`.

`configureNotch()` allocates combs during CPU0 setup (`src/adc/sensor_setup.cpp:347`). An out-of-range channel's comb is then deleted by `createNotchFilter()` from `ADC_Sampler::begin()` (`src/adc/sampling.h:531`), which runs inside `loopRT` after `rtAllocBegin()` (`src/main.cpp:754-759`). Thus the fallback adds a real heap free on core 1. A 2000 sps channel in adaptive mode is one concrete trigger: 2000/80=25 does not fit.

This occurs during the RT task's initialization, before its steady sampling/control loop; no recurring heap activity or startup deadline failure was demonstrated. The distinction matters because the simulator counts only `loopRTIter()` (`test/host-stub/mppt-sim.cpp:501-503`), while its call to `adcSampler.begin()` is outside that interval at line 693. Zero iteration allocations do not prove zero RT-task heap operations.

If the rule is literally no heap anywhere on core 1, relocate/defer the free. If startup is the intended exception, state that exception rather than using the iteration counter as broader proof. This is a scope/contract finding, not a demonstrated live-converter timing failure.

### 4. P3 (c): documentation incorrectly implies every protection path bypasses the comb

Location: `website/docs/internals/signal-filters.md:100-101` (“Protection reads the raw sample”).

The Vout OV trip does use `last` (`src/mppt.h:763`) and is unaffected by the comb's delay. However, Iout OC includes median and EWM thresholds at 1.25x and 1.15x the limit (`src/mppt.h:815-817`), alongside the raw 1.5x trip. Reverse-average-current protection also reads EWM (`src/mppt.h:844`); short-current/low-output-voltage protection reads EWM at line 895. The new FIR therefore changes some protection timing and response to smeared spikes.

Narrow the assurance to raw Vout OV and other specifically identified raw branches. No failure of the raw emergency threshold was found; no claim of unchanged latency for all protection branches is justified.

## SURVIVED

### Arithmetic and storage

The tap derivation is correct for integration of a piecewise-linear interpolant over the last L sample intervals. Before writing x[t], `at(n_)` is x[t-N-1]. The running sum update removes it and adds x[t], leaving x[t] through x[t-N]. The correction coefficients create exactly the documented endpoint weights. Their sum is L, and their first moment divided by L is L/2.

Independent double-precision transfer-function evaluation:

| fs | L for 100 Hz | At 100 Hz | At 99 / 101 Hz | At 200 Hz | At 300 Hz, including aliasing |
|---:|---:|---:|---:|---:|---:|
| 450 | 4.5 | -46.50 dB | -40.24 / -40.29 dB | -28.29 dB | -16.81 dB |
| 513 | 5.13 | -52.90 dB | -43.44 / -39.27 dB | -38.83 dB | -29.23 dB |
| 1470 | 14.7 | -80.63 dB | -39.97 / -40.30 dB | -67.21 dB | -58.54 dB |

The actual float test gives about -78.5 dB at 1470 sps rather than the ideal -80.6 dB; this is consistent with the documented approximate -78 dB. The claimed -46 / -40 dB at 450 sps survives.

An independent C++ oracle checked **2,000,000 samples**, including wraparound, repeated changes of floor(L), pass-through and re-enable, invalid lengths, and seed() at every ring position. Maximum error versus a direct double FIR was **63.6 microvolts at roughly 80 V**; arbitrary-position seeded DC error was **7.63 microvolts**. ASan/UBSan reported no fault. The existing test also passed its 20-million-sample drift test (39 microvolts). Re-summing at every 24-sample wrap bounds accumulation in time; it is not a universal voltage-independent absolute-error bound.

NaN/infinite L do not fit. Maximum valid floor(L)=22 requires 24 taps, within the ring. Seeding after reset is correct. All existing ripple-frequency/comb/notch-depth tests passed (`ripple-test.log`). The gainAmp repair correctly makes requested dB equal centre attenuation.

### RT path and consumers

No new allocation, deallocation, libm call, logging, or race was found in comb filter()/tune(). Retuning and filtering both run synchronously in ADC_Sampler::update on the RT core; configuration/source selection occur before it starts. Comb allocation is 116 bytes per selected physical channel on the target. Failed-rate fallback deletion is the startup exception described above.

A same-integer tune updates coefficients; a change of floor(L) re-sums at most 23 floats. The sampler retunes one sensor per update. The per-sample path re-sums once per ring wrap, so its worst work is bounded but not uniform. Host microbenchmarks measured roughly 3.7 ns/sample without retunes, 4.5 ns with a same-floor retune each sample, and 8.5 ns with worst-range floor changes each sample. These measure host code only; **ESP32 wall-clock retune cost and flash-operation latency remain unverified** because no device was used.

Fresh Xtensa main.cpp and sensor_setup.cpp objects compiled with the repository's recorded ESP32-S3 flags. Comb methods inline into Sensor::add_sample and PhysicalSensor::retuneNotch. Their fresh text and literal relocations match the existing linked objects. That map places them at IRAM addresses 0x40392478 and 0x4039147c; the division helper __divsf3 resolves to ROM at 0x40002274. No new standalone flash-resident comb helper was emitted. The existing full build identifies itself as 93cddf27-dirty: this review verified fresh objects and their matching mapped functions, **not a fresh full production-firmware link**. expf is only reached during configuration, outside the steady RT path.

Integration order is notch -> comb -> median -> EWM. `last` and `previous` are assigned before either filter; the ripple detector also consumes pre-filter `last`. Vout OV stays raw. Zero-offset calibration accumulates raw `last` and reseeds filters on reset. Filtered consumers include voltage/current limiters, tracker and sweep power, energy metering/telemetry, rectifier calibration, self-tests, and the filtered protection branches identified above. Apart from the missing controller timing model, no new broken consumer was established.

Only physical sensors receive combs. Virtual current sensors continue deriving from raw physical `last` values and then applying their existing median/EWM; they do not automatically gain inverter rejection. This matches the physical-only setup and is relevant when interpreting current limiters and telemetry. Fallback correctly enables the notch for a non-fitting channel **if** its frequency is below the existing 0.45*fs limit; a sufficiently slow channel has neither filter, as before.

### Default behavior and configuration

With no new keys: `rippleMode=1`, no comb allocation, and depth=-15 dB. Unknown mode strings warn and fall back to notch. Positive or NaN depth warns and falls back to -15 dB; the warning describes NaN imperfectly but the fallback is safe. Those paths were exercised through actual configuration parsing and gave the same summary as explicit -15 dB. Negative infinity is accepted as <=0 and maps to zero centre amplitude; it does not amplify.

The ESP32-S3 target gain was executed in **isolated QEMU**, using ESP-IDF 6.0.3, GCC 15.2 and picolibc:

```text
GAIN_AUDIT old_const=3e361887 old_runtime=3e361887 new_runtime=3e361887
GAIN_AUDIT 0.177827939391 0.177827939391 0.177827939391
```

Thus old constant-folded sqrt(pow), old runtime sqrt(pow), and new runtime expf agree for the default. The remaining default notch coefficient/filter path is unchanged. The full host output of the 900-second default china-inv cap run is **byte-identical between a00e9093 and 4ba10efb**, including startup logs and all traces/statistics. This verifies the tested default behavior, not universal equivalence for every exceptional input or compiler configuration.

Both new keys appear in parser, sensor.md, and editor META/FILE_KEYS/default metadata. No missing three-place update was found. Default remains notch; no supplied production sensor configuration opts into comb/both.

### Simulation reproduction and limits

Both revisions were built from `git archive` copies using their own `test/host-stub/mppt-sim.sh sanity`; both passed. Noise profiles from HEAD were supplied to both binaries by absolute path. The matrix used at most four simulation workers; the additional matrix used two. All 18 matrix processes exited zero and reported zero allocations/frees inside the measured RT iteration.

Representative exact commands, run from the head archive:

```sh
OUT=/tmp/codex-review-fugu37/mppt-head bash test/host-stub/mppt-sim.sh sanity
SIM_NOISE=/tmp/codex-review-fugu37/head/test/host-stub/noise/china-inv.fnz \
  SIM_NOISE_EXP=0 SIM_CONF=sensor.conf:ripple_filter=both \
  /tmp/codex-review-fugu37/mppt-head wx-cloud-cv
```

900-second replay results (percent of cap-limited available energy; no-cap row has the same denominator as GMPP):

| Replay / scenario | Notch, -15 dB | Comb | Both |
|---|---:|---:|---:|
| china-inv / wx-cloud-cv | 80.0% | 96.3% | 96.6% |
| pv8a-inv / wx-cloud-cv | 37.5% | 94.0% | 94.1% |
| china-inv / wx-cloud | 96.3% | 95.2% | 95.7% |

The -30 dB china-inv cap run fell to **8.4%**, reproducing the severe deeper-notch regression in this scenario. Strong cap benefit, the disadvantage with uncapped spiky current, and the nonlinear-median explanation survive. The cap replay rows all have one common calibration-failure trip at startup; no-cap rows have two. They are included in energy totals, not silently excluded.

Independent pure-sine bus-ripple runs, fixed filter at 100 Hz, fs=450, amplitude 0.26 V, wx-cloud-cv, 900 s:

| Actual ripple | Notch | Comb |
|---|---:|---:|
| 103 Hz | 70.4% | 99.6% |
| 120 Hz | 0.1% | 96.5% |

These support robust comb benefit away from its tuning frequency, but **do not reproduce the exact quoted 98%/16% 120-Hz cell** or the exact 8–17% loss range. The prose does not supply that cell's full conditions. Those exact percentages remain unverified; neither contradiction nor arithmetic error is inferred from a differently parameterized cell. The added drift setter preserves oscillator phase; a new slowly drifting-frequency matrix was not run.

All five new `.fnz` files regenerated **byte-for-byte** from local source captures using inverter_profiles.py plus noise_profile.py (`profile-audit.log`). This establishes reproducibility of conversion, not correctness of its physical assumptions:

- The generator assumes a 100 Hz fundamental to infer rates. Corrected Vout/Iout rates are roughly 451–512 sps. This creates well-centred test tones and does not independently measure hardware clock/frequency drift. A strongest-line/harmonic identification error would change inferred rates. China Vin has no qualifying line and keeps 400 sps.
- Vin/Vout absolute ripple is inferred by setting capture means to 60/26.5 V; capture telemetry is absent. Ratios are preserved, but absolute amplitudes are model inputs. Committed Vout RMS spans about 0.147–0.320 V and Iout RMS 0.826–3.384 A.
- The decoder defaults SIM_NOISE_EXP to 1. The intended new-profile experiment requires **explicit SIM_NOISE_EXP=0**. Omitting it changes disturbance amplitude with charger current and makes it vanish while off. In this review, china notch cap harvest changed from 80.0% to 82.8%, and its startup calibration trip vanished. The required override is in the commit/context but is not carried in FNZ metadata or prominently documented with the new profile list. Always record it with results.
- Residuals are open-loop additive disturbances; the 0.3 Hz trend is removed, records loop, and their boundaries are cross-faded. China is only 6.5 seconds long. This is not a coupled inverter/charger plant validation.
- Replay runs on one simulated ADC tick rate. Vin profiles with a different rate are indexed by truncation/repetition in noiseHook (`mppt-sim.cpp:270-279`), unlike physically independent ADC clocks; this can add sidebands and affect filter comparisons. Vout/Iout near the selected tick rate replay sample-for-sample.

These are scope qualifications (b), not invented firmware defects. They can favor a period-aligned average, and they prevent turning the reproduced percentages into a field-performance guarantee. Keep default notch, correct the opt-in timing model and documentation, and treat any later physical trial as outstanding work. No physical trial, full-device fault validation, or production release was performed.

Evidence index: `cells.json`, `extra-cells.json` and per-cell `.log` files; `math_check.py`, `math-results.txt`; `comb_audit.cpp`, `comb-audit-san.log`, `comb-audit.log`; `settling.cpp`, `settling.log`; `ripple-test.log`; `profile_audit.py`, `profile-audit.log`; `compile_target.py`, `compile_selected.py`, target `.asm`/`.o` files; `gain-qemu/main/main.c`, `gain-qemu/run.log`. All are beside this report, outside the reviewed repository.
