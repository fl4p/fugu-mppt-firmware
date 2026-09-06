## ACCESS

`git -C /Users/fab/dev/pv/fugu-mppt-firmware status --short | head -20`:

```text
 M .claude/agent-memory/ee-code-verifier/MEMORY.md
 M .claude/memory/MEMORY.md
 M .claude/memory/project_mcpwm_dt_pair_two_calls.md
 M config/lab/f2_test/conf/converter.conf
 M config/lab/fbuck_lab_bench_open_output/conf/board.conf
 M config/psu_12v/conf/converter.conf
 M doc/Configuration.md
 M doc/Console.md
 M "doc/Power Loop.md"
 M etc/config-tool/conf-editor.html
 M etc/fugu_console.py
 M etc/influx_binary_proxy.py
 M etc/test_fugu_console_modes.py
 M src/buck.h
 M src/cli.cpp
 M src/mppt.cpp
 M src/mppt.h
?? .claude/agent-memory/ee-code-verifier/project_mcpwm_39khz_tick_arithmetic.md
?? .claude/agent-memory/ee-code-verifier/project_mcpwm_cmphs_before_cmpls_shootthrough.md
?? .claude/agent-memory/ee-code-verifier/project_mcpwm_deadtime_update_method_immediate.md
```

Playwright MCP `<title>`:

```text
Motor Control Pulse Width Modulator (MCPWM) - ESP32-S3 - — ESP-IDF Programming Guide v6.1 documentation
```

## FINDINGS

1. **(c) Critical — “duty still” can engage and indefinitely retain real reverse current against a stiff output.**  
   The code equates an unchanged PWM count with a no-load converter having converged to `D0`, but it has no evidence that the output is floating. A manual target, a zero controller output, quantization, or a stiff load can all hold duty below the physical zero-current ratio. The lower threshold then engages after the hold, and the still-lower drop threshold keeps it engaged indefinitely. See [buck.h:1546](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1546), [buck.h:1552](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1552), and [buck.h:1597](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1597).

   For 26.2/70.5 V:

   ```text
   r          = 0.371631
   err        = 0.007508
   converge   = 0.364124
   drop       = 0.354124
   ```

   Thus:

   - The observed stationary `D=0.369` would nominally produce about **−6.2 A** on a stiff output.
   - A fresh settled engagement at the converge floor permits about **−17.6 A** nominal.
   - An already-engaged gate can remain on at the drop floor with about **−41.1 A** nominal.
   - Including the admitted worst-case ratio error, those last two bounds become approximately **−35.3 A** and **−58.8 A**.

   Magnitude is bounded only under fixed voltages, measurement error, and resistance; **duration is not bounded**. Consequently, the documentation’s “for the few samples it takes” claim is false ([Configuration.md:422](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Configuration.md:422)). The table and config-tool descriptions also incorrectly summarize the gate as engaging only “above” the ratio ([Configuration.md:132](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Configuration.md:132), [conf-editor.html:324](/Users/fab/dev/pv/fugu-mppt-firmware/etc/config-tool/conf-editor.html:324)).

2. **(c) Critical — the hold counters are not clocked by fresh voltage samples, so the filter-settling guarantee is absent.**  
   `protect()`—and therefore `updateForcedPwmGate()`—runs outside the `haveNewSample` condition, while duty control runs only when `Vout::numSamples` advances ([main.cpp:1013](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:1013), [main.cpp:1020](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:1020)). A streamed ADC wake is reported as `NewData` even when averaging emitted no new relevant voltage sample ([sampling.h:592](/Users/fab/dev/pv/fugu-mppt-firmware/src/adc/sampling.h:592), [sampling.h:653](/Users/fab/dev/pv/fugu-mppt-firmware/src/adc/sampling.h:653)).

   Both `fpwmStillCnt` and `fpwmGateCnt` nevertheless advance on every such call ([buck.h:1561](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1561), [buck.h:1604](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1604)). They can therefore count repeated stale EWMA values as “settling.” If Vout stops updating while another ADC remains live, duty is frozen by `haveNewSample=false`, the gate calls that “settled,” and it can engage without the stated filter catch-up ever occurring. The documented unit “samples” is consequently not a Vin/Vout-filter sample, and the requirement that the hold exceed the filter span is not enforced ([Configuration.md:439](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Configuration.md:439)).

3. **(c) Critical — ramp-down intentionally accumulates substantial reverse current, and `fpwmRecheckAfterStep()` does not actually clamp the LS on ordinary steps.**  
   The nominal D0-to-drop band is `err + margin = 0.017508`, or about 71.8 ticks at a 4103-tick period. With the manual eight-count ramp ([mppt.cpp:331](/Users/fab/dev/pv/fugu-mppt-firmware/src/mppt.cpp:331)), disengagement fires on roughly the **ninth decreasing sample after crossing D0**; including worst-case ratio error, about the **thirteenth**.

   Fractional control has no fixed sample bound:

   ```text
   N ≈ ceil(71.8 / |fp counts per sample|)
   ```

   At one count per sample that is about 72 samples nominally or 103 with worst-case ratio error; as `fp` approaches zero, the dwell is unbounded.

   Worse, on the firing step `fpwmRecheckAfterStep()` only clears `fpwmEngaged` ([buck.h:1619](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1619)). While forced PWM was active, `computeDCM()` forced `dcmHysteresis=false` ([buck.h:1394](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1394)). An ordinary −8 or −1 step is not a `largerDecrease`, so the subsequent `computePwmRectMax()` still chooses the CCM complementary window and commits it ([buck.h:1298](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1298), [buck.h:1310](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1310)). The rectifier is not clamped until the next `protect()` call. Thus the recheck does not fulfill its stated “duty about to be committed” protection and leaves one additional full control interval at maximum reverse current.

4. **(c) High — LS fade-in is guaranteed only in manual PWM, not in all relevant modes.**  
   Manual mode does call `pwmPerturb(0)` on every sampler wake ([main.cpp:1080](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:1080)). Automatic modes instead call `pwmPerturbFractional(fp)` ([mppt.cpp:261](/Users/fab/dev/pv/fugu-mppt-firmware/src/mppt.cpp:261)), which calls `pwmPerturb()` only when the truncated integer step is nonzero ([buck.h:1358](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1358)).

   Gate engagement raises `pwmRectMax` but does not raise `pwmRect` ([buck.h:1633](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1633)); the actual fade exists only inside `pwmPerturb()` ([buck.h:1328](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1328)). Therefore a PSU or PV controller exactly at equilibrium (`fp==0`) can engage forced PWM while leaving the LS at its diode-emulation minimum indefinitely, carrying current through the body diode. MPPT and sweep normally generate nonzero motion and eventually fade, but there is no every-tick guarantee for them either.

5. **(b) High — `forcedPwmRequested()` is used as a proxy for “current sensor unusable,” although that is not part of `forced_pwm` semantics.**  
   The relaxed sensor/HB-failure branch is selected immediately on request, while the gate is still armed ([mppt.h:755](/Users/fab/dev/pv/fugu-mppt-firmware/src/mppt.h:755)). The assumption happens to hold for this power-loop rig, but the documented general use is also PSU regulation ([Configuration.md:380](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Configuration.md:380)), potentially with a working sensor. Such a board loses the strict `0.8·D > ratio` check and retains only the much later `0.5·D > ratio` check during bring-up. [Console.md:130](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Console.md:130) incorrectly says protections relax only once forced PWM engages.

6. **(c) Medium — the new sync-rect hysteresis does not cover a fresh latch whose sensor remains exactly at 0.01 A.**  
   The latch initializes re-armed (`false`) at [buck.h:121](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:121), and its first off comparison remains strictly `< SyncRectOffCurrent` ([buck.h:1475](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1475)). A dead/quantized sensor pinned at exactly 0.010 A never enters the latched-off state, so the original full DCM-window fade and reverse-current sawtooth can recur after a cold start until some reading dips strictly below 0.01 A.

7. **(c) Low — the new bare `sync` status path introduces formal cross-core data races.**  
   `fpwmStillCnt` is labelled RT-only, but `forcedPwmEngageDuty()` reads it and is called by the core-0 console ([buck.h:170](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:170), [buck.h:594](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:594), [cli.cpp:114](/Users/fab/dev/pv/fugu-mppt-firmware/src/cli.cpp:114)). The threshold floats are likewise RT-written and core-0-read despite being plain non-atomic objects ([buck.h:172](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:172)). `dutyCtrlEff()` and the new status output also read plain PWM state concurrently. The likely consequence on ESP32-S3 is stale/incoherent diagnostics, rather than erroneous actuation, but under C++ these are data races and undefined behavior.

8. **(b) Low — the PSU profile disables the safety gate based on a now-obsolete operating assumption.**  
   [config/psu_12v/conf/converter.conf:3](/Users/fab/dev/pv/fugu-mppt-firmware/config/psu_12v/conf/converter.conf:3) says the gate withholds forced PWM at light/no load, but the new settled path exists specifically to engage there. `fpwm_gate=0` consequently depends on the PSU output never being precharged or stiff/sinking; otherwise it restores the destructive forced-PWM ramp from zero. The warning documents that risk, but the stated rationale for opting out is no longer coherent with the implementation.

9. **(a) Low — Power Loop uses the wrong loop-resistance expression.**  
   [Power Loop.md:47](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Power%20Loop.md:47) states `R_loop = DCR + Rds`, while the stated hardware model and [Configuration.md:393](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Configuration.md:393) use DCR plus two switch resistances. This makes the two current-slope descriptions internally inconsistent.

## SURVIVED

- The gate’s own normal forward-threshold freeze does **not by itself** self-promote into the settled threshold: on the first moving passing sample `gateCnt=1` and `stillCnt=0`; it reaches `gateCnt=hold` one call before `stillCnt=hold`. External/stale/quantized duty holds remain unsafe as described above.
- “Decreasing steps only” is the correct directional trigger for both buck and boost; increasing control duty moves away from reverse current. The defect is the low floor and ineffective immediate LS handoff.
- The `vh > vl` and minimum-voltage guards keep `dZero` strictly physical. A separate `max(0, dZero)` is not needed.
- The boost error term `r·kErr`, rather than `dZero·kErr`, is correct.
- A working current sensor does not leave `syncRectOffHyst` stuck OFF: while `il < 0.1 A`, `computeDCM()` is forced true, so the latch can clear at `il >= 0.04 A` and `vl >= 1 V`; in CCM the latch is ignored. Retaining it across `disable()` is fail-safe.
- The request word’s sequence/immediate payload is coherently published with CAS and release/acquire ordering. `forcedPwm`, `fpwmEngaged`, `fpwmUngated`, and `fpwmGateCnt` are atomic; `fpwmAckWord` and `fpwmLastCtrl` remain RT-only.
- `forcedPwmGateArming()`’s `!disabled()` guard prevents the previously identified restart deadlock.
- `fpwm_gate_hold` is clamped before narrowing to `[1, 65535]`; saturation and the `65535` endpoint work.
- The new RT logic introduces no allocation, exception, or OS wait beyond existing logging; formats use `%hu`/`%f`, not `%hh` or `%ll`. New members have explicit initializers, and `git diff --check` passed.
- Config-tool `META`, `FILE_KEYS`, `TYPE_KEYS`, and `DEFAULTS` all contain the three new keys with getter-compatible types and defaults.
- Manual PWM’s LS fade path is confirmed. The automatic-mode claim is the part that fails.
- The review was read-only; no file was modified.
