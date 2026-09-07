*this document is an LLM generated placeholder*

# Codex xhigh review of e88b91c (charger: EOC fix, temperature policy, partial-charge hold)

Run 2026-09-07, `codex exec -c model_reasoning_effort=xhigh` with network + a headed Chrome over CDP. All 12 findings were verified against the code and addressed in the follow-up commit; see git log.
## ACCESS

- CDP `Browser`: `Chrome/152.0.7977.77`
- Playwright `document.title`: `Charging Marine Lithium Battery Banks | Nordkyn Design`
- Reference: [Nordkyn Design article](https://nordkyndesign.com/charging-marine-lithium-battery-banks/)

## FINDINGS

1. **Critical — (c) The cold block can force substantial current into a low-SoC cold pack.**  
   [src/charger.h:423](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:423) clamps the load-following pin to
   `n_cells*(cv_min-recharge_vfloor_band)-vout_offset_max`. With the supplied 8S configuration this is `25.96 V`. For a cold pack at `24.8 V`, hold entry seeds from the bus but the first step clamps the pin *up* to 25.96 V, while [src/charger.h:373](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:373) leaves `ioutLim` unlimited because `ibat` is finite. At 20–60 mΩ that permits roughly 19–58 A, capped only by the normal 40 A limit, despite `_coldBlocked`. The test uses an artificially high bus voltage and checks only that the pin moves downward, not that current reaches zero. Additionally, when cold and partial holds overlap, [src/charger.h:486](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:486) uses `_holdIbatTarget()`; a deficit above the partial ceiling can therefore command up to **+1 A charging** into the cold pack rather than the claimed zero.

2. **High — (c) A dormant converter cannot start at dawn while partial-held or cold-blocked.**  
   In the only inactive MPPT start path, [src/main.cpp:1060](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:1060) treats every `chargeHold()` as “full” and [src/main.cpp:1077](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:1077) consequently refuses to start the sweep. If fry and flat both shut down overnight while holding 80% SoC, both remain at 0 W in full sun until another 56 Ah is discharged or the full-charge interval expires. A cold hold remains off until warming/expiry. This contradicts the promise that these modes keep serving loads, and [src/main.cpp:782](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:782) simultaneously disables the stuck watchdog.

3. **High — (c) The boot gate trusts a two-frame termination decision after only one frame.**  
   The new line latch requires two frames at [src/charger.h:222](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:222), but `_termDecided` becomes true after the first evaluation at [src/charger.h:343](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:343). The boot condition then stops waiting at [src/main.cpp:1067](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:1067). With warmed `ibat`, the first high-cell frame from a full pack increments the streak without latching; the RT task can immediately begin an open-loop sweep rather than waiting about 12 seconds for the second frame. This creates exactly the full-pack charge pulse the gate is intended to prevent, potentially before a 3.65 V BMS cutoff.

4. **High — (c) `ibat` has a frame stamp but no freshness policy, defeating both cold and partial holds.**  
   [src/charger.h:139](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:139) records `ibat_t`, but `ibatSmoothed()` remains finite forever. If the last value is +20 A and current frames stop while cold temperature frames continue, the cold path leaves `ioutLim` unrestricted, [src/charger.h:421](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:421) performs at most one step and then freezes the pin, and `chargeHold()` suppresses recovery. Charging can therefore continue indefinitely. The same dropout freezes `ahSinceFull`, can strand a partial hold, and lets fresh cell frames be evaluated against stale current for termination.

5. **High — (a) The advertised 0.25 A divide-by-zero floor does not cover a valid zero `Ibat_lim`.**  
   MQTT explicitly accepts zero at [src/charger.h:630](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:630), and configuration has no positive-value assertion. `Iout_max()` starts with that zero and then computes `min(0, 0.25)` at [src/charger.h:698](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:698), returning zero. The normalized controller divides by that setpoint at [src/pd_control.h:35](/Users/fab/dev/pv/fugu-mppt-firmware/src/pd_control.h:35), eventually producing INF/NAN and shutdown. During a cold/partial hold, finding 2 can then prevent restart. The floor must be applied to the final controller setpoint, or zero must trigger an explicit safe-disable path.

6. **High — (c) A partial hold remains logically asserted after stale cell data disables its physical hold.**  
   Once cell data expires, `batDataOk` becomes false and [src/charger.h:466](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:466) stops load-following, then the stale-BMS branch glides to `Vbat_fallback` at [src/charger.h:559](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:559). `_partialHold` nevertheless remains true and [src/charger.h:653](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:653) continues suppressing periodic sweeps and the stuck watchdog. A 26.4 V partial pack can consequently be charged toward the 26.96 V fallback; if a sibling holds the bus above fallback, this converter instead remains at 0 W. Either state can last until BMS recovery or the full-charge interval.

7. **High — (c) Hot derating cannot derive load current independently on a shared bus.**  
   [src/charger.h:381](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:381) subtracts the shared pack current from this converter’s output. Example: at 50 °C the 40 A pack limit should derate to 20 A; with two converters each producing 20 A, a 10 A load makes shared `ibat=30 A`. Each computes `max(20-30,0)=0` load and retains a 20 A output cap, leaving the pack at 30 A—10 A above the claimed derated limit. Independent controllers cannot recover total load or allocate the shared pack-current allowance from these two signals.

8. **Major — (c) Temperature freshness is global rather than per sensor.**  
   All four values share one `temp_t` at [src/charger.h:97](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:97). If sensor 0 reports −5 °C once and its topic dies while sensor 1 continues reporting 20 °C, sensor 1 continually refreshes the global stamp while the stored −5 °C remains in `tempMin()` forever. Both converters can remain cold-blocked—and excluded from startup/watchdog recovery—indefinitely. Out-of-range failures have the same problem because the previous finite value is never cleared.

9. **Major — (a) “One hour” is actually three hours while active and roughly 24 hours while disabled.**  
   Expiry counts 3,600 calls at [src/charger.h:311](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:311), not elapsed time. Normal LF cadence is three seconds at [src/main.cpp:79](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:79), and disabled cadence is eight times slower at [src/main.cpp:1182](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:1182). The CLI even labels this counter as seconds. The new test loops 3,700 times without advancing time, so it codifies the implementation rather than testing the claimed hour.

10. **Major — (c) Configuration permits partial-charge hysteresis below 0% SoC.**  
    [src/charger.h:71](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:71) does not validate `recharge_dod`, and `partial_charge` is not checked against it. With `partial_charge=0.10`, `recharge_dod=0.20`, and 280 Ah, the partial ceiling is 252 Ah discharged but release requires more than 308 Ah at [src/charger.h:405](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:405)—nominally −10% SoC. Negative or oversized `recharge_dod` also produces immediate flapping or unreachable release thresholds. A nonzero partial ceiling needs `0 < recharge_dod < partial_charge`; the new guard should also require `Cbat > 0`, not merely finite.

11. **Medium — (a) The new partial-cycle test overflows on its stated on-target platform.**  
    `driveCounter()` takes microsecond timestamps as `unsigned long` at [test/test_charger.cpp:41](/Users/fab/dev/pv/fugu-mppt-firmware/test/test_charger.cpp:41). That is 32-bit on ESP32 and wraps after about 71.6 minutes. The new test accumulates roughly 4,221 seconds before its 4,200-second move at [test/test_charger.cpp:642](/Users/fab/dev/pv/fugu-mppt-firmware/test/test_charger.cpp:642); conversion of the end timestamp wraps below the start, so the loop performs no integration and the on-target test should fail or test the wrong transition. Host `unsigned long` is 64-bit, explaining why the supplied host run passes.

12. **Low — (b) New hold state crosses cores through ordinary `bool` fields.**  
    `_partialHold` and `_coldBlocked` are written by core 0 and read through `chargeHold()` by core 1 without `volatile` or atomics at [src/charger.h:650](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:650). Aligned ESP32-S3 loads and coherent caches make this likely to work in practice, but it remains a C++ data race. A stale transition can briefly admit a sweep after cold blocking or retain the no-start gate after release.

## SURVIVED

- The main EOC correction is sound: the feedback target is fixed at `cv_eoc` while charging and `cv_min` after termination. The previous current-dependent positive-feedback path is removed, and the termination-line arithmetic still matches the Nordkyn interpolation.
- `Vout_max()` remains bounded by configured `Vbat_max`; I found no state/glide/release path that returns a higher setpoint. NAN initialization is handled before the load follower uses the pin.
- Normal `full_charge_interval_s * 1000000ULL` arithmetic is 64-bit and safe for the allowed sub-365-day range. The 32-bit cell timestamp age subtraction is wrap-safe.
- `releaseVoutPinning()` does not normally run inside a hold: the hold branch precedes the no-authority branch, and the watchdog is gated. The `_wasPartial`/`_wasTerminated` transitions did not expose an upward setpoint beyond `Vbat_max`.
- I found no deterministic two-converter oscillation solely from both load followers taking the same step; the concrete shared-bus defect is the hot-current/load calculation above.
- Empty BMS topics plus default `partial_charge=0` preserve the no-BMS behavior. PSU mode continues to use PSU limits/setpoints rather than the charger’s new limits.
- Topic trimming and ordinary trailing commas work; four entries are subscribed and extras are intentionally ignored.
- All new configuration keys are present in both `doc/Configuration.md` and `etc/config-tool/conf-editor.html`. No `<sstream>`, incompatible newlib-nano length modifiers, or RT-ISR work was introduced. The added state is small.
- The supplied host suite completed with `38 passed, 0 failed`; an additional host compile with `-Werror=missing-field-initializers` succeeded. The EOC regression and ordinary single-converter hot-derate tests are meaningful. The load-following tests are direction tests rather than plant tests, and there is no coverage for disabled-dawn startup, low-SoC cold blocking, stale `ibat`, stale individual temperature sensors, stale cell data during partial hold, overlapping cold/partial state, shared converters, zero runtime limits, or the first-frame boot-sweep window.
- No repository files were modified.

## END

I would not deploy `e88b91c` to fry/flat until the cold-floor, inactive-start, first-frame termination gate, and stale-`ibat` failures are addressed and exercised with a plant-aware/shared-bus test.

=== codex exit 0 ===
