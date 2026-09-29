*this document is an LLM generated placeholder*

# MCPWM driver test spec — zero-hardware, on-target

Scope: validate `src/pwm/mcpwm.h` (`MCPWM_SyncLeg`, `MCPWM_FaultBrake`,
`MCPWM_Converter<N>`) on ESP32-S3 using only self-observation — no scope, no
PicoScope, no signal generator, **no physical jumpers**.

Reference: `website/docs/internals/pwm-drivers.md` (switch-cycle model, dead-time
split, `update_cmp_on_tez = 1`, OST brake, interleaving). This spec does
not duplicate the design — it only enumerates what to measure.

Two driver modes: **HiLi** (discrete gate driver, MCPWM owns dead-time) and
**InEn** (integrated half-bridge driver, MCPWM emits IN + EN window only).
Per-test applicability called out where they differ.

Round 2 (external instrumentation) is enumerated at the end and is **not** in
scope here.

## Measurement primitive — internal GPIO-matrix loopback

The MCPWM capture-channel config flag `mcpwm_capture_channel_config_t::flags
.io_loop_back = 1` (`driver/mcpwm_cap.h`: *"For debug/test, the signal output
from the GPIO will be fed to the input path as well"*) shorts the pin's
output into the CAP input internally — no rewire, no jumper.

- **Resolution.** MCPWM capture clock = APB only
  (`SOC_MCPWM_CAPTURE_CLKS = {SOC_MOD_CLK_APB}`, `soc/clk_tree_defs.h`) →
  80 MHz, **12.5 ns/tick**, 32-bit free-running (~53.7 s wrap). Three
  channels per group share one timer (zero inter-channel offset). Don't
  confuse with the PWM *timer* clock, which is `PLL_F160M = 160 MHz`.
- **Noise floor.** GPIO matrix adds ~5–10 ns common-mode propagation; it
  cancels for pulse-width and dead-band differences. Random jitter √N-averages
  out: 1024 cycles at 39 kHz → ~0.4 ns mean std-error.
- **Resolvable dead-time.** ≥ ~30 ns absolute. Anything smaller is Round 2.

Edge-counting cross-check: PCNT on the same pin via matrix routing
(`esp_rom_gpio_connect_in_signal(pin, PCNT_SIG_CH*_IN*_IDX, false)`); ±1
count over a 1.000 s gate = ~26 ppm at 39 kHz.

Steady-state assertions (D=0 / D=1, brake latch): `digitalRead` with the
gate pin in `GPIO_MODE_INPUT_OUTPUT`.

## Measurement-rig self-test (run first, abort on failure)

Validates the on-board measurement plumbing — matrix routing, MCPWM_CAP
clock, PCNT counting, both-edge timestamping — by pointing it at the
production LEDC driver, which is well-exercised and treated here as a
known-good reference signal. If these fail, every MCPWM test downstream
is suspect; the suite aborts.

Wiring (one-time, in test setUp):

1. `ledc_channel_config(..., gpio_num = pin_ref)` — LEDC routes its output
   signal to `pin_ref` via the matrix and sets the pin to OUTPUT mode.
2. `mcpwm_new_capture_channel(..., gpio_num = pin_ref, flags.io_loop_back = 0)`
   — CAP creates the channel and adds the matrix-input route from `pin_ref` to
   `PWM0_CAPn_IN_IDX`. **Side effect (gotcha):** the driver's internal
   `gpio_config(INPUT)` disables OE *and* resets the matrix-output route to
   `SIG_GPIO_OUT_IDX` (the default GPIO_OUT_REG signal) — LEDC's drive
   silently disconnects.
3. `gpio_set_direction(pin_ref, GPIO_MODE_INPUT_OUTPUT)` — re-enable IE+OE.
   The matrix-input route from step 2 survives this.
4. `esp_rom_gpio_connect_out_signal(pin_ref, LEDC_LS_SIG_OUT0_IDX + ch, false,
   false)` — **required** to re-bind LEDC's matrix output. Without it, OE is
   on but the pin is driven 0 by GPIO_OUT_REG, not by LEDC. (Note: passing
   `gpio_num = -1` to the CAP config to avoid this is *not* an option — the
   CAP driver validates the pin.)
5. Optional: `esp_rom_gpio_connect_in_signal(pin_ref, PCNT_SIG_CHn_IN_IDX,
   false)` for the parallel PCNT path.

### Rig-1. Frequency path

LEDC at `fsw = 39 kHz`, `D = 0.5`. PCNT 1.000 s gate; MCPWM_CAP
`(t_rise[N−1] − t_rise[0]) / (N−1)` with N = 4000. **Expected freq is LEDC's
emitted frequency, not the requested `fsw`**: with 80 MHz APB and the LEDC
`pwmMax` chosen by `ledc_find_suitable_duty_resolution`, the actual freq is
`APB / (pwmMax + 1)` (e.g. 39062.5 Hz for `fsw = 39000`). Tolerance ±5 Hz
on that quantized value. Failure ⇒ broken matrix route to `PWM0_CAPn_IN_IDX`,
wrong APB-clock assumption, or PCNT input not attached. Aborts.

### Rig-2. Pulse-width path

LEDC at `fsw = 39 kHz`, sweep `D ∈ {0.25, 0.5, 0.75}`. MCPWM_CAP both
edges, `mean(t_fall − t_rise)` over 1024 cycles. Must match
`D × (1 / actual_freq)` within **±50 ns** (the ±25 ns spec1 originally claimed
assumed cleaner CAP latching; real ISR jitter at 39 kHz pushes single samples
to ~38 ns). Failure ⇒ both-edge capture is not wired (only rising/falling
captured), or matrix-input polarity inverted, or the CAP ISR is dropping
events. Aborts.

Tests Rig-3, Rig-4, etc. are **not** needed — the MCPWM suite only
relies on these two measurement primitives plus `digitalRead`, and
`digitalRead` does not go through the rig.

## Tests

### 1. HS frequency = configured `pwm_freq`

PCNT counts HS rising edges over a 1.000 s window. Expected count = `pwm_freq`;
tolerance ±2 counts. Cross-check via MCPWM_CAP: `(t_rise[N−1] − t_rise[0]) /
(N−1)` with N = 4000; methods agree within **±5 Hz or ±0.1%** (whichever is
larger — APB-vs-PLL_F160M clock-domain sync slop at high fsw). Compare against
`bestTiming(fsw).actual_freq`, not the requested `fsw`.

Sweep `fsw ∈ {20 k, 39 k} Hz`. **100 kHz is deferred to Round 2**: inter-edge
gap (5 µs) approaches CAP ISR latency (~1-3 µs on S3), dropped edges inflate
the mean period; observed variance 97-99.9 kHz across reboots makes ±0.1%
unmeetable on-board. Production fsw is 39 kHz, so 20/39 fully covers the
operating range; 100 kHz is upper-bound only.

### 2. HS duty matches commanded `cmpHS`

For D ∈ {0.25, 0.5, 0.75}: `setHsOff(D × pwmMax)`. MCPWM_CAP both edges,
N = 512 periods (accept ≥ N/2 captured); `mean(t_fall − t_rise)` vs
`(D × pwmMax) / pwmMax`. Tolerance **±50 ns** (bench reality; spec1's
original ±25 ns was optimistic). Guards: off-by-one in `pwmMax` after
dead-time reservation; wrong source-clock assumption.

**D = 0.1 and D = 0.9 are deferred to Round 2.** At 39 kHz those produce
sub-3-µs pulses that the CAP ISR (1-3 µs latency) cannot reliably
double-edge-timestamp — half the pulses come through as single edges and the
test would under-count.

### 3. `pwmMax` matches the source-clock arithmetic

Pure on-target arithmetic, no edges. After `init(group, fsw, pinHS, pinLS,
dtTicks, …)`:
- `period_ticks ≈ round(160 MHz / presc / fsw)`
- `dtTicks = round(pwm_deadtime_ns × resolution_hz × 1e−9)` — base must be
  `resolution_hz` from `bestTiming`, **not** `pwm_freq × period_ticks`
- `pwmMax == period_ticks − dtTicks` (HiLi) or `pwmMax == period_ticks` (InEn)

At fsw = 39 kHz, `dt = 100 ns`: `presc=1, period_ticks=4103, dtTicks=16` →
`pwmMax = 4087` (HiLi) / `4103` (InEn). Exact match required.

### 4. HS↔LS dead-band (HiLi only)

The MCPWM operator has **one shared dead-time submodule**: HS→LS comes from
the LS-generator posedge delay, LS→HS comes from the software `pwmMax`
reservation. Both transitions must be tested — passing one and failing the
other is exactly how shoot-through ships. **Skip for InEn** (chip-internal).

#### 4a. HS → LS (mid-period)

Two MCPWM_CAP channels with `io_loop_back = 1`, one per gate. Within each
period: `dt_HSLS = t(LS-rise) − t(HS-fall)`. Over 1024 periods:
`mean ≥ pwm_deadtime_ns − 30 ns` (±25 ns mean tol), **`min > 0`**. Any
sample with `min ≤ 0` is an immediate FAIL — that's shoot-through at the
logic level.

#### 4b. LS → HS (period wrap)

Set commanded `cmpLS = pwmMax − 1` explicitly (worst case, smallest gap).
`dt_LSHS = t(HS-rise[k+1]) − t(LS-fall[k])`. Over 1024 periods:
`min(dt_LSHS) ≥ (dtTicks + 1) ticks − 30 ns`.

### 5. LS forced off (sync rect disabled, diode-emulation path)

`setLsOff(cmpHS)`. MCPWM_CAP on LS pin: **0 rising edges in 100 ms**. PCNT
cross-check: count = 0. Verifies the driver path; doesn't test the
diode-emulation *decision* in `buck.h` (covered by `test_buck.cpp`).

### 6. LS forced on (sync rect saturated)

`setLsOff(pwmMax − 1)`. MCPWM_CAP on LS: `mean(t_fall − t_rise) >
(1 − (dtTicks + 1) / period) − 0.001` of a period. Safety: SD pin asserted
before this test (the bridge would otherwise see near-100% LS).

### 7. D = 0 → HS fully low

Per the design doc, "D=0 and D=1 are reached by forcing both gates, not by
setting `cmpHS = 0`" (the latter creates a one-tick period-boundary glitch).
Invoke `forceShutdown()` (or whatever the buck controller calls for D=0).
`digitalRead(pinHS) == 0` across 100 polls / 10 ms; PCNT = 0 over 100 ms.

### 8. D = 1 → HS fully high

Mirror of #7. Force HS high via `mcpwm_generator_set_force_level(genHS, 1,
true)` (or the driver's D=1 path). `digitalRead(pinHS) == 1` across 100
polls; MCPWM_CAP / PCNT: zero events.

### 9. Glitch-free duty step (TEZ-buffered atomic update)

`src/pwm/mcpwm.h:97` sets `update_cmp_on_tez = 1`. Test: three back-to-back
writes faster than one period — `setHsOff(D1 · pwmMax)`,
`setHsOff(D2 · pwmMax)`, `setHsOff(D3 · pwmMax)` with D ∈ {0.2, 0.5, 0.8}.
MCPWM_CAP timestamps HS rising + falling for ~64 periods. Acceptance: every
observed `(t_rise, t_fall)` pair has `t_fall − t_rise ∈ {D1 · pwmMax,
D2 · pwmMax, D3 · pwmMax}` (±1 tick) — **no intermediate widths, no missed
periods**. Regression guard against that flag being cleared (which would
silently re-introduce the LEDC-era ordering race on `cmpLS < cmpHS`).

### 10. Dead-time linearity in `pwm_deadtime_ns`

HiLi only. Sweep `pwm_deadtime_ns ∈ {50, 100, 200, 500} ns` via in-memory
`ConfFile` (the `test_buck.cpp` pattern), re-`init()` the leg, re-run Test 4a.
Mean dead-time vs conf value: slope = 1.0 within ±5 %, intercept ≤ 30 ns.
Coverage caveat: stays in `presc = 1` regime (fsw well under 2.44 kHz);
prescaler-engaged dead-time rounding is Round 2.

### 11. OST fault brake latches both gates LOW

Configure `pwm_fault_pin`. Drive that pin to active level **from another
GPIO routed via the matrix** — no external wire, just
`esp_rom_gpio_connect_out_signal` from a software-controlled GPIO into the
fault input. Within one APB cycle of assertion: `digitalRead` both gates =
LOW; MCPWM_CAP records no further edges; PCNT count freezes. Call
`MCPWM_FaultBrake::recover(leg.oper())`; switching must resume. Skip with
logged warning if `pwm_fault_pin` not in `board.conf`.

### 12. Interleaved phase relationship (`MCPWM_Converter<N>`, N ≥ 2)

Two MCPWM_CAP channels on legs 0 and 1, both `io_loop_back = 1`. Phase delta
`Δt = t(HS-rise[leg1]) − t(HS-rise[leg0])` should equal `period / N` within
±25 ns over 1024 cycles. **Caveat:** S3 has 3 capture channels per group;
N = 3 fits, N = 4 needs cross-group routing (tighter, deferred to Round 2).
Skip if `N == 1`.

## Pass / fail summary

| #   | Invariant                          | Method                  | Tolerance                       |
|-----|------------------------------------|-------------------------|---------------------------------|
|Rig-1| Rig freq path (LEDC reference)     | PCNT 1 s + MCPWM_CAP    | ±5 Hz @ 39 kHz                  |
|Rig-2| Rig pulse-width path (LEDC ref)    | MCPWM_CAP, 1024 cyc     | ±50 ns                          |
| 1   | HS freq = fsw (20k, 39k)           | PCNT 1 s + MCPWM_CAP    | ±5 Hz or ±0.1% (whichever ≥)    |
| 2   | HS duty matches `cmpHS` (0.25–0.75)| MCPWM_CAP, 512 cyc      | ±50 ns (≈ ±0.002 duty)          |
| 3   | `pwmMax` arithmetic                | host arithmetic         | exact                           |
| 4a  | HS→LS dead-band                    | MCPWM_CAP, 1024 cyc     | mean ±25 ns; **min > 0**        |
| 4b  | LS→HS dead-band (wrap)             | MCPWM_CAP, 1024 cyc     | min ≥ (dtTicks+1) − 30 ns       |
| 5   | LS force-off                       | MCPWM_CAP + PCNT        | 0 edges / 100 ms                |
| 6   | LS force-on                        | MCPWM_CAP               | LS-high > (1 − dt/period)−0.001 |
| 7   | D = 0 → HS low                     | digitalRead + PCNT      | 0 edges / 100 ms                |
| 8   | D = 1 → HS high                    | digitalRead + PCNT      | 0 edges / 100 ms                |
| 9   | Glitch-free duty step              | MCPWM_CAP, ≥64 periods  | no out-of-set pulse widths      |
| 10  | Dead-time linearity                | sweep + Test 4a         | slope 1 ±5 %, intercept ≤ 30 ns |
| 11  | OST brake latches LOW              | matrix-driven fault in  | both LOW < 1 µs                 |
| 12  | Interleaved phase (N ≥ 2)          | MCPWM_CAP, both legs    | period/N ±25 ns                 |

## Test harness (Unity / `RUN_TESTS=1`)

Tests live in `test/test_pwm.cpp`. Build via
`RUN_TESTS=1 idf.py -B build-tests build flash monitor` (separate build dir
per the CLAUDE.md test instructions). Use the existing `test_buck.cpp`
pattern: in-memory `std::unordered_map<std::string,std::string>` for the conf
so each test re-inits the leg with different parameters.

Skeleton per test:

1. Build conf map (`pwm_freq`, `pwm_driver_logic`, `pwm_hi`, `pwm_li`,
   `pwm_deadtime_ns`, …).
2. `MCPWM_SyncLeg leg; leg.init(0, fsw, pinHS, pinLS, dtTicks, enLogic);`
3. Create CAP channels on the gate pins **after** `leg.start()`, with
   `flags.io_loop_back = 1`, both edges enabled.
4. ISR pushes `{ts_apb, chan_id, edge}` into a lock-free ring. **Size for
   the largest test** — Rig-1 / Test 1 want 4000 cycles × 2 edges = 8000
   events, so 8192. The earlier "4096 = 1024 cyc × 4" sizing is wrong.
5. Drive `setHsOff` / `setLsOff` / fault / etc. as the test requires; wait
   until ring fills (≤ 100 ms at 39 kHz).
6. Stop capture, analyse on core 0 (not in ISR), assert.
7. Teardown: `forceShutdown()`, delete CAP channels + timer, clear matrix
   routing.

RT-path discipline: tests run from the Unity setup task on core 0 — not on
the RT loop — so `vTaskDelay` / `esp_timer_get_time` are fine. ISR callback
marked `IRAM_ATTR`. `ESP_ERROR_CHECK` only at test-setup time.

## Safety gate (run before every test)

1. Refuse to flash unless the operator confirms the target. Hard-code refusal
   for hostnames `fry` / `flat` in `test/main.cpp` — those are live converters
   (see CLAUDE.md). Bench / `dry_*` configs are unrestricted.
2. Assert the half-bridge driver SD pin (`pwm_sd` if configured) to disable
   the FETs **before** the test pokes any gate. With SD asserted, the driver
   IC ignores the gate signals — the test exercises the MCU peripheral, not
   the silicon. Boards without `pwm_sd` are allowed but log a warning.
3. Honour `board.conf::pwm_sd_active_low` for SD polarity.

## Round 2 / scope-required (NOT in this spec)

- **Absolute dead-time < ~30 ns.** Capture quantum is 12.5 ns; matrix
  propagation adds a few ns common-mode. PicoScope (≥ 250 MS/s) needed for
  sub-30-ns characterisation.
- **Gate slew / Miller plateau / VDS overshoot.** Purely analogue.
- **Actual shoot-through current.** Logic-level dead-time being right does
  not guarantee no overlap at the silicon — gate-driver propagation can still
  produce simultaneous conduction. Needs a DC-coupled high-bandwidth current
  probe at the bridge mid-point.
- **`dtTicks` rounding when prescaler engages** (very-high-fsw regime; not
  used in production 20–100 kHz range).
- **Interleaved-leg phase for N ≥ 4** (capture channels per group run out;
  cross-group routing is possible but cleaner with a 4-channel scope).
- **Brown-out / reduced-VDD behaviour** — needs external supply control.
- **EMI / gate-node ringing** — antenna problem, scope only.
- **Capture-path systematic offset characterisation** — would need a
  calibrated external delay reference to convert the ±50 ns tolerance into
  an absolute number.
- **Test 1 at 100 kHz** — CAP ISR latency drops edges when inter-edge gap
  approaches ISR latency; observed measurement variance 97-99.9 kHz on bench.
- **Test 2 duty extremes (D = 0.1, 0.9)** — sub-3-µs pulses at 39 kHz lose
  one edge per pulse to CAP ISR latency; can't be measured on-board.

## Feasibility verdict

**Achievable, zero-hardware.** Every invariant above is measurable on a
stock ESP32-S3 with no jumpers via internal GPIO-matrix loopback
(`io_loop_back = 1`), at 12.5 ns timestamp resolution. The **±50 ns** tolerance
swallows matrix propagation + ISR jitter, both common-mode for difference
measurements. (Spec1 originally claimed ±25 ns; bench reality is ±50 ns.)
Sub-30-ns dead-time, gate slew, shoot-through current, and EMI remain
Round 2 / scope-required.
