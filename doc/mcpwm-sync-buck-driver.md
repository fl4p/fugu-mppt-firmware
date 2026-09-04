*this document is an LLM generated placeholder*

# MCPWM synchronous-buck PWM driver

Design spec for the MCPWM-based gate driver that replaces the LEDC implementation in
`src/pwm/ledc.h`. Targets ESP32-S3 and classic ESP32 (ESP-IDF ≥ 5.5).

## Why MCPWM

LEDC has no hardware dead-time, no hardware fault input, no native multi-channel phase
control, and forces a fixed 2048-tick period. MCPWM gives us all four: a per-operator
dead-time submodule, an OST brake driven by a GPIO fault, timer sync sources for
interleaved legs, and a 16-bit period counter we can size to the available source
clock.

## Scope and non-goals

In scope: edge-aligned (count-up) PWM, two-switch synchronous buck (HS + SR), hardware
dead-time, GPIO fault brake, N interleaved legs sharing one fault source.

Out of scope: center-aligned (up-down) carriers — HS-at-TEZ alignment is what the
existing buck controller and ADC sample timing assume. On-chip analog comparator faults —
GPIO faults only.

## Switch-cycle model

Count-up timer; one period = `period_ticks`. Per leg, two comparators schedule the two
turn-off events; turn-on of HS is the period boundary (TEZ).

Define:
- `cmpHS` = HS turn-off count = `pwmCtrl` (controller duty)
- `cmpLS` = LS turn-off count = `pwmCtrl + pwmRect` (rectifier on-time set by the
  diode-emulation logic in `buck.h`)

Generator actions (count-up direction only):

| Mode   | genHS                                | genLS                                              |
|--------|--------------------------------------|----------------------------------------------------|
| `HiLi` | HIGH at TEZ, LOW at cmpHS            | HIGH at **cmpHS**, LOW at cmpLS                    |
| `InEn` | HIGH at TEZ, LOW at cmpHS  (= IN)    | HIGH at **TEZ**,  LOW at cmpLS   (= EN window)     |

`HiLi` drives the HS and SR MOSFETs through a discrete gate driver with no built-in
interlock — the MCPWM dead-time submodule is responsible for shoot-through prevention.
`InEn` drives an integrated half-bridge driver (e.g. IR2814 family) where the chip
inserts its own dead-time; MCPWM emits IN and an EN window only.

Invariants the driver enforces (so the controller never has to think about them):
- `cmpHS < cmpLS < pwmMax`
- The LS conduction window never wraps past TEZ.
- D = 0 and D = 1 are reached by forcing both gates, not by setting `cmpHS = 0` or
  `cmpHS = period_ticks` (those produce one-tick glitches at the period boundary).

## Timing — `bestTiming(fsw)`

For a given switching frequency, pick the largest `period_ticks` that fits in 16 bits
using the highest available source clock and an integer group prescaler. The result is
the highest duty resolution the hardware can give us at `fsw`.

Source clock: `MCPWM_TIMER_CLK_SRC_DEFAULT` resolves to `PLL_F160M` = 160 MHz on both
ESP32-S3 and classic ESP32 in IDF 5.5.

Algorithm (returns `resolution_hz`, `period_ticks`, `actual_freq`):
1. `presc = 1`; while `src_clk / presc / fsw > 65535`, increment `presc`.
2. `resolution_hz = src_clk / presc`.
3. `period_ticks = round(resolution_hz / fsw)`.
4. `actual_freq = resolution_hz / period_ticks` (the frequency the timer actually
   produces; may differ from requested by less than `0.5 · resolution_hz / period_ticks²`).

Worked example: `fsw = 39 kHz`, `src_clk = 160 MHz` → `presc = 1`, `period_ticks ≈ 4103`,
`resolution = 160 MHz`, `actual_freq ≈ 38997 Hz` (~12-bit duty).

The driver exports `pwmMax`. After dead-time reservation (next section):
`pwmMax = period_ticks - dtLhTicks`. The controller clamps all comparator writes
to `[0, pwmMax - 1]`.

## Dead-time (HiLi)

Each MCPWM operator has **one shared dead-time submodule** — the posedge / negedge
delays in `mcpwm_dead_time_config_t` cannot be configured independently for both
generators. We therefore split the two transitions:

- **HS → LS (mid-period, at `cmpHS`):** delay the LS *rising* edge by `dtHlTicks` via
  `mcpwm_generator_set_dead_time(genLS, genLS, {posedge_delay_ticks = dtHlTicks})`. HS
  falls at `cmpHS` (no delay); LS rises `dtHlTicks` later. Dead-band = `dtHlTicks`.
- **LS → HS (period wrap, TEZ):** reserved in software by reducing `pwmMax`:
  `pwmMax = period_ticks - dtLhTicks`. Since the controller clamps `cmpLS ≤ pwmMax - 1`,
  LS goes low at least `dtLhTicks + 1` ticks before TEZ.

The two mechanisms are independent — a RED register write versus a `pwmMax` reservation
— so the two transitions carry **one value each**, and they need not be equal. `hl == 0
&& lh > 0` is a legal state: the dead-time submodule stays bypassed (no path claim, so
no 1-tick falling delay on HS, and `setDeadTimeTicks` still refuses to arm it later)
while the wrap band is still reserved out of `pwmMax`.

Conversion: `ticks = round(pwm_deadtime_{hl,lh}_ns × 1e-9 × resolution_hz)`, each key
defaulting to `pwm_deadtime_ns`. Must use the true `resolution_hz` from `bestTiming()`,
not `pwm_freq × period_ticks` (they only agree by accident when
`period_ticks = resolution_hz / pwm_freq` exactly).

The realized HS→LS gap is `dtHlTicks - 1`: claiming the dead-time path costs the HS
generator a 1-tick FED (see `init()`). The LS→HS band is `period_ticks - cmpLS` and
depends on no delay register; with every caller capping `cmpLS` at `pwmMax - 1` its
tightest realized value is `dtLhTicks + 1`, i.e. one tick wider than configured.

`InEn` mode passes `0, 0`; the half-bridge driver chip owns the dead-time.

## Comparator updates — TEZ-buffered

Both comparators are created with `update_cmp_on_tez = true`. Writes to `cmpHS` and
`cmpLS` are double-buffered and committed atomically at the next TEZ. Consequences:

- Order of `setHsOff()` / `setLsOff()` is irrelevant — no shrinking-first / growing-second
  dance.
- The wrong-direction race (write a smaller `cmpLS` after the counter has already passed
  it, the comparator event for the period is missed, LS stays HIGH to the wrap) cannot
  occur — the new value only takes effect at TEZ.
- Worst-case update latency = one PWM period. At 39 kHz that is ≈ 26 µs, well inside
  the RT loop budget.

## Fault brake (zero-CPU shutdown)

One GPIO fault per MCPWM group, shared by all legs in that group:
- `mcpwm_new_gpio_fault` with configurable `active_level` and matching pull resistor.
- `mcpwm_operator_set_brake_on_fault` with `MCPWM_OPER_BRAKE_MODE_OST` (one-shot trip;
  latches until explicitly cleared).
- On each generator, `mcpwm_generator_set_action_on_brake_event(..., GEN_ACTION_LOW)`
  so both gates go to the safe level the instant the fault asserts, with no CPU
  involvement.
- Recovery is explicit (`mcpwm_operator_recover_from_fault`) — a fault never silently
  clears, so any sensor-watchdog or driver-fault trip stays latched until firmware
  decides to re-arm.

## Software-forced shutdown

Distinct from the brake — used for normal disable / re-arm sequences from the RT path:
- `mcpwm_generator_set_force_level(g, 0, true)` on both generators (register write, no
  allocation, ISR-safe).
- Released with `set_force_level(g, -1, true)`. Re-arm sequence: write both comparators
  to the desired new values, *then* clear the force; the next TEZ will apply both
  comparators and resume switching from a known state.

## Interleaving — N legs

`MCPWM_Converter<N>` holds an `std::array` of N legs (= N operators / N timers) in one
group plus one fault brake. Phase relationship:

- Leg 0's timer publishes a sync source on TEZ (`mcpwm_new_timer_sync_src`).
- Legs 1..N-1 take that sync and set `count_value = period_ticks × i / N`
  (`mcpwm_timer_set_phase_on_sync`), giving uniform 360°/N spacing.
- Sync is one-shot at start; the timers run free afterward (sub-tick drift between
  legs is below the comparator quantum and not corrected).
- `setHsOff` / `setLsOff` fan out to all legs. Per-leg phase trimming is not in scope.

`N = 1` is the same code with no sync source created.

## Public driver surface

`MCPWM_FaultBrake`
- `initGpio(group, pin, activeHigh)` — register the fault input.
- `bindLeg(operator, genHS, genLS)` — install OST brake + LOW actions on this leg.
- `recover(operator)` — clear the latched OST condition.

`MCPWM_SyncLeg`
- `init(group, fsw, pinHS, pinLS, dtHlTicks, dtLhTicks, enLogic, fixedTicks = 0)` — build timer,
  operator, comparators, generators, dead-time. `fixedTicks > 0` overrides
  `bestTiming()` (kept for migration / bit-identical replays; not the production path).
- `setHsOff(uint16_t)`, `setLsOff(uint16_t)` — comparator writes (TEZ-buffered).
- `start()` — enable + `START_NO_STOP`.
- `forceShutdown()`, `clearForce()` — RT-safe force-level on both gates.
- `pwmMax` — period after dead-time reservation; controller clamps to `[0, pwmMax - 1]`.

`MCPWM_Converter<N>`
- Same `init(...)` taking pin arrays of length N plus optional fault pin.
- Fanned-out `setHsOff` / `setLsOff` / `forceShutdown` / `clearForce`.

## Configuration (`board.conf`)

| key                      | meaning                                                    |
|--------------------------|------------------------------------------------------------|
| `pwm_freq`               | Hz; passed to `bestTiming`                                 |
| `pwm_driver_logic`       | `HiLi` or `InEn` (selects the genLS action table above)    |
| `pwm_hi` / `pwm_li`      | gate pins (HiLi)                                           |
| `pwm_in` / `pwm_en`      | IN / EN pins (InEn)                                        |
| `pwm_sd` (optional)      | driver SD pin, driven high in `init`                       |
| `pwm_deadtime_ns`        | HiLi dead-time in ns, both transitions; ignored when `InEn` |
| `pwm_deadtime_hl_ns`     | HS→LS override (RED register, realized −1 tick)            |
| `pwm_deadtime_lh_ns`     | LS→HS override (`pwmMax` reservation, realized exactly)    |
| `pwm_fault_pin` (opt.)   | GPIO fault input pin                                       |
| `pwm_fault_active_high`  | fault polarity (0/1); pull resistor set accordingly        |

## Invariant: a latched period must never have `cmpLS < cmpHS`

In HiLi mode the LS generator has **no TEZ action** — it goes HIGH at `cmpHS` and LOW at `cmpLS`
(`src/pwm/mcpwm.h`), so **LS holds its level across the period wrap**. A period that latches with
`cmpLS < cmpHS` therefore runs: the `cmpLS`→LOW event passes as a no-op, `cmpHS` drives LS HIGH,
and at TEZ HS goes HIGH on top of it — both FETs on for most of a period.

`cmpHS` and `cmpLS` are two separate registers and both latch on TEZ, so a TEZ falling between the
two writes publishes a *mixed* pair. Order the writes so the mixed pair stays ordered:

* **HS widening** (`cmpHS` increasing) — write `cmpLS` first.
* **HS narrowing** — write `cmpHS` first.

The normal control path is safe without this because duty moves by a few counts per tick and
`pwmRectMin` (~330 ct) covers the gap. It matters wherever `cmpHS` jumps by a large amount:
`applyPendingPwmFreqRt()` rescales it by `newPeriod/oldPeriod`, so a 75 → 39 kHz change at duty
0.61 moves `cmpHS` from 1281 to 2464 against a stale `cmpLS` of 2100 — 25.6 µs of shoot-through.
