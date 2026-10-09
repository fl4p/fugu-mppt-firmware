---
title: "PWM Drivers (MCPWM)"
sidebar_position: 7
---

# MCPWM synchronous-buck PWM driver

This page specifies the MCPWM-based gate driver that replaces the LEDC implementation in
`src/pwm/ledc.h`. It targets ESP32-S3 and classic ESP32 (ESP-IDF ≥ 5.5).

## MCPWM vs LEDC

MCPWM has four features that LEDC lacks: a per-operator dead-time submodule, an OST brake
driven by a GPIO fault, timer sync sources for interleaved legs, and a 16-bit period counter
that we can size to the available source clock. LEDC has no hardware dead-time, no hardware
fault input, and no native multi-channel phase control, and it forces a fixed 2048-tick period.

## Scope

The driver covers edge-aligned (count-up) PWM, a two-switch synchronous buck (HS + LS),
hardware dead-time, a GPIO fault brake, and N interleaved legs sharing one fault source.

Center-aligned (up-down) carriers are out of scope, because the existing buck controller and
ADC sample timing assume HS-at-TEZ alignment. On-chip analog comparator faults are also out of
scope; the driver supports GPIO faults only.

## Switch-cycle model

The timer counts up, and one period is `period_ticks`. HS turns on at the period boundary
(TEZ). Per leg, two comparators schedule the two turn-off events:

- `cmpHS` = HS turn-off count = `pwmCtrl` (controller duty)
- `cmpLS` = LS turn-off count = `pwmCtrl + pwmRect` (rectifier on-time set by the
  diode-emulation logic in `buck.h`)

The generators act in the count-up direction only, as the following table shows:

| Mode   | genHS                                | genLS                                              |
|--------|--------------------------------------|----------------------------------------------------|
| `HiLi` | HIGH at TEZ, LOW at cmpHS            | HIGH at **cmpHS**, LOW at cmpLS                    |
| `InEn` | HIGH at TEZ, LOW at cmpHS  (= IN)    | HIGH at **TEZ**,  LOW at cmpLS   (= EN window)     |

`HiLi` drives the HS and LS MOSFETs through a discrete gate driver with no built-in
interlock, so the MCPWM dead-time submodule is responsible for shoot-through prevention.

`InEn` drives an integrated half-bridge driver (e.g. IR2814 family), and the chip inserts its
own dead-time. MCPWM emits only IN and an EN window.

The driver enforces these invariants, so the controller does not have to:

- `cmpHS < cmpLS < pwmMax`
- The LS conduction window never wraps past TEZ.
- The driver reaches D = 0 and D = 1 by forcing both gates, not by setting `cmpHS = 0` or
  `cmpHS = period_ticks`. Those values produce one-tick glitches at the period boundary.

## Timing — `bestTiming(fsw)`

For a given switching frequency, `bestTiming()` picks the largest `period_ticks` that fits in
16 bits, using the highest available source clock and an integer group prescaler. The result is
the highest duty resolution the hardware can give us at `fsw`.

The source clock `MCPWM_TIMER_CLK_SRC_DEFAULT` resolves to `PLL_F160M` = 160 MHz on both
ESP32-S3 and classic ESP32 in IDF 5.5.

The algorithm returns `resolution_hz`, `period_ticks`, and `actual_freq` in four steps:

1. `presc = 1`; while `src_clk / presc / fsw > 65535`, increment `presc`.
2. `resolution_hz = src_clk / presc`.
3. `period_ticks = round(resolution_hz / fsw)`.
4. `actual_freq = resolution_hz / period_ticks`. This is the frequency the timer actually
   produces. It may differ from the requested frequency by less than
   `0.5 · resolution_hz / period_ticks²`.

For example, `fsw = 39 kHz` and `src_clk = 160 MHz` give `presc = 1`, `period_ticks ≈ 4103`,
`resolution = 160 MHz`, and `actual_freq = 160e6 / 4103 ≈ 38995.9 Hz` (the integer `actual_freq`
is 38995). That is about 12-bit duty.

The driver exports `pwmMax`, which is the period after the dead-time reservation described in the
next section: `pwmMax = period_ticks - dtLhTicks`. The controller clamps all comparator writes
to `[0, pwmMax - 1]`.

## Dead-time (HiLi)

Each MCPWM operator has one shared dead-time submodule. The posedge and negedge delays in
`mcpwm_dead_time_config_t` can't be configured independently for both generators. The driver
therefore handles the two transitions with separate mechanisms:

- **HS → LS (mid-period, at `cmpHS`):** delay the LS *rising* edge by `dtHlTicks` via
  `mcpwm_generator_set_dead_time(genLS, genLS, {posedge_delay_ticks = dtHlTicks})`. HS
  falls at `cmpHS` after the 1-tick falling-edge delay it gets from claiming the dead-time path; LS rises
  `dtHlTicks` after `cmpHS`. Realized dead-band = `dtHlTicks − 1` (see below).
- **LS → HS (period wrap, TEZ):** reserved in software by reducing `pwmMax`:
  `pwmMax = period_ticks - dtLhTicks`. Since the controller clamps `cmpLS ≤ pwmMax - 1`,
  LS goes low at least `dtLhTicks + 1` ticks before TEZ.

The two mechanisms are independent: one is a RED register write and the other a `pwmMax`
reservation. Each transition therefore carries its own value, and the two values need not be
equal. `hl == 0 && lh > 0` is a legal state. The dead-time submodule stays bypassed, so there is
no path claim and no 1-tick falling delay on HS, and `setDeadTimeTicks` still refuses to arm it
later. The wrap band is still reserved out of `pwmMax`.

The realized gaps differ from the configured values by one tick:

- The realized HS→LS gap is `dtHlTicks - 1`, because claiming the dead-time path costs the HS
  generator a 1-tick FED (see `init()`).
- The LS→HS band is `period_ticks - cmpLS` and depends on no delay register. With every caller
  capping `cmpLS` at `pwmMax - 1`, its tightest realized value is `dtLhTicks + 1`, i.e. one tick
  wider than configured.

The driver converts nanoseconds to ticks with
`ticks = round(pwm_deadtime_{hl,lh}_ns × 1e-9 × resolution_hz)`. Each key defaults to
`pwm_deadtime_ns`. The conversion must use the true `resolution_hz` from `bestTiming()`, not
`pwm_freq × period_ticks`. The two only agree by accident, when
`period_ticks = resolution_hz / pwm_freq` exactly.

`InEn` mode passes `0, 0`, because the half-bridge driver chip owns the dead-time.

## Comparator updates — TEZ-buffered

Both comparators are created with `update_cmp_on_tez = true`. Writes to `cmpHS` and `cmpLS` are
double-buffered and latched at the next TEZ. A write pair is atomic only if no TEZ falls between
the two writes. This has three consequences:

- The order of `setHsOff()` / `setLsOff()` matters, because a TEZ between the two writes publishes
  a mixed pair. The next section explains the hazard and the write order.
- The wrong-direction race can't occur, because the new value only takes effect at TEZ. In that
  race, firmware writes a smaller `cmpLS` after the counter has already passed it, the comparator
  event for the period is missed, and LS stays HIGH to the wrap.
- The worst-case update latency is one PWM period. At 39 kHz that is ≈ 26 µs, well inside
  the RT loop budget.

## Invariant: a latched period must never have `cmpLS < cmpHS`

In HiLi mode the LS generator has no TEZ action. It goes HIGH at `cmpHS` and LOW at `cmpLS`
(`src/pwm/mcpwm.h`), so LS holds its level across the period wrap. A period that latches with
`cmpLS < cmpHS` therefore runs as follows: the `cmpLS`→LOW event passes as a no-op, `cmpHS` drives
LS HIGH, and at TEZ HS goes HIGH on top of it. Both FETs stay on for most of a period.

`cmpHS` and `cmpLS` are two separate registers and both latch on TEZ, so a TEZ falling between the
two writes publishes a *mixed* pair. Order the writes so the mixed pair stays ordered: when HS
widens (`cmpHS` increasing), write `cmpLS` first; when HS narrows, write `cmpHS` first.

The normal control path is safe without this because duty moves by a few counts per tick and
`pwmRectMin` (~330 ct) covers the gap. The per-tick commit (`drvCommit`) always writes HS then LS,
since the step between ticks is small.

The order matters wherever `cmpHS` jumps by a large amount. On a PWM-frequency change
(`src/buck.h`), `applyPendingPwmFreqRt()` rescales `cmpHS` by `newPeriod/oldPeriod`, which moves
it by hundreds of counts. For example, a 75 → 39 kHz change at duty 0.61 moves `cmpHS` from 1281
to 2464 against a stale `cmpLS` of 2100, giving 25.6 µs of shoot-through. On this path the
writes are ordered: LS first when HS widens, HS first when HS narrows, so every mixed pair keeps
`cmpLS >= cmpHS`. The comparators are written first when the period shrinks, and the period first
when it grows.

## Fault brake (zero-CPU shutdown)

Each MCPWM group has one GPIO fault, shared by all legs in that group. The driver sets it up as
follows:

- `mcpwm_new_gpio_fault` with configurable `active_level` and matching pull resistor.
- `mcpwm_operator_set_brake_on_fault` with `MCPWM_OPER_BRAKE_MODE_OST` (one-shot trip;
  latches until explicitly cleared).
- On each generator, `mcpwm_generator_set_action_on_brake_event(..., GEN_ACTION_LOW)`
  so both gates go to the safe level the instant the fault asserts, with no CPU
  involvement.
- Recovery is explicit (`mcpwm_operator_recover_from_fault`). A fault never clears
  silently, so any sensor-watchdog or driver-fault trip stays latched until firmware
  decides to re-arm.

## Software-forced shutdown

The RT path uses a software force, separate from the brake, for normal disable and re-arm
sequences:

- `mcpwm_generator_set_force_level(g, 0, true)` on both generators (register write, no
  allocation, ISR-safe).
- Released with `set_force_level(g, -1, true)`. Re-arm sequence: write both comparators
  to the desired new values, *then* clear the force; the next TEZ will apply both
  comparators and resume switching from a known state.

## Interleaving — N legs

`MCPWM_Converter<N>` holds an `std::array` of N legs (= N operators / N timers) in one
group plus one fault brake. The legs keep their phase relationship as follows:

- Leg 0's timer publishes a sync source on TEZ (`mcpwm_new_timer_sync_src`).
- Legs 1..N-1 take that sync and set `count_value = period_ticks × i / N`
  (`mcpwm_timer_set_phase_on_sync`), giving uniform 360°/N spacing.
- Leg 0's TEZ re-syncs legs 1..N-1 every period (the sync source stays connected), so the
  phase offset is re-imposed each cycle. Changing leg 0's period changes all legs.
- `setHsOff` / `setLsOff` fan out to all legs. Per-leg phase trimming is not in scope.

`N = 1` is the same code with no sync source created.

## Public driver surface

The driver exposes three classes.

`MCPWM_FaultBrake`
- `initGpio(group, pin, activeHigh)`: register the fault input.
- `bindLeg(operator, genHS, genLS)`: install OST brake + LOW actions on this leg.
- `recover(operator)`: clear the latched OST condition.

`MCPWM_SyncLeg`
- `init(group, fsw, pinHS, pinLS, dtHlTicks, dtLhTicks, enLogic, fixedTicks = 0)`: build timer,
  operator, comparators, generators, dead-time. `fixedTicks > 0` overrides
  `bestTiming()` (kept for migration / bit-identical replays; not the production path).
- `setHsOff(uint16_t)`, `setLsOff(uint16_t)`: comparator writes (TEZ-buffered).
- `start()`: enable + `START_NO_STOP`.
- `forceShutdown()`, `clearForce()`: RT-safe force-level on both gates.
- `pwmMax`: period after dead-time reservation; controller clamps to `[0, pwmMax - 1]`.

`MCPWM_Converter<N>`
- Same `init(...)` taking pin arrays of length N plus optional fault pin.
- Fanned-out `setHsOff` / `setLsOff` / `forceShutdown` / `clearForce`.

## Configuration (`board.conf`)

The driver reads these keys from `board.conf`:

| key                      | meaning                                                    |
|--------------------------|------------------------------------------------------------|
| `pwm_freq`               | Hz; passed to `bestTiming`                                 |
| `pwm_driver_logic`       | `HiLi` or `InEn` (selects the genLS action table above)    |
| `pwm_hi` / `pwm_li`      | gate pins (HiLi)                                           |
| `pwm_in` / `pwm_en`      | IN / EN pins (InEn)                                        |
| `pwm_sd` (optional)      | driver SD pin, driven high in `init`                       |
| `pwm_deadtime_ns`        | HiLi dead-time in ns, both transitions; ignored when `InEn` |
| `pwm_deadtime_hl_ns`     | HS→LS override (RED register, realized −1 tick)            |
| `pwm_deadtime_lh_ns`     | LS→HS override (`pwmMax` reservation; tightest realized band `dtLh + 1` tick) |
| `pwm_fault_pin` (opt.)   | GPIO fault input pin                                       |
| `pwm_fault_active_high`  | fault polarity (0/1); pull resistor set accordingly        |
