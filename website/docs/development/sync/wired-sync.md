---
title: "Wired Clock Sync (wsync)"
sidebar_position: 3
---

# Wired inter-chip MCPWM clock sync (`WITH_WSYNC`)

Wired sync locks a follower's MCPWM timer to a leader's with nanosecond-class jitter. The beacon
servo (`bsync`) is the microsecond-class alternative. The leader emits a pulse that its hardware
locks to the MCPWM TEZ event, and on the pulse's rising edge each follower's hardware reloads its
timer count. No software runs in the loop.

Edge-detection dispersion in the driver, the coupling RC, and the input threshold is in the low
single-digit nanoseconds, under one 6.25 ns tick. The follower resamples the sync edge in the
160 MHz group clock, so the relative jitter is up to one tick (6.25 ns). A follower never
free-runs for more than one period, so the sync absorbs crystal drift (±ppm) as a sub-tick
correction every cycle.

The follower runs its period 2 ticks short (`wsyncLeadTicks`). Its own TEZ therefore fires before
the leader's edge arrives, and the sync only truncates a period that has already wrapped. The gates
take their normal TEZ actions with the software-reserved LS→HS dead band intact, and the sync
drives only LS.

The sync must not drive HS. An HS sync action would switch HS on in the same clock that switches
LS off. The LS→HS band comes from `pwmMax -= dtTicks`, which holds only at a real period boundary,
so it isn't hardware dead time. An out-of-phase edge would command zero dead time and 30–80 ns of
cross-conduction.

The 2-tick margin is ~490 ppm at 39 kHz. The worst-case crystal mismatch is ~40 ppm, plus one
tick of resync quantization.

## Configuration

Wired sync uses one Kconfig option and four conf keys:

- Kconfig: `CONFIG_FUGU_WITH_WSYNC=y` (depends on `FUGU_WITH_MCPWM`).
- `board.conf::pwm_sync_pin`: the sync GPIO. On the leader it's the pulse output; on the follower
  it's the sync input, pulled down.
- `converter.conf::sync_role`: `none` (default), `leader`, or `follower`. Requires
  `pwm_driver=mcpwm`.
- `converter.conf::sync_phase_deg` (leader only): the pulse offset from the leader's own TEZ as an
  angle, which shifts the follower's period start (`180` = interleave, `0` = in phase). The angle
  doesn't depend on frequency, so it survives a `pwm_freq` change.
- `converter.conf::sync_phase_ns` (leader only): an additive trim on `sync_phase_deg` for wire and
  receiver propagation delay. That delay is a fixed time rather than an angle, so it has its own
  key.

A follower ignores `sync_phase_deg` and `sync_phase_ns` and logs a warning. Its reload target is
fixed at count 0, because any value past a live comparator would re-open the skipped-event hazard
described under [Leader pulse and follower reload](#leader-pulse-and-follower-reload).

Both devices must use the same `pwm_freq`. The sync re-phases the follower every cycle and doesn't
replace its period.

## Leader pulse and follower reload

The leader's pulse is high for ~1 µs once per period, at TEZ by default. A dedicated MCPWM operator
on the same timer generates it, so the gate-drive dead-time submodule is untouched.

On the follower, the sync edge reloads the count to 0 and latches the comparator and period shadow
registers (`update_cmp_on_sync`, `update_period_on_sync`, and `update_dead_time_on_sync`). If the
follower's crystal runs slow, the sync reloads the count below the period every cycle and the
follower's own TEZ never fires. The follower still gets its shadow latching from the sync event.

For the gates, the sync is only a partial TEZ substitute. Only the LS generator has a sync action
(`mcpwm_generator_set_action_on_sync_event`), so a jump at an arbitrary phase doesn't land in a
clean period-start state. On HiLi hardware, the LOW-at-sync action prevents HS/LS overlap after a
mid-cycle jump. HS has no sync action by design, for the dead-time reason above. HS turn-on stays
with TEZ, which the follower always reaches first because of `wsyncLeadTicks`.

An arbitrary-phase edge therefore causes no shoot-through, but it still distorts the pulses. On
HiLi (`enLogic=0`, `board.conf::pwm_driver_logic=HiLi`), the sync action drives LS low:

- A sync while HS is high forces LS low, leaves HS high, and resets the counter. HS stays on for
  another full `cmpHS` interval, which approaches twice its normal on-time.
- A sync while LS is high chops LS and skips the next HS pulse.

On InEn (`enLogic=1`), the same action drives LS/EN high, so the anomaly differs. The analysis above
applies to HiLi only.

For this reason the firmware arms the sync input before `start()`, while the gates are idle, and
nothing may arm a follower that hasn't positively qualified a live leader.

## Coupling circuit (DC-blocked, tolerates ~1 V static ground offset)

The network only blocks DC. It rejects a static ground offset but passes common-mode transients,
and both sides share a galvanic path. C1's reactance at the pulse edge is ~50 Ω against a ~7.7 k
node, so a ground-to-ground step couples in essentially unattenuated: at the receiver, a 2 V CM
step with a ≤1 µs edge looks the same as the sync pulse. Outside a quiet bench, use a 1:1 pulse
transformer or a digital isolator instead of this network.

The network connects the leader's GPIO to the follower's sync input:

```
  LEADER                                                      FOLLOWER

                                                            3V3_follower      3V3_follower
                                                               │                  │ (+100 nF to GND)
                                                              ┌┴┐            ┌────┴────┐
                                                              │ │ R1         │74LVC1G17│
                                                              │ │ 33k        │         │
             100Ω           twisted pair        C1 1nF        └┬┘            │         │
  GPIO ─────[████]───────────────────────────────┤├───────────●────[████]───►│A       Y├──── GPIO
  (out)                                                       │      1k      └────┬────┘   (sync in)
                                                              ┌┴┐                 │
                                                              │ │ R2              │
                                                              │ │ 10k             │
                                                              └┬┘                 │
                                                               │                  │
  GND ───────────────────────────────────────────┤├───────────●───────────────────●────── GND_follower
             (pair return, ~1 V DC offset OK)   C2 100nF (C0G/film ≥50 V)
```

C1's follower-side pin connects to the bias node ●. That node joins R1, R2, and the 1 k resistor,
which feeds the optional Schmitt buffer's input, or the GPIO directly without the buffer. C2's
follower-side pin goes straight to follower GND. C1 and C2 aren't connected to each other. C2 is
the pulse's return path: joining it to the bias node would force the return current through R2 and
block the edge. R1 and R2 bias the idle node to ~0.77 V, and a 3.3 V edge through C1 rides on top
of that level.

Any GPIO works through the GPIO matrix, but boot behavior limits the choice of pins:

- Leader output: IO0 is acceptable. The strap is sampled only at the leader's own reset, and the
  AC-coupled wire can't pull it. Check that net for a BOOT button or debounce capacitor.
- Follower input: don't use a strapping pin (IO0, IO3, IO45, or IO46). The ~0.77 V bias and the
  pulse train can strap a rebooting follower into ROM download mode. U0RXD works well because it's
  silent at boot and isn't a strapping pin. It costs that device's UART serial-RX console; USB,
  telnet, and BLE still work.
- Don't use U0TXD on the leader. Its ROM boot output reaches a converting follower as a burst of
  spurious sync edges, faster than the period rate.

Choose and fit the components as follows:

- C1 couples the edge. C2 closes the high-frequency return loop while standing off the DC ground
  offset. For the pulse, C2 is in series with C1, so at 10 nF it's a 9 % series element rather than
  a negligible "ground bond". Use 100 nF (C_eff 0.99 nF), film or C0G, rated ≥50 V. Class-2 ceramic
  capacitance drops with bias voltage.
- With the buffer fitted, the node idles at the bare-divider value, 3.3·10/(33+10) ≈ 0.77 V. The
  internal pull-down that the sync-source config forces on (`src/pwm/mcpwm.h`) loads only the
  buffer output. Check the idle level and the pulse against the 74LVC1G17's V_T− and V_T+ at the
  follower's VCC (datasheet), not against the S3 pad's V_IL.
- Without the buffer, that pull-down is in parallel with R2 and the node idles lower. Check that
  level against the pad's V_IL.
- The node's Thevenin resistance is 33k‖10k ≈ 7.7 k. With C_eff 0.99 nF, τ ≈ 7.6 µs, so the level
  droops ≈ 12 % over a 1 µs pulse.
- The 74LVC1G17 Schmitt buffer, powered from the follower's 3V3, sits between the bias node and the
  GPIO and is optional. Without it, wire the 1 k resistor straight to the GPIO. The S3 pad has no
  input hysteresis, so a slow or ringing edge on this high-impedance node next to a switching stage
  can trigger more than once. A false edge disturbs the gates, as described under
  [Leader pulse and follower reload](#leader-pulse-and-follower-reload). Fit the buffer for long or
  noisy lines, or when the bench checklist shows extra edges.
- Route the pair away from the power stage. If the cable is shielded, connect the shield at one
  end only.

## Interaction with bsync

`bsync` and `sync_role=follower` are mutually exclusive at runtime, because the wire owns the
period. The `bsync` service refuses to start on a wired-sync follower. Run `bsync` on the leader,
or in wireless-only setups.

## Bench checklist

Ticked items have passed on the bench:

- [x] Scope the leader pulse: ~1 µs, every period, fixed relative to the leader's HS rising edge.
- [x] Confirm that the wire delivers: `wsync` on the follower reads the leader's rate (38.98 kHz),
      and 0.00 kHz with the leader's `sync_role` set to `none` (the diagnostic has been seen to
      fire).
- [ ] Confirm the follower locks: both switch nodes stay stationary relative to each other, with no
      beat.
- [ ] Measure the propagation delay (leader TEZ → follower reload) and subtract it with
      `sync_phase_ns`.
- [ ] Pull the wire: the follower free-runs and the beat returns. Reconnect it: the follower relocks
      within one period.
- [ ] Inject a 1 V DC offset between the grounds: the lock doesn't change, and no DC current flows
      in the pair.

`wsync` reports the running role (`wsync role=follower|leader`) before the count. A leader counts
its own outgoing pulse at exactly `pwm_freq`, the same as a locked follower, and `sync_role` is read
once at boot, so the conf file can disagree with what the driver is doing. The count accumulates
across the 16-bit PCNT wrap (`accum_count` plus a high-limit watch point), so a noise-multiplied
edge rate reads high instead of wrapping into a plausible healthy rate. `wsync` divides the count
by the measured window, not the requested one.

The host-side check is `FuguDevice.wsync_status()` in fugu-py. It's tri-state: it returns `None`
(unverified) instead of a pass whenever it can't establish the role, `pwm_freq`, or the count.

## Sync on a USB pad (no GPIO header)

On a board that breaks out only USB and I2C, the sync wire arrives on GPIO19 (USB D−), which is
also half of the USB-Serial-JTAG PHY. The pad can serve the PHY or the GPIO matrix, but not both,
so `drvInit` chooses at boot (`src/pwm/wsync_usb.h`). This path runs only when `pwm_sync_pin` is a
USB pin. Boards that use pin 44 or pin 0 take the unchanged path.

At boot, a follower on a USB pad goes through these steps:

1. If `usb_serial_jtag_is_connected()` reports SOF activity (not just VBUS), a host is present.
   The firmware leaves the pad to USB and runs without wired sync.
2. Otherwise, the firmware disables the PHY pad and qualifies the line. Two 20 ms PCNT windows
   must both be within ±10% of `pwm_freq` and agree with each other.
3. If the line qualifies, the firmware keeps the pad and arms the follower. Otherwise, it restores
   the pad and falls back to USB.

The probe handles two pad details:

- It uses a 25 ns PCNT glitch filter to match the MCPWM PIN filter. A wider qualification window
  would pass a line whose sub-window ringing then re-phases the timer.
- `pcnt_new_channel()` always enables the pad pull-up and disables the pull-down, which parks the
  AC-coupled node near mid-rail. The probe re-applies `GPIO_PULLDOWN_ONLY` immediately afterwards,
  as `initSyncIn()` does on the operational path.

On the reject path, the teardown order is fixed (`stop → disable → del_channel → del_unit → pad
enable`), so the pad never returns to the PHY while its pulls are still being changed.

`wsync` reports the outcome (`mode=usb (no sync edges)`, `mode=usb (host active)`, …). Without that
report, a USB fallback looks the same as a configured `sync_role=none`.

A leader on a USB pad runs only step 1. With no active host, it disables the PHY pad and drives
its pulse on D−. With a host, it keeps USB and runs unsynced. A leader has nothing to qualify, so it
keeps the pad for the whole run, and a host plugged in later gets no USB console until the next
boot. Reach that board over BLE or Wi-Fi.

### Two USB-only boards on one cable

If both boards use a USB pad, a C-to-C data cable carries the sync on D−. The receptacle joins A7
and B7, so either orientation works. A charge-only cable has no D− wire.

The cable bypasses the coupling circuit above: it connects the two GNDs and the two VBUS nets
directly, and the pulse is DC-coupled. Use a direct cable only when both converters share a ground
reference with no DC offset between the boards, and neither board can back-feed the other through
VBUS. Otherwise, put the coupling circuit on a USB-C breakout (D− = sync, GND through C2, VBUS and
CC left open) and connect the cable there.

### Re-arming after the leader comes up late

The firmware decides the pad at boot. A board that booted with USB attached, or before its leader
was running, therefore stays in USB mode for that whole run.

To retry, run `wsync arm` over BLE. It sets a one-shot NVS flag, and on the next boot the firmware
skips only the USB host pre-check and runs the probe anyway. A leader takes the pad
unconditionally. The flag doesn't skip qualification: with `sync_role=follower` and no leader on
the wire, the board still falls back to USB, because a follower armed against a dead
wire takes its first arbitrary-phase sync edge while the gates are already switching, and the HS
anomaly described above puts a double-length pulse into the half-bridge. `wsync arm off` clears a
pending flag.

The firmware reads, clears, and commits the flag in `setup()` before `converter.init()` runs. A
crash inside the probe therefore returns the board to automatic mode instead of repeating the
forced probe on every boot. A plain power cycle also returns it to automatic mode.
`KeyValueStorage::commit()` exists for this case: `writeString()` only stages the `nvs_set_str`,
and `esp_restart()` would discard it.

The USB-pad path was bench-validated on a USB-only board on 2026-08-19: OTA over BLE,
`pwm_sync_pin=19`, locked at 38.99–39.04 kHz across three windows against `pwm_freq=39000`, with
no ADC errors and a steady sampler. With the pin pointed at D+ (GPIO20, not wired), `wsync`
reported `mode=usb (no sync edges)` and the firmware restored USB.

## Switching-frequency changes

The firmware refuses a runtime switching-frequency change (`pwm-freq`) while wired sync is armed,
on a leader or on a follower that qualified its line. Wired sync depends on three settings that
have no re-arm path:

- The leader's pulse comparators are absolute ticks that `initSyncOut()` writes once.
- A follower's period is set `wsyncLeadTicks` short in `init()`.
- Both ends assume the same `pwm_freq`.

The refusal checks the effective mode, not `sync_role`. A board whose follower probe fell back to
USB has a free-running period, so the change is allowed.
