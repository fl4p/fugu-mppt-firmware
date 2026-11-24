*this document is an LLM generated placeholder*

# Idle power: open items (handoff, 2026-11-18)

Handoff from the idle-power session (2026-11-15 to 11-18). Raw data and logs:
`doc/research/idle-power-sweeps/` (untracked). Background notes: `doc/dev-notes/rf-power.md`,
memory `project_heap_power_budget`, kb `esp32/realtime-and-power.md` and
`esp32/usb-console-and-recovery.md`.

## 1. BUG, fix first: a converter at the idle clock does not start at dawn

**Status: open. Mitigated on fry by config, latent on every board whose rate guard passes.**

Observed on fry, 2026-11-18, build `fry-brk1-608-ge4cb0616` (contains fd6ad79a), from
`pv/fugu_console.log` on havan:

- 2026-11-17 17:18:29 `idle profile on`, 17:18:36 `idle profile: cpu 80 MHz, sample rate 512/512`.
- 2026-11-18 07:15 to 07:21:32 `START blocked: Vin-Vout` (normal, Vin 25.5 V).
- 07:21:56 to 07:34:45: 17 lines `START blocked: idle profile` with Vin rising 33.6 V to 57.4 V,
  status `st=START,1`, 511 sps. No `idle profile off (start)` line, no `Start sweep`, no
  `Converter enabled`. About 13 minutes of sun not harvested.
- 07:34:45 another session sent `set-config board.conf idle_cpu_mhz 0` and `restart`; fry started
  normally after the reboot (07:35:12 `Converter enabled`).

flat did not hit it the same night: its guard tripped at dusk (`cpu stays 160 MHz, sample rate
1076/1478 at 80 MHz`), so the start inhibit was never held. flat's dawn was normal (lag at most
881 us in the first 80 minutes, below the 1000 us goal).

Likely cause (from code reading, NOT verified): the start inhibit `InhibitIdle` is released only
after the network loop sees the wake flag (`converter.takeStartWanted()` in `idleProfileTick`,
`src/main.cpp`). The flag is raised in two places: the RT start branch (`src/main.cpp` ~1354,
`if (blocked == kIdleBlock) converter.wantStart()`) and a refused `pwmPerturb()` (`src/buck.h`
~1381). With a sweep armed from the previous day (`st=START,1`, `mppt.active()` true), the RT loop
takes the `mppt.update()` branch instead of the start branch, and the start is presumably rejected
by `startBlockReason()` before `pwmPerturb()` is reached, so neither place raises the flag.

Why the bench test missed it: the dawn start was reproduced with `dc 0` then `pwr idle 1` then
`mppt`, which gives `st=START,0` (no armed sweep). Idle entry was tested in `st=START,1`, the exit
was not, because the mock's Vin cannot be flipped at runtime.

To do:

1. Reproduce on bench-139c in `st=START,1`: needs a runtime way to move the mock's Vin above and
   below Vout (add a `pwr`-style knob to the fake ADC, or a console command that scales the fake
   Vin), then: boot in START with Vin < Vout, wait for the profile (or `pwr idle 1`), raise Vin,
   expect `idle profile off (start)` and `Converter enabled`.
2. Fix so that ANY start refusal caused only by `InhibitIdle` raises the wake, in every RT path
   (start branch, armed sweep, PSU/PV-sim, manual, rect-cal, coil measurement). Simplest robust
   form: raise the wake inside `startBlockReason()`'s idle branch or wherever the idle bit is
   tested, not at call sites.
3. Add a backstop that does not depend on the flag: the network loop itself leaves the profile
   when the start condition without the idle bit is satisfied (e.g. `idleBlocker()` returns
   "(start)" when `startBlockReason()` is exactly the idle block).
4. Add a host or QEMU test for "armed sweep + idle clock + Vin rises".
5. Until fixed and tested: keep `board.conf::idle_cpu_mhz=0` on converters. fry has it since
   2026-11-18 07:34 UTC. flat still has `idle_cpu_mhz=80` and is safe only because its guard
   latches every night; setting it to 0 as well is the safe choice (needs a restart to apply).

### Fix (2026-11-18, uncommitted, host-tested and compiled only, not run on hardware)

Cause confirmed: with a sweep armed, `MpptController::update()` returned at its start gate
(`converter.disabled() && !startCondition()`) without raising the wake.

- `MpptController::startBlockReasonRt()` (`src/mppt.h`): `startBlockReason()` plus `converter.wantStart()`
  when the only blocker is the idle bit. Used by both RT start gates: the start branch in `loopRTNewData`
  (`st=START,0`) and the gate at the top of `update()` (`st=START,1`). `pwmPerturb()` keeps its own raise
  (manual duty, measure-coil, rect-cal, sweep enable).
- Backstop 1 (`idleProfileTick`): leaves the profile when `InhibitIdle` is set, the mode is not manual and
  `startBlockReason()` has been exactly `idle profile` for more than 1 s (`idleBackstopWake()` in
  `src/etc/idle_profile.h`). Log `idle profile off (start, backstop)`: that line means an RT path
  lacks the wake.
- Backstop 2 (`idleCpuTick`): `InhibitIdle` set in CPU state `boot` or `pre` is cleared, logged once as
  `idle profile: stray start inhibit cleared`.
- Bench knobs (`CONFIG_FUGU_WITH_PWR_KNOBS` only): `pwr vin <0..2>` scales the fake ADC channel feeding
  Vin, `pwr idle-after <5..3600>` shortens the 5 min idle time.
- Tests: `test/host-stub/mppt-sim.sh idle-wake` (armed sweep and no sweep: inhibit set, Vin rises, wake
  raised, start after release; fails at "START,1 wake raised" with the old gate), `idle-guard-test`
  (backstop timing). Still to do: the bench run on bench-139c (procedure in the session report), then
  `idle_cpu_mhz` back to 80 on fry.

### Bench result and second cause (2026-11-18)

Bench-139c, mock profile, `pwr vin` to move PV voltage. With the first version of the fix the start
in `st=START,1` came only through the backstop (`idle profile off (start, backstop)`, 1.1 s after
Vin rose), in the forced and the automatic path; `st=START,0` used the primary path. Cause: the idle
block reason was a `static constexpr const char *` compared by pointer, and with
`-fno-merge-constants` the literal has a different address per translation unit, so the comparison
in `mppt.cpp` never matched. `kIdleBlock` is now one array object defined in `mppt.cpp`. After that,
automatic path in `st=START,1`: `START blocked: idle profile` -> `idle profile off (start)` 0.2 s
later -> `start released, 302 samples in 200 ms at 160 MHz` -> `Converter enabled`.

Still open: not run on a converter. fry and flat have `idle_cpu_mhz=0`; setting it back to 80 on fry
needs an update to a main containing this fix and a watched dawn.

### Converter test in progress (fry, started 2026-11-18 10:45 UTC)

At Fab's request fry was updated to the fleet image `fry-brk1-685-gc9078ee4` (clean main with the fix,
`sdkconfig.fleet`) with `board.conf::idle_cpu_mhz=80`. flat runs the same image with `idle_cpu_mhz=0`.
Expected in fry's log: at dusk `idle profile: cpu 80 MHz, sample rate 512/512`; at dawn
`START blocked: idle profile` for well under a second, `idle profile off (start)`,
`start released, N samples in M ms at 160 MHz`, `Converter enabled`. `(start, backstop)` instead of
`(start)` means an RT path still misses the wake. **If `START blocked: idle profile` persists with Vin
above Vout: `set-config board.conf idle_cpu_mhz 0` and `restart` on fry.** Dawn on fry was about
07:21 UTC on 11-18. Result: not yet known.

## 2. Other open points of the idle profile

- **flat sample rate at 80 MHz**: 1076 of 1478 samples/s. Cause not measured; from code reading the
  three I2C register reads per INA226 sample are mostly CPU time in the Arduino/IDF driver.
  `board.conf::i2c_hs_freq` (direct register I2C) might make 80 MHz viable on flat; never validated
  while converting.
- **Guard coverage**: the rate guard's trip path is confirmed on flat (real) and on the mock at
  7000 samples/s. For the internal continuous ADC the counter only drops once the DMA pool
  overflows, so the guard does not see added batching latency there.
- **Not run**: profile blockers other than converter/start (PSU, manual duty, rect-cal, bsync, OTA
  hold, coil measurement, USB drive), the pending-BLE-client case, the conf error path for
  `idle_cpu_mhz`, the BLE telemetry stream at sparse rate, the binary telemetry flush path
  (compiled only), ESP-IDF 5.5 build (IDF 5.5 not installed), `etc/idle_switch_stall.py` after its
  exit-detection fix.
- **PSU predicate**: the older WiFi MAX_MODEM switch still treats a PSU delivering under 10 W as
  idle after 5 minutes. Left unchanged on purpose; decide whether to align it with the profile's
  predicate.
- **Night telemetry gaps**: one point per 15 minutes while idle (`tele.conf::idle_interval_ms`).
  Anything alerting on InfluxDB gaps must tolerate that. flat currently has the default again.
- **`pwr rt` cap** was raised to 10000 Hz for the guard test (knobs build only).
- Published docs: `website/docs/reference/config/board.md` describes the start-wait behaviour as
  working; revisit the wording once item 1 is fixed.

## 3. Measurements still owed

- **Battery-side current** (planned with Fab for 2026-11-16, not done): board input current at
  26 V as-is at night, in the idle profile, and with the MCU held in reset. Needed to turn die
  temperature deltas into mA and to check the deep-sleep estimate (0.15 to 0.18 W of about 0.25 W,
  desk study only).
- Die-temperature results so far (bench-139c, 1 C sensor steps): see memory note and
  `doc/research/idle-power-sweeps/`. fry in the profile: die 33 C to 28 C at 20 C board.
  flat with the CPU part latched off: 35 C to about 29 C.

## 4. Light sleep (experiment done, not a feature)

Bench result: with `sdkconfig.pm_ls` (PM + tickless idle +
`CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP`), WiFi MAX_MODEM and BLE advertising up, the chip
slept 70 % of a 240 s window and the die fell from 38.6 C to 26 C; telnet stayed connected.
DFS 80/40 MHz alone gave about 31 C. Before this can become firmware behaviour:

- every GPIO floats in light sleep (gate, `pwm_sd`, `bflow_sd`, LED): needs `gpio_sleep_sel_dis`
  or explicit sleep levels, verified on a scope;
- real boards must stop the MCPWM timer and the continuous ADC at night (both hold
  `ESP_PM_APB_FREQ_MAX`), so dawn detection needs a low-rate sampling mode;
- the network loop wakes at 1 kHz and the telemetry flush task at 50 Hz, mDNS at 10 Hz;
- USB-Serial-JTAG disappears from the host and needed a replug on macOS;
- `sleep_cpu_configure: Failed to enable CPU power down during light sleep` was logged, not
  investigated;
- item 1 shows how easily a night mode blocks the dawn start: any sleep mode needs a start path
  that does not depend on a single flag.

## 5. Deep sleep until PV exceeds battery voltage (desk study only)

Feasible on schematic revision f8de1ac: PV voltage is on GPIO4 (RTC-capable ADC pin), a 60 s timer
wake with an ADC check at the top of `setup()` is the simplest wake, and the UCC21330 and backflow
pull-ups keep the stage off. Costs: no MQTT/BLE/telemetry at night, charger termination and
coulomb-counter state lost each morning, cold boot sweep every dawn, an unconfirmed OTA image rolls
back, the flash-suspend probe erases a sector per wake unless deep-sleep wakes count as soft
resets, per-boot flash writes in the energy meter. The 12 V boost cannot be switched off (AP3012
SHDN tied to 3.3 V). Recommendation at the time: night profile first, then light sleep, deep sleep
as an opt-in last step. Needs the current measurement of item 3 before deciding.

## 6. State left behind

- **main**: a4aaa87e (pwr knobs), 488f3360 (idle profile), fa7685b7 (PM window, mock esp_timer
  source, `sdkconfig.pm_ls`), 26020240 (15 min idle telemetry, lone-point flush), 1604eaf8 (armed
  sweep no longer blocks entry), fd6ad79a (start waits for exit, rate guard; contains the bug of
  item 1).
- **flat**: updated by this session on 2026-11-17 09:25 UTC to 607-gfd6ad79a with
  `idle_cpu_mhz=80`, `idle_interval_ms=900000`; later updated by another session to
  608-ge4cb0616. Lock released.
- **fry**: never touched by this session. Now on 655-g1f40d4e7 with `idle_cpu_mhz=0`.
- **bench-139c** (`/dev/cu.usbmodem1101`, 192.168.1.125): knobs build of fd6ad79a plus the raised
  `pwr rt` cap, mock config with `sensor.conf::expected_hz=0`, lab WiFi stored, `tele.conf` enabled
  and pointed at this Mac (192.168.1.204), stock bootloader (no bootguard bootloader). Lock
  released.
- Build dirs in the main checkout: `build-pwr`, `build-pwr-off`, `build-pm`, `build-pm-ls`,
  `build-esp32-idle`.
- The two scheduled follow-up checks of this session never ran (they were session-only).
