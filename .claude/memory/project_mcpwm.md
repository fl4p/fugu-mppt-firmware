---
name: project_mcpwm
description: MCPWM gate driver: IDF API traps (ACTION_END, force latch, CAP order, dead-time two calls, TEZ shadowing, PCNT pull-up), resolution audit, split dead-time, Kconfig driver choice, fry/flat migration
metadata:
  type: project
---

## Gate driver choice and fleet state
Since 1e5c022 (2026-10-09) the gate driver is build-time `choice FUGU_GATE_DRIVER`: `FUGU_GATE_LEDC` | `FUGU_WITH_MCPWM` (default; LEDC default only on classic ESP32) | `FUGU_WITH_VCONV`. `FUGU_WITH_LEDC` now only means the peripheral (fan, `anaw`); both drivers compiled = `#error`. `converter.conf::pwm_driver` is obsolete (WARN only, Fab chose warn over refuse). Reused sdkconfigs keep their old MCPWM bit.
- `etc/ota.py` (a2a46be) / `etc/ota_ble.py` (edac4a8) probe `pwm-freq` (no arg) and prompt across a driver change (`--allow-driver-change`); LEDC replies `pwm-freq: n/a`.
- flat MCPWM since 2026-11-08 18:48 UTC, fry since 11-09 09:35 UTC (OTA at ~217 W, lag 4.6-4.9 ms = LEDC). **Both MCPWM: build fry/flat with the default config.** Both HiLi, pins hi21/li14/sd47, dt=0 (driver chip interlock). Obsolete keys (pwm_driver, legacy rect_offset, flat boost/vout_max) deleted 11-12 via del-config; del-config at ~120 W peaked flat lag 62 ms, no trip.
- **Why:** Fab found the runtime key + LEDC/MCPWM/VCONV dependency web confusing.

## First live run (flat, 2026-06-08)
pwmMax 4103 (160 MHz) vs LEDC 2047 (80 MHz APB): +1 duty bit at 39 kHz; flat lag 76 ms -> 3 ms; validated to ~335 W CCM. Review fixes that shipped: F1 boot-safe (comparators 0 + `forceShutdown()` after `start()`), F2 `clearForce()` on re-enable after comparator write, `disable()` parks cmp=0, F3 interleave phase uses true `periodTicks`, F4 `dtTicks=0` for InEn.
- Exposed tracker 2048-array OOB -> [[project_crash_rootcauses]].
- `limits.conf` gotcha: `Limits` ctor asserts `Vin_max*Iin_max < P_max*4`; lowering `p_max` needs `iin_max` lowered too. A bad limits.conf crash-loops both OTA slots (shared littlefs) and `conf-check` swallows asserts; check by hand. `p_max` needs reboot; manual `dc` bypasses it.
- flat has no Iin sensor (virtual); use Vout·Iout for output power.
- Fab (EE): flat/fry gate driver chips have built-in interlock, cannot shoot through; dt=0 is right, `pwm_deadtime_ns` would only add loss.

## IDF API traps
1. **`MCPWM_GEN_*_EVENT_ACTION_END()`** is `{}` and trips `-Werror=missing-field-initializers`. Use the singular `mcpwm_generator_set_action_on_{timer,compare,brake}_event` (see `src/pwm/mcpwm.h`).
2. **Force level latches.** `set_force_level(gen, level, true)` overrides all actions across timer start/stop and reconfig until `set_force_level(gen, -1, true)`. Every `forceShutdown()` needs a `clearForce()` on the next enable (`buck.h::pwmPerturb`). Signature: gate flat at 0 V while `mcpwm_dump` counter runs and `gpiodump` routing is right.
3. **CAP ISRs enqueue out of order** (one ISR per channel; edges ~200 ns apart). Timestamps are right; insertion-sort the ring by `ts` with `(int32_t)(a.ts - b.ts) > 0`. Cross-period edge pairing must reset its "have prev" flag on each period boundary and every no-match branch. Needed for any multi-CapChan test.
4. **`set_dead_time` configures one path.** Always two calls, one per generator (bypass calls skip `swap_out_path` and undo the previous bypass). HS gets FED=1 tick to force non-bypass, LS gets RED=dt (`MCPWM_SyncLeg::init`, `::setDeadTimeTicks`). Consequences: realized HS->LS gap = `dtTicks-1` (dt=1 means none; floor 2, user-facing physical floor 50 ns); going bypass (dt=0) -> armed at runtime crosses the outputs between the two calls, so runtime dt is retune-only, refused when `dtTicks_==0`. Filed espressif/esp-idf#18654.
5. **Dead-time registers are not TEZ-shadowed by default** (`mcpwm_hal_operator_reset` -> update at once) even when comparators are. Set `update_dead_time_on_tez = 1` (+`_on_sync` for wsync followers). Codex round 4: the "shrinking RED fires a pending rise" mechanism is UNPROVEN (needs a controlled on-silicon test), so describe TEZ latching as a precaution. Topology bits (outswap, bypass) are live, not shadowed. Write order when dt changes: see Split dead-time below (an hl increase widens the LS span BEFORE arming the delay — a TEZ between the writes is shoot-through-safe but not bootstrap-safe; `src/buck.h` setDeadTime comment). The old blanket "dt first, clamp duty second" is superseded.
6. **Dead-time delays LS turn-on**: conduction = `pwmRect - dtTicks`. A runtime dt increase must raise `pwmRect` into `[newRectMin, newMax - pwmCtrl - 1]` in the same transition (not next tick). `pwmCtrlMax = pwmMax - pwmRectMin - 1` (cmpLS < period). `getPwmTickRate()` should return `resolution_hz` (fsw·periodTicks is 106 ppm off, 0.05 % on a wsync follower).
7. **`pcnt_new_channel()` forces pull-UP / disables pull-down** on the edge GPIO (`pulse_cnt.c:826-830`, IDF 5.5). The wsync AC-coupled receiver needs pull-down (0.64 V idle, R_PD 45k) or it reload-storms. Call `gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY)` right after; `buck.h` survives only because `initSyncIn()` runs later. See [[project_bsync_beacon_pwm_sync]].

## Resolution audit (2026-08-11, 452761c6)
After LEDC->MCPWM (pwmMax ~2047 -> ~4103) the only silent regression was `boot_refresh_ns`, now 2000 ns default (320 ct; Fab's measured fbuck floor is 300 ct). Also fixed: manual ramp step `±max(1, pwmMaxDriver/512)`, LS fade-in threshold `driverPwmMax/64`. PD gains, lock-in, fade rate, log thresholds already scale; `rect_offset_ns`, `pwm_deadtime_ns`, low-duty gates are ns-based. `pwmCtrlMin = 1` intentionally. **How to apply:** re-audit hardcoded counts on any future resolution change. Details in [[project_rectification]].

## LS min pulse on fbuck (2026-08-04)
"No LS gate signal" at ~0 A was the pwmRectMin keep-alive (80 ct ~500 ns) minus 200 ns dead-time = ~300 ns. First HW validation of the HiLi dtTicks>0 path-swap config (fry/flat run dt=0). Make it visible with `dc 1506 500`.

## Split dead-time (40b1676, 2026-08-26)
`board.conf pwm_deadtime_hl_ns` / `_lh_ns` (default `pwm_deadtime_ns`); hl = ctrl-off->rect-on (RED, realized hl-1), lh = rect-off->ctrl-on at wrap (`pwmMax = periodTicks - lh`, realized lh+1). Console `dt`, `dt <ns>`, `dt <hl> <lh>`. Plan `plans/split-deadtime.md` (settled; don't re-litigate naming). Codex fixes to keep: malformed per-edge value falls back to the common value, not 0; a runtime hl increase widens the LS span BEFORE arming the delay; the gate verifier's `dt_lh` row commands LS to max. **Open:** on-target Unity suite and fbuck split-value scope check not run (Fab waived for the fry/flat MCPWM move on 11-08; their configs are symmetric).
