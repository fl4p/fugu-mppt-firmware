*this document is an LLM generated placeholder*

# Split dead-time: one value per transition (HL / LH)

## Motivation

The two dead-bands of a synchronous leg are physically different problems that today share
one knob (`board.conf::pwm_deadtime_ns`):

- **HL** (ctrl-off → rect-on, mid-period): implemented as hardware RED on the rect (LS)
  rising edge. With positive coil current the switch node commutates by itself at HS-off;
  the gap only needs to cover HS gate turn-off. Every excess ns is body-diode conduction
  (~0.8 V · I · t · fsw).
- **LH** (rect-off → ctrl-on, period wrap): implemented purely in software as
  `pwmMax = periodTicks − dtTicks`. The node sits on the body diode until HS hard-switches
  regardless; the gap only needs to cover LS turn-off. Excess here costs body-diode time
  *and* commandable duty span (max duty headroom in CCM).

The single knob forces `max(HL_need, LH_need)` on both edges. Since the two mechanisms are
already independent (RED register vs `pwmMax` reservation), splitting is cheap.

Naming follows the existing doc language (`HS->LS` / `LS->HS`) and the gate verifier's
`dt_hl` row. The names are **leg-relative** (genHS = ctrl output on `[0, hsOff]`, genLS =
rect): in boost topology the physical FETs swap but hl still means ctrl-off → rect-on.

## Config keys (board.conf)

| key                  | default            | meaning                                    |
|----------------------|--------------------|--------------------------------------------|
| `pwm_deadtime_ns`    | 0 (unchanged)      | common value, used for both edges          |
| `pwm_deadtime_hl_ns` | = `pwm_deadtime_ns`| HL override (hardware RED, realized −1 ct) |
| `pwm_deadtime_lh_ns` | = `pwm_deadtime_ns`| LH override (pwmMax reservation, exact)    |

No config migration: existing boards keep symmetric behavior. InEn boards ignore all
three (driver chip owns dead-time, both stay 0).

## Phase 1 — `MCPWM_SyncLeg` (src/pwm/mcpwm.h)

1. `dtTicks_` → `dtHlTicks_`, `dtLhTicks_`; `getDtTicks()` → `getDtHlTicks()` /
   `getDtLhTicks()` (rename fully, fix call sites — no alias).
2. `init(..., dtTicks, ...)` → `init(..., dtHlTicks, dtLhTicks, ...)`:
   - RED block (path-claim FED trick + `b_red`) runs iff `dtHlTicks > 0`.
   - `pwmMax = periodTicks − dtLhTicks` (unconditionally, also when hl == 0).
   - New legal state: `hl == 0 && lh > 0` — dt submodule stays bypassed (no FED claim, no
     1-tick HS falling delay), wrap band still reserved. InEn passes 0/0 as before.
3. `setDeadTimeTicks(dt)` → `setDeadTimeTicks(uint16_t hl, uint16_t lh)`:
   - hl retune requires the module armed at boot (`dtHlTicks_ != 0`), same
     `ESP_ERR_INVALID_STATE` refusal and same reasons (arming from bypass crosses the
     outputs). hl floor stays 2 (1 = no gap after the FED claim).
   - lh is a pure software update: `pwmMax = periodTicks − lh`. Out of scope for now:
     accepting lh-only retunes while hl is bypassed — keep the existing "armed module
     required" gate for the whole call, note the relaxation as follow-up.
   - Bounds: `hl < periodTicks`, `lh < periodTicks` (caller owns the /32 ceiling as today).
4. Update the header comment block (lines ~181–210) and the wrap-side safety comment: the
   LH band is `periodTicks − cmpLS` and depends on neither delay register — the
   applyPendingDeadTimeRt ordering argument survives with two values, re-derive it in the
   comment there.

## Phase 2 — `SynchronousConverter` (src/buck.h)

1. **Conf parse in `drvInit()`** (lines ~152–185): read the three keys, resolve
   `hlNs`/`lhNs`, then run the existing validation *per value*:
   - range guard (0..1e6, NaN) per value;
   - ceiling `period/32` per value;
   - the `==1 → bump to 2` fix and the "realized gap = X−1" 50 ns warning apply to **hl
     only**; lh has no −1 (gap is exact) but gets the same <50 ns warning without offset;
   - "quantized away to nothing" warning per value.
2. **Runtime request word**: `dtReqWord`/`dtAckWord` stay one 32-bit atomic, repacked as
   `seq[31:22] | hl[21:11] | lh[10:0]`. Fits: both values are producer-clamped to
   `period/32 ≤ 65535/32 = 2047` (11 bits). Consumer re-validates both against
   `periodTicks/32` as today.
3. **`requestDeadTimeNs(float hlNs, float lhNs, uint16_t &hlTicks, uint16_t &lhTicks)`**:
   - kMinGapNs (50 ns) floor: hl realized is `hl−1` ticks → `hl ≥ 1 + ceil(50ns)`;
     lh realized is exact → `lh ≥ ceil(50ns)`.
   - per-value `period/32` ceiling; the rectRefresh headroom check
     (`rectRefreshTicks + t > period/4`) uses **hl** (it is the LS-turn-on delay).
4. **`applyPendingDeadTimeRt()`**:
   - `newMax = periodTicks − lh`; `newRectMin = rectRefreshTicks + hl` (buck);
   - clamp/commit logic unchanged otherwise; driver call becomes
     `setDeadTimeTicks(hl, lh)`; log line prints both.
5. **Derived quantities** — audit every `drvDtTicks()` user:
   - `pwmRectMin = rectRefreshTicks + hl` (init() line ~830 and applyPending);
   - `pwmCtrlMax = driverPwmMax − pwmRectMin − 1` (driverPwmMax already carries lh);
   - `getPwmTickRate()` LEDC fallback `pwmFrequency * (driverPwmMax + dt)` uses **lh**
     (that is the value carved out of pwmMax);
   - `getDeadTimeNs()` → `getDeadTimeHlNs()` / `getDeadTimeLhNs()`;
   - `hasDeadTime()` semantics unchanged (module exists), `deadTimeTicksFor()` unchanged
     (pure ns→ticks).

## Phase 3 — Console (src/cli.cpp)

1. `dt` (cmdDeadTime):
   - no args → report both: `dt hl=%.0f ns (%u ct, gap %u ct) lh=%.0f ns (%u ct)
     pwmMax=%u minLS=%u maxHS=%u`;
   - `dt <ns>` → set both (back-compat);
   - `dt <hl> <lh>` → set independently;
   - the "lowering needs manual PWM" gate compares quantized ticks per value (lowering
     *either* edge requires manual PWM); bypass-at-boot refusal keys on hl == 0.
2. `pwm-dump` (line ~1918): `ls_on = ls_on_base + hl`.
3. Doc string comments referencing `pwm_deadtime_ns` persistence → mention the new keys.

## Phase 4 — Docs + editor metadata (one commit, per CLAUDE.md rule)

- `doc/Configuration.md` board.conf table: add both keys, note hl realized −1 tick and
  lh eating duty span.
- `doc/Console.md` `dt` row: two-arg form, new report format.
- `doc/mcpwm-sync-buck-driver.md` §dead-time (~lines 78–93, 158, 173): two values, the
  `hl==0 && lh>0` bypassed-but-reserved state.
- `etc/config-tool/conf-editor.html`: add `pwm_deadtime_hl_ns`/`pwm_deadtime_lh_ns` to
  META + FILE_KEYS **by hand** — do NOT run `scrape_conf_keys.py --write` (it clobbers
  hand-maintained entries).

## Phase 5 — Tests

On-target (`RUN_TESTS=1 idf.py -B build-tests build flash monitor`, bench S3):

1. Mechanical: update `McpwmLegRig.init()` and all `setDeadTimeTicks` call sites in
   `test/test_pwm.cpp` for the new signatures (symmetric values → existing assertions
   unchanged).
2. New: asymmetric init (e.g. hl=32, lh=80) — assert `pwmMax = periodTicks − 80`,
   measured HS→LS gap ≈ (32−1) ticks via the existing `analyse_deadbands()` CAP rig, and
   ls_to_hs.min_ticks > 0.
3. New: `hl=0, lh>0` boot — module bypassed (`setDeadTimeTicks` → INVALID_STATE), no
   1-tick HS fall delay, `pwmMax` still reserved.
4. Runtime: change hl and lh independently through `setDeadTimeTicks`, assert pwmMax
   tracks lh only and the measured gap tracks hl only.
5. Host: `etc/mcpwm_gate_verify.py` — add a `dt_lh` row (LS-fall → next HS-rise at wrap)
   next to `dt_hl`; extend `test/host_py/test_pwm_helpers.py` accordingly.

## Phase 6 — Validation

1. `idf.py build` (esp32s3) + `idf.py -B build-esp32 build` (LEDC path untouched but
   compiles the same headers).
2. On-target test suite on the bench S3 (fugu-esp32s3-*, Fugu2 board).
3. fbuck bench: boot with split values (e.g. `pwm_deadtime_hl_ns=200`,
   `pwm_deadtime_lh_ns=100`), verify with `dt`, `pwm-dump`, and the gate verifier /
   PicoScope on the switch node before any duty above minimum.
4. fry/flat: no action required — configs unchanged → identical symmetric behavior.
   Do not OTA until the bench validation above is green.

## Non-goals

- Arming the dt submodule from bypass at runtime (unsafe, unchanged).
- lh-only retune while hl is bypassed (trivial later, noted in setDeadTimeTicks).
- Adaptive/current-dependent dead-time.
- InEn and LEDC drivers (no dead-time module; unchanged).

## Estimated diff

~40 lines mcpwm.h, ~80 buck.h, ~30 cli.cpp, ~100 tests, plus docs/metadata.
