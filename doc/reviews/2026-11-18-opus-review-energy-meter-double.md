*this document is an LLM generated placeholder*

# Review: c9078ee4 "metering: accumulate lifetime energy in double"

Scope: `src/metering.h` (SolarEnergyMeter, DailyEnergyMeter::update, PersistentState, commit/similar),
`src/math/statmath.h` TrapezoidalIntegrator, RT call site `src/mppt.cpp:55`, telemetry `src/mppt.cpp:611-612`.
No device contacted, no tracked file modified. Scratch programs live in `/tmp/emeter-review/` (not in the repo).

## Verdict

The diagnosis is right and the fix is complete for the bug it targets. No blocking issue found. The commit has
one new low-severity side effect (a double that is not atomic is read across cores) and leaves a few pre-existing
items that are worth knowing about.

## Diagnosis, verified numerically

- f32 ulp at 16485.7 Wh = 0.001953 Wh (computed). Per-sample increment at 100 W × 2 ms = 5.6e-5 Wh. Round-to-nearest
  drops any increment below ulp/2 = 9.8e-4 Wh, so at 16.5 kWh **the float total only moves when one sample
  is worth more than 1758 W × 2 ms**. At 8466 Wh (ulp 9.8e-4) the limit is 879 W. Both live units ran at 100 to 500 W,
  so no sample ever registered. The conclusion holds for any sample period up to about 7 ms at 500 W, so it does
  not depend on the 2 ms figure being exact.
- The observed "~1 Wh/day" crawl fits occasional long-dt samples (stalls/gaps up to the 4 s maxDt), where a single
  increment exceeds ulp/2. *Suspected*: not reproduced against the device logs.
- Host simulation (real `TrapezoidalIntegrator`, 1 h, 2 ms, constant power; `/tmp/emeter-review/sim.cpp`):

| E0 (Wh) | P (W) | float dE (Wh) | double dE (Wh) | expected |
|---|---|---|---|---|
| 0 | 100 | 99.43 (99.4 %) | 100.00 | 100.00 |
| 512 | 100 | 109.88 (**110 %**) | 100.00 | 100.00 |
| 1024 | 100 | **0.00** | 100.00 | 100.00 |
| 1024 | 500 | 439.5 (88 %) | 500.05 | 500.00 |
| 4000 | 500 | 783.0 (**157 %**) | 500.05 | 500.00 |
| 8466 | 500 | **0.00** | 500.05 | 500.00 |
| 16485 | 100 | **0.00** | 100.00 | 100.00 |
| 16485 | 500 | **0.00** | 500.05 | 500.00 |

  (The +0.014/+0.07 Wh on every row is a sim artifact: the first `add()` with `lastTime = 0` integrates the first
  second. On the device the first sample comes more than 4 s after boot and is dropped.) In the old code E_today
  stayed at 0.0000 at 1024 Wh and above, which matches the field symptom. One year at 16485 Wh and 300 W in double:
  dE = 2,628,000.43 Wh against 2,628,000.00 expected, so double rounding is negligible.
- Besides stalling, the **old float total also over-counted** by up to +57 % when the increment lay just above ulp/2,
  because each sample was rounded up to a full ulp (512 Wh at 100 W, 4000 Wh at 500 W). The historical `E`/`E_today` series in
  InfluxDB is therefore wrong in both directions before the fix, not merely flat.

## Completeness of the fix (verified by reading)

| Item | State |
|---|---|
| Integrator accumulator `metering.h:210` | `TrapezoidalIntegrator<float, time_us, double>`; `value += (D)(float increment)`. The float increment has relative error ~1e-7 and no large-offset loss. OK |
| `_prevTotalEnergy` `metering.h:115` | double. OK |
| `update()` parameter `metering.h:163`, caller `metering.h:262` | double, no `(float)` cast left. OK |
| Threshold `metering.h:176` `prev + 1e-3f` | 1e-3f is promoted to double; the comparison runs in double. OK |
| `e` `metering.h:178` | `(float)(total - prev)`: the difference is taken in double first, so the cast is harmless. OK |
| `PersistentState::totalEnergy` `metering.h:200` | already `double` before this commit (checked `c9078ee4^`), so the file layout is unchanged and `static_assert(sizeof == 32)` still holds |
| `commit()` `metering.h:269` / `similar()` `metering.h:218` | double end-to-end; `std::abs(double)` < 5 Wh. No truncation |
| Restore `metering.h:237`, `statmath.h:295` | `restore(D)` takes the stored double unchanged. After the OTA the unit restores its stuck value (e.g. 16485.x) exactly and counts on from there. OK |
| Log formats `metering.h:182,190,239` | `%.2f` with a double argument is correct (varargs promote float anyway) |
| Telemetry `mppt.cpp:611` | `addField("E", double, 1)`: of the overloads, only `addField(const char*, float, int)` takes 3 args, so the double is narrowed to float unambiguously (text `%.1f`, binary F32). Fine, see below |
| Other readers of `totalEnergy` | none besides `mppt.cpp:611` (grep of `src/`, `etc/*.py`) |

### Telemetry `E` as f32 with 1 decimal

f32 ulp: 0.00195 Wh at 16485.7, 0.0078 at 100 kWh, 0.125 at 1.05 MWh. One decimal stays exact up to 2^18 Wh
(262 kWh, ulp 0.031). Above 2^19 (524 kWh) the ulp passes 0.05 and the last digit becomes quantised. At these units' yield
(a few kWh/day) that is more than a century away. **Fine.** `difference()`/`derivative()` queries on `E` keep
~0.002 Wh resolution per point at today's level.

### `E_today` jump on the first update after restore

None. `restore()` sets `_prevTotalEnergy = 0` (`metering.h:160`), and the first qualifying `update()` only primes
`prev = total` without adding (`if (_prevTotalEnergy > 0)` at `metering.h:177`). So the first update after an OTA
adds nothing for the jump from 0 to 16485. The cost is that energy between restore and the first update (≤ 3 s,
`lfPeriod`) is not counted, which is negligible. In the sim, E_today grew by exactly the post-priming delta.

### Day accumulator resolution (float `energyYield`)

`update()` runs every 3 s (`main.cpp:112` `lfPeriod`, `main.cpp:1293`), so each add is P × 3 s (0.0025 Wh at
3 W, 0.42 Wh at 500 W). That is far above the float ulp at daily totals (2.4e-4 Wh at 2-4 kWh). In a simulated 12 h half-sine day
(`/tmp/emeter-review/day.cpp`) E_today matched the double total delta to 1e-6 relative at 3, 30, 500 and
1500 W peak (e.g. 3819.6924 vs 3819.6947 Wh). **Fine.**

## Findings, ranked

1. **Low (new, analytical, not observed): the 64-bit `value` is written on core 1 and read without
   synchronisation on core 0.** `meter.add()` runs in `loopRT` (core 1); `totalEnergy.get()` is read by
   `SolarEnergyMeter::update()` (loopLF), `telemetry()` and `commit()` (rt_defer, core 0). The LX7 has no 64-bit store, so
   a reader can see one new and one old word (the old float was a single aligned 32-bit word, never torn). The error is
   bounded by one high-word step, 2^(e−21): **0.0156 Wh at 16485 Wh** and 0.0078 Wh at 8466 Wh (computed). Effects:
   telemetry E is off by ≤ 0.016 Wh for one point; `commit()` may persist a value off by ≤ 0.016 Wh; the day
   accumulator self-heals because it sums telescoping differences (a high read only delays the next increment). This
   needs no action. If wanted, reading through a seqlock or a 32-bit snapshot would remove it.
2. **Low (pre-existing, verified by grep): the lifetime total is persisted only at sweep start
   (`mppt.h:940`) and on supply-UV (`mppt.h:546`), rate-limited to 2 min and skipped if under 5 Wh (`metering.h:218-224`).**
   There is no commit on `restart`/OTA, so a reboot loses the energy since the last sweep commit (≤ 30 min of
   production, e.g. ~250 Wh at 500 W). Before this fix the loss was masked because the total did not move.
3. **Info (pre-existing data): InfluxDB `E`/`E_today` history before the OTA is unusable.** The total was flat above
   ~1 kWh at these power levels, and in the transitional ranges it was inflated by up to +57 % (table above). After the OTA, E
   continues from the stuck value, so the lifetime total undercounts by whatever was lost; nothing in firmware can
   recover it. Re-baselining E from the integrated `P` field in InfluxDB would be a separate decision.
4. **Info (tests): no regression test covers a large offset.** `test/test_meter.cpp:6` and
   `test/host-stub/integrator-test.cpp:16` still instantiate `<float, ..., float>` and start at 0, where float
   works. `test/test_statmath.cpp:112-127` uses double but small values. A test that does `restore(16485.0)` and
   then integrates 100 W × 2 ms for a few thousand samples would have caught this (float: dE = 0).
5. **Info (pre-existing, negligible): first sample after construction.** With `lastTime = 0`, an `add()` within
   4 s of `wallClockUs()` zero integrates a ramp from t = 0 (`statmath.h:282`). On the device the first meter sample
   comes later than that, so the sample is dropped.

## RT-path cost on core 1

Cross-compiled the integrator `add()` for esp32s3 (xtensa-esp-elf 15.2, -O2): the double version adds exactly two
libcalls over the float version, `__extendsfdf2` and `__adddf3` (both use `__floatundisf` for dt). Both live in
the S3 mask ROM (`esp_rom/esp32s3/ld/esp32s3.rom.libgcc.ld`: `__adddf3 = 0x40002184`,
`__extendsfdf2 = 0x400022a4`), so they cannot cause flash-cache stalls. libgcc bodies are 116 and 37 instructions
in total; the common add path is a few dozen instructions. **Estimate: ~60-150 cycles ≈ 0.25-0.6 µs at 240 MHz per
sample** (only when `power > 0.1 W`), against a ~2 ms sample period: < 0.03 % of the budget. *Estimated, not measured*:
`rtcount("mppt.update.meterAdd")` (`mppt.cpp:57`) on a device would confirm it. `CoulombCounter` already uses a
double-accumulating `TrapezoidalIntegrator<>`, so soft-double was already present in the firmware.

## Checks run

- `test/host-stub/integrator-test.cpp`: built with clang++ (host stubs) and passed (rc 0). It covers only the float
  instantiation from 0.
- `test/test_meter.cpp` and `test/test_statmath.cpp` are on-target Unity tests and were not run (no device).
- `/tmp/emeter-review/sim.cpp`: before/after accumulation table above, the one-year double run, f32 ulps and the torn-read bound.
- `/tmp/emeter-review/day.cpp`: day-accumulator resolution.
- `/tmp/emeter-review/rt.cpp`: xtensa-esp32s3 cross-compile and libcall listing.
- `metering.h` could not be host-compiled as a whole (`store.h` needs `esp32-hal.h`), so the day-meter logic in the sims
  is a verbatim copy of `DailyEnergyMeter::update`'s accumulation lines. The full firmware was not rebuilt; the
  overload resolution at `mppt.cpp:611` was checked by reading the three `addField` signatures
  (`tele/telemetry.h:57-59`, `tele/line_protocol.h:18-27`, `tele/sym_line_protocol.h:166-171`).
