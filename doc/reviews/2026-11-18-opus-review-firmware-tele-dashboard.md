*this document is an LLM generated placeholder*

# Review: metering / sweep-curve / tele-mqtt / vconv sun / sdkconfig.fleet (2026-11-18)

Commits: 6aa17297, 79ed4339, 63d6a4c7, 6a2fe183, 207c5bdc, 3dfe1c10, e7b4ea84, 1a028a5d (main).
Method: read the diffs plus the surrounding code, built and ran the host-stub tests, and compared
the fleet sdkconfig with the build dirs on disk. No tracked file was modified and no device was contacted.

## What was run

| Test | Result |
|---|---|
| `test/host-stub/tele-mqtt-test.cpp` (header clang++ line) | builds (2 format warnings in the stub's ESP_LOGI), **all 7 tests pass** |
| `test/host-stub/plot-test.cpp` with ASan | **238 checks, 0 failures** |
| `test/host-stub/vconv-test.cpp` + `src/sim/vconv.cpp` | **1343/1343 pass** (incl. 3 new Sun sections) |
| `test/host-stub/mqtt-service-test.cpp` | **all pass** (incl. `publish_binary_len`) |
| `test/test_meter.cpp::test_meter_persistent_state` | **not run**: on-target Unity only, no device was used |
| Host layout check of old/new `PersistentState` | old 32 B, new 32 B, `todayEnergy` @24, `todayPinMax` @28 |
| `build-fleet/` (built at 207c5bdc-dirty, so includes the `static_assert`) | image 1,689,808 B of a 1,871,872 B OTA slot (182 KB free); the xtensa build compiled the `sizeof == 32` assert |
| Target build with `FUGU_WITH_TELE_MQTT=y` | **not done**: it would need a new sdkconfig fragment, and this review could only create the report file |

## Findings, ranked

### 1. MEDIUM: the similar() fix lets `timeLastPower` go stale, so a reboot on a low-yield day splits the day (6aa17297; src/metering.h:214-221, 147)

The old `similar()` compared `todayEnergy` against `totalEnergy`. With a fleet-sized total that comparison is always "different", so every `commit()` wrote the file. Commits run at sweep start (src/mppt.h:923) and on supply-UV (src/mppt.h:529), with a 2-minute floor. Each write refreshed the stored `timeLastPower`. With the fix, a write needs |ΔE_total| ≥ 5 Wh, |ΔE_today| ≥ 5 Wh, a boot-count change, or (since 79ed4339) |ΔP_max| ≥ 5 W. `timeLastPower` is not compared at all.

Failure scenario: an overcast November day averages under ~10 W, so less than 5 Wh accrues between 30-minute re-sweeps. The stats file then keeps a `timeLastPower` several hours old even though power is flowing. An OTA or crash reboot at that point runs `DailyEnergyMeter::restore()`, and its `now - timeLastPower < 3600*3` check at metering.h:147 fails. Today's energy is archived to the daily ring as a separate "day", and `E_today` restarts from 0. Before the fix, `timeLastPower` was at most one sweep (~30 min) old.

- **Verified** by reading the code: the commit call sites, the rate limiting in `FlashValueStore::update`, and the restore condition.
- **Not reproduced** on target.
- **Fix:** add a bounded-staleness term to `similar()`, for example `&& std::abs(a.timeLastPower - b.timeLastPower) < 3600`. Writes stay below one per hour from this term, and staleness stays inside the 3 h window.
- **Side effect, intended and worth knowing:** up to 5 Wh of total and daily energy can now be lost on a power cut.

### 2. MEDIUM (only with `CONFIG_FUGU_WITH_TELE_MQTT=y`, which is off by default and not in sdkconfig.fleet): synchronous ~1 KB publish from the network loop (207c5bdc/3dfe1c10; src/tele/tele_mqtt.cpp:70-75, 104)

`flush()` calls `esp_mqtt_client_publish` with QoS 0 on `loopNetwork_task`, every 500 ms or whenever the batch passes 896 B. It can also fire from inside `teleMqttEnqueue`, that is, in the middle of `mppt.telemetry()`. esp-mqtt's QoS 0 publish takes the client API lock and writes to the socket synchronously. A message near 1 KB plus the topic also exceeds the default 1024 B out buffer, which sends it down the fragmented-write path.

Failure scenario: with weak Wi-Fi, a full TCP send window, or `mqtt_task` holding the lock, the write blocks for up to the transport timeout (esp-mqtt default network timeout is 10 s; **suspect**, from library knowledge, not measured). Everything else on `loopNetwork_task` stalls for that time:
- `process_queued_tasks()`: rt_defer ring overflow means dropped sweep-start `commit`/`begin` and dropped `plot`.
- `loopLF`: `meter.update`, the BMS-staleness handling, and the OTA-valid confirm.
- The console.

The RT protection on core 1 is unaffected.

- The `mqttLogCallback` mirror already publishes synchronously, so the pattern is not new. tele-mqtt adds a guaranteed periodic 1 KB load on top of it.
- **Before enabling this on fry/flat:** measure the worst-case publish time on a weak link, or lower `network.timeout_ms`, or skip the flush while the previous publish took longer than N ms.
- **Verified** by reading: the call path and the task.
- **Suspect:** the blocking duration.

### 3. LOW-MEDIUM: sdkconfig.fleet changes live-converter behaviour beyond what its commit message says (1a028a5d; sdkconfig.fleet:14,17,18)

The flag set is verified identical to `build-fry-main/sdkconfig` plus four options: `BLE_TELE`, `BLE_ADV`, `TELE_BINARY` and **`PWR_KNOBS`**. The commit message names only BLE telemetry and the advert broadcast.

- **`PWR_KNOBS`** adds the `pwr` experiment command. It is reachable over every console, MQTT included. It can stop the MCPWM timer (`pwr pwm 0`) and change the CPU frequency, PM settings and RT rate. It also adds an extra `timerOff()/timerResume()` branch to the gate-enable path (src/buck.h:439-441), and sets the network-loop delay from `g_pwrNetLoopMs`. This is an experiment tool on two real power converters. It should be a conscious choice, not a side effect of the fleet fragment.
- **`BLE_ADV`** is active by default. `tele.conf::adv_ms` defaults to 500 (src/tele/tele_adv.cpp:43), so after the OTA both converters start switching between connectable and non-connectable adverts and rewriting the adv data every 500 ms (adv interval 100-250 ms). The BLE link is the documented backup when Wi-Fi/MQTT dies, and fry's link is already weak. A reconciler bug here would take away that backup. It also adds radio duty, and an earlier note found BLE raised die temperature.
- **Recommendation:** either drop `PWR_KNOBS` from the fleet fragment or state it in the commit message. Provision `adv_ms=0` on fry/flat, or bench-soak `BLE_ADV` on an S3 Fugu2 first.

### 4. LOW: the sweep block is now held permanently, costing about 1.9 KB of steady-state heap (63d6a4c7; src/etc/plot.h:24-48)

`PlotPoints` is now 4 + 4 + 2·120·8 = 1928 B plus TLSF overhead. It is allocated at the first sweep and never freed. Before, a 3848 B block existed only between sweep start and plot. Peak heap during a sweep is therefore lower, but the steady-state free heap on fry/flat drops by about 1.9 KB permanently. That matters on boards running at 14-30 KB free, and the 11-14 heap incident on flat came from heap exhaustion. Acceptable, but count it in the heap budget.

### 5. LOW: `sweep-curve` returns wrong or empty results in some sweep states; cosmetic only (63d6a4c7; src/cli.cpp:495-518, src/mppt.h:1386-1412)

- **No-MPP sweep** (`_stopSweep` returns before `handOver()`, common at dawn/dusk), `abortSweep()`, or a trip mid-sweep: `collecting` stays set. `last()` then returns nullptr and `sweep-curve` prints `sc: none` until the next completed sweep. The previous good curve is gone too, because it lived in the same block.
- **Very short sweep:** if it ends before the deferred `begin()` runs, `begin()` arms a stale `collecting` after the sweep, with the same `none` result.
- **While sweeping:** `pl.block` is read directly. Before `begin()` has run, that is the previous, sorted sweep labelled `sweeping=1`. Afterwards, RT appends concurrently, so pairs can be torn. The data stays in bounds because `n` is snapshotted and the array is fixed-size.
- **No memory-safety issue:** the block is never freed, and `plot`, `begin` and the console all run on `loopNetwork_task` (MQTT `cmd_input` is deferred there, telnet and UART dispatch there).

### 6. LOW: tele-mqtt wire-mode and accounting edges (src/tele/tele_mqtt.cpp)

- **Wire mode:** the lease forces `g_teleBinary` globally (:59). The lease refuses to start while UDP runs text, but nothing stops the UDP service from starting *during* a lease (`svc on tele`, or a Wi-Fi-up self-heal after the tele service failed). `teleLoadWireConf()` then keeps binary because the lease is active, and a text UDP receiver gets binary datagrams until the lease ends (up to 1 h).
- **Lease length:** re-issuing `tele-mqtt N` sets the end to now+N, so it can shorten the lease. The header comment says "starts or extends". The test asserts the shortening, so the comment is what is wrong.
- **Dropped counter:** while MQTT is disconnected, frames from the UDP/BLE producers still go into `buf` and then count as `dropped` once it is full. The counter overstates real loss, and the buffer is discarded on reconnect anyway.
- **Publish failure while connected:** that drops a batch which may carry the table burst. Recovery comes at the next heartbeat (≤ 200 points or 120 s) or new-symbol burst. That is acceptable, but no test covers it.

### 7. LOW: vconv sun numeric corners, simulator only (e7b4ea84)

- **Seed conversion:** `vconv set sun_seed <v>` does `(uint32_t) std::fabs(v)` (src/cli.cpp:2131). For v ≥ 2^32 that is undefined behaviour; it saturates on Xtensa. Clamp it.
- **Float clock:** `SunModel::t` is a float that advances by 0.05 s steps (vconv.h:32). Above about 2.1e6 s the ulp is 0.125, so `t += 0.05` no longer advances and the "day" freezes. Above about 1e6 s the steps are quantised to 0.0625 s. This only applies to `sun_day_s + night` above about 1e6. Real-day values (86 400 s, ulp about 0.008) are fine. The cloud test uses `setDay(1e6)` but only reaches t ≤ 72 000.
- **Cross-core races:** the console setters and `sun.reset()` write `SunModel` fields from core 0 while the simulator steps on the RT side. The result is a one-step glitch at worst, the same pattern as the other vconv setters.

### 8. LOW, pre-existing (not introduced here, noted while reading)

- **Daily ring double-store:** after the day-end branch archives a day, the stats file keeps `todayEnergy`, because nothing commits at night. A reboot within 3 h of the last power continues that day, and the next day-end archives it again.
- **Torn `wallClockMs()` reads on core 0:** this is a 64-bit value written by core 1. The lease end can be off by about 4295 s on a torn read, and other core-0 users are affected the same way.

## No finding (checked)

- **`PersistentState` layout:** 32 B in both versions on host and xtensa (the fleet build compiled the assert). `todayPinMax` occupies the old tail padding at @28. Garbage from old images is 0 or 0xFF (NaN), and `sanePinMax()` maps both to 0. Values in (0, 1e5) would pass, which is acceptable. A downgraded image reads only the fields it knows. The file size is unchanged, so `load()` accepts it.
- **Flash-write frequency:** not increased. Commit call sites are unchanged, the 2-minute floor still applies, and the fix reduces writes. The new P_max term can only add writes at existing commit points.
- **P_max_today:** written by `update()` on core 0 (loopLF) and read by `telemetry()`/`commit()` on core 0. Aligned 32-bit floats.
- **`sc:` buffer bounds:** a flush happens whenever `len > 140`, so every append starts with at least 52 bytes left in the 192 B buffer. `snprintf` truncates safely, and the over-long return value is used only for the flush test, never as an offset. NaN/Inf samples are now rejected in `Plot::add`.
- **Plot handoff:** the atomics are correct. RT never allocates, and `plot()`/`begin()` run in FIFO order on the same deferred queue.
- **Binary-safe `publish` overload:** correct. `alert.cpp`'s 4-argument call still binds to the text overload, and no caller passes a null payload to the new `strlen` path.
- **Symbol table:** fully resent (`forceResend`) on lease start and on every reconnect, and the buffer is cleared on disconnect. 3dfe1c10 ships the batch before an oversized frame instead of dropping it, and a test covers that.
- **Lease logic:** refusals, expiry, `seconds*1000` (≤ 3.6e6, no overflow), the 3600 s cap, MQTT stop leading to off, the 16 KB heap gate and bad_alloc handling, and freeing the buffer on stop are all correct. With the Kconfig option off, the `mppt.cpp` and `tele_core.cpp` hooks compile to inline no-ops.
- **Sun model:**
  - G→0 is floored to Isc 1e-4 and Voc 0.5, and `PvModel` stays finite (tested).
  - NaN config values fall through `fmax`/`fmin` to safe values (day off, rate 0, edge 0, len 1).
  - A tiny `sun_day_s` is clamped to 1 s, seed 0 maps to 1, and `log(1-u)` is safe because u < 1.
  - `cloud` stays within [1-depth, 1].
  - The CLI rejects non-finite values.
- **Fleet image size:** 182 KB of headroom in the OTA slot.

## Do the tests prove what the commits claim?

- **tele-mqtt-test** covers refusals, the record format (table then data frames), expiry, lease 0, MQTT stop, the resend on reconnect, the cap and the oversized-frame flush. It does not cover publish failure, the cap-full drop, or producers running while disconnected.
- **plot-test** gains `block_kept_after_plot`: the block is kept, sorted and reused, and is hidden from `last()` while collecting. It does not cover the no-MPP path where `handOver()` is never called, or `cmdSweepCurve` itself.
- **vconv-test** covers the clear day, wrap-around, determinism per seed, the renewal mean of the cloud dip, the night floors, a base change while dark, and Voc at G = 0.3.
- **test_meter_persistent_state** checks the file round trip and the 0xFF tail. It does not exercise `SolarEnergyMeter::load()`/`restore()` with `pinMax`, and nothing tests the corrected `similar()` lambda, which is the subject of finding 1. It was not run, because it needs an on-target run.
