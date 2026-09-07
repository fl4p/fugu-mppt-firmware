*this document is an LLM generated placeholder*

# Codex review of e246ea8 -- issue #65 stale-as-fresh telemetry

Run 2026-09-07, `codex exec` with network + headed Chrome (Playwright MCP over CDP).
Access verified against the launching session's own git log and browser.

## ACCESS

Live shell query, `git log --oneline -3`:

```text
e246ea8 lf: decouple control scheduling from console print pacing, mark stale temperatures
818ad1d ota_ble: a device that does not come back is a FAILURE again
52914a3 ota_ble: use the shared esp-ota-ble host module; abort unconditionally on disconnect
```

Playwright fetched the requested ESP-IDF page. Its `<title>` was:

```text
Temperature Sensor - ESP32-S3 - — ESP-IDF Programming Guide v5.5 documentation
```

## FINDINGS

1. **High — (c) A failed read renews the freshness of an old cached value.**

   [`SingleValueSensor::read()`](src/adc/temperature.h:22) stamps `_lastReadUs` before knowing whether `readImpl()` produced a measurement. If `temperature_sensor_get_celsius()` fails, [`Esp32TempSensor::readImpl()`](src/adc/temperature.h:198) returns `NAN` without clearing its EWMA, while [`lastFresh()`](src/adc/temperature.h:32) subsequently returns the old finite `last()` value because the timestamp is new. The NTC has the same shape: [`EWMA::add()` ignores NaN](src/math/statmath.h:81), preserving its previous finite cache.

   Thus a sensor which worked once and then fails on every attempt can remain “fresh” forever. That recreates stale-as-fresh telemetry and stale OTP inputs through a different failure mechanism. Freshness must be committed with the cache only after a successful update, or a failed read must invalidate the cache.

2. **High — (c) Staleness fails open for the running converter’s OTP and Wi-Fi cutoff.**

   [`protectLf()`](src/mppt.h:552) uses `stale_NaN > Temp_max`, which is false, so stale sensors do not stop a running converter. [`wifiShutdownIfHot()`](src/main.cpp:710) similarly does nothing for NaN. Automatic MPPT normally falls back to a 25% power cap at [`src/mppt.cpp:63`](src/mppt.cpp:63), but manual mode calls `updateManual()` instead of the thermal-derating path at [`src/main.cpp:1046`](src/main.cpp:1046). It can therefore continue without either derate or stale-temperature shutdown.

   Conversely, [`startBlockReason()`](src/mppt.h:508) blocks a start when MCU temperature is NaN. Mechanically, this matches the previous policies—optional NTC NaN allowed, MCU NaN blocks start, runtime NaN ignored—but those policies are not safety-consistent when NaN now also means “the mandatory refresh task stopped.” A running converter needs an explicit stale policy, such as stop/backoff or at least full fan plus a hard power limit.

3. **Medium — (c) `_lastReadUs` is a new cross-core data race and can tear.**

   `_lastReadUs` is a plain 64-bit field at [`src/adc/temperature.h:11`](src/adc/temperature.h:11). The repository itself documents that 64-bit accesses are not atomic on Xtensa at [`src/charger.h:103`](src/charger.h:103). Compiling a minimal access with this checkout’s Xtensa GCC 14.2 emitted two `l32i` instructions for a read and two `s32i` instructions for a write.

   At the 32-bit-low-word rollover every 4,294.967296 seconds—about 71m35s—a torn value can appear roughly 71 minutes old, returning NaN, or roughly 71 minutes in the future, incorrectly returning fresh. Practically this will usually affect only one or a few RT iterations, briefly applying the 25% cap or blocking a start, but in C++ it is formally undefined behavior at every concurrent access.

   The EWMA floats also have C++ data races, but aligned floats compile as single 32-bit loads/stores and cannot tear at the hardware level. Those float races predate this commit; the 64-bit timestamp race is new.

4. **Medium — (c, pre-existing but widened) The temperature maximum calculation loses a valid die temperature when NTC is NaN.**

   At [`src/mppt.cpp:51`](src/mppt.cpp:51), `ntcTemp` begins as NaN and `ucTemp > ntcTemp` is false. A valid, hot MCU temperature therefore cannot drive the fan or thermal derate when the NTC is missing, failed, or uniquely stale. The fan receives NaN and all comparisons in [`fanUpdateTemp()`](src/cooling.h:72) are false, leaving its prior duty unchanged.

   This bug existed with raw `last()`, but `lastFresh()` adds more ways for NTC to become NaN. A NaN-aware maximum such as `fmaxf()` would retain either valid sensor and produce NaN only when neither is usable.

5. **Medium — (c) The 20-second margin is reachable during supported core-0 operations.**

   The scheduler check happens after console processing, networking, and all service ticks at [`src/main.cpp:1192`](src/main.cpp:1192). Legitimate blockers include:

   - `sleep`, explicitly allowed for 60 seconds at [`src/cli.cpp:1827`](src/cli.cpp:1827).
   - FTP passive connection setup, which can wait twice for 30 seconds at [`FtpServer.cpp:1344`](components/SimpleFTPServer/SimpleFTPServer/FtpServer.cpp:1344).
   - HTTP OTA, bounded by a two-minute watchdog, although that path first stops conversion and halts ADC sampling at [`src/cli.cpp:639`](src/cli.cpp:639).

   Ordinary Wi-Fi reconnect is bounded around six seconds, scope waits one tick, and MQTT connection work is asynchronous. ESP-IDF 5.5’s temperature read is a register read with a 300 µs delay only when changing measurement range—not a plausible 20-second block ([driver](https://github.com/espressif/esp-idf/blob/v5.5/components/esp_driver_tsens/src/temperature_sensor.c#L302-L318), [raw-read implementation](https://github.com/espressif/esp-idf/blob/v5.5/components/esp_hw_support/sar_periph_ctrl_common.c#L108-L166)).

   The cache really is stale during these windows, but the sensor can be perfectly healthy. Automatic conversion drops to 25%; manual conversion remains running without stale OTP; a disabled converter cannot auto-start until core 0 resumes and refreshes.

6. **Low — (c) Print pacing is not exactly preserved, and telnet no longer forces an immediate line.**

   Telnet still zeroes `lastTimeOutUs` at [`src/tele/telnet_service.cpp:98`](src/tele/telnet_service.cpp:98), but that value is now tested only inside `loopLF()`. The outer scheduler at [`src/main.cpp:1204`](src/main.cpp:1204) ignores it, so connection can wait almost three seconds for the next LF tick. Previously zero caused the outer gate to enter immediately.

   More generally, after arbitrary console input the 3-second live-converter print deadline is sampled only on fixed 3-second work ticks. The automatic line therefore occurs 3–<6 seconds later rather than approximately three seconds later. With six-second polling the number of lines is normally unchanged, but their spacing can cluster around the next poll.

7. **Low — (b) `lastTimeOutUs` is not strictly set only after output.**

   [`loopLF()`](src/main.cpp:993) assigns it after calling `lfStatusLine()`, but that function may return without printing when sensors are unavailable or a coil measurement is active at [`src/main.cpp:839`](src/main.cpp:839). Console input also intentionally sets it without printing at [`src/console.cpp:43`](src/console.cpp:43). The assumption holds on the ordinary valid-sensor/non-measurement automatic path, but not universally.

8. **Medium — (c) Three raw-cache consumers remain, including one safety permission.**

   - [`wifiLoop(mppt.ucTemp.last() < 80)`](src/main.cpp:1151) permits Wi-Fi reconnection using stale-low temperature. Because it executes before `lfControl()`, recovery from a long stall can add another six-second reconnect wait before temperature is refreshed.
   - The START-block diagnostic prints raw old temperatures at [`src/main.cpp:887`](src/main.cpp:887), potentially immediately after the main status line printed `--`.
   - The LCD still displays raw NTC at [`src/viz/lcd_service.h:45`](src/viz/lcd_service.h:45).

   The LCD omission is merely misleading. Leaving the `<80` connection gate on raw data is not defensible as a safety decision; stale/NaN should fail that permission closed.

No category **(a) arithmetic error** was found.

## SURVIVED

- **Issue #65’s polling acceptance test passes by code trace.** Successful commands still call the interim snapshot at [`src/cli.cpp:2135`](src/cli.cpp:2135), but this no longer changes `lastLfWorkUs`. The non-interim path consequently reaches `lfControl()` every approximately three seconds regardless of `dc 0`, `dc 1`, or one-/six-second status polling. Both temperatures are refreshed at [`src/main.cpp:817`](src/main.cpp:817). The values therefore track in roughly three-second/filter-smoothed steps rather than freezing. Hardware confirmation remains unverified.

- **The scheduler decoupling fixes the main starvation.** Watchdogs, charger update, meter update, OTA validation, LED work, and temperature reads are no longer gated by console quiet time.

- **NaN publication behavior is sound once staleness is detected.** Text and binary telemetry omit NaN fields; BLE advertising converts NaN to its `-128` sentinel at [`src/tele/tele_adv.cpp:117`](src/tele/tele_adv.cpp:117); the main status line prints `--`.

- **The new three-second idle read cadence has no material power or RT regression.** The die sensor is installed and continuously enabled in [`begin()`](src/adc/temperature.h:192); reads do not power-cycle it. The normal IDF 5.5 path is short, and the potentially 300 µs range-change delay already occurred every three seconds while converting. Idle changes only call frequency.

- **SPS and byte-rate arithmetic survives.** `lastWindowUs`, `lastNSamples`, and `bytesSent` now represent consecutive approximately three-second windows. The byte formula remains the correct decimal kB/s conversion. An idle line now reports the latest three-second window rather than a 24-second average, but it is not mathematically wrong. Interim SPS intentionally uses the last completed window.

- **The watchdog’s newly reachable shutdown is intentional.** Under a fast poller it can now reach `stopAndBackoff(4)` after three consecutive low-SPS windows at [`src/main.cpp:727`](src/main.cpp:727). Previously it never ran at all. The `dt > 0.9*lfPeriod` condition remains true for normal full windows; it is a window-validity gate, not protection against the newly restored execution.

- **All four implementations were converted correctly by inspection.** `TempSensorGPIO_NTC`, S3/IDF5, S3/IDF<5, and non-S3 define `readImpl()` and override `last()`. There are no other subclasses or polymorphic `read()` callers. The S3 scope registration and sampling both use the same `this` identity; S3 already had the base/vtable, while the newly polymorphic non-S3 branch contains no scope calls. The known classic-target build failure was not rechecked.

- **The two-buffer status formatting is safe.** Argument order is irrelevant because the lambda writes distinct arrays. `snprintf(..., 8, ...)` is bounded and sufficient for valid physical temperature ranges, NaN, infinity, and `--`; pathological oversized finite conversions would be truncated, not overflowed.

Source access log:

- [Issue #65](https://github.com/fl4p/fugu-mppt-firmware/issues/65) — inspected in full via authenticated `gh issue view` and title-validated through Playwright; primary raw text + rendered DOM; load-bearing.
- [ESP-IDF 5.5 temperature-sensor documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/peripherals/temp_sensor.html) — inspected relevant application, API, and thread-safety sections through Playwright; rendered DOM validated; load-bearing.
- [ESP-IDF 5.5 temperature_sensor.c](https://github.com/espressif/esp-idf/blob/v5.5/components/esp_driver_tsens/src/temperature_sensor.c) and [sar_periph_ctrl_common.c](https://github.com/espressif/esp-idf/blob/v5.5/components/esp_hw_support/sar_periph_ctrl_common.c#L108-L166) — inspected relevant implementations in rendered GitHub DOM and the matching local IDF checkout; load-bearing.
- `https://www.espressif.com/en/products/socs/esp32-s3` — Playwright returned a 502 error page; de-scoped because no conclusion depends on it.
- Search record: repository/issue seeds plus Serper queries for the IDF driver source and a contrary “64-bit access cannot tear” claim; snippets were used only for discovery. No external contrary primary displaced the direct Xtensa GCC assembly result.

