---
name: project_wifi_connect_during_conversion_lag
description: "WiFi-on during conversion costs ~3ms loop lag (vs 50ms watchdog); safe, measured via reset-lag"
metadata: 
  node_type: memory
  type: project
  originSessionId: 2d219a47-1387-44f0-b5e3-4ad1a0a4d7b3
---

Measured (2026-05-30, bench mock on /dev/cu.usbmodem1301, vconv plant ~815W) whether enabling/reconnecting WiFi mid-conversion is safe, using the `reset-lag` console cmd + the status line's `lag=N㎲` (g_app.maxLoopLag, main.cpp:609 — only accrues while `!converter.disabled()`).

- Baseline converting, WiFi down: maxLoopLag ~0.7–0.8 ms.
- `wifi on` during 815W (the manual cmd bypasses the power gate, fires connect_wifi_async once): loop lag rose to a steady **~3.0 ms** and held; one earlier run showed 2 transient lag-print wraps (uint64 artifact, newlib-nano) at the connect instant. Either way **no stopAndBackoff / "No samples…shutdown" / ADC-error**, conversion uninterrupted.
- Cross-check: fry & flat (real converters) run in production with WiFi UP while converting at `lag=2878/3790㎲` — matches the bench ~3ms. Steady-state WiFi-up-during-conversion is proven safe.

Safety boundary is the no-sample watchdog **50000 µs** (`maxNoSamplesTimeoutUs`, → stopAndBackoff). 3 ms is ~6% of budget → wide margin on the mock. **Caveat:** mock has no real INA226/continuous-ADC ISR to starve; on real HW the failure mode is [[project_ina226_timeout_loop_latency_shutdown]] (alert-miss → loop-latency shutdown), which is exactly why firmware gates AUTO-reconnect at `mppt.tracker._curPower >= 10 && !disabled` (main.cpp:874, see [[project_wifi_reconnect_power_gated_daytime_stuck]]). Manual `wifi on` is NOT gated.

**dry_int follow-up (2026-05-30, same S3 unit, MAC 9c:13:9e:f4:04:98):** dry_int = `adc=esp32adc1` (REAL internal continuous ADC; vin/vout/iout/ntc physical, **iin virtual** — NO INA226, my earlier "needs INA226" was a misread). Provisioned cleanly (`. ./idf-export.sh` first so parttool.py is on PATH; `ESPPORT=… python3 ./provision.py lab/dry_int` — the `-p` flag breaks argparse). On the bench nothing is wired to the ADC pins → vin=0 → `st=UV`, converter never runs, so maxLoopLag stays 0 (lag only accrues while converting) — can't measure converting-lag here. But the meaningful probe for the real-ADC path is **sampler continuity**: across a full `wifi off → wifi on → wifi off 60` cycle the esp32adc1 sampler held **~744 sps (≈ dry_int expected_hz 730), 101% retention, ZERO ADC-stall / backoff / loop-latency / "ADC error" events**. So WiFi RF/PLL bring-up does NOT starve the real continuous ADC. Caveat: rssi stayed 0 (no matching AP), so this exercised the connect/RF transient, not a fully-associated link with traffic. New bench tool: `etc/scope_client/wifi_adc_continuity.py` (polls `status`, N-delta sps + alert regex). Device left on dry_int (idle/UV); restore the 815W bench sim with `provision.py lab/vconv_mock`.

Bench tool left at `etc/scope_client/wifi_lag_test.py` (untracked): drives reset-lag → wifi on, parses lag/rssi/power, flags watchdog trips. Uses fugu.transport.SerialTransport (read()=blocking readline, write(bytes); no flush_input/write_line; set `t.ser.timeout` after open; `.close()` is NotImplemented — harmless at exit).

(2026-11-14: wifi_adc_continuity.py / wifi_lag_test.py were deleted from etc/ in the dead-file cleanup; recover from git history if needed.)
