Topic notes (compacted 2026-11-24; each holds several dated sections).

## Feedback
- [Device handling](feedback_device_handling.md) — lock.sh list before fry/flat writes, read-only while diagnosing, ADC error = unhealthy, BLE when silent
- [Git & writing rules](feedback_workflow_git.md) — commit --author Fabian, always main, push by captured sha, no obvious comments
- [Agent orchestration](feedback_agent_orchestration.md) — Opus subagents in background, Paseo not tiles, codex exec < /dev/null, offload

## Fleet behaviour (fry/flat)
- [Charger termination](project_charger_termination.md) — float key is `cv_float`; charger.conf needs `restart`; CV hold + top-balance live
- [BMS feed + flat Vout error](project_charger_bms_feed.md) — flat reads 0.3–0.9 V low while converting (no Vout RC); don't trim vout_rl
- [Tracker + sweep](project_tracker_sweep.md) — biggest loss = wrong local peak until next sweep; rev_k rejected, #70 deferred
- [Backflow trip + wire guard #47](project_backflow_wire_guard.md) — join-at-float highD trip accepted; wire_protect warn only, RTC pinned
- [Ripple filtering](project_ripple_filter.md) — notch before median; adaptive notch live; comb on main, fry trial suspended by 1009 OTA
- [RT loop latency](project_rt_loop_latency.md) — flash auto-suspend + IRAM set brought flat <1 ms (closed 11-18)
- [Heap + idle power](project_heap_power_budget.md) — BLE ~110 KB heap; telemetry interval is the biggest idle-power lever

## Power stage
- [MCPWM gate driver](project_mcpwm.md) — set_dead_time once per generator; fry+flat on MCPWM since 11-09
- [Sync-rect timing](project_rectification.md) — fry rect_offset -325 ns via rect-cal; flat stays 0 until Vout sensing fixed
- [Coil L + DCM ringing](project_coil_and_ringing.md) — fry ~76-80 µH, flat ~51 µH (different coils); LS-pulse damping can't work
- [PSU + PV-sim modes](project_psu_pv_modes.md) — vconv-validated; real-HW dummy-load check pending, don't OTA to fry/flat
- [INA226](project_ina226.md) — alert path (CVRF re-arm, MASK_EN read load-bearing), latency shutdowns, bus-OV knob
- [bsync beacon PWM sync](project_bsync_beacon_pwm_sync.md) — two-device scope-validated (σ 0.09–0.12 µs, 1 µs p-p)

## Firmware bugs + internals
- [Crash root causes](project_crash_rootcauses.md) — bad_alloc aborts, telnet/mDNS UAFs, IPC deadlock, tracker OOB, BLE assert
- [ADC sampler](project_adc_sampler.md) — watchdog must not gate read(); TaskNotify pdTRUE; IDF needs uniform atten
- [Logging stack budget](project_logging_stack_budget.md) — vprintf_mux runs on caller's stack: sys_evt/wifi/ping overflows
- [vprintf_mux bugs](project_vprintf_mux.md) — spent va_list, static loc_buf race, vsnprintf truncation OOB
- [WiFi reconnect](project_wifi_reconnect.md) — midday reboot leaves 0.0.0.0 (reconnect only <10 W); AP power-off panic open
- [BLE NUS console](project_ble_console.md) — TX backpressure, stale macOS GATT cache (ATT 3), UTF-8 glyph split
- [Telemetry pipeline](project_telemetry.md) — mppt in tm.fabi.me/ha_van via havan relay; binary=1 + tamp on fry/flat
- [Own FTP + HTTP conf server](project_own_ftp_http_server_1123.md) — 8c0f7e02 HW-tested on bench; ~20 KB heap peak
- [USB config drive (#40)](project_usb_msc_config_drive.md) — CONFIG_FUGU_WITH_USB_MSC: FAT over conf/, commit on eject

## OTA, access, tooling
- [OTA lessons](project_ota.md) — check ota-owners.jsonl + bootinfo first; fry+flat on 1009-g44e16128 (11-24 15:30); BLE verb ota-ble
- [Coredump + peek](project_coredump.md) — 64K partition needs serial flash; esp-coredump --core-format raw
- [Reaching fry/flat](reference_device_access.md) — NAT ports flip, check hostname; MQTT console pv/log/<host>/cmd; creds in env files
- [NAT router .231](project_nat_router.md) — OTA through it can lock it up; liveness WDT reboots after ~3 min dead upstream
- [Build toolchain traps](project_build_toolchain.md) — newlib-nano: no %hh/%ll; IDF 6.0.3 only; perf.h single-TU; conf-editor
- [vconv builds](project_vconv_builds.md) — S3/classic recipes, 0 W OTA hazard, MPPT fragility mechanism
- [Classic ESP32 target](project_esp32_classic_target.md) — IRAM overlay, dead console RX, buck-test UART hijack
- [Wokwi](project_wokwi.md) — BLE off, adc=fake; telnet forward RSTs after 1-2 cmds; CI token location
- [Bench rigs + lab tools](reference_bench_rigs.md) — 11-24: only bench139c + fry/flat here; fbuck/fboost absent; PicoScope, USB-JTAG

## Plans + research
- [STM32WB09KE port](project_stm32wb09_port.md) — branch stm32wb09-port; needs arm-none-eabi+CubeWB0
- [External comms port plan](project_ext_comms_port_plan.md) — VE.Direct-pinout port + CAN/RS-485 adapters; HW plan in Fugu2 repo
- [LFP longevity research](reference_lfp_longevity_research.md) — doc/LFP Longevity Research.md + kb note
