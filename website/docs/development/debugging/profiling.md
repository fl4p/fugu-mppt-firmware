---
title: Profiling
sidebar_position: 6
description: "You can profile the ESP32 with instrumented or sampling methods."
---

# Profiling on ESP32

*Original hand-written note: [doc/dev-notes/performance profiling.md](https://github.com/fl4p/fugu-mppt-firmware/blob/main/doc/dev-notes/performance%20profiling.md).*

You can profile the ESP32 with instrumented or sampling methods. The main options are:

* SEGGER SystemView, over ESP-IDF App Trace and JTAG, is the method Espressif documents. Of the options here, it comes
  closest to a full profiler on ESP-IDF. Enable `CONFIG_APPTRACE_SV_ENABLE=y`, attach a JTAG probe (the built-in USB-JTAG
  on the S3 works), and open SystemView on the host. You get task and ISR timelines, context-switch traces, and optional
  user markers. See the ESP-IDF Application Level Tracing Library and
  [SystemView Tracing](https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/api-guides/app_trace.html#app-trace-system-behaviour-analysis-with-segger-systemview).
  * Tracealyzer (Percepio) is commercial. It consumes the SystemView protocol and has a nicer UI. It uses the same data
    path as SystemView.
* GDB sampling uses openocd and `xtensa-esp32-elf-gdb`, with a periodic `bt` as a poor-man's sampling profiler.
* FreeRTOS runtime stats come from the per-task run-time counters (`vTaskGetRunTimeStats()`, `uxTaskGetSystemState()`,
  `vTaskGetInfo()`). They need `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y` and `CONFIG_FREERTOS_USE_TRACE_FACILITY=y`,
  but no hardware. The `rt-stats` command reads them a few tasks at a time with `vTaskGetInfo()`, because the other two
  hold the kernel lock for milliseconds (see
  [Kernel critical sections from core 0](real-time-latency.md#kernel-critical-sections-from-core-0)).
* esp32-semihosting-profiler is integrated into this firmware (`CONFIG_FUGU_WITH_SPROFILER`) and uses xtensa perfmon.

## Profiling tools in this firmware

The firmware has four built-in profiling tools:

* sprofiler (sampling), described in [esp32-semihosting-profiler](#esp32-semihosting-profiler).
* rtprof (sampling, core 1 only), described in [rtprof](#rtprof).
* rt-stats (sampling), in `src/etc/perf.h` and `src/cli.cpp`. The console command `rt-stats` samples the FreeRTOS
  run-time counters over ~2 s in the console task and prints CPU% per task per core. It then prints the RT task's heap
  operations (`rt-alloc:`, with `CONFIG_FUGU_RT_ALLOC_COUNT`), the loop-rate watchdog threshold, and the longest
  control-loop gap seen meanwhile (`loop gap during command: max <n> us`). Use it to check whether the RT loop is
  being preempted, or which task is using most of core 0.
* rtcount (per-section instrumentation). Wrap any RT path with `rtcount("name")`. It uses the xtensa cycle counter to
  accumulate count, min, max, and total per label. Calls exist in `mppt.cpp`, `sampling.h`, and `main.cpp`.
  `rtcount_print(reset)` prints the output, and `cmdResetLag` in `cli.cpp` calls it from the `reset-lag` command. To dump
  and zero the counters, run `reset-lag` over serial, telnet, or MQTT. Use it to find which step in `loopRTNewData` is
  slow.

## esp32-semihosting-profiler

The profiler builds on ESP-IDF semihosting. See the
[semihost_vfs example](https://github.com/espressif/esp-idf/blob/master/examples/storage/semihost_vfs/README.md).

The profiler is opt-in via `CONFIG_FUGU_WITH_SPROFILER=y`. Set it in `idf.py menuconfig` → "Fugu MPPT firmware", or in an
sdkconfig fragment. Default builds exclude the `esp32-semihosting-profiler` component to save flash (~6 KB) and DIRAM
(~8 KB `.bss`).

The top `CMakeLists.txt` reads this symbol before `project()` to drop the component. Run `idf.py reconfigure build` after
you toggle it. The legacy `WITH_SPROFILER` env var is rejected. `main.cpp` guards the profiler init with
`#ifdef WITH_SPROFILER`, and `main/CMakeLists.txt` sets the matching compile def.

At boot, `setup()` reads `sprofiler_hz` from [`pprof.conf`](../../reference/config/pprof.md) (0 = off, about 100–300).
The profiler starts only if a debugger is attached at that moment; otherwise the boot log prints
`sprofiler configured but not debugger attached`. A malformed `pprof.conf` skips the profiler and does not stop the boot.

To capture a profile, create `pprof.conf`, run openocd and the monitor on the host, and then run the analysis script.
Semihosting writes `sprof.out` to openocd's working directory, so start openocd in a scratch directory:

```
# create /littlefs/conf/pprof.conf:
# sprofiler_hz=100

# host terminal 1, in a scratch directory (here data/ in the repo root):
cd data
openocd -f board/esp32s3-builtin.cfg

# host terminal 2:
idf.py monitor

# .. let the program run some time ..

# host terminal 1:
ctrl+c
# now data/sprof.out is written
python3 ../components/esp32-semihosting-profiler/sprofiler.py

# macos: brew install qcachegrind

# tune PROFILING_ITEMS_PER_BANK
```

## rtprof

rtprof is a call-stack sampler for core 1, built with `CONFIG_FUGU_WITH_RTPROF=y` (default off). A 2 kHz timer ISR on
core 1 unwinds the interrupted stack and counts every flash-resident return address in a fixed hash table
(`CONFIG_FUGU_RTPROF_SLOTS`, default 4096 slots of 8 bytes). It needs no debugger.

The console command has three forms:

* `rtprof` prints `rtprof samples=… task=… lost=… entries=… elf=…`, then the table as `rtp` lines, then `rtprof end`.
* `rtprof clear` resets the table.
* `rtprof all` resets it and also counts IRAM and ROM addresses, which shows where the core waits.

`etc/rtprof_map.py <log> <elf>` maps a captured dump to functions. With `--lf out.lf` it writes linker mapping
fragments that place the sampled functions in IRAM (`noflash`); `main/rt_iram.lf` was generated this way. On the
ESP32-S3 the RT path is already in IRAM, so rtprof sees little of it. Build with the environment variable
`FUGU_RT_FLASH=1` to keep it in flash for a first profile of the whole path.

## gprof

Espressif provides a gprof component:

* [espressif/gprof in the component registry](https://components.espressif.com/components/espressif/gprof)
* [esp_gprof.c source](https://github.com/espressif/esp-iot-solution/blob/master/components/gprof/src/esp_gprof.c)

## Further options

Other building blocks for profiling are:

* `vTaskGetRunTimeStats`, described in [ESP32 performance profiling](https://blog.drorgluska.com/2022/12/esp32-performance-profiling.html).
* xtensa_perfmon, declared in
  [xt_perfmon.h](https://github.com/pycom/pycom-esp-idf/blob/master/components/esp32/include/xtensa/xt_perfmon.h).
* `esp_cpu_get_cycle_count()`.

## FreeRTOS runtime stats

To use the FreeRTOS runtime stats, see the
[real_time_stats example](https://github.com/espressif/esp-idf/tree/master/examples/system/freertos/real_time_stats). The
relevant options are:

* `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS`
* `FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER` ((Top) → Component config → FreeRTOS → Port → Choose the clock source for run
  time stats)

## Profiling options in sdkconfig

Two sdkconfig options enable profiling statistics in ESP-IDF components:

* `CONFIG_ESP_EVENT_LOOP_PROFILING` enables collection of statistics in the event loop library. These include the number
  of events posted to or received by an event loop, the number of callbacks involved, the number of events dropped to a
  full event loop queue, the run time of event handlers, and the number of times and run time of each event handler.
* `CONFIG_ESP_TIMER_PROFILING` makes `esp_timer_dump` dump information such as the number of times the timer was started,
  the number of times the timer has triggered, and the total time it took for the callback to run. This option has some
  effect on timer performance and the amount of memory used for timer storage, so use it only for debugging or testing.

## GCC instrumentation profiling

GCC can instrument code with `-fprofile-arcs`. It's an open question whether this works with ESP-IDF. See the
[GCC instrumentation options](https://gcc.gnu.org/onlinedocs/gcc/Instrumentation-Options.html).

## Related libraries

These libraries and examples also relate to profiling on ESP32:

* [ccomp_timer](https://github.com/espressif/idf-extra-components/tree/master/ccomp_timer)
* [perfmon example](https://github.com/espressif/esp-idf/tree/master/examples/system/perfmon)
* [LiluSoft/esp32-semihosting-profiler](https://github.com/LiluSoft/esp32-semihosting-profiler)
* [Carbon225/esp32-perfmon](https://github.com/Carbon225/esp32-perfmon)
* [ESP32 forum thread](https://esp32.com/viewtopic.php?t=39619)
