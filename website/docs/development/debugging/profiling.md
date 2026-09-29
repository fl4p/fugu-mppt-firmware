---
title: Profiling
sidebar_position: 6
---

# Profiling on esp32

There are a couple of ways (instrumented and sampled):

* SEGGER SystemView (over ESP-IDF App Trace / JTAG) — the Espressif-blessed way. Enable CONFIG_APPTRACE_SV_ENABLE=y, attach a JTAG probe (built-in USB-JTAG
   on the S3 works), open SystemView on the host. You get task/ISR timelines, context-switch traces, optional user markers. Docs: ESP-IDF Application Level
   Tracing Library + SystemView Tracing. The closest thing to a "real" profiler on ESP-IDF.
  * Tracealyzer (Percepio) — commercial, consumes the SystemView protocol, prettier UI. Same data path as #1.
* GDB sampling — openocd + xtensa-esp32-elf-gdb, periodic bt for a poor-man's sampling profiler.
* FreeRTOS runtime stats — vTaskGetRunTimeStats() / uxTaskGetSystemState(). Needs CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y +
   CONFIG_FREERTOS_USE_TRACE_FACILITY=y. Zero hardware required. This is exactly what your rt-stats already wraps.
* esp32-semihosting-profiler (integrated into this firmware `WITH_SPROFILER`)
  * uses xtensa perfmon


In this firmware:
* sprofiler (sampling)
* rt-stats (sampling)
  * (src/etc/perf.h, src/cli.cpp:292)
    Console command rt-stats spawns a one-shot task that samples FreeRTOS runtime stats over ~2 s and prints CPU% per task per core. Best for "is the RT loop
    being preempted" or "who is hogging core 0.
* rtcount (per-section instrumentation)
  *   Wrap any RT path with rtcount("name"); it uses the xtensa cycle counter to accumulate count/min/max/total per label. Already sprinkled through mppt.cpp,
      sampling.h, main.cpp. Output is printed by rtcount_print(reset) — `cmdResetLag` in cli.cpp calls it from the reset-lag command, so run reset-lag over serial/telnet/MQTT
      to dump and zero the counters. Best for "which step in loopRTNewData is slow."



## esp32-semihosting-profiler
* https://github.com/espressif/esp-idf/blob/master/examples/storage/semihost_vfs/README.md

> **Opt-in via `CONFIG_FUGU_WITH_SPROFILER=y`** (`idf.py menuconfig` → "Fugu MPPT firmware", or an
> sdkconfig fragment). Default builds exclude the `esp32-semihosting-profiler` component to save
> flash (~6 KB) and DIRAM (~8 KB `.bss`). The top `CMakeLists.txt` reads this symbol before
> `project()` to drop the component; run `idf.py reconfigure build` after toggling it. The legacy
> `WITH_SPROFILER` env var is rejected. `main.cpp` guards the profiler
> init with `#ifdef WITH_SPROFILER`, and `main/CMakeLists.txt` sets the matching compile def
> (mirrors the `WITH_BLE` flag pattern).

```
# create /littlefs/conf/pprof.conf:
# sprofiler_hz=100

# host terminal 1:
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

## gprof
https://components.espressif.com/components/espressif/gprof
https://github.com/espressif/esp-iot-solution/blob/master/components/gprof/src/esp_gprof.c

## Further options
* vTaskGetRunTimeStats https://blog.drorgluska.com/2022/12/esp32-performance-profiling.html
* xtensa_perfmon
  https://github.com/pycom/pycom-esp-idf/blob/master/components/esp32/include/xtensa/xt_perfmon.h
* esp_cpu_get_cycle_count()

## vTask
* example https://github.com/espressif/esp-idf/tree/master/examples/system/freertos/real_time_stats
* CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
* FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER ((Top) → Component config → FreeRTOS → Port → Choose the clock source for run time stats)

https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/api-guides/app_trace.html#app-trace-system-behaviour-analysis-with-segger-systemview



## sdkconfig
* CONFIG_ESP_EVENT_LOOP_PROFILING
  Enables collections of statistics in the event loop library such as the number of events posted
  to/recieved by an event loop, number of callbacks involved, number of events dropped to to a full event
  loop queue, run time of event handlers, and number of times/run time of each event handler.

* CONFIG_ESP_TIMER_PROFILING
  If enabled, esp_timer_dump will dump information such as number of times the timer was started,
  number of times the timer has triggered, and the total time it took for the callback to run.
  This option has some effect on timer performance and the amount of memory used for timer
  storage, and should only be used for debugging/testing purposes.



## GCC Instrumentation Profiling

-fprofile-arcs

* does this actually work with esp-idf?
  https://gcc.gnu.org/onlinedocs/gcc/Instrumentation-Options.html



ä libs
https://github.com/espressif/idf-extra-components/tree/master/ccomp_timer
https://github.com/espressif/esp-idf/tree/master/examples/system/perfmon
https://github.com/LiluSoft/esp32-semihosting-profiler
https://github.com/Carbon225/esp32-perfmon
https://esp32.com/viewtopic.php?t=39619