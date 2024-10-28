# esp32-semihosting-profiler
* https://github.com/espressif/esp-idf/blob/master/examples/storage/semihost_vfs/README.md
```
# create /littlefs/conf/pprof.conf:
# sprofiler_hz=100

# host terminal 1:
cd data
openocd -f board/esp32s3-builtin.cfg


# host terminal 2:
idf.py monitor

# host terminal 1:
ctrl+c
# now data/sprof.out is written
python3 ../components/esp32-semihosting-profiler/sprofiler.py

# macos: brew install qcachegrind

# tune PROFILING_ITEMS_PER_BANK
```

# TODO review
* https://github.com/LiluSoft/esp32-semihosting-profiler
* vTaskGetRunTimeStats https://blog.drorgluska.com/2022/12/esp32-performance-profiling.html
* xtensa_perfmon
  https://github.com/pycom/pycom-esp-idf/blob/master/components/esp32/include/xtensa/xt_perfmon.h
* esp_cpu_get_cycle_count()

# vTask
* example https://github.com/espressif/esp-idf/tree/master/examples/system/freertos/real_time_stats
* CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
* FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER ((Top) → Component config → FreeRTOS → Port → Choose the clock source for run time stats)


# sdkconfig
* CONFIG_ESP_EVENT_LOOP_PROFILING
  Enables collections of statistics in the event loop library such as the number of events posted
  to/recieved by an event loop, number of callbacks involved, number of events dropped to to a full event
  loop queue, run time of event handlers, and number of times/run time of each event handler.

* CONFIG_ESP_TIMER_PROFILING
  If enabled, esp_timer_dump will dump information such as number of times the timer was started,
  number of times the timer has triggered, and the total time it took for the callback to run.
  This option has some effect on timer performance and the amount of memory used for timer
  storage, and should only be used for debugging/testing purposes.



# GCC Instrumentation Profiling

-fprofile-arcs

* does this actually work with esp-idf?
  https://gcc.gnu.org/onlinedocs/gcc/Instrumentation-Options.html

# JTAG

TODO



# latency
