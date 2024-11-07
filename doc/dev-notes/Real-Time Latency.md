* writing the flash (energy meter) causes latency issues?
    * TODO write test

```

void networkLoop() {
  // wifi stuff and everything that 
}

void adcAlertInterrupt() {
  vTaskNotifyGiveFromISR(controlLoopTask);
}

void controlLoop() {
  while(true) {
    ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1));
    adcRead();
    pwmWrite();
  }
}

void main() {
    controlLoopTask = createTaskCore1(controLoop, {.prio=20});
    networkLoopTask = createTaskCore0(networkLoop);
}

```

* `controlLoop` is our time critical task. we want the response time, i.e. the time the uC takes to react on an analog
  input change to the output, be less than 1 millisecond
* calling `ESP_LOGx(...)` usually writes `UART`, which may block longer
* `core1` is our real-time core, everything that is not related to the controlLoop or can block longer runs on `core0`
* `controlLoop` runs exclusively on `core1` with elevated priority
* use a queue to defer calls from the `controlLoop` to `networkLoopTask` (e.g. logging)
* notice that `controlLoop` doesn' t call `yield()` or `vTaskDelay()`. `ulTaskNotifyTake` will block while ADC is busy,
  so
  FreeRTOS housekeeping (`IDLE` task) can run. TODO: specify housekeeping, what does idle task do?

# meausures

assume blocking networkt code runs on core0
we want to run the latency -sensitive loop on core1.
arduino's loop() is not RT capable because it does UART stuff between calls
so run arduino and the network on core0, and the loopRT on core1

// TODO use xSemaphoreGiveFromISR()

# configTICK_RATE_HZ

defaults to 1000 (1tick = 1ms)
for better RT perf set this to 10000
this is the shortest amount of time a task can wait
dont set higher than 1000 !
https://www.esp32.com/viewtopic.php?t=1341#p6082

#wdt
https://esp32.com/viewtopic.php?t=14477

# esp32 adc1

* response time of adc1 appears to increase when WiFi enabled
    * TODO verify
    * TODO use adc2?

https://github.com/MacLeod-D/ESp32-Fast-external-IRQs

https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/api-guides/performance/speed.html#speed-targeted-optimizations
"In general, it is not recommended to set task priorities higher than the built-in Bluetooth/802.15.4 operations as
starving them of CPU may make the system unstable. For very short timing-critical operations that do not use the
network, use an ISR or a very restricted task (with very short bursts of runtime only) at the highest priority (24).
Choosing priority 19 allows lower-layer Bluetooth/802.15.4 functionality to run without delays, but still preempts the
lwIP TCP/IP stack and other less time-critical internal functionality - this is the best option for time-critical tasks
that do not perform network operations. Any task that does TCP/IP network operations should run at a lower priority than
the lwIP TCP/IP task (18) to avoid priority-inversion issues."

# TODO

* try unpinned tasks
*

# esp32s2

* core1 is more performant than core0


* FastLED appears to have a significant lag

# instrumentation profiling of code latency

`gcc -pg`
https://stackoverflow.com/questions/7290131/how-does-gccs-pg-flag-work-in-relation-to-profilers
implement mcount for ESP32 (see esp32-semihosting-profiler)

https://github.com/MacLeod-D/ESp32-Fast-external-IRQs

# rtcount

```
rtcount_print :
key                          num       tot   mean    max maxNum
adc.update              36761737 201234640      5  34734 10528868
mppt.update             12244296 161811306     13    753 8302755
protect                 36732884 298683769      8    537 10520083
adc.update.hasData      36761736 1576494944     42    277 24928691

```

```
rtcount_print :
key                          num       tot   mean    max maxNum
mppt.update             18930352 396081207     20    755 18452797
mppt.startSweep               11      7598    690    722      7
adc.update.handleSensorCalib  56848970 104091131      1    435  53647
protect                 56790990 549161721      9    367      0
adc.update.hasData      56887286 1304901885     22    296 54448783
adc.update              56887286  81186758      1    294 55961535
loopNewData             56844290  83121435      1    293 51416048
adc.update.getSample    56848970 262392390      4    279 43230846
adc.update.AddSampleVirtual  18948096  67719059      3    259 18136411
adc.update.addSample    56848970 167718961      2    232 28838738
adc.update.pre          56887287  71026559      1  
```

```
rtcount_print :
key                                  num       tot   mean    max maxNum
adc.update.getSample           178010278 476241806      2  34601 107427244
mppt.update                     59288893 1005532037     16  34426 35777832
protect                        177865294 1406598724      7  32692 58894801
micros                         178154656 405998661      2  32623 156025214
mppt.startSweep                      253    183189    724    766    137
adc.update.AddSampleVirtual     59306279 203131843      3    538 43849404
adc.update.hasData             178154655 1056389751      5    536 83190215
adc.update                     178154655 255527731      1    530 156025232



rtcount_print :
key                                  num       tot   mean    max maxNum
protect                        720495257 1193071824      1  38146 115103855
adc.update.hasData             721038471 3386893558      4  36684 345675918
mppt.update                    240165136 4108143820     17    847 182623930
mppt.startSweep                        2      1438    719    754      1
start                          721038472 781230800      1    530 576578725
adc.update.handleSensorCalib   720540801  31003272      0    419 547443066
adc.update.AddSampleVirtual    240177087 855618884      3    301 158338037
adc.update.addSample           720540801 794266807      1    292 287884402
protect.pre                    720495257 274942314      0    284 676866487
loopNewData                    720531260  84327997      0    283 662507698
micros                         721038473 727540760      1    283 57643726
adc.update.pre                 721038473  24744438      0    237 144052547
adc.update.startReading        720540801 120960605      0    229 100746055
adc.update.getSample           720540801 1166212132      1    131 585236293
adc.update                     721038473  47885776      0    120 201659165
mppt.update.pre                240165137    344493      0     99 9590907
mppt.startSweep.pre                    2        18      9     11      0


```

```
V=56.45/29.18 I= 1.3/ 2.39A  71.8W -34℃31℃ 1136sps  0㎅/s PWM(H|L|Lm)= 218| 185| 185 st=↑MPPT,1 lag=0.9ms lt=0.9ms N=4788104 rssi=-12
I (6010354) mppt: periodic zero-current calibration
PWM disabled (duty cycle was 245)
I (6010355) mppt: Start sweep
I (6010355) mppt: Start calibration
I (6010355) sensor: U_in_raw reset calibration
I (6010355) adc_fake: Reset channel 1 at 1715387325
I (6010355) adc_fake: Reset channel 2 at 1715387325
I (6010355) sensor: U_out_raw reset calibration
I (6010355) adc_fake: Reset channel 1 at 1715387325
I (6010370) sampler: Sensor U_in_raw calibration: avg=56.4545 std=0.000000
I (6010370) sampler: Sensor Io calibration: avg=0.0000 std=0.000000
I (6010370) sampler: Sensor Io midpoint-calibrated: 0.000000
I (6010371) sampler: Sensor U_out_raw calibration: avg=29.1818 std=0.000000
I (6010371) sampler: Calibration done!
Backflow switch disabled
I (6010579) store: Wrote /littlefs/stats (size 32)
I (6010580) flash: Wrote flash value /littlefs/stats
V=56.45/29.18 I= 0.0/ 0.00A   0.0W -34℃31℃  0sps  0㎅/s PWM(H|L|Lm)=  65| 123| 123 st=SWEEP,1 lag=34.8ms lt=34.8ms N=2608 rssi=-12
Current above threshold 0.20
Backflow switch enabled
Low-side switch enabled
V=56.45/29.18 I= 1.1/ 2.09A  62.8W -34℃31℃ 1135sps  0㎅/s PWM(H|L|Lm)= 209| 178| 178 st=SWEEP,1 lag=34.8ms lt=34.8ms N=14613 rssi=-12
V=56.45/29.18 I= 1.4/ 2.63A  79.0W -34℃31℃ 1137sps  0㎅/s PWM(H|L|Lm)= 354| 302| 302 st=SWEEP,1 lag=34.8ms lt=34.8ms N=26624 rssi=-12
V=56.45/29.18 I= 1.5/ 2.83A  85.1W -34℃31℃ 1136sps  0㎅/s PWM(H|L|Lm)= 498| 424| 424 st=SWEEP,1 lag=34.8ms lt=34.8ms N=38633 rssi=-11
V=56.45/29.18 I= 1.6/ 2.96A  89.0W -34℃31℃ 1135sps  0㎅/s PW
```

# Flash Cache

* IRAM
* https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/performance/speed.html#measuring-performance
* https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/performance/speed.html#speed-targeted-optimizations
* noflash https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/linker-script-generation.html
    * https://github.com/espressif/esp-idf/blob/v4.2.2/components/freertos/linker.lf

# GCC Instrumentation
https://gcc.gnu.org/onlinedocs/gcc/Instrumentation-Options.html

* `-pg` flag
* https://stackoverflow.com/a/7290284/2950527
* inject call to mcount (or _mcount, or __mcount
* https://www.math.utah.edu/docs/info/gprof_toc.html
* https://docs-archive.freebsd.org/44doc/psd/18.gprof/paper.pdf

# Off-loading critical parts

The INA226 can be programmed to trigger an alert on bus over-voltage. this signal can be wired to the shut-down input of
the gate driver to instantly turn off the DC-DC converter. The INA226 has a minimum conversion time of 140µs.



run arduino:
```
CONFIG_ARDUINO_RUNNING_CORE=0
CONFIG_ARDUINO_RUN_CORE0=y
CONFIG_ARDUINO_EVENT_RUNNING_CORE=0
CONFIG_ARDUINO_EVENT_RUN_CORE0=y
CONFIG_ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE=0
CONFIG_ARDUINO_SERIAL_EVENT_RUN_CORE0=y
CONFIG_ARDUINO_UDP_RUNNING_CORE=0
CONFIG_ARDUINO_UDP_RUN_CORE0=y
```