---
title: Real-time latency
sidebar_position: 4
---

# Real-time performance on ESP32 with Wi-Fi

With networking enabled, the ESP32 executes code that is not real-time capable and thus can block for a couple of
milliseconds.
Luckily, we have 2 cores, so we can use one core for all the non-RT and the other for the RT code.

The DC-DC converter control loop needs to have a fast load transient response to minimize transient surge voltages at
the output. Latency is the time between an input change and a response to this change at the output.

A control loop might look like this:

```
void criticalTask() {
  while(true) {
    adcRead();
    pwmWrite();
    yield();
  }
}
```

Latency should be deterministic. it is the maximum.
On a general purpose CPU, a lot of things can happen besides our critical task

## Real-time loop

Here's a rough pseudocode of how to achieve good real-time performance on the ESP32 while Wi-Fi is enabled:

```
void adcAlertInterrupt() {
  vTaskNotifyGiveFromISR(controlLoopTask);
}

void controlLoop() {
  while(true) {
    ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1));
    adcRead();
    updateControl();
    pwmWrite();
  }
}

void networkLoop() {
  // wifi stuff and everything else not RT-critical
}


void main() {
    controlLoopTask = createTaskCore1(controLoop, {.prio=20});
    networkLoopTask = createTaskCore0(networkLoop);
}

```

* `controlLoop` is our time critical task. we want the response time, i.e. the time the uC takes to react on an analog
  input change to the output, be less than 1 millisecond
* `core1` is our real-time core, everything that is not related to the controlLoop or can block longer runs on `core0`
* calling `ESP_LOGx(...)` usually writes `UART` and/or USB JTAG, which may block longer
* use a (non-blocking) queue to defer calls from the `controlLoop` to `networkLoopTask` (e.g. logging)
* `controlLoop` runs exclusively on `core1` with elevated priority
* notice that `controlLoop` doesn't call `yield()` or `vTaskDelay()`. `ulTaskNotifyTake` will block while ADC is busy,
  so FreeRTOS housekeeping (`IDLE` task) can run. TODO: specify housekeeping, what does idle task do?
* instead of semaphores we use task notifications which are faster according to FreeRTOS documentation

## ESP32(-S3) internal ADC

With the esp-idf API `esp_adc/adc_continuous.h` we cannot program the ADC conversion time. It appears to be always
working at the shortest possible time. This is why single shot measurements are quite noisy and it is better to use
continuous DMA reading and averaging with the highest possible sampling rate (83kHz for ESP32-S3).

Reading the DMA ring buffer from the "big" control loop might be to slow and we loose samples.
It might be useful to add another critical loop with even higher priority than the control loop that just reads and
averages the ADC samples.

Additionally, in this adc averaging loop we can implement a fast shutdown path to further reduce the response time
to OV or OC transients (load disconnect or short-circuit).

**Landmine — a no-sample watchdog must not gate the read() that feeds it.** `ADC_ESP32_Cont` has a
no-sample watchdog (`isGood()` returns false when the DMA delivered nothing for >1 s) so a stalled
internal ADC halts the converter instead of running MPPT on a stale Vin. But `read()` is the *only*
place that drains the DMA ring **and** refreshes the watchdog's `lastDataUs_`. An early version of
`ADC_Sampler::_updateAdc` checked `isGood()` *before* `read()` and returned `AdcError` on a stale
flag — so a single transient >1 s gap (e.g. a WiFi-reconnect storm starving the RT loop) latched the
ADC dead forever: the gate blocked the only call that could clear it, and `resetPeripherals` couldn't
reliably break out. Fix: for the `StreamedCallback` backend, **drain `read()` first** (a live DMA
self-clears), then report `AdcError` from the watchdog afterwards. General rule: a liveness watchdog
must never sit in front of the operation that proves liveness.

**…but drain-first was necessary-not-sufficient — the real boot `ADC error` was `wait()` starving
`read()`.** The reorder above still left every device tripping `E (….) main: ADC error` a second or
two after boot. Root cause was *not* in the ADC code at all but in `TaskNotification::wait()`
(`src/etc/rt.h`): it returned `ulTaskNotifyTake(pdFALSE, …) == 1`, i.e. true only when *exactly one*
notification was pending. At boot `loopRT` arms the watchdog in `start()`, then sits in the
`delay(1000)` under `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS` before the drain loop spins up — so the
conv-done ISR piles up *thousands* of notifications. `wait()` then returns false (`count != 1`),
`hasData()` is false, `read()` is **never called**, `lastDataUs_` never refreshes, and the watchdog
trips on its first `isGood()` (instrumented: `reads=0 hits=0 stale≈1100ms`). The drain-first reorder
can't help when the thing gating the drain is `hasData()` itself. Fix: make `wait()` a proper
clear-on-exit binary semaphore — `ulTaskNotifyTake(pdTRUE, …) != 0` — so any pending count reads as
one wakeup and `read()` drains the whole ring. This also removes a latent steady-state bug (the old
`==1` dropped a sample whenever ≥2 frames queued between iterations) and matches the FreeRTOS
"as-binary-semaphore" pattern `TaskNotification` already cites. The pre-watchdog firmware
build hid this: the same burst just cost a few harmless spin iterations.
(Diagnostic gotcha: `%lld` in `ESP_LOG` corrupts args under newlib-nano — the first instrumentation
pass printed impossible values; use 32-bit `%ld`/`%lu` casts. See *Configuration* / newlib notes.)

## Set explicit core affinity

```
CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y
CONFIG_LWIP_TCPIP_TASK_AFFINITY=0x0
CONFIG_PTHREAD_DEFAULT_CORE_NO_AFFINITY=0x0

CONFIG_ARDUINO_RUNNING_CORE=0
CONFIG_ARDUINO_RUN_CORE0=y
CONFIG_ARDUINO_EVENT_RUNNING_CORE=0
CONFIG_ARDUINO_EVENT_RUN_CORE0=y
CONFIG_ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE=0
CONFIG_ARDUINO_SERIAL_EVENT_RUN_CORE0=y
CONFIG_ARDUINO_UDP_RUNNING_CORE=0
CONFIG_ARDUINO_UDP_RUN_CORE0=y
```

* assume networking code runs on core0.
* we want to run the latency -sensitive loop on core1.
* arduino's loop() is not RT capable because it does UART stuff between calls (do not use)
* so run arduino and the network on core0, and the loopRT on core1

## Why RT_CORE=1 (core1), not core0

ESP32-S3's two LX7 cores are functionally symmetric for a control loop, so the choice
isn't about raw throughput. The concrete reason to keep RT on core1:

- NVS / littlefs / OTA writes happen from core0 (services, console). Each flash write
  briefly disables the CPU cache; non-IRAM code on *both* cores stalls during that
  window (see below).
- With RT on core1, core0's *CPU* work (services, console) never preempts the RT loop.
  Flash writes are different: they disable the cache on both cores, so only IRAM code (the
  ADC continuous-DMA ISR) keeps running. The RT loop, including protection, stalls for the
  write; see
  [Flash-cache disable stalls…](#flash-cache-disable-stalls-the-non-iram-rt-path-incl-the-alert-isr).
- If RT moved to core0, every `set-config` / OTA chunk / coulomb-counter persist would
  briefly steal cycles from the ADC/MPPT/PWM path.

Flipping RT_CORE to 0 would also require flipping every `CONFIG_*_PINNED_TO_CORE_0` and
`CONFIG_*_AFFINITY_CPU0` to its `_1`/`CPU1` counterpart — significant sdkconfig churn for
no gain. The `xPortGetCoreID() == 0` asserts in `loopNetwork_task` would also need to
become `RT_CORE ^ 1`.

## esp_timer ISR placement

Default in IDF is `CONFIG_ESP_TIMER_ISR_AFFINITY_CPU0` (ISR on PRO_CPU). An earlier sdkconfig
flipped that to CPU1 so any `dispatch_method = ESP_TIMER_ISR` callbacks would run with
RT-core latency. This firmware doesn't use ESP_TIMER_ISR dispatch anywhere — IDF's internal
esp_timer consumers (Wi-Fi keepalives, MQTT, NimBLE GAP, FreeRTOS-timers-via-service-task)
all dispatch to the task on CPU0. Net effect of the old placement: the RT path ate periodic
timer-ISR preemption for callbacks that ran on the other core anyway.

Current setting: `CONFIG_ESP_TIMER_ISR_AFFINITY_CPU0=y` (ISR away from RT_CORE). Re-enable
CPU1 affinity if a future safety callback (e.g. high-rate OV/OC watchdog) needs to dispatch
in ISR context on the RT core.

The check macros in `src/etc/rt_core_check.h` enforce this and the other task placements
at compile time — adding a Kconfig that drifts will fail the build.

## GPIO alert ISR placement (INA226 / ADS)

The INA226 (and ADS1x15) alert pin drives a GPIO interrupt whose handler does
`vTaskNotifyGiveFromISR(loopRT)` to wake the RT sampler. If that GPIO ISR runs on **core0**, the
notify is cross-core: it has to raise a scheduler interrupt on core1, adding latency and jitter to
the wake path that the RT loop blocks on. We want the alert ISR on **RT_CORE** so the notify is
local.

The catch: the GPIO ISR is a single shared service, not per-pin. `arduino-esp32`'s
`attachInterrupt()` *lazily* calls `gpio_install_isr_service()` on the first attach, pinning that
shared service to whatever core called it. The first `attachInterrupt` happens in `setupSensors()`,
which runs in `setup()` on **core0** — so by default every alert ISR lands on core0.

To control it we pre-install the service on RT_CORE *before* any `attachInterrupt` runs (gated by
`PIN_GPIO_ISR_TO_RT_CORE` in `main.cpp`); the later lazy install is then a no-op.

**Landmine — do not wrap `gpio_install_isr_service()` in `esp_ipc_call_blocking(RT_CORE, …)`.**
That function does its *own* internal `esp_ipc_call_blocking()` to the calling core (via
`gpio_isr_register` → `esp_intr_alloc` on the target core). Calling it from inside an IPC callback on
RT_CORE makes that core's single `ipc` worker wait on itself → **permanent deadlock in `setup()`**,
before `loopRT` even exists, so nothing reboots it. This was diagnosed via JTAG (loopTask blocked in
`esp_ipc_call_blocking`, `ipc1` blocked inside `gpio_install_isr_service`) and it silently bricked two
field units after an OTA. The correct way to run it on RT_CORE is a **short-lived task pinned to
RT_CORE** that calls `gpio_install_isr_service()` and notifies setup() when done — the `ipc` worker
stays free, the nested IPC completes, and the ISR lands on RT_CORE.

**IRAM — do NOT install with `ESP_INTR_FLAG_IRAM`.** `attachInterrupt()` registers arduino-esp32's
`__onPinInterrupt` dispatcher, which lives in flash (not IRAM). An IRAM-installed service keeps firing
while the flash cache is disabled — i.e. during *any* flash write (coulomb/stats persist, config
save, OTA) — and then jumps into that cached dispatcher, panicking with `Cache disabled but cached
memory region accessed` (seen on a live converter the instant a flash op coincided with an INA226 alert; the
mock-ADC bench never hits it because it has no `attachInterrupt`). Install with flags `0` instead: the
alert is simply masked for the brief cache-off window. RT_CORE affinity comes from the installing
task, independent of the flag, so latency in normal (cache-enabled) operation is unchanged.

This couples to the loop-latency shutdowns seen on live converters: when INA226 alert edges are missed/late
the RT sampler starves and the latency watchdog trips `stopAndBackoff`. Lower, deterministic wake
latency (ISR local to RT_CORE) reduces that pressure — the watchdog itself is correct, the starvation
is the bug.

## Flash-cache disable stalls the non-IRAM RT path (incl. the alert ISR)

*During a core0 flash write core1 keeps running only its **IRAM-resident** code (the ADC
continuous-DMA ISR).* A flash erase/write — littlefs (config
read **or** write, the `get-config`/`set-config` path, coulomb/stats persist), NVS, OTA — disables
the SPI-flash **cache globally** for its duration, and IDF parks the *other* core in IRAM while the
op runs. So any **non-IRAM** code stalls too, on whichever core it's pinned to. Core pinning isolates
the RT loop from core0's *CPU* work, not from a flash-cache-disable.

The INA226 sampling path is **non-IRAM**: the alert GPIO ISR is installed with flags `0` (it must be
— see the IRAM note above), so it is **masked for the whole cache-off window**, and the I2C read in
`loopRT` lives in flash. So *any* littlefs / NVS / OTA write — including a console `get-config`, which
is why polling tools hurt (see the loop-latency section) — freezes the INA226 sampler for the op's
duration. That's a sampler-starvation source distinct from missed alert edges, and another feeder of
the loop-latency shutdowns. Pinning doesn't help; what helps is **fewer/shorter flash ops on the hot
path** (persist cadence, avoid `get-config` storms) or moving OV/OC to the hardware
INA226-alert→gate-driver shutdown (see *Off-loading critical parts*), which is immune to cache state.

## Note about configTICK_RATE_HZ

defaults to 1000 (1tick = 1ms).
this is the shortest amount of time a task can wait.
not recommended to set to 10000, as it has a lot of overhead.
consider 2000Hz ?
https://www.esp32.com/viewtopic.php?t=1341#p6082

## wdt

https://esp32.com/viewtopic.php?t=14477

## Links

https://github.com/MacLeod-D/ESp32-Fast-external-IRQs

https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/api-guides/performance/speed.html#speed-targeted-optimizations
"In general, it is not recommended to set task priorities higher than the built-in Bluetooth/802.15.4 operations as
starving them of CPU may make the system unstable. For very short timing-critical operations that do not use the
network, use an ISR or a very restricted task (with very short bursts of runtime only) at the highest priority (24).
Choosing priority 19 allows lower-layer Bluetooth/802.15.4 functionality to run without delays, but still preempts the
lwIP TCP/IP stack and other less time-critical internal functionality - this is the best option for time-critical tasks
that do not perform network operations. Any task that does TCP/IP network operations should run at a lower priority than
the lwIP TCP/IP task (18) to avoid priority-inversion issues."

## esp32s2

* core1 is more performant than core0
* FastLED appears to have a significant lag (does it use bit banging?)

## instrumentation profiling of code latency

`gcc -pg`
https://stackoverflow.com/questions/7290131/how-does-gccs-pg-flag-work-in-relation-to-profilers
implement mcount for ESP32 (see esp32-semihosting-profiler)

https://github.com/MacLeod-D/ESp32-Fast-external-IRQs

## rtcount

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

## Deferred logging still mallocs on the RT core

Logging from `loopRT` (core1) is deferred: once `loggingEnableDefer()` runs (just before the RT loop starts),
`ESP_LOGx`/`UART_LOG`/`printf_mux` on core1 take the `enqueue_log()` path instead of writing UART/USB synchronously
(`src/logging.cpp`). So the UART blocking is *not* on the RT path. But `enqueue_log()` still does `new char[l+1]` per
entry, and `new` takes the global heap lock. During boot core0 is bringing up Wi-Fi/LWIP/MQTT-TLS with large
allocations that hold that lock for milliseconds, so the core1 `new` can stall on it.

Symptom: a one-shot multi-ms spike in `adc.update.handleSensorCalib` (e.g. max=9ms at an early `maxNum`), mean ~1µs.
The first sensor-calibration completion fires 2-3 `ESP_LOGI`s back-to-back (`src/adc/sampling.h`), each a contended
`new`, all attributed to that one rtcount window. It does not recur once boot allocation traffic settles.

To remove it, get the allocation off the RT path: preallocated buffer pool / fixed-size ring for the async log queue
instead of `new char[l+1]` per entry.

## Console commands trip the loop-latency watchdog

**Symptom:** a live converter cycled `Loop latency high (<200 Hz), shutdown!` → `stopAndBackoff(5s)` →
re-sweep + recalibrate, with no cache panics and no reboots — a symptom long attributed to INA226
alert misses.

**Root cause:** every shutdown fired ~1 s after a console command (`hostname`, `ip`, `uptime`,
`getc`), including pure in-memory ones, so it is **not** a flash read. Handling a command on core0
stalls the RT loop on core1 below the 200 Hz floor: the command's `received serial command` log, its
response and the `OK:` marker run through the console mux and allocate, holding the heap lock long
enough that core1's RT path stalls for a whole watchdog window (same contention as *Deferred logging
still mallocs on the RT core* above). A discovery / health poller that sends `ip`/`hostname`/`uptime`
therefore shut the converter down on every poll.

**Rule / fixes:**

- **Applied:** the loop-latency watchdog requires the low-sps condition to persist across 3
  consecutive windows before `stopAndBackoff`, so a one-off core0 stall (a poll) can't trip it
  (`lfWatchdog`). Per-sample OV/OC protection is unaffected. This is a mitigation.
- Still open (removes the contention at the source): get the log-queue allocation off the RT path
  (preallocated ring, as in *Deferred logging* above).
- Still open: keep pollers off a converter's console, or have lightweight commands (`ip`/`uptime`)
  avoid the heavy logging/alloc path.

## Boot-log backlog → MQTT, and the wifi-task stack trap

To make boot debuggable remotely, `logging.cpp` captures early log lines into an 8 KB buffer
(`s_bootLog`) and replays them to each sink as it attaches (`addLogCallback`) — so MQTT, which can't
connect until WiFi is up (well after `setup()`), still gets the boot sequence in `pv/log/<host>`. The
buffer freezes on first attach; `") mqtt:"`-tagged lines are skipped so the one-shot replay isn't
dropped by `mqttLogCallback`'s own filter.

**Trap (bricked a board):** moving the `esp_log → vprintf_` hook (`enable_esp_log_to_telnet`) to the
*start* of `setup()` — to capture the `setup()` body — routes the **wifi task**'s connect-time logging
burst through `vprintf_mux`, whose `loc_buf[300]` stack buffer (plus `vsnprintf` + callback frames)
overflows the wifi task's **3072-byte stack** → `***ERROR*** A stack overflow in task wifi has been
detected` → reboot loop, hung *before* any service starts (no telnet / MQTT / BLE → serial reflash
only). A mock-ADC bench board that never associates with a real AP boots clean and hides the bug.
Post-setup reconnects (AP loss / a slow WPA handshake) overflow the same way once the hook is active.

**Fix:** `vprintf_()` detects the wifi task (`pcTaskGetName`, guarded by `xPortCanYield()` so it's
never called from an ISR) and routes it to the light default `old_vprintf` (UART only), bypassing
`vprintf_mux` entirely, so connect *and reconnect* bursts are safe. (There is **no
`CONFIG_ESP_WIFI_TASK_STACK_SIZE`** in IDF 5.5 — the 3072 B is internal.)

Keeping `enable_esp_log_to_telnet()` **after** `registerServices()` is now belt-and-suspenders, not
load-bearing. Generally: any small-stack system task (wifi 3072 B) that logs through `vprintf_mux`
risks this — keep the heavy 300 B-buffer formatting path off those tasks.

## Flash Cache

* IRAM
* https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/performance/speed.html#measuring-performance
* https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/performance/speed.html#speed-targeted-optimizations
* noflash https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/linker-script-generation.html
    * https://github.com/espressif/esp-idf/blob/v4.2.2/components/freertos/linker.lf

## GCC Instrumentation

https://gcc.gnu.org/onlinedocs/gcc/Instrumentation-Options.html

* `-pg` flag
* https://stackoverflow.com/a/7290284/2950527
* inject call to mcount (or _mcount, or __mcount
* https://www.math.utah.edu/docs/info/gprof_toc.html
* https://docs-archive.freebsd.org/44doc/psd/18.gprof/paper.pdf

## Off-loading critical parts

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


## Console `tasks` / `rt-stats` wedged the continuous-ADC DMA (2026-05-30)

`uxTaskGetSystemState()` (used by the `tasks` and `rt-stats` console commands) walks every TCB under
`taskENTER_CRITICAL(&xKernelLock)` — measured ~1.16 ms for 8 tasks, scaling ~linearly, so ~2 ms on a
networked converter. While core 0 holds that lock, our IRAM `conv_done` callback on core 1 spins in
`vTaskNotifyGiveFromISR()`, stalling the ADC driver ISR so it can't recycle DMA descriptors.

The IDF continuous-ADC driver keeps a *fixed* `INTERNAL_BUF_NUM = 5` frames of DMA descriptors
(independent of `max_store_buf_size`, which only sizes the software ring/pool). At the old
`conv_frame_size = 64 B` that's only 5 × ~192 µs ≈ **0.96 ms** of headroom — less than the critical
section — so the DMA ran dry and **halted**, recovering only via `resetPeripherals()` (stop+start).
This is pre-existing (a 05-28 build reboots on `rt-stats`); the 05-29 no-sample watchdog merely made it
visible. `flush_pool`/bigger `max_store_buf_size` do **not** help — the wedge is descriptor starvation,
not pool overflow.

Fix:
- **A** — `conv_frame_size` raised to 128 B (`ADC1_READ_LEN` 128→256), giving 5 × ~0.38 ms ≈ 1.9 ms of
  DMA headroom so the driver rides through the critical section. Cost: conv-done / OV-protection
  latency rises from ~192 µs to ~384 µs. (A busier converter whose critical section exceeds ~1.9 ms
  still wedges; the loopRT watchdog (B) then resets+recovers it without a converter backoff. Bump
  `ADC1_READ_LEN` to 384/512 for more headroom at the cost of more latency.)
- **B** — `loopRT` ADC watchdog unified + made transient-tolerant: a stall is reset promptly
  (~300 ms throttle) and the converter is stopped only if it persists > ~800 ms (genuine dead ADC),
  so a diagnostic-induced blip no longer trips a backoff on a live converter.
