---
title: Real-time latency
sidebar_position: 4
---

# Real-time latency

The converter control loop must respond to a load transient fast enough to keep output overshoot small. Latency here
is the time from an input change to the PWM response, and for protection only the worst case counts, not the mean.
With networking enabled the ESP32 runs code that can block for milliseconds, so the firmware splits the work across the
two cores: the real-time (RT) loop owns core 1, and everything else runs on core 0.

This page covers what keeps the RT loop fast, what still stalls it from the other core, and how to measure it.

## Latency budget and watchdogs

`loopRT` (`src/main.cpp`) runs once per ADC sample: sampling, protection, the PD controllers, MPPT and the PWM update.
Three supervisors on the same task detect a loop that falls behind:

| Supervisor | Trips when | Action |
|---|---|---|
| Loop-rate watchdog (`lfWatchdog`) | Samples per second stay below `sensor.conf::expected_hz` for 3 consecutive ~3 s windows (outside calibration and manual PWM) | `Loop latency high (…), shutdown!`, `stopAndBackoff(4)` |
| ADC stall watchdog | The ADC reports an error, or no fresh sample arrived for 200 ms outside calibration | `resetPeripherals()` at most every 300 ms; `ADC stall <n> ms, shutdown` and `stopAndBackoff(16)` if the stall persists > 800 ms; system restart after 60 s |
| No-sample check | No sample at all 20 s after start | `Never got a sample! Please check ADC`, converter disabled; system restart once uptime passes 15 min |

The per-sample OV/OC cutouts in `mppt.protect` are independent of these and run on every sample that arrives. The
status line reports `lag=` in µs: the longest interval between two loop iterations while the converter was enabled,
since the last `reset-lag` or periodic sweep.

`expected_hz` is documented in [sensor.conf](../../reference/config/sensor.md); `0` disables the loop-rate watchdog.

## RT loop structure

The loop never sleeps voluntarily. It blocks on a task notification given by the ADC interrupt, which also lets the
idle task run while the ADC converts. Simplified:

```cpp
void adcAlertIsr() {
    vTaskNotifyGiveFromISR(rtTask, &woken);
}

void loopRT() {
    while (true) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1))) { // clear-on-exit
            adcRead();
            protect();
            updateControl();
            pwmWrite();
        }
    }
}

void setup() {
    xTaskCreatePinnedToCore(loopRT, "loopRt", 16384, nullptr, RT_PRIO, nullptr, RT_CORE); // RT_PRIO = 20
    // Arduino loop() = network loop, core 0
}
```

- **Task notifications, not semaphores.** `TaskNotification` (`src/etc/rt.h`) wraps them; FreeRTOS documents them as
  faster than a binary semaphore.
- **Clear on exit.** `wait()` calls `ulTaskNotifyTake(pdTRUE, …)` and treats any non-zero count as one wake-up, so
  `read()` drains everything that accumulated. A decrement-by-one variant that returns true only for a count of exactly
  one starves `read()` whenever notifications pile up, for example during the 1 s start-up delay that
  `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS` adds before the loop starts.
- **No `yield()` or `vTaskDelay()` in the RT path.** Blocking on the ADC is the only wait.
- **The 1 ms wait timeout is one tick.** It applies to the internal ADC; the INA226 backend derives its timeout from
  the conversion time. `CONFIG_FREERTOS_HZ=1000` sets the shortest time a task can wait. Raising it to 10 kHz costs a
  lot of scheduler overhead; 2 kHz would be the next step to evaluate.
- **Priority 20.** `loopRT` sits above the lwIP TCP/IP task (18) but below the Wi-Fi and Bluetooth controller tasks
  (23), which are pinned to core 0.

## Core layout and affinity

`RT_CORE` is `1` and `NON_RT_CORE` is `0` (`src/util.h`). The Arduino `loop()` on core 0 runs `loopNetwork_task`
(console, Wi-Fi, telemetry, MQTT, `loopLF`) and asserts that it runs on core 0. Arduino's `loop()` is not suitable
for RT work because the runtime does UART work between calls.

`sdkconfig.defaults` pins every Arduino and network task to core 0:

```ini
CONFIG_ARDUINO_RUNNING_CORE=0
CONFIG_ARDUINO_RUN_CORE0=y
CONFIG_ARDUINO_EVENT_RUNNING_CORE=0
CONFIG_ARDUINO_EVENT_RUN_CORE0=y
CONFIG_ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE=0
CONFIG_ARDUINO_SERIAL_EVENT_RUN_CORE0=y
CONFIG_ARDUINO_UDP_RUNNING_CORE=0
CONFIG_ARDUINO_UDP_RUN_CORE0=y

CONFIG_MQTT_TASK_CORE_SELECTION_ENABLED=y
CONFIG_MQTT_USE_CORE_0=y

CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y
CONFIG_LWIP_TCPIP_TASK_AFFINITY=0x0
CONFIG_MDNS_TASK_AFFINITY_CPU0=y
CONFIG_MDNS_TASK_AFFINITY=0x0
CONFIG_PTHREAD_DEFAULT_CORE_NO_AFFINITY=0x0

CONFIG_ESP_TIMER_ISR_AFFINITY_CPU0=y
CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0=y
```

With BLE enabled, `sdkconfig.ble` adds `CONFIG_BT_CTRL_PINNED_TO_CORE_0=y` and `CONFIG_BT_NIMBLE_PINNED_TO_CORE_0=y`.
The Wi-Fi task stays on its IDF default, core 0.

`src/etc/rt_core_check.h` turns placement drift into a build error: Arduino runtime, main task, esp_timer task and ISR,
lwIP, mDNS, default pthread core, NimBLE host, Wi-Fi and MQTT tasks must all be off `RT_CORE`.

### Why core 1

The two ESP32-S3 cores are symmetric, so the choice is about isolation, not throughput. The tasks that produce
non-RT load (Wi-Fi, lwIP, MQTT, console, services, NVS/littlefs/OTA writers) run on core 0, and with the RT loop on
core 1 their CPU time never preempts it. Flash writes are the exception; see
[Flash-cache-disable windows](#flash-cache-disable-windows).

Moving the RT loop to core 0 would mean flipping every `_CORE0`/`_CPU0`/`_PINNED_TO_CORE_0` setting above to its core-1
counterpart and changing the core-0 assert in `loopNetwork_task`, for no gain.

## Interrupts

### esp_timer ISR

IDF defaults the esp_timer ISR to core 0, and the firmware keeps it there (`CONFIG_ESP_TIMER_ISR_AFFINITY_CPU0=y`). No
code in the firmware uses `ESP_TIMER_ISR` dispatch; IDF's own esp_timer users (Wi-Fi, MQTT, NimBLE, FreeRTOS timers)
dispatch to the esp_timer task on core 0. An ISR on the RT core would only add periodic preemption to the ADC/MPPT/PWM
path. Move it to core 1 only if a future safety callback must run in ISR context on the RT core, and update the check
in `rt_core_check.h` with it.

### GPIO alert ISR (INA226, ADS1x15)

The INA226 and ADS1x15 backends attach a falling-edge interrupt on the ALERT pin whose handler notifies the RT task.
On core 0 that notify is cross-core and has to raise a scheduler interrupt on core 1, which adds latency and jitter to
the wake-up the RT loop blocks on. The handler belongs on `RT_CORE`.

The GPIO ISR is one shared service, not a per-pin interrupt. arduino-esp32's `attachInterrupt()` installs it lazily on
first use, on the calling core, and the first call comes from `setupSensors()` in `setup()` on core 0. The firmware
therefore installs the service on `RT_CORE` before any `attachInterrupt()` (`pinGpioIsrToRtCore()`, gated by
`PIN_GPIO_ISR_TO_RT_CORE`, default `1`); the later lazy install is a no-op.

Two constraints apply to that install:

- **Use a pinned task, not `esp_ipc_call_blocking()`.** `gpio_install_isr_service()` performs its own
  `esp_ipc_call_blocking()` to allocate the interrupt on the calling core. Called from inside an IPC callback on
  `RT_CORE`, the core's single IPC worker waits on itself and `setup()` deadlocks permanently, before `loopRT` or any
  network service exists. The firmware runs the install in a short-lived task pinned to `RT_CORE` that notifies
  `setup()` when done.
- **Install with flags `0`, not `ESP_INTR_FLAG_IRAM`.** `attachInterrupt()` registers arduino-esp32's
  `__onPinInterrupt` dispatcher, which lives in flash. An IRAM service keeps firing while the flash cache is disabled
  and then jumps into that dispatcher, which panics with `Cache disabled but cached memory region accessed`. A mock-ADC
  configuration never attaches the interrupt and cannot reproduce this. With flags `0` the alert is masked for the
  cache-off window. The core affinity comes from the installing task, not from the flag.

A late or missed alert starves the sampler, and the loop-rate watchdog then shuts the converter down. That shutdown is
correct; the starvation is the fault to fix.

### IRAM requirements

- The continuous-ADC conversion-done callback and everything it calls are `IRAM_ATTR`
  (`s_conv_done_cb`, `ADC_ESP32_Cont::convDoneCallback`, `TaskNotification::notifyFromIsr`), and
  `CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE=y` is required; `adc_esp32_cont.cpp` fails the build without it. This ISR keeps
  running during flash operations.
- Anything reached from an IRAM-safe ISR must be in IRAM or DRAM. An ISR that calls into flash code must not be
  installed as IRAM-safe.

## What stalls the RT core

Core pinning isolates the RT loop from core 0's CPU time. It does not isolate it from shared resources: the flash
cache, kernel spinlocks and the heap lock.

### Flash-cache-disable windows

**Mechanism.** A littlefs read or write, an NVS commit, an OTA write or an erase disables the SPI-flash cache for its
duration, and IDF parks the other core in IRAM while the operation runs. Only IRAM-resident code keeps running on
core 1, which here is the continuous-ADC DMA ISR. The rest of the RT loop, including protection, stops. The INA226 path
is entirely non-IRAM: its alert ISR is masked (flags `0`, see above) and its I2C read runs from flash, so every flash
operation freezes the INA226 sampler for the operation's duration.

**Sources in this firmware.** Console `get-config` and `set-config`, coulomb-counter and statistics persistence
(`/littlefs/stats`), configuration saves, NVS and OTA.

**What to do.** Keep flash operations off the hot path: persist at a low cadence and avoid scripts that poll
`get-config`. Protection that must survive a flash operation belongs in hardware; see
[Off-loading protection to hardware](#off-loading-protection-to-hardware).

### Kernel critical sections from core 0

**Mechanism.** `uxTaskGetSystemState()`, used by the `tasks` and `rt-stats` console commands, walks every task control
block under `taskENTER_CRITICAL(&xKernelLock)`. While core 0 holds that lock, the IRAM conversion-done callback on
core 1 spins in `vTaskNotifyGiveFromISR()`, so the ADC driver's ISR cannot recycle DMA descriptors.

**Magnitude** (measured). About 1.16 ms for 8 tasks, scaling roughly linearly, so about 2 ms on a networked
converter.

**What the firmware does.** Sizes the DMA frames to ride through about 1.9 ms (see
[DMA descriptor headroom](#dma-descriptor-headroom)), and if the DMA still halts, the ADC stall watchdog restarts it
without a converter backoff unless the stall persists past 800 ms.

**What to avoid.** Running `tasks` or `rt-stats` repeatedly on a converter under load, and adding other calls that hold
kernel locks for milliseconds on core 0.

### Heap-lock contention from logging and console commands

**Mechanism.** After `loggingEnableDefer()` (called just before the loop starts), `ESP_LOGx`, `UART_LOG` and
`printf_mux` on core 1 do not write UART or USB. `enqueue_log()` (`src/logging.cpp`) formats into a heap buffer and
queues it for core 0. That allocation (`new (std::nothrow) char[201]`, dropped on failure or when more than 200 entries
are queued) takes the global heap lock. Whenever core 0 holds that lock for a long time, core 1 waits:

- During boot, Wi-Fi, lwIP and MQTT-TLS bring-up make large allocations. The symptom is a one-shot spike in the
  `adc.update.handleSensorCalib` rtcount label (max 9 ms at an early `maxNum`, mean about 1 µs): the first sensor
  calibration completes with two or three back-to-back `ESP_LOGI` calls (`src/adc/sampling.h`), each a contended
  allocation. It does not recur once boot allocation settles.
- Every console command logs `received serial command`, prints its response and an `OK:` marker through the console
  mux, which allocates. This stalled the RT loop below the watchdog floor for a whole window even for commands that
  touch neither flash nor hardware (`hostname`, `ip`, `uptime`); a discovery or health poller sending those commands
  shut the converter down on every poll.

**What the firmware does.** The loop-rate watchdog requires three consecutive starved windows before it backs off, so
a single core-0 stall cannot trip it. This is a mitigation; the contention remains.

**Still open.** Replace the per-entry allocation with a preallocated ring, and keep lightweight commands off the heavy
logging path.

**What to avoid.** Logging from the RT loop in steady state, and polling a converter's console.

### Logging on small-stack system tasks

This does not stall the RT core, but it can hang a device in `setup()` where only a serial reflash recovers it.

The boot-log backlog and remote log sinks are described in [Logging](logging.md). They route `ESP_LOGx` output
through `vprintf_mux`, including output from system tasks.

`vprintf_mux` formats into a 300-byte stack buffer and calls the sink callbacks. On the IDF Wi-Fi task, whose stack is
3072 bytes (internal; IDF 5.5 has no `CONFIG_ESP_WIFI_TASK_STACK_SIZE`), a connect or reconnect burst through that path
overflows the stack (`***ERROR*** A stack overflow in task wifi has been detected`) and the device reboot-loops
before any network service starts. A board that never associates with
a real access point does not reproduce it. The firmware guards against it in three places:

- `vprintf_()` detects the Wi-Fi task by name (only in task context, checked with `xPortCanYield()`) and sends its
  output to the default `vprintf` (UART only), bypassing `vprintf_mux`.
- `enable_esp_log_to_telnet()` is called after `registerServices()`, late in `setup()`.
- `CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE=4096` gives the system event task, which also logs through the hook, room
  for the same path.

Any small-stack system task that logs through `vprintf_mux` has the same risk.

## Internal ADC (continuous mode)

`ADC_ESP32_Cont` (`src/adc/adc_esp32_cont.h`) drives ADC1 through the IDF continuous (DMA) driver.

- **Conversion time is fixed.** `esp_adc/adc_continuous.h` does not expose it, and the ADC appears to run at its
  shortest conversion time, which makes single-shot readings noisy. The firmware samples continuously at a high raw
  rate (up to 83 kHz on the ESP32-S3) and averages in `read()`: `sensor.conf::esp32adc1_sr` sets the raw rate and
  `esp32adc1_avg` (1–1023) the number of conversions per delivered sample.
- **`read()` runs on the RT loop.** If the loop drains the DMA ring too slowly, samples are lost. A dedicated
  higher-priority task that only drains and averages, with a fast OV/OC shutdown path of its own, would shorten the
  response to a load disconnect or short circuit; the firmware does not have one.

### DMA descriptor headroom

The IDF driver keeps a fixed `INTERNAL_BUF_NUM = 5` frames of DMA descriptors. `max_store_buf_size` sizes only the
software ring, and `flush_pool` does not help: a stalled conversion-done ISR starves the descriptors, not the pool.
Headroom is therefore five frame times, set by `conv_frame_size = ADC1_READ_LEN / 2`:

| `ADC1_READ_LEN` | `conv_frame_size` | Frame time at 83 kHz | DMA headroom (5 frames) | Conversion-done latency |
|---|---|---|---|---|
| 128 | 64 B | ~192 µs | ~0.96 ms | ~192 µs |
| 256 (current) | 128 B | ~0.38 ms | ~1.9 ms | ~384 µs |

At 64 B frames the headroom is shorter than a `uxTaskGetSystemState()` critical section, and the DMA halts until
`resetPeripherals()` restarts it. With more than about 13 tasks the critical section can exceed 1.9 ms again; the ADC
stall watchdog then recovers the DMA. `ADC1_READ_LEN` of 384 or 512 buys more headroom at the cost of more latency.
Frame time scales inversely with `esp32adc1_sr`.

### No-sample watchdog

`isGood()` returns false when the DMA has delivered nothing for 250 ms (`kNoDataTimeoutUs`), so a stalled ADC halts
the converter instead of running MPPT on a stale input voltage. `read()` is both the only DMA drain and the only place
that refreshes the watchdog, so `ADC_Sampler::_updateAdc` calls `read()` first and checks `isGood()` afterwards. If
the check came first, a single long gap would latch the ADC as dead permanently. The general rule: a liveness
watchdog must never gate the operation that proves liveness.

## Off-loading protection to hardware

The INA226 can raise its ALERT output on bus over-voltage. Wired to the gate driver's shutdown input, it turns the
converter off independently of the firmware, the flash cache and core scheduling. The INA226's shortest conversion time
is 140 µs. The INA226 has a single ALERT pin, and the INA226 backend already uses it as the conversion-ready
interrupt, so using it for over-voltage takes that interrupt away from sampling.

## Measuring latency

| Tool | Answers | Reference |
|---|---|---|
| `lag=` in the status line | Longest loop interval since the last `reset-lag` | [Console](../../reference/console.md) |
| `rtcount("label")` + `reset-lag` | Which section of the RT loop is slow (count, total, mean, max, and the sample index of the max) | [rtcount](rtcount.md) |
| `rt-stats` | CPU % per task and core over about 2 s | [Profiling](profiling.md) |
| `tasks` | Task placement, priority, stack headroom | [Console](../../reference/console.md) |
| Sampling profiler (`CONFIG_FUGU_WITH_SPROFILER`), SystemView, gprof | Where time goes on average | [Profiling](profiling.md) |

`rt-stats` and `tasks` call `uxTaskGetSystemState()` and stall the ADC DMA for about 1–2 ms themselves; see
[Kernel critical sections from core 0](#kernel-critical-sections-from-core-0).

GCC's `-pg` instrumentation inserts a call to `mcount` (or `_mcount`, `__mcount`) at every function entry, and the
target has to provide that function: the Espressif gprof component listed under [Profiling](profiling.md), or an
implementation built on the esp32-semihosting-profiler.

### Reading rtcount output

For latency, sort by `max`: the maximum execution time of a block, not its mean, determines the response time. `maxNum`
is the sample index at which the maximum occurred, so a small `maxNum` points to start-up. A label's time is measured
from the previous `rtcount()` call, so a stall anywhere between two labels is attributed to the second.

The dumps below were committed in November 2024, taken with an earlier rtcount that printed integer microseconds and a 32-bit
`tot`. The current version prints fractional microseconds and adds `min`/`minNum` columns. Several labels show maxima
of 32–38 ms. The console excerpt comes from a mock-ADC build of the same period, whose status line printed `lag` in
ms; it shows `lag` jumping from 0.9 ms to 34.8 ms across a sweep start that also wrote `/littlefs/stats`.

<details>
<summary>Example rtcount dumps (committed November 2024)</summary>

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

</details>

## Checklist

- Pin nothing to core 1 except `loopRT` and short-lived setup tasks such as the GPIO ISR installer, and never move a
  system task there; `rt_core_check.h` must keep building.
- Do not call `vTaskDelay()`, `yield()` or anything that blocks on I/O from `loopRT`. Block only on the ADC.
- Do not log from the RT loop in steady state. Deferred logging still allocates on core 1.
- Do not allocate on the RT path. `rtcount` uses a fixed table for this reason.
- Install an interrupt with `ESP_INTR_FLAG_IRAM` only if the handler and everything it calls is in IRAM; install
  flash-resident handlers with flags `0`.
- Install interrupts that wake the RT loop from a task on `RT_CORE`, never through `esp_ipc_call_blocking()` to the
  same core.
- Keep flash writes rare and short; do not poll `get-config`, `tasks` or `rt-stats` on a converter under load.
- Put a liveness watchdog after the operation that proves liveness, never in front of it.
- Keep heavy log formatting off small-stack system tasks.
- Check a change with `rtcount` maxima and `lag=`, not with means.

## Further reading

- [ESP-IDF Speed Optimization, ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/performance/speed.html):
  measuring performance, targeted optimizations, the priorities of built-in tasks and IRAM-safe interrupt handlers.
- [ESP-IDF Speed Optimization, choosing task priorities (ESP32-H2 edition)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/api-guides/performance/speed.html#choosing-task-priorities-of-the-application):
  the guidance behind `RT_PRIO = 20`. It recommends priority 19 for time-critical tasks that do no networking,
  preempting lwIP (18), and the highest priority (24) only for very short bursts.
- [ESP-IDF Linker Script Generation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/linker-script-generation.html):
  placing functions in IRAM with `noflash` fragments; the
  [FreeRTOS `linker.lf` in ESP-IDF v4.2.2](https://github.com/espressif/esp-idf/blob/v4.2.2/components/freertos/linker.lf)
  is a worked example.
- [ESP32: 3 million external interrupts per second](https://github.com/MacLeod-D/ESp32-Fast-external-IRQs): fast
  external-interrupt handling on the ESP32.
- [Increasing RTOS Tick Rate, >1000Hz](https://www.esp32.com/viewtopic.php?t=1341#p6082) (ESP32 forum): raising
  `CONFIG_FREERTOS_HZ` for sub-millisecond waits.
- [Getting error message: "Task watchdog got triggered"](https://esp32.com/viewtopic.php?t=14477) (ESP32 forum):
  task watchdog triggers with several FreeRTOS tasks.
- [GCC Instrumentation Options](https://gcc.gnu.org/onlinedocs/gcc/Instrumentation-Options.html): `-pg` and the other
  instrumentation flags.
- [How does GCC's `-pg` flag work in relation to profilers?](https://stackoverflow.com/a/7290284/2950527) (Stack
  Overflow): how the inserted `mcount` calls feed gprof.
- [GNU gprof manual](https://www.math.utah.edu/docs/info/gprof_toc.html) and
  [gprof: a Call Graph Execution Profiler](https://docs-archive.freebsd.org/44doc/psd/18.gprof/paper.pdf) (Graham,
  Kessler, McKusick): the profiler that `-pg` output is made for.
