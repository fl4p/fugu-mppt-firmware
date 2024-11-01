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