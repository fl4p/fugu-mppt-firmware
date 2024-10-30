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

