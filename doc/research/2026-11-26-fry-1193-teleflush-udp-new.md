*this document is an LLM generated placeholder*

# fry 1193-g91418052 boot panic (2026-11-26)

Fleet image with `CONFIG_LWIP_CHECK_THREAD_SAFETY=y` panicked ~2 s after `setup()` on fry, 3/3 OTA attempts, rolled back to 1181-g307a325c. Core dump backtrace of the crashed task:

```
Crashed task handle: 0x3fcd32f8, name: 'teleflush', GDB name: 'process 1070412536'
Crashed task is not in the interrupt context
#0  panic_abort (details=0x3fcd9cc0 <g_shared_buffers+7868> "abort() was called at PC 0x4206bcd1 on core 0") at IDF/components/esp_system/panic.c:471
#1  0x403864bc in esp_system_abort (details=0x3fcd9cc0 <g_shared_buffers+7868> "abort() was called at PC 0x4206bcd1 on core 0") at IDF/components/esp_system/port/esp_system_chip.c:87
#2  0x403853fd in abort () at IDF/components/esp_libc/src/abort.c:38
#3  0x4206bcd4 in udp_new () at IDF/components/lwip/lwip/src/core/udp.c:1239
#4  0x4201f600 in AsyncUDP::writeTo (this=0x3fcb849c <asyncUdp>, data=0x3fcf34ac <error: Cannot access memory at address 0x3fcf34ac>, len=901, addr=0x3fcd9da0 <g_shared_buffers+8092>, port=8086, tcpip_if=TCPIP_ADAPTER_IF_MAX) at managed_components/espressif__arduino-esp32/libraries/AsyncUDP/src/AsyncUDP.cpp:770
#5  0x4201f6b2 in AsyncUDP::writeTo (this=0x3fcb849c <asyncUdp>, data=0x3fcf34ac <error: Cannot access memory at address 0x3fcf34ac>, len=901, addr=..., port=8086, tcpip_if=TCPIP_ADAPTER_IF_MAX) at managed_components/espressif__arduino-esp32/libraries/AsyncUDP/src/AsyncUDP.cpp:846
#6  0x42104678 in sendBinaryBatch (dst=..., comp=..., raw=..., port=8086) at src/tele/telemetry.cpp:320
#7  0x42104765 in TelemetryService::flushQueue (this=<optimized out>, addr=...) at src/tele/telemetry.cpp:355
#8  0x4210494b in TelemetryService::flushTask (arg=0x3fcb6370 <telemetryService>) at src/tele/telemetry.cpp:334
#9  0x4038a258 in vPortTaskWrapper (pxCode=0x42104940 <TelemetryService::flushTask(void*)>, pvParameters=0x3fcb6370 <telemetryService>) at IDF/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147
```

AsyncUDP::writeTo() creates its pcb with udp_new() on the caller thread (teleflush). Fix: telemetry sends over a BSD socket. Bench139c with tele on (binary=1) ran clean with the check enabled.
