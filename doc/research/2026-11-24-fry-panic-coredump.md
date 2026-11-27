*this document is an LLM generated placeholder*

# fry panic 2026-11-24 ~18:00 UTC (fw fry-brk1-1040), coredump decode

Retrieved 2026-11-25 with `fugu_console.py --mqtt --name fry --coredump get` (dump still on fry, not erased); decoded with etc/idf-devtools/elf_archive.py (matched build 37960009).

```
matched build 37960009 (sha-in-core)
espcoredump.py v1.17.2
===============================================================
==================== ESP32 CORE DUMP START ====================

Crashed task handle: 0x3fcc5078, name: 'tcpip', GDB name: 'process 1070354552'
Crashed task is not in the interrupt context
Panic reason: abort() was called at PC 0x420677ac on core 0

================== CURRENT THREAD REGISTERS ===================
exccause       0x0 (IllegalInstructionCause)
excvaddr       0x0
epc1           0x42010462
epc2           0x0
epc3           0x400559e0
epc4           0x0
epc5           0x0
epc6           0x0
eps2           0x0
eps3           0x60021
eps4           0x0
eps5           0x0
eps6           0x0
pc             0x40386544          0x40386544 <panic_abort+16>
lbeg           0x40056f5c          1074098012
lend           0x40056f72          1074098034
lcount         0x0                 0
sar            0x2                 2
ps             0x60320             394016
threadptr      <unavailable>
br             <unavailable>
scompare1      <unavailable>
acclo          <unavailable>
acchi          <unavailable>
m0             <unavailable>
m1             <unavailable>
m2             <unavailable>
m3             <unavailable>
expstate       <unavailable>
f64r_lo        <unavailable>
f64r_hi        <unavailable>
f64s           <unavailable>
fcr            <unavailable>
fsr            <unavailable>
a0             0x803864e0          -2143787808
a1             0x3fcc6b90          1070361488
a2             0x3fcc6bd0          1070361552
a3             0x3fcc6bfd          1070361597
a4             0xa                 10
a5             0x1                 1
a6             0x0                 0
a7             0x3fcc4f34          1070354228
a8             0x3fcb1f24          1070276388
a9             0x1                 1
a10            0x3fcc6c1d          1070361629
a11            0x3fcc6c1d          1070361629
a12            0xa                 10
a13            0x3fcc4f34          1070354228
a14            0x1                 1
a15            0x60123             393507

==================== CURRENT THREAD STACK =====================
#0  panic_abort (details=0x3fcc6bd0 "abort() was called at PC 0x420677ac on core 0") at /Users/fab/dev/esp/idf6.0.3/components/esp_system/panic.c:471
#1  0x403864e0 in esp_system_abort (details=0x3fcc6bd0 "abort() was called at PC 0x420677ac on core 0") at /Users/fab/dev/esp/idf6.0.3/components/esp_system/port/esp_system_chip.c:87
#2  0x40385421 in abort () at /Users/fab/dev/esp/idf6.0.3/components/esp_libc/src/abort.c:38
#3  0x420677af in pbuf_free (p=0x3fcedb00 <uart_rx_buff+1004>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/pbuf.c:734
#4  0x420682dc in tcp_seg_free (seg=0x3fcece40) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:1641
#5  0x420682f4 in tcp_segs_free (seg=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:1626
#6  0x4206858c in tcp_pcb_purge (pcb=0x3fced90c <uart_rx_buff+504>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:2208
#7  tcp_pcb_purge (pcb=0x3fced90c <uart_rx_buff+504>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:2173
#8  0x420685d0 in tcp_pcb_remove (pcblist=0x3fcbb124 <tcp_active_pcbs>, pcb=0x3fced90c <uart_rx_buff+504>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:2230
#9  0x42068a39 in tcp_close_shutdown (pcb=0x3fced90c <uart_rx_buff+504>, rst_on_unacked_data=1 '\001') at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:398
#10 0x42068a78 in tcp_close_ext (pcb=<optimized out>, rst_on_unacked_data=1 '\001') at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:504
#11 0x4206fe94 in lwip_netconn_do_close_internal (conn=0x3fcee09c) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/api_msg.c:1055
#12 0x420703a4 in lwip_netconn_do_delconn (m=0x3fcd7f80 <g_shared_buffers+380>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/api_msg.c:1246
#13 0x42065e6e in tcpip_thread (arg=0x0) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/tcpip.c:136
#14 0x4038a27c in vPortTaskWrapper (pxCode=0x42065e54 <tcpip_thread>, pvParameters=0x0) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147

======================== THREADS INFO =========================
  Id   Target Id          Frame 
* 1    process 1070354552 panic_abort (details=0x3fcc6bd0 "abort() was called at PC 0x420677ac on core 0") at /Users/fab/dev/esp/idf6.0.3/components/esp_system/panic.c:471
  2    process 1070398996 0x400559e0 in _xtos_set_intlevel ()
  3    process 1070433136 0x400559e0 in _xtos_set_intlevel ()
  4    process 1070514332 vTaskDelay (xTicksToDelay=1) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/tasks.c:1615
  5    process 1070347284 0x40383e22 in esp_cpu_wait_for_intr () at /Users/fab/dev/esp/idf6.0.3/components/esp_hw_support/cpu.c:64
  6    process 1070348940 0x40383e22 in esp_cpu_wait_for_intr () at /Users/fab/dev/esp/idf6.0.3/components/esp_hw_support/cpu.c:64
  7    process 1070406644 vTaskDelay (xTicksToDelay=20) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/tasks.c:1615
  8    process 1070351108 prvProcessReceivedCommands () at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/timers.c:856
  9    process 1070355788 0x400559e0 in _xtos_set_intlevel ()
  10   process 1070355904 0x400559e0 in _xtos_set_intlevel ()
  11   process 1070331108 0x400559e0 in _xtos_set_intlevel ()
  12   process 1070329708 0x400559e0 in _xtos_set_intlevel ()
  13   process 1070398780 0x400559e0 in _xtos_set_intlevel ()
  14   process 1070500912 0x400559e0 in _xtos_set_intlevel ()
  15   process 1070461468 0x400559e0 in _xtos_set_intlevel ()
  16   process 1070376900 0x400559e0 in _xtos_set_intlevel ()
  17   process 1070338740 0x400559e0 in _xtos_set_intlevel ()


       TCB             NAME PRIO C/B  STACK USED/FREE
---------- ---------------- -------- ----------------
0x3fcc5078            tcpip    18/18         816/2752
0x3fccfe14           loopRt    20/20        1072/7104
0x3fcd8370        mqtt_task      5/5        1600/6580
0x3fcec09c         loopTask      1/1         768/7416
0x3fcc3414            IDLE0      0/0          704/816
0x3fcc3a8c            IDLE1      0/0          704/824
0x3fcd1bf4        teleflush      1/1         672/3408
0x3fcc4304          Tmr Svc      1/1         672/1360
0x3fcc554c          sys_evt    20/20         832/3772
0x3fcc55c0   arduino_events    19/19         976/3112
0x3fcbf4e4             ipc1    20/20          736/528
0x3fcbef6c             ipc0    23/23          736/536
0x3fccfd3c             mdns      1/1         768/3316
0x3fce8c30      nimble_host    21/21         848/3236
0x3fcdf21c     btController    23/23         800/3288
0x3fcca7c4             wifi    23/23         800/2768
0x3fcc12b4        esp_timer    22/22         736/3344

==================== THREAD 1 (TCB: 0x3fcc5078, name: 'tcpip') =====================
#0  panic_abort (details=0x3fcc6bd0 "abort() was called at PC 0x420677ac on core 0") at /Users/fab/dev/esp/idf6.0.3/components/esp_system/panic.c:471
#1  0x403864e0 in esp_system_abort (details=0x3fcc6bd0 "abort() was called at PC 0x420677ac on core 0") at /Users/fab/dev/esp/idf6.0.3/components/esp_system/port/esp_system_chip.c:87
#2  0x40385421 in abort () at /Users/fab/dev/esp/idf6.0.3/components/esp_libc/src/abort.c:38
#3  0x420677af in pbuf_free (p=0x3fcedb00 <uart_rx_buff+1004>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/pbuf.c:734
#4  0x420682dc in tcp_seg_free (seg=0x3fcece40) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:1641
#5  0x420682f4 in tcp_segs_free (seg=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:1626
#6  0x4206858c in tcp_pcb_purge (pcb=0x3fced90c <uart_rx_buff+504>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:2208
#7  tcp_pcb_purge (pcb=0x3fced90c <uart_rx_buff+504>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:2173
#8  0x420685d0 in tcp_pcb_remove (pcblist=0x3fcbb124 <tcp_active_pcbs>, pcb=0x3fced90c <uart_rx_buff+504>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:2230
#9  0x42068a39 in tcp_close_shutdown (pcb=0x3fced90c <uart_rx_buff+504>, rst_on_unacked_data=1 '\001') at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:398
#10 0x42068a78 in tcp_close_ext (pcb=<optimized out>, rst_on_unacked_data=1 '\001') at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/core/tcp.c:504
#11 0x4206fe94 in lwip_netconn_do_close_internal (conn=0x3fcee09c) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/api_msg.c:1055
#12 0x420703a4 in lwip_netconn_do_delconn (m=0x3fcd7f80 <g_shared_buffers+380>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/api_msg.c:1246
#13 0x42065e6e in tcpip_thread (arg=0x0) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/tcpip.c:136
#14 0x4038a27c in vPortTaskWrapper (pxCode=0x42065e54 <tcpip_thread>, pvParameters=0x0) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147

==================== THREAD 2 (TCB: 0x3fccfe14, name: 'loopRt') =====================
#0  0x400559e0 in _xtos_set_intlevel ()
#1  0x4038a52c in vPortClearInterruptMaskFromISR (prev_level=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/include/freertos/portmacro.h:547
#2  vPortExitCritical (mux=0x0) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:528
#3  0x40000006 in _WindowOverflow4 ()

==================== THREAD 3 (TCB: 0x3fcd8370, name: 'mqtt_task') =====================
#0  0x400559e0 in _xtos_set_intlevel ()
#1  0x4038a52c in vPortClearInterruptMaskFromISR (prev_level=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/include/freertos/portmacro.h:547
#2  vPortExitCritical (mux=0x3fcc4fe8) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:528
#3  0x403878f0 in xQueueGenericSend (xQueue=0x3fcc4f94, pvItemToQueue=0x3fcd7e50 <g_shared_buffers+76>, xTicksToWait=<optimized out>, xCopyPosition=0) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/queue.c:1056
#4  0x4206ec20 in sys_mbox_post (mbox=0x3fcbabcc <tcpip_mbox>, msg=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/port/freertos/sys_arch.c:244
#5  0x42066039 in tcpip_send_msg_wait_sem (fn=0x42070310 <lwip_netconn_do_delconn>, apimsg=0x3fcd7f80 <g_shared_buffers+380>, sem=0x3fcd84f8 <g_shared_buffers+1780>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/tcpip.c:460
#6  0x4206f1e1 in netconn_apimsg (fn=0x42070310 <lwip_netconn_do_delconn>, apimsg=0x3fcd7f80 <g_shared_buffers+380>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/api_lib.c:134
#7  0x4206f272 in netconn_prepare_delete (conn=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/api_lib.c:217
#8  0x42065343 in lwip_close (s=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/lwip/lwip/src/api/sockets.c:845
#9  0x4206f04c in lwip_close_r_wrapper (ctx=0x0, fd=52) at /Users/fab/dev/esp/idf6.0.3/components/lwip/port/esp32xx/vfs_lwip.c:80
#10 0x4200eaad in esp_vfs_close (r=<optimized out>, fd=52) at /Users/fab/dev/esp/idf6.0.3/components/vfs/vfs_calls.c:144
#11 0x42007134 in close (fd=52) at /Users/fab/dev/esp/idf6.0.3/components/esp_libc/src/syscalls.c:41
#12 0x42076e81 in tcp_connect (host=<optimized out>, hostlen=<optimized out>, port=1882, cfg=0x3fcecda8, error_handle=<optimized out>, sockfd=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/esp-tls/esp_tls.c:484
#13 0x42077135 in esp_tls_plain_tcp_connect (host=0x3fcd38e0 <error: Cannot access memory at address 0x3fcd38e0>, hostlen=13, port=1882, cfg=0x3fcecda8, error_handle=0x3fced5f4, sockfd=0x3fcece14) at /Users/fab/dev/esp/idf6.0.3/components/esp-tls/esp_tls.c:578
#14 0x420787ba in tcp_connect (t=0x3fcec134, host=0x3fcd38e0 <error: Cannot access memory at address 0x3fcd38e0>, port=1882, timeout_ms=10000) at /Users/fab/dev/esp/idf6.0.3/components/tcp_transport/transport_ssl.c:149
#15 0x420d185e in esp_transport_connect (t=<optimized out>, host=0x3fcd38e0 <error: Cannot access memory at address 0x3fcd38e0>, port=1882, timeout_ms=10000) at /Users/fab/dev/esp/idf6.0.3/components/tcp_transport/transport.c:123
#16 0x4201bac4 in esp_mqtt_task (pv=0x3fcd36a0) at /Users/fab/dev/pv/fugu-fleet-wt/managed_components/espressif__mqtt/mqtt_client.c:2029
#17 0x4038a27c in vPortTaskWrapper (pxCode=0x4201b82c <esp_mqtt_task>, pvParameters=0x3fcd36a0) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147

==================== THREAD 4 (TCB: 0x3fcec09c, name: 'loopTask') =====================
#0  vTaskDelay (xTicksToDelay=1) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/tasks.c:1615
#1  0x42131621 in loopNetwork_task (arg=0x0) at /Users/fab/dev/pv/fugu-fleet-wt/src/main.cpp:2011
#2  loop () at /Users/fab/dev/pv/fugu-fleet-wt/src/main.cpp:716
#3  0x42011518 in loopTask (pvParameters=0x0) at /Users/fab/dev/pv/fugu-fleet-wt/managed_components/espressif__arduino-esp32/cores/esp32/main.cpp:84
#4  0x4038a27c in vPortTaskWrapper (pxCode=0x420114cc <loopTask(void*)>, pvParameters=0x0) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147

==================== THREAD 5 (TCB: 0x3fcc3414, name: 'IDLE0') =====================
#0  0x40383e22 in esp_cpu_wait_for_intr () at /Users/fab/dev/esp/idf6.0.3/components/esp_hw_support/cpu.c:64
#1  0x4038650e in esp_vApplicationIdleHook () at /Users/fab/dev/esp/idf6.0.3/components/esp_system/freertos_hooks.c:58
#2  0x4038845a in prvIdleTask (pvParameters=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/tasks.c:4350
#3  0x4038a27c in vPortTaskWrapper (pxCode=0x40388398 <prvIdleTask>, pvParameters=0x0) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147

==================== THREAD 6 (TCB: 0x3fcc3a8c, name: 'IDLE1') =====================
#0  0x40383e22 in esp_cpu_wait_for_intr () at /Users/fab/dev/esp/idf6.0.3/components/esp_hw_support/cpu.c:64
#1  0x4038650e in esp_vApplicationIdleHook () at /Users/fab/dev/esp/idf6.0.3/components/esp_system/freertos_hooks.c:58
#2  0x4038845a in prvIdleTask (pvParameters=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/tasks.c:4350
#3  0x4038a27c in vPortTaskWrapper (pxCode=0x40388398 <prvIdleTask>, pvParameters=0x0) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147

==================== THREAD 7 (TCB: 0x3fcd1bf4, name: 'teleflush') =====================
#0  vTaskDelay (xTicksToDelay=20) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/tasks.c:1615
#1  0x42104b6f in TelemetryService::flushTask (arg=0x3fcb6148 <telemetryService>) at /Users/fab/dev/pv/fugu-fleet-wt/src/tele/telemetry.cpp:325
#2  0x4038a27c in vPortTaskWrapper (pxCode=0x42104b5c <TelemetryService::flushTask(void*)>, pvParameters=0x3fcb6148 <telemetryService>) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147

==================== THREAD 8 (TCB: 0x3fcc4304, name: 'Tmr Svc') =====================
#0  prvProcessReceivedCommands () at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/timers.c:856
#1  prvTimerTask (pvParameters=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/timers.c:688
#2  0x4038a27c in vPortTaskWrapper (pxCode=0x4038ae18 <prvTimerTask>, pvParameters=0x0) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147

==================== THREAD 9 (TCB: 0x3fcc554c, name: 'sys_evt') =====================
#0  0x400559e0 in _xtos_set_intlevel ()
#1  0x4038a52c in vPortClearInterruptMaskFromISR (prev_level=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/include/freertos/portmacro.h:547
#2  vPortExitCritical (mux=0x3fcc52dc) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:528
#3  0x40387c2c in xQueueReceive (xQueue=0x3fcc5288, pvBuffer=0x3fcc7fc0, xTicksToWait=<optimized out>) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/queue.c:1632
#4  0x420e5510 in esp_event_loop_run (event_loop=0x3fcc525c, ticks_to_run=4294967295) at /Users/fab/dev/esp/idf6.0.3/components/esp_event/esp_event.c:657
#5  0x420e5526 in esp_event_loop_run_task (args=0x3fcc525c) at /Users/fab/dev/esp/idf6.0.3/components/esp_event/esp_event.c:106
#6  0x4038a27c in vPortTaskWrapper (pxCode=0x420e551c <esp_event_loop_run_task>, pvParameters=0x3fcc525c) at /Users/fab/dev/esp/idf6.0.3/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:147

```
