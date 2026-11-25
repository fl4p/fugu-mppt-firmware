---
name: fry-pbuf-ref0-panic-1124
description: fry PANIC 2026-11-24 18:03:53 UTC (build 1040): tcpip abort in pbuf_free = SYN seg pbuf ref==0 on MQTT connect-timeout close; undetermined (heap corruption vs upstream), no heap in core
metadata:
  type: project
---

fry PANIC 2026-11-24 18:03:53 UTC (fry-brk1-1040, sha 37960009). The tcpip thread aborted in pbuf_free while purging `unacked` of a
SYN_SENT pcb. esp-mqtt had closed fd 52 after `ret=0x8006` (ESP_TLS CONNECTION_TIMEOUT, 10 s, broker 192.168.1.200:1882). flat lost MQTT
in the same 18:02-18:04 window: a shared network outage, so the close path itself is routine.

- Silent assertions (`OPTIMIZATION_ASSERTIONS_SILENT`) merge every LWIP_ASSERT into one abort call. gdb then reports pbuf.c:734
  (`p != NULL`). Hand-decoding showed `l8ui a7,a2,14; beqz a7 -> abort`: the abort is **`p->ref > 0`** on the FIRST pbuf of the
  chain (a6/count=0, a7=0, a2=0x3fcedb00).
- The coredump holds task stacks only (64 KB partition; CAPTURE_DRAM needs >= 128 KB). Heap objects cannot be read.
- Toolchain objdump/gdb (esp-14.2/15.2) print raw words for this ELF and do not decode them. Decode Xtensa by hand.
- Our code calls no raw lwIP API (sockets only); the Wi-Fi TX path copies (no SPIRAM, so no tx_by_ref). HEAP_POISONING is off in
  the fleet build. There is one PANIC in the log since 11-13.

2026-11-25 follow-up on bench139c:
- CONFIG_LWIP_CHECK_THREAD_SAFETY aborted on the first boot. Arduino `configTzTime()` runs lwIP sntp (udp_new, sys_timeout)
  on loopTask without core locking: a real race. Fixed by switching to the `esp_sntp_*` calls, which post to the tcpip thread.
  It was a candidate seed for the heap corruption, but it runs only until the first time sync, so it was not active at 18:03.
- Bench cost (vconv image, 10+ min each): light heap poisoning cost -13 KB free heap, -11 KB min-ever and -9 KB in the largest
  block, so it is not used on fry/flat (sdkconfig.d/heap_poison is opt-in). The lwIP check plus the pbuf probe cost about
  0.6 KB of RAM and showed no measurable CPU or lag change, so the fleet build has both.

**How to apply:** if it repeats, read `p/x g_pbufProbe` in the core. `crash overrun|lwip|pbuf` checks each mechanism.
