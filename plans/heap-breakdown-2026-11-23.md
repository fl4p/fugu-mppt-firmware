*this document is an LLM generated placeholder*

# Heap breakdown of the fleet image (bench139c, 2026-11-23)

Measured on bench139c only. No command was sent to fry or flat.

## Setup

- Image: `sdkconfig.defaults;sdkconfig.fleet;sdkconfig.vconv_s3;sdkconfig.heaptrack` from
  branch `heap-breakdown-1123` (fry-brk1-828-g238f9928, IDF 6.0.3). The resolved sdkconfig differs
  from the fleet build only in `FUGU_WITH_MCPWM`/`WSYNC` → `FUGU_WITH_VCONV` (bench139c's littlefs
  uses `adc=vconv`; a non-vconv image cannot sample there) and `CONFIG_HEAP_TASK_TRACKING=y`.
  Static DIRAM use: fleet 219 719 B, this image 219 835 B, so the heap size is the same within 0.1 KB.
- Services as on fry: mqtt, tele, ftp, telnet, scope, ble on; lcd off. Tele ran the text wire first,
  then `binary 1` (fry's free heap is matched only with the binary wire, see below).
- Console: USB serial (on havan, `/dev/ttyACM0`), so reading the heap allocates nothing. Telnet and
  MQTT readings were 3-10 KB noisier because their own output sits in lwIP/MQTT buffers.
- Each figure is the median of 8-13 `heap` samples ~1.5 s apart. Spread is ±0.3 KB, with single
  outliers up to +2 KB.
- Owner tags add 4 B per allocated block (`MULTI_HEAP_BLOCK_OWNER_SIZE`). At ~760 blocks that is
  ~3.0 KB. "Corrected" figures subtract it. Nothing else changes: the IDF task-tracking hooks are
  stubbed out (see "Tooling").

## Totals

| | bench, corrected | fry (reported) |
|---|---|---|
| INTERNAL free (median) | 21.8 KB + 3.0 = **24.9 KB** | 24.9 KB |
| INTERNAL allocated | 211.7 KB − 3.0 = 208.6 KB in 760 blocks | |
| allocated + free | 233.5 KB | 239.9 KB total (`heap_caps_get_total_size` also counts TLSF headers/control) |
| INTERNAL min-ever | 11.9 KB (incl. svc toggles, one BLE stream) | 10-13 KB |
| DMA free / min-ever | 14.0 KB / 4.1 KB | min-ever 2.6-4.9 KB |

The bench reproduces fry's free heap to 0.1 KB once the binary telemetry wire is on. With the text
wire it was 3.9 KB higher (25.7 KB raw).

## Per task (owner of each block)

The heap layer tags every block with the task that allocated it. A task's stack and TCB are tagged
with the task that **created** it, so the big rows are creators, not consumers. Sizes are allocated
block sizes, raw / corrected for the 4 B tag.

| owner | held raw | blocks | held corrected | note |
|---|---:|---:|---:|---|
| loopTask | 147 316 | 496 | 145 332 | `setup()`: BLE and Wi-Fi init, app objects, stacks of the tasks it created; includes the 3.6 KB buffers of `heap tasks` (serial console runs here) |
| wifi | 20 120 | 50 | 19 920 | Wi-Fi driver runtime buffers |
| ipc0 | 17 784 | 59 | 17 548 | startup allocations made before the scheduler ran (inferred: highest-priority task then) |
| (gone) main_task | 10 920 | 31 | 10 796 | loopTask stack + TCB, Arduino init (inferred owner) |
| btController | 7 720 | 21 | 7 636 | BT controller runtime |
| tcpip | 4 404 | 29 | 4 288 | lwIP pcbs/pbufs |
| teleflush | 2 300 | 4 | 2 284 | binary wire buffers |
| (no task) | 2 052 | 14 | 1 996 | before the first task |
| (gone) startup task | 1 792 | 15 | 1 732 | unidentified deleted startup task |
| (gone) teleflush | 1 496 | 2 | 1 488 | static tele buffers of an earlier teleflush instance; a later stop added a 2 176 B block, 4 more restarts added nothing (kept, not leaked) |
| mqtt_task | 824 | 14 | 768 | |
| nimble_host | 320 | 11 | 276 | 104 B before the first BLE connection |
| (gone) others | 648 | 6 | 624 | |
| sys_evt, arduino_events, mdns, ipc1 | 452 | 13 | 400 | |
| **sum** | 218 148 | 765 | 215 088 | 211 512 without the `heap tasks` buffers |

### Stacks

Block size as allocated (configured size + TLSF rounding) and the most ever used (block − high-water
mark). The bench run is short and makes 0 W, so live use can be higher.

| task | stack block | used | never used |
|---|---:|---:|---:|
| loopRt | 8 700 | 2 072 | 6 628 |
| loopTask | 8 700 | 4 196 | 4 504 |
| mqtt_task | 8 700 | 2 864 | 5 836 |
| sys_evt | 4 860 | 1 768 | 3 092 |
| teleflush | 4 348 | 1 556 | 2 792 |
| btController | 4 348 | 1 792 | 2 556 |
| nimble_host | 4 348 | 3 280 | 1 068 |
| arduino_events | 4 348 | 1 452 | 2 896 |
| mdns | 4 348 | 2 028 | 2 320 |
| esp_timer | 4 348 | 1 300 | 3 048 |
| tcpip | 3 708 | 1 956 | 1 752 |
| wifi | 3 708 | 2 744 | 964 |
| Tmr Svc | 2 172 | 800 | 1 372 |
| IDLE0 / IDLE1 | 1 596 each | 888 / 884 | 708 / 712 |
| ipc0 / ipc1 | 1 340 each | 960 / 972 | 380 / 368 |
| **17 tasks** | **72 508** | **31 512** | **40 996** |

TCBs come on top, roughly 0.35 KB each, about 6 KB in total (estimate, not measured).

## Per feature

Change in free INTERNAL heap when the feature is removed, from the fry-like baseline of 21.8 KB.

| feature | free heap gained | how measured | composition |
|---|---:|---|---|
| BLE (compiled out) | **+110.7 KB** | no-BLE image (same overlays + `FUGU_WITH_BLE=n`), text wire on both | 82.6 KB heap (loopTask −74.7, btController −7.7, ipc0 −1.5) + 27.3 KB larger heap (BT `.bss/.data` and IRAM code in DIRAM) |
| `svc off ble` | +0.25 KB | svc | by design: only advertising stops, the stack stays up (`bleConsoleEnd`) |
| Wi-Fi off (`wifi off 3`) | +55.3 KB | console | also stops mqtt, tele, ftp, telnet, scope and mdns. Minus those: **≈28.5 KB** for the Wi-Fi driver, its task and netif. IDF logged `timeout when WiFi un-init`, so some may stay allocated. tcpip/lwIP core stay up |
| mqtt | +17.0 KB | svc | mqtt_task stack 8.7 KB + TCB, loopTask −12.4, tcpip −3.4 (socket), task-held 0.8 |
| mdns task | +5.3 KB | one boot where mdns failed to start ("Failed adding service") | stack 4.35 KB + TCB + records |
| tele | +4.6 KB | svc | teleflush stack 4.35 KB + TCB. 1.5-3.7 KB of static buffers stay after `svc off` |
| tele binary vs text wire | +3.9 KB | `tele.conf binary` 1 → 0 | |
| scope / ftp / telnet (idle) | +0.35 / +0.24 / +0.21 KB | svc | listening socket and mDNS record. An FTP session peaks at ~20 KB (earlier note) |

Console connections, sampled every second over serial while the client was connected:

| connection | cost while connected | after disconnect |
|---|---:|---|
| MQTT console session | none visible | none |
| BLE console, idle | 0.6 KB | 0.5 KB stayed in one run, recovered in the other |
| BLE console + `tele-ble 1` stream | 6.0 KB | recovered |

## Budget

| item | KB | basis |
|---|---:|---|
| task stacks | 72.5 | measured block sizes |
| TCBs | ~6 | estimate |
| BLE without its two stacks | ~73 | BLE delta − 8.7 KB stacks − TCBs |
| Wi-Fi without its stack | ~24 | Wi-Fi-off delta − stack − TCB |
| MQTT without its stack | ~8 | |
| telemetry (binary wire + static buffers) | ~7.6 | |
| scope, ftp, telnet, mdns records | ~1.5 | |
| remainder: Arduino core, app objects, lwIP core, NVS, littlefs, esp_timer | ~16 | allocated 208.6 KB − the rows above |

The remainder is the part no measurement here separates. Each figure is a difference of medians,
good to about ±1 KB.

Compared with the earlier estimate (stacks ~68, Wi-Fi 25-35, BLE 30-40, lwIP 10-20 KB): stacks and
Wi-Fi match, but BLE is 2-3× larger than estimated (≈83 KB heap plus 27 KB of static RAM).

## Levers

1. **BLE configuration, about 8-12 KB (estimate, unmeasured).** `sdkconfig.ble` sizes NimBLE
   for throughput: `TRANSPORT_ACL_FROM_LL_COUNT=24`×255 B, `MSYS_2` 24×320 B, `MSYS_1` 12×256 B,
   `TRANSPORT_EVT_COUNT=30`, and `BT_CTRL_BLE_MAX_ACT=6` for a single-connection peripheral. Halving
   the ACL/MSYS pools saves ~8 KB of buffers, and `MAX_ACT` 6→2 saves controller memory. Bench
   check: OTA throughput (`ota-ble`) and the telemetry stream must not drop. The structural option
   is dropping the Arduino BLE wrapper for raw NimBLE (unmeasured), or a no-BLE fleet variant
   (+110 KB).
2. **Stack sizes, about 10-12 KB.** 41 KB of stack was never touched on the bench. Candidates:
   loopRt 8.7 → 4.5 KB (−4), mqtt_task 8.7 → 6 KB (−2.5; the code comment records an overflow
   at ~6 KB, so keep margin), esp_timer, sys_evt and arduino_events each to ~2.5-3 KB (−1.8 each).
   Precondition: read `tasks` on fry/flat after days of runtime (a read-only command) and keep
   ≥1.5 KB of headroom over the live high-water mark.
3. **Wi-Fi/lwIP buffers, about 5-8 KB (estimate).** `ESP_WIFI_STATIC_RX_BUFFER_NUM=8` (≈1.6 KB
   each, allocated at init), dynamic RX/TX 32/32, `MGMT_SBUF_NUM=32`, AMPDU RX window 6. Static RX
   8 → 4 saves ~6.4 KB at the cost of RX burst tolerance (OTA download speed). The
   `LWIP_TCP_SND_BUF/WND` of 5.76 KB caps per-socket use but is not reserved up front.

Smaller items:
- The binary wire costs 3.9 KB more than text.
- teleflush keeps 1.5-3.7 KB of static buffers while the service is off.
- A BLE stream connection costs 6 KB transiently. That plus the binary wire plausibly explains
  fry's min-ever of 10-13 KB.

## Caveats (bench vs live)

- Vconv plant instead of INA226/ADS + MCPWM: the sensor and gate-driver objects differ (the static
  size is equal within 0.1 KB). There is no real BMS/charger MQTT traffic and no Home Assistant
  load, and the plant made 0 W (`START blocked: bf-fail` in the mock).
- No live clients: no rpi BLE relay, dashboard or telnet sessions. fry's min-ever reflects those
  transients.
- The BLE and Wi-Fi totals include what `svc off` cannot free (the stack stays up). They were
  measured by compiling BLE out and by `wifi off`, both of which also remove dependent services.
  The table separates those using the per-service deltas.
- The attribution of ipc0, main_task and the startup lumps is inferred from FreeRTOS behaviour
  before the scheduler starts, not traced. Per-call-site attribution would need heap tracing.
- The `heap tasks` walk holds the heap lock while it walks every block, takes the kernel lock in
  `uxTaskGetSystemState`, and runs `heap_caps_check_integrity_addr` per task. It is for the bench
  only.

## Tooling

- `heap tasks` and the extra `heap` line (`INTERNAL allocated … in … blocks, free …`) exist only
  with `CONFIG_HEAP_TASK_TRACKING` (`sdkconfig.heaptrack`). In the fleet config, `cli.cpp.obj` is
  byte-identical to main (disassembly with relocations; text 96 466 B), and no linker wrap is added.
- **IDF 6.0.3 task-tracking hooks hang the S3.** `heap_caps_update_per_task_info_alloc` takes a
  FreeRTOS mutex on every malloc. `lock_acquire_generic` (lazy libc lock init) mallocs inside
  `portENTER_CRITICAL`, so a contended take blocks with interrupts masked. First image: INT_WDT
  in `setup()` → `consoleInit()` → `uart_driver_install` (coredump decoded, then erased from the
  device). Second image skipped the hooks inside critical sections but hung silently after boot:
  no Wi-Fi, BLE advertised without accepting connections, rolled back on reset. Final design: the
  three hooks are `--wrap`ped to no-ops, and `heap tasks` walks the owner tags with
  `heap_caps_get_per_task_info`. This gives current figures per task, with no per-task peak and no
  locking in malloc.
- Live per-task heap peaks are therefore not available. Use `heap` min-ever plus the deltas above.
