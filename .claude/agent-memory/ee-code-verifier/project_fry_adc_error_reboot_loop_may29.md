---
name: fry-adc-error-reboot-loop-may29
description: fry's "ADC error" on 2026-05-29 = internal continuous-ADC (vin ch3 + ntc ch7) DMA stalls every boot, resetPeripherals doesn't revive it, device reboots ~15s; real root is a heap/vtable corruption being hunted with HEAP_POISONING. gpio_install warning is benign.
metadata:
  type: project
---

fry "E (NNNN) main: ADC error" on 2026-05-29.

**Failing backend:** internal ESP32-S3 continuous ADC (`ADC_ESP32_Cont`, src/adc/adc_esp32_cont.*), NOT INA226/ADS. Pattern on this DMA = `{atten=3, ch=3}` (vin) + `{atten=3, ch=7}` (ntc) + duplicate vin. Signature: `adc_esp32: no ADC samples for ~1000 ms` → isGood() false → sampler returns AdcError → `main: ADC error` + `mppt: backoff 5s`. ("backoff 5s" is the default backoffSec=5 in shutdownDcdc; the AdcError path passes stopAndBackoff(16) but that only sets delayStartUntil, NOT the log value — do not read "5s" as old firmware.)

**Behavioral contrast that localizes it:** flat hits the same `no ADC samples` once per boot, calls `resetPeripherals: restarting continuous ADC` ONCE, and recovers (444 sps, charges normally). fry's resetPeripherals does NOT revive the DMA — it re-stalls immediately and the device reboots at ~15s uptime, looping (boots seen ~17:55:37, :55:46, :56:03, :56:19, :56:36, :56:51, :57:01, :57:49). The ~15s reboot had NO panic/backtrace in the havan MQTT log (that path dies on reboot; the real backtrace goes to USB-CDC only).

**Two candidate causes the user flagged — verdict:**
- `gpio: gpio_install_isr_service(526): already installed` = BENIGN. It maps to main.cpp:284 (`ESP_ERR_INVALID_STATE` → "already installed (likely on CPU0); RT alert pinning skipped"). The dangerous version was the nested-IPC *deadlock* ([[project_crash_rootcauses]]), already fixed in 97f47a8; that was a boot HANG (silence), not an ADC-error spam. Red herring.
- `HEAP_POISONING=COMPREHENSIVE` (working-tree-only in sdkconfig.defaults, NOT committed) is a DIAGNOSTIC, not a cause. Its own comment documents the real bug: fry crashed in `wifi_nvs_compare_cfg_diff->nvs_get_u8` because a WiFi-NVS handle's **vtable pointer was overwritten with `_xt_lowint1`** (an ISR return addr) = stack-overflow-into-heap or UAF. The ADC stall + ~15s reboot loop is most likely a downstream symptom of that corruption (or the poisoning abort firing). Heap poisoning only adds ~5-10% perf + 16B/alloc; it does not by itself kill the IRAM-safe ADC ISR.

**A/B not yet performed** — fry was held by the user's `idf.py monitor` on /dev/cu.usbmodem1201 (the USB-CDC port; classic DTR/RTS reset does nothing on USB-Serial/JTAG, esptool couldn't open it busy). NAT telnet (192.168.1.173:233) timed out because fry reboots every ~15s. Reach fry live only via havan MQTT log or by taking the USB port.

Self-heal code refs: AdcError handler main.cpp:554-569 (resetPeripherals throttled to 8s); the never-got-a-sample reboot at main.cpp:594 (60s) and :599-605 (`nowMs>20000` → disable+restart at 15min); resetPeripherals in adc_esp32_cont.h:107 (deinit+start). Related: [[project_adc_sampler]], [[project_adc_sampler]], [[project_ina226_timeout_loop_latency_shutdown]].
