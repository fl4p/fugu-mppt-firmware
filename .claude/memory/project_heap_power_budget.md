---
name: project_heap_power_budget
description: Heap breakdown (BLE ~110 KB, WiFi ~28, mqtt 17; fleet free ~36 KB after 894) and idle-power levers (telemetry interval biggest, idle profile, BLE modem sleep)
metadata:
  type: project
---

## Heap breakdown
2026-11-23, bench139c fry-like setup (`heap tasks`, bench-only `sdkconfig.heaptrack`; plans/heap-breakdown-2026-11-23.md).
BLE compiled in ~110 KB (`svc off ble` frees 0.25 KB); Wi-Fi ~28.5 KB; mqtt 17 KB (8.7 KB stack); tele 4.6 KB, binary
tele +3.9 KB. 41 KB stack untouched at 0 W. Levers: BLE buffers 8-12 KB, stack trims 10-12 KB.
IDF 6.0.3 `CONFIG_HEAP_TASK_TRACKING` hooks hang/INT_WDT the S3 (command stubs them). `STATIC_RX_BUFFER_NUM` is dead:
arduino-esp32 forces 4/32.
21d62bd6: OTA releases the other radio (+~65 KB for Wi-Fi OTA). Live on fry+flat as 894-gcc30ca19 (11-23 18:15): free
35.8 KB, min-ever ~29 KB, DMA min 20.8 KB (before: min-ever 10-13 KB, flat DMA min 2.6 KB).
**How to apply:** pick levers here before adding RAM users. See [[project_ota]].

## Idle-power levers
2026-11-15.., bench 139C die-temp deltas (proxy only; get a real current measurement before ranking small levers).
Tooling: `pwr` (`CONFIG_FUGU_WITH_PWR_KNOBS`, `sdkconfig.pwr`/`sdkconfig.pm`), `etc/pwr_sweep.py`, data
`doc/research/idle-power-sweeps/`.
- Telemetry 20 ms → 5 s interval: -6.7 K (biggest). WiFi PS none +8.7 K; BLE off -3.2 K, adv 1 s -2.0 K; 80 MHz
  -1.9 K; RT 100 Hz -1.2 K; WiFi off -1.9 K; WiFi TX power: no effect.
- PM DFS 80/40 ~31 C; auto light sleep 26 C (70 % asleep) but USB-JTAG vanished, GPIOs float (gate pins need
  `gpio_sleep_sel_dis`), ADC/MCPWM must stop at night. Not firmware behaviour yet; deep sleep only studied.
- Idle profile 488f3360 (after 5 min idle: `board.conf::idle_cpu_mhz`, `ble.conf::idle_adv_ms`,
  `tele.conf::idle_interval_ms`, 0 disables). Fixes: 1604eaf8 (armed sweep counted as busy), fd6ad79a (start waits for
  boot clock; rate guard keeps 160 MHz if rate at idle clock <97 %). Dawn-start bug fixed, confirmed on fry 11-19;
  both converters on idle_cpu_mhz=80 since 11-19. flat at 80 MHz only reaches ~1070 of 1475 sps (guard handles it).
**Lesson:** test idle entry/exit with the sweep armed (st=START,1, mock held via `hv_v_rh`), not via `dc 0`.
Open items: plans/2026-11-18-idle-power-open-items.md. Related: [[project_rt_loop_latency]].

## BLE modem sleep
FIXED in `sdkconfig.ble`: BT controller had no modem sleep (+9 °C). `CONFIG_BT_CTRL_MODEM_SLEEP=y` + `_MODE_1` (only S3
mode), LPCLK main XTAL, TX 0 dBm = `DFT_TX_POWER_LEVEL_N0`. BLE-on 64 → 56 °C. Gotcha: an existing `sdkconfig` overrides
the overlay; never set `_EFF` symbols. Chip temp = 2nd temp on the status line (`etc/chiptemp.py`). Generic BLE/light
sleep thermal facts: Canonical: ~/dev/kb/esp32/realtime-and-power.md (Bluetooth and light-sleep thermal limits).
