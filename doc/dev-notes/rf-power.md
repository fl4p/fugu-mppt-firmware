*this document is an LLM generated placeholder*

# RF power saving (WiFi + BLE modem-sleep)

Both radios idle hot if left at full duty. Two independent modem-sleep mechanisms cut that.
Core 1 (RT: ADC/MPPT/PWM) is untouched by either — both radios live on core 0.

## WiFi

`WiFi.setSleep(...)` in `src/tele/telemetry.cpp::connect_wifi_async()` (baseline) and
`src/main.cpp::networkLoopTick()` (day/night switch):

- Baseline on connect: `WIFI_PS_MIN_MODEM` — modem sleeps between DTIM beacons (~100 ms),
  console/MQTT/OTA stay responsive. (Default was `WIFI_PS_NONE` — radio always on.)
- Idle > 5 min (not converting: `converter.disabled() || tracker._curPower < 10`): deepen to
  `WIFI_PS_MAX_MODEM` to cut night standby draw. (This `converting` test once also gated WiFi
  reconnect, but that gate was dropped — RF bring-up adds only ~3 ms RT-loop lag, well under the
  watchdog; reconnect is now gated only on `ucTemp < 80`.)
- Back to `MIN_MODEM` the instant PV power returns. `psMode` is reset on the WiFi-up edge
  because a reconnect re-applies the `MIN_MODEM` baseline.
- Transitions log `WiFi power save: MAX_MODEM (night/idle)` / `MIN_MODEM (active)`.
- Skipped while bsync runs (it holds `WIFI_PS_NONE`); re-asserted once it stops.

Why power-gated on PV rather than a clock: robust to NTP failure, handles overcast, reuses a
threshold the firmware already trusts. The 5-min hysteresis stops flapping at dawn/dusk.

Not done: `WIFI_PS_MAX_MODEM` as the *baseline* (adds latency to unsolicited inbound TCP —
telnet/MQTT-cmd/OTA), and `CONFIG_PM_ENABLE` (CPU DFS/light-sleep would jitter the RT loop).

## BLE

Controller modem-sleep via `CONFIG_BT_CTRL_MODEM_SLEEP` / `..._MODE_1` in `sdkconfig.ble`.
BLE-on previously ran the S3 ~9 °C hotter (RF/PLL never sleeping). Advertising is left at the
NimBLE fast default (~30–60 ms) by day (the idle profile below slows it at night) and the device only ever advertises (peripheral, no scanning),
so there's no scan cost to trim.

## Idle profile (2026-11-15)

`idleProfileTick()` in `src/main.cpp` widens the night switch. After 5 min with the converter off and
nothing else active (PSU, manual duty > 0, sweep/ADC calibration, rect-cal, measure-coil, bsync, HTTP or
BLE OTA, USB drive), it sets: CPU `board.conf::idle_cpu_mhz` (default 80), connectable BLE adv
`ble.conf::idle_adv_ms` (default 1000), telemetry point interval `tele.conf::idle_interval_ms`
(default 5000). 0 disables a part. It leaves on the first network-loop pass that sees any of these
active; WiFi PS keeps its own >= 10 W predicate. Log: `idle profile on: cpu 80 MHz, adv 1000 ms,
tele 5000 ms, WiFi MAX_MODEM` / `idle profile off (converter): ...`. Any `pwr` cpu/pm/ble-adv/tele/
wifi-ps knob suspends it until reboot.

Bench S3 die temperature vs 160 MHz / MIN_MODEM / default adv: adv 1000 ms -2.0 K, 80 MHz -1.9 K,
MAX_MODEM within noise, all combined (plus a slower RT loop) -5.1 K; both radios off -6.2 K.
Compiled only (2026-11-15), not yet run on hardware.

## Measuring

`etc/chiptemp.py` reads the on-die sensor — the real metric for any RF power change. Compare
before/after at the same ambient; modem-sleep deltas are single-digit °C.
