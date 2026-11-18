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
(default 900000 = 15 min, max 3600000). 0 disables a part. It leaves on the first network-loop pass that sees any of these
active; WiFi PS keeps its own >= 10 W predicate. Log: `idle profile on: cpu 80 MHz, adv 1000 ms,
tele 900000 ms, WiFi MAX_MODEM` / `idle profile off (converter): ...`. Any `pwr` cpu/pm/ble-adv/tele/
wifi-ps knob suspends it until reboot.

### Converter start waits for the boot clock (2026-11-17)

Field result that prompted it: flat (INA226, ~1475 samples/s, 0.68 ms period) ran ~1070 samples/s at
80 MHz, left the profile 0.1 s *after* `Converter enabled` and latched `lag=1025 us` (normal dawn starts
845-916 us). fry (512 samples/s) was unaffected.

- **Start inhibit.** `SynchronousConverter::startInhibit` is now a bit set: `InhibitCal` (rect-cal save,
  `pwr pwm`, unchanged between them) and `InhibitIdle`. Core 0 sets `InhibitIdle` *before* lowering the
  clock (then waits 10 ms and checks that no start slipped through, as `pwr pwm` does) and clears it only
  after the boot clock is back and the RT loop has run `kIdleSettleN` = 128 samples and
  `kIdleSettleUs` = 200 ms at it. `pwmPerturb()` is the only place a converter is enabled; it refuses
  while any bit is set, so no mode (MPPT sweep, PSU/PV-sim, `dc`/`+N`, rect-cal, measure-coil) can
  switch at the idle clock.
- **Wake request.** A start refused for `InhibitIdle` sets `converter.startWanted`: in `pwmPerturb()`,
  and in `MpptController::startBlockReasonRt()` when `startBlockReason()` returns only `idle profile`.
  The RT loop gates both start paths with it: the start branch (no sweep armed, `st=START,0`) and the
  top of `MpptController::update()` (sweep armed, `st=START,1`). Until 2026-11-18 only the start branch
  raised it, so with a sweep armed the converter never started (fry, 13 min at dawn). The next
  network-loop pass (`idleProfileTick()` runs first in it) leaves the profile: CPU first, then telemetry
  interval and BLE advert. Log: `idle profile off (start): cpu 160 MHz ...`, then
  `idle profile: start released, <n> samples in <ms> ms at 160 MHz`. While it waits the status line
  logs `START blocked: idle profile`. A blocked network loop delays the start by the length of the
  block; the RT loop re-raises the request every pass, so nothing is lost.
- **Backstops.** `idleProfileTick()` also leaves on its own when `InhibitIdle` is set, the mode is not
  manual and `startBlockReason()` has been exactly `idle profile` for more than 1 s
  (`idleBackstopWake()`), whether or not the RT flag came: log `idle profile off (start, backstop)`.
  That line means an RT start path lacks the wake. `idleCpuTick()` clears `InhibitIdle` when the CPU
  state is `boot` or `pre` (no state that owns it), logging `idle profile: stray start inhibit cleared`
  once per boot.
- A pack held full in daylight (no sweep) still raises the request, so the profile stays out then.

### CPU-part guard

On every entry the profile measures the RT sample rate (Vout `numSamples` over `kIdleRateWinUs` = 3 s,
`esp_timer` timestamps) at the boot clock, switches (start inhibited), skips 0.5 s, and measures again.
Below `kIdleMinRateRatio` = 0.97 of the pre-switch rate it goes back to the boot clock and latches the
CPU part off until reboot (RAM only; BLE adv, telemetry and WiFi stay in the profile):
`idle profile: cpu stays 160 MHz, sample rate 1070/1475 at 80 MHz`. Passing logs
`idle profile: cpu 80 MHz, sample rate 1475/1475`. No usable pre-switch rate (0, sampler not running)
keeps the boot clock and retries every window. A window with a flash write (`fugu_flash_seq`) or an ADC
calibration is not judged; after 4 such windows at the idle clock the rate counts as missing and the
clock is restored. `status`'s `sps` and `pwr`'s `rt=` are not used: the first is a 3 s average of the
same counter shared with the latency watchdog, the second is the ADC's nominal (INA226: boot-measured)
rate, not the loop's. With the continuous internal ADC the counter only drops once the DMA pool
overflows, so there the guard detects lost samples, not added batch latency.

Bench knobs (`CONFIG_FUGU_WITH_PWR_KNOBS`): `pwr idle 0|1|auto` forces the profile off/on through the
same code (blockers still apply; a refused start ends a forced on; not allowed after a suspending knob),
`pwr gap` prints and resets the max RT loop period (`g_app.cmdLoopLag`, latched with the converter on
or off) with the Vout sample rate since the last call, `pwr idle-after <s 5..3600>` shortens the 5 min
idle time of the automatic path (RAM only), `pwr vin <0..2>` scales the fake ADC channel that feeds Vin
(`adc=fake` only). The `pwr` line shows `idle=<on>/<auto|off|on|susp>/<idle-after>s
icpu=<boot|pre|test|low|settle|latched|->` and `vin=<scale>`. `etc/idle_switch_stall.py -p <port> --cycles 20` measures
the stall of the clock switch.

Clock-switch stall (mock board, 1470 samples/s, 20 enter/exit cycles, 2026-11-17, `etc/idle_switch_stall.py`): not measurable. Worst loop gap in the interval containing the switch: 800 µs entering 80 MHz, 765 µs leaving, against 804 µs over the same time without switching. A start from the profile (three runs) peaked at 909-915 µs, a start without it at 901-923 µs. The mock holds its rate at 80 MHz up to 5000 samples/s; at 7000 the guard tripped (6552/7042) and the rate recovered at 160 MHz. Not measured on an INA226 board.
(enter: _ us, exit: _ us, steady at 80 MHz: _ us, baseline at 160 MHz: _ us).

While `g_teleMinPointUs >= 1 s` (`teleSparse()`, idle profile or `pwr tele`), every point carries the
fields otherwise sent every 10th/20th/40th point (temperatures, lag, heap, mppt_state, Ro/Rc/Rb, ...), and
the UDP flush task sends a partial batch at once instead of holding it until ~MSS fills (which at
15 min/point would have held points for hours). Entering idle right after a 20 ms point means the
next point comes one full interval later; on leaving idle the next point goes out immediately. Every
sparse binary point also carries the full symbol table (120 s heartbeat).

Bench S3 die temperature vs 160 MHz / MIN_MODEM / default adv: adv 1000 ms -2.0 K, 80 MHz -1.9 K,
MAX_MODEM within noise, all combined (plus a slower RT loop) -5.1 K; both radios off -6.2 K.
Compiled only (2026-11-15), not yet run on hardware.

## Measuring

`etc/chiptemp.py` reads the on-die sensor — the real metric for any RF power change. Compare
before/after at the same ambient; modem-sleep deltas are single-digit °C.
