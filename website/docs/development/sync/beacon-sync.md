---
title: "Beacon Clock Sync (bsync)"
sidebar_position: 1
---

# Beacon-sniffing MCPWM clock sync (`bsync`)

Beacon sync locks the switching clocks of multiple converters to a shared timebase recovered
from 802.11 beacons. The mechanism is receive-only. It needs neither association nor
transmission, so it keeps working after `wifi off`, with no TX bursts on the 3V3 rail during
precision measurements. It doesn't turn the station link off by itself. For the radio-quiet
state, run `wifi off` (see [Radio state](#radio-state)).

The implementation is in `src/sync/bsync.{h,cpp}`, with driver hooks in `src/pwm/mcpwm.h`
(`setPeriodTicks`, `count`, `update_period_on_empty`). The dedicated beacon source is documented
in [beacon-node.md](beacon-node.md).

## Mechanism

The sync runs in four stages:

1. Common-view offset: both devices sniff the *same* AP's beacons (promiscuous mode, MGMT
   filter, BSSID match). Each beacon carries the AP's 64-bit TSF timestamp (first field of the
   body, offset 24). The RX hardware stamps arrival with `rx_ctrl.timestamp`, which is its own µs
   clock. Bench measurements show that this clock is neither the TSF epoch nor esp_timer. It is
   crystal-locked to esp_timer at a constant offset and ticks with or without association. The
   STA TSF runs only while associated, so the sync doesn't use it.

   A max-filter of `(rxExt − espAtCallback)` bridges the rx clock to the esp_timer domain.
   Callback latency is strictly positive, so the max converges to the true offset minus the
   latency floor. The floor comes from identical firmware on identical chips, so it cancels
   chip-to-chip. An alpha-beta filter (offset + drift rate) then tracks
   `offset = espTimer − AP`. The estimate therefore predicts through beacon gaps and doesn't lag
   the crystal-drift ramp. The AP's own crystal error is common-mode.
2. Phase grid: target = shared time × ticks/µs modulo (P + ½) ticks, where P is the nominal
   period (e.g. 4103 @ 39 kHz / 160 MHz). The half-tick grid centers each device's steady-state
   trim inside the {P, P+1} dither range regardless of crystal sign. The math runs in half-ticks
   (integral modulus 2P+1), and an int/frac split keeps doubles exact.
3. Servo: a 1 Hz loop acts on the phase error. It pair-reads esp_timer and the MCPWM counter
   with a skew-bounded retry and calls no wifi API in the loop. The control law is
   `u = 0.5 + P·r + kp·e + ∫ki·e`, where the measured drift rate `r` feeds the frequency trim
   forward.

   The feedforward is necessary because a PI alone can't capture a crystal offset of tens of
   ppm. The P-term's pull-in range is only ~2.5 ppm, and the wrapped phase error integrates to
   ~zero. Without the feedforward, the servo slips cycles at a +13 ppm crystal offset. The
   feedforward uses `rSmooth`, a ~30 s EWMA of the drift rate, because the raw per-beacon `r` is
   noisy. When the last accepted beacon is older than 3 s, the servo coasts on
   `0.5 + P·rSmooth + iAcc`. It never reports lock without a fresh timebase. The state is
   `locked` while the phase error is below 5 µs, otherwise `acquiring` (or `coasting`).
4. Actuator: a 1 kHz first-order sigma-delta dithers the period register between P and P+1
   (mean = P + u). Updates latch on TEZ, so they're glitch-free. The period never goes below
   nominal P. In a shrunken period, a comparator at P−1 would miss its event and hold the LS
   gate high for a full cycle, which means shoot-through on HiLi boards.

At 39 kHz the expected performance is zero average frequency drift, relative phase bounded
~±1–3 µs (±15–40°), and 1 tick (6.25 ns) of added cycle-to-cycle jitter. Degree-level
interleaved current sharing needs a sync wire (MCPWM GPIO sync input).

## Setup

Beacon sync requires a `CONFIG_FUGU_WITH_BSYNC=y` build (default off) with networking and the MCPWM gate driver. Run
these commands on each device:

```
set-config bsync.conf bssid aa:bb:cc:dd:ee:ff   # the sync AP, same on all devices
set-config bsync.conf channel 6
set-config bsync.conf hw_only 1                  # with a beacon node that also injects frames
svc on bsync                                     # starts it and saves enabled=1
wifi off                                         # radio-quiet: drop the STA link (see below)
svc list                                         # DETAIL column: bsync status
```

The `bsync` status in `svc list` has this form:

```
locked e=+0.8µs u=0.553 drift=+13.0ppm beacons=1234 age=0.1s rej=2 dom=0 rv=0/0 d=477370µs
```

The fields are the lock state, the phase error `e`, the servo output `u` (mean period offset in ticks), the
estimated crystal drift against the AP, the accepted beacons and the age of the last one, rejected beacons,
rx-clock epoch jumps (`dom`), revalidations and the ones that rebased (`rv`), and the rx→esp_timer bridge offset `d`.

The service refuses to start without a valid `bssid`, with a `channel` outside 1–14, on a build without the MCPWM gate
driver, on a wired-sync follower, or while a `pwm-freq` change is still pending. The loop gains scale with `bw`
(default 1.0, clamped to 0.05–4); see [bsync.conf](../../reference/config/bsync.md).

:::warning
A bare `wifi off` is persisted (NVS) and survives reboots until `wifi on`; `wifi off <minutes>`
is temporary. After a bare `wifi off` the device is reachable only over a non-Wi-Fi console
(USB/serial, or BLE if built in), so have one at hand before you send it over telnet or MQTT.
:::

`phase_us` shifts one device on the grid. For example, half a period ≈ 12.8 µs gives a 180°
interleave.

### Radio state

While bsync is enabled, it deliberately keeps the radio up in unassociated STA + promiscuous
mode, even after `wifi off`. That combination, with association and TX torn down and the RX-only
sniffer alive, is the intended radio-quiet measurement state. Bsync logs a warning each time it
re-arms the radio. For full radio silence, also run `svc off bsync`.

While the device is associated, the STA link owns the channel, so bsync can't apply the conf
`channel`. It logs a warning, and the sync AP must share the STA's channel.

### Changing the switching frequency

While this service is Running, it refuses a runtime switching-frequency change (`pwm-freq`).
The servo caches `nomPeriod_`/`ticksPerUs_` at start and dithers the period register around them
at 1 kHz, so it would drive the period straight back to the old frequency. Run `svc off bsync`
first. `phase_us` is a time, so you have to recompute a 180-degree interleave value after a
frequency change either way.

## Bench validation

The service was validated on a boost converter at 70 V, with a consumer AP on channel 1. These
behaviours were confirmed:

- TSF: `esp_wifi_get_tsf_time` returns 0 until association, so the STA TSF cannot serve an
  unassociated sniffer. The rx-clock → esp_timer bridge above replaces it and needs no
  association and no wifi API in the servo.
- Rx clock: `rx_ctrl.timestamp` is its own clock. The measured `rx − esp_timer` is constant
  within a Wi-Fi session (477.37 ms in one session, arbitrary per session) and crystal-locked
  (<1 ppm relative). Consecutive beacon stamps land exactly on the 102 400 µs TBTT grid
  (hardware-latched).
- Lock: e = ±0.6…2.3 µs sustained, and `u` settles at exactly 0.5 + P·13 ppm. This is
  single-device phase against the AP grid. The servo stays locked while the converter boosts at
  70 V (dither glitch-free).
- RX-only operation: the servo stays locked through `wifi off`. Self-heal re-arms the radio
  unassociated with the warning described under [Radio state](#radio-state), and beacons keep
  flowing without any TX.
- Wrong BSSID: a placeholder BSSID gives `acquiring` and `beacons=0`, and the servo never claims
  lock.
- Power save: association resets power-save to `WIFI_PS_MIN_MODEM`, which sleeps through most
  beacons, so the service re-forces `WIFI_PS_NONE` every second.
- Two devices: a boost board (e=+0.8 µs) and a buck board (e=−0.9 µs) locked simultaneously to
  the same grid. The relative phase was 1.7 µs (~24° at 39 kHz) with zero average frequency
  offset (the crystals differ by ~1.6 ppm). A board relocks from its conf after a reboot.

The beacon accept rate in that setup was 0.2–3/s, far below the 10/s TBTT, because of a
congested channel and converter RF right at the antenna. The alpha-beta prediction bridged the
gaps and lock held.

These checks are still open:

- Coast: with the AP gone, the state should go to `coasting`, the phase should slip slowly
  (crystal temperature drift only), and the servo should relock on AP return without a duty
  glitch.
- Timer index: `MCPWM_SyncLeg::count()` reads hardware timer 0 of the group. This holds while the
  gate leg allocates the first timer; re-check it if another timer is ever allocated first.
- Noise floor: RX-only measurement noise compared with the radio fully off. `WIFI_PS_NONE` keeps
  the RF on continuously, so the chip runs warmer.

## Oscilloscope results

A two-channel scope on both converters' LS gates (×10 probes) measured the ground truth. Relative
edge timing was extracted per trigger event over 10-minute runs. A dedicated
[beacon node](beacon-node.md) on a clear channel sent ~60 frames/s: softAP beacons every 100 TU
plus esp_timer-injected frames every 20 ms, with the same BSSID. The table lists the measured
phase noise for each beacon stream and loop bandwidth:

| stream, bw            | fast σ | slow σ | p2p (10 min) |
|-----------------------|--------|--------|--------------|
| mixed 60/s, bw=0.2    | 0.23 µs| 0.39 µs| 2.5 µs       |
| mixed 60/s, bw=0.05   | 0.17 µs| ~1.5 µs| 5.7 µs       |
| **hw-only 10/s, bw=1.2** | **0.12 µs** | **0.09 µs** | **1.0 µs** |

The results depend on these factors:

- Three estimator properties are needed for a clean steady state:
  - Reject-streak median revalidation: after 8 consecutive rejected beacons (residual > 500 µs),
    the next 15 offsets are collected and the estimate rebases on their median only if it
    disagrees by more than 500 µs. A re-seed from one sample caused µs phase steps.
  - The rx→esp bridge freezes after acquisition (300 accepted beacons), because the max-filter
    ratchet rebased mid-run.
  - `rSmooth`, the ~30 s EWMA of the alpha-beta drift rate on the feedforward path, because raw
    per-frame `r` is FM noise.
- bw=0.05 is below the crystals' relative drift-wander corner. The phase random-walks between
  corrections (~6–8 min hunting, never converges), so bw should stay above it.
- Injected frames are the dominant noise source. `esp_wifi_80211_tx` frames pick up software
  scheduling/queueing jitter that common-view geometry doesn't cancel. Hardware TBTT beacons are
  MAC-scheduled and hw TSF-stamped, and they're ~4× cleaner at 1/6 the rate. The timestamp field
  of injected beacons is also hw-overwritten, but their *timing* is soft.
  `bsync.conf::hw_only=1` selects the hardware beacons by frame length (frames shorter than
  64 B are dropped; a full-IE beacon is >80 B, the injected skeleton 47 B). The best validated
  config is hw_only=1, bw=1.2, which gives σ 0.16 µs and p2p ±0.5 µs (~2° σ at 39 kHz).
- The recommended config is hw_only=1, bw=1.2, with the node at the stock 100 TU; the injected
  frames are not needed. IDF validates `wifi_ap_config_t::beacon_interval >= 100 TU`, so a
  higher hw-beacon rate would need patching the check or poking the TBTT register. That isn't
  worth it: slow wander already sits below the per-beacon fast noise, so stamp noise rather than
  rate limits the loop.

These measurement pitfalls apply:

- The servo's reported `e` over-states physical error (~×4.5).
- Polling the console during a capture stretches the 1 kHz dither slots (core-0 esp_timer
  starvation), which causes µs ripple.
- ×100 probes are SNR-starved on 3 V gates, and fast σ inflates 3×.
- A SW node reads soft/load-dependent at light load, so probe the LS gate.
