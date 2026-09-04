*this document is an LLM generated placeholder*

# Beacon-sniffing MCPWM clock sync (`bsync`)

Locks the switching clocks of multiple converters to a shared timebase recovered from 802.11
beacons — **receive-only**: the device never associates or transmits, so it works with WiFi
"off" (no TX bursts on the 3V3 rail during precision measurements). Implementation:
`src/sync/bsync.{h,cpp}`, driver hooks in `src/pwm/mcpwm.h` (`setPeriodTicks`, `count`,
`update_period_on_empty`). The dedicated beacon source is documented in
[bsync-beacon-node.md](bsync-beacon-node.md).

## Mechanism

1. **Common-view offset**: both devices sniff the *same* AP's beacons (promiscuous mode, MGMT
   filter, BSSID match). Each beacon carries the AP's 64-bit TSF timestamp (first field of the
   body, offset 24); the RX hardware stamps arrival with `rx_ctrl.timestamp` — **its own µs
   clock** (bench-measured: not the TSF epoch, not esp_timer, but crystal-locked to esp_timer
   at a constant offset; ticks with or without association — the STA TSF only runs while
   associated, so it is NOT used). The rx clock is bridged to the esp_timer domain by a
   max-filter of `(rxExt − espAtCallback)`: callback latency is strictly positive, so the max
   converges to the true offset minus the latency floor; the floor is identical firmware on
   identical chips and cancels chip-to-chip. `offset = espTimer − AP` is then tracked by an
   alpha-beta filter (offset + drift rate), so the estimate predicts through beacon gaps and
   doesn't lag the crystal-drift ramp. The AP's own crystal error is common-mode.
2. **Phase grid**: target = shared time × ticks/µs modulo (P + ½) ticks (P = nominal period,
   e.g. 4103 @ 39 kHz / 160 MHz). The half-tick grid centers each device's steady-state trim
   inside the {P, P+1} dither range regardless of crystal sign. Math in half-ticks (integral
   modulus 2P+1), int/frac split keeps doubles exact.
3. **Servo**: 1 Hz on the phase error (pair-read esp_timer + MCPWM counter, skew-bounded
   retry — no wifi API in the loop). `u = 0.5 + P·r + kp·e + ∫ki·e`: the measured drift rate
   `r` feeds the frequency trim **forward** — the P-term's pull-in range is only ~2.5 ppm and
   the wrapped phase error integrates to ~zero, so a PI alone cannot capture a tens-of-ppm
   crystal offset (observed live: stuck slipping cycles at +13 ppm until the feedforward was
   added). Coasts on `0.5 + P·r + iAcc` when beacons go stale (>3 s); never reports lock
   without a fresh timebase.
4. **Actuator**: 1 kHz first-order sigma-delta dithers the period register between P and P+1
   (mean = P + u). Updates latch on TEZ (glitch-free). **Never below nominal P** — a
   comparator at P−1 would miss its event in a shrunken period and hold the LS gate high for a
   full cycle (shoot-through on HiLi boards).

Expected performance at 39 kHz: zero average frequency drift, relative phase bounded ~±1–3 µs
(±15–40°), added cycle-to-cycle jitter 1 tick (6.25 ns). Not sufficient for degree-level
interleaved current sharing — that needs a sync wire (MCPWM GPIO sync input).

## Setup

```
set-config bsync.conf bssid aa:bb:cc:dd:ee:ff   # the sync AP, same on all devices
set-config bsync.conf channel 6
set-config bsync.conf enabled 1
svc on bsync
svc                                              # statusDetail: lock state, e, u, drift, counters
```

`phase_us` shifts one device on the grid (e.g. half a period ≈ 12.8 µs for 180° interleave).
Requires `converter.conf pwm_driver=mcpwm`.

**`wifi off` interaction (deliberate):** while bsync is enabled it keeps the radio up in
unassociated STA + promiscuous mode even after `wifi off` — that combination (association/TX
torn down, RX-only sniffer alive) is the intended radio-quiet measurement state. It logs a
warning each time it re-arms the radio. For full radio silence run `svc off bsync` too. While
associated, the conf `channel` cannot be applied (the STA link owns the channel) — a warning
is logged and the sync AP must share the STA's channel.

## Bench verification (2026-08-04, fboost boost bench @70 V, FRITZ!Box ch1)

- [x] **TSF does NOT run in unassociated sniffer mode** — confirmed live (`esp_wifi_get_tsf_time`
  returns 0 until association). This killed the original TSF design; replaced by the rx-clock →
  esp_timer bridge above, which needs no association and no wifi API in the servo.
- [x] **`rx_ctrl.timestamp` is its own clock** — measured: `rx − esp_timer` constant (477.37 ms
  that session, arbitrary per wifi session), crystal-locked (<1 ppm relative); consecutive
  beacon stamps land exactly on the 102 400 µs TBTT grid (hardware-latched).
- [x] **Lock**: e = ±0.6…2.3 µs sustained, `u` settles at exactly 0.5 + P·13 ppm; single-device
  phase vs the AP grid. Locked while the converter boosts at 70 V (dither glitch-free).
- [x] **RX-only / `wifi off`**: stays locked through `wifi off`; self-heal re-arms the radio
  unassociated with the documented warning; beacons keep flowing without any TX.
- [x] **Known-bad bssid** (guard check): placeholder BSSID → `acquiring`, `beacons=0`, never
  claimed lock.
- [x] **PS interference**: association resets power-save to MIN_MODEM which sleeps through
  most beacons — onTick re-forces `WIFI_PS_NONE` every second.
- [x] **Two-device relative sync** (electronic): fboost e=+0.8 µs and fbuck e=−0.9 µs locked
  simultaneously to the same grid → relative phase 1.7 µs (~24° @39 kHz), zero average
  frequency offset (crystals differ ~1.6 ppm). fboost also auto-relocked from conf after an
  unattended reboot. Scope shot of both switch nodes (trigger on one, other edge stands
  still) still worth taking for the record.
- [ ] **Coast**: kill the AP; state → `coasting`, phase slips slowly (crystal temp drift
  only), relocks on AP return without a duty glitch.
- [ ] **Timer index assumption**: `MCPWM_SyncLeg::count()` reads hw timer 0 of the group —
  held on the bench (grid math locks, so the count is real); re-check if a second leg/capture
  timer ever allocates first.
- [ ] RX-only measurement noise floor vs radio fully off (PS_NONE runs the RF continuously —
  chip runs warmer, cf. BLE modem-sleep thermal note).

Beacon accept rate on the bench was 0.2–3/s (congested channel + converter RF right at the
antenna), far below the 10/s TBTT — the alpha-beta prediction bridges the gaps and lock held
regardless.

## Oscilloscope campaign results (2026-08-04, dedicated beacon node)

Ground truth measured with a two-channel scope on both converters' LS gates (×10 probes),
relative edge timing extracted per trigger event, 10-minute runs, dedicated XIAO beacon node
on a clear channel (~60 frames/s: softAP beacons every 100 TU + esp_timer-injected frames
every 20 ms, same BSSID).

| stream, bw            | fast σ | slow σ | p2p (10 min) |
|-----------------------|--------|--------|--------------|
| mixed 60/s, bw=0.2    | 0.23 µs| 0.39 µs| 2.5 µs       |
| mixed 60/s, bw=0.05   | 0.17 µs| ~1.5 µs| 5.7 µs       |
| **hw-only 10/s, bw=1.2** | **0.12 µs** | **0.09 µs** | **1.0 µs** |

Findings, in causal order:

- Three firmware fixes were prerequisites for clean steady state: reject-streak **median
  revalidation** (a re-seed from one sample caused µs phase steps), **freezing the rx→esp
  bridge** after acquisition (max-filter ratchet rebased mid-run), and **rSmooth** (~30 s EWMA
  of the alpha-beta drift rate on the feedforward path — raw per-frame `r` is FM noise).
- bw=0.05 is below the crystals' relative drift-wander corner: the phase random-walks between
  corrections (~6–8 min hunting, never converges). Don't go that low.
- **Injected frames are the dominant noise source.** `esp_wifi_80211_tx` frames pick up
  software scheduling/queueing jitter that common-view geometry does not cancel; hardware
  TBTT beacons (MAC-scheduled, hw TSF-stamped — the timestamp field of injected beacons is
  also hw-overwritten, but their *timing* is soft) are ~4× cleaner at 1/6 the rate.
  `bsync.conf::hw_only=1` selects them by frame length (full-IE beacon >80 B vs 47 B injected
  skeleton). Best validated config: **hw_only=1, bw=1.2** → σ 0.16 µs, p2p ±0.5 µs (~2° σ at
  39 kHz).
- Production config: **hw_only=1, bw=1.2, node at the stock 100 TU** (the injector can go).
  IDF validates `wifi_ap_config_t::beacon_interval >= 100 TU`, so more hw-beacon rate would
  need patching the check or poking the TBTT register — not worth it: slow wander already
  sits below the per-beacon fast noise, so the loop is stamp-noise-limited, not rate-limited.

Measurement traps (they cost runs): the servo's reported `e` over-states physical error
(~×4.5); polling the console during a capture stretches the 1 kHz dither slots (core-0
esp_timer starvation) → µs ripple; ×100 probes are SNR-starved on 3 V gates (fast σ inflates
3×); a SW node reads soft/load-dependent at light load — probe the LS gate.

A runtime switching-frequency change (`pwm-freq`) is **refused while this service is Running**: the
servo caches `nomPeriod_`/`ticksPerUs_` at start and dithers the period register around them at 1 kHz,
so it would drive the period straight back to the old frequency. `svc off bsync` first. `phase_us` is a
TIME, so a 180-degree interleave value has to be recomputed after a frequency change either way.
