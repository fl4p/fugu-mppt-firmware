---
name: project-bsync-beacon-pwm-sync
description: "bsync syncs MCPWM clocks via RX-only beacon sniffing; SCOPE-VALIDATED two-device: hw_only=1 bw=1.2 on dedicated XIAO beacon node → σ 0.09-0.12µs, p2p 1µs @39kHz; injected frames = noise source"
metadata: 
  node_type: memory
  type: project
  originSessionId: 3dee211c-6787-40a5-957b-f6ddf1dc59a8
  modified: 2026-08-04T18:17:49.028Z
---

`bsync` service (src/sync/bsync.{h,cpp}) phase-locks MCPWM switching clocks across converters
via common-view beacon sniffing (promiscuous RX-only). Bench-validated 2026-08-04 on the fboost
boost bench (converting 70 V): **locked e=±0.6–2.3 µs**, u settles at exactly 0.5+P·drift,
stays locked through `wifi off` (self-heal re-arms RX-only radio, logs warning). Grid P+½ tick,
G=2P+1 half-tick math; sigma-delta dithers period {P,P+1} (never below P — LS-comparator skip
= shoot-through). Conf bsync.conf (bssid required, channel, phase_us, kp, ki).

**Hard-won findings (would cost hours to rediscover):**
- **STA TSF does NOT run unassociated** — esp_wifi_get_tsf_time()=0 in sniffer mode. Don't
  design on it.
- **rx_ctrl.timestamp is its OWN µs clock**: not TSF-epoch, not esp_timer, but crystal-locked
  to esp_timer at a constant per-session offset (~477 ms observed); hardware-latched (stamps
  land exactly on the 102 400 µs TBTT grid). Bridge to esp_timer via max-filter of
  (rxExt−espAtCb); latency floor cancels chip-to-chip.
- **PI alone cannot lock**: P-term pull-in ~2.5 ppm, wrapped phase error integrates to ~0 →
  stuck cycle-slipping at +13 ppm crystal offset. Fix = feedforward the alpha-beta drift rate:
  u = 0.5 + P·r + kp·e + iAcc.
- Association resets power-save to MIN_MODEM → sleeps through most beacons; onTick re-forces
  WIFI_PS_NONE every second.
- Beacon accept rate only 0.2–3/s on a congested channel next to a switching converter (vs
  10/s TBTT) — alpha-beta prediction bridges it fine.
- `add_ap()` (telemetry.cpp:79) hit the [[project_crash_rootcauses]] live:
  wifi-add silently didn't persist; workaround = set-config wifi.conf ssid_X / ssid_X_psk.
- Root `sdkconfig` had CONFIG_FUGU_WITH_MCPWM unset (regenerated at some point) — bsync
  compiles itself out silently without it; re-enabled locally 8-04. Check before fry/flat OTA.

**Two-device sync VERIFIED electronically (8-04):** fboost e=+0.8µs + fbuck e=−0.9µs locked
simultaneously → relative phase 1.7µs (~24° @39kHz), zero beat; fboost auto-relocked from conf
after unattended reboot. fbuck runs unassociated-from-boot (conf channel path). fbuck config
modernized in 8076bba (missing charger.conf had aborted setup → converter uninit → `restart`
panics in ledc_stop; defensive fix still owed).

**Scope campaign findings (8-04 afternoon, HaasoscopePro two-channel, XIAO beacon node 60/s):**
- µs-scale phase STEPS root-caused twice: (1) reject-streak full re-seed re-acquired from ONE
  sample (8 rejects = 133ms at 60 frames/s) → fixed with 15-frame median revalidation (f7f4b2d);
  (2) dEst_ max-filter ratchet rebasing mid-run → frozen after acquisition.
- White ±2µs shot noise = raw drift-rate r applied as DIRECT frequency feedforward: per-frame
  alpha-beta gains make r-noise ~36× worse at 60/s vs 10/s (B·resid/dt per frame) → ±1µs/s
  phase slew. Fixed: feedforward uses ~30s EWMA of r (rSmooth_). Proper fix later: rate-
  normalized alpha-beta gains (A~1/rate, B~1/rate²).
- Console polling during capture stretches the 1kHz dither slots (core-0 esp_timer starvation)
  → µs phase ripple. Never poll `svc` during precision runs; consider moving sigma-delta into
  the RT loop.
- `wifi off` NOT persisted: after any reboot, littlefs wifi creds re-associate and drag the
  sniffer channel away from the beacon node (fbuck sat at beacons=0 a whole run). Post-flash
  ritual: `wifi off` + `svc rs bsync`. Durable fix TODO (persist flag or bsync channel-hold).
- **DEFINITIVE numbers (capture 10, both gates ×10, bw=0.2, all fixes active): fast σ 0.23µs,
  slow σ 0.39µs, steady-state σ 0.47µs / p2p 2.1µs** (~7° p2p @39kHz). Earlier fast readings
  0.53/1.57µs were probe artifacts (SW-node soft transition / ×100 SNR). Static inter-board
  offset +4.4µs (phase_us + path, trimmable). Devices' reported e over-states physical error
  (slope 0.22). Scope app tends to freeze mid-run (dup-event md5 check catches it); REMEDY:
  send `STOP\n` then `START\n` to SCPI :32001 — toggles the run button, un-wedges acquisition
  without app restart (source /Users/fab/dev/pv/HaasoscopePro/software/SCPIsocket.py; socket
  has NO channel/trigger/impedance config, only K/IDN/RATES/DEPTHS/START/STOP/SINGLE/FORCE).
  Extractor amplitude gate must be min/max-based, not percentile (290ns pulse = 2% of window).

- **bw=0.05 is TOO LOW (capture 12): fast σ improves 0.23→0.17µs but slow wander blows up to
  σ~1.5µs / p2p 5.7µs** — servo bandwidth drops below the crystals' relative drift-wander
  corner, phase random-walks between corrections (~6-8min hunting, never reaches `locked`).
  **bw=0.2 is the operating point**; optimum maybe 0.1-0.15, untested.
- 32-bit rx-timestamp wrap can seed the 64-bit bridge on the wrong side: fbuck showed
  d=-4294489193µs (= 478103 − 2^32) after a service restart near a wrap. Harmless (constant,
  stays locked) but cosmetic fix candidate in the epoch re-seed.
- **HaasoscopePro SCPI steering IMPLEMENTED (local patch, uncommitted, propose upstream):**
  SCPIsocket.py gained CHAN/OHM/ACDC/ATT/TENX/CHANON/TWOCHAN/GAIN/OFFSET/TRIGLEVEL/TRIGDELTA/
  TRIGPOS/TRIGCHAN/EDGE/DEPTH/TIMEFAST/TIMESLOW/CONFIG? — queued to the GUI thread (Qt widgets
  are thread-affine) and drained by a 20ms QTimer wired in HaasoscopeProQt.open_socket.
  Client helper: scratchpad/scope_ctl.py (Scope().setup_gates() = whole bsync capture config).
  Also fixed: data_channel() min'd depth ACROSS channels, truncating the wider channel's record.
  App must be restarted to load the patch.

- **INJECTED FRAMES ARE THE NOISE SOURCE (capture 13, 8-04): hw_only=1 (softAP TBTT beacons
  only, ~10/s) + bw=1.2 → fast σ 0.12µs, slow σ 0.09µs, p2p 1.0µs over 10min** — 4× less slow
  breathing than the mixed 60/s stream despite 6× fewer beacons. esp_wifi_80211_tx frames carry
  software scheduling jitter the common view doesn't cancel; hw TBTT beacons are clean.
  hw_only conf key filters by sig_len<64 (injected skeleton 47B vs full-IE beacon >80B).
  **PRODUCTION CONFIG: hw_only=1 bw=1.2, node at stock 100 TU** (injector can be removed).
  IDF validates beacon_interval >=100 TU (why the injector existed); faster TBTT would need
  hacks and isn't needed — slow wander is already below the per-beacon fast noise.
- Phase-0 answer (by inference): hw DOES overwrite the timestamp field of injected beacons
  (rej=0 at 60/s lock would be impossible with the sentinel constant).
- fbuck/fboost serial ports SWAP on re-enumeration (like fry/flat NAT) — `hostname` before
  driving; ESPPORT autodetect can grab the XIAO (never reset it) — always pass -p explicitly.

**Open:** node-side injector removal (keep 100 TU); optional bw sweep at hw_only; coast/AP-loss
test; RX-only noise-floor comparison.
Requires pwm_driver=mcpwm ([[project_mcpwm]]). XIAO node on
/dev/cu.usbmodem101 — never reset it (ROM download mode trap; replug w/o BOOT).
