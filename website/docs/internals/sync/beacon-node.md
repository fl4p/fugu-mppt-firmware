---
title: "bsync Beacon Node"
sidebar_position: 2
---

*this document is an LLM generated placeholder*

# bsync beacon node (`etc/bsync-beacon/`)

A dedicated, always-on beacon source for [beacon-sync.md](beacon-sync.md): a minimal ESP-IDF
softAP (no Arduino, no fugu deps, ~160 LOC) whose hardware TBTT beacons are the shared
timebase the converters' `bsync` service locks to. Replaces the household AP — quiet channel,
full 10/s accept rate at the converters (vs 0.2–3/s next to a switching converter on a
congested channel), no dependency on AP reboots/channel hops. The converters stay strictly
RX-only; the node is the only transmitter and sits away from the analog front-ends.

Validated on a Seeed XIAO ESP32-S3; any S3 devkit works.

## Why beacons, why 100 TU

Only the MAC's beacon engine inserts the TSF timestamp **in hardware at TX time** — that is
what makes the timestamps µs-accurate with no software in the loop. Two consequences:

- `wifi_ap_config_t::beacon_interval` is validated ≥100 TU (102.4 ms) by ESP-IDF, so 10/s is
  the ceiling without patching the check or poking the TBTT register directly. Not worth it:
  at 10/s the loop is stamp-noise-limited, not rate-limited (slow wander sits below the
  per-beacon fast noise — see the campaign table in beacon-sync.md).
- Raw-injected frames (`esp_wifi_80211_tx`) get their timestamp *field* hw-overwritten too
  (verified: a sentinel-stamped injected beacon is accepted by the receiver's residual gate,
  which a verbatim sentinel could never pass), but their TX *timing* is soft-scheduled — that
  queueing jitter is not common-view-cancelled and measured ~4× worse slow wander. The
  firmware still contains the 20 ms injector from that experiment; receivers filter it out
  with `bsync.conf::hw_only=1` (frame length: full-IE beacon >80 B vs 47 B injected skeleton).
  Production config keeps `hw_only=1`; the injector is slated for removal.

## Firmware

`main/main.c`: hidden-SSID softAP (`bsync-p0`, WPA2, nobody joins), channel `#define CHANNEL`
(13), country `DE` (ch 12/13 legal), `esp_wifi_set_max_tx_power(84)`, DHCP server stopped
after AP start (waits for the default `AP_START` handler to avoid the stop/start race) — the
node emits beacons and nothing else. Console on USB-Serial/JTAG
(`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` in sdkconfig.defaults).

The boot banner prints the AP MAC — that is the `bssid` to configure on the converters
(base MAC + 1).

**User LED** (GPIO 21): solid = app entered; 0.5 Hz blink = AP up + injector running.
Dark with the port enumerated = the chip is sitting in ROM download mode (see traps).

## Build / flash

```bash
. ./idf-export.sh
cd etc/bsync-beacon
idf.py set-target esp32s3     # once
idf.py -p /dev/cu.usbmodemXXX flash
```

## Receiver setup (per converter)

```
set-config bsync.conf bssid <node mac>
set-config bsync.conf channel 13
set-config bsync.conf hw_only 1
set-config bsync.conf enabled 1
wifi off          # NOT persisted — repeat after every reboot, else the STA
svc rs bsync      # re-associates and drags the sniffer off the node's channel
```

## Traps

- **XIAO ROM download mode**: if the board was (re)plugged with BOOT held, esptool's RTS
  reset re-enters download mode forever (strap latched at power-on) — silent console, no
  beacons, port still enumerates. Recovery: replug **without** touching BOOT.
- `ESPPORT` autodetect (idf-export.sh) can grab the node's port; always pass `-p` explicitly
  when flashing converters, and never reset the node casually.
- A receiver associated to any AP cannot tune the sniffer channel — `wifi off` first
  (see beacon-sync.md).
