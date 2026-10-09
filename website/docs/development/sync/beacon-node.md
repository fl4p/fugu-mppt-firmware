---
title: "bsync Beacon Node"
sidebar_position: 2
---

# bsync beacon node (`etc/bsync-beacon/`)

The beacon node is a dedicated, always-on beacon source for [beacon-sync.md](beacon-sync.md).
Its hardware TBTT beacons are the shared timebase that the converters' `bsync` service locks
to. The node is a minimal ESP-IDF softAP of ~160 LOC, with no Arduino and no fugu deps.

The node replaces the household AP. Its channel is quiet, so the converters accept the full
10/s, compared with 0.2–3/s next to a switching converter on a congested channel. AP reboots
and channel hops no longer matter. The converters stay strictly RX-only. The node is the only
transmitter and sits away from the analog front-ends.

The node is validated on a Seeed XIAO ESP32-S3. Any S3 devkit works.

## Why beacons, why 100 TU

Only the MAC's beacon engine inserts the TSF timestamp in hardware at TX time. This makes
the timestamps µs-accurate with no software in the loop, and it has two consequences:

- ESP-IDF validates `wifi_ap_config_t::beacon_interval` as ≥100 TU (102.4 ms), so 10/s is
  the ceiling unless you patch the check or poke the TBTT register directly. Neither is
  worth doing. At 10/s, stamp noise limits the loop, not the rate: slow wander sits below
  the per-beacon fast noise (see the campaign table in beacon-sync.md).
- The hardware also overwrites the timestamp *field* of raw-injected frames
  (`esp_wifi_80211_tx`). This is verified: the receiver's residual gate accepts a
  sentinel-stamped injected beacon, which a verbatim sentinel could never pass. The TX
  *timing* of injected frames is soft-scheduled, though. Common view doesn't cancel that
  queueing jitter, and the measured slow wander was ~4× worse.

The firmware still contains the 20 ms injector from that experiment. Receivers filter it out
with `bsync.conf::hw_only=1`, which tells the frames apart by length: a full-IE beacon is
>80 B and the injected skeleton is 47 B. The production config keeps `hw_only=1`, and the
injector is slated for removal.

## Firmware

The firmware in `main/main.c` runs a hidden-SSID softAP (`bsync-p0`, WPA2) that nobody joins.
It uses these settings:

- channel `#define CHANNEL` (13)
- country `DE`, where ch 12/13 are legal
- `esp_wifi_set_max_tx_power(84)`

The firmware stops the DHCP server after the AP starts. It waits for the default `AP_START`
handler first to avoid the stop/start race. The node emits beacons and nothing else.

The console runs on USB-Serial/JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` in
sdkconfig.defaults). The boot banner prints the AP MAC (base MAC + 1), which is the `bssid`
to configure on the converters.

The user LED (GPIO 21) is solid once the app is entered. It blinks at 0.5 Hz when the AP is
up and the injector is running. If the LED stays dark while the port enumerates, the chip is
sitting in ROM download mode (see [Traps](#traps)).

## Build and flash

Build and flash the node with ESP-IDF 5.5 or later:

```bash
. $IDF_PATH/export.sh   # ESP-IDF 5.5+
cd etc/bsync-beacon
idf.py set-target esp32s3     # once
idf.py -p /dev/cu.usbmodemXXX flash
```

## Receiver setup (per converter)

Run these console commands on each converter.

```
set-config bsync.conf bssid <node mac>
set-config bsync.conf channel 13
set-config bsync.conf hw_only 1
set-config bsync.conf enabled 1
wifi off          # else the STA re-associates and drags the sniffer off the node's
svc rs bsync      # channel; bare `wifi off` persists across reboots until `wifi on`
```

## Traps

- XIAO ROM download mode: if the board was (re)plugged with BOOT held, esptool's RTS reset
  re-enters download mode forever, because the strap is latched at power-on. The console is
  silent and no beacons go out, but the port still enumerates. To recover, replug without
  touching BOOT.
- If your shell setup autodetects `ESPPORT`, it can pick the node's port. Always pass `-p`
  explicitly when flashing converters, and never reset the node casually.
- A receiver associated to any AP can't tune the sniffer channel, so run `wifi off` first
  (see beacon-sync.md).
