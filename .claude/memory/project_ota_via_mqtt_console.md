---
name: project_ota_via_mqtt_console
description: OTA fry/flat through the MQTT console (--name); `ota` stops conversion itself; telnet NAT ports flip and telnet OTA ran 3x slower
metadata:
  type: project
---

On 2026-11-12, OTAs sent over the NAT telnet tunnel (127.0.0.1:1223x through havan-ts) ran at ~14 KB/s and hit the 120 s OTA watchdog at 97-98% (one stalled silently). The fry/flat port mapping also flipped between consecutive connections, so a hostname check on one connection did not hold for the next.

The same image sent through the MQTT console finished in 37-40 s (42-45 KB/s), matching 11-11's telnet OTAs. Sequence: tunnel `ssh -L 11882:127.0.0.1:1882 havan-ts`, then `fugu_console.py --mqtt 127.0.0.1 --mqtt-port 11882 --mqtt-user/--mqtt-pass (etc/mqtt.env) --name <host> --stdin` with `hostname`, `ota http://192.168.1.200:9000/fugu-firmware.bin` (relay: etc/ota-via-relay.sh).

**Why:** the hostname lock is by name, not by port. Possible cause of the slowdown: the open telnet session streaming logs, or conversion load. Untested.
**How to apply:** prefer the MQTT console for OTA. Firmware 08cf227 raised OTA_TIMEOUT_SECONDS to 5 min.

**No `dc 0` needed (Fab, 11-12):** the app `ota` command latches manual 0, waits for the converter to disable, halts the ADC, and restores the previous mode if the download fails. A manual `dc 0` skips that restore.

**Silent failure (11-12 14:37):** fry's first OTA stopped at 1 %, logged no error, and fry went back to MPPT on the old image (no otaWD task left in rt-stats). A retry 10 min later worked (58 s). Always confirm with `bootinfo` after the reboot; the `App:` line the `ota` command prints is the OLD version.
