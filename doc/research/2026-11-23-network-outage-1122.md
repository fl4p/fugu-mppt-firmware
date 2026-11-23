*this document is an LLM generated placeholder*

# Van network outage, 2026-11-22 10:31–10:42 UTC

Read-only investigation on 2026-11-23. No device, router, or havan state was changed.

## Root cause

**havan was rebooted on purpose.** At 10:31:36 UTC an agent session ran `sudo systemctl reboot` on havan. The session was
`~/.claude/projects/-Users-fab-dev-ha-home-assistant-addons-batmon-ha/2c7139c9-…`, working on havan memory and zram, and it
ran the command over Tailscale from this Mac (100.64.0.2, "fmb"). Fab approved it at 10:29:33 ("3: yes reboot"). The reboot
was needed to clear a zram kernel oops that the same session's subagent caused at about 10:01, when it set up zram live.

Shutdown then hung for about 6 minutes. `systemd-zram-setup@zram0` could not kill the zram-generator process left stuck
in D state by the oops, and SIGKILL timed out 4 times at 90 s each (10:33:08 to 10:37:39). The kernel reset at 10:37:39,
havan booted at 10:40:51, and it was back on Tailscale at about 10:42:22.

Every symptom follows from havan being down:

- **Remote access:** havan is the tailnet entry and the `havan-ts` jump host, so from outside it looked unreachable.
- **MQTT:** the broker runs on havan (192.168.1.200:1882), so converter MQTT and the console log were down.
  `pv/fugu_console.log` is written by `fugu-mqtt-console-log.service` on havan.
- **BMS data:** batmon runs on havan, so no BMS frames reached the converters. That caused flat's stale-cell fallback and
  the ramp of its Vout cap.
- **Telemetry:** the InfluxDB UDP relay runs on havan, so the `mppt`, `batmon` and `nimble_proxy` series all stop.

**Confidence: high.** The journal shows the reboot command, who ran it and from where. The agent transcript shows Fab's
approval and the exact time. The gap edges in every data source line up with the shutdown and boot times.

**The converter OTAs did not cause it.** No OTA, txpower boost or router write reached fry or flat before 10:31. The OTA
session (68f57afd) first ran `ota_mqtt.py -n` (dry run) at 10:33, after the outage had started, and found itself blocked. The
real OTAs ran at 10:45:01 (flat) and 10:47:47 (fry), after havan was back.

**A second, independent event happened inside the window.** At 10:38:00 another session (`nimble-ble-proxy`, 62b85f60)
OTA'd the .231 NAT router to 0.2.7 with `POST /update`, which rebooted it. That cut the converters' Wi-Fi AP for about
15–20 s. It did not cause the outage (the router answered normally from 10:35 to 10:37), but it widened it. It also cleared
the router's `/wifihist` ring, so the router-side record of 10:31–10:38 is lost.

## Timeline (UTC)

| Time | Event | Source |
|---|---|---|
| 09:45:21, 09:45:37 | Pi "Undervoltage detected" (2 s each), background only | havan `journalctl -k` |
| ~10:01 | zram set up live under memory pressure: zstd init fails, zram module oopses, a zram-generator process hangs in D state, kernel tainted | batmon-ha session report at 10:24:46; kb `tooling/zram-live-setup-oops-rpi-6-12.md` |
| 10:09:45 / 10:12:23 | Same session restarts batmon and the cmt collectors (memory caps) | havan journal, sudo lines |
| 10:24:46 | Session asks Fab whether to reboot havan to clear the oops | transcript 2c7139c9, line 4584 |
| 10:29:33 | Fab: "1 yes and push. 2: yes on-demand. 3: yes reboot" | transcript 2c7139c9, line 4593 |
| 10:31:15–10:31:22 | havan-browser/autofix-browser switched to socket activation | havan journal |
| 10:31:21 | Last nbp-poll sample of .231 (uptime 345 438 s, `wd_probe_ok=1`, STA RSSI −80) | InfluxDB `nimble_proxy` |
| 10:31:35.7 / 10:31:36.08 | Last `mppt` points: flat 572 W, fry 428 W | InfluxDB `mppt` |
| 10:31:36.15 | Last converter line in the console log (fry status line) | `pv/fugu_console.log.2` |
| **10:31:36.35** | **`sudo systemctl reboot`**, user fab, ssh from 100.64.0.2 | havan journal (boot −1) |
| 10:31:36.6 | tailscaled stops, havan goes offline on the tailnet | havan journal |
| 10:31:37 | Last batmon point | InfluxDB `batmon` |
| 10:31:57 | docker, Home Assistant, mosquitto, NetworkManager and wpa_supplicant stop; eth0 goes down | havan journal |
| 10:33:08–10:37:39 | Shutdown waits on `systemd-zram-setup@zram0` (hung pid 247395): SIGTERM, then SIGKILL, timing out each time | havan journal |
| 10:34:02–10:34:33 | OTA session 68f57afd: Tailscale shows havan offline; ping to .200/.231/.1 and `nc 192.168.1.1 80` all fail from its shell; fry/flat OTA "BLOCKED" | transcript 68f57afd |
| 10:35:06–10:36:18 | Direct BLE from the Mac to flat and fry works (stale BMS) | transcript daa59809 |
| 10:35:31–10:36:28 | Session 62b85f60 curls 192.168.1.231 from the same Mac and gets answers. Router uptime 345 745 s; `/liveness` failures 0, armed; `/txpower` wifi 11 dBm, `revert_s` 0, so no boost active | transcript 62b85f60 |
| 10:37:39 | havan: `systemd-reboot`, hardware reset | havan journal |
| **10:38:00** | **.231 OTA'd to 0.2.7 (`POST /update`) and reboots.** Its uptime is 242 s at 10:42:02 | transcript 62b85f60; InfluxDB `nimble_proxy` |
| 10:40:51 | havan boots (wtmp "shutdown 10:31:57 → boot 10:40:54") | `last -x -F` |
| 10:41:18 | Console logger reconnects to the broker | `pv/fugu_console.log.2` |
| 10:41:19 | First converter lines again. flat uptime 60 446 s (no reboot), `netguard: mqtt down 0 s, re-inits 7` | `pv/fugu_console.log.2` |
| 10:41 | fry `mppt` points resume; flat's resume at 10:44 | InfluxDB `mppt` |
| 10:42:02 | nbp-poll resumes | InfluxDB `nimble_proxy` |
| 10:42:22 | havan reachable on Tailscale again (direct 192.168.1.200) | transcript 2c7139c9 |
| 10:42–10:48 | Battery goes from +38 A charging to about −4…+14 A; voltage 26.73 → 26.2 V (a load came on after HA restarted; not investigated) | InfluxDB `A`/`V` `ant24_*` |
| 10:45:01 / 10:47:47 | flat / fry OTA to 804-gba794456 | `elf-archive/ota-owners.jsonl` |

The converters' clocks agree with the havan journal to within a second.

## Answers to the questions

### 1. What went down first, and what stayed up

havan went down first, by command. Its services stopped at 10:31:36–10:31:57.

What stayed up:

- **.231 router:** up until its own OTA reboot at 10:38:00. It answered HTTP from the Mac between 10:35 and 10:37. Its liveness
  watchdog had 0 failures, because the gateway `:80` probe kept passing.
- **LAN AP and RUTX gateway (192.168.1.1):** no evidence they went down. The router's gateway probe kept passing. The OTA
  session's failed pings came from macOS Local Network privacy on that agent's host app, not from the LAN: a different
  session on the same Mac reached .231 at the same moment.
- **fry and flat:** kept converting (about 440 W and 590 W) and never rebooted. BLE worked throughout.

### 2. Did havan reboot or lose its link?

havan did a clean, requested reboot, and eth0 went down only as part of it. There are no NetworkManager, wpa or link events
before 10:31:36. The uplink was not the cause.

### 3. Converter logs

The log has a gap from 10:31:36 to 10:41:19, because the logger lives on havan. Afterwards:

- no reboot lines;
- flat's uptime is continuous;
- flat's `netguard` reports `mqtt down 0 s` right after reconnecting.

The device log is not stored on the device, so Wi-Fi drops inside the gap cannot be seen. That includes the expected ~20 s
AP loss when the router rebooted at 10:38.

### 4. InfluxDB

| Series | No points |
|---|---|
| `mppt` fry | 10:31:36 to 10:41 |
| `mppt` flat | 10:31:36 to 10:44 |
| `batmon`, `V`/`A` (ant24) | 10:31:37 to 10:42 |
| `nimble_proxy` | 10:31:21 to 10:42:02 |

Per-minute counts queried on 11-23 are still 0 for 10:32–10:40, so **the data is lost, not back-filled**. The converters'
UDP telemetry has no buffer for a relay that is gone.

nbp-poll back-fills `/wifihist` after an outage, but the router's ring restarted at 10:38, so only samples from 10:42 on came
back. nbp-poll did not record the 10:38 router reboot as a `nimble_proxy_reboot` either, because it was itself down.

### 5. Earlier similar outages (since 11-02)

These are daytime gaps in `mppt` where both converters were silent, matched against havan's wtmp and journal. The journal
only goes back to 11-07 12:37, so earlier reboots have no recorded cause.

| Window (UTC) | havan | .231 router | Cause |
|---|---|---|---|
| 11-05 07:18–07:37 | unclean boot 07:17 (wtmp has no shutdown record) | ? | unknown (journal rotated) |
| 11-05 11:29–12:03 | clean reboot at 12:00 | ? | unknown initiator; telemetry was already gone 30 min before the reboot |
| 11-05 13:05–14:14 | clean reboot at 13:35 | ? | unknown; same pattern |
| 11-07 09:33–10:04 | clean reboot at 10:03 | ? | unknown; same pattern |
| 11-07 10:57–15:08 | clean reboot at 11:28 | ? | unknown; 4 h gap |
| 11-12 15:45–16:25 | up the whole time | lost: batmon's ESPHome API to .231 dropped at 15:43:48 | router-side (NAT router wedge class); not a havan outage |
| 11-16 14:43–14:46 | **unclean**: journal ends 14:42:31, NTP sync 14:45:44 | stayed up (uptime continuous) | unknown; the 3-minute return looks like a power cycle |
| 11-20 09:49–10:02 | **unclean**: journal ends 09:48:41, back 10:01:46 | stayed up (uptime continuous) | power: "van router and appliances went off at about 09:49 UTC after an appliance overvoltage protection tripped" (fugu-fw session, channel `fugu`, 09:56). fry also had an INT_WDT reboot then (BLE conn-update assert, see memory) |
| **11-22 10:31–10:42** | clean, requested reboot plus 6 min zram hang | OTA reboot at 10:38 | **this incident: deliberate reboot** |

Also seen:

- **11-02 to 11-05:** clean havan reboots at 04:04 every day (scheduled, at night, outside the converters' daytime window).
- **Evening gaps after about 17:40:** converters at night, not outages.

### 6. Power correlation

- **Before 10:31:** nothing. The battery read 26.73 V, charging at about +38 A, with a smooth rising trend. Both converters
  were steady.
- **Undervoltage:** havan's Pi logged it at 09:45 and 10:59, but not between 10:25 and 10:41. `vcgencmd get_throttled` =
  `0x50000`: undervoltage and throttling have occurred since boot.
- **The Pi's supply is marginal:** there are dozens of undervoltage events since 11-15. It is not implicated here.
- **Inverter or heavy load:** no switching before the outage. After havan and HA came back, a load of roughly 1 kW appeared
  between 10:42 and 10:48 (battery current went from +38 A to about −4 A). That needs its own look at the HA inverter
  automations, which re-initialised at 10:41:46.

## What the evidence rules out

- **The fry/flat OTA:** the first OTA command came after the outage started, and the first real OTA was at 10:45.
- **A txpower boost:** `/txpower` showed `revert_s 0` and 11 dBm at 10:36, and no OTA through `ota_mqtt.py` had run.
- **A .231 crash or liveness reboot:** it was up with 0 liveness failures until the deliberate OTA at 10:38.
- **A havan crash, OOM, or power or brown-out:**
  - the shutdown was a logged, orderly `systemctl reboot`;
  - `power_reset` is 0;
  - there was no undervoltage in that window.
- **An uplink, LTE or LAN AP loss:** no link events. The .231 gateway probe passed, and .231 answered the Mac.
- **A converter reboot or Wi-Fi fault:** uptime is continuous and MQTT reconnected immediately.

## Next steps

1. **Treat a havan reboot as a van-wide outage.** It takes down:
   - the MQTT broker;
   - the BMS feed, so both chargers fall back to stale cells;
   - telemetry;
   - the console log;
   - remote access.

   Before rebooting havan, an agent should take a `havan` lock with `lock.sh` and post on the `fugu` channel. Other
   sessions then won't mistake it for a fault, start OTAs, or OTA the router in the middle of it, as happened at 10:38.
2. **Bound shutdown time.** A `TimeoutStopSec=` drop-in for `systemd-zram-setup@.service` (or a lower `DefaultTimeoutStopSec`)
   keeps a wedged unit from adding 6 minutes. Rebooting with `systemctl reboot --force` after a known kernel oops would
   also avoid it.
3. **nbp-poll:** persist the last seen uptime and boot count to disk, so a router reboot that happens while havan is down
   (like 10:38) still produces a `nimble_proxy_reboot` point and a `/log` snapshot.
4. **Find the cause of havan's unclean resets** (11-05 07:17, 11-16 14:42, 11-20 09:48):
   - log the Pi 5 `power_reset` value and `vcgencmd get_throttled` at every boot into the journal or InfluxDB;
   - consider `RuntimeWatchdogSec=` so a hang becomes a visible watchdog reset;
   - keep the journal longer (currently only from 11-07);
   - log the 12 V/5 V supply (undervoltage events are frequent).
5. **Telemetry loss:** UDP telemetry to the havan relay has no buffer. If loss during havan outages matters, either send
   `mppt` directly to tm.fabi.me as a fallback, or have the converter keep a short ring buffer of tele frames to replay on
   reconnect.
6. **BMS single point of failure:** batmon and the broker both live on havan. The stale-cell fallback worked as designed
   (cap ramp), but the converters lose real cell data whenever havan is down.
7. **Agent diagnostics:** "No route to 192.168.x" from an agent shell is often macOS Local Network privacy, not the LAN.
   On 11-22 another session reached .231 from the same Mac at the same time. Check with a second path (Tailscale status,
   BLE, another session) before concluding that the LAN is down.

## Sources

- **havan:** `journalctl -b -1/-2/-3/0`, `last -x -F`, `tailscale status`/`debug prefs`, `/home/fab/bin/nbp_poll.py`,
  `~/nbp-reboots/`, `/proc/device-tree/chosen/power/power_reset`, `vcgencmd get_throttled`.
- **Console logs:** `pv/fugu_console.log.2` (10:28–10:50).
- **InfluxDB `ha_van`:**
  - `mppt` (per-minute counts since 11-02);
  - `nimble_proxy`, `nimble_proxy_wifi`, `nimble_proxy_reboot`;
  - `V`/`A` for `ant24_voltage`/`ant24_current`.
- **Router:** `GET /liveness` and `/crash` on 2026-11-23, plus the values recorded on 11-22 at 10:35–10:36 by session 62b85f60.
- **Agent transcripts** in `~/.claude/projects/`:
  - batmon-ha `2c7139c9` (the reboot and the approval);
  - nimble-ble-proxy `62b85f60` (the router OTA at 10:38);
  - fugu-mppt-firmware `68f57afd` (the converter OTAs) and `fb783428` (coordinator);
  - `-Users-fab-dev-ha` `369e2f94`.
- **Repo:** `elf-archive/ota-owners.jsonl`.
