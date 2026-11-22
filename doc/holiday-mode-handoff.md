*this document is an LLM generated placeholder*

# Holiday auto mode: status and handoff (2026-11-22)

## What it does

`charger.conf::holiday_charge` > 0 enables an automatic SoC ceiling when the pack barely cycles
(owner away). Per 24 h window (monotonic clock, starts at boot) the charger computes the mean pack
discharge current while discharging (BMS `ibat` < -0.2 A).

- **Quiet window:** ≥ 80 % ibat coverage and mean < `holiday_ibat`. A covered window with < 2 h of
  discharge is judged by Ah (`holiday_ibat` × 2 h) and counts as quiet.
- **Enter:** `holiday_days` (default 3) quiet windows in a row.
- **Exit:** a window mean above `holiday_ibat`, or within the running window after 3 h of discharge
  averaging > 1.2 × `holiday_ibat`.
- **Unevaluable windows** (coverage < 80 %, or a stuck ibat reading with ≥ 2 h discharge) pause the
  streak. Two in a row reset it, but only while not on holiday: missing data never ends an active holiday.
- **On holiday:** the partial-charge hold uses ceiling `holiday_charge` (min with `partial_charge` if
  both set) and releases `holiday_band` below it. The weekly full charge (`full_charge_interval`)
  still happens. Discharge caused by the hold itself (load-follow target down to -1 A) is not counted.
- **Override:** console `holiday auto|on|off` (also via MQTT/BLE). `status` prints
  `holiday ON|off (mode) quiet days n/N (gaps g) today: discharging h, mean x/lim A`.
- **Reboot survival:** holiday flag, streak, gaps, override, full-charge age and the BMS
  remaining-charge reference at the last full are kept in RTC_NOINIT RAM (`ChargerRtc`, MAGIC v2,
  checksum). They survive software resets/OTA, not power loss. An implausible restored reference
  (deficit outside -5 %…100 % of `bat_c`) is dropped.
- `charger.conf` is read only at boot: changing keys needs a `restart`.

Docs: `website/docs/guide/charging/termination.md` (holiday subsection), `reference/config/charger.md`,
`reference/console.md`, `internals/lfp-longevity.md` §8.

## Commits

| Commit | Content |
|---|---|
| bd29156b | Holiday auto mode, RTC retention, console override, tests, docs |
| e85b7316 | Missing data never ends an active holiday |
| 64b8498a | Review fixes: hold's own drain ignored, zero-load days quiet, implausible restored ref dropped, RTC MAGIC v2 |

QEMU suite: 295 tests, 0 failures at 64b8498a.

## Deployed configuration (fry and flat, since 2026-11-18)

- `partial_charge 0` (charge to 100 % when in use)
- `holiday_charge 0.6`, `holiday_ibat 2.3`, `holiday_band 0.1`, `holiday_days` default 3
- Resulting holiday band: about 50–60 % SoC.

Both converters currently run `fry-brk1-792-ga90529d5-dirty` (another session's build; contains holiday mode).

## Why these numbers

From one year of `bat_caravan` BMS data (280 Ah):

- Daily discharged Ah was rejected as the signal: unattended summer days ranged 6.6–13.5 %/day
  depending on cloud cover, overlapping home days.
- Mean discharge current separates better: unattended/away days 1.4–2.0 A, home days p25 2.9 A,
  median 5.1 A.
- 60 % ceiling / 10 % band (Fab's choice): same calendar-aging plateau as 30–50 % (37.5 vs 62.5 %
  storage nearly equal in the literature), below the ~70 % step, about 5 days of base-load reserve.

## Field validation

- **2026-11-22**, first complete window on hardware (11-21 18:00 to 11-22 09:30 UTC): both
  converters measured 1.85 A mean over 14.3 h of discharge. batmon (1-min Influx) gives exactly the
  same 1.85 A. Fab confirmed nobody was in the van, so the 2.3 A threshold is correct.
  Load pattern of an empty van: 0.5–0.7 A base plus 6–7 A bursts about hourly (fridge/inverter).
- Expected: holiday on after three quiet windows (about 11-24 18:00 UTC) unless reboots reset the
  running window.
- No holiday on/off transition has been observed on hardware yet.

## Open items

1. **Window progress is not RTC-saved.** Every OTA/restart discards the running 24 h window (the streak
   survives). With frequent OTAs, holiday mode can't build up quiet days. Fix: save window start age,
   coverage, discharge time and Ah in `ChargerRtc` (about +12 B, bump MAGIC). Not done; Fab not yet decided.
2. **Verify on hardware:** first holiday on transition (log `holiday on:`), pack held at about 60 %,
   exit on return, RTC survival across a real `restart` (test at night at 0 W: `holiday on`, `restart`,
   `status`, then `holiday auto`).
3. **Two chargers decide independently.** Windows start at each converter's own boot, so they can
   disagree for up to a day. The non-holiday unit then charges to full and the cap has no effect.
   Aligning windows to local midnight was declined; at least expose holiday state in telemetry.
4. **Accepted trade-off:** a BMS current stuck at exactly 0 A counts as quiet. Worst case the pack is
   held at 60 % while the owner is home, until the weekly full charge.
5. **Late return:** a return late in a window can still count that window as quiet (mean diluted by the
   quiet part), delaying exit by up to about a day.

## Unrelated issue found while checking (owned by session fugu-mppt-firmware-92)

The self-test build (b2332679, OTA'd 11-21 13:11 flat / 14:17 fry) lowered steady-state free heap by
about 6–7 KB (quiet night: 32.4 KB on build 783, about 25 KB now). Static sections differ by only
about 1 KB, so it is a persistent boot-time allocation. Session 92 is hunting it and asked that fry/flat
not be flashed or reconfigured for this until it reports. Separately, all builds dip hard by day
(783 hit about 1 KB min-ever on 11-20), visible in Influx `mppt` fields `heap`/`heap_min`.

## Operational notes

- `etc/ota_mqtt.py --name <dev> --bin <bin> -n` first, then without `-n`; take the `fry`/`flat` locks
  (`~/.claude/skills/lock/lock.sh`).
- Several `set-config` in one MQTT console session: only the first was applied. Send one per connection,
  then read back with `get-config` and run `conf-check`.
- Overnight OTA attempts on 11-18 stalled and wedged the NAT router once; daytime retries with the
  gateway TX boost succeeded first try.
