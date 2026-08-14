---
name: fugu
description: Build, flash, provision and bring up Fugu MPPT firmware on the fboost/fbuck bench converters — toolchain location, the app-flash-not-flash rule, config profiles, getting BLE on the air, and the console commands that are safe to send while diagnosing. Use whenever a task involves fugu-mppt-firmware, a fugu board (fboost, fbuck, fry, flat, flu, fmetal), the power-loop rig, `provision.py`, or a board that won't appear on BLE — from any project.
---

# Fugu firmware & bench bring-up

Repo: `/Users/fab/dev/pv/fugu-mppt-firmware` (`git@github.com:fl4p/fugu-mppt-firmware.git`).

**The repo's `doc/` is authoritative and maintained — do not duplicate it here.** `Console.md`,
`Services.md`, `Configuration.md`, `Power Loop.md`, `Coil Inductance Measurement.md`,
`Automated Bench Tests.md`, `Agentic Programming.md`, plus root `CLAUDE.md`. This skill is only the
operational sequence, which lives in no single doc and is what actually costs time.

## Maintaining this skill — read before you edit it

**You are expected to update this file when you learn something operational that it got wrong or
does not cover.** A trap that cost you an hour and is not written here is a trap the next session
pays for again.

This file lives at `fugu-mppt-firmware/.claude/skills/fugu/SKILL.md` and is **version-controlled in
the firmware repo**. `~/.claude/skills/fugu` is a symlink to it, so editing it either way modifies
the repo — commit the change (that path alone; other sessions have unrelated work in flight) so it
is reviewable and revertible.

This file is loaded into every fugu session's context, so a wrong line is worse than a missing one:
it turns one session's mistake into durable, confident guidance for everybody. Six rules, not
optional.

1. **Never write a claim you did not verify.** If you inferred it, say so in the text
   ("suspected", "unconfirmed"). If you observed it, say what you observed. A guess stated flatly
   here will be believed and acted on months from now.
2. **Grep the repo's `doc/` and the GitHub issues FIRST.** Most "discoveries" here are already
   documented — reverse-current in `measure-coil`, the `enabledDefault` mechanism, `FLASH_IN_PROJECT`
   were each "found" by an agent that had not looked. If a repo doc covers it, link the doc; do not
   restate it.
3. **Operational content only.** Commands, sequences, ordering, traps, where things live. Firmware
   behaviour, electrical findings and suspected bugs belong in the repo's `doc/*.md` or a GitHub
   issue — not here. If you think firmware has a defect, file an issue and link it from here in one
   line.
4. **Correct in place; do not append.** Fix the wrong sentence. Never add a second line that
   contradicts an existing one, and never add a changelog, "update log" or dated-findings section.
   This is a reference, not a journal — if it reads as a log it has failed.
5. **Re-read the file immediately before editing** — sessions run concurrently, your copy is stale.
6. **Stay under ~200 lines.** Past that, consolidate or move content into the repo docs. An
   unbounded skill stops being read. Deleting something proven wrong is as valuable as adding.

## Before touching hardware

Take a lock (`lock` skill) on every shared resource: the board's serial port (`usbmodem*` — check
the exact name, `usbmodem101` and `usbmodem1101` are different boards), plus `fugu-rig` / `scope`
if a measurement is involved. Several agent sessions work this repo concurrently; assume a dirty
tree and another session mid-edit. Never `pkill` a daemon you did not start.

## Toolchain

ESP-IDF is **not on PATH** and `which idf.py` finds nothing. Installs are recorded in
`~/.espressif/esp_idf.json` / `idf-env.json`. Currently:

```bash
source /Users/fab/dev/esp/idf5.5/export.sh      # IDF 5.5.1 — matches what the boards run
```

## Build

```bash
idf.py -B build-<tag> build
```

* **Use a separate `-B` build dir.** The shared `build/` may be in use by another session.
* Build options are Kconfig (`CONFIG_FUGU_WITH_*`), **not** env vars — the build errors out if a
  legacy `WITH_*` env var is set. `WITH_NETW` and `WITH_BLE` default **on**; `MCPWM`,
  `MEASURE_COIL`, `VCONV`, `SPROFILER` default **off**, but the tree's checked-in `sdkconfig` may
  differ — read it, don't assume.
* For a variant without disturbing the shared `sdkconfig`: copy it to scratch, edit, then
  `idf.py -B build-<tag> -D SDKCONFIG=/path/to/sdkconfig.<tag> build`.

**Verify features against the built binary, not the config** — the config you think you passed is a
proxy, the binary is the artifact:

```bash
strings -a build-<tag>/fugu-firmware.bin | grep -F "NUS console"        # WITH_BLE
strings -a build-<tag>/fugu-firmware.bin | grep -F "measure-coil L0:"   # WITH_MEASURE_COIL
```

## Flash — use `app-flash`, never `flash`

**Identify the target first** unless you just used this exact port: `esptool.py -p PORT chip_id`
plus the boot banner's `Project name:` line. Fugu units, the NAT router and other ESP32s on this
Mac look identical on USB — this check once stopped NAT-router firmware from wiping a live
converter (chip said S3, banner said `fugu-firmware`; the expected target was a classic ESP32).

```bash
idf.py -B build-<tag> -p /dev/cu.usbmodemXXX app-flash
```

`idf.py flash` also writes the littlefs config partition, and for esp32s3 the CMake default source
is `config/lab/dry_mock` with `FLASH_IN_PROJECT` — a profile with `adc=fake`,
`pwm_deadtime_ns=0`, and PWM pins that do not match a real board. On a live power stage that is a
hazard, and it lands before you can provision.
See **[issue #61](https://github.com/fl4p/fugu-mppt-firmware/issues/61)**. `app-flash` writes only
the app partition and leaves littlefs and NVS alone.

Check the partition table matches first — `bootinfo` reports e.g. `ota_0 @0x010000 (1828KB)`
against the build's `Smallest app partition is 0x1c9000` (= 1828 KB).

### When esptool can't reset the board

`Failed to connect to ESP32-S3: No serial data received` — and a failed attempt can leave the board
responding to **neither** the app nor the bootloader (silent console, `chip_id` also fails). It is
not bricked; nothing was written. Recover by:

1. retrying — a second `esptool.py --chip esp32s3 -p PORT chip_id` often just connects; or
2. manual download mode: hold **BOOT**, tap **EN/RESET**, release BOOT.

Some boards set `pwm_sync_pin=44` (= U0RXD) for the wired-sync follower, with `board.conf` noting
"serial-RX console dead on this board" — suspected but unconfirmed as the reason auto-reset is
flaky. Console still works over USB-CDC regardless.

### When the port vanishes

On macOS the S3's USB-JTAG serial node can disappear and re-enumerate with a **different suffix**
within seconds of plugging in (`usbmodem11301` → `usbmodem1101`), so a glob-based `ESPPORT`
autodetect errors "no matches found" or goes stale mid-session. Wait 2–3 s and re-run
`ls /dev/cu.usbmodem*`; if `system_profiler SPUSBDataType` shows the Espressif device but no node
exists yet, `ioreg -l -r -c AppleUSBACMData` → `IOCalloutDevice` is the `/dev/cu.*` path.

## Provision a config profile

```bash
ESPPORT=/dev/cu.usbmodemXXX ./provision.py config/lab/<profile>
```

Builds a littlefs image from the profile dir and writes it via `parttool.py` (so it needs bootloader
entry too — same recovery as above). Profiles live in `config/`: `lab/fbuck_lab_bench`,
`lab/fbuck_lab_bench_open_output`, `lab/fboost`, `fmetal`, `lab/dry_mock`, …

Symptoms of **no config partition**: `ls /` fails, `ntc=-273℃`, `0sps` in the status line. The conf
path is `/littlefs/conf/`, not `/conf/`.

`open_output` vs plain `fbuck_lab_bench` is a real distinction: with the output open Vout floats
toward Vin, so the battery profile's `vout_max=29` trips OV above ~30 V Vin. Only use `open_output`
for genuinely open-output sweeps.

## Get BLE on the air

Three independent things must all be true; each has bitten separately.

1. **`WITH_BLE` in the image.** `svc` won't even list `ble` otherwise, `help` shows no `wifi`/`ble`
   commands, and `tasks` shows no NimBLE host — a `WITH_NETW=0/WITH_BLE=0` build has ~8 tasks
   (`loopRt`, `loopTask`, IDLEs, `Tmr Svc`, `ipc0/1`, `esp_timer`). No runtime setting can fix
   this; it needs a rebuild.
2. **Hostname.** The advertised name is `fugu-` + hostname. Set it: `hostname <name>` → advertises
   `fugu-<name>`. Unset, it defaults to a MAC-derived `fugu-esp32s3-<digits>`.
   **The read-back is cached** — `hostname` still prints the *old* value until reboot, which looks
   like the write failed. It didn't; `restart` and re-check.
3. **The service must be enabled.** It can ship `ENABLED=no`, in which case it never starts at boot
   and the board is silent despite BLE being compiled in:

   ```
   svc                  # NAME/STATE/ENABLED table
   svc on ble           # enables + starts, and persists
   ```

Then **confirm over the air**, not from the log — scan with `bleak` and look for the name:

```python
from bleak import BleakScanner
devs = await BleakScanner.discover(timeout=15.0)
```

The bench tooling's `--buck-name` defaults to `fugu-fbuck` and `--boost-name` to `fugu-fboost`, so
match those unless you also update the invocation.

A **bonded** Mac failing at subscribe with ATT code 3 ("Writing is not permitted") after a firmware
change that altered the GATT layout is a stale macOS GATT cache, not a firmware bug:
`blueutil --unpair <mac>`, then reconnect. Renaming the device does not help — the cache is keyed
by the bond, not the name. (Code 5/15 is a different failure: insufficient auth/encryption.)

## Console commands: read-only vs state-changing

**While diagnosing, send only read-only commands.** Safe: `bootinfo`, `tasks`, `uptime`, `status`,
`svc`, `hostname` (no arg), `ls`, `cat`, `get-config`, `rt-stats`, `mem`, `heap`, `ip`.

Ask first before: `bf`/`panel`, `dc`, `sweep`, `mppt`, `sync`, `psu`, `vset`/`iset`/`ovset`,
`measure-coil`, `restart`, `ota`/`ota-ble`, `short-ls`, `adc-restart`. A bare `bf` or `dc` takes an
argument-less default that changes converter state — this has altered someone's live test before.
Flashing firmware is pre-authorised on bench units; toggling converter state is not the same thing.

## Coil inductance

`doc/Coil Inductance Measurement.md` already carries **measured** values from full duty sweeps
(~550 DCM points each): **fry ≈ 79.8 µH** (IQR 16 %), **flat ≈ 50.9 µH** (IQR 11 %). The doc states
the 1.57× difference is two different inductors, not measurement error, cross-checked against a
battery shunt. Check there before measuring anything.

Note `coil.conf::L0=40e-6` in both `fbuck_lab_bench*` profiles matches **neither** board and is
stale.

`measure-coil l0|ls [steps|hs] [dwell_ms] [apply]` is an **active duty sweep**, not a passive probe:
it needs `Vin > Vout + 1`, keeps only DCM points, stops on CCM entry / `Iout > min(2 A, Iout_max)` /
protection trip. `apply` writes `coil.conf::L0` — it reports an IQR but does not gate on it, so a
wide-spread result can be persisted. Read the IQR before trusting an applied value.

## Power-loop rig

fboost supplies fbuck's input; the rig recirculates. `doc/Power Loop.md` and
`dcdc-tools/verifications/vin-sweep/BRIEF-power-loop-rig.md` are the references. Two standing traps:

* **Low duty is the dangerous end.** Below fboost's count, fbuck boosts back into it and the
  reverse-current protection trips. The trip is protection working, not a fault.
* **`--sync forced` or it silently runs DCM**, which `loss.py` assumes away. It's read back from
  firmware and the point is refused on mismatch.

For bench measurement conventions (ground springs not clips, `--probes`/`--dut`, restarting the
bench daemon after editing `bench/*.py`), see that brief's trap list — it applies to anything run
on this rig.
