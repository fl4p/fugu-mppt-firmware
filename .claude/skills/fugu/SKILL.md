---
name: fugu
description: Build, flash, provision and bring up Fugu MPPT firmware on the fboost/fbuck bench converters — toolchain location, the app-flash-not-flash rule, config profiles, getting BLE on the air, and the console commands that are safe to send while diagnosing. Use whenever a task involves fugu-mppt-firmware, a fugu board (fboost, fbuck, fry, flat, flu, fmetal), the power-loop rig, `provision.py`, or a board that won't appear on BLE — from any project.
---

# Fugu firmware & bench bring-up

Repo: `/Users/fab/dev/pv/fugu-mppt-firmware` (`git@github.com:fl4p/fugu-mppt-firmware.git`).

**The repo's `doc/` is authoritative and maintained — do not duplicate it here.** `Console.md`,
`Services.md`, `Configuration.md`, `Power Loop.md`, `Coil Inductance Measurement.md`,
`Automated Bench Tests.md`, `Agentic Programming.md`, **`Bench Operations.md`** (the operational
detail behind this file), plus root `CLAUDE.md`. This skill is the headline traps and sequence;
when a line here says "details: Bench Operations", the evidence and full recipes are there.

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
if a measurement is involved. One reader per port — contention kills the other process silently
(`lsof /dev/cu.usbmodemXXX` first). Several agent sessions work this repo concurrently; assume a
dirty tree and another session mid-edit: `git diff --cached --name-only` before every commit and
`git log origin/main..HEAD` before every push — both have swallowed/published another session's
work (details: Bench Operations). Never `pkill` a daemon you did not start, and never *start* one
either (`brew services start …` was noticed and objected to).

## Toolchain

ESP-IDF is **not on PATH** and `which idf.py` finds nothing. Enter the env with the repo wrapper,
**from the repo root only** (it sources IDF 5.5.1 by relative path):

```bash
. ./idf-export.sh
```

It `deactivate`s any venv — invoke repo Python tools as `.venv/bin/python3 etc/…` afterwards — and
it exports `ESPPORT` from the **first** `/dev/cu.usbmodem*` match, a coin flip with two boards
attached (this has flashed the wrong board): always pass `-p` explicitly. Bare `esptool` can be a
broken PlatformIO shim; `python -m esptool` after sourcing always works. The vendored IDF carries a
local NimBLE patch (`etc/patches/`) that **an IDF update silently reverts** — reapply after bumps.

## Build

```bash
idf.py -B build-<tag> build
```

* **Use a separate `-B` build dir.** The shared `build/` may be in use by another session.
* Build options are Kconfig (`CONFIG_FUGU_WITH_*`), **not** env vars — the build errors out if a
  legacy `WITH_*` env var is set. `WITH_NETW` and `WITH_BLE` default **on**; `MCPWM`,
  `MEASURE_COIL`, `VCONV`, `SPROFILER` default **off**. The local `sdkconfig` is **gitignored** and
  regenerable; flags appended to it **vanish on the next regeneration** — this has shipped builds
  missing BLE_TELE and silently pruned bsync (a lost `MCPWM=y` and Kconfig depends-on). Durable
  flags go in `sdkconfig.defaults` or a fragment.
* For a variant without disturbing the shared `sdkconfig`: copy it to scratch, edit, then
  `idf.py -B build-<tag> -D SDKCONFIG=/path/to/sdkconfig.<tag> build`. Multiple defaults fragments
  must be **quoted** (`-DSDKCONFIG_DEFAULTS="a;b"` — unquoted, the shell eats the second one
  silently); `idf.py set-config` does not exist; `SDKCONFIG` as an env var has no effect.

**Verify features against the built binary, not the config** — the config you think you passed is a
proxy, the binary is the artifact:

```bash
strings -a build-<tag>/fugu-firmware.bin | grep -F "NUS console"        # WITH_BLE
strings -a build-<tag>/fugu-firmware.bin | grep -F "measure-coil L0:"   # WITH_MEASURE_COIL
```

On a running board the same check is one console verb: a feature's command answering
`Command not found` means it is not in the image — no runtime setting fixes that.

## Flash — use `app-flash`, never `flash`

**Identify the target first, and re-verify after every replug** — the port↔board mapping *swaps*
(four observed incidents incl. flashing a foreign S3 — Bench Operations). Cheapest check, no board
touch: `system_profiler SPUSBDataType` prints each S3's **efuse MAC as its USB Serial Number**;
then console `hostname`/banner; `python -m esptool -p PORT chip_id` (resets the board) last.
esptool errors reading like link noise (`Invalid head of packet`, `Corrupt data`, ROM boot-loop)
can mean *wrong device* — check identity before blaming cables.

```bash
./flash.sh <name> -B build-<tag> -p /dev/cu.usbmodemXXX app-flash
```

`flash.sh` just sets `FUGU_DEVICE=<name>` so the archived ELF is keyed by board (else the unstable
port basename — a later coredump decode by `--device` misses). **A bare `./flash.sh <name>` with no
idf.py args runs a full `idf.py build flash`** — always append `app-flash`.

`idf.py flash` also writes the littlefs config partition, and for esp32s3 the CMake default source
is `config/lab/dry_mock` with `FLASH_IN_PROJECT` — a profile with `adc=fake`,
`pwm_deadtime_ns=0`, and PWM pins that do not match a real board. On a live power stage that is a
hazard, and it lands before you can provision.
See **[issue #61](https://github.com/fl4p/fugu-mppt-firmware/issues/61)**. `app-flash` writes only
the app partition and leaves littlefs and NVS alone.

Check the partition table matches **before every flash to a port not just verified** — `bootinfo`
reports e.g. `ota_0 @0x010000 (1828KB)` against the build's `Smallest app partition is 0x1c9000`
(= 1828 KB). esptool does *not* check: it wrote the 1.7 MB fugu app over a foreign S3's 1448 KB
app partition and 216 KB of its data, reporting `Hash of data verified. Done`.

### When esptool can't reset the board

`Failed to connect to ESP32-S3: No serial data received` — and a failed attempt can leave the board
responding to **neither** the app nor the bootloader (silent console, `chip_id` also fails). It is
not bricked; nothing was written. Recover by:

1. retrying — a second `python -m esptool --chip esp32s3 -p PORT chip_id` often just connects; or
2. manual download mode: hold **BOOT**, tap **EN/RESET**, release BOOT.

The inverse also happens: a board stuck **in** download mode (replugged with BOOT held, or a
hand-rolled DTR/RTS reset holding IO0 — never do that) — esptool works perfectly while the app
never runs, reading as a firmware hang. Recover: plain replug *without* touching BOOT; probe with
`--before no_reset chip_id`, kick to the app with `--before no_reset run`. And a **dead USB serial
≠ dead board**: one answered instantly over BLE — switch transport (Bench Operations).

Some boards set `pwm_sync_pin=44` (= U0RXD) for the wired-sync follower, with `board.conf` noting
"serial-RX console dead on this board" — suspected but unconfirmed as the reason auto-reset is
flaky. Console still works over USB-CDC regardless.

### When the port vanishes

On macOS the S3's USB-JTAG serial node can disappear and re-enumerate with a **different suffix**
within seconds of plugging in (`usbmodem11301` → `usbmodem1101`), so a glob-based `ESPPORT`
autodetect errors "no matches found" or goes stale mid-session. Wait 2–3 s and re-run
`ls /dev/cu.usbmodem*`; if `system_profiler SPUSBDataType` shows the Espressif device but no node
exists yet, `ioreg -l -r -c AppleUSBACMData` → `IOCalloutDevice` is the `/dev/cu.*` path.
(zsh aborts the *whole* command on any unmatched glob — quote patterns, split per-glob;
`2>/dev/null` doesn't help.)

## OTA

`etc/ota.py` (Wi-Fi; flags in `CLAUDE.md`) refuses non-interactively: dirty images, `WITH_NETW=n`,
`WITH_VCONV=y` — commit or `git stash push -- <paths>` first. Over BLE:
`.venv/bin/python3 etc/ota_ble.py -n <name> -y`, **detached** (2–3+ min) and **one at a time** (one
Mac radio). The version string is git-describe, so an uncommitted rebuild keeps the old string and
the tool *skips* with a success-looking `☑️ skip:` — pass `-f` when the tree changed without a
commit; a real push prints hundreds of progress lines. A failed/killed push leaves the device
armed: `ota-ble abort` (the verb — docs say `otab`), retry. Post-OTA/`restart` the board is silent
~10–15 s; scan misses and READY timeouts there are retryable. `doc/OTA over BLE.md` + Bench Ops.

## Provision a config profile

```bash
ESPPORT=/dev/cu.usbmodemXXX ./provision.py config/lab/<profile>
```

Builds a littlefs image from the profile dir and writes it via `parttool.py` (so it needs bootloader
entry too — same recovery as above). Profiles live in `config/`: `lab/fbuck_lab_bench`,
`lab/fbuck_lab_bench_open_output`, `lab/fboost`, `fmetal`, `lab/dry_mock`, …

Symptoms of **no config partition**: `ls /` fails, `ntc=-273℃`. (`0sps` is *not* a reliable
symptom: a status-line bug — fixed in `6b83991` — produced it from any scripted console session,
and NaN/`0sps` is normal for ~15 s after boot while the sampler calibrates; re-read before calling
an ADC dead.) The conf path is `/littlefs/conf/`, not `/conf/`.

The deployed config **drifts from `config/` in the repo** — `get-config` is ground truth for what
a board runs. Read back every `set-config` in the same invocation (a stray `~` framing corruption
has silently dropped writes); `set-config k ""` stores the quotes literally; wifi keys apply only
after `restart`. Read a whole live partition back with `etc/dump_littlefs.py` — it **reboots the
board**. Details: Bench Operations.

`wifi.conf` is gitignored repo-wide (it holds the lab PSK), so **no profile in a fresh clone has
one** and a board provisioned from it comes up with no credentials — set them with
`wifi-add <ssid>:<psk>` over serial/BLE, creds are in the global `~/.claude/CLAUDE.md`, not the repo.
Keys: `doc/Configuration.md`. For the same reason, don't assert on `wifi.conf` in a host test.

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

Then **confirm over the air**, not from the log — a `bleak` `BleakScanner.discover()` scan,
matching by **name**: with BLE_ADV telemetry the adv payload has no room for the NUS UUID, so a
UUID-filtered scanner shows nothing while the board is fine. A board **absent from scans entirely**
is usually *held*, not dead: `NIMBLE_MAX_CONNECTIONS=1`, and a connected board stops advertising
(or advertises non-connectably) — culprits are another agent's console or the rpi bridge, and the
link lives in bluetoothd (killing the client doesn't free it): `bluetoothctl disconnect <mac>` on
the holder, or `svc rs ble` on the device, restores it. A WiFi scan-loop against a missing AP also
starves BLE (`wifi off` fixed it). And don't trust the *first* scan after a rename — CoreBluetooth
caches `d.name` across scans; re-scan before concluding the rename failed.

The bench tooling's `--buck-name` defaults to `fugu-fbuck` and `--boost-name` to `fugu-fboost`, so
match those unless you also update the invocation.

A **bonded** Mac failing at subscribe with ATT code 3 ("Writing is not permitted") after a firmware
change that altered the GATT layout is a stale macOS GATT cache, not a firmware bug. The cache is
keyed by BLE address, so renaming doesn't help; the fix is toggling Bluetooth —
`blueutil -p 0 && blueutil -p 1` (`doc/OTA over BLE.md`) — noting `blueutil` is **not installed**
by default on this Mac, and unpairing otherwise needs the GUI (ask the user).

An **unbonded** host is the opposite case, ATT code 5/15 `Insufficient Authentication` on the first
console write — it scans and connects fine, then every write fails. `ble_security` defaults to
`justworks` (encrypted link) and the `fbuck_lab_bench*` profiles ship no `ble.conf`, so a freshly
provisioned board refuses writes from a Mac it has never paired with. Pair it, or drop the link
encryption: `set-config ble.conf ble_security none` + **`restart`** — `svc rs`/`off`/`on ble` do
NOT re-apply `ble.conf` (props are set once at stack init and the wrapper is not reinit-safe,
`console_ble.cpp`). ATT code 17 "Insufficient Resources" is a third case: NimBLE mbuf starvation
from rapid reconnect cycles — `restart` the board, space out connects. Values and defaults are in
`doc/dev-notes/ble-dev.md`; symptom→cause table in Bench Operations.

## Console

The client (`doc/Agentic Programming.md` has the design; traps in Bench Operations):

```bash
timeout 30 .venv/bin/python3 etc/fugu_console.py -p PORT -c "status" -c "svc" | grep -a -v '^V='
```

Not executable — always via `.venv/bin/python3`; always `timeout`-wrapped (it doesn't exit);
`grep -a` (stream has binary bytes). **`^V=` above IS the status line** — drop it to read command
replies, KEEP it (`grep -a '^V='`) to read rig state, which lives nowhere else: `V=Vin/Vout`,
`I=Iin/Iout`, W, both temps, `sps`, `CCM|DCM(H|L|Lm)=` counts, `st=` mode, `lag`, `N`. `status`
alone reports limits and averages, not the operating point. Batched replies interleave with log
output and can
**fabricate** errors (`Command not found` next to the correct reply) — re-send singly before
trusting an `ERR:`, or quiet the board first (`log <tag> error`). A read batched right after a
state-change shows the **pre-change** state — sleep 3–10 s between; ~12 s after `restart`. Async
events (trip reasons) only reach a connection open when they fire: trigger + device-side `sleep` +
read in ONE `--stdin` script. `bf`/`dc`/`sync` are silently no-ops outside manual mode — verify in
the status line, absence of `ERR` proves nothing.

**While diagnosing, send only read-only commands.** Safe: `bootinfo`, `tasks`, `uptime`, `status`,
`svc`, `hostname` (no arg), `ls`, `cat`, `get-config`, `rt-stats`, `mem`, `heap`, `ip`,
`pwm-dump` (the only readout of the real pwmMax), `wsync`, `scan-i2c`.

Ask first before: `bf`/`panel`, `dc`, `sweep`, `mppt`, `sync`, `psu`, `vset`/`iset`/`ovset`,
`measure-coil`, `restart`, `ota`/`ota-ble`, `short-ls`, `adc-restart`, `adc-reset`. A bare `bf` or
`dc` takes an argument-less default that changes converter state — this has altered someone's live
test before. Flashing firmware is pre-authorised on bench units; toggling converter state is not
the same thing.

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
`dcdc-tools/verifications/vin-sweep/BRIEF-power-loop-rig.md` are the references. Four standing traps:

* **Low duty is the dangerous end.** Below fboost's count, fbuck boosts back into it and the
  reverse-current protection trips. The trip is protection working, not a fault.
* **`--sync forced` or it silently runs DCM**, which `loss.py` assumes away. It's read back from
  firmware and the point is refused on mismatch.
* **fboost must be LOCKED at `dc 2499`** — every pwr-metering ladder was calibrated there and
  preflight refuses otherwise. Set it if it is off (standing operator instruction, 2026-08-14).
  The firmware comes up at 2499 **by itself**, so a `dc 0` reading straight after a shutdown can be
  stale: observed 2026-08-14, preflight refused on "dc 0" and a direct read a minute later showed
  2499 with fresh telemetry. Re-read before concluding the boost is down.
* **Shutdown order: fboost duty DOWN FIRST, then fbuck to zero.** Reversing it unloads a pumping
  fboost into its reverse-current trip. Loaded pwr-metering runs deliberately **end energised** at
  the lowest ladder rung (parking to zero would collapse the loop), so shutdown is a separate
  operator step and the runner says so on every exit.

For bench measurement conventions (ground springs not clips, `--probes`/`--dut`, restarting the
bench daemon after editing `bench/*.py`), see that brief's trap list — it applies to anything run
on this rig.
