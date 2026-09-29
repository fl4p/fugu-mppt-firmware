*this document is an LLM generated placeholder*

# Docusaurus documentation site — outline

## Decisions

| Topic      | Decision                                                                                      |
|------------|-----------------------------------------------------------------------------------------------|
| Source     | `website/docs/`, kebab-case filenames. `doc/` keeps internal notes only                       |
| Versioning | none                                                                                          |
| Hosting    | GitHub Pages, `url: https://fl4p.github.io`, `baseUrl: /fugu-mppt-firmware/`                  |
| Scope      | generic — no fry/flat, NAT router, havan, credentials, resource locks, fboost/fbuck/flu names |
| Plugins    | mermaid, KaTeX (remark-math/rehype-katex), local search — already in `website/package.json`   |
| Markdown   | keep `markdown.format: 'detect'` — 7 of 37 sources fail as MDX, all pass as CommonMark        |

A ✅ below means the text is current; it does NOT mean publishable as-is. Every moved page gets a genericity pass
(see "Review fixes").

Legend: ✅ existing page, light edit · ✏️ existing material, rewrite/merge · 🆕 to write

## Sidebar 1 — Guide

1. **Introduction** (`/docs/intro`) — what Fugu is, features, topologies (buck, boost, PSU, PV-sim) ✏️ README intro
2. **Hardware**
    - Supported boards: Fugu2/fmetal, Fugu1 (ADS1015 / internal ADC), solar-boost, psu_12v ✏️ README + `config/`
    - ESP32-S3 vs classic ESP32 🆕
    - Wiring & internal ADC ✅ `Internal ADC.md`
3. **Getting started**
    - Toolchain — generic ESP-IDF install/export with the reader's IDF path; `idf-export.sh` only as an adaptable
      convenience (it hardcodes `../../esp/idf5.5`) ✏️ README + `Bench Operations.md`
    - Build & flash, Kconfig feature flags ✏️ README "Building"/"Configuring Build"
    - Provision a board config (`provision.py`) ✏️ README "Board Configuration"
    - First power-up checklist (dry mock → real panel) 🆕
4. **Connecting** — serial, telnet, BLE, MQTT; `fugu_console.py` ✏️ `Console.md` + `Agentic Programming.md` §1
5. **Updating firmware**
    - OTA over Wi-Fi (`ota.py`, `-n`/`-m`, rollback safety net) ✏️ CLAUDE.md OTA section
    - OTA over BLE ✅ `OTA over BLE.md` + `ble-ota-transports.md`
6. **Battery charging**
    - LFP charging ✅ `LFP Charging.md`
    - Termination & recharge ✅ `Termination.md`
    - BMS integration via MQTT ✏️ README "BMS Communication"
7. **Operating modes** — MPPT, manual duty, PSU CV, PV simulator 🆕 from `plans/psu-mode.md`, `plans/pv-sim-mode.md`
8. **Telemetry & Home Assistant** — MQTT/HA, InfluxDB over UDP, BLE telemetry stream, custom manufacturer-data
   advertising (`src/tele/tele_adv.cpp`), optional BLE→Influx relay ✏️ `dev-notes/ble-telemetry.md`.
   BTHome is a **proposal** (not implemented) — label it so or keep it off the Guide ✏️ `BTHome Advertising.md`
9. **Troubleshooting / FAQ** — ADC errors, coredumps, brick recovery, stale GATT cache 🆕

## Sidebar 2 — Reference

- **Configuration files** — one page per conf file: board, sensor, limits, coil, converter, charger, tracker, wifi,
  mqtt, tele, ftp, telnet, lcd, scope, ble, bsync, pprof, vconv ✏️ split `Configuration.md`
- **Config editor** (`etc/config-tool/conf-editor.html`) 🆕
- **Console commands** — system / control / diagnostics / network / services ✏️ `Console.md`
- **MQTT topics & payloads** 🆕
- **Telemetry fields** (`Ui`, `Uo`, `P`, `I`, `lag`, …) 🆕
- **Services** (`svc`, per-service conf) ✅ `Services.md`
- **Partition layout & flash map** 🆕
- **Host tools** — `ota.py`, `ota_ble.py`, `scope.py`, `measure_coil.py`, `elf_archive.py`, `provision.py`,
  `dump_littlefs.py`, `fugu_health.py` 🆕

## Sidebar 3 — How it works

- **Architecture overview** — dual-core layout, RT loop, core pinning (mermaid) ✏️ CLAUDE.md "Architecture"
- **Control loop pipeline** — sampler → protection → PD controllers → tracker → PWM ✏️ README + `Control Loop.md`
- **Sensors & ADC backends** — ADS1x15, INA226, internal DMA, virtual sensors ✏️ README + `Sensors.md` +
  `dev-notes/ina226.md`
- **Signal filtering** — notch, median, EWM, adaptive ripple notch ✅ `Signal Filters.md` (+ `doc/img/` plots)
- **MPPT tracker** — sweep, fast/slow P&O ✏️ README
- **Synchronous buck & diode emulation** ✅ `Diode Emulation.md`
- **PWM drivers** — LEDC vs MCPWM, dead-time ✅ `mcpwm-sync-buck-driver.md`
- **DCM ringing** (theory; measurement lives in Lab) ✅ `DCM Ringing.md`
- **Multi-converter sync** — bsync beacon, wired sync ✅ `dev-notes/beacon-sync.md`, `bsync-beacon-node.md`,
  `wired-sync.md`
- **Background: LFP longevity** ✅ `LFP Longevity Research.md`

## Sidebar 4 — Lab

1. **Overview** — setups by requirement: no power stage (mock ADC), simulated converter (vconv, Wokwi), single
   converter + PSU/load, two-converter power loop; half-bridge safety rules 🆕
2. **Lab config profiles** — a few explicitly chosen, sanitized templates per setup; never `config/lab/*` wholesale
   (`dry_mock`/`wokwi_mock` `mqtt.conf` hold broker credentials) 🆕
3. **Bench operations** — port identification, flashing, port recovery, live config edits, BLE failure modes
   ✏️ generic parts of `Bench Operations.md`
4. **Power-loop rig** — boost → buck → PSU recirculation
    - Topology & purpose (mermaid) ✏️ `Power Loop.md`
    - Configuring both ends ✏️ `Power Loop.md`
    - Bring-up order & `fpwm_gate` ✅ `Power Loop.md`
    - PV-sim source (`mode=pv`) ✅ `Power Loop.md`
    - Pitfalls — LS current sensing reads ~0, no reverse blocking in forced PWM, PSU current limit is the only
      backstop ✏️ generic lessons from the fugu skill + a reviewed generic extract of the brief's §7–§8 traps.
      No public link: `fl4p/dcdc-tools` is private (404 for readers)
5. **Measurements** — scope capture of gate/switch node (PicoScope as example), gate-driver verification
   (`pwm-test-spec1.md`), coil inductance ✅ `Coil Inductance Measurement.md`, DCM ring frequency, efficiency &
   thermal soak 🆕, ADC noise via adcscope
6. **Automated bench tests** ✅ `Automated Bench Tests.md`

## Sidebar 5 — Development

- **Repo layout & submodules** (`idf-devtools`, `adcscope`, `fugu-py`) 🆕
- **Build** — Kconfig flags, sdkconfig layering, build dirs, ccache ✏️ `dev-notes/build-speed.md`
- **Coding conventions** — no `<sstream>`, newlib-nano printf, `IRAM_ATTR`, `-Werror` initializers, no delays on the RT
  path ✏️ CLAUDE.md "Conventions"
- **Testing** — Unity on-target 🆕, e2e suite (`etc/e2e-test`) ✏️, simulators (vconv, Wokwi) ✏️
  `dev-notes/simulator wokwi.md`
- **Debugging** — coredumps & ELF archive ✏️, `peek` ✅ `Peek Command.md`, logging ✅ `Logging.md`, real-time latency ✅
  `dev-notes/Real-Time Latency.md`, rtcount & profiler ✏️ `Real-time Counter.md`, `performance profiling.md`
- **Binary size budget** ✅ `dev-notes/binary size.md`
- **Agentic programming** ✅ `Agentic Programming.md`
- **Contributing** ✏️ README

## Blog / design notes (optional)

Dated posts, e.g. `2026-09-08-image-layout-stability-for-ota.md`.

## Not published

`doc/reviews/`, `doc/superpowers/`, `plans/`, `NOTES.MD`, `vibe.md`, `commerce.md`, `web.MD`, `Test Cases.MD`,
plan-style/one-off dev-notes (`ble-dev.md`, `services.md`, `telnet-wifi-off-uaf.md`, `tmp`, `hw failurs`,
`spin-off.md`), and all lab-specific material (fry/flat, NAT router, havan, credentials, resource locks, device names
and BLE identities).

Lab-only operational detail scrubbed from published pages (fry/flat recipes, NAT, live-converter procedures,
multi-agent bench rules, named-board case studies) moves to internal notes under `doc/lab/` — not only git history —
because the fugu skill, CLAUDE.md and memories cite it. The skill links those notes for lab work and
`website/docs/` for generic behaviour. Credentials stay in the gitignored env files, never in `doc/lab/`.

## Review fixes (codex, `doc/reviews/2026-09-29-codex-docs-site-plan.md`, verified against code)

**Genericity pass needed** (device names, `havan.local`, private IPs, lab campaigns): Agentic Programming (§ live
converters, ~346–435), Coil Inductance (fry/flat case study ~272–365, IP ~223), DCM Ringing (board table + private
paths ~37–77; keep the retractions), Real-Time Latency (~103, 193, 198, 367, 406), Power Loop (`fbuck_lab_bench`,
`fboost_pv`), Automated Bench Tests (~31–43, 249–253), wired-sync (flu wiring ~159, 199), beacon-sync (~65–83),
Diode Emulation (~166), Termination (~115), Bench Operations (multi-agent section ~224), Console (broker IP ~85),
BTHome (~180), LFP Longevity (local archive path ~203).

**Content wrong vs code — fix while moving:**
- `OTA over BLE.md`: `otab begin/end/abort` → `ota-ble begin/end/abort` (`src/cli.cpp:2088`); document direct-BLE vs
  proxy completion guarantees separately (`etc/ota_ble.py`).
- `Services.md`: service is `tele`, not `telemetry` (`src/tele/telemetry_service.h:19`), default off; add `bsync`.
- `Configuration.md`: BLE service defaults **off** (`src/tele/console_ble_service.h:23`).
- `Internal ADC.md`: no automatic ADS→internal fallback; backend is what `sensor.conf` selects.
- `mcpwm-sync-buck-driver.md`: reconcile dead-time gap statements (realized gap = dt−1) with `src/pwm/mcpwm.h`.
- `Signal Filters.md`: describe the implemented adaptive notch (`src/adc/sampling.h`), drop the speculative tail.
- `Automated Bench Tests.md`: separate runnable tools from proposed coverage; dead link to `Test Cases.MD`.
- `pwm-test-spec1.md` (internal loopback) ≠ PicoScope verifier — distinct Lab procedures.
- Build page: carry all Kconfig options from `main/Kconfig.projbuild` (BLE_TELE, BLE_ADV, WSYNC, BSYNC, LEDC, …).

**Migration mechanics:**
- Keep an old→new path manifest; update inbound refs: README (its `Serial Console.md` link is already broken),
  `src/charger.h`, `src/buck.h`, `src/cli.h`, `src/sync/bsync.h`, `src/main.cpp`, `etc/fugu_console.py`,
  `etc/e2e-test/test_measure_coil.py`, `test/test_pwm.cpp`, `etc/config-tool/spec.md`,
  `.claude/agents/ee-code-verifier.md`, CLAUDE.md, `~/.claude-sc/skills/fugu/SKILL.md`.
- CLAUDE.md conf-key rule → "update the per-file page under `website/docs/reference/config/` and the editor
  metadata"; leave a short `doc/Configuration.md` pointer until all references are migrated.
- `website/src/remark/repo-links.mjs` turns non-page relative links into GitHub URLs without an existence check —
  make it verify the target exists (or fail the build), and don't link readers to unpublished material.

**Ownership, one home each:** Services — config/commands in Reference, internals in How it works. Install — Getting
started; Bench ops = board identity/recovery; Development/Build = sdkconfig layering + speed. LFP Charging (settings)
vs Termination (algorithm). Coil — procedure in Lab, equations in How it works, CLI in Reference. Sync — theory in
How it works, wiring/bring-up in Lab.

## Next steps

1. `docusaurus.config.ts` (GitHub Pages, mermaid, KaTeX, local search) and `sidebars.ts` with the five sidebars.
2. Move ✅ pages into `website/docs/` (kebab-case, `git mv` to keep history), fix relative links and image paths.
3. 🆕 stub pages so the sidebar is complete; ✏️ pages rewritten incrementally.
4. GitHub Actions workflow to build and deploy to Pages.
