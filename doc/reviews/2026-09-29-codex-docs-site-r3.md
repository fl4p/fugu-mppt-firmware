*this document is an LLM generated placeholder*

# Codex review r3: docs site commit f7fd1cc

Reviewer: codex-cli 0.154.0, `codex exec`, 2026-09-29, with a working Playwright browser. Prompt and full log: `~/codex-reviews/docs-site-r3/`. Findings were verified against the code before fixing.

## ACCESS

Both fetched through Playwright:

1. Local Getting Started: `<title>Getting Started | Fugu MPPT Firmware</title>`.
2. GitHub: `<title>GitHub - fl4p/fugu-mppt-firmware: An open source Arduino ESP32 MPPT Charger firmware equipped with charging algorithms, WiFi, LCD menus & more! · GitHub</title>`.

Reviewed committed `f7fd1cc`, excluding uncommitted changes. Repository links below pin that commit. No files edited.

## FINDINGS

1. **High · (a) factual error — `dc 0` is not an unconditional stop.**  
   [operating-modes.md:75](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/guide/operating-modes.md#L75) says it “always works.” With an on-device coil measurement running, [cli.cpp:184](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/src/cli.cpp#L184) rejects **all** `dc` commands before inspecting the requested duty. A reader following the advertised shutdown procedure receives `dc: busy measuring` while the measurement continues driving PWM. Document this exception rather than promising an emergency stop.

2. **High · (a) factual error — the advertised build-time battery-voltage override does nothing.**  
   [build-options.md:63](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/guide/getting-started/build-options.md#L63) says `FUGU_BAT_V` hardcodes the maximum battery voltage. [main/CMakeLists.txt:150](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/main/CMakeLists.txt#L150) defines the macro, but no firmware source consumes it; [charger.h:49](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/src/charger.h#L49) unconditionally reads `charger.conf::vout_max`. Selecting a lower voltage through this advertised mechanism leaves the configured higher voltage effective.

3. **High · (a) factual error — r1’s comparator-ordering correction overstates its coverage.**  
   [pwm-drivers.md:119](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/internals/pwm-drivers.md#L119) says `buck.h` orders writes by direction so every mixed pair remains ordered. That logic exists in the **frequency-change** path at [buck.h:987](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/src/buck.h#L987). Ordinary per-tick commits still unconditionally write HS first, then LS at [buck.h:435](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/src/buck.h#L435). Narrow the documented guarantee. This review establishes the discrepancy, not a newly measured shoot-through event.

4. **High · (b) unstated assumption — the Measurements verifier recipe assumes disconnected power hardware.**  
   [measurements.md:50](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/lab/measurements.md#L50) provides wiring and a runnable command without requiring panel/battery disconnection. The verifier drives near-full duty at [mcpwm_gate_verify.py:227](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/etc/mcpwm_gate_verify.py#L227); its hostname check accepts default-named real boards. The necessary warning was added to [automated-bench-tests.md:251](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/lab/automated-bench-tests.md#L251), but this independent entry point omits it.

5. **Med · (a) factual error — documented host-tool arguments are rejected.**  
   [connecting.md:76](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/guide/connecting.md#L76) and [host-tools.md:51](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/reference/host-tools.md#L51) advertise `fugu_console.py --ble <name>`. [The parser:846](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/etc/fugu_console.py#L846) makes `--ble` boolean; the correct form is `--ble --name <name>`. An in-memory test of the committed parser returned exit 2 for the documented form. Separately, [host-tools.md:249](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/reference/host-tools.md#L249) advertises `influx_binary_proxy.py --insecure`, absent from its [parser:274](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/etc/influx_binary_proxy.py#L274).

6. **Med · (c) real defect — r1’s executable-permission fix missed Host Tools.**  
   All five commands at [host-tools.md:39](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/reference/host-tools.md#L39) directly execute `etc/fugu_console.py`. Its committed Git mode is **100644**, despite its [shebang:1](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/etc/fugu_console.py#L1). They fail with permission denied on a clean Unix checkout. Prefix them with Python, as the corrected Getting Started page does.

7. **Med · (c) real defect — another quick start still provisions the incomplete mock profile.**  
   [provisioning.md:18](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/guide/getting-started/provisioning.md#L18) directly provisions `config/lab/dry_mock`. Its committed tree lacks `charger.conf`; [charger.h:49–55](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/src/charger.h#L49) consequently rejects the missing voltage and prevents control-loop startup. Use the amended-copy procedure already at [first-power-up.md:39](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/guide/getting-started/first-power-up.md#L39). I did **not** find the previously reported mock-profile MQTT credentials in this commit.

8. **Med · (c) real defect — the publication guard implements only part of the exclusion policy.**  
   [plans/docs-site.md:117](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/plans/docs-site.md#L117) excludes `Test Cases.MD`, `NOTES.MD`, `vibe.md`, `commerce.md`, and `web.MD`. [repo-links.mjs:11](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/src/remark/repo-links.mjs#L11) only excludes five directories. In-memory probes accepted and rewrote links to **all five excluded files under `doc/`**. Missing-file and excluded-directory probes correctly threw. This is incomplete enforcement, not an observed current link leak.

9. **Med · (c) real defect — search fails on the supplied built-site server.**  
   Searching for `diode` requested `search-index.json?_=1932b012`, which redirected to `search-index.json/?_=1932b012` and returned HTML 404; the browser raised `Unexpected token '<'` and produced no results. The plain JSON URL succeeds. [docusaurus.config.ts:31](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docusaurus.config.ts#L31) enables query hashing; the installed Docusaurus [serve.js:60](/Users/fab/dev/pv/fugu-mppt-firmware/website/node_modules/@docusaurus/core/lib/commands/serve.js:60) misclassifies that query-bearing asset before slash normalization. **GitHub Pages search behavior remains unverified**; the demonstrated failure is local preview.

10. **Med · (c) real defect — large tables overflow the mobile page.**  
    [custom.css:22](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/src/css/custom.css#L22) forces tables to `display: table`, losing their bounded scrolling behavior. Through Playwright at 390 px, the [console table](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/reference/console.md#L44) expanded the document to **696 px**, and [board.conf](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/reference/config/board.md#L10) to **707 px**. At desktop width both also extend beyond the article column. Restore table-local horizontal scrolling.

11. **Low · (c) real defect — wide Mermaid diagrams become unreadably small on phones.**  
    The horizontal [architecture pipeline:42](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/internals/architecture.md#L42) and [power-loop diagram:16](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/lab/power-loop.md#L16) render successfully, but scale their roughly 1353/1093-unit widths into 343 px. Browser measurements imply approximately **4/5 px label fonts**. Provide scrolling, enlargement, or a narrower diagram arrangement.

12. **Low · (b) unstated assumption — the `<sstream>` explanation assumes the locally modified toolchain.**  
    [conventions.md:20](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/development/conventions.md#L20) attributes `basic_stringbuf_nop()` to Espressif’s toolchain. The installed header contains it, but Espressif’s matching [release-source constructor:121](https://github.com/espressif/gcc/blob/esp-14.2.0_20241119/libstdc%2B%2B-v3/include/std/sstream#L121), fetched through Playwright, does not. Keep the project’s anti-bloat convention, but identify the hook as local rather than promising this linker failure to stock-toolchain users.

13. **Low · (a) factual error — telemetry names the wrong lag-reset command.**  
    [telemetry-fields.md:59](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/reference/telemetry-fields.md#L59) says `rt-stats` resets `lag`. [cli.cpp:1031](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/src/cli.cpp#L1031) only prints runtime statistics; [cli.cpp:413](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/src/cli.cpp#L413) implements the reset in **`reset-lag`**. Otherwise a reader’s post-change observation still includes the earlier peak.

14. **Low · (c) real defect — the BLE build correction introduced a non-runnable shell line.**  
    [ota-ble.md:21](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/website/docs/guide/updating/ota-ble.md#L21) puts `idf.py menuconfig (enable CONFIG_FUGU_WITH_BLE), then idf.py build` inside a Bash block. With an inert `idf.py` stub, zsh fails before invocation: `no matches found: (enable CONFIG_FUGU_WITH_BLE),`. Put the instruction in a comment and provide separate runnable commands.

15. **Low · (c) real defect — CLAUDE.md’s new section is inside a shell fence.**  
    The fence opens at [CLAUDE.md:136](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/CLAUDE.md#L136), the new Documentation Site heading starts immediately afterward, and the fence closes at line 148. The entire policy renders as shell code and contaminates the copyable test command. Move the section outside that fence.

## SURVIVED

- **Commit isolation:** every inspected hunk in `src/`, `etc/`, and `main/` is a comment/docstring/help-text path rewrite. No functional firmware or Python changes found. README and `.claude/` changes are documentation changes; `/build-*` is the intended ignore-rule correction.
- **CLAUDE.md’s six requested technical corrections are accurate:** GNU C++20 ([CMake:241](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/main/CMakeLists.txt#L241)); runtime assertion ([main.cpp:605](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/src/main.cpp#L605)); timer ISR/task on core 0 ([sdkconfig.defaults:152](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/sdkconfig.defaults#L152)); target-dependent littlefs image ([CMakeLists.txt:130](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/CMakeLists.txt#L130)); actual ADC/watchdog messages; and `setupSensors()` in [sensor_setup.cpp:102](https://github.com/fl4p/fugu-mppt-firmware/blob/f7fd1cc/src/adc/sensor_setup.cpp#L102).
- **Earlier-review disposition:**

  | Earlier findings | Result |
  |---|---|
  | r1 #1, #4, #5, #7–11, #14–16 | Original-location fixes verified: keys, tracked pages, path serialization, complete extracts, OTA distinction, dead-time, notch, mock recipe, recovery precaution, references, arithmetic. |
  | r1 #2, #6, #12, #13 | Incomplete or introduced another error: findings 3, 8, 6, 14 above. |
  | r1 #3 | Original warning corrected; separate Measurements recipe still needs it, finding 4. |
  | r2 #1–6, #8–19 | Original-location corrections verified against committed source. Mock-profile repair remains inconsistent elsewhere, finding 7. |
  | r2 #7 | Absolute-path leak and existing excluded links removed; guard remains incomplete. Profile folder names remain, but were not counted as device identities under this run’s rule. |

- **Privacy:** scans covered committed `website/docs`, `website/src`, and supplied build HTML, search index, and JS bundles. No private host/address/device-identity or `/Users` leak found. No credentials found in `doc/lab/`. Placeholder MACs, example passwords, ordinary “flat,” and configuration-folder names were excluded from false positives.
- **Rendering:** KaTeX rendered 13 expressions on Diode Emulation and nine on LFP Charging, with fonts loaded and no KaTeX errors. All requested Mermaid pages rendered. Both OS tab groups switched together correctly. Landing-page content and five navigation cards worked. Responsive layout and search exceptions are above.
- **Other source checks:** MQTT topic construction/BMS expiry, telemetry field production and cadences, principal operating-mode transitions, troubleshooting thresholds, measurement-tool arguments, and bootguard integration survived, apart from the listed findings. Bootguard behavior was source-checked, not hardware-tested.
- **Clean-checkout prerequisites:** package/lockfile specifications match; Node 22 satisfies the declared engines; both previously ignored pages are tracked; full history, website triggers, Pages permissions, and artifact path are present. Current relative repository-link targets exist in the commit. **Fresh `npm ci`, build, and deployment remain unverified** because running them would write files.
