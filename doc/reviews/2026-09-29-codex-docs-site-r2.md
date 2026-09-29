*this document is an LLM generated placeholder*

# Codex review r2: docs site migration (technical accuracy)

Reviewer: codex-cli 0.154.0, `codex exec`, 2026-09-29. Prompt and full log: `~/codex-reviews/docs-site-r2/`. Findings were verified against the code before fixing; see the fix notes in the session summary.

## ACCESS

1. Local intro `<title>`: **unverified** — no Playwright MCP tools were exposed in this session.
2. GitHub `fl4p/esp-ota-ble` `<title>`: **unverified**, same limitation.

Browser rendering remains **unverified**; static HTML inspection cannot establish Mermaid rendering or table layout. Claims requiring excluded files, hardware testing, or external InfluxDB verification also remain unverified. Ran the requested Git inventories. No files changed.

## FINDINGS

1. **[high · (a) factual error] Zero fallback does not disable conversion.**  
   [charger.md:16](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/reference/config/charger.md:16) promises `vout_max_fallback=0` disables the converter. [charger.h:714](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:714) applies `vpack_pin` only when **positive**; once the fallback reaches zero, the effective limit returns to `Vbat_max`. Its documented “build-time” default is also wrong: [charger.h:56](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:56) derives it from cell count × `cv_float`. The BMS guide correctly warns about zero; the reference contradicts it.

2. **[high · (a) factual error] The power-loop recipe uses ignored topology and voltage keys.**  
   [power-loop.md:25](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/power-loop.md:25) prescribes `boost=1` and `converter.conf::vout_max=75`; the buck example repeats these legacy keys. [buck.h:1111](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1111) reads `topo`, defaulting to **buck**. Voltage targets and hard limits come from [charger.h:49](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:49) and [mppt.h:48](/Users/fab/dev/pv/fugu-mppt-firmware/src/mppt.h:48). Following this recipe does not establish the intended topology or target voltage. The console’s `set-config converter.conf vout_max 28.5` example has the same defect.

3. **[high · (b) unstated assumption] “Always safe” diagnostics assume sufficient ADC timing headroom.**  
   [agentic-programming.md:320](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/development/agentic-programming.md:320) calls `tasks` and `rt-stats` always safe; [testing.md:51](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/development/testing.md:51) labels the console cluster safe on live converters. But [cli.cpp:544](/Users/fab/dev/pv/fugu-mppt-firmware/src/cli.cpp:544) calls `uxTaskGetSystemState()`, and [adc_esp32_cont.h:22](/Users/fab/dev/pv/fugu-mppt-firmware/src/adc/adc_esp32_cont.h:22) explicitly documents that busier configurations can exceed DMA headroom and require recovery. The cluster invokes these commands in [test_console_plan.py:62](/Users/fab/dev/pv/fugu-mppt-firmware/etc/e2e-test/test_console_plan.py:62). Conditional safety is defensible; the unconditional promise is not.

4. **[high · (a) factual error] Reverse-current protection’s default is inverted.**  
   [limits.md:21](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/reference/config/limits.md:21) says `reverse_current_paranoia` defaults to `0`. [mppt.h:53](/Users/fab/dev/pv/fugu-mppt-firmware/src/mppt.h:53) defaults it to **1**. This also changes the derived overvoltage multiplier, [mppt.h:1240](/Users/fab/dev/pv/fugu-mppt-firmware/src/mppt.h:1240), so it is more than an incorrect label.

5. **[med · (b) unstated assumption] The first-install quick start assumes an already provisioned flash layout.**  
   [getting-started/index.md:29](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/getting-started/index.md:29) uses only `app-flash`, then provisioning. ESP-IDF registers only the application image for that target: [esptool_py/CMakeLists.txt:20](/Users/fab/dev/esp/idf5.5/components/esptool_py/CMakeLists.txt:20). [provision.py:79](/Users/fab/dev/pv/fugu-mppt-firmware/etc/idf-devtools/provision.py:79) only writes the named filesystem partition. Neither installs the bootloader or partition table. A blank board cannot reach a working installation through this sequence.

6. **[med · (a) factual error] Recovery says it preserves configuration while executing a destructive full flash.**  
   [troubleshooting.md:58](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/troubleshooting.md:58) says “keeping the board configuration,” then runs `idf.py flash`. [CMakeLists.txt:127](/Users/fab/dev/pv/fugu-mppt-firmware/CMakeLists.txt:127) includes littlefs in that operation. The subsequent warning acknowledges the overwrite but does not make the preservation instruction true; reprovisioning also loses device-specific edits.

7. **[med · (c) real defect] The public-content boundary is not enforced.**  
   Three confirmed publication leaks remain:
   - [config-profiles.md:31](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/config-profiles.md:31) publishes `fbuck`/`fboost` profile names explicitly excluded by [plans/docs-site.md:12](/Users/fab/dev/pv/fugu-mppt-firmware/plans/docs-site.md:12). They also appear in generated HTML and the search index.
   - [docusaurus.config.ts:51](/Users/fab/dev/pv/fugu-mppt-firmware/website/docusaurus.config.ts:51) passes absolute paths as plugin options. The supplied [built JavaScript:2](/Users/fab/dev/pv/fugu-mppt-firmware/website/build/assets/js/main.4be71849.js:2) actually contains `/Users/fab/dev/pv/fugu-mppt-firmware`. A CI rebuild would change the path, not remove the serialization.
   - [agentic-programming.md:384](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/development/agentic-programming.md:384) links excluded `doc/superpowers` material; [automated-bench-tests.md:15](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/automated-bench-tests.md:15) links excluded `Test Cases.MD`. [repo-links.mjs:18](/Users/fab/dev/pv/fugu-mppt-firmware/website/src/remark/repo-links.mjs:18) checks existence but rewrites these into public GitHub links.

8. **[med · (a) factual error] Copyable inductance commands are wrong by a million.**  
   [console.md:83](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/reference/console.md:83), its script example, and Agentic Programming use `set-config coil.conf L0 50`. `L0` is in **henries**; 50 µH requires `50e-6`. [buck.h:1136](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1136) reads the value directly and checks `fsw × L0 × 0.95`. The example stores 50 H and causes initialization to fail on reload/reboot.

9. **[med · (c) real defect] The Lab mock quick start cannot run the claimed control loop.**  
   [lab/index.md:30](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/index.md:30) provisions `dry_mock` unchanged and claims the loop runs. That profile lacks `charger.conf`; [charger.h:49](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:49) defaults missing `vout_max` to NaN and rejects it at line 55. [first-power-up.md:41](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/getting-started/first-power-up.md:41) already supplies the missing file. The Lab recipe needs the same correction.

10. **[med · (c) real defect] The “minimal” internal-ADC example omits mandatory settings.**  
    [examples.md:10](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/reference/config/examples.md:10) omits `esp32adc1_avg` and `esp32adc1_sr`. [adc_esp32_cont.h:66](/Users/fab/dev/pv/fugu-mppt-firmware/src/adc/adc_esp32_cont.h:66) reads both without usable defaults and rejects an averaging count outside 1–1023. `ignore_calibration_constraints` cannot bypass this constructor check. Its GPIO comments also describe S3 mappings while calling the example merely “ESP32.”

11. **[med · (a) factual error] The sensor transform equation is wrong.**  
    [sensors.md:44](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/internals/sensors.md:44) gives `y = factor·x + midpoint`. [sampling.h:29](/Users/fab/dev/pv/fugu-mppt-firmware/src/adc/sampling.h:29) implements **`(x − midpoint) × factor`**. The documented equation gives incorrect calibration values whenever midpoint is nonzero.

12. **[med · (a) factual error] Build defaults confuse Kconfig fallback values with repository defaults.**  
    [build-options.md:39](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/getting-started/build-options.md:39) lists MCPWM and WSYNC as off. [sdkconfig.defaults:245](/Users/fab/dev/pv/fugu-mppt-firmware/sdkconfig.defaults:245) enables both. Label the table as bare Kconfig defaults or report what a fresh repository build actually selects.

13. **[med · (c) real defect] The single-test build examples remove the test runner.**  
    [testing.md:42](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/development/testing.md:42) recommends `MAIN_SRC=../test/test_buck.cpp`; [agentic-programming.md:178](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/development/agentic-programming.md:178) does likewise for `test_vconv.cpp`. These files contain test functions, not application entry points. [main/CMakeLists.txt:29](/Users/fab/dev/pv/fugu-mppt-firmware/main/CMakeLists.txt:29) replaces the source list, omitting the `setup()`/`loop()` runner in [test/main.cpp:366](/Users/fab/dev/pv/fugu-mppt-firmware/test/main.cpp:366). These are not valid standalone test builds.

14. **[med · (a) factual error] The bootloader offset is S3-specific but presented as universal.**  
    [partitions.md:19](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/reference/partitions.md:19) lists `0x0`. Classic ESP32 uses **`0x1000`**, as defined by [bootloader/Kconfig.projbuild:9](/Users/fab/dev/esp/idf5.5/components/bootloader/Kconfig.projbuild:9). Distinguish the targets in the flash map.

15. **[med · (a) factual error] Rectifier timing conversion uses the wrong count range.**  
    [coil.md:13](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/reference/config/coil.md:13) and [diode-emulation.md:129](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/internals/diode-emulation.md:129) give `ns × 1e-9 × fsw × pwmMax`. [buck.h:1067](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1067) explicitly explains why `pwmMax` is wrong: it excludes dead-time counts. MCPWM uses the peripheral resolution through `getPwmTickRate()`. The documented formula incorrectly makes a fixed physical delay depend on the reserved duty range.

16. **[low · (a) factual error] MPPT reset does not require 30 seconds continuously below 85%.**  
    [mppt-tracker.md:67](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/internals/mppt-tracker.md:67) describes sustained low power. [tracker.h:127](/Users/fab/dev/pv/fugu-mppt-firmware/src/tracker.h:127) instead tests current power below 85% and an **MPP record older than 30 seconds**. One low-power observation can reset an old record.

17. **[low · (a) factual error] The bench page incorrectly denies console key deletion.**  
    [bench-operations.md:157](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/bench-operations.md:157) says there is no way to clear a key from the console. [cli.cpp:1454](/Users/fab/dev/pv/fugu-mppt-firmware/src/cli.cpp:1454) implements `del-config`, also correctly documented elsewhere.

18. **[low · (a) factual error] The named inductance-bias constant and formula are misreported.**  
    [diode-emulation.md:52](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/internals/diode-emulation.md:52) says `InductivityDcBias=0.05`, applied as `1−constant`. [buck.h:68](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:68) defines **0.95**, multiplied directly at line 1144. The resulting 5% reduction is correctly described, but the source-level explanation is not.

19. **[low · (a) factual error] Fan control is currently binary, not variable-speed PWM.**  
    [console.md:46](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/reference/console.md:46) promises speed 0–100, and Build Options attributes it to LEDC. [cooling.h:47](/Users/fab/dev/pv/fugu-mppt-firmware/src/cooling.h:47) immediately performs `digitalWrite(duty > 0.1)` and returns. The PWM implementation below is unreachable: values above 10% turn it fully on.

## SURVIVED

- **Sibling checkout paths and recursive cloning:** the quick-start directory arrangement matches `ESP_OTA_BLE_DIR` and `ESP_BOOTGUARD_DIR`.
- **Application/data partition arithmetic:** OTA sizes, aligned offsets, 28 KiB inter-slot gap, and 92 KiB remaining space match `partitions.csv`.
- **MPPT principal constants:** fast/slow rates, steps, thresholds, slow-mode hysteresis, 5 W sweep minimum and 30 s unsuccessful-sweep backoff match source.
- **Services and telemetry:** `tele` and BLE default off; MQTT command input defaults off. Checked MQTT topic construction, telemetry field names/cadences, and the 17-byte advertising record match implementation.
- **BMS guide corrections:** 180 s freshness, derived fallback voltage, and its warning about zero fallback are supported by `charger.h`.
- **ADC/backend correction:** selection is explicit; there is no automatic ADS-to-internal fallback. The `conversion_eff` versus `power_conversion_eff` warning is accurate.
- **Bootguard:** the locally available dependency defaults to three counted crashes and download-mode recovery. This survived source inspection, not a hardware trial.
- **C++ standard:** the main component explicitly selects GNU C++20.
- **Config-editor behavior:** overlay replacement semantics, changed-key uploads, deletion commands and confirmation match the editor source.
- **Publication scans beyond finding 7:** no additional author hostnames, private addresses or credentials were identified in the inspected site text; the MAC matches were explicit placeholders.
- **Static math output:** the supplied diode-emulation HTML contains KaTeX markup without `katex-error`. Visual math, Mermaid and table rendering remain unverified.
