*this document is an LLM generated placeholder*

# Codex review r1: docs site migration (privacy and migration integrity)

Reviewer: codex-cli 0.154.0, `codex exec`, 2026-09-29. Prompt and full log: `~/codex-reviews/docs-site-r1/`. Findings were verified against the code before fixing; see the fix notes in the session summary.

## ACCESS

Both fetched successfully through Playwright:

1. Local intro: `<title>Introduction | Fugu MPPT Firmware</title>`.
2. GitHub dependency: `<title>GitHub - fl4p/esp-ota-ble: OTA firmware update over BLE for ESP32 — transport-agnostic receiver (staging ring, credit-window flow control, streaming SHA-256) · GitHub</title>`.

## FINDINGS

Reviewed all 31 moved Markdown pages against HEAD `695feee`, the configuration split, five lab extracts, sources, and built artifacts. Findings explicitly identified as retained already existed at HEAD.

1. **High — (a) Wrong topology key in the power-loop recipe.**  
   [power-loop.md:26](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/power-loop.md:26) instructs `boost=1`; line 40 uses `boost=0`. The implementation reads **`topo`**, defaulting to **buck** ([src/buck.h:1111](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:1111)). Copying the boost recipe without an existing `topo=boost` leaves the wrong topology selected. Replace these with `topo=boost`/`topo=buck`. Retained from `HEAD:doc/Power Loop.md:11`.

2. **High — (a) Comparator writes are falsely described as atomic and order-independent.**  
   [pwm-drivers.md:114](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/internals/pwm-drivers.md:114) says both writes commit atomically and their order is irrelevant. [src/buck.h:969](/Users/fab/dev/pv/fugu-mppt-firmware/src/buck.h:969) explicitly explains that TEZ between writes publishes a mixed pair; lines 979–989 describe possible simultaneous gate conduction and implement direction-dependent ordering. This retained documentation contradicts an implemented shoot-through precaution.

3. **High — (b) The hostname safety claim assumes the maintainer’s naming convention.**  
   [automated-bench-tests.md:250](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/automated-bench-tests.md:250) says real converters cannot match the verifier’s allow-list. The guard only matches `^fugu(-esp32s3-.*)?$` ([mcpwm_gate_verify.py:160](/Users/fab/dev/pv/fugu-mppt-firmware/etc/mcpwm_gate_verify.py:160)); unnamed real boards receive that same default hostname ([tele_core.cpp:25](/Users/fab/dev/pv/fugu-mppt-firmware/src/tele/tele_core.cpp:25)). The verifier then drives near-full gate duty ([mcpwm_gate_verify.py:227](/Users/fab/dev/pv/fugu-mppt-firmware/etc/mcpwm_gate_verify.py:227)). State that the name check does **not** establish that power hardware is disconnected.

4. **High — (c) Two authored pages are silently gitignored.**  
   [.gitignore:94](/Users/fab/dev/pv/fugu-mppt-firmware/.gitignore:94), `build-*`, ignores both:
   - `website/docs/development/build-speed.md`
   - `website/docs/guide/getting-started/build-options.md`

   Neither is tracked. Existing pages link to them, including [build.md:96](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/development/build.md:96) and [getting-started/index.md:83](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/getting-started/index.md:83). Normal staging omits them despite the successful local build; the configured broken-link checks would then fail in a clean checkout. Narrow the ignore rule or explicitly exempt these sources.

5. **Med — (c) The built client bundle exposes the author’s absolute filesystem path.**  
   Browser retrieval confirmed `/Users/fab/dev/pv/fugu-mppt-firmware` and its `website/docs` subdirectory in [main.4be71849.js:2](/Users/fab/dev/pv/fugu-mppt-firmware/website/build/assets/js/main.4be71849.js:2). They originate in the remark-plugin options at [docusaurus.config.ts:51](/Users/fab/dev/pv/fugu-mppt-firmware/website/docusaurus.config.ts:51), which Docusaurus serializes into client configuration. Keep absolute paths inside build-time plugin code. A CI rebuild would substitute its runner paths; the reviewed local artifact already leaks the personal paths.

6. **Med — (c) Published links bypass the “Not published” boundary.**  
   [agentic-programming.md:384](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/development/agentic-programming.md:384) links two `doc/superpowers/` documents; [automated-bench-tests.md:15](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/automated-bench-tests.md:15) links `doc/Test Cases.MD`. Both categories are excluded by [plans/docs-site.md:117](/Users/fab/dev/pv/fugu-mppt-firmware/plans/docs-site.md:117). Playwright confirmed rendered GitHub links. [repo-links.mjs:18](/Users/fab/dev/pv/fugu-mppt-firmware/website/src/remark/repo-links.mjs:18) checks existence but imposes no publication restriction; an in-memory probe likewise accepted `doc/lab/live-converters.md`. Remove the current links and enforce the intended boundary.

7. **Med — (c) The coil lab extract is incomplete.**  
   [coil-inductance-fry-flat.md:100](/Users/fab/dev/pv/fugu-mppt-firmware/doc/lab/coil-inductance-fry-flat.md:100) ends mid-sentence: “under-reports current ~6–7 % (see”. It omits `HEAD:doc/Coil Inductance Measurement.md:362`: the calibration-analysis reference and the conditional instruction to trim `Iout` calibration, with `L0` following. Its “verbatim” claim at line 5 is therefore inaccurate. Additionally, this extract changes the “Open questions” heading level, and the latency extract changes two heading levels versus HEAD lines 367 and 398. Restore the missing line and qualify or honor “verbatim.”

8. **Med — (a) BLE OTA completion guarantees remain contradictory.**  
   [ota-ble.md:131](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/updating/ota-ble.md:131) presents disconnect plus re-advertising as success without separating transports. But `HEAD:doc/ble-ota-transports.md:15` already requires the target digest on the new running slot for direct OTA, preserved at [ble-ota-transports.md:20](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/updating/ble-ota-transports.md:20). The [public dependency documentation](https://github.com/fl4p/esp-ota-ble/blob/main/doc/host-transports.md), verified through Playwright, explicitly limits advertising-only confirmation to the proxy. Apply the planned direct/proxy distinction.

9. **Med — (a) The dead-time contradiction explicitly targeted by the plan remains.**  
   [pwm-drivers.md:87](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/internals/pwm-drivers.md:87) says HS has no delay and the gap equals `dtHlTicks`; line 105 correctly says `dtHlTicks − 1`. The implementation installs a one-tick HS falling-edge delay ([mcpwm.h:206](/Users/fab/dev/pv/fugu-mppt-firmware/src/pwm/mcpwm.h:206)). Reconcile the earlier explanation with the actual realized gap. This is retained from HEAD, not a changed number.

10. **Med — (c) The filtering page retains the speculative advice the plan required replacing.**  
    [signal-filters.md:72](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/internals/signal-filters.md:72) still publishes “Claude Recommends,” including dropping the Vout notch and changing filtering architecture, without explaining the implemented adaptive notch. The actual implementation defaults adaptive tuning on ([sampling.h:293](/Users/fab/dev/pv/fugu-mppt-firmware/src/adc/sampling.h:293)), searches 80–140 Hz, and gates retuning on SNR. This misses the explicit requirement at [plans/docs-site.md:143](/Users/fab/dev/pv/fugu-mppt-firmware/plans/docs-site.md:143).

11. **Med — (c) The public profile recipe still provisions an unsanitized, incomplete lab profile.**  
    [config-profiles.md:60](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/config-profiles.md:60) directly provisions `dry_mock`, then instructs users to replace networking settings afterward. That profile contains nonempty broker credentials at [mqtt.conf:3](/Users/fab/dev/pv/fugu-mppt-firmware/config/lab/dry_mock/conf/mqtt.conf:3) and lacks `charger.conf`, whose positive `vout_max` is required by [charger.h:49](/Users/fab/dev/pv/fugu-mppt-firmware/src/charger.h:49). The safer copy/add-charger/remove-MQTT procedure already exists at [first-power-up.md:39](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/getting-started/first-power-up.md:39). Use it consistently. This concerns provisioning existing credentials, not credentials embedded in the site.

12. **Med — (c) Prominent console commands fail on a clean checkout.**  
    [getting-started/index.md:31](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/getting-started/index.md:31), [connecting.md:17](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/connecting.md:17), and other pages invoke `./etc/fugu_console.py`. Its Git mode is `100644`, without executable permission. This was explicitly documented in `HEAD:doc/Bench Operations.md:19–20` and remains acknowledged at [bench-operations.md:24](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/bench-operations.md:24). Use an explicit Python interpreter consistently.

13. **Med — (b) The BLE quick start assumes the author’s private shell helper exists.**  
    [ota-ble.md:21](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/guide/updating/ota-ble.md:21) starts with `. ./idf-export.sh`. The file is untracked and explicitly ignored at [.gitignore:48](/Users/fab/dev/pv/fugu-mppt-firmware/.gitignore:48). Its maintainer-specific behavior is documented in `HEAD:doc/Bench Operations.md:15–23`. A reader’s clone lacks it. Use the generic ESP-IDF export instructions already provided elsewhere, with this helper clearly optional.

14. **Low — (c) A generic recovery precaution disappeared with the multi-agent section.**  
    The migrated [bench-operations.md:223](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/lab/bench-operations.md:223) ends without HEAD’s instruction to confirm BLE or network access before removing USB (`HEAD:doc/Bench Operations.md:236–238`). It survives only in the internal archive. This is a generic board-access precaution, not identifying data or multi-agent coordination, and should remain in the public procedure.

15. **Low — (c) Stale references remain.**  
    [logging.md:25](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/development/debugging/logging.md:25) still names `doc/dev-notes/Real-Time Latency.md`; [lfp-longevity.md:261](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/internals/lfp-longevity.md:261) names the old charging and termination paths; [runtime-pwm-frequency.md:206](/Users/fab/dev/pv/fugu-mppt-firmware/plans/runtime-pwm-frequency.md:206) retains `doc/dev-notes/{beacon,wired}-sync.md`. Their replacements are explicit in the [migration mapping:14](/Users/fab/codex-reviews/docs-site-r1/migrate_docs.py:14), lines 32–34 and 44. Also, [dcm-ringing-boards.md:120](/Users/fab/dev/pv/fugu-mppt-firmware/doc/lab/dcm-ringing-boards.md:120) preserves a now-broken relative `Diode Emulation.md` link; add a current navigation link outside the verbatim extract.

16. **Low — (a) The PWM worked frequency is arithmetically wrong.**  
    [pwm-drivers.md:75](/Users/fab/dev/pv/fugu-mppt-firmware/website/docs/internals/pwm-drivers.md:75) gives approximately 38,997 Hz for 160 MHz / 4,103. The quotient is **38,995.8567 Hz**; the integer `actual_freq` returned by [mcpwm_timing.h:18](/Users/fab/dev/pv/fugu-mppt-firmware/src/pwm/mcpwm_timing.h:18) is **38,995**. Retained from `HEAD:doc/mcpwm-sync-buck-driver.md:70`.

## SURVIVED

- **Most privacy scrubbing held:** no additional device hostnames, private IPs, `havan*`, `fabi.me`, device-specific MAC/BLE identities, personal emails, or private-repository links found in published sources/build/search index. Profile folder names, the standard NUS UUID, and placeholder MAC were not counted as leaks.
- **Lab credentials:** no credentials found in the five `doc/lab/` extracts.
- **Technical preservation:** coil/DCM measurements, formulas, retractions, and uncertainty survived de-identification. The split configuration reference retained its substantive content and numerical values apart from deliberate corrections.
- **Archive preservation:** bench, DCM, and live-converter extract bodies matched HEAD verbatim; latency sections were complete apart from heading-level changes. The coil truncation is finding 7.
- **Images:** all four migrated plots are byte-identical to HEAD, visually contain no identifying labels, and contain no EXIF/XMP chunks.
- **Navigation:** all 3,628 local links/anchors checked across 85 built HTML pages resolved. Current repository hyperlink targets existed.
- **Link transformer:** correct GitHub URL construction, space encoding, and fragment preservation passed; a missing repository-file probe threw as intended.
- **Workflow:** website path trigger, `fetch-depth: 0`, Pages/OIDC permissions, artifact path, and `npm ci` configuration are present. Lockfile dependencies match `package.json`. Fresh CI deployment remains **unverified**; no rebuild or deployment was run.
- **Planned corrections that held:** `tele`, BLE and bsync defaults; explicit ADC selection without fallback; BTHome proposal labeling; complete Kconfig option coverage; separation of internal-loopback and external-scope verification.
- **Internal coordination:** resource-lock instructions and the multi-agent bench section were removed from published pages.
