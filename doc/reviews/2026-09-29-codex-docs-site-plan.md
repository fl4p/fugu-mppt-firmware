*this document is an LLM generated placeholder*

## ACCESS

- Playwright fetched `https://docusaurus.io/docs/deployment`; `<title>`: **Deployment | Docusaurus**.
- `npm view @easyops-cn/docusaurus-search-local peerDependencies --json` failed with **EPERM** accessing npm’s cache. A no-write cache retry failed with **ENOTDIR**. The npm-command result is **unverified**.
- Direct registry retrieval succeeded. Latest version **0.55.3** reports:

```json
{
  "react": "^16.14.0 || ^17 || ^18 || ^19",
  "react-dom": "^16.14.0 || 17 || ^18 || ^19",
  "open-ask-ai": "^0.7.3",
  "@docusaurus/theme-common": "^2 || ^3"
}
```

`open-ask-ai` is optional. React 19 and Docusaurus 3 satisfy the declared peers; this is not a runtime-search test. [Registry metadata](https://registry.npmjs.org/@easyops-cn/docusaurus-search-local/latest)

**Scope correction:** config already exists in the working tree. It uses selected `doc/` files and one sidebar. I reviewed that existing, uncommitted scaffold alongside the plan. I made no repository edits or site build; syntax checks ran in memory.

## FINDINGS

Tags: **(a)** factual/mapping error; **(b)** unstated assumption that holds; **(c)** design defect. Ranked by consequence.

### 1. High — “✅ → move” would publish explicitly excluded material **(c)**

The blanket move step at [plans/docs-site.md:122](/Users/fab/dev/pv/fugu-mppt-firmware/plans/docs-site.md:122) conflicts with its genericity requirement. These are substantive publication edits:

| Source | Evidence | Actual work |
|---|---|---|
| [Agentic Programming.md:346](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Agentic Programming.md:346>) | Named live converters; NAT/proxy endpoints at 358–370; `havan.local` at 413 and 435 | Rewrite/remove the live-converter section and replace earlier named-device examples. Not a light edit. |
| [Coil Inductance Measurement.md:272](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Coil Inductance Measurement.md:272>) | Extensive fry/flat case study through 365; private-IP example at 223 | Extract the generic measurement procedure. Remove or anonymize the case study without losing measurement qualifications. |
| [DCM Ringing.md:37](</Users/fab/dev/pv/fugu-mppt-firmware/doc/DCM Ringing.md:37>) | Named-board table, private verification paths at 52–75, board-specific retractions from 77 | Substantial separation of theory from internal evidence. Preserve the retractions and uncertainty. |
| [Real-Time Latency.md:367](</Users/fab/dev/pv/fugu-mppt-firmware/doc/dev-notes/Real-Time Latency.md:367>) | Named-device incident narratives; more at 103, 193, 198, 406 | Extract current guidance from historical debugging notes. |
| [Power Loop.md:38](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Power Loop.md:38>) | `fbuck_lab_bench`; `config/lab/fboost_pv` at 103 | Replace owner profiles with complete generic examples, including the ✅ PV-sim subsection. |
| [Automated Bench Tests.md:31](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Automated Bench Tests.md:31>) | Named profiles at 31, 41–43; fry/flat and private IP at 249–253 | Rewrite setup/profile instructions and protection assumptions. |
| [wired-sync.md:159](/Users/fab/dev/pv/fugu-mppt-firmware/doc/dev-notes/wired-sync.md:159) | `flu`-specific USB-pad wiring; named validation at 199 | Separate general synchronization from this board-specific procedure. |
| [beacon-sync.md:65](/Users/fab/dev/pv/fugu-mppt-firmware/doc/dev-notes/beacon-sync.md:65) | fboost/fbuck campaign at 65–83 | Anonymize the experiment and retain its limitations. |
| [Diode Emulation.md:166](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Diode Emulation.md:166>) | Named-device timing offsets | Local rewrite. |
| [Termination.md:115](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Termination.md:115) | fry/flat incident | Local rewrite. |

The ✏️ sources also need deliberate extraction: [Bench Operations.md:224](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Bench Operations.md:224>) contains multi-agent working procedures; [Console.md:85](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Console.md:85) contains a private broker address; [BTHome Advertising.md:180](</Users/fab/dev/pv/fugu-mppt-firmware/doc/BTHome Advertising.md:180>) contains owner-specific calibration.

**The `config/lab/*` wildcard is particularly unsafe.** The plan calls these templates at line 73, but even apparently generic profiles contain configured credentials and private endpoints: `config/lab/dry_mock/conf/mqtt.conf:2,4` and `config/lab/wokwi_mock/conf/mqtt.conf:4`. Credential values are intentionally not repeated here. Select and sanitize explicit templates; do not copy that directory wholesale.

I scanned every uniquely cited Markdown source plus README, CLAUDE, and the external skill for the requested terms. Matches such as “flat key=value,” “InfluxDB,” lexical “token,” and ordinary console “session” are false positives, not leaks. “Claude Recommends” in Signal Filters is editorial residue; Codex provenance in LFP Longevity is not itself a credential leak. The latter does expose a local archive path at [line 203](</Users/fab/dev/pv/fugu-mppt-firmware/doc/LFP Longevity Research.md:203>).

### 2. High — BTHome is presented as available functionality, but its source is an implementation proposal **(a)**

[Plan lines 38–39](/Users/fab/dev/pv/fugu-mppt-firmware/plans/docs-site.md:38) place BTHome in the operating guide.

However:

- [BTHome Advertising.md:11](</Users/fab/dev/pv/fugu-mppt-firmware/doc/BTHome Advertising.md:11>) explicitly says HA BTHome discovery is unavailable.
- Lines 80 and 153 propose new firmware files and `bthome.conf`.
- Current [tele_adv.cpp:31](/Users/fab/dev/pv/fugu-mppt-firmware/src/tele/tele_adv.cpp:31) implements a custom 17-byte record with magic `0xF7`.
- [ble-telemetry.md:66](/Users/fab/dev/pv/fugu-mppt-firmware/doc/dev-notes/ble-telemetry.md:66) correctly documents manufacturer-data advertising and its decoder.

Document MQTT HA integration, BLE streaming, and custom advertising as implemented. Keep BTHome explicitly proposed until implementation exists.

### 3. High — Several “light edit” sources contain operationally wrong instructions **(a)**

| Source | Contradiction | Required correction |
|---|---|---|
| [Internal ADC.md:54](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Internal ADC.md:54>) | Claims automatic fallback when ADS is absent. [sensor_setup.cpp:34](/Users/fab/dev/pv/fugu-mppt-firmware/src/adc/sensor_setup.cpp:34) explicitly selects the configured backend; initialization failure throws at 52–54. | Explain sensor configuration changes alongside rewiring. |
| [OTA over BLE.md:62](</Users/fab/dev/pv/fugu-mppt-firmware/doc/OTA over BLE.md:62>) | Documents `otab begin/end/abort`; [cli.cpp:2090](/Users/fab/dev/pv/fugu-mppt-firmware/src/cli.cpp:2090) registers `ota-ble`. | Correct executable commands, distinguishing them from `OTAB` response records. |
| [OTA over BLE.md:130](</Users/fab/dev/pv/fugu-mppt-firmware/doc/OTA over BLE.md:130>) | Describes re-advertisement as success for the host generally. [ota_ble.py:199](/Users/fab/dev/pv/fugu-mppt-firmware/etc/ota_ble.py:199) verifies the exact image on a changed running slot for direct BLE; proxy verification remains weaker at 515–519. | Document the two completion guarantees separately. |
| [Services.md:146](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Services.md:146) | Calls the service `telemetry`; the actual name is `tele`. Its blanket network-service default at 175–176 also misses telemetry’s default-off behavior. | Match [telemetry_service.h:19](/Users/fab/dev/pv/fugu-mppt-firmware/src/tele/telemetry_service.h:19). Add registered `bsync`; correct the BLE header path at 128. |
| [Configuration.md:294](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Configuration.md:294) | Says BLE is enabled by default. [console_ble_service.h:22](/Users/fab/dev/pv/fugu-mppt-firmware/src/tele/console_ble_service.h:22) defaults it off. | Distinguish firmware fallback from supplied profile values. |
| [mcpwm-sync-buck-driver.md:87](/Users/fab/dev/pv/fugu-mppt-firmware/doc/mcpwm-sync-buck-driver.md:87) | Says HS has no delay and gap equals `dtHlTicks`; line 104 correctly says the realized gap is one tick shorter. Line 191 says the other transition is “realized exactly,” contradicting line 107 and code. | Reconcile timing explanations against [mcpwm.h:185](/Users/fab/dev/pv/fugu-mppt-firmware/src/pwm/mcpwm.h:185). |

These are stronger reasons to withdraw ✅ than the ubiquitous placeholder notices.

### 4. Medium — Some source-to-page promises exceed the source material **(a)**

- **Signal filtering:** the plan promises adaptive ripple-notch documentation at [line 60](/Users/fab/dev/pv/fugu-mppt-firmware/plans/docs-site.md:60). The source ends with speculative recommendations, including deleting ANF and changing filter order: [Signal Filters.md:71](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Signal Filters.md:71>). It does not explain the implemented adaptive detector in [sampling.h:293](/Users/fab/dev/pv/fugu-mppt-firmware/src/adc/sampling.h:293). This needs a rewrite against code.
- **Automated bench tests:** the source is primarily a test matrix and proposed automation, not instructions for an already automated full matrix. Its [automation section at 262](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Automated Bench Tests.md:262>) says to wrap the cases. Separate runnable tools from proposed coverage.
- **Gate-driver verification:** [pwm-test-spec1.md:3](/Users/fab/dev/pv/fugu-mppt-firmware/doc/pwm-test-spec1.md:3) describes zero-hardware internal-loopback tests. The external PicoScope verifier is separately documented at [Automated Bench Tests.md:244](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Automated Bench Tests.md:244>). The Lab mapping should distinguish those procedures.
- The tree already has `Architecture.md`, `Testing.md`, `Debugging.md`, and `OTA Updates.md`. Reuse/review those before recreating their material from CLAUDE or declaring it new.

All 37 uniquely named Markdown sources were found after resolving `doc/` and `doc/dev-notes/` shorthand. **No wholly nonexistent named source was found.** But `bsync-beacon-node.md`, `wired-sync.md`, `Real-time Counter.md`, and `performance profiling.md` need explicit `doc/dev-notes/` paths in an executable migration manifest.

### 5. Medium — The configuration inventory would drop real configuration surfaces **(c)**

[Plan lines 44–45](/Users/fab/dev/pv/fugu-mppt-firmware/plans/docs-site.md:44) promise one page per file but omit:

- `wifi.conf`: credentials/NVS precedence and roaming — [telemetry.cpp:46](/Users/fab/dev/pv/fugu-mppt-firmware/src/tele/telemetry.cpp:46).
- `pprof.conf`: profiler configuration — [main.cpp:446](/Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:446).
- `vconv.conf`: simulation configuration — [sensor_setup.cpp:62](/Users/fab/dev/pv/fugu-mppt-firmware/src/adc/sensor_setup.cpp:62).

“services” is not a configuration filename. Expand it explicitly into `ftp`, `telnet`, `lcd`, `scope`, `ble`, and `bsync`, alongside the already listed MQTT/telemetry files.

Two additional concrete inventory gaps:

- Reference’s six listed host tools omit configuration backup/extraction, [dump_littlefs.py:2](/Users/fab/dev/pv/fugu-mppt-firmware/etc/dump_littlefs.py:2), and the read-only health tool, [fugu_health.py:2](/Users/fab/dev/pv/fugu-mppt-firmware/etc/fugu_health.py:2).
- The build mapping relies on README’s shorter flag table. Preserve all current Kconfig options, particularly `BLE_TELE`, `BLE_ADV`, `WSYNC`, `BSYNC`, `LEDC`, and `INA226_MEASURED_RATE`, from [main/Kconfig.projbuild:36](/Users/fab/dev/pv/fugu-mppt-firmware/main/Kconfig.projbuild:36).

I would **not** invent additional command-page omissions: scripts, network diagnostics, manual PWM, and service commands already have a reasonable home in the proposed Console reference.

### 6. Medium — Moving files leaves maintenance instructions and inbound references behind **(c)**

The “fix relative links” step is insufficient. Concrete inbound references include:

- README: `Console.md` at 27, Internal ADC at 29/134, MCPWM at 102, Diode Emulation at 280. Its `Serial Console.md` link at 170 is **already broken**.
- Source: `src/charger.h:25`, `src/buck.h:70`, `src/cli.h:8`, `src/sync/bsync.h:10`, `src/main.cpp:748`.
- Tools/tests: `etc/fugu_console.py:5`, `etc/e2e-test/test_measure_coil.py:26`, `test/test_pwm.cpp:1`.
- Maintenance rules: [CLAUDE.md:126](/Users/fab/dev/pv/fugu-mppt-firmware/CLAUDE.md:126), `.claude/agents/ee-code-verifier.md:62`, `etc/config-tool/spec.md:175`.
- External skill: [~/.claude-sc/skills/fugu/SKILL.md:259](/Users/fab/.claude-sc/skills/fugu/SKILL.md:259), plus references at 229, 341, 428, and 445.

**Smallest mitigation:** define an old-path → new-path manifest; update active references in the migration; retain a temporary `doc/Configuration.md` pointer to a stable new configuration index. Change the key-maintenance rule to require updating the corresponding per-file page **and** editor metadata. The split itself is workable.

### 7. Medium — The plan and existing scaffold disagree about source paths, routes, and navigation **(a)**

The existing config already sets:

- `path: '../doc'`, explicit inclusion list, `routeBasePath: '/'`;
- search `docsDir: '../doc'`, `docsRouteBasePath: '/'`;
- edit links into `main/doc/`;
- one sidebar.

See [docusaurus.config.ts:43](/Users/fab/dev/pv/fugu-mppt-firmware/website/docusaurus.config.ts:43) and [sidebars.ts:4](/Users/fab/dev/pv/fugu-mppt-firmware/website/sidebars.ts:4).

The plan requires `website/docs/`, five sidebars, and `/docs/intro`. Also, current [Introduction.md:3](/Users/fab/dev/pv/fugu-mppt-firmware/doc/Introduction.md:3) has `slug: /`.

Update those settings together. Merely moving/renaming files would leave the scaffold reading the old directory and generating different routes. CLAUDE’s new documentation-site instructions at 132 also need reconciliation.

### 8. Medium — Preserve CommonMark detection and make link migration explicit **(c)**

In-memory compilation of the 37 source documents, with math, GFM, frontmatter, and comment compatibility:

- **Seven failed under MDX syntax.**
- **None failed under CommonMark syntax.**
- This does not prove a complete Docusaurus build.

Examples include `Console.md:124`’s exposed `<ns>`, `Signal Filters.md:81`’s template notation, and `ina226.md:6`’s `<10ms`. Bare `{P, P+1}` at `beacon-sync.md:27` can become JavaScript under MDX. Conversely, Peek’s `<addr>` examples are code-formatted; they are not evidence of a failure.

Keep the existing `markdown.format: 'detect'`. Docusaurus otherwise treats `.md` as MDX by default. [Markdown format documentation](https://docusaurus.io/docs/markdown-features)

Other distinctions:

- HTML comments in LFP’s embedded SVG are **not an observed blocker** with the checked configuration; Docusaurus 3.10 has comment compatibility.
- Spaces are not inherently a compile error. Renaming requires updating links and sidebar IDs.
- Default discovery includes lowercase `.md`/`.mdx`; `.MD` needs normalization if ever included. Here `Test Cases.MD` is excluded, but [Automated Bench Tests.md:14](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Automated Bench Tests.md:14>) still links to it.
- Existing links to excluded specs occur at [Agentic Programming.md:449](</Users/fab/dev/pv/fugu-mppt-firmware/doc/Agentic Programming.md:449>). `Console.md:60` already points to the wrong directory for Real-time Counter.
- Backticked source paths are not hyperlinks. I did not find grounds to claim every `src/*.h` mention breaks the site.

The existing [repo-links.mjs:15](/Users/fab/dev/pv/fugu-mppt-firmware/website/src/remark/repo-links.mjs:15) converts non-page relative links to GitHub URLs **without checking existence**. Thus a typo can become an external broken link that the site’s internal checker no longer catches. It also links readers to excluded material. Replace “rewrite everything else” with explicit reviewed destinations.

`onBrokenLinks` throws by default during production builds; Markdown-link handling is separate. The existing config correctly makes both strict. External GitHub availability still needs separate checking. [Configuration reference](https://docusaurus.io/docs/api/docusaurus-config)

### 9. Medium — Private “further reading” cannot support a public procedure **(c)**

The link at [plan line 83](/Users/fab/dev/pv/fugu-mppt-firmware/plans/docs-site.md:83) returned **“Page not found · GitHub · GitHub”** through Playwright. The brief’s contents remain **unverified**; this establishes public inaccessibility, not absence.

Of the options at lines 116–117:

- “Internal” labeling does not make it useful public further reading.
- A folder cannot independently be made public inside a private GitHub repository; visibility is repository-wide. [GitHub visibility documentation](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/managing-repository-settings/setting-repository-visibility)
- Publish a reviewed generic extract and keep private provenance outside the published page.

### 10. Medium — The public toolchain path assumes the owner’s checkout layout **(c)**

The Getting Started mapping names `idf-export.sh`, but [idf-export.sh:5](/Users/fab/dev/pv/fugu-mppt-firmware/idf-export.sh:5) sources `../../esp/idf5.5/export.sh`, forces ESP32-S3, and performs host-specific serial discovery.

A generic guide needs installation/export instructions using the reader’s IDF location and explicit target/port selection. Describe this wrapper as a convenience requiring adaptation.

### 11. Low — Ownership between sections needs tightening; five sidebars are defensible **(c)**

The issue is duplicated ownership, not sidebar count:

- **Services** is mostly implementation architecture—base classes, lifecycle hooks, registry—despite placement in Reference. Keep names/configuration/commands in Reference; move internals to How it works or Development.
- **Getting Started / Bench Operations / Development Build:** own basic installation once; Bench covers equipment identity/recovery; Development covers layering and build optimization.
- **LFP Charging / Termination:** both describe the same termination mathematics. Keep operational settings in Guide and link to one algorithm explanation.
- **Coil measurement:** own the procedure in Lab, equations in How it works, CLI arguments in Reference.
- **Sync:** the cited material mixes theory with wiring/build/bring-up. Split those reader tasks rather than copying the complete notes under How it works.

The declared DCM theory-versus-measurement split and telemetry setup-versus-field-reference split are already sound. Cross-link them.

## SURVIVED

- **(b)** Search’s declared peer compatibility with React 19 and Docusaurus 3 is valid; optional `open-ask-ai` is not a mandatory missing dependency.
- **(b)** GitHub Pages `url` and `baseUrl` are correct. Existing `organizationName: 'fl4p'`, `projectName: 'fugu-mppt-firmware'`, and `trailingSlash: true` are coherent. Organization/project fields are not required by an Actions artifact deployment, though harmless. [Deployment documentation](https://docusaurus.io/docs/deployment)
- **(b)** Mermaid and KaTeX are already configured correctly: Mermaid theme plus `markdown.mermaid: true`; remark-math 6, rehype-katex 7, and a KaTeX stylesheet. Preserve these during migration. [Math setup](https://docusaurus.io/docs/markdown-features/math-equations)
- Every named Markdown source exists after resolving directory shorthand. Binary-size notes retain dated measurement qualifications and match the current OTA-slot size; I would not label them fabricated or stale merely because they contain historical results.
- No technical defect was found in the owner’s choices of `website/docs/`, kebab-case filenames, no versioning, or GitHub Pages.
- Five sidebars are reasonable for this breadth. The plan needs corrected sources, publication boundaries, and migration mechanics before execution—not a wholesale IA replacement.
