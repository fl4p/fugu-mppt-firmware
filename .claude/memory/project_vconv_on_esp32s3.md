---
name: vconv build on ESP32-S3 (sdkconfig gotcha)
description: How to build CONFIG_FUGU_WITH_VCONV on ESP32-S3; the non-obvious -DSDKCONFIG trick needed because Kconfig depends-on makes MCPWM=y un-overridable via defaults fragments
created: 2026-08-12T18:36:39.860Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_008bdc8caffeVdGrLk68Apwgsw
---

Building vconv (virtual converter, `CONFIG_FUGU_WITH_VCONV=y`) on **ESP32-S3** requires a separate build dir and a non-obvious SDKCONFIG trick.

**The gotcha:** `sdkconfig.defaults` has `CONFIG_FUGU_WITH_MCPWM=y`. VCONV's Kconfig has `depends on !FUGU_WITH_MCPWM`, so VCONV is invisible while MCPWM is on. You **cannot** override MCPWM to `n` via a later sdkconfig defaults fragment — Kconfig treats `CONFIG_FUGU_WITH_MCPWM=n` as "not set" and falls back to the `=y` from `sdkconfig.defaults`. `# CONFIG_FUGU_WITH_MCPWM is not set` also doesn't work. The root `sdkconfig` file (from the main build) also has `CONFIG_FUGU_WITH_MCPWM=y`, and the top-level CMakeLists.txt's `fugu_kconfig_bool` reads it.

**The fix:** Pass `-DSDKCONFIG=build-vconv/sdkconfig` (pointing to a path in the build dir that doesn't exist yet) so IDF creates a fresh sdkconfig in the build dir instead of reusing the root one. Combined with `-DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.vconv_s3"` where `sdkconfig.vconv_s3` has `# CONFIG_FUGU_WITH_MCPWM is not set` + `CONFIG_FUGU_WITH_VCONV=y`.

**Full build command:**
```bash
idf.py -B build-vconv -DSDKCONFIG=build-vconv/sdkconfig \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.vconv_s3" build
```

**Config:** `config/lab/vconv_mock/` is the S3 vconv board config (mcu=esp32s3). To flash it, temporarily point `FUGU_LITTLEFS_SRC` in top-level `CMakeLists.txt` to `config/lab/vconv_mock`, or provision separately with `./provision.py vconv_mock`.

**PSU mode testing:** `config/lab/vconv_mock/conf/converter.conf` now has `mode=psu` + `psu_vout=28` for PSU mode testing. The vconv plant simulates a buck converter with PV input (Isc=13, Voc=76) and battery load (27V, 0.05Ω). `vconv bat <v>` / `vconv bat short` / `vconv bat open` can simulate load steps and fault conditions for PSU mode validation.

See also [[project_vconv_on_esp32_classic]] for the classic ESP32 variant.
