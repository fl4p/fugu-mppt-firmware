---
title: Contributing
sidebar_position: 11
---

# Contributing

Contributions to hardware design, firmware and documentation are welcome. Open an
[issue](https://github.com/fl4p/fugu-mppt-firmware/issues) or a pull request, or get in touch via the maintainer's
GitHub profile to share your experience.

## Before you open a pull request

1. Follow the [coding conventions](conventions.md).
2. Build both targets: `idf.py build` (ESP32-S3) and `idf.py -B build-esp32 build` (classic ESP32, after
   `set-target esp32`). If your change touches feature flags, also run `etc/matrix_build.sh`. See [Build](build.md).
3. Run the tests that cover your change. [Testing](testing.md) describes them, and this table shows which tests to run
   for each kind of change:

| Change | Run |
|---|---|
| Pure logic, parsers, services | Host tests in `test/host-stub/` and `test/host_py/` |
| Control loop, sensors, drivers | On-target Unity suite: `RUN_TESTS=1 idf.py -B build-tests build` |
| Console commands, networking | `python etc/e2e-test/run_e2e.py --cluster console --serial <port>` |
| Control behaviour without hardware | A `CONFIG_FUGU_WITH_VCONV=y` build with `config/lab/vconv_mock` |
| Power stage | A current-limited bench setup, see [Lab overview](../lab/index.md) |

4. If you added, renamed or removed a `.conf` key, update its [reference page](../reference/config/index.md) and
   `etc/config-tool/conf-editor.html`.

## Commit messages

Keep commit messages short and abstract. Say what changed and why, not which functions were called. The following
example shows the difference:

| Instead of | Write |
|---|---|
| `keep retrying that same network via WiFi.reconnect() for wifi.conf::switch_delay seconds (default 30, 0=off)` | `keep retrying that same network for switch_delay seconds` |

An optional area prefix is common, for example `ota_ble: …` or `doc: …`.

## Documentation site

The documentation site is a [Docusaurus](https://docusaurus.io) project in `website/`. Its pages are Markdown files
in `website/docs/`. To preview or build the site, run these commands:

```bash
cd website
npm ci
npm start        # live preview with reload
npm run build    # static build, fails on broken links
```

### Add a page

1. Create a `.md` file in the folder of the sidebar it belongs to (`guide/`, `reference/`, `internals/`, `lab/`,
   `development/`). Docusaurus generates the sidebars from the folder structure. A new sub-folder gets a
   `_category_.json` with `label` and `position`.
2. Start the page with front matter:

   ```yaml
   ---
   title: My page
   sidebar_position: 4
   ---
   ```

3. Link other pages with relative `.md` paths. Relative links to repository files outside `website/docs/` (sources,
   configs, scripts) are rewritten to GitHub URLs. A link to a missing file fails the build.
4. Run `npm run build` before pushing.
