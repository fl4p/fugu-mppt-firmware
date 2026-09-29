---
title: Contributing
sidebar_position: 11
---

*this document is an LLM generated placeholder*

# Contributing

Contributions to hardware design, firmware and documentation are welcome. Open an
[issue](https://github.com/fl4p/fugu-mppt-firmware/issues) or a pull request, or get in touch via the maintainer's
GitHub profile to share your experience.

## Before you open a pull request

1. Follow the [coding conventions](conventions.md).
2. Build both targets: `idf.py build` (ESP32-S3) and `idf.py -B build-esp32 build` (classic ESP32, after
   `set-target esp32`). For changes touching feature flags, run `etc/matrix_build.sh`; see [Build](build.md).
3. Run the tests that cover your change, see [Testing](testing.md):

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

Keep them short and abstract: say what changed and why, not which functions were called.

| Instead of | Write |
|---|---|
| `keep retrying that same network via WiFi.reconnect() for wifi.conf::switch_delay seconds (default 30, 0=off)` | `keep retrying that same network for switch_delay seconds` |

An optional area prefix is common, e.g. `ota_ble: …`, `doc: …`.

## Documentation site

The site is [Docusaurus](https://docusaurus.io) in `website/`; pages are Markdown files in `website/docs/`.

```bash
cd website
npm ci
npm start        # live preview with reload
npm run build    # static build, fails on broken links
```

### Add a page

1. Create a `.md` file in the folder of the sidebar it belongs to (`guide/`, `reference/`, `internals/`, `lab/`,
   `development/`). Sidebars are generated from the folder structure; a new sub-folder gets a `_category_.json`
   with `label` and `position`.
2. Start it with front matter:

   ```yaml
   ---
   title: My page
   sidebar_position: 4
   ---
   ```

3. Link other pages with relative `.md` paths. Relative links to repository files outside `website/docs/` (sources,
   configs, scripts) are rewritten to GitHub URLs; a link to a missing file fails the build.
4. Run `npm run build` before pushing.
