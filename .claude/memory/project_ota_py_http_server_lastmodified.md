---
name: project_ota_py_http_server_lastmodified
description: etc/ota.py Last-Modified KeyError = http.server on :9000 serving the wrong dir
metadata: 
  node_type: memory
  type: project
  originSessionId: 7c3ae65c-d2a4-4d52-a650-a134872a4247
---

Running `etc/ota.py` directly (not via `./ota.sh`) needs a `python3 -m http.server 9000`
already running **from the repo root** — ota.py hardcodes the URL `http://<ip>:9000/build/fugu-firmware.bin`
(ota.py:241) and does `requests.head(url).headers['Last-Modified']` (ota.py:243).

Failure seen 2026-06-08: `err <ip> flat 'last-modified'` then `flat: ❌`, device never downloads
(uptime unchanged). Cause = a **stale** `http.server` on :9000 left over from a prior session,
serving `build-mcpwm/` instead of repo root, so `/build/fugu-firmware.bin` → 404 (no Last-Modified
header → KeyError). The device download would 404 too.

Fix: `lsof -nP -iTCP:9000 -sTCP:LISTEN`, check its cwd (`lsof -a -p <pid> -d cwd`), kill it, then
`python3 -m http.server 9000 --bind 0.0.0.0` from repo root; verify `curl -sI http://<lan-ip>:9000/build/fugu-firmware.bin`
returns 200 + Last-Modified. `./ota.sh` does this automatically; it forwards its argv to ota.py and requires `-n` or `-m` (since 2026-09-29).

Note: flat runs MCPWM. The MCPWM image is the **default `build/`** here because root `sdkconfig`
has `CONFIG_FUGU_WITH_MCPWM=y` (gitignored); `build-mcpwm/` was a stale morning build. Verify the
image driver via `build/config/sdkconfig.h` before OTAing a converter. Dirty builds are refused
non-interactively by the dirty-guard (ota.py:340) — commit + stash WIP for a clean git-describe.
See [[project_mcpwm_validated_on_live_flat]].
