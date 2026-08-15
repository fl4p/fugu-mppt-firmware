---
name: conf-editor-scraper-clobbers-file-keys
description: etc/config-tool/scrape_conf_keys.py --write DROPS hand-maintained FILE_KEYS entries — hand-edit conf-editor.html instead
metadata: 
  node_type: memory
  type: project
  originSessionId: 46c7e808-20eb-4e70-8a46-f280e1c16d5a
  modified: 2026-08-15T06:27:16.233Z
---

`etc/config-tool/scrape_conf_keys.py --write` regenerates the FILE_KEYS block in
`etc/config-tool/conf-editor.html` from what it can scrape out of the code, and that misses
keys read through helpers or in files it doesn't scan: it dropped all `ctrl_*_kp/kd/td`
(built dynamically by `pdLoadGains`), the whole `bsync.conf` entry, `ble.conf`'s
`ble_security`/`ble_passkey`, and `tele.conf`'s `adv_ms`/`binary`/`ble` (observed
2026-08-15).

**Why:** the checked-in FILE_KEYS is effectively hand-maintained beyond what the scraper
reproduces; running `--write` silently regresses it.

**How to apply:** when adding conf keys, edit META and FILE_KEYS in conf-editor.html by
hand (keys sorted, one file per entry) and do NOT run the scraper --write; if you do run
it, diff and restore the dropped entries before committing.
