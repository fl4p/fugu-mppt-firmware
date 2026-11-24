---
name: project_own_ftp_http_server_1123
description: Own FTP server + HTTP conf endpoint (8c0f7e02) HW-tested on bench139c 2026-11-23; ~20 KB heap peak with 2 busy sessions
metadata:
  node_type: memory
  type: project
  originSessionId: 72b769f1-4382-4d67-bf42-f4e0a47f6104
  modified: 2026-11-23T09:22:33.957Z
---

8c0f7e02 replaced SimpleFTPServer (src/tele/ftp_server.*) and added CONFIG_FUGU_WITH_HTTP_CONF (default off). 2026-11-23 on bench139c via havan: 20/20 ftplib checks + curl pass (2 sessions, 421, EPSV, PORT, RST/ABOR keep original). Heap min-ever fell ~52→34 KB during the tests (lwIP TCP buffers dominate).

flat (running 823-g13023313 at ~545 W, 11-23): 2 sessions + 1 KB STOR cost ~4 KB heap (min-ever 17.1→13.1 KB, largest block 12 KB), lag 0.85→1.65 ms, no ADC errors. Reached via nimble-ble-proxy portmaps 192.168.1.231:2144→flat:21, 50009/50010 passthrough (client must ignore the 192.168.4.x PASV IP). Bench under vconv RT load: 3×20/20, no errors.

fry (840-ga082476f, 11-23): ftp.conf pasv_port=50011, NAT 2142->fry:21 + 50011/50012; PASV/EPSV announce 50011, 2-session test OK, ~3 KB heap. Creds on both boards: ftp.conf (user `usr`), not NVS.

**Why:** fry/flat run with heap min ~8 KB, so a busy FTP session there could exhaust heap.
**How to apply:** before relying on FTP on fry/flat, check `heap` min-ever under an FTP transfer. Test-harness traps: ftplib `sock.close()` doesn't close (makefile ref), `abort()` reads only the 426 of 426+226. Fallback password = `esp32s3-<efuse MAC as LE uint64 hex>`.
