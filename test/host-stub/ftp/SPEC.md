*this document is an LLM generated placeholder*

# FTP e2e test spec

End-to-end regression tests for the vendored
[SimpleFTPServer](../../../components/SimpleFTPServer/SimpleFTPServer)
fork. The lib is compiled as a plain host binary against
[arduino-host-shim](../arduino-shim), bound to `127.0.0.1` on hardcoded
ports, and driven over real TCP from the same process.

## Run

```bash
cmake -S test/host-stub/ftp -B build && cmake --build build
./build/ftp_e2e_test
```

Expected: `ALL FTP REGRESSION TESTS PASSED` (9 tests, ~2 s).

## Harness

| Component | Notes |
|---|---|
| Control port | `12121` (`kCmdPort`) |
| PASV port | `12122` (`kPasvPort`, the lib's `pasvPort` member) |
| Credentials | `USER ftp` / `PASS secret` |
| Test isolation | One forked subprocess per test (assertion or SIGSEGV in one test does not affect later ones — but the server thread lives in the parent, so a bug that crashes the server takes out subsequent tests too) |
| `FtpDial` | One TCP connection per test; sends commands with `\r\n`; `readCode()` parses RFC 959 multi-line replies (`ddd-...` lines until `ddd `) and returns the final 3-digit code |
| `pasv()` | Sends PASV, parses `227 ... (h1,h2,h3,h4,p1,p2)`, returns the port |
| `openData()` | Opens a fresh TCP socket to the PASV port for the data channel |
| `drainData()` | Reads until peer closes, returns everything |

Each `FtpDial` constructor sets `SO_RCVTIMEO=2s` so a missing reply surfaces
as `readCode()==-1` instead of hanging.

## Tests

### 1. `pass_without_user` — PASS-before-USER bypass

Covers fl4p/SimpleFTPServer@d134ceb security fix #1. The buggy code printed
`503 ` for PASS-without-USER **and** fell through to authenticate if the
password matched (missing `else`), queuing both `503 ` and `230 Ok` on the
wire.

```
← 220 banner
→ PASS secret
← 503                  (first reply; assert == 503)
← !=230                (second reply; on buggy code: 230 ⇒ FAIL)
→ PWD
← !=257                (PWD must not succeed)
```

The killer assertion is `second != 230`: the only way the server queues a
230 after rejecting PASS is if the bypass fall-through fired.

### 2. `malformed_port` — PORT NULL deref

Security fix #3. The buggy parser called `atoi(++p)` without checking that
the prior `strchr(p, ',')` returned non-NULL, so `PORT 1` SIGSEGV'd the
server.

```
→ USER ftp / PASS secret              ← 331 / 230
→ PORT 1
← 501                                 (patched; buggy: server crashes)
→ NOOP
← 200                                 (proves parser didn't smash state)
```

### 3. `port_bounce` — FTP-bounce / SSRF

Security fix #4. Original code accepted any `h1,h2,h3,h4` in PORT, letting
an authenticated client point the server at an arbitrary host. Patched
version requires PORT IP to match `client.remoteIP()`.

```
→ USER ftp / PASS secret              ← 331 / 230
→ PORT 192,168,1,1,0,21               (control peer is 127.0.0.1, not this)
← 501                                 (refused)
```

### 4. `cwd_dotdot_clamps` — `..` from root stays at root

The lib's `makePath` walks leading `../` segments up `cwdName` but clamps
at `/`. Verifies that a deliberate over-walk doesn't escape the root via
the command interface. (Embedded `..`, e.g. `RETR /a/../b`, is not
normalized by the lib and falls through to the FS layer, where behavior
depends on the underlying filesystem — out of scope here.)

```
→ USER ftp / PASS secret              ← 331 / 230
→ CWD /                               ← 250
→ PWD                                 ← 257
→ CWD ..                              ← 250 (succeeds as no-op at root)
→ PWD                                 ← 257                (still at root)
```

### 5. `upload_download` — PASV + STOR + RETR round-trip

Functional coverage of the full passive-mode data path. No security bug
attached; this is the smoke test for everything PASV/data-channel.

```
→ USER ftp / PASS secret              ← 331 / 230
→ PASV
← 227 Entering Passive Mode (127,0,0,1,47,90)    # 47*256+90 = 12122
open data socket to 127.0.0.1:12122

→ STOR /upload.bin
← 150
data → server: "hello fugu ftp host-shim\n"  (25 B)
close data socket
← 226

→ PASV
← 227 ...
open data socket
→ RETR /upload.bin
← 150
data ← server: 25 B
assert: bytes match payload exactly
← 226
```

Asserts byte-exact equality of upload vs. download payload (`got == payload`).

### 6. `large_file_round_trip` — multi-chunk STOR + RETR

Functional smoke test for the data-transfer loops (`doFiletoNetwork` /
`doNetworkToFile`). Uses a 10 KB deterministic pattern (byte `i % 251`)
so any off-by-one corruption mid-buffer would land on a different value.
The transfer crosses the 1436-byte `BUFFERSIZE` (`CONFIG_TCP_MSS` on
ESP32) several times, forcing multi-iteration chunking.

```
→ ... login + PASV ...
→ STOR /large.bin                     ← 150
data → server: 10240 B (drained-as-written to avoid kernel send-buffer stall)
                                      ← 226
→ ... PASV ...
→ RETR /large.bin                     ← 150
data ← server: 10240 B
assert: size match AND byte-exact match
                                      ← 226
```

### 7. `dele_then_retr_fails` — DELE removes a file and RETR confirms it's gone

Mutation round-trip + negative read. Also documents the lib's reply codes
(DELE=250, RETR-nonexistent=550).

```
→ ... login + PASV ...
→ STOR /todelete.bin                  ← 150 / 226           (set up victim)
→ DELE /todelete.bin                  ← 250 "Deleted ..."
→ RETR /todelete.bin                  ← 550 "<path> not found."
```

### 8. `mlsd_subdir` — MLSD honors optional pathname arg

Covers fl4p/SimpleFTPServer@488d4d4 ("Honor optional pathname argument in
LIST/NLST/MLSD"). Pre-fix, the server silently re-listed `cwdName` even
when the client sent `MLSD /sub`.

```
→ USER ftp / PASS secret              ← 331 / 230
→ MKD /mlsdsub                        ← 257 or 550 (idempotent)
→ PASV + STOR /mlsdsub/marker.txt     (1-byte payload)   ← 150 / 226

→ PASV
open data socket
→ MLSD /mlsdsub
← 150
data ← server: directory listing of /mlsdsub
← 226

assert: listing contains "marker.txt"
assert: listing does NOT contain "mlsdsub"     (cwd is /, so a fallback
                                                listing would show the
                                                subdir entry itself)
```

Verified to catch the regression: stripping the 23-line cwd-swap block
from `processCommand` reverts to listing the cwd; the listing then shows
the leftover `BBBB…` directory from `long_cwd_path` and is missing
`marker.txt`, so the assertion fires.

### 9. `long_cwd_path` — `makePath` stack overflow (runs last)

Security fix #2. The buggy `makePath` used `strncat(dst, src, FTP_CWD_SIZE)`
where the size arg is "max bytes from src", not buffer size, so a long
`workingDir + "/" + param` could overflow the 263-byte `fullName` stack
buffer.

```
→ USER ftp / PASS secret              ← 331 / 230
→ MKD /BBBB…(100 B's)                 ← 257 or 550
→ CWD /BBBB…(100 B's)                 ← 250                (workingDir = 101 B)
→ CWD AAAA…(200 A's, relative)        ← 500                (101 + 1 + 200 > 263 B)
→ NOOP
← 200                                 (server must still be alive)
```

The relative arg is sized to slip past the 263-B per-line cmd buffer
(`FTP_CMD_SIZE`) but blow the 263-B path buffer when concatenated with
the long workingDir.

**Ordered last:** on buggy code, the stack smash propagates from the
server thread into the parent's address space and kills the entire test
binary (`exit=139`). Running this test last means the earlier regressions
still produce per-test PASS/FAIL output on a buggy build.

## Verification matrix

| Test | Patched | At pre-fix commit | Detection mode |
|---|---|---|---|
| pass_without_user | PASS | FAIL (assert) | `second==230` after PASS proves bypass |
| malformed_port | PASS | FAIL (SIGSEGV in parent) | server crashes |
| port_bounce | PASS | FAIL (assert) | server accepts non-peer IP |
| cwd_dotdot_clamps | PASS | (no known regression — guard test) | PWD after `CWD ..` must be 257 |
| upload_download | PASS | (no upstream bug — smoke test) | byte mismatch ⇒ assert |
| large_file_round_trip | PASS | (no upstream bug — smoke test) | size or byte mismatch ⇒ assert |
| dele_then_retr_fails | PASS | (no upstream bug — smoke test) | DELE≠250 or RETR-deleted≠550 |
| mlsd_subdir | PASS | FAIL (assert) | listing missing `marker.txt`, contains cwd entries |
| long_cwd_path | PASS | FAIL (NOOP timeout / SIGABRT) | server dead after long CWD |
