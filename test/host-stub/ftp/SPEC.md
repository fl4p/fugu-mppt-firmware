*this document is an LLM generated placeholder*

# FTP e2e test spec

End-to-end tests for the firmware FTP server
([src/tele/ftp_server.cpp](../../../src/tele/ftp_server.cpp)). The server is
compiled natively (POSIX sockets, POSIX file API, [../esp_log.h](../esp_log.h)
for logging), serves a fresh `mkdtemp()` root on loopback ports and is driven
over real TCP. Built with ASan + UBSan.

## Run

```bash
cmake -S test/host-stub/ftp -B test/host-stub/ftp/build && cmake --build test/host-stub/ftp/build
./test/host-stub/ftp/build/ftp_e2e_test
```

Expected: `ALL 18 FTP TESTS PASSED` (~10 s, most of it in the timeout tests).

## Harness

| Component | Notes |
|---|---|
| Server | `FtpServer` in the parent process, `poll(nowMs)` every 1 ms from a thread (under a mutex) |
| Root | `/tmp/ftp-e2e-XXXXXX`, shared by all tests in order, removed at exit |
| Control port | `12121` (`kCmdPort`) |
| Passive ports | `12122` + session index (`kPasvPort`) |
| Credentials | `USER ftp` / `PASS secret` |
| Timeouts | per test: the parent sets `server.timeouts` before forking the test |
| Isolation | one forked child per test; an assertion fails only that test. 100 ms pause between tests so the server frees the session slots |
| `FtpDial` | one control connection; `readCode(ms)` parses RFC 959 multi-line replies, returns the final code, `-1` on timeout, `-2` once the peer closed; `last` holds the final reply line |
| `pasv()` / `epsv()` | send PASV/EPSV, return the advertised port |
| `put()` / `get()` | PASV + STOR / RETR of a whole payload |

## Tests

| # | Test | Checks |
|---|---|---|
| 1 | `pass_without_user` | `PASS` before `USER` → `503`, no `230` follows, `PWD` is not `257` |
| 2 | `malformed_port` | `PORT 1` → `501`, `NOOP` still `200` |
| 3 | `port_bounce` | `PORT 192,168,1,1,0,21` (not the control peer) → `501` |
| 4 | `cwd_dotdot_clamps` | `CWD /`, `CWD ..` → still `257 "/"` |
| 5 | `upload_download` | PASV + STOR, PASV + RETR, byte-exact round trip (data socket connected before the command) |
| 6 | `large_file_round_trip` | 10 KB pattern through the 1 KB transfer buffer |
| 7 | `dele_then_retr_fails` | `DELE` → `250`, `RETR` of the deleted file → `550` |
| 8 | `mlsd_subdir` | `MLSD /mlsdsub` lists the subdir content, not the cwd |
| 9 | `two_sessions_interleaved` | two logged-in sessions get different passive ports; A's STOR stays open while B runs LIST; then A's RETR and B's STOR interleave; all payloads byte-exact |
| 10 | `over_limit_421` | with 2 sessions logged in, the 3rd and 4th connection get `421` and are closed; both sessions still answer `NOOP` and transfer; after `QUIT` a new session gets in |
| 11 | `epsv` | `EPSV` → `229 (|||port|)`, `NLST /` with the data connection opened after the command; `EPSV ALL` → `200` |
| 12 | `active_port` | `PORT` and `EPRT` to a listener on 127.0.0.1, RETR over the server-initiated connection; `EPRT` protocol 2 → `522`, foreign IP → `501`, port < 1024 → `501` |
| 13 | `data_timeout` | `dataMs` = 500: PASV then LIST without connecting → `425` after ~0.5 s, `NOOP` → `200`; RETR without PASV/PORT → `425`; PORT to a closed port, LIST → `425` |
| 14 | `aborted_upload_keeps_original` | upload `keep.conf`; a second STOR aborted by `ABOR` (with Telnet IP/Synch prefix) → `426` + `226`, file unchanged; a third STOR whose control connection drops mid-transfer → file unchanged |
| 15 | `misc_commands` | `FEAT`/`SYST` before login; `LIST`/`CWD` before login → `530`; wrong user → `530`; `OPTS UTF8 ON`, `TYPE I`/`X`, unknown → `502`, 400-byte line → `500`; `SIZE`, `MDTM`, `MLST`, `RNFR`/`RNTO` (and `RNTO` without `RNFR` → `503`), `MKD`/`CWD`/`CDUP`/`RMD`; `RETR ../../../../etc/hosts` resolves inside the root → `550`; `QUIT` → `221` |
| 16 | `auth_timeout` | `authMs` = 1000: `USER` but no `PASS` → `421`, then closed |
| 17 | `idle_timeout` | `idleMs` = 1000: idle after login → `421`, then closed |
| 18 | `long_cwd_path` | cwd of 101 B + 200 B relative `CWD` exceeds `PATH_LEN` → `500`; `NOOP` → `200` |

Tests 1-8 and 18 were carried over from the SimpleFTPServer fork's
regression suite (PASS bypass, PORT NULL deref, FTP bounce, path buffer
overflow, MLSD path argument).
