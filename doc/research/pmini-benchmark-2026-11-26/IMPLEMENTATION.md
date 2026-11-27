# Native telemetry implementation — 2026-11-27

Implemented in the working tree by the coordinator with gpt-5.6-luna implementation
agents. No commits, live configuration changes or flashing were performed.
Runtime activation and packet format: `doc/dev-notes/pmini-telemetry.md`.

The actual native adaptive packet path (including PMA2 metadata, preserved symbol
tables, CRC, size splitting and fallback) saves 44.3% on fry and 41.9% on flat
against tamp at the same 1.8-second batching, on the saved 4,096-point corpus.
51 PMA2 packets reconstruct the exact original wire bytes in Python and JS;
two additional packets select tamp. Largest payloads are 1,376/1,362 bytes.
These results supersede the earlier metadata-free 54–57% estimates for the
implemented format. See `native-results.json` for source fingerprints and timings.
The database excerpts remain short active-stream samples and may contain floats
rounded by the original text telemetry path.

## Transport guard checklist

1. **Unevaluable input:** empty, malformed, noncanonical or overflowing frame
   lengths fail the packet wrapper. Unsupported pmini schemas retain a raw/tamp
   candidate. Failure to create any bounded packet is reported as a drop.
2. **Monotonicity:** frames beyond the cap cannot enter the batch; a packet over
   its limit is split only at complete frame boundaries. A single frame that
   cannot fit fails. Tests cover 8,192-byte input limits, tiny output limits,
   long varint tails, and partial destination preservation.
3. **Preconditions:** actual Fugu C++ frames, native encoder, Python decoder and
   JS decoder are exercised together; legacy raw/tamp behavior remains tested.
   Firmware compilation uses the ESP32-S3 toolchain, not merely host headers.
4. **Source of truth:** replay records hashes of actual native source files and
   uses saved original query results. No timing or size result is taken from a
   stale build cache or substituted for on-board measurement.
5. **Persistence:** a private dtype-map copy spans the entire adaptive pack;
   failed output assembly does not commit it. Receiver table updates are atomic.
   Transport restarts clear pending data and reset encoder state.
6. **Provenance:** host runtime/allocation measurements are explicitly host-only.
   Deadline tests cover first-sample aging and no renewed deadline on append;
   network delivery and actual ESP32 runtime are not declared verified.
7. **Known-bad calibration:** tests deliberately exercise malformed/truncated
   frames, CRC-valid invalid metadata, oversize packets, a failed pack followed
   by data without definitions, deadline far tails, and reconnect/restart paths.
8. **Fix versus mute:** pending output is bounded and oversized packets are
   actually split. Allocation and send failures are counted/logged. The encoder
   changes transmitted bytes; the deadline changes when batches are flushed.

Measured wrapper cost on the host is approximately 0.30–0.35 ms median per
1.8-second batch, including native tamp and pmini candidates, with a recorded
maximum below 1.3 ms in the final replay. This is not an ESP32 time budget.
The native codec and adapter allocation/guard probes are in `NATIVE-CODEC.md`
and `PACKET-REVIEW.md`; neither is a complete on-board heap peak measurement.

## Build limitation

An isolated full ESP-IDF build was attempted in `/tmp/fugu-pmini-idf`.
The existing bootguard dependency required RTC-reservation settings absent from
the starting sdkconfig; these were added to the temporary configuration only.
The build subsequently stopped in unrelated, pre-existing working-tree edits to
`src/cli.cpp`: `BatChargerParams` lacks `awaitingDetect` (lines 1945/2225) and
`vbatAuto` (line 2225). Those edits were preserved. The touched telemetry sources
and all three new native implementation units cross-compile successfully with
ESP32-S3 flags, including pmini, BLE telemetry and MQTT telemetry enabled.
A complete firmware link and on-board RAM/latency qualification remain open.

Python changes are uncommitted inside the `etc/fugu` submodule; a release must
include the nested commit and superproject pointer, alongside the superproject
implementation/tests. Unrelated existing changes were not reverted.

## Final check results

- ASAN/UBSAN native codec, adapter, bounded packet/deadline and MQTT integration
  runner passed. Adapter's separate deterministic malformed-input sweep passed
  100,000 cases.
- Python PMA2 tests: 5 passed. JavaScript PMA2 plus legacy telemetry suites:
  76 passed. The final PMA2-only suite also passed after the packet-size guard.
- Native replay: all 4,096 input points reconstructed exactly; all 51 PMA2
  packets also reconstructed exactly in JavaScript. Final native source hashes
  match `native-results.json`.
- ESP32-S3 cross-compile: pmini_codec, pmini_packet, tele_packet, tele_core,
  telemetry, tele_ble and tele_mqtt all passed. Compiler stack-usage reports were
  inspected; the largest individual new-code frame was 960 bytes. This is not
  a measured worst-case task stack high-water mark.
- Configuration editor's scraped key/default tables are in sync, including
  `tele.conf pmini`. Whitespace checks pass for the changed implementation.
- Independent final review found no remaining confirmed wire-format defect;
  its unverified target/delivery limits are retained in `FINAL-REVIEW.md`.

## Main integration — 2026-11-27

Integrated onto firmware main `ec5c73d1` and Python receiver main `7f30b47` in
isolated worktrees, preserving unrelated local changes. The receiver is committed
and pinned by the superproject. Main's UDP socket helper replaces the older
AsyncUDP call in the pmini send path; its return value retains drop accounting.
The host runner supports main's legacy feature macros as well as Kconfig names.
Fresh integration checks: ASAN/UBSAN host suite, 5 Python tests, 76 JavaScript
tests, and exact native replay of 4,096 points passed. Runtime opt-in remains off.
No OTA or device configuration changes were performed as part of integration.
