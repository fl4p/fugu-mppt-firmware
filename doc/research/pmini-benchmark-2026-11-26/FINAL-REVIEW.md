# Final review: native pmini telemetry

Date: 2026-11-27

## Result

The reviewed host implementation is internally consistent for the exercised
PMA2/MTP2 paths. I found no remaining confirmed firmware wire-format defect in
the final read. The review is not a device qualification: the ESP32 build is
blocked by unrelated dirty `cli.cpp`/`BatChargerParams` state, and no firmware
timing, heap, BLE notification, MQTT delivery, or UDP delivery was measured on
hardware.

## Findings resolved before this review

- `TelePacketEncoder::pack()` now evaluates recursive candidates using a staged
  `PminiPacket` and commits its SID cache only after the complete packet list is
  accepted (`src/tele/tele_packet.cpp:25-50`). This prevents a failed size/OOM
  attempt from teaching the cache a table that was never delivered.
- `TelePacketBatch::fits()` explicitly rejects a point larger than the whole
  8192-byte batch (`src/tele/tele_packet.h:23-34`).
- Python and JavaScript PMA2 decoders inspect the MTP2 header/CRC and compare
  shard shape with the descriptor count before materializing rows
  (`etc/fugu/teledec.py:162-173`, `etc/config-tool/teledec.js:212-216`). This
  enforces the PMA2 128-row bound before the generic pmini 4096-row decoder can
  allocate work buffers.
- Native packet parsing validates complete table frames, SID datatypes, field
  widths/uniqueness, timestamps, schema ordering, and bounded dimensions before
  handing words to the codec (`src/tele/pmini_packet.cpp:114-324`). The adapter
  keeps its SID state private and only swaps it after PMA2 construction.

## Validation performed

- `./test/host-stub/pmini-tests.sh`: native codec golden/rollback tests,
  adapter tests, packet-size splitting, rejected-pack cache regression, and
  MQTT sink tests passed.
- `node --test etc/config-tool/test/teledec_pmini.test.js`: 4/4 passed after
  the JavaScript shape preflight.
- The parent run reports the native replay corpus round-tripping all saved
  4096-point samples and the latest sanitizer/fuzz run passing. Those are host
  results, not target results.

## Remaining limits

- The JavaScript decoder's generic line rendering accumulates wire timestamps as
  `Number`; timestamps above `2**53-1` are not display-exact
  (`etc/config-tool/teledec.js:48-65`). PMA2 reconstruction itself is exact:
  it combines the decoded low/high words with `BigInt` before re-encoding the
  timestamp. Current epoch-millisecond timestamps are below the display limit;
  this is a pre-existing generic-decoder limitation, not a new PMA2 wire defect.
- Transport delivery remains best-effort. UDP short writes are counted/logged,
  while BLE/MQTT retry/reconnect behavior and the 1800 ms batching target were
  only host-tested (`src/tele/telemetry.cpp:365-401`,
  `src/tele/tele_ble.cpp:183-211`, `src/tele/tele_mqtt.cpp:126-149`).
- Full firmware compilation is blocked by unrelated dirty workspace changes;
  real-device heap/latency/packet-loss testing was not performed.
