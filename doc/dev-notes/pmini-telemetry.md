# Pmini telemetry

Pmini is an opt-in codec for binary UDP, BLE NUS telemetry, and leased MQTT
telemetry. Firmware OTA still uses its existing transforms. Existing text,
raw-binary (codec 0), and tamp (codec 1) receivers remain supported when pmini
is disabled. Receivers must support codec 2 before it is enabled.

## Enable

`CONFIG_FUGU_WITH_TELE_PMINI` defaults on when binary telemetry is compiled.
Runtime configuration defaults **off** (`tele.conf pmini=0`). In the config
editor, `tele.conf` now exposes the `pmini` boolean.

For UDP, after updating the Python receiver:

```text
set-config tele.conf binary 1
set-config tele.conf pmini 1
svc rs tele
```

For an already-running BLE or MQTT sink, stop and restart that sink after
changing the setting (`tele-ble 0` then `tele-ble 1`, or `tele-mqtt 0` then a
new lease). Each sink snapshots its mode at start. Disable with `pmini=0` and
restart the relevant sinks. No device configuration is changed by building.

The Python receiver changes live inside the `etc/fugu` git submodule:
`teledec.py` and its new `pmini.py`. A release must include those changes and
update the submodule pointer; checking in only the superproject will not ship
the Python decoder. The browser receiver is `etc/config-tool/teledec.js`.

## Batching and delivery

Pmini mode batches at most 8,192 raw bytes and flushes after **1,800 ms** from
the first queued sample. UDP carries the producer's monotonic enqueue time,
so queue residence counts against the batching deadline. Later samples do not
renew that deadline. A periodic transport tick flushes even without a new
sample. Sparse UDP telemetry continues to flush promptly. Smaller buffers or
packet limits may flush earlier.

The target is two-second telemetry latency, allowing nominally 200 ms beyond
batching for encoding, scheduling and transport. This is not a hard network
or real-time delivery guarantee: ESP32 encoding time, peak heap and worst-case
radio/scheduler delay still need hardware qualification. BLE congestion,
disconnection and MQTT/UDP publish failure can lose samples. Failures are
reported through existing sink drop counters or UDP warnings.

Every emitted codec payload is at most 1,440 bytes (UDP uses its configured
TCP_MSS). Oversized output is split at complete wire-frame boundaries. The
encoder compares pmini with tamp and raw bytes and uses the smallest candidate.
Mixed measurement/tag sets, unsupported F64 fields, incompatible field order,
unknown schema, or pmini allocation failure retain the raw/tamp candidate.
Malformed input, an individually unpacketizable frame, or failure to allocate
any bounded output is dropped explicitly. Input and pending output are bounded;
there is no unbounded retry queue.

## Format and recovery

Codec 2 contains PMA2 version 1, an envelope around an MTP2 pmini shard:

1. `PMA2`, byte version `1`, canonical varint event count.
2. Common Data prefix length and bytes, including the Data frame type,
   measurement SID, tag SIDs, and tag-list terminator.
3. Union field count, followed by field SID and WireDT pairs.
4. Original frame-order descriptors: `0` plus length and exact length-prefixed
   Table frame bytes, or `1` for the next data row.
5. MTP2 shard (no additional length prefix), then little-endian CRC-32 of all
   preceding envelope bytes. The MTP2 shard also carries its own CRC.

Rows contain timestamp low/high u32 words, a presence mask, and one raw word
per union field. Absent values have zero words and cleared presence bits;
widths and float bit patterns are preserved. Per-row field order constraints
build a stable union order; contradictory constraints use legacy fallback.
The adapter bounds rows to 128, union fields to 32 and SIDs to 4,096.

Each compressed shard has independent entropy state. Tables remain at their
original positions after reconstruction. Symbol names retain the existing
stream-table heartbeat/recovery semantics: loss does not poison subsequent
compression state, but a lost new-symbol definition may temporarily prevent
rendering until its table resend. This is not a new reliable schema-negotiation
protocol. Each transport owns a private dtype cache and never reads the
producer's concurrently changing SymbolTable. Malformed packets cannot commit
partial schema changes to either host decoder.

## Validation

Run from the firmware root:

```sh
SANITIZE=1 test/host-stub/pmini-tests.sh
.venv/bin/python -m pytest -q test/host_py/test_teledec_pmini.py
node --test etc/config-tool/test/teledec_pmini.test.js
.venv/bin/python test/host_py/pmini_native_replay.py
```

The replay uses saved unaggregated database excerpts from fry and flat. It
compiles the actual native wire encoder, codec and adaptive packet wrapper,
checks exact decoded bytes, records packet limits and selected codecs, and
writes `doc/research/pmini-benchmark-2026-11-26/native-results.json`.
Database float rounding remains a corpus limitation; this is not captured
radio traffic or an ESP32 execution benchmark. See the adjacent native codec,
packet, and final review notes for guard checks and measured host costs.
