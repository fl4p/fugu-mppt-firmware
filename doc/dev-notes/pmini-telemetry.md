# Pmini telemetry

Pmini is the default adaptive codec for binary UDP, BLE NUS telemetry, and leased MQTT
telemetry. Firmware OTA still uses its existing transforms. Existing text,
raw-binary (codec 0), and tamp (codec 1) receivers remain supported when pmini
is disabled. Receivers must support codec 2 before it is enabled.

## Enable

`CONFIG_FUGU_WITH_TELE_PMINI` defaults on when binary telemetry is compiled.
Runtime configuration defaults **on** when compiled in. A missing `tele.conf pmini`
key enables it; an explicit `pmini=0` keeps the legacy raw/tamp wire. Text UDP
telemetry is unchanged. The config editor exposes this default as `pmini=1`.

Havan's relay supports codec 2. Update any other receivers before using the new
default. For UDP, select the binary wire (and clear any explicit opt-out):

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
`teledec.py`, `pmini.py`, `_pmini_mtp2.py` and `PMINI-LICENSE`. A release must include those changes and
update the submodule pointer; checking in only the superproject will not ship
the Python decoder. The browser receiver requires `etc/config-tool/pmini.js` before `teledec.js`.
See [pinned dependency details](pmini-dependency.md).

## Batching and delivery

Pmini mode batches at most 2,048 raw bytes and flushes after **1,000 ms** from
the first queued sample. UDP carries the producer's monotonic enqueue time,
so queue residence counts against the batching deadline. Later samples do not
renew that deadline. A periodic transport tick flushes even without a new
sample. Sparse UDP telemetry continues to flush promptly. Smaller buffers or
packet limits may flush earlier.

The target is two-second telemetry latency, allowing nominally 1,000 ms beyond
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
producer's concurrently changing SymbolTable. After retained raw/tamp fallback, valid table frames refresh the encoder cache
for subsequent batches, including a heartbeat after unknown data. Failed packetization
rolls back that cache. Table definitions are never applied retroactively.

Malformed packets cannot commit
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

## Bench-driven memory and deadline fixes (2026-11-27)

The first hardware run failed with BLE enabled: allocation failures drove internal
heap down to 1,088 bytes and the encoder stayed on tamp after losing its initial
schema. With BLE disabled, pmini worked but clock-aligned sample age reached
2,109 ms. The original 8 KiB / 1.8 s settings are superseded.

Transport batches now reserve 2 KiB and wait at most 1 s. Encoding tries pmini
before allocating tamp's worst-case output, avoids a simultaneous raw candidate,
appends PMA2/MTP2 directly with rollback, and uses views when splitting or removing
the MQTT codec byte. BLE pending output is bounded to 4 KiB. The adapter's 8 KiB
input-format limit remains available for offline replay; live transport batching
uses the smaller limit.

A retained raw/tamp packet now teaches the encoder its validated table frames,
even when unknown data precedes the table. Only later packets may use that schema.
Recursive attempts and complete pack failures cannot advance the committed cache.
The captured failing BLE-on stream now replays exactly: three tamp packets followed
by 37 pmini packets, instead of 40 tamp packets.

Guard review:

1. Missing or malformed table definitions fail explicitly. Unknown data uses raw/tamp;
   table observation validates table frames and framing, not opaque data bodies.
2. Oversize batches and invalid frame lengths stay rejected at the far tail. Later
   samples cannot refresh the first sample's deadline.
3. The table observer runs only for a retained fallback; state is committed only
   after the complete output-vector append succeeds.
4. The native replay records input-source fingerprints and exercises the production
   encoder, adapter and packet wrapper, not a parallel implementation.
5. Failed pack attempts leave both caller output and committed schema unchanged;
   partial direct-append failures restore the original prefix.
6. Raw/tamp fallback is reported as such. Smaller batches reduce saved-corpus
   savings to about 33%; the earlier 42–44% figures used larger batches.
7. Regressions cover unknown data before a heartbeat, rejected packetization,
   malformed tables, direct-append failure and a producer-independent deadline.
8. This reduces actual allocations and waiting time. Diagnostic timing is logged
   once when the UDP telemetry service stops; it does not replace the fix.

Host checks: ASAN/UBSAN suite, 100,000 malformed adapter inputs, 5 Python tests,
76 JavaScript tests, exact 4,096-point corpus replay and captured failure replay.
A 1,984-byte synthetic packet with 14 rows and 24 fields used 10,925 bytes of
incremental peak heap including output and encoder workspace in a host allocator
probe; 100 encodes averaged 161 us. These host numbers are diagnostic, not ESP32
qualification. The hardware validation record below is the device evidence.

Hardware rerun: `bench139c` (ESP32-S3, 4 MB flash, no PSRAM, virtual converter),
BLE service enabled throughout, relay 0.17 on havan. The 180 s passive capture
contains 212 packets / 8,194 decoded points, zero decoder errors, 208 pmini packets
and 4 tamp packets. Median sample age was 605 ms, p99 1,042 ms, maximum 1,163 ms;
no captured sample exceeded 2 s. USB clock alignment was acknowledged in 29 ms.
Pmini payloads were 42.9% smaller than tamp recompression of those same payloads;
maximum packet size was 1,041 bytes. These are sample-to-receiver-host ages, not
InfluxDB commit latency or a guarantee for every network condition.

Two service restarts recovered normally. Worst complete adaptive encode time was
183,846 us on ESP32 (includes the pmini/tamp comparison), measured before sending.
Internal heap minimum was 16,492 bytes during the capture and 16,076 at final stop;
430 device log lines contained no allocation failures, panic, encoding failures
or UDP-send errors. The bench was restored to telemetry disabled / binary disabled,
pmini unset, BLE enabled. No production converter was changed. This verifies UDP
with BLE resident, not simultaneous BLE telemetry streaming or MQTT on hardware.
Aggregate results and firmware/source hashes are in
`doc/research/pmini-bench-2026-11-27/`; raw captures and diagnostics remain local.
The tested pre-commit image's sources are fingerprinted there; its `-dirty` suffix
reflects these fixes before commit.
