# Native pmini codec guard notes

Status: host-validated prototype; not an ESP32 memory or latency qualification and
not a hardware/transport closure.

## Guard checklist

1. **Unevaluable input.** `pminiEncode` rejects null input, zero dimensions,
   dimensions above 128 rows or 64 columns, overflowed dimensions, and allocation
   or encoding failures. It returns `false` and leaves the caller's output string
   unchanged. The packet adapter rejects malformed frames, empty batches, mixed
   schemas, unsupported F64 fields, and rows beyond its bound; its fallback caller
   counts failed bytes rather than treating them as verified output.

2. **Monotonicity.** Increasing rows or columns beyond the native bounds remains
   a rejection. The packet-size splitter recursively splits only at complete frame
   boundaries; a one-frame packet that still exceeds the limit fails rather than
   emitting an oversized result.

3. **Preconditions.** The native encoder consumes row-major 32-bit field words;
   schema/type conversion is external. The adapter validates MTP frame prefixes,
   table SIDs/types, field widths, timestamp termination, field uniqueness, and
   per-row field ordering before invoking the codec. Build-time integration and
   the live ESP32 call path still require target validation.

4. **Source of truth.** Transform selection, bucket tables, static rANS
   normalization, MTP2 trailers, and CRC are ported from
   `/Users/fab/dev/esp/pmini/pmini/pmini.py` and `pmini_py.py`; the native test
   includes a mixed-transform golden shard. The adapter has its own PMA2 schema
   and frame parser, which is not the Python pmini shard schema.

5. **Persistence.** Failed native encoding does not append output. Failed packet
   encoding leaves the destination vector unchanged; transport queues clear or
   count dropped bytes on failure. The PMA2 UDP path checks the `writeTo` return
   count and logs the exact dropped-byte count; a failed datagram is dropped,
   not reported as delivered. This is delivery accounting, not retry or
   end-to-end receipt verification.

6. **Provenance.** The native measurements below are host diagnostics only. No
   value here means that an ESP32 allocation, deadline, UDP delivery, or BLE
   notification was verified.

7. **Known-bad calibration.** The dedicated host test covers null input, the
   129-row bound, atomic rollback, an empty shard, and a mixed-transform golden
   vector. AddressSanitizer/UndefinedBehaviorSanitizer passed 200 deterministic
   random shards. Existing Python `pmini.decode_words` decoded all generated
   shards and every <=128-row batch from both saved `flat.json` and `fry.json`
   corpus snapshots byte-exactly at the word level.

8. **Fix vs mute.** Codec failure paths return false and are counted/dropped by
   callers; they do not merely suppress a warning. UDP write failures are
   explicitly counted/logged rather than muted as successful sends.

## Host measurements

On the ARM64 development Mac, a 128-row by 24-column random-word encode averaged
298.5 us over 1,000 iterations and produced 12,666 bytes. After dimension-scaled
reserves and direct output (no temporary whole-shard duplicate), an instrumented
100-row run from each saved typed corpus peaked at 16,569 live bytes for `flat`
(23 words/row) and 18,754 bytes for `fry` (27 words/row), including the input
vectors and output strings. The increases above the pre-call input baselines
were about 7.4 KiB and 8.0 KiB respectively. This allocator probe is diagnostic
only: ESP-IDF allocators, fragmentation, PSRAM placement, compiler, and target
timing differ.
