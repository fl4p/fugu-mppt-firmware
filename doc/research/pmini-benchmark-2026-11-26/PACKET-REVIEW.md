# Native pmini packet adapter review

Review scope: `src/tele/pmini_packet.{h,cpp}` and an independent read-through of
`src/tele/pmini_codec.cpp` against the canonical Python implementation in
`/Users/fab/dev/esp/pmini/pmini/pmini.py` and `pmini_py.py`. No native codec source
was changed during this review.

## Frozen packet format

`PminiPacket::packet()` appends a codec-2 payload only; the transport supplies
the one-byte codec ID. The payload is:

```
PMA2                         4-byte magic
version                      u8 = 1
event_count                  canonical unsigned varint
common_prefix_len, prefix    varint + bytes (Data byte, measurement, tags, tag terminator)
union_count                  canonical varint, 1..32
union entries                SID varint, WireDT u8, first in the stable union order
event descriptors            0 + raw table wire record, or 1 for one Data row
MTP2 shard                   native pmini bytes, unchanged
crc32                        u32le over every preceding PMA2 byte
```

The raw table record includes its original length prefix and frame bytes. Data
rows are encoded as MTP2 words `[timestamp_lo, timestamp_hi, presence, values]`.
The union order is a stable topological order of every row's field-order edges;
an order cycle falls back to tamp. Thus omitted fields retain their absence and
field order is reconstructed without row SID lists. f64 is deliberately refused
because the native MTP2 interface carries one u32 word per column.

## Native codec comparison

The C++ codec matches the Python constants and transform set: MTP2 header,
u32 row/column words, CONST/RAW/D1/DICT/SPLIT/SAME plans, 12-bit rANS frequency
normalisation, LSB-first bit packing, reverse stream traversal, 16-bit block
trailers, and CRC-32. The checked-in C++ golden test passes byte-for-byte against
the committed Python-generated vector (75-byte shard):

```text
c++ -std=c++17 -Wall -Wextra -Werror -Isrc/tele \
  src/tele/pmini_codec.cpp test/host-stub/pmini-codec-test.cpp -o /tmp/pmini-codec-test
/tmp/pmini-codec-test
# pmini native golden/guard tests passed (75 bytes)
```

This is a source review plus golden-vector evidence, not an ESP32 execution or
peak-heap proof. The full Python pytest suite was not runnable on this host
because the selected Python installation has no `pytest` module.

## Adapter bounds and failure behavior

The parser bounds one input batch at 8192 bytes, 128 data rows, 32 union fields,
4096 SIDs, a 4096-byte common prefix, and 512 descriptors. It accepts only
canonical varints, complete length-prefixed frames, valid table dtypes, known
string measurement/tag SIDs, known numeric field SIDs, complete values, and one
complete timestamp. f64, duplicate fields, unsupported frame types, mixed
measurement/tag prefixes, and contradictory field-order cycles return false.

All candidate output is built in a local string and is appended only after
native encoding and CRC succeed. `std::bad_alloc` also returns false. The SID
cache is private to the adapter and staged per call: a malformed/partial batch
cannot publish a table map. A complete table-only fallback publishes its valid
tables so a split next batch can be encoded; a successful data batch publishes
its tables after encoding.

### Worst-case transient memory estimate

For the explicit adapter limits (64-bit host/ESP C++ `size_t`), the largest
adapter-owned pieces are approximately:

| allocation | bound |
|---|---:|
| 512 compact event descriptors | 4 KiB |
| compact row/ref indexes (128 rows, input-offset refs) | about 12 KiB |
| row words (128 x 35 u32) | 18 KiB |
| staged SID map, prefix, schemas | 9 KiB |
| envelope plus shard/input literals | about 25 KiB |

The conservative adapter total is now about 70 KiB transient, before allocator
headers and the native codec's own working vectors. A host allocation tracker on
the 6,700-byte, 50-row/24-field native smoke batch measured 19,868 bytes peak
for the adapter plus native codec (excluding stack and allocator metadata). The
actual ESP32 peak still needs a target-build measurement; the adapter does not
allocate based on an unbounded SID or frame length.

## Validation

Host warning-clean tests and sanitizers:

```text
c++ -std=c++17 -Wall -Wextra -Werror -I. -Isrc/tele \
  test/host-stub/pmini-packet-test.cpp src/tele/pmini_packet.cpp \
  -o /tmp/pmini-packet-test
/tmp/pmini-packet-test                         # passed

c++ -std=c++17 -O1 -g -fsanitize=address,undefined \
  -fno-omit-frame-pointer -Wall -Wextra -Werror -I. -Isrc/tele \
  test/host-stub/pmini-packet-fuzz-test.cpp src/tele/pmini_packet.cpp \
  -o /tmp/pmini-packet-fuzz-asan
ASAN_OPTIONS=halt_on_error=1 /tmp/pmini-packet-fuzz-asan # 100,000 random inputs, passed

# The same ASAN/UBSAN flags on pmini-codec-test: passed.
```

The packet test covers table-only state seeding, reset, omitted fields,
late-field union ordering, contradictory reorder cycles, f64 fallback,
truncation, and protection against publishing a table before a later malformed
frame. A native-codec smoke test with ASAN/UBSAN also encoded a real PMA2 packet.

On this ARM64 macOS host, an `-O2` run over a 6,700-byte batch (50 rows, 24
numeric fields, native codec, 100 repetitions) measured 71.2 microseconds per
adapter call and produced 371-byte PMA2 payloads. This is a host diagnostic only;
it is not an ESP32 latency claim.

## Guard checklist

1. Unevaluable input: malformed, unknown, unsupported, oversized, allocation,
   and native-codec failures all return false without appending output.
2. Monotonicity: increasing input size, row/field/SID counts, or adding an order
   contradiction can only reach fallback; no failure is cached as success.
3. Preconditions: frame, table, SID, dtype, width, timestamp, schema, and native
   column bounds are checked before the encoder call.
4. Source of truth: values are copied from the current frame bytes and dtypes
   from complete table frames into a private staged map; the producer table is
   never read concurrently.
5. Persistence: failed partial batches do not publish a map; table-only and
   successful batches publish only after complete validation.
6. Provenance: false means the caller must retain/send the original bytes using
   tamp; no fallback is reported as pmini success.
7. Known-bad calibration: sanitizer fuzzing and focused malformed/reorder/f64
   tests observe false and unchanged output.
8. Fix versus mute: PMA2 carries raw value bytes (as u32 words), timestamps,
   presence, schema dtypes/order, and literal table frames; it does not suppress
   a parse warning while emitting lossy data.
