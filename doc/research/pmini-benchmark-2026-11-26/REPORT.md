# Fugu pmini versus tamp benchmark — 2026-11-26

**Result:** worth a native telemetry prototype for active, batched streams; not a universal tamp replacement. On these two real-data excerpts pmini saves 20–23% at modeled BLE batching and about 47% at the configured UDP raw batch cap. Single-sample packets grow 36–41%. No firmware was changed or flashed.

## Corpus and provenance

Unaggregated `ha_van.mppt` records fetched read-only from InfluxDB. Each query selects the first 2,048 records from 2026-11-25 10:00–12:00 UTC. This is a short active-stream sample from each device, not a two-hour dataset or a representative full-day/fleet corpus. No forward filling, averaging, synthetic resampling, or deletion of sparse fields. Snapshot JSON includes the exact query and no credentials.

- fry: 2048 points, 45.631 s, median spacing 21 ms, 24 distinct fields.
- flat: 2048 points, 42.949 s, median spacing 21 ms, 21 distinct fields.

Fugu checkout HEAD: `b2af3c8a75f6fcff64acd57889a2a28605a74a39` (existing working-tree changes). Pmini HEAD: `47e3db969939f5a271d3bdbca276fa2798520378` (clean). `results.json` fingerprints both input snapshots, the benchmark scripts, actual Fugu encoder/tamp sources, and pmini Python kernels. The measured code is the working-tree source, not an assumed pristine HEAD.

## Method and accounting

- Baseline calls the actual `BinaryLineProtocol` and `TampCompress` C++ implementation through a host bridge, compiling vendored tamp C with `-O2`. Timestamp injection supplies each database timestamp. Numeric types follow the current MPPT producer: f32, int32 and boolean. Native ARM64 match-search code is used; no ESP32 speed claim follows.
- The database may contain rounded values originally sent via text telemetry. Replay preserves the fetched values at firmware wire precision, but cannot recover lost float bits. These are reconstructed packets, not recorded binary traffic. Production field order is reconstructed from the current source; snapshots may originate from older firmware.
- Pmini receives fixed-schema rows: absolute timestamp split into two u32 words, a u32 presence mask, and all numeric fields. Absent field slots contain zero but remain absent through the mask. Float bits, timestamps and presence are preserved. Every reconstructed C++ field is checked byte-for-byte against its corresponding typed row value.
- Main comparison is steady-state payload: tamp gets real data frames with table frames removed and a one-byte codec ID; pmini gets its complete MTP2 shard (including CRC) plus a proposed one-byte codec ID and one-byte pre-shared schema ID. Schema negotiation/recovery is **not implemented or charged**. Both require schema knowledge; this does not establish total deployed protocol savings. `tamp_wire` additionally records the actual baseline including table bursts/heartbeats.
- Control: the same fixed rows are transposed field-by-field into columns and then tamp-compressed, with two bytes of envelope. Pmini is compared against this as well as the existing wire representation. No delta preprocessing is added to this control.
- UDP model uses this checkout’s `CONFIG_TCP_MSS=1440` and `maxBatchRaw = MSS*3/2 = 2160` bytes, closing before the next full encoded point would overflow. BLE assumes a 244-byte notify payload, a 366-byte raw flush threshold and a 250 ms timer; grouping follows input timestamps and whole points. This models batching, not scheduler phases, queue drops, retransmissions, ATT segmentation or actual negotiated MTU. BLE at a 20-byte notify payload could behave closer to the single-sample case.
- Fixed row-count batches expose the latency/ratio tradeoff. Partial final batches are included. Sizes exclude IP/UDP/ATT framing; BLE record delimiters/length prefixes are also excluded. Large history batches are not assumed to fit one network packet.

## Results

Positive savings means smaller than steady-state binary+tamp. Bytes are totals across all 2,048 points per device.

| Device | Batch policy | Mean rows | Tamp bytes | Pmini bytes | Savings | Columnar+tamp bytes | Median sample span |
|---|---|---:|---:|---:|---:|---:|---:|
| fry | rows-1 | 1.0 | 120,378 | 169,772 | -41.0% | 128,738 | 0 ms |
| fry | rows-2 | 2.0 | 88,155 | 107,010 | -21.4% | 105,327 | 21 ms |
| fry | rows-4 | 4.0 | 70,686 | 67,859 | +4.0% | 88,359 | 63 ms |
| fry | rows-8 | 8.0 | 61,267 | 47,665 | +22.2% | 77,955 | 147 ms |
| fry | rows-16 | 16.0 | 55,528 | 36,711 | +33.9% | 71,739 | 316 ms |
| fry | rows-32 | 32.0 | 51,787 | 28,662 | +44.7% | 66,488 | 653 ms |
| fry | rows-64 | 64.0 | 49,903 | 22,935 | +54.0% | 61,960 | 1327 ms |
| fry | rows-128 | 128.0 | 48,943 | 19,767 | +59.6% | 59,082 | 2676 ms |
| fry | rows-256 | 256.0 | 48,476 | 18,066 | +62.7% | 57,129 | 5376 ms |
| fry | ble-244B-chunk | 7.3 | 62,143 | 49,695 | +20.0% | 79,395 | 146 ms |
| fry | udp-2160B | 39.4 | 50,952 | 26,889 | +47.2% | 64,931 | 840 ms |
| flat | rows-1 | 1.0 | 118,062 | 160,240 | -35.7% | 121,411 | 0 ms |
| flat | rows-2 | 2.0 | 86,657 | 101,668 | -17.3% | 100,037 | 21 ms |
| flat | rows-4 | 4.0 | 69,593 | 64,624 | +7.1% | 84,106 | 63 ms |
| flat | rows-8 | 8.0 | 60,386 | 45,472 | +24.7% | 74,329 | 147 ms |
| flat | rows-16 | 16.0 | 54,831 | 34,965 | +36.2% | 68,096 | 314 ms |
| flat | rows-32 | 32.0 | 51,353 | 28,452 | +44.6% | 63,521 | 650 ms |
| flat | rows-64 | 64.0 | 49,371 | 23,786 | +51.8% | 59,785 | 1320 ms |
| flat | rows-128 | 128.0 | 48,416 | 21,128 | +56.4% | 57,296 | 2664 ms |
| flat | rows-256 | 256.0 | 47,912 | 19,772 | +58.7% | 55,462 | 5348 ms |
| flat | ble-244B-chunk | 7.6 | 60,763 | 46,604 | +23.3% | 74,889 | 146 ms |
| flat | udp-2160B | 40.2 | 50,440 | 26,932 | +46.6% | 62,241 | 857 ms |

## Memory and timing

Workspace figures below count allocated array/bytearray payloads in the existing Python encoder. They exclude object headers, input/output materialization, decoder memory and transient allocations, and are **not measured ESP32 heap peaks**. Defaults retain relatively large fixed buffers; tuning or a C++ port could change this. Tamp’s 1 KiB dictionary likewise excludes packet/string buffers.

Host timings are single-run per-batch medians on this ARM64 Mac: C++/C tamp through ctypes versus CPython pmini with a reused workspace, including BytesIO creation for its input. They are diagnostic only, not a fair comparison of algorithm speed or an estimate of ESP32 execution time.

| Device / policy | Pmini workspace buffers | Max pmini packet | Host tamp median | Host pmini median |
|---|---:|---:|---:|---:|
| fry / ble-244B-chunk | 20,755 B | 237 B | 57 µs | 900 µs |
| fry / udp-2160B | 22,339 B | 571 B | 155 µs | 3272 µs |
| flat / ble-244B-chunk | 20,668 B | 222 B | 57 µs | 891 µs |
| flat / udp-2160B | 22,300 B | 590 B | 161 µs | 3017 µs |

## Validation and conclusion

All 8,830 tested batches passed exact pmini decode-to-frame-byte equality, tamp decode-to-source equality for both wire variants and the columnar control. Each source point’s C++ field bytes and timestamp were also checked against the proposed typed representation. Empty/unknown input schemas or codec errors abort the run; there is no success cache. Assertions must remain enabled (do not run Python with `-O`).

For these active samples, a native pmini prototype is justified, particularly for UDP or history transfer. Keep tamp available for small/sparse batches and firmware OTA. Before deployment, implement and cost schema negotiation/recovery, test full-day and sparse/idle captures plus actual binary packet captures, and measure native ESP32 peak memory and execution time. The current benchmark establishes compression size on the stated corpus, not real-world device performance or a completed firmware integration.

## Reproduce

From the firmware repository root, using its Python environment with tamp installed:

```sh
.venv/bin/python doc/research/pmini-benchmark-2026-11-26/run.py /Users/fab/dev/esp/pmini
```

Uses the saved JSON snapshots, recompiles the host bridge into `/tmp/fugu-pmini-benchmark`, verifies all round trips, and rewrites `results.json`. `fetch.py` can refresh the same queries using `INFLUX_HA_VAN_USER` and `INFLUX_HA_VAN_PASS`; do not print those values. Local data and code only; no external web research was used.
