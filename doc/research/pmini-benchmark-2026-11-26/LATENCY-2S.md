# Two-second latency follow-up — 2026-11-27

Same saved 4,096 database samples and accounting/limitations as REPORT.md. Fresh run of run_2s.py, with 1,800 ms and 2,000 ms batching deadlines, no raw-byte cap. Each batch starts at its first sample and excludes the first sample at or beyond its deadline. This is a deadline-flush model; the benchmark does not exercise an actual timer or measure end-to-end delivery.

| Device | Batching deadline | Mean rows | Tamp bytes | Pmini bytes | Savings | Largest pmini payload | Workspace buffers |
|---|---:|---:|---:|---:|---:|---:|---:|
| fry | latency-1800ms | 81.9 | 49548 | 21557 | 56.5% | 957 B | 24547 B |
| fry | latency-2000ms | 89.0 | 49358 | 21146 | 57.2% | 1047 B | 24979 B |
| flat | latency-1800ms | 85.3 | 48896 | 22477 | 54.0% | 1025 B | 24460 B |
| flat | latency-2000ms | 93.1 | 48801 | 22101 | 54.7% | 1121 B | 24892 B |

All 94 batches passed the existing byte-for-byte checks. Pmini payloads include codec/schema IDs and CRC; schema negotiation and network headers are excluded. Workspace counts exclude input/output and object/transient overhead.

A two-second end-to-end requirement needs a shorter batching deadline: 1.8 seconds leaves a provisional 200 ms for encoding, scheduling and transmission, but this margin is not validated on ESP32. The firmware currently has smaller transport batch limits; using this layout and larger batches requires implementation changes. Timer flushes must work even if no further sample arrives. Preserve packet-size limits as an independent early-flush condition.

Recommendation: pmini is a stronger candidate with this latency allowance. The 1.8-second case preserves nearly all of the measured size benefit; validate native memory/time and network delivery before promising the deadline.

Reproduce: `.venv/bin/python doc/research/pmini-benchmark-2026-11-26/run_2s.py /Users/fab/dev/esp/pmini`. Results and source fingerprints: `results_2s.json`.
