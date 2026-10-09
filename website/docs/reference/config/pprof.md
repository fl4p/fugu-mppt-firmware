---
title: pprof.conf
sidebar_position: 17
---

# pprof.conf

`pprof.conf` configures the sampling profiler. It has one key:

| key            | unit | type | default | description                                                             |
|----------------|------|------|---------|-------------------------------------------------------------------------|
| `sprofiler_hz` | Hz   | int  | 0       | Sampling profiler frequency, ~100–300 (needs OpenOCD attached); 0 = off |
