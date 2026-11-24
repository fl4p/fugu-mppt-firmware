---
title: pprof.conf
sidebar_position: 17
---

# pprof.conf

`pprof.conf` configures the sampling profiler of `CONFIG_FUGU_WITH_SPROFILER` builds. The firmware reads it once at boot and starts the profiler only if a debugger (OpenOCD) is attached at that moment; a parse error skips the profiler. It has one key:

| key            | unit | type | default | description                                                             |
|----------------|------|------|---------|-------------------------------------------------------------------------|
| `sprofiler_hz` | Hz   | int  | 0       | Sampling profiler frequency, about 100–300; 0 = off                    |
