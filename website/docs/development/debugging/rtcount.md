---
title: rtcount
sidebar_position: 5
---

# Real-Time Counter / Profiler

The `rtcount` profiler measures the real-time latency of labeled blocks in the time-critical loop.
Each `rtcount("<label>")` call in the loop marks the end of a labeled block. The call uses cycle counters to capture
the time that has passed since the previous call.

The profiler keeps statistics of the elapsed time (min, max, mean). `rtcount_print(false)` displays them, and
`rtcount_print(true)` also resets them, as the `reset-lag` console command does.
The output is sorted by `max`, the most important statistic.

Real-time performance depends on the maximum time the CPU spends executing a block. Speed profiling, by contrast,
looks at the average or total execution time. The average gives information about the empirical distribution of
the measured execution times.

To precisely profile an expression, enclose it between two `rtcount` calls:

```

rtcount("someFunc.pre");
someFuncToMeasure();
rtcount("someFunc");

```

## Implementation notes

`rtcount()` runs on the RT core and must never touch the heap. A first-seen key allocating mid-loop once tripped a
TLSF heap assert. For this reason, stats live in a fixed, pre-allocated `rtcount_entry[RTCOUNT_MAX]` table
(`src/etc/rt.h`) instead of a map.

The fixed table has two consequences for callers:

- Labels must be string literals (or otherwise interned `const char*`). Lookup matches by pointer, not
  by string content. Two identical-looking literals from different translation units count
  separately, and a constructed or temporary string won't match itself across calls.
- There can be at most `RTCOUNT_MAX` (64) distinct labels. The table is capped and never grows, so excess
  labels are silently dropped. If a profiling session needs more, bump the constant in `rt.h`.

The profiler accumulates `total`/`max`/`min` in CPU cycles and divides them by the core clock (MHz) at print time.
Sub-microsecond blocks keep their precision instead of truncating to whole µs.
