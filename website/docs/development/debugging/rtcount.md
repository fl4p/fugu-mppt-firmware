---
title: rtcount
sidebar_position: 5
---

# Real-Time Counter / Profiler

the firmware includes a simple real-time latency profiler.
within the time-critical loop, you'll find `rtcount("<label>")` calls.
Each of these call marks the end of labeled block.
`rtcount()` captures the time that has passed since the last call.
For time measurements it uses cycle counters.
The profiler stores statistics of the elapsed (min, max, mean), which can be displayed
with `rtcount_print(false)` (`rtcount_print(true)` also resets them, as the `reset-lag` console command does).
The most important statistic is the `max` and the results are sorted by the max value.

When evaluating the real-time performance of a code block, we focus on the maximum time spent by the CPU to execute that
blocks. This is different from speed performance profiling, were the average or total execution is of interest.

The average gives us information about the empirical distribution of the measured execution times.

To precisely profile an expression, enclose it between two `rtcount` calls:

```

rtcount("someFunc.pre");
someFuncToMeasure();
rtcount("someFunc");

```

## Implementation notes

Stats live in a fixed, pre-allocated `rtcount_entry[RTCOUNT_MAX]` table (`src/etc/rt.h`), not a map —
`rtcount()` runs on the RT core and must never touch the heap (a first-seen key allocating mid-loop once
tripped a TLSF heap assert).

Two consequences for callers:

- **Labels must be string literals** (or otherwise interned `const char*`). Lookup matches by pointer, not
  by string content, so two identical-looking literals from different translation units would count
  separately, and a constructed/temporary string won't match itself across calls.
- **At most `RTCOUNT_MAX` (64) distinct labels.** Excess labels are silently dropped (the table is capped,
  never grown). Bump the constant in `rt.h` if a profiling session needs more.

`total`/`max`/`min` are accumulated in CPU cycles and divided by the core clock (MHz) at print time, so
sub-microsecond blocks keep their precision instead of truncating to whole µs.

