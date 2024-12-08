# Real-Time Counter / Profiler

the firmware includes a simple real-time latency profiler.
within the time-critical loop, you'll find `rtcount("<label>")` calls.
Each of these call marks the end of labeled block.
`rtcount()` captures the time that has passed since the last call.
For time measurements it uses cycle counters.
The profiler stores statistics of the elapsed (min, max, mean), which can be displayed
with `rtcount_print();` (or if you send `reset-lag` on the console).
The most important statistic is the `max` and the results are sorted by the max value.

To precisely profile an expression, enclose it between two `rtcount` calls:

```

rtcount("someFunc.pre");
someFuncToMeasure();
rtcount("someFunc");

```

