#pragma once

#include <cstddef>

// Core-1 call-stack sampler (CONFIG_FUGU_WITH_RTPROF): a timer ISR on core 1 unwinds the interrupted
// stack and counts every flash-resident return address in a fixed hash table.
void rtprofStart(unsigned hz);
// all: count IRAM/ROM addresses too (where the core waits), not only flash ones
void rtprofClear(bool all = false);
// prints "rtprof samples= ..." and the table as "pc:count" lines through UART_LOG
void rtprofDump();
