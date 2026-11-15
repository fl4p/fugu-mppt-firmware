#pragma once

#include <cstdint>

// Heap evidence: a failed-allocation hook that only records, and a printer for the core-0
// low-frequency loop (failed allocations, low free heap), both rate-limited.
void heapDiagBegin();

void heapDiagLf(uint32_t nowMs);
