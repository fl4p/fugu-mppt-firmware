// Host sim heap counter, see sim_alloc.cpp.
#pragma once
#include <cstdint>

struct SimRtAlloc {
    volatile bool active;  // inside the simulated RT iteration
    volatile bool paused;  // harness work inside the iteration (log hook), not counted
    bool armed;            // past the grace time: record sites
    bool mallocHooked;
    uint64_t allocs, frees, armedAllocs, armedFrees;
    uintptr_t sites[16];
    int nSites;
    double (*now)();
    double nowSec() const { return now ? now() : 0; }
};

extern SimRtAlloc g_simRtAlloc;

struct SimAllocPause {
    bool was = g_simRtAlloc.paused;
    SimAllocPause() { g_simRtAlloc.paused = true; }
    ~SimAllocPause() { g_simRtAlloc.paused = was; }
};
