// Host sim: counts heap allocations and frees made while g_simRtAlloc.active (inside loopRTIter()).
// Global operator new/delete are replaced; on macOS the default malloc zone is hooked as well, so
// direct malloc/calloc/realloc/free (libc internals included) are counted. The first distinct
// call sites past the grace time are printed with a backtrace.
#include "sim_alloc.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <execinfo.h>
#include <new>
#include <unistd.h>

#ifdef __APPLE__
#include <malloc/malloc.h>
#include <mach/mach.h>
#endif

SimRtAlloc g_simRtAlloc{};

static int t_inHook = 0; // not thread_local: a macOS TLV allocates on first use, inside the hook (sim is single-threaded)

static void report(bool isFree) {
    auto &a = g_simRtAlloc;
    if (isFree) ++a.frees; else ++a.allocs;
    if (!a.armed) return;
    if (isFree) ++a.armedFrees; else ++a.armedAllocs;
    ++t_inHook;
    void *bt[24];
    int n = backtrace(bt, 24);
    uintptr_t h = isFree;
    for (int i = 2; i < n && i < 8; ++i) h = h * 1000003u ^ (uintptr_t) bt[i];
    bool seen = false;
    for (int i = 0; i < a.nSites; ++i) if (a.sites[i] == h) seen = true;
    if (!seen && a.nSites < (int) (sizeof(a.sites) / sizeof(a.sites[0]))) {
        a.sites[a.nSites++] = h;
        fprintf(stderr, "!! RT %s, site #%d (t=%.4f s), backtrace:\n", isFree ? "free" : "alloc", a.nSites, a.nowSec());
        backtrace_symbols_fd(bt, n, STDERR_FILENO);
    }
    --t_inHook;
}

static inline void count(bool isFree) {
    if (g_simRtAlloc.active && !g_simRtAlloc.paused && !t_inHook) report(isFree);
}

void *operator new(size_t n) {
    count(false);
    ++t_inHook;
    void *p = malloc(n ? n : 1);
    --t_inHook;
    if (!p) throw std::bad_alloc();
    return p;
}
void *operator new[](size_t n) { count(false); ++t_inHook; void *p = malloc(n ? n : 1); --t_inHook; if (!p) throw std::bad_alloc(); return p; }
void *operator new(size_t n, const std::nothrow_t &) noexcept { count(false); ++t_inHook; void *p = malloc(n ? n : 1); --t_inHook; return p; }
void *operator new[](size_t n, const std::nothrow_t &) noexcept { count(false); ++t_inHook; void *p = malloc(n ? n : 1); --t_inHook; return p; }
static inline void del(void *p) {
    if (!p) return;
    count(true);
    ++t_inHook;
    free(p);
    --t_inHook;
}
void operator delete(void *p) noexcept { del(p); }
void operator delete[](void *p) noexcept { del(p); }
void operator delete(void *p, size_t) noexcept { del(p); }
void operator delete[](void *p, size_t) noexcept { del(p); }
void operator delete(void *p, const std::nothrow_t &) noexcept { del(p); }
void operator delete[](void *p, const std::nothrow_t &) noexcept { del(p); }

#ifdef __APPLE__
// malloc() dispatches through the registered zones, so each is hooked (not malloc_default_zone(), a forwarder)
static constexpr int kZones = 8;
static malloc_zone_t *zPtr[kZones];
static malloc_zone_t zOrig[kZones];
static int nZones = 0;

static const malloc_zone_t &orig(malloc_zone_t *z) {
    for (int i = 0; i < nZones; ++i) if (zPtr[i] == z) return zOrig[i];
    return zOrig[0];
}
static void *zMalloc(malloc_zone_t *z, size_t n) { count(false); return orig(z).malloc(z, n); }
static void *zCalloc(malloc_zone_t *z, size_t k, size_t n) { count(false); return orig(z).calloc(z, k, n); }
static void *zRealloc(malloc_zone_t *z, void *p, size_t n) { count(false); return orig(z).realloc(z, p, n); }
static void *zMemalign(malloc_zone_t *z, size_t a, size_t n) { count(false); return orig(z).memalign(z, a, n); }
static void zFree(malloc_zone_t *z, void *p) { if (p) count(true); orig(z).free(z, p); }
static void zFreeSize(malloc_zone_t *z, void *p, size_t n) { if (p) count(true); orig(z).free_definite_size(z, p, n); }
#if defined(MALLOC_ZONE_FN_PTR) || defined(MALLOC_TYPE_TMO_ENABLED)
#define SIM_TYPED_ZONE 1
// typed entry points (zone version >= 16): plain malloc/calloc/realloc land here on current SDKs
static void *zTMalloc(malloc_zone_t *z, size_t n, malloc_type_id_t t) { count(false); return orig(z).malloc_type_malloc(z, n, t); }
static void *zTCalloc(malloc_zone_t *z, size_t k, size_t n, malloc_type_id_t t) { count(false); return orig(z).malloc_type_calloc(z, k, n, t); }
static void *zTRealloc(malloc_zone_t *z, void *p, size_t n, malloc_type_id_t t) { count(false); return orig(z).malloc_type_realloc(z, p, n, t); }
static void *zTMemalign(malloc_zone_t *z, size_t a, size_t n, malloc_type_id_t t) { count(false); return orig(z).malloc_type_memalign(z, a, n, t); }
static void *zTMallocOpt(malloc_zone_t *z, size_t a, size_t n, uint64_t o, malloc_type_id_t t) { count(false); return orig(z).malloc_type_malloc_with_options(z, a, n, o, t); }
#endif

static bool hookZone(malloc_zone_t *z) {
    if (nZones >= kZones) return false;
    const vm_address_t page = (vm_address_t) z & ~(vm_address_t) (vm_page_size - 1);
    const vm_size_t len = (vm_address_t) z + sizeof(malloc_zone_t) - page; // the struct may cross a page
    if (vm_protect(mach_task_self(), page, len, 0, VM_PROT_READ | VM_PROT_WRITE) != KERN_SUCCESS)
        return false;
    zPtr[nZones] = z;
    zOrig[nZones++] = *z;
    z->malloc = zMalloc;
    z->calloc = zCalloc;
    z->realloc = zRealloc;
    z->free = zFree;
    if (z->version >= 5 && z->memalign) z->memalign = zMemalign;
    if (z->version >= 6 && z->free_definite_size) z->free_definite_size = zFreeSize;
#ifdef SIM_TYPED_ZONE
    if (z->version >= 16) {
        if (z->malloc_type_malloc) z->malloc_type_malloc = zTMalloc;
        if (z->malloc_type_calloc) z->malloc_type_calloc = zTCalloc;
        if (z->malloc_type_realloc) z->malloc_type_realloc = zTRealloc;
        if (z->malloc_type_memalign) z->malloc_type_memalign = zTMemalign;
        if (z->malloc_type_malloc_with_options) z->malloc_type_malloc_with_options = zTMallocOpt;
    }
#endif
    return true; // left writable: the struct may share its page with allocator state
}

__attribute__((constructor)) static void hookMallocZones() {
    vm_address_t *zones = nullptr;
    unsigned n = 0;
    bool ok = malloc_get_all_zones(mach_task_self(), nullptr, &zones, &n) == KERN_SUCCESS && n;
    for (unsigned i = 0; ok && i < n; ++i) ok = hookZone((malloc_zone_t *) zones[i]);
    g_simRtAlloc.mallocHooked = ok;
    if (!ok) fprintf(stderr, "sim_alloc: malloc zones not hooked, counting operator new/delete only\n");
}
#endif
