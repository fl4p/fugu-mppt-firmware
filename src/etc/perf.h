#pragma once

#include "logging.h" // printf_mux

#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/freertos_debug.h"
#include "esp_memory_utils.h"
#include "esp_attr.h"
#include "util.h" // flashOpActive

#ifndef CONFIG_FREERTOS_USE_TRACE_FACILITY
#error "CONFIG_FREERTOS_USE_TRACE_FACILITY not enabled"
#endif

// Task tables without uxTaskGetSystemState(). That call holds the kernel lock, with interrupts masked on
// the calling core, while it counts every task's untouched stack byte by byte (~40 ns/byte at 160 MHz,
// ~2 ms for ~17 tasks), and core 1 spins on the same lock in every notify, block and tick.
//
// Here the lists are walked only under the kernel lock, in chunks. A chunk resumes at a list index
// (an integer, never a list pointer) from that list's head, copies up to kTaskChunk tasks not seen yet
// (no stack scan) and releases the lock at that count or at the end of the list. No list pointer
// survives a release, so a task deleted or relinked between chunks cannot leave a dangling iterator; it
// can only be missed or met twice, which the "seen" check and the task count catch.
struct TaskRow {
    TaskHandle_t h;
    uint32_t rt;
    const uint8_t *stk, *stkEnd; // stack bounds, copied under the lock
    uint8_t prio, state;
    int8_t core;
    char name[configMAX_TASK_NAME_LEN];
};
static constexpr int kMaxTaskRows = 32, kTaskChunk = 2;

#if !CONFIG_FREERTOS_SMP && configNUMBER_OF_CORES > 1
#include "esp_private/freertos_idf_additions_priv.h" // prvTakeKernelLock: taskENTER_CRITICAL(&xKernelLock)
#define TASKS_LOCK() prvTakeKernelLock()
#define TASKS_UNLOCK() prvReleaseKernelLock()
#else
#define TASKS_LOCK() vTaskSuspendAll()
#define TASKS_UNLOCK() xTaskResumeAll()
#endif

// One locked chunk from list index *list: copies up to kTaskChunk unseen tasks into rows[n..]; returns the
// new n, or -1 if a flash op is in progress. Leaves in *list where to resume (all ones: walk complete) and
// in *want the task count read under the same lock. IRAM, as is all it calls (CONFIG_FREERTOS_IN_IRAM), but xTaskGetNext reads its table of
// non-ready lists from flash rodata: one cache line, a ~1 us miss, or ~260 us while an erase is suspended.
// Hence the flash check under the lock: with interrupts masked on core 0 no op can start there, and one
// starting on core 1 blocks on this lock taking the flash mutex (pvTaskIncrementMutexHeldCount).
static IRAM_ATTR NOINLINE_ATTR int taskChunk(TaskRow *rows, int n, int max, UBaseType_t *list, int *want) {
    int added = 0;
    TaskIterator_t it = {};
    it.uxCurrentListIndex = *list;
    TASKS_LOCK();
    if (flashOpActive()) {
        TASKS_UNLOCK();
        return -1;
    }
    *want = (int) uxTaskGetNumberOfTasks();
    int r = 0;
    for (int k = 0; k < 2 * kMaxTaskRows && added < kTaskChunk && n < max && (r = xTaskGetNext(&it)) != -1; ++k) {
        const TaskHandle_t h = it.pxTaskHandle;
        *list = (UBaseType_t) r;
        if (h) {
            int i = 0;
            while (i < n && rows[i].h != h) ++i;
            if (i == n) {
                TaskRow &w = rows[n++];
                ++added;
                TaskStatus_t st;
                vTaskGetInfo(h, &st, pdFALSE, eInvalid); // pdFALSE: no stack scan
                TaskSnapshot_t sn;
                vTaskGetSnapshot(h, &sn);
                w.h = h;
                w.rt = (uint32_t) st.ulRunTimeCounter;
                w.stk = (const uint8_t *) st.pxStackBase;
                w.stkEnd = (const uint8_t *) sn.pxEndOfStack;
                w.prio = (uint8_t) st.uxCurrentPriority;
                w.state = (uint8_t) st.eCurrentState;
                const BaseType_t c = xTaskGetCoreID(h);
                w.core = (int8_t) (c == tskNO_AFFINITY ? -1 : c);
                size_t j = 0;
                for (; j + 1 < sizeof(w.name) && st.pcTaskName[j]; ++j) w.name[j] = st.pcTaskName[j];
                w.name[j] = 0;
            }
        }
        if (!it.pxNextListItem) { // that was the list's last task
            *list = (UBaseType_t) r + 1;
            break;
        }
    }
    TASKS_UNLOCK();
    if (r == -1) *list = ~(UBaseType_t) 0; // walked past the last list
    return n;
}

// Fills rows; repeats the chunked walk (bounded) while tasks are missing. Sleeps >= 1 ms between chunks,
// longer than one RT iteration, so an RT iteration waits for at most one chunk. Waits out flash ops
// (max ~1 s).
static int taskRows(TaskRow *rows, int max) {
    int n = 0, want = 0, busy = 0;
    for (int pass = 0; pass < 3; ++pass) {
        UBaseType_t list = 0;
        for (int c = 0; c < 4 * max && list != ~(UBaseType_t) 0 && n < max;) {
            const int r = taskChunk(rows, n, max, &list, &want);
            if (r >= 0) { n = r; ++c; }
            else if (++busy > 500) break;
            vTaskDelay(2);
        }
        if (n >= want || busy > 500) break;
    }
    if (n < want || !want) printf_mux("(%d of %d tasks listed%s)\n", n, want, busy > 500 ? ", flash busy" : "");
    return n;
}

static const char *taskCoreStr(const TaskRow &r) { return r.core < 0 ? "any" : (r.core == 0 ? "0" : "1"); }

// Untouched (0xa5-filled) bytes from the stack's low end, bounded by its copied extent. Runs without a
// lock; if the task was deleted since the copy, the memory is freed heap (still mapped) and the value is
// meaningless but the read is safe.
static unsigned stackFree(const TaskRow &r) {
    if (!esp_ptr_byte_accessible(r.stk) || !esp_ptr_byte_accessible(r.stkEnd - 1) || r.stkEnd <= r.stk
        || r.stkEnd - r.stk > 65536)
        return 0;
    const uint8_t *p = r.stk;
    while (p < r.stkEnd && *p == 0xa5) ++p;
    return (unsigned) (p - r.stk);
}

// tasks: state, priority, pinned core and min-ever free stack (bytes).
void print_task_table() {
    static const char st[][4] = {"run", "rdy", "blk", "sus", "del", "?"};
    auto *rows = (TaskRow *) malloc(sizeof(TaskRow) * kMaxTaskRows);
    if (!rows) { printf_mux("tasks: oom\n"); return; }
    const int n = taskRows(rows, kMaxTaskRows);
    printf_mux("%-16s %-4s %3s %4s %9s\n", "NAME", "STAT", "PRI", "CORE", "STKFREE_B");
    for (int i = 0; i < n; ++i) {
        const TaskRow &r = rows[i];
        printf_mux("%-16s %-4s %3u %4s %9u\n", r.name, st[r.state < 5 ? r.state : 5], (unsigned) r.prio,
                   taskCoreStr(r), stackFree(r));
    }
    free(rows);
}

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
// CPU share per task over `ms`, busiest first. CORE% is per core: the elapsed time is one core's worth,
// so a pinned task reads 0..100 % of its own core (idle = 100 % - sum on that core).
static void print_real_time_stats(uint32_t ms) {
    auto *a = (TaskRow *) malloc(sizeof(TaskRow) * 2 * kMaxTaskRows);
    if (!a) { printf_mux("rt-stats: oom\n"); return; }
    TaskRow *b = a + kMaxTaskRows;
    const uint32_t t0 = (uint32_t) portGET_RUN_TIME_COUNTER_VALUE();
    const int na = taskRows(a, kMaxTaskRows);
    vTaskDelay(pdMS_TO_TICKS(ms));
    int nb = taskRows(b, kMaxTaskRows);
    const uint32_t total = (uint32_t) portGET_RUN_TIME_COUNTER_VALUE() - t0;
    for (int j = 0; j < nb;) {
        int i = 0;
        while (i < na && !(a[i].h == b[j].h && !strcmp(a[i].name, b[j].name))) ++i;
        if (i == na) {
            printf_mux("| %s | Created\n", b[j].name);
            b[j] = b[--nb];
            continue;
        }
        b[j].rt -= a[i].rt;
        a[i].h = nullptr;
        ++j;
    }
    for (int i = 1; i < nb; ++i) { // busiest first
        const TaskRow key = b[i];
        int j = i - 1;
        for (; j >= 0 && b[j].rt < key.rt; --j) b[j + 1] = b[j];
        b[j + 1] = key;
    }
    printf_mux("%-16s %4s %10s %5s\n", "TASK", "CORE", "RUN_TIME", "CORE%");
    for (int i = 0; i < nb; ++i)
        printf_mux("%-16s %4s %10" PRIu32 " %4" PRIu32 "%%\n", b[i].name, taskCoreStr(b[i]), b[i].rt,
                   total ? (uint32_t) ((uint64_t) b[i].rt * 100 / total) : 0);
    for (int i = 0; i < na; ++i)
        if (a[i].h) printf_mux("| %s | Deleted\n", a[i].name);
    free(a);
}
#else
static void print_real_time_stats(uint32_t) {
    printf_mux("CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS not enabled\n");
}
#endif

// Synchronous in the console task, so concurrent rt-stats are serialized by the dispatcher and the `OK:`
// reply follows the table. Single includer (main.cpp); cli.cpp calls these extern.
void print_real_time_stats_blocking() {
    print_real_time_stats(2000);
}
