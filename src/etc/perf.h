#pragma once

#include "logging.h" // printf_mux

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS

#ifndef CONFIG_FREERTOS_USE_TRACE_FACILITY
#error "CONFIG_FREERTOS_USE_TRACE_FACILITY not enabled"
#endif


extern "C" {
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#define ARRAY_SIZE_OFFSET   5   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

/**
 * @brief   Function to print the CPU usage of tasks over a given duration.
 *
 * This function will measure and print the CPU usage of tasks over a specified
 * number of ticks (i.e. real time stats). This is implemented by simply calling
 * uxTaskGetSystemState() twice separated by a delay, then calculating the
 * differences of task run times before and after the delay.
 *
 * @note    If any tasks are added or removed during the delay, the stats of
 *          those tasks will not be printed.
 * @note    This function should be called from a high priority task to minimize
 *          inaccuracies with delays.
 * @note    When running in dual core mode, each core will correspond to 50% of
 *          the run time.
 *
 * @param   xTicksToWait    Period of stats measurement
 *
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_NO_MEM        Insufficient memory to allocated internal arrays
 *  - ESP_ERR_INVALID_SIZE  Insufficient array size for uxTaskGetSystemState. Trying increasing ARRAY_SIZE_OFFSET
 *  - ESP_ERR_INVALID_STATE Delay duration too short
 */
static esp_err_t print_real_time_stats(TickType_t xTicksToWait) {
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    uint32_t start_run_time, end_run_time;
    esp_err_t ret;
    uint32_t total_elapsed_time;

    //Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    start_array = (decltype(start_array)) malloc(sizeof(TaskStatus_t) * start_array_size);
    if (start_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    vTaskDelay(xTicksToWait);

    //Allocate array to store tasks states post delay
    end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    end_array = (decltype(end_array)) malloc(sizeof(TaskStatus_t) * end_array_size);
    if (end_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get post delay task states
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    //Calculate total_elapsed_time in units of run time stats clock period.
    total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto exit;
    }

    {
        // CORE% is per-core: total_elapsed_time is one core's worth of ticks, so a
        // core-pinned task reads 0..100% of its own core (idle = 100% - sum on that core).
        struct Row { const char *name; uint32_t rt; uint32_t pct; const char *core; };
        Row *rows = (Row *) malloc(sizeof(Row) * start_array_size);
        int nrows = 0;
        //Match each task in start_array to those in the end_array
        for (int i = 0; i < start_array_size; i++) {
            TaskHandle_t h = start_array[i].xHandle;
            int k = -1;
            for (int j = 0; j < end_array_size; j++) {
                if (start_array[i].xHandle == end_array[j].xHandle) {
                    k = j;
                    //Mark that task have been matched by overwriting their handles
                    start_array[i].xHandle = NULL;
                    end_array[j].xHandle = NULL;
                    break;
                }
            }
            if (k >= 0 && rows) {
                uint32_t et = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
                BaseType_t c = xTaskGetCoreID(h);
                rows[nrows].name = start_array[i].pcTaskName;
                rows[nrows].rt = et;
                rows[nrows].pct = (uint32_t) (((uint64_t) et * 100) / total_elapsed_time);
                rows[nrows].core = (c == tskNO_AFFINITY) ? "any" : (c == 0 ? "0" : "1");
                ++nrows;
            }
        }
        //Sort by CPU time, busiest first (insertion sort, ~15 rows)
        for (int i = 1; i < nrows; ++i) {
            Row key = rows[i];
            int j = i - 1;
            for (; j >= 0 && rows[j].rt < key.rt; --j) rows[j + 1] = rows[j];
            rows[j + 1] = key;
        }
        printf_mux("%-16s %4s %10s %5s\n", "TASK", "CORE", "RUN_TIME", "CORE%");
        for (int i = 0; i < nrows; ++i)
            printf_mux("%-16s %4s %10" PRIu32 " %4" PRIu32 "%%\n",
                       rows[i].name, rows[i].core, rows[i].rt, rows[i].pct);
        free(rows);
    }

    //Print unmatched tasks
    for (int i = 0; i < start_array_size; i++) {
        if (start_array[i].xHandle != NULL) {
            printf_mux("| %s | Deleted\n", start_array[i].pcTaskName);
        }
    }
    for (int i = 0; i < end_array_size; i++) {
        if (end_array[i].xHandle != NULL) {
            printf_mux("| %s | Created\n", end_array[i].pcTaskName);
        }
    }
    ret = ESP_OK;

exit: //Common return path
    free(start_array);
    free(end_array);
    return ret;
}
}
#else
inline esp_err_t print_real_time_stats(TickType_t xTicksToWait) {
    printf_mux("CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS not enabled\n");
    return ESP_FAIL;
}
#endif

// Run the 2s stats measurement synchronously in the caller's (console) task. Blocking keeps it
// serialized by the console dispatcher — no detached task to pile up when `rt-stats` is spammed
// (which SW-reset the device) — and the `OK:` reply lands *after* the table, so scripted callers
// actually capture the output. The vTaskDelay inside yields, so core0 housekeeping and the task WDT
// keep running during the window; the RT loop on core1 is unaffected. (perf.h has a single includer
// per the ODR note; cli.cpp calls this extern.)
void print_real_time_stats_blocking() {
    print_real_time_stats(pdMS_TO_TICKS(2000));
}
