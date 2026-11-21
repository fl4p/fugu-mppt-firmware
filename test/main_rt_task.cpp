// rtLoopRunning() on target (QEMU or a bench board): MAIN_SRC=../test/main_rt_task.cpp, see test/rt_task_qemu.py
#include <Arduino.h>

#include "etc/rt_task.h"
#include "util.h"

time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;
void systemRestart() { esp_restart(); }
void consoleFlush() {}

static int fails = 0;

static void expect(const char *name, bool want) {
    const bool got = rtLoopRunning(), pass = got == want;
    if (!pass) ++fails;
    printf("RTTEST %s %s -> %d\n", pass ? "PASS" : "FAIL", name, got);
}

static volatile bool spin = true;

static void busyTask(void *) {
    while (spin) {} // never yields: a loop that is merely busy
    vTaskDelete(nullptr);
}

static void parkedTask(void *) {
    for (;;) vTaskDelay(1000);
}

static void selfDeleteTask(void *) { vTaskDelete(nullptr); }

void setup() {
    printf("RTTEST begin\n");
    expect("never created", false);

    TaskHandle_t h = nullptr;
    xTaskCreatePinnedToCore(parkedTask, kRtTaskName, 4096, nullptr, 5, &h, 1);
    delay(50);
    expect("blocked", true);
    vTaskDelete(h);
    delay(50);
    expect("deleted", false);

    xTaskCreatePinnedToCore(busyTask, kRtTaskName, 4096, nullptr, 20, &h, 1);
    delay(50);
    expect("busy, never yields", true);
    spin = false;
    delay(50);

    xTaskCreatePinnedToCore(selfDeleteTask, kRtTaskName, 4096, nullptr, 5, nullptr, 1);
    delay(50);
    expect("deleted itself", false);

    xTaskCreatePinnedToCore(parkedTask, "otherTask", 4096, nullptr, 5, &h, 1);
    delay(50);
    expect("other task only", false);
    vTaskDelete(h);

    printf("RTTEST done %d failures\n", fails);
}

void loop() { delay(1000); }
