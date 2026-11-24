#pragma once

// ConfFile stages a rewrite as <name>.cft and renames it over <name>; the USB drive stages as .new.
constexpr const char *CONF_FILE_STAGE_SUFFIX = ".cft";
constexpr const char *USB_DRIVE_STAGE_SUFFIX = ".new";

#ifdef WITH_USB_MSC
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Held by a ConfFile rewrite and by the USB-drive commit: one replaces conf files at a time.
struct ConfWriteLock {
    ConfWriteLock() { xSemaphoreTake(mtx(), portMAX_DELAY); }
    ~ConfWriteLock() { xSemaphoreGive(mtx()); }
    ConfWriteLock(const ConfWriteLock &) = delete;
    ConfWriteLock &operator=(const ConfWriteLock &) = delete;

private:
    static SemaphoreHandle_t mtx() {
        static StaticSemaphore_t buf;
        static SemaphoreHandle_t m = xSemaphoreCreateMutexStatic(&buf);
        return m;
    }
};
#else
struct ConfWriteLock {
    ConfWriteLock() {}
};
#endif
