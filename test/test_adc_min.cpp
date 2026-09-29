// Minimal on-target test for the GPIO-ISR core affinity that the sensor alert path depends on.
//
// The INA226 / ADS1115 "conversion-ready" alert (adc/ina226.h, adc/ads.h) is delivered through
// attachInterrupt(). arduino-esp32 installs the shared GPIO ISR service lazily on the *calling*
// core the first time any attachInterrupt() runs, and that install core fixes which CPU every GPIO
// alert dispatcher fires on. If it lands on core 0 the alert hop adds IPC latency to the RT sampler
// wake on core 1 -> "no ADC samples for 1000 ms" starvation. src/main.cpp::pinGpioIsrToRtCore()
// pre-installs the service on RT_CORE to avoid that.
//
// These tests prove (a) the install core fully determines the dispatcher core, and (b) the
// pre-install-then-attachInterrupt sequence the firmware uses really pins the alert ISR to RT_CORE.

#include <unity.h>
#include <Arduino.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "util.h"   // RT_CORE, NON_RT_CORE
#include "etc/rt.h" // RT_PRIO

// A GPIO used as a self-triggering interrupt source: driven as an output that feeds its own
// input edge, so no external wiring is needed. 21 is the Fugu2 HS gate input, so pulses are ~1 us and
// the pin idles LOW; still only run on a board without a powered power stage. Override per board.
#ifndef TEST_ISR_GPIO
#define TEST_ISR_GPIO 21
#endif

static const char *TAG = "test_adc";

static volatile int s_isrCore = -1;
static volatile uint32_t s_isrCount = 0;
static portMUX_TYPE s_pulseMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR onEdgeArg(void *) { s_isrCore = xPortGetCoreID(); ++s_isrCount; }
static void onEdgeArduino() { s_isrCore = xPortGetCoreID(); ++s_isrCount; }

struct InstallCtx { esp_err_t r; TaskHandle_t caller; };

static void installTask(void *arg) {
    auto *ctx = (InstallCtx *) arg;
    ctx->r = gpio_install_isr_service(0);
    xTaskNotifyGive(ctx->caller);
    vTaskDelete(nullptr);
}

// Install the shared GPIO ISR service on `core` via a task pinned there, mirroring
// pinGpioIsrToRtCore(): gpio_install_isr_service() runs a nested esp_ipc_call_blocking() to the
// calling core, so it must not itself run inside an IPC call. The dispatcher's affinity follows the
// installing core.
static esp_err_t installGpioIsrServiceOnCore(int core) {
    InstallCtx ctx{ESP_FAIL, xTaskGetCurrentTaskHandle()};
    TaskHandle_t t = nullptr;
    xTaskCreatePinnedToCore(installTask, "isrInit", 3072, &ctx, RT_PRIO, &t, core);
    if (!t) return ESP_FAIL;
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
    return ctx.r;
}

static void selfTrigger() {
    for (int i = 0; i < 8 && s_isrCount == 0; ++i) {
        portENTER_CRITICAL(&s_pulseMux); // bound the high time; the latched edge fires the ISR after exit
        gpio_set_level((gpio_num_t) TEST_ISR_GPIO, 1);
        esp_rom_delay_us(1);
        gpio_set_level((gpio_num_t) TEST_ISR_GPIO, 0);
        portEXIT_CRITICAL(&s_pulseMux);
        delay(2);
    }
}

// Install the service on `installCore`, register a raw handler on TEST_ISR_GPIO, fire one rising
// edge, and return the core the handler ran on (-1 = never fired). Tears everything down so it can
// be called for either core within one boot.
static int captureIsrCoreRaw(int installCore) {
    s_isrCore = -1;
    s_isrCount = 0;
    esp_err_t r = installGpioIsrServiceOnCore(installCore);
    if (r != ESP_OK) { ESP_LOGE(TAG, "install on core %d failed: %s", installCore, esp_err_to_name(r)); return -2; }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << TEST_ISR_GPIO,
        .mode = GPIO_MODE_INPUT_OUTPUT, // output that also feeds its own input buffer
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io);
    gpio_set_level((gpio_num_t) TEST_ISR_GPIO, 0);
    gpio_isr_handler_add((gpio_num_t) TEST_ISR_GPIO, onEdgeArg, nullptr);

    selfTrigger();

    gpio_isr_handler_remove((gpio_num_t) TEST_ISR_GPIO);
    gpio_set_intr_type((gpio_num_t) TEST_ISR_GPIO, GPIO_INTR_DISABLE);
    gpio_uninstall_isr_service();
    return s_isrCore;
}

// The dispatcher core follows the install core: installing on core 0 -> ISR on core 0.
void test_gpio_isr_lands_on_install_core_0() {
    int core = captureIsrCoreRaw(0);
    ESP_LOGW(TAG, "install core 0 -> ISR ran on core %d (fired %lu)", core, (unsigned long) s_isrCount);
    TEST_ASSERT_GREATER_THAN_UINT32(0, s_isrCount); // rig sanity: edge actually fired
    TEST_ASSERT_EQUAL_INT(0, core);
}

// ...and installing on RT_CORE -> ISR on RT_CORE. This is the affinity the RT sampler needs.
void test_gpio_isr_lands_on_install_core_rt() {
    int core = captureIsrCoreRaw(RT_CORE);
    ESP_LOGI(TAG, "install core RT=%d -> ISR ran on core %d (fired %lu)", RT_CORE, core, (unsigned long) s_isrCount);
    TEST_ASSERT_GREATER_THAN_UINT32(0, s_isrCount);
    TEST_ASSERT_EQUAL_INT(RT_CORE, core);
}

// Faithful firmware repro: pre-install on RT_CORE, THEN attachInterrupt() (as setupSensors does for
// the INA226 alert). arduino-esp32's lazy gpio_install_isr_service() then logs the benign
// "GPIO isr service already installed" line and the alert ISR fires on RT_CORE. Must be the first
// attachInterrupt() of the run (arduino caches its install state), so it runs before the raw tests.
void test_attachinterrupt_after_preinstall_uses_rt_core() {
    gpio_uninstall_isr_service();
    TEST_ASSERT_EQUAL(ESP_OK, installGpioIsrServiceOnCore(RT_CORE));

    s_isrCore = -1;
    s_isrCount = 0;
    pinMode(TEST_ISR_GPIO, OUTPUT);
    gpio_set_level((gpio_num_t) TEST_ISR_GPIO, 0);
    attachInterrupt(digitalPinToInterrupt(TEST_ISR_GPIO), onEdgeArduino, RISING);
    selfTrigger();
    detachInterrupt(digitalPinToInterrupt(TEST_ISR_GPIO));

    ESP_LOGI(TAG, "attachInterrupt after RT pre-install -> ISR ran on core %d (fired %lu)",
             s_isrCore, (unsigned long) s_isrCount);
    TEST_ASSERT_GREATER_THAN_UINT32(0, s_isrCount);
    TEST_ASSERT_EQUAL_INT(RT_CORE, s_isrCore);
    gpio_uninstall_isr_service();
}
