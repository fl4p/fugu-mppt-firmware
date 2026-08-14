
#include "logging.h"

#include <Arduino.h>
#include <esp_app_desc.h>
#include <Wire.h>
#include <USB.h>


#include "adc/sampling.h"

#include "console.h"
#include "tele/console_ble.h"
#include "cli.h"
#include "buck.h"
#include "mppt.h"
#include "service.h"
#include "adc/sensor_setup.h"
#include "util.h"
#include "viz/lcd.h"
#include "viz/lcd_service.h"
#include "viz/led.h"
#include "tele/console_ble_service.h"
#include "etc/network_shim.h"
#ifdef WITH_NETW
#include "tele/telemetry.h"
#include "tele/ftp_service.h"
#include "tele/telnet_service.h"
#include "tele/telemetry_service.h"
#include "tele/scope_service.h"
#include "tele/home_assistant.h"
#include "etc/ota.h"
#endif
#include "sync/bsync.h"
#ifdef WITH_MEASURE_COIL
#include "selftest/measure_coil.h"
#endif

#include "etc/version.h"

#include "etc/perf.h"
#ifdef WITH_SPROFILER
#include <sprofiler.h>
#endif

#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED

#include <hal/usb_serial_jtag_ll.h>

#endif

#include "storage/key-value.h"
#include "app_state.h"

#include <esp_task_wdt.h>
#include <esp_pm.h>
#include <esp_timer.h>
#include <esp_ota_ops.h>
#include <driver/gpio.h>
#include <dirent.h>

#include "etc/rt_core_check.h"


ADC_Sampler adcSampler{}; // schedules async ADC reading
VIinVout<const Sensor *> sensors{nullptr, nullptr, nullptr, nullptr};
SynchronousConverter converter; // buck or boost
LedIndicator led;
MpptController mppt{adcSampler, sensors, converter, lcdService.lcd}; // lcd owned by lcdService

time_us loopWallClockUs_ = 0;

time_us lastLoopTime = 0;

time_us timeLastSampler = 0;

time_us delayStartUntil = 0;

const auto lfPeriod = 3000000ULL; //(mppt.tracker.avgPower.get() < 1) ? 3000000 : 3000000;

time_us lastTimeOutUs = 0;
uint32_t lastNSamples = 0;

unsigned long lastMpptUpdateNumSamples = 0;

float conversionEfficiency;

AppState g_app{}; // mode flags / per-iteration max-lag bookkeeping (see app_state.h)

KeyValueStorage nvs{};

void systemRestart();

static void loopNetwork_task(void *arg);

static void loopRT(void *arg); // this is the critical one

static void loopRTNewData(time_ms nowMs);

#ifdef WITH_NETW
// arduino-esp32's WiFiGeneric.cpp:298 calls esp_netif_create_default_wifi_ap() unconditionally,
// but esp_wifi defines it only when CONFIG_ESP_WIFI_SOFTAP_SUPPORT=y. We turn SOFTAP off in
// sdkconfig.defaults (saves a few KB) and never act as an AP, so stub it inline here so the
// link succeeds. Inline because a separate TU was getting --gc-sections'd out of libmain.a
// before the linker had a chance to satisfy arduino's undefined ref. Signature must match
// esp_netif.h (included via bsync.h -> esp_wifi.h in MCPWM builds).
#include <esp_netif.h>
extern "C" esp_netif_t *esp_netif_create_default_wifi_ap(void) { return nullptr; }
#endif


// Single reused log literal for the repeated "failed to read <conf>.conf" catch blocks in setup().
static void logConfErr(const char *name, const std::exception &e) {
    ESP_LOGE("main", "conf %s: %s", name, e.what());
}

// Try-load a conf file. On parse error, log and (if fatal) set g_app.setupErr; return an empty ConfFile
// so the caller can keep going with defaults instead of throwing out of setup() → reboot loop.
// noWarnIfMissing forwards to ConfFile's flag (some confs are optional even when present).
static ConfFile loadConfSafe(const char *path, bool fatal = true, bool noWarnIfMissing = false) {
    try {
        return ConfFile{path, noWarnIfMissing};
    } catch (const std::exception &e) {
        logConfErr(path, e);
        if (fatal) g_app.setupErr = true;
        return ConfFile{};
    }
}

// I2C + LED + LCD bring-up from board.conf. SDA=255 means "no I2C wired" (LED only). The
// skip_assert escape exists for bench boards where the pre-init bus assertion is too strict.
static void setupI2C(const ConfFile &boardConf, bool &noI2C) {
    if (!boardConf) return;
    try {
        auto i2c_freq = boardConf.getLong("i2c_freq", 100000);
        auto i2c_sda = boardConf.getByte("i2c_sda", 255);
        noI2C = (i2c_sda == 255);
        if (!noI2C) {
            if (!boardConf.getByte("skip_assert", 0))
                try {
                    auto i2c_scl = boardConf.getLong("i2c_scl");
                    assertPinState(i2c_sda, true, "i2c_sda");
                    assertPinState(i2c_scl, true, "i2c_scl");
                    pinMode(i2c_scl, OUTPUT);
                    digitalWrite(i2c_scl, LOW);
                    usleep(1);
                    assertPinState(i2c_sda, true, "i2c_sda(scl_short)");
                } catch (const std::exception &e) {
                    ESP_LOGE("main", "error %s", e.what());
                    g_app.setupErr = true;
                }
            ESP_LOGI("main", "i2c pins SDA=%hi SCL=%hi freq=%lu", i2c_sda, boardConf.getByte("i2c_scl"), i2c_freq);
            if (!Wire.begin(i2c_sda, (uint8_t) boardConf.getLong("i2c_scl"), i2c_freq)) {
                ESP_LOGE("main", "Failed to initialize Wire");
                g_app.setupErr = true;
            }
        } else {
            ESP_LOGI("main", "no i2c_sda pin set");
        }

        led.begin(boardConf);
        led.setHexShort(0x111);

        if (!noI2C) {
            if (!lcdService.initLcd()) {
                ESP_LOGE("main", "Failed to init LCD");
            } else {
                lcdService.displayMessage("Fugu FW v" FIRMWARE_VERSION "\n" __DATE__ " " __TIME__, 2000);
            }
        }
    } catch (const std::exception &ex) {
        ESP_LOGE("main", "error %s", ex.what());
        g_app.setupErr = true;
    }
}

// Bring up WiFi at boot (when enabled) and load tele.conf. Returns a default-constructed TeleConf
// when WITH_NETW=0 or NO_WIFI is set; mppt.begin tolerates that.
static TeleConf setupNetworkAtBoot() {
    TeleConf teleConf{};
#ifdef WITH_NETW
#ifdef NO_WIFI
    g_app.disableWifi = true;
#endif
    // bare `wifi off` persists across reboots (nvs), `wifi off <minutes>` does not
    nvs.open();
    if (!nvs.readString("wifi_off", "").empty()) {
        g_app.disableWifi = true;
        ESP_LOGI("main", "WiFi disabled (wifi off), `wifi on` to re-enable");
    }
    nvs.close();
    if (!g_app.disableWifi) {
        connect_wifi_async();
        bool res = wait_for_wifi();
        led.setHexShort(res ? 0x565 : 0x200);
        lcdService.displayMessage(
            res ? ("WiFi connected.\n" + wifiLocalIp()) : "WiFi timeout.", 2000);

        try {
            teleConf = ConfFile{"/littlefs/conf/tele.conf"};
        } catch (const std::exception &e) {
            // telemetry host is optional — a malformed tele.conf must not brick the device
            logConfErr("tele.conf", e);
        }
    }
#endif
    return teleConf;
}

// Sensors → converter → MPPT/tracker bring-up. Sets g_app.setupErr on any failure. Kept as one unit
// because these initializations fail-together: without sensors the converter can't run safely,
// without coil/converter conf the PWM driver can't init.
static void setupConverterAndMppt(const ConfFile &boardConf, const Limits &lim, const TeleConf &teleConf, bool noI2C) {
    try {
        setupSensors(boardConf, lim);
        mppt.initSensors(boardConf);
#ifdef WITH_NETW
        scope->addChannel(&mppt, 0, 'u', 12, "vout_filt"); // scope owned by scopeService (scope -> &scopeObj)
#endif

        // converterConf outlives the block below: mppt.begin() reads the ctrl_* gains from it, so
        // its warnUnknownKeys() has to wait until after that (else those keys read as unknown).
        ConfFile converterConf{};

        if (!g_app.setupErr) {
            ConfFile coilConf{"/littlefs/conf/coil.conf"};
            ConfFile chargerConf{"/littlefs/conf/charger.conf"};
            converterConf = ConfFile{"/littlefs/conf/converter.conf"};

            mppt.charger.begin(chargerConf);
            converter.init(converterConf, boardConf, coilConf);
            chargerConf.warnUnknownKeys();
            coilConf.warnUnknownKeys();
        }

        if (!g_app.setupErr && !adcSampler.adcStates.empty()) {
            ConfFile trackerConf{"/littlefs/conf/tracker.conf", true};
            try {
                mppt.begin(trackerConf, boardConf, converterConf, lim, teleConf);
            } catch (...) {
                // begin() throws on a malformed ctrl_* value. The unknown-key warning is what names
                // a *typo'd* key, so it has to survive that throw — otherwise the one diagnostic
                // that explains the failed boot is swallowed by the failure itself.
                converterConf.warnUnknownKeys();
                throw;
            }
            trackerConf.warnUnknownKeys();
            converterConf.warnUnknownKeys();
        }
    } catch (const std::runtime_error &er) {
        ESP_LOGE("main", "error during sensor/converter/tracker setup: %s", er.what());
        adcSampler.adcStates.clear();
        g_app.setupErr = true;
        if (!noI2C) scan_i2c();
    }
}

// Default: install the GPIO ISR service on RT_CORE *before* any attachInterrupt() runs, so that
// arduino-esp32's lazy gpio_install_isr_service() call inside attachInterrupt is a no-op (already
// installed) and the INA226/ADS1115 alert handlers fire on RT_CORE. Otherwise the first
// attachInterrupt — invoked from setupSensors() on CPU0 — pins the GPIO ISR to CPU0, adding IPC
// latency variance to the RT sampler wake path (see [[project_ina226_timeout_loop_latency_shutdown]]).
// Set to 0 in CMake / a build flag to revert to the previous CPU0 behavior for A/B comparison.
#ifndef PIN_GPIO_ISR_TO_RT_CORE
#define PIN_GPIO_ISR_TO_RT_CORE 1
#endif

#if PIN_GPIO_ISR_TO_RT_CORE
struct GpioIsrInstallCtx {
    esp_err_t r;
    TaskHandle_t caller;
};

// gpio_install_isr_service() internally does its own esp_ipc_call_blocking() to the calling core
// (to esp_intr_alloc the dispatcher on that core). Running it from *inside* esp_ipc_call_blocking()
// makes RT_CORE's single ipc task wait on itself → permanent deadlock in setup() (observed: boot
// hang). So run it on a plain task pinned to RT_CORE instead — the ipc worker stays free, the nested
// IPC completes, and the GPIO ISR lands on RT_CORE.
static void installGpioIsrTask(void *arg) {
    auto *ctx = (GpioIsrInstallCtx *) arg;
    // flags=0, NOT ESP_INTR_FLAG_IRAM: attachInterrupt registers arduino-esp32's __onPinInterrupt
    // dispatcher, which lives in flash. An IRAM-installed service keeps firing while the flash cache
    // is disabled (any flash write — coulomb/stats persist, config save, OTA) and then jumps into
    // that cached handler → "Cache disabled but cached memory region accessed" panic. Non-IRAM masks
    // the alert for the brief cache-off window instead. RT_CORE affinity comes from this task, not
    // the flag.
    ctx->r = gpio_install_isr_service(0);
    xTaskNotifyGive(ctx->caller);
    vTaskDelete(nullptr);
}

static void pinGpioIsrToRtCore() {
    GpioIsrInstallCtx ctx{ESP_FAIL, xTaskGetCurrentTaskHandle()};
    TaskHandle_t t = nullptr;
    xTaskCreatePinnedToCore(installGpioIsrTask, "gpioIsrInit", 3072, &ctx, RT_PRIO, &t, RT_CORE);
    if (!t) {
        ESP_LOGE("main", "failed to spawn gpioIsrInit; RT alert pinning skipped");
        return;
    }
    if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000))) {
        ESP_LOGE("main", "gpioIsrInit timed out; RT alert pinning skipped");
        return;
    }
    if (ctx.r == ESP_OK)
        ESP_LOGI("main", "gpio_install_isr_service pinned to RT_CORE=%d", RT_CORE);
    else if (ctx.r == ESP_ERR_INVALID_STATE)
        ESP_LOGW("main", "gpio_install_isr_service already installed (likely on CPU0); RT alert pinning skipped");
    else
        ESP_LOGE("main", "gpio_install_isr_service(RT_CORE) failed: %s", esp_err_to_name(ctx.r));
}
#endif

// Register the optional non-RT subsystems as services and start the enabled ones. MQTT keeps its
// mppt/home-assistant wiring here (out of mqtt.cpp) via preStart, re-run on every start. Network
// services that aren't Running at boot self-heal on the WiFi-up edge in loopNetwork_task.
static void registerServices() {
#ifdef WITH_NETW
    MQTT.preStart = [](const ConfFile &mqttConf) {
        mppt.charger.beginMqtt(mqttConf);
        MQTT.onConnected = haMqttSendDiscovery;
    };
    // Periodic HA power publish, throttled inside MqttService::onTick (only ticks while Running).
    MQTT.tickHook = [] {
        if (!mppt.sensorPhysicalI || !mppt.sensorPhysicalU) return; // sensor setup failed, mppt.begin() skipped
        float pow = mppt.sensorPhysicalI->ewm.avg.get() * mppt.sensorPhysicalU->ewm.avg.get();
        haMqttUpdate({.power = mppt.isSweeping() ? NAN : pow});
    };
    g_services.registerService(&MQTT);
    g_services.registerService(&telemetryService);
    g_services.registerService(&ftpService);
    g_services.registerService(&telnetService);
#endif
    g_services.registerService(&lcdService);
#ifdef WITH_NETW
    g_services.registerService(&scopeService);
#endif
#ifdef WITH_BLE
    g_services.registerService(&bleConsoleService);
#endif
#ifdef HAVE_BSYNC
    g_services.registerService(&bsyncService); // no requiresNetwork: sniffs beacons w/o association
#endif
    g_services.startEnabledAtBoot(wifiIsConnected());
}

// A freshly-OTA'd image boots as ESP_OTA_IMG_PENDING_VERIFY (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE).
// setup() is otherwise unguarded — loopRT (and its WDT coupling) only exists after setup() returns,
// and the TWDT watches idle, which a yielding hang keeps feeding — so a one-shot esp_timer covers the
// whole setup() window: if setup() never finishes, reboot, and the bootloader reverts to the previous
// slot since this image was never confirmed.
static constexpr int BOOT_WATCHDOG_TIMEOUT_S = 30;
static constexpr uint32_t OTA_VALIDATE_UPTIME_MS = 20000;
static esp_timer_handle_t s_bootWdt = nullptr;

static void bootWatchdogFire(void *) {
    ESP_LOGE("main", "boot watchdog: setup() didn't finish in %ds — restarting (rollback if unconfirmed)",
             BOOT_WATCHDOG_TIMEOUT_S);
    esp_restart();
}

static void bootWatchdogArm() {
    const esp_timer_create_args_t args = {
        .callback = bootWatchdogFire,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "bootWdt",
        .skip_unhandled_events = false,
    };
    if (esp_timer_create(&args, &s_bootWdt) == ESP_OK)
        esp_timer_start_once(s_bootWdt, (uint64_t) BOOT_WATCHDOG_TIMEOUT_S * 1000000ULL);
    else
        ESP_LOGE("main", "failed to arm boot watchdog");
}

static void bootWatchdogDisarm() {
    if (!s_bootWdt) return;
    esp_timer_stop(s_bootWdt);
    esp_timer_delete(s_bootWdt);
    s_bootWdt = nullptr;
}

// Confirm a PENDING_VERIFY image once the RT loop is proven alive (sampling). Runs from loopLF on
// core 0 (one-shot otadata flash write). Non-pending state (normal flash) → cheap skip. Until this
// runs, a reset reverts to the previous slot — that's the rollback safety net.
static void lfMarkOtaValid() {
    static bool done = false;
    if (done || millis() < OTA_VALIDATE_UPTIME_MS) return;
    if (!timeLastSampler || (wallClockUs() - timeLastSampler) > 1000000ULL) return; // RT sampler alive?

    done = true;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
            ESP_LOGI("main", "OTA image confirmed valid (rollback cancelled)");
        else
            ESP_LOGE("main", "failed to confirm OTA image valid");
    }
}

void setup() {
    consoleInit();
    setupCli();
    ESP_LOGI("main", "*** %s", format_version());
    bootWatchdogArm();

    rtcount_test_cycle_counter();

    nvs.init();

    if (!mountLFS()) {
        ESP_LOGE("main", "Error mounting LittleFS partition!");
        g_app.setupErr = true;
    }

#ifdef WITH_SPROFILER
    try {
        ConfFile pprofConf{"/littlefs/conf/pprof.conf", true};

        auto sprofHz = (uint32_t) pprofConf.getLong("sprofiler_hz", 0); // 100~300
        if (sprofHz && esp_cpu_dbgr_is_attached()) {
            // only start the profiler with OpenOCD attached?
            ESP_LOGI("main", "starting sprofiler with freq %lu (samples/bank=%i)", sprofHz, PROFILING_ITEMS_PER_BANK);
            sprofiler_initialize(sprofHz);
        } else if (sprofHz) {
            ESP_LOGW("main", "sprofiler configured but not debugger attached");
        }
    } catch (const std::exception &e) {
        // a malformed pprof.conf must not brick the device — the profiler is purely diagnostic
        ESP_LOGE("main", "error reading pprof.conf, skipping profiler: %s", e.what());
    }
#endif // WITH_SPROFILER

    // A malformed board.conf must not abort setup() (which would reboot and re-read the same bad
    // file → boot loop). Fall back to an empty config and run the safe-idle path via g_app.setupErr.
    ConfFile boardConf = loadConfSafe("/littlefs/conf/board.conf");

    if (!boardConf) {
        if (DIR *d = opendir("/littlefs/conf")) {
            for (dirent *e; (e = readdir(d));)
                ESP_LOGI("main", "file: %s", e->d_name);
            closedir(d);
        }
    }

    auto mcuStr = boardConf.getString("mcu", "");
    if (mcuStr != CONFIG_IDF_TARGET) {
        ESP_LOGE("main", "board.conf expects MCU '%s', but target is '%s'", mcuStr.c_str(), CONFIG_IDF_TARGET);
        g_app.setupErr = true;
    }

    bool noI2C = true;
    setupI2C(boardConf, noI2C);

    TeleConf teleConf = setupNetworkAtBoot();

    Limits lim{};
    try {
        ConfFile limitsConf{"/littlefs/conf/limits.conf"};
        lim = Limits{limitsConf};
        limitsConf.warnUnknownKeys();
    } catch (const std::runtime_error &er) {
        logConfErr("limits.conf", er);
        g_app.setupErr = true;
    }

#if PIN_GPIO_ISR_TO_RT_CORE
    // Must run before setupConverterAndMppt → setupSensors → ADC_INA226/ADS::init → attachInterrupt.
    pinGpioIsrToRtCore();
#endif

    setupConverterAndMppt(boardConf, lim, teleConf, noI2C);

    if (!g_app.setupErr) {
        xTaskCreatePinnedToCore(loopRT, "loopRt", 4096 * 4, NULL, RT_PRIO, NULL, RT_CORE);
    } else {
        led.setHexShort(0x200);
    }

    registerServices();

    // esp_log -> vprintf_mux (telnet/MQTT/boot-log sinks). Kept LATE on purpose: hooking it before
    // WiFi connects routes the wifi task's connect-time logging burst through vprintf_mux's 300B
    // stack buffer, overflowing the 3072B wifi task stack (this bricked flat). So the boot-log
    // backlog captures from here on, not the setup() body — accept that over the brick.
#ifdef WITH_NETW
    enable_esp_log_to_telnet();
#endif

#if CONFIG_HEAP_POISONING_COMPREHENSIVE
    ESP_LOGW("main", "HEAP_POISONING=COMPREHENSIVE: every alloc has head/tail canaries + fill, expect 5-10%% perf hit and ~16B/alloc RAM cost. Debug-only.");
#elif CONFIG_HEAP_POISONING_LIGHT
    ESP_LOGW("main", "HEAP_POISONING=LIGHT: every alloc has tail canary, free() asserts on overrun. Debug-only — revert sdkconfig when done.");
#endif

    bootWatchdogDisarm();
    ESP_LOGI("main", "setup() done.");

    /*g_app.manualPwm = true;
    adcSampler.cancelCalibration();
    pwm.pwmPerturb(1327); // this can destroy the BF switch
    pwm.enableLowSide(true);
    mppt.bflow.enable(true); */
}


void loop() {
    // use arduino loop for networking (non-critical stuff)
    loopNetwork_task(nullptr);
}

static esp_err_t disable_cpu_power_saving(void) {
    esp_err_t ret = ESP_OK;

#ifdef CONFIG_PM_ENABLE
    static esp_pm_lock_handle_t s_cli_pm_lock = nullptr;

    ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "noPwrSave", &s_cli_pm_lock);
    if (ret == ESP_OK) {
        ESP_ERROR_CHECK(esp_pm_lock_acquire(s_cli_pm_lock));
        ESP_LOGI("main", "Successfully created CLI pm lock");
    } else {
        if (s_cli_pm_lock != NULL) {
            esp_pm_lock_delete(s_cli_pm_lock);
            s_cli_pm_lock = NULL;
        }
        ESP_LOGW("main", "Failed to create CLI pm lock: %s", esp_err_to_name(ret));
    }
#endif
    return ret;
}

void stopAndBackoff(uint32_t secondsDelay) {
    if (!converter.disabled())
        mppt.shutdownDcdc("stopAndBackoff");
    if (g_app.psuMode() && secondsDelay <= 5)
        delayStartUntil = wallClockUs() + 100000ULL;
    else
        delayStartUntil = wallClockUs() + static_cast<time_us>(secondsDelay) * 1000000ULL;
}

static void loopRT(void *arg) {
    // low-latency control loop task

    try {
        adcSampler.begin();
    } catch (const std::runtime_error &er) {
        ESP_LOGE("main", "error starting ADC sampler: %s", er.what());
        while (true) {
            loopWallClockUs_ = esp_timer_get_time();
            vTaskDelay(10);
        }
    }

    disable_cpu_power_saving();

    assert(xPortGetCoreID() == RT_CORE);
    vTaskPrioritySet(nullptr, 20); // highest priority (24)
    // ^ see https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/api-guides/performance/speed.html#choosing-task-priorities-of-the-application

    ESP_LOGD("main", "Loop running on core %i", (int) xPortGetCoreID());

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    ESP_LOGW("main", "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS enabled!");
    delay(1000);
#endif

    // enable log defer just before starting the RT loop
    // this way we instantly know about things going wrong during start-up
    loggingEnableDefer();

    while (true) {
        rtcount("start");
        loopWallClockUs_ = esp_timer_get_time();
        rtcount("micros");
        auto &nowUs(loopWallClockUs_);

        time_ms nowMs = nowUs / 1000ULL;

        rtcount("adc.update.pre");
        auto samplerRet = adcSampler.update();
        rtcount("adc.update");

        if (adcSampler.halted) continue;

        if (samplerRet == ADC_Sampler::UpdateRet::CalibFailure)
            stopAndBackoff(4);

        // ADC stall watchdog. AdcError = the continuous-ADC isGood() tripped (DMA stalled); a stale
        // timeLastSampler = the sampler is alive but no fresh sample reached the controller. Either way
        // we restart the ADC promptly. The converter is stopped only if the stall PERSISTS past
        // kAdcSustainedMs — a transient wedge (e.g. a console uxTaskGetSystemState() briefly starving
        // the DMA) recovers on one reset and must NOT trip a backoff on a live converter. Calibration
        // legitimately withholds fresh samples, so it's excluded from the timeLastSampler arm.
        constexpr time_ms kAdcResetThrottleMs = 300, kAdcSustainedMs = 800, kAdcRestartMs = 60000;
        static time_ms adcStalledSinceMs = 0, lastAdcResetMs = 0;

        bool adcStalled = samplerRet == ADC_Sampler::UpdateRet::AdcError ||
                          (samplerRet != ADC_Sampler::UpdateRet::NewData && !adcSampler.isCalibrating() &&
                           timeLastSampler && nowUs - timeLastSampler > 200000);

        if (adcStalled) {
            if (!adcStalledSinceMs) adcStalledSinceMs = nowMs;
            if (nowMs - lastAdcResetMs > kAdcResetThrottleMs) { // restart a wedged DMA quickly
                lastAdcResetMs = nowMs;
                adcSampler.resetPeripherals();
            }
            if (nowMs - adcStalledSinceMs > kAdcSustainedMs && !converter.disabled() && !mppt.inBackoff()) {
                ESP_LOGE("main", "ADC stall %llu ms, shutdown (nSamples=%lu)", nowMs - adcStalledSinceMs,
                         lastMpptUpdateNumSamples);
                stopAndBackoff(16);
            }
            if (nowMs - adcStalledSinceMs > kAdcRestartMs) systemRestart();
        } else {
            adcStalledSinceMs = 0;
        }

        if (samplerRet != ADC_Sampler::UpdateRet::NewData) {
            // Do not make console PSU shutdown/override depend on a fresh ADC sample. A wedged
            // sampler is exactly when `psu off` must still be able to transfer ownership to the
            // RT core and disable/ramp the converter.
            mppt.applyPendingPsuCommandRt(false);
            if (adcSampler.isCalibrating() && mppt.boardPowerSupplyUnderVoltage()) {
                ESP_LOGW("main", "Board power supply UV %.2f!", mppt.boardPowerSupplyVoltage());
                adcSampler.cancelCalibration();
            }

            if (!timeLastSampler and nowMs > 20000) {
                converter.disable();
                ESP_LOGE("main", "Never got a sample! Please check ADC");
                if (nowMs > (1000 * 60 * 15)) {
                    systemRestart();
                }
                delay(5000);
            }
        } else {
            loopRTNewData(nowMs);
            rtcount("loopRTNewData");
        }


        auto lag = nowUs - lastLoopTime;
        if (lastLoopTime && lag > g_app.maxLoopLag && !converter.disabled()) g_app.maxLoopLag = lag;
        lastLoopTime = nowUs;

#if CAPTURE_LOOP_DT
        auto now2 = micros();
        auto loopDT = now2 - nowUs;
        if (loopDT > g_app.maxLoopDT && !pwm.disabled())
            g_app.maxLoopDT = loopDT;
#endif

        // Not need to yield or call vTaskDelay here
        // waiting for adc values does the necessary blocking

        //vTaskDelay(0); // this resets the Watchdog Timer (WDT) for some reason
        //vTaskDelay(1);
        //yield();
    }
}

static std::string mpptStateStr() {
    auto st = mppt.getState();
    std::string arrow;
    if (st == MpptControlMode::MPPT) {
        if (mppt.tracker.slowMode) {
            arrow = mppt.tracker._direction ? "⇡" : "⇣";
        } else {
            arrow = mppt.tracker._direction ? "↑" : "↓";
        }
    }

    mppt.limIdxSampled.reset();

    return arrow + MpptState2String[(uint8_t) st];
}

// Shut down WiFi when chip temperature gets dangerous. BLE on ESP32-S3 ran the chip hot enough
// to need this even before the modem-sleep fix; the temperature ceiling here is a last resort.
static void wifiShutdownIfHot(float chipTempC) {
#ifdef WITH_NETW
    if (chipTempC > 95 && wifiIsConnected()) {
        ESP_LOGW("main", "High chip temperature, shut-down WiFi");
        flush_async_uart_log();
        vTaskDelay(pdMS_TO_TICKS(200));
        wifiHardOff();
    }
#endif
}

// Latency watchdog. Fires when the RT loop is producing fewer samples per second than required
// (slow ADC, blocked path, etc.). Requires the starvation to persist across TRIP_WINDOWS consecutive
// lfPeriod (~3s) windows before backing off, so a one-off core-0 stall — e.g. console-command
// log-alloc contention (a discovery/health poller sending ip/uptime tripped this on every poll, see
// doc/dev-notes/Real-Time Latency.md) — can't trigger stopAndBackoff; only sustained starvation does.
// Per-sample OV/OC cutouts (mppt.protect) are independent and still fire every arriving sample.
static void lfWatchdog(time_us nowUs, uint32_t dt, uint32_t sps, uint32_t nSamples) {
    static uint8_t starvedWindows = 0;
    constexpr uint8_t TRIP_WINDOWS = 3;

    bool starved = (dt > (lfPeriod * 0.9f)) && sps < g_app.loopRateMin && !converter.disabled() &&
                   nSamples > max(g_app.loopRateMin * 5, 200) &&
                   !g_app.manualPwm() && lastTimeOutUs && (nowUs - adcSampler.getTimeLastCalibrationUs()) > 2000000;
    if (!starved) {
        starvedWindows = 0;
        return;
    }
    if (++starvedWindows < TRIP_WINDOWS) {
        ESP_LOGW("main", "Loop latency high (%lu<%hu Hz), window %d/%d", sps, g_app.loopRateMin,
                 (int) starvedWindows, (int) TRIP_WINDOWS);
        return;
    }
    starvedWindows = 0;
    auto loopRunTime = (nowUs - adcSampler.getTimeLastCalibrationUs());
    ESP_LOGE("main", "Loop latency high (%lu<%hu Hz), shutdown! (nSamples=%lu D=%u rt=%.1fs)",
             sps, g_app.loopRateMin, nSamples, converter.getCtrlOnPwmCnt(), loopRunTime * 1e-6f);
    stopAndBackoff(4);
}

// Stuck-at-floor watchdog. flat intermittently backs off to the PWM floor in CV and never climbs
// out despite full input headroom (Vin>>Vout) and ~0 output power — a pre-existing fault (seen on
// the known-good image; fry on the same pack never hits it; root cause still open). A clean reboot
// reliably recovers it (a re-sweep does not — the limiting controller re-pins the floor). So reboot
// if the floor+headroom+no-power condition persists. Gated against night/sweep/calibration/manual
// and brief start transients via the sustained timeout.
// TODO(remove): this whole watchdog is a stopgap for an unresolved CV-floor/limit lockup. Once the
// root cause is fixed it should go — it has no place rebooting a healthy converter. The 2h timeout
// keeps it from firing on the normal low-power dawn ramp (Vin>>Vout, Iout<0.3A is legit then).
// Uses esp_timer_get_time() (monotonic 64-bit µs) directly — avoids the instrumented
// wallClockUs() overhead and keeps the watchdog independent of the RT loop.
static void lfStuckWatchdog() {
    static int64_t stuckSinceUs = 0;
    static bool triedRelease = false;
    constexpr int64_t RELEASE_US = 30ll * 1000000ll;          // try in-place latch release first
    constexpr int64_t TIMEOUT_US = 2ll * 3600ll * 1000000ll;  // then reboot (e.g. limit corruption)
    const int64_t nowUs = esp_timer_get_time();

    // "Dead in clear sun": near-zero output despite ample input headroom. Covers all observed
    // lockups — the CV-floor latch (shared-pack EOC feedback), the Vin-OV/limit-corruption backoff
    // loop (converter cycling, not steadily at the floor), and stalled sweeps. The authority gate
    // should now prevent the CV-floor latch; this stays a backstop. Gated against night (no
    // headroom), calibration, manual PWM and brief transients (sustained timeout).
    bool headroom = sensors.Vin && sensors.Vout
                    && (converter.boost()
                            ? sensors.Vin->ewm.avg.get() > mppt.limits.Vin_min + 2.0f
                            : sensors.Vin->ewm.avg.get() > sensors.Vout->ewm.avg.get() + 8.0f);
    bool noPower = sensors.Iout && fabsf(sensors.Iout->ewm.avg.get()) < 0.3f;
    // NOTE: do not gate on !adcSampler.isCalibrating() — a corruption/Vin-OV loop repeatedly
    // re-runs ADC calibration (resetPeripherals), which would keep resetting the stuck-timer and
    // defeat this watchdog. A legitimate calibration is far shorter than TIMEOUT_US, so the
    // sustained timer already excludes it.
    const float psuSetpoint = mppt.getPsuSetpoint();
    bool stuck = headroom && noPower && !g_app.manualPwm()
                 && (g_app.psuMode()
                         ? (std::isfinite(psuSetpoint)
                            && sensors.Vout->ewm.avg.get() < psuSetpoint - 2.0f)
                         : !bool(mppt.charger.termCond));

    if (!stuck) { stuckSinceUs = 0; triedRelease = false; return; }
    if (!stuckSinceUs) { stuckSinceUs = nowUs; return; }

    // Stage 1: in-place CV-floor latch release (no reboot). If it cleared a latch, let MPPT climb.
    if (!triedRelease && (nowUs - stuckSinceUs) >= RELEASE_US) {
        triedRelease = true;
        if (mppt.releaseCvFloorLatch("stuck watchdog")) {
            ESP_LOGW("main", "stuck watchdog: released CV-floor latch in place, retrying");
            return;
        }
    }

    // Stage 2: release didn't help (or nothing to release) → reboot (clears limit corruption etc).
    if ((nowUs - stuckSinceUs) < TIMEOUT_US) return;

    ESP_LOGE("main", "No output %us despite headroom (D=%u Vin=%.1f Vout=%.1f Iout=%.2f), restarting",
             (unsigned) ((nowUs - stuckSinceUs) / 1000000ul), converter.getCtrlOnPwmCnt(),
             sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(), sensors.Iout->ewm.avg.get());
    vTaskDelay(pdMS_TO_TICKS(200)); // let the log reach MQTT/telnet before the reboot
    esp_restart();
}

// Low-frequency control reads (RT-adjacent, not in the ADC fast path): NTC + chip temp,
// charger termination state, and the thermal-cap WiFi cutoff.
static void lfControl() {
#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    g_app.usbConnected = usb_serial_jtag_is_connected();
#endif
    mppt.ntc.read();
    mppt.ucTemp.read();
    if (sensors.Vout && sensors.Iout) {
        const float vout = sensors.Vout->ewm.avg.get();
        const float iout = sensors.Iout->ewm.avg.get();
        // EOC vpack_pin feedback must only run when this converter actually drives Vout. On a shared
        // bus held up by another charger, a floored converter has no authority and would otherwise
        // latch vpack_pin down (the stuck-at-floor lockup). Require real duty + output current.
        uint16_t pwmMargin = mppt.converter.pwmCtrlMax / 256;
        if (pwmMargin < 8) pwmMargin = 8;
        const uint16_t minAuthorityPwm = mppt.converter.getCtrlOnPwmMin() + pwmMargin;
        const bool currentShowsAuthority = std::isfinite(iout) && (iout > 0.5f || bool(mppt.charger.termCond));
        const bool voutAuthority = !mppt.converter.disabled()
                                   && mppt.converter.getCtrlOnPwmCnt() > minAuthorityPwm
                                   && currentShowsAuthority;
        mppt.charger.update(vout, iout, voutAuthority);
    }
    wifiShutdownIfHot(mppt.ucTemp.last());
}

// One-line UART/MQTT/telnet status line. WITH_MEASURE_COIL skips it during the coil sweep so the
// measurement output isn't intermixed with the status print.
static void lfStatusLine(uint32_t nSamples, uint32_t sps, uint32_t dt) {
#ifdef WITH_MEASURE_COIL
    if (!sensors.Vin || isMeasuring()) return;
#else
    if (!sensors.Vin) return;
#endif
    UART_LOG(
        "V=%4.*f/%5.*f I=%4.*f/%5.*fA %5.1fW %.0f℃%.0f℃ %2lusps %2lu㎅/s %s(H|L|Lm)=%4hu|%4hu|%4hu"
        " st=%5s,%i lag=%lu㎲ N=%lu rssi=%hi",
        sensors.Vin->last >= 9.55f ? 1 : 2, sensors.Vin->last,
        sensors.Vout->last >= 9.55f ? 2 : 2, sensors.Vout->last,
        sensors.Iin->ewm.avg.get() >= 9.55f ? 1 : 2, sensors.Iin->ewm.avg.get(),
        sensors.Iout->ewm.avg.get() >= 9.55f ? 2 : 2, sensors.Iout->ewm.avg.get(),
        sensors.Vin->ewm.avg.get() * sensors.Iin->ewm.avg.get(),
        mppt.ntc.last(), mppt.ucTemp.last(),
        sps,
        dt ? (uint32_t) (bytesSent * 1000llu / dt) : 0,
        converter.inDCM() ? "DCM" : "CCM",
        converter.getCtrlOnPwmCnt(), converter.getRectOnPwmCnt(), converter.getRectOnPwmMax(),
        g_app.manualPwm()
            ? "MANU"
            : (g_app.psuMode()
                   ? "PSU"
                   : (mppt.converter.disabled() && !mppt.startCondition()
                          ? (mppt.boardPowerSupplyUnderVoltage() ? "UV" : "START")
                          : mpptStateStr().c_str())),
        (int) mppt.active(),
        g_app.maxLoopLag,
        nSamples,
        (int16_t) wifiRssi()
    );

    // While idle in START, name the startCondition() clause that's blocking the start
    // (throttled: on-change, else every 30s). Diagnoses stuck pre-dawn starts from the log.
    if (mppt.converter.disabled() && !g_app.manualPwm() && !g_app.psuMode()) {
        const char *reason = mppt.startBlockReason();
        static const char *lastReason = nullptr;
        static time_us lastLogUs = 0;
        auto nowUs = wallClockUs();
        if (reason && (reason != lastReason || nowUs - lastLogUs > 30000000)) {
            ESP_LOGI("mppt", "START blocked: %s (Vin=%.1f Vout=%.1f ntc=%.0f mcu=%.0f)", reason,
                     sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(),
                     mppt.ntc.last(), mppt.ucTemp.last());
            lastReason = reason;
            lastLogUs = nowUs;
        }
    }
}

// RGB LED color from current converter state (manual / idle / sweep / MPPT / CV / topping).
static void lfUpdateLed(time_us nowUs) {
    if (g_app.manualPwm()) {
        uint8_t i = constrain((sensors.Vout->last * sensors.Iout->last) / mppt.limits.P_max * 255, 1, 255);
        led.setRGB(0, i, i);
        return;
    }
    if (mppt.converter.disabled()) {
        if (g_app.setupErr) { led.setHexShort(0x600); return; }
        if (!sensors.Vin) return;
        if (mppt.boardPowerSupplyUnderVoltage(true)) { led.setHexShort(0x100); return; }
        if (nowUs > 60000000ULL * 15) led.setHexShort(0x000); // turn off light at night (protect insects)
        else led.setHexShort(sensors.Vout->last > sensors.Vin->last ? 0x100 : 0x300);
        return;
    }
    switch (mppt.getState()) {
        case MpptControlMode::Sweep: led.setHexShort(0x303); break; // purple
        case MpptControlMode::MPPT:
            led.setHexShort(sensors.Iout->ewm.avg.get() > 0.2f ? 0x230 : 0x111);
            break;
        case MpptControlMode::CV: led.setHexShort(0x033); break;
        default: led.setHexShort(0x310); break; // CC/CP/topping
    }
}

void loopLF(const time_us &nowUs, bool interim) {
    auto &nSamples(sensors.Vout ? sensors.Vout->numSamples : lastNSamples);
    // The sps/throughput window keeps its OWN baseline: lastTimeOutUs is shared cadence state
    // that other code legitimately stomps (console.cpp pushes it per input byte to pause logs,
    // telnet onConnect zeroes it to force output) — deriving the rate from it produced the
    // notorious bogus "0sps" (and fed lfWatchdog a corrupted rate).
    static time_us lastWindowUs = 0;
    static uint32_t lastSps = 0;
    auto dt = nowUs - lastWindowUs;

    // Console commands print an immediate status line (cli.cpp): a read-only snapshot with the
    // last full-window rate — it must not consume the window.
    if (interim) {
        lfStatusLine(nSamples, lastSps, dt);
        return;
    }

    uint32_t sps = (dt > 20000) ? (uint64_t) (nSamples - lastNSamples) * 1000000llu / dt : lastSps;
    lastSps = sps;

    lfWatchdog(nowUs, dt, sps, nSamples);
    lfStuckWatchdog();
    lfControl();
    lfMarkOtaValid();
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    coredumpStampIfNew();
#endif

    // converter.init() logs L0/rect_offset_ns during setup(), before the MQTT/telnet log sinks exist,
    // so it only reaches the boot serial console. Re-emit it once the MQTT sink is up so the config
    // (e.g. rect_offset_ns) is visible in the remote log after a boot.
    static bool loggedConvCfg = false;
    if (!loggedConvCfg) {
        auto *mq = g_services.findByName("mqtt");
        if (mq && mq->state() == ServiceState::Running) { loggedConvCfg = true; converter.logConfig(); }
    }

    lfStatusLine(nSamples, sps, dt);

    lastNSamples = nSamples;
    lastWindowUs = nowUs;
    bytesSent = 0;

    if (mppt.converter.disabled())
        mppt.meter.update(); // always update the meter

    lfUpdateLed(nowUs);
}

static void loopRTNewData(time_ms nowMs) {
    // cap control update rate to sensor sampling rate (see below). rate for all 3 sensors are equal.
    // we choose Vout here because this is the most critical control value (react fast to prevent OV)
    auto nSamples = sensors.Vout->numSamples;

    bool haveNewSample = (nSamples - lastMpptUpdateNumSamples) > 0;

    // Enabling PSU mode needs this fresh Vin sample. Non-enabling commands may also arrive here
    // and share the same RT-owned transition path.
    mppt.applyPendingPsuCommandRt(haveNewSample);

    if (haveNewSample)
        timeLastSampler = wallClockUs();

    // auto range current sense ADC channel TODO add hysteresis
    /* float adcRangeBound = 2.0f;
    adcSampler.adc->setMaxExpectedVoltage(
            sensors.Iin->adcCh,
            (std::max(sensors.Iin->last, sensors.Iin->ewm.avg.get()) < sensors.Iin->transform.apply(adcRangeBound))
            ? adcRangeBound
            : sensors.Iin->transform.apply_inverse(30.f)
    ); */


    if (unlikely(adcSampler.isCalibrating())) {
        mppt.shutdownDcdc("calib", 0); // calibration must resume MPPT immediately on completion
    } else {
        if (mppt.active() or g_app.manualPwm()) {
            rtcount("protect.pre");
            bool mppt_ok = true;
            if (!mppt.converter.disabled()) {
                mppt_ok &= mppt.protect(g_app.manualPwm());
                rtcount("protect");
                mppt_ok &= mppt.protectLf(g_app.manualPwm());
                rtcount("protectLf");
            }
            if (mppt_ok) {
                if (haveNewSample) {
                    if (!g_app.manualPwm()) {
                        rtcount("mppt.update.pre");
                        mppt.update();
                        rtcount("mppt.update");
                    } else {
                        mppt.updateManual();
                    }
                    lastMpptUpdateNumSamples = nSamples;
                }
            } else {
                stopAndBackoff(4);
            }
        } else if (wallClockUs() > delayStartUntil && mppt.startCondition()) {
            if (g_app.psuMode()) {
                mppt.bflow.enable(true);
                converter.pwmPerturb(1);
                delayStartUntil = wallClockUs() + 4 * 1000000ULL;
                ESP_LOGI("mppt", "PSU: arming converter from cold start");
            } else if (!g_app.manualPwm()) {
                // Don't open-loop sweep into a possibly-full pack: a sweep ramps duty 0->max and dumps a
                // charge pulse (the recharge-after-reboot we saw on repeated OTAs). Skip while terminated.
                // Right after boot termCond isn't known until the first BMS cell frame arrives, so if a BMS
                // source is configured but silent, wait briefly for it; if it never comes, sweep anyway
                // (the charger then holds Vbat_fallback float, so there's no real overcharge).
                static int64_t bmsBootDeadline = 0;
                static bool bmsWasFresh = false;
                constexpr int64_t BMS_BOOT_WAIT_US = 12'000'000;
                bool bmsFresh = mppt.charger.batSt.haveValidCellVoltage();
                // A stale BMS cell frame can leave termCond stuck true. Don't trust it: only treat
                // the pack as full when termCond is true AND the BMS data is fresh.
                bool full = bool(mppt.charger.termCond) && bmsFresh;
                // Wait for termCond to actually be evaluated (cell voltage AND warm ibat), not just
                // for the first cell frame — otherwise the sweep could fire in the gap before ibat
                // smoothing warms up and termination latches. Also wait if the BMS just went stale,
                // so a brief outage doesn't immediately allow a sweep into a possibly-full pack.
                bool waitingForBms = mppt.charger.hasBmsCellSource()
                                     && (!mppt.charger.terminationDecided() || !bmsFresh);
                if (bmsWasFresh && !bmsFresh)
                    bmsBootDeadline = 0; // reset the wait window on fresh -> stale transition
                bmsWasFresh = bmsFresh;
                if (waitingForBms && bmsBootDeadline == 0)
                    bmsBootDeadline = esp_timer_get_time() + BMS_BOOT_WAIT_US;
                bool bmsWaitElapsed = bmsBootDeadline != 0 && esp_timer_get_time() > bmsBootDeadline;
                if (!full && (!waitingForBms || bmsWaitElapsed)) {
                    rtcount("mppt.startSweep.pre");
                    mppt.startSweep();
                    rtcount("mppt.startSweep");
                    delayStartUntil = wallClockUs() + 4 * 1000000ULL;
                }
            }
        }

        if (scope && sensors.Vout && haveNewSample)
            scope->addSample12(&mppt, 0,
                               (uint16_t) max(0.f, sensors.Vout->last / 60.0f *
                                                   2000.0f));
    }


    if (g_app.manualPwm()) {
        if (!converter.disabled())
            converter.pwmPerturb(0); // this will increase LS duty cycle if possible
    }
}


// Per-tick WiFi + network-service work for loopNetwork_task. Handles auto-reenable timer, the
// temp-gated wifiLoop, and self-healing network services on the WiFi-up edge.
static void networkLoopTick() {
#ifdef WITH_NETW
    if (g_app.disableWifi && g_app.wifiReenableMs && (int32_t) (wallClockMs() - g_app.wifiReenableMs) >= 0) {
        g_app.wifiReenableMs = 0;
        g_app.disableWifi = false;
        UART_LOG("WiFi re-enabled after timeout");
        connect_wifi_async();
    }

    // WiFi-down edge: `wifi off` only sets disableWifi (from a console/telnet/MQTT input callback);
    // do the actual teardown HERE, outside that callback. Stop network services while the netif is
    // still valid, then drop WiFi — deiniting lwip from inside telnet.loop() freed pbufs under the
    // log mirror (a use-after-free). Latched so it runs once per down edge.
    static bool wifiTorndown = false;
    if (g_app.disableWifi) {
        if (!wifiTorndown) {
            g_services.stopNetworkServices();
            disconnect_wifi(true);
            wifiTorndown = true;
        }
    } else if (wifiTorndown) {
        wifiTorndown = false;
    }

    if (!g_app.disableWifi) {
        /* RF/PLL bring-up adds only ~3ms RT-loop lag (measured bench + fry/flat), well under the
         * watchdog, so connect regardless of conversion; still gate on uC temp.
         */
        bool converting = !converter.disabled() && mppt.tracker._curPower >= 10;
        wifiLoop(mppt.ucTemp.last() < 80);

        // self-heal: bring up enabled network services on the WiFi-up edge (they fail to start
        // at boot when WiFi isn't connected yet). _wifiConnected() has set up MDNS by now.
        static bool wifiWasUp = false;
        static int8_t psMode = -1; // -1 unset, 0 MIN_MODEM, 1 MAX_MODEM
        bool wifiUp = wifiIsConnected();
        if (wifiUp && !wifiWasUp) { g_services.startEnabledNetworkServices(); psMode = -1; } // connect reset PS to MIN
        wifiWasUp = wifiUp;

        // Night = not converting for a while: drop to MAX_MODEM to cut standby battery draw. Back to
        // MIN_MODEM the instant PV power returns so daytime console/MQTT/OTA stay responsive.
        static time_ms lastActiveMs = 0;
        time_ms nowMs = wallClockMs();
        if (converting) lastActiveMs = nowMs;
        int8_t want = (nowMs - lastActiveMs > 5u * 60 * 1000) ? 1 : 0;
        if (wifiUp && want != psMode) {
            psMode = want;
            wifiSetPowerSave(want);
            UART_LOG("WiFi power save: %s", want ? "MAX_MODEM (night/idle)" : "MIN_MODEM (active)");
        }
    }
#endif
}

// True while a TCP scope client is attached and its blocking netLoop() will provide the cooperative
// yield this iteration; otherwise loopNetwork_task must yield explicitly.
static bool scopeKeepsAwake() {
#ifdef WITH_NETW
    return scopeService.state() == ServiceState::Running && scopeService.hasClient();
#else
    return false;
#endif
}

static void loopNetwork_task(void *arg) {
    //ESP_LOGI("main", "Net loop running on core %i", xPortGetCoreID());
    assert(xPortGetCoreID() == 0);

    auto nowMs(wallClockMs());

    loopUart(nowMs);
    flush_async_uart_log();
    process_queued_tasks();

    networkLoopTick();

    // ftp / telnet / telemetry / lcd / scope ticks (only the Running ones do work)
    g_services.tickAll();


    if ((wallClockUs() - lastTimeOutUs) >= (mppt.converter.disabled() ? (lfPeriod * 8) : lfPeriod) or !lastTimeOutUs) {
        loopLF(wallClockUs(), false);
        // HA power publish moved to MqttService::onTick (MQTT.tickHook, wired in setup()).
        lastTimeOutUs = wallClockUs();
    }

    // Preserve the cooperative yield: scope's netLoop() blocks ~1 tick when a client is attached
    // and serves as the yield; otherwise we must yield explicitly.
    if (!scopeKeepsAwake())
        vTaskDelay(pdMS_TO_TICKS(1));
}

// Give the telnet client a chance to drain a FIN before we wipe the stack. stopAll() would slam
// the socket shut, so do this ahead of it.
static void restartCloseTelnet() {
#ifdef WITH_NETW
    telnetService.beginClose();
    for (int i = 0; i < 200 && telnetService.closePending(); ++i) delay(10);
#endif
}

void systemRestart() {
    converter.disable();
    UART_LOG("Rebooting");
    restartCloseTelnet();
    g_services.stopAll(); // tear down enabled services while WiFi is still up (see mqtt_task overflow)
    delay(300);
    ESP.restart();
}


[[maybe_unused]] void esp_task_wdt_isr_user_handler() {
    //throw std::runtime_error("reboot");
    if (esp_cpu_dbgr_is_attached()) return;

    enqueue_task([] {
        ESP_LOGE("main", "Restart after WDT trigger");
        systemRestart();
    });
}
