
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
#include "tele/tele_mqtt.h"
#if WITH_FTP
#include "tele/ftp_service.h"
#endif
#if WITH_HTTP_CONF
#include "tele/http_conf_service.h"
#endif
#include "tele/telnet_service.h"
#include "tele/telemetry_service.h"
#if WITH_HA
#include "tele/home_assistant.h"
#endif
#include "etc/ota.h"
#endif
#if WITH_SCOPE
#include "tele/scope_service.h"
#endif
#include "sync/bsync.h"
#ifdef WITH_USB_MSC
#include "usb/usb_dev.h"
#endif
#ifdef WITH_MEASURE_COIL
#include "selftest/measure_coil.h"
#endif
#include "selftest/bflow_test.h"

#include "etc/version.h"

#include "etc/perf.h"
#ifdef WITH_SPROFILER
#include <sprofiler.h>
#endif

#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED

#include <hal/usb_serial_jtag_ll.h>

#endif

#include "storage/key-value.h"
#include "tele/alert.h"
#include "conf_hash.h"
#include "app_state.h"

#include <esp_task_wdt.h>
#include <esp_pm.h>
#include <esp_timer.h>
#include <esp_ota_ops.h>
#ifdef CONFIG_BOOTGUARD_ENABLE
#include "bootguard.h"
#endif
#include <driver/gpio.h>
#include <dirent.h>

#include "etc/rt_core_check.h"
#include "etc/rt_task.h"
#include "etc/flash_probe.h"
#include "etc/heap_diag.h"
#include "etc/net_guard.h"
#include <ota_ble.h>
#if WITH_RTPROF
#include "etc/rtprof.h"
#endif
#if CONFIG_FUGU_WITH_PWR_KNOBS
#include "etc/pwr_knobs.h"
#endif
#include "etc/idle_profile.h"
#include <soc/rtc.h>
#include <fugu_flash_chip.h>


ADC_Sampler adcSampler{}; // schedules async ADC reading
VIinVout<const Sensor *> sensors{nullptr, nullptr, nullptr, nullptr};
SynchronousConverter converter; // buck or boost
static void IRAM_ATTR hwOvIsr() { converter.isrForceOff(); }
LedIndicator led;
MpptController mppt{adcSampler, sensors, converter, lcdService.lcd}; // lcd owned by lcdService
#ifdef WITH_TELE_MQTT
void teleProducePoint() { mppt.telemetry(); }
#endif

time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;

time_us lastLoopTime = 0;

time_us timeLastSampler = 0;

time_us delayStartUntil = 0;

const auto lfPeriod = 3000000ULL; //(mppt.tracker.avgPower.get() < 1) ? 3000000 : 3000000;

time_us lastTimeOutUs = 0;
// Deadline for the low-frequency CONTROL work. Deliberately separate from lastTimeOutUs, which is
// console print-pacing state that every input byte pushes forward (console.cpp:43): sharing them
// let a host polling faster than lfPeriod postpone lfControl()/the watchdogs/lfMarkOtaValid()
// indefinitely, and publish the frozen NTC/die temperature as a current reading (issue #65).
time_us lastLfWorkUs = 0;
uint32_t lastNSamples = 0;

unsigned long lastMpptUpdateNumSamples = 0;


AppState g_app{}; // mode flags / per-iteration max-lag bookkeeping (see app_state.h)

KeyValueStorage nvs{};

void systemRestart();

static void loopNetwork_task(void *arg);

static void loopRT(void *arg); // this is the critical one

static void loopRTNewData(time_ms nowMs);
#if WITH_RTPROF
volatile uint8_t g_rtLogTest = 0;
#endif

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
            bool wireOk = false; // I2C interrupt on the RT core, see setupSensors()
            runOnCore(RT_CORE, [&] { wireOk = Wire.begin(i2c_sda, (uint8_t) boardConf.getLong("i2c_scl"), i2c_freq); });
            if (!wireOk) {
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

static RTC_NOINIT_ATTR ChargerRtc s_chargerRtc;

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
            mppt.charger.attachRtc(&s_chargerRtc);
            converter.init(converterConf, boardConf, coilConf);
            g_hwOvIsrHook = hwOvIsr;
#ifndef HAVE_MCPWM
            if (boardConf.getFloat("ina22x_ov", 0) > 0)
                ESP_LOGW("main", "ina22x_ov: gate driver is not MCPWM, the alert ISR cannot force the gates off");
#endif
            mppt.rectCal.autoRun = coilConf.getByte("rect_cal_auto", 1);
            mppt.rectCal.persist = rectCalPersist;
            mppt.rectCal.init(converter.getRectOnOffset(), !converter.rectOffsetNeedsCal());
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
    // ESP_INTR_FLAG_IRAM keeps the ADC alerts (and the INA226 OV trip) live while the flash cache is
    // off. Only where rt_iram_extra.lf puts arduino-esp32's __onPinInterrupt dispatcher in IRAM;
    // elsewhere it is in flash and an IRAM service would panic ("Cache disabled but cached memory
    // region accessed"), so the alert is masked during flash writes there. RT_CORE affinity comes
    // from this task, not the flag.
#if FUGU_GPIO_ISR_IRAM
    ctx->r = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
#else
    ctx->r = gpio_install_isr_service(0);
#endif
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
#if WITH_HA
        MQTT.onConnected = haMqttSendDiscovery;
#endif
    };
#if WITH_HA
    // Periodic HA power publish, throttled inside MqttService::onTick (only ticks while Running).
    MQTT.tickHook = [] {
        if (!mppt.sensorPhysicalI || !mppt.sensorPhysicalU) return; // sensor setup failed, mppt.begin() skipped
        float pow = mppt.sensorPhysicalI->ewm.avg.get() * mppt.sensorPhysicalU->ewm.avg.get();
        haMqttUpdate({.power = mppt.isSweeping() ? NAN : pow});
    };
#endif
    g_services.registerService(&MQTT);
    g_services.registerService(&telemetryService);
#if WITH_FTP
    g_services.registerService(&ftpService);
#endif
#if WITH_HTTP_CONF
    g_services.registerService(&httpConfService);
#endif
    g_services.registerService(&telnetService);
#endif
    g_services.registerService(&lcdService);
#if WITH_SCOPE
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
static constexpr uint32_t OTA_CONFIRM_DEADLINE_MS = 2 * 3600 * 1000;

// A flash erase/program now won't stall the RT loop at power: auto-suspend passed its boot self-test,
// or the converter is off or near idle.
bool flashWriteSafe() {
    return fugu_flash_sus_on() || converter.disabled() || std::fabs(mppt.meter.rtPower.load(std::memory_order_relaxed)) < 5.f;
}
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

// Keep the rollback net armed until WE say so.
//
// Arduino's initArduino() otherwise confirms a PENDING_VERIFY image itself
// (managed_components/espressif__arduino-esp32/cores/esp32/esp32-hal-misc.c:312) whenever
// CONFIG_APP_ROLLBACK_ENABLE=y: the weak verifyRollbackLater() returns false and the weak
// verifyOta() returns true unconditionally, so the image is marked valid before setup() runs —
// before the boot watchdog is even armed — and lfMarkOtaValid() below then always finds a
// non-pending state and skips. Measured on flu 2026-09-08: a fresh BLE OTA reported
// `rollback=VALID` at 8 s uptime, well inside the 20 s gate. That is the safety net silently
// absent, so a bad image boot-loops instead of reverting.
//
// Overriding the hook defers the decision to lfMarkOtaValid(), which confirms only once the RT
// loop is proven healthy. C linkage: the weak symbol comes from a C translation unit.
#ifdef CONFIG_APP_ROLLBACK_ENABLE
extern "C" bool verifyRollbackLater() { return true; }
#endif

// Confirm a PENDING_VERIFY image once the RT loop is proven alive (sampling). Runs from loopLF on
// core 0 (one-shot otadata flash write). Non-pending state (normal flash) → cheap skip. Until this
// runs, a reset reverts to the previous slot — that's the rollback safety net.
static void lfMarkOtaValid() {
    static bool done = false, healthy = false, deferred = false;
    if (done || millis() < OTA_VALIDATE_UPTIME_MS) return;
    if (!timeLastSampler || (wallClockUs() - timeLastSampler) > 1000000ULL) return; // RT sampler alive?
    if (!healthy) {
        healthy = true;
#ifdef CONFIG_BOOTGUARD_ENABLE
        bootguard_mark_healthy();
#endif
    }
    if (!deferred) {
        esp_ota_img_states_t st;
        if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) != ESP_OK ||
            st != ESP_OTA_IMG_PENDING_VERIFY) {
            done = true;
            return;
        }
    }
    // without flash auto-suspend the otadata erase stalls the RT loop ~60 ms: wait for low power
    if (!flashWriteSafe()) {
        if (millis() < OTA_CONFIRM_DEADLINE_MS) {
            if (!deferred) ESP_LOGW("main", "OTA confirm deferred until output < 5 W (a reset now rolls back)");
            deferred = true;
            return;
        }
        ESP_LOGW("main", "OTA confirm deadline, writing at power");
    }

    done = true;
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
        ESP_LOGI("main", "OTA image confirmed valid (rollback cancelled)");
    else
        ESP_LOGE("main", "failed to confirm OTA image valid");
}

__attribute__((cold)) static void wireGuardConf(const ConfFile &c) {
    auto &w = mppt.charger.wireGuard.conf;
    w.mode = c.getByte("wire_protect", 1);
    w.pWarn = c.getFloat("wire_p_warn", 2.f);
    w.pDerate = c.getFloat("wire_p_derate", 4.f);
    w.pTrip = c.getFloat("wire_p_trip", 8.f);
    w.drMin = c.getFloat("wire_dr_min", 4.f) * 1e-3f;
    w.rMax = c.getFloat("wire_r_max", 0.f) * 1e-3f;
    assert_throw(w.mode <= 2, "wire_protect must be 0, 1 or 2");
    assert_throw(w.pWarn > 0 && w.pWarn <= w.pDerate && w.pDerate <= w.pTrip,
                 "need 0 < wire_p_warn <= wire_p_derate <= wire_p_trip");
    assert_throw(w.drMin > 0 && w.rMax >= 0, "need wire_dr_min > 0, wire_r_max >= 0");
}

// fixed address at the top of RTC slow memory (main/wire_rtc.ld)
static WireGuard::Rtc s_wireRtc __attribute__((section(".rtc_wire")));
// TODO drop once no device runs build 804: its copy at 0x50000004 (fit) / 0x50000024 (day)
static RTC_NOINIT_ATTR struct {
    WireGuard::FitRtc fit;
    WireGuard::DayRtc day;
} s_wireRtcOld;

__attribute__((cold)) static void wireGuardLoad() {
    auto &g = mppt.charger.wireGuard;
    if (!g.conf.mode) return;
    const bool cur = s_wireRtc.magic == WireGuard::RTC_MAGIC;
    if (g.restoreDay(cur ? s_wireRtc.day : s_wireRtcOld.day))
        ESP_LOGI("wire", "open day retained: %u Rc samples, %u sweeps", g.daySamples(), g.daySweeps());
    if (g.restoreFit(cur ? s_wireRtc.fit : s_wireRtcOld.fit)) ESP_LOGI("wire", "offset regression retained");
    WireGuard::Blob b;
    nvs.open();
    const bool have = nvs.readBlob("wire", &b, sizeof b);
    nvs.close();
    if (have && !g.load(&b, sizeof b)) ESP_LOGW("wire", "stored baseline rejected, learning again");
    else if (have) ESP_LOGI("wire", "baseline: %u days, Rc %.2f mOhm, Ro %.2f mOhm", g.dayCount(),
                            g.baseRc() * 1e3f, g.baseRo() * 1e3f);
}

static void idleProfileSetup(const ConfFile &boardConf);

__attribute__((cold)) void setup() {
#ifdef WITH_NETW
    bootLogBegin();
#endif
    consoleInit();
    setupCli();
    ESP_LOGI("main", "*** %s", format_version());
    bootWatchdogArm();
    heapDiagBegin();
    flashSuspendProbe();

    rtcount_test_cycle_counter();

    nvs.init();

    if (!mountLFS()) {
        ESP_LOGE("main", "Error mounting LittleFS partition!");
        g_app.setupErr = true;
    } else if (!confDropStaging("/littlefs/conf")) {
        ESP_LOGE("main", "stale conf stage left in /littlefs/conf");
    }
    // Fingerprint the config as loaded at boot (conf-hash compares it with the current files to
    // reveal set-config writes not yet applied by a restart). Read-only, before any conf is parsed.
    confHashCaptureBoot();

#ifdef WITH_USB_MSC
    usbDevBegin();
#endif

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
    idleProfileSetup(boardConf);

    TeleConf teleConf = setupNetworkAtBoot();

    Limits lim{};
    try {
        ConfFile limitsConf{"/littlefs/conf/limits.conf"};
        lim = Limits{limitsConf};
        wireGuardConf(limitsConf);
        limitsConf.warnUnknownKeys();
    } catch (const std::runtime_error &er) {
        logConfErr("limits.conf", er);
        g_app.setupErr = true;
    }

    wireGuardLoad();

#if PIN_GPIO_ISR_TO_RT_CORE
    // Must run before setupConverterAndMppt → setupSensors → ADC_INA226/ADS::init → attachInterrupt.
    pinGpioIsrToRtCore();
#endif

#if WITH_WSYNC
    // Consume the `wsync arm` one-shot BEFORE the converter can touch the USB pad: clearing it
    // afterwards would repeat the forced probe forever if the probe itself crashed, and clearing
    // it at all is what keeps a power cycle a way back to automatic mode.
    nvs.open();
    if (!nvs.readString("wsync_arm", "").empty()) {
        nvs.writeString("wsync_arm", "");
        nvs.commit();
        converter.wsyncArmRequest = true;
    }
    nvs.close();
#endif

    setupConverterAndMppt(boardConf, lim, teleConf, noI2C);
    if (mppt.sensorPhysicalI) selftestLoad();

    if (!g_app.setupErr) {
        // ~2 KB used (log formatting runs on core 0). Heap is the scarcer resource on the S3.
        xTaskCreatePinnedToCore(loopRT, kRtTaskName, 8192, NULL, RT_PRIO, NULL, RT_CORE);
#if WITH_RTPROF
        rtprofStart(2000);
#endif
    } else {
        led.setHexShort(0x200);
    }

    registerServices();

    // esp_log -> vprintf_mux (telnet/MQTT/boot-log sinks). Kept LATE on purpose: hooking it before
    // WiFi connects routes the wifi task's connect-time logging burst through vprintf_mux's 300B
    // stack buffer, overflowing the 3072B wifi task stack (this bricked flat). So esp_log lines
    // reach the boot-log backlog from here on; UART_LOG lines since bootLogBegin().
#if defined(WITH_NETW) || defined(WITH_USB_MSC)
    enable_esp_log_to_telnet();
#endif

#if !CONFIG_HEAP_POISONING_DISABLED
    ESP_LOGW("main", "heap poisoning on: per-alloc RAM + CPU cost, debug builds only");
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

#ifdef CONFIG_PM_ENABLE
esp_pm_lock_handle_t s_cli_pm_lock = nullptr; // held after RT start; `pwr pm` releases it
#endif

static esp_err_t disable_cpu_power_saving(void) {
    esp_err_t ret = ESP_OK;

#ifdef CONFIG_PM_ENABLE

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

void stopAndBackoff(uint32_t secondsDelay, const char *who = "stopAndBackoff") {
    // Two independent gates hold the converter down: delayStartUntil here, and mppt's
    // _backoffUntilUs (via startCondition()). The restart needs BOTH, so the longer one wins and
    // arming shutdownDcdc's 5 s default below would mask the 100 ms fast retry entirely -- which it
    // did, since shutdownDcdc's own psuFastRetry excludes who=="stopAndBackoff" by name. That
    // exclusion is about not counting maintenance toward the trip/latch counter and is kept; it
    // must not also decide the duration. Pass 0 so the fast path is governed by delayStartUntil
    // alone. 100 ms still rate-limits the retry ~45x below the sample rate, which is what the
    // default backoff exists to prevent. A sweep's converter is off while it calibrates, and the
    // sweep would restart it, so a sweep is shut down too.
    const bool psuFast = g_app.psuMode() && secondsDelay <= 5;
    if (!converter.disabled() || mppt.isSweeping())
        mppt.shutdownDcdc(who, psuFast ? 0 : std::min<uint32_t>(secondsDelay, 5));
    delayStartUntil = wallClockUs() + (psuFast ? 100000ULL
                                              : static_cast<time_us>(secondsDelay) * 1000000ULL);
}

static void loopRT(void *arg) {
    // low-latency control loop task
    rtAllocBegin();

    try {
        adcSampler.begin();
        setLoopRateMin();
    } catch (const std::runtime_error &er) {
        ESP_LOGE("main", "error starting ADC sampler: %s", er.what());
        g_app.setupErr = true; // the network loop advances the wall clock instead
        vTaskDelete(nullptr);
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
        setWallClockUs(esp_timer_get_time());
        rtcount("micros");
        auto &nowUs(loopWallClockUs_);

        time_ms nowMs = nowUs / 1000ULL;

        if (unlikely(adcSampler.adcResetPending())) {
            RT_COLD(
            if (adcSampler.halted) {
                adcSampler.dropAdcReset();
                ESP_LOGW("main", "ADC reset dropped, sampler halted");
            } else {
                stopAndBackoff(6); // >5 s: no PSU fast retry
                bool ok = adcSampler.applyAdcResetRt();
                setWallClockUs(esp_timer_get_time());
                if (!ok) ESP_LOGE("main", "ADC reset failed");
                stopAndBackoff(ok ? 6 : 30); // re-arm from the end of the reset
                if (timeLastSampler) timeLastSampler = nowUs; // grace for the stall watchdog
            }
            );
        }

#if WITH_RTPROF
        if (unlikely(g_rtLogTest)) {
            RT_COLD(
            const uint8_t k = --g_rtLogTest;
            g_rawLogForce = k & 1;
            static const char lit[] = "lit";
            char tmp[8] = "stack";
            ESP_LOGI("rtlog", "%s i=%d f=%.2f s=%s u=%lu g=%.3f t=%s x=%x a=%.1f b=%.1f c=%.1f h=%hu e=%d",
                     g_rawLogForce ? "raw" : "fmt", -42 - k, 3.14159f, lit, 4000000000UL, 1e-3, tmp, 0xbeef,
                     1.5f, -2.5f, 1234.5f, (uint16_t) 65000, 7);
            UART_LOG("%s uartlog i=%d f=%.2f s=%s", g_rawLogForce ? "raw" : "fmt", k, 2.5f, lit);
            g_rawLogForce = false;
            );
        }
#endif
        rtcount("adc.update.pre");
        auto samplerRet = adcSampler.update();
        rtcount("adc.update");

        // outside the NewData branch: a dead-time or frequency change must also land while the
        // converter idles
        converter.applyPendingDeadTimeRt();
        mppt.applyPendingPwmFreqRt();

        static bool wasHalted = false;
        if (adcSampler.halted) {
            wasHalted = true;
            continue;
        }
        if (wasHalted && timeLastSampler) timeLastSampler = nowUs; // grace for the stall watchdog
        wasHalted = false;

        if (samplerRet == ADC_Sampler::UpdateRet::CalibFailure)
            stopAndBackoff(4, "calib-fail");

        // ADC stall watchdog. AdcError = the continuous-ADC isGood() tripped (DMA stalled); a stale
        // timeLastSampler = the sampler is alive but no fresh sample reached the controller. Either way
        // we restart the ADC promptly. The converter is stopped only if the stall PERSISTS past
        // kAdcSustainedMs — a transient wedge (e.g. a console uxTaskGetSystemState() briefly starving
        // the DMA) recovers on one reset and must NOT trip a backoff on a live converter. Calibration
        // legitimately withholds fresh samples, so it's excluded from the timeLastSampler arm.
        constexpr time_ms kAdcResetThrottleMs = 300, kAdcSustainedMs = 800, kAdcRestartMs = 60000;
        static time_ms adcStalledSinceMs = 0, lastAdcResetMs = 0;
        static bool wasCalibrating = false;

        // calibration withholds NewData; give the first post-calibration sample the usual window
        bool calibrating = adcSampler.isCalibrating();
        if (wasCalibrating && !calibrating && timeLastSampler) timeLastSampler = nowUs;
        wasCalibrating = calibrating;

        bool adcStalled = samplerRet == ADC_Sampler::UpdateRet::AdcError ||
                          (samplerRet != ADC_Sampler::UpdateRet::NewData && !calibrating &&
                           timeLastSampler && nowUs - timeLastSampler > 200000);

        if (adcStalled) {
            RT_COLD(
            if (!adcStalledSinceMs) adcStalledSinceMs = nowMs;
            mppt.lvPgood.release();
            if (nowMs - lastAdcResetMs > kAdcResetThrottleMs) { // restart a wedged DMA quickly
                lastAdcResetMs = nowMs;
                ESP_LOGW("main", "ADC stall (%s, last sample %lu ms ago), resetting",
                         samplerRet == ADC_Sampler::UpdateRet::AdcError ? "AdcError" : "stale",
                         (unsigned long) (timeLastSampler ? (nowUs - timeLastSampler) / 1000 : 0));
                adcSampler.resetPeripherals();
            }
            if (nowMs - adcStalledSinceMs > kAdcSustainedMs && !converter.disabled() && !mppt.inBackoff()) {
                ESP_LOGE("main", "ADC stall %llu ms, shutdown (nSamples=%lu)", nowMs - adcStalledSinceMs,
                         lastMpptUpdateNumSamples);
                stopAndBackoff(16);
            }
            if (nowMs - adcStalledSinceMs > kAdcRestartMs) systemRestart();
            );
        } else {
            adcStalledSinceMs = 0;
        }

        if (samplerRet != ADC_Sampler::UpdateRet::NewData) {
            // Do not make console PSU shutdown/override depend on a fresh ADC sample. A wedged
            // sampler is exactly when `psu off` must still be able to transfer ownership to the
            // RT core and disable/ramp the converter.
            mppt.applyPendingPsuCommandRt(false);
            if (adcSampler.isCalibrating() && mppt.boardPowerSupplyUnderVoltage()) {
                RT_COLD(
                ESP_LOGW("main", "Board power supply UV %.2f!", mppt.boardPowerSupplyVoltage());
                adcSampler.cancelCalibration();
                );
            }

            if (!timeLastSampler and nowMs > 20000) {
                RT_COLD(
                converter.disable();
                ESP_LOGE("main", "Never got a sample! Please check ADC");
                if (nowMs > (1000 * 60 * 15)) {
                    systemRestart();
                }
                delay(5000);
                );
            }
        } else {
            loopRTNewData(nowMs);
            rtcount("loopRTNewData");
        }


        auto lag = nowUs - lastLoopTime;
        if (lastLoopTime && lag > g_app.maxLoopLag && !converter.disabled()) g_app.maxLoopLag = lag;
        if (lastLoopTime && lag > g_app.cmdLoopLag) g_app.cmdLoopLag = lag;
        {
            static uint32_t fseq = 0;
            const uint32_t sq = fugu_flash_seq;
            auto &m = g_flashLagUs[(sq != fseq || (sq & 1)) ? 0 : 1];
            if (lastLoopTime && lag > m) m = lag;
            fseq = sq;
        }
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
// website/docs/development/debugging/real-time-latency.md) — can't trigger stopAndBackoff; only sustained starvation does.
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
    // headroom), calibration, manual PWM, a no-MPP sweep backoff (dim light) and brief transients.
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
    bool stuck = headroom && noPower && !g_app.manualPwm() && !mppt.noMppBackoff()
                 && (g_app.psuMode()
                         ? (std::isfinite(psuSetpoint)
                            && sensors.Vout->ewm.avg.get() < psuSetpoint - 2.0f)
                         : !mppt.charger.chargeHold());

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

static void wireAlert(const WireGuard &g) {
    const int8_t a = g.wf();
    if (a == WireGuard::WF_UNVERIFIED) {
        if (g.unverifiedLong())
            alert("wire", AlertLevel::Unverified, "no wire data on the last %u charging days", WireGuard::UNV_DAYS + 1);
        else alert("wire", AlertLevel::Unverified, "no fresh wire data");
        return;
    }
    const auto f = g.fastPath();
    alert("wire", (AlertLevel) a, "Pw %.1fW dR %.1fmOhm (%s)%s%s%s", g.pw(), (f.fresh ? f.dr : g.rcPath().dr) * 1e3f,
          f.fresh ? "fast" : "slow", g.armed() ? "" : " warn-only", g.rbWarn() ? ", shared path up" : "",
          g.dayWarn() ? ", day dR up" : "");
}

std::atomic<bool> g_wireResetReq{false};

// Baseline save: only with the converter off, a flash write stalls the RT loop for tens of ms.
static void lfWireGuard(float iout) {
    auto &g = mppt.charger.wireGuard;
    if (!g.conf.mode) return;
    if (g_wireResetReq.exchange(false)) g.reset();
    const uint32_t nowS = (uint32_t) (wallClockUs() / 1000000);
    const auto &w = mppt.charger.wire;
    const bool running = !mppt.converter.disabled();
    static bool first = true;
    static uint8_t lastLvl = 0;
    static bool lastRb = false;
    if (g.tick(nowS, iout, running, w.own(nowS), w.offset(nowS), w.shared(nowS), mppt.outZ.get(nowS),
               mppt.outZ.stamp()) || first) {
        first = false;
        wireAlert(g);
    }
    if (g.level() != lastLvl || g.rbWarn() != lastRb) {
        ESP_LOGW("wire", "level %u -> %u%s, shared %d, Pw %.1f W, cap %.1f A", lastLvl, g.level(),
                 g.armed() ? "" : " (warn only)", (int) g.rbWarn(), g.pw(), g.ioutCap());
        lastLvl = g.level();
        lastRb = g.rbWarn();
    }
    static uint32_t rtcS = 0;
    if (nowS - rtcS >= 10) {
        rtcS = nowS;
        s_wireRtc.magic = WireGuard::RTC_MAGIC;
        g.saveDay(s_wireRtc.day);
        g.saveFit(s_wireRtc.fit);
    }
    static uint32_t lastTryS = 0;
    if (g.saveDue(running) && (!lastTryS || nowS - lastTryS >= 600)) {
        lastTryS = nowS;
        nvs.open();
        const bool ok = nvs.writeBlob("wire", &g.blob(), sizeof(WireGuard::Blob)) && nvs.tryCommit();
        nvs.close();
        if (ok) g.saved();
        else ESP_LOGE("wire", "baseline save failed");
    }
}

// Low-frequency control reads (RT-adjacent, not in the ADC fast path): NTC + chip temp,
// charger termination state, and the thermal-cap WiFi cutoff.
static void lfControl() {
#ifdef WITH_USB_MSC
    g_app.usbConnected = usbCdcConnected();
#elif CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
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
        const bool currentShowsAuthority = std::isfinite(iout) && (iout > 0.5f || mppt.charger.chargeHold());
        const bool voutAuthority = !mppt.converter.disabled()
                                   && mppt.converter.getCtrlOnPwmCnt() > minAuthorityPwm
                                   && currentShowsAuthority;
        mppt.charger.update(vout, iout, voutAuthority, g_app.psuMode());
        mppt.charger.updateWire(vout, iout, !mppt.converter.disabled());
        lfWireGuard(iout);
    }
    alertTick();
    wifiShutdownIfHot(mppt.ucTemp.lastFresh());

    // A stale fitted sensor trips protectLf (`temp-stale`) and blocks starts, also in manual PWM. The
    // NaN fallbacks downstream (25 % P_max derate, fan holds its level) are only defence in depth.
    static bool warnedStale = false;
    bool stale = mppt.tempStale();
    if (stale != warnedStale) {
        warnedStale = stale;
        if (stale) ESP_LOGW("main", "temperature stale (ntc=%d mcu=%d), converter held off",
                            (int) mppt.ntc.fresh(), (int) mppt.ucTemp.fresh());
        else ESP_LOGI("main", "temperature refresh recovered");
    }
}

// One-line UART/MQTT/telnet status line. WITH_MEASURE_COIL skips it during the coil sweep so the
// measurement output isn't intermixed with the status print.
__attribute__((cold)) static void lfStatusLine(uint32_t nSamples, uint32_t sps, uint32_t dt) {
#ifdef WITH_MEASURE_COIL
    if (!sensors.Vin || isMeasuring()) return;
#else
    if (!sensors.Vin) return;
#endif
    // A frozen temperature must not read like a measured one: print "--" once the refresh path has
    // stopped stepping the filter (issue #65).
    char ntcStr[8], ucStr[8];
    auto tempStr = [](char *buf, const SingleValueSensor &s) -> const char * {
        if (!s.fresh()) snprintf(buf, 8, "--");
        else snprintf(buf, 8, "%.0f", s.last());
        return buf;
    };
    UART_LOG(
        "V=%4.*f/%5.*f I=%4.*f/%5.*fA %5.1fW %s℃%s℃ %2lusps %2lu㎅/s %s(H|L|Lm)=%4hu|%4hu|%4hu"
        " st=%5s,%i lag=%lu㎲ N=%lu rssi=%hi",
        sensors.Vin->last >= 9.55f ? 1 : 2, sensors.Vin->last,
        sensors.Vout->last >= 9.55f ? 2 : 2, sensors.Vout->last,
        sensors.Iin->ewm.avg.get() >= 9.55f ? 1 : 2, sensors.Iin->ewm.avg.get(),
        sensors.Iout->ewm.avg.get() >= 9.55f ? 2 : 2, sensors.Iout->ewm.avg.get(),
        sensors.Vin->ewm.avg.get() * sensors.Iin->ewm.avg.get(),
        tempStr(ntcStr, mppt.ntc), tempStr(ucStr, mppt.ucTemp),
        sps,
        dt ? (uint32_t) (bytesSent * 1000llu / dt) : 0,
        converter.inDCM() ? "DCM" : "CCM",
        converter.getCtrlOnPwmCnt(), converter.getRectOnPwmCnt(), converter.getRectOnPwmMax(),
        g_app.manualPwm()
            ? "MANU"
            : (g_app.psuMode()
                   ? (mppt.isPvActive() ? "PVS" : "PSU")
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
        // before the first sample every sensor is stale; not worth a line
        if (reason && !(!strcmp(reason, "temp-stale") && std::isnan(sensors.Vin->ewm.avg.get())) &&
            (reason != lastReason || nowUs - lastLogUs > 30000000)) {
            ESP_LOGI("mppt", "START blocked: %s (Vin=%.1f Vout=%.1f ntc=%.0f mcu=%.0f)", reason,
                     sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(),
                     mppt.ntc.lastFresh(), mppt.ucTemp.lastFresh());
            lastReason = reason;
            lastLogUs = nowUs;
        }
    }
}

// Wired-sync follower: is the leader's pulse still arriving on the sync pin? Edge rate between
// loopLF calls, sampled without clearing the counter (the `wsync` command clears it, which shows
// up as a negative delta and is skipped instead of read as a loss). Returns a warning color, or
// 0 while the rate matches pwm_freq — a locked wire adds no color of its own.
static uint16_t lfWsyncWarnColor(time_us nowUs) {
    if (!converter.wsyncFollower || !converter.wsyncHasCounter()) return 0;
    static int lastCount = 0;
    static time_us lastUs = 0;
    static uint16_t warn = 0;
    int c = converter.wsyncCount();
    int dc = c - lastCount;
    time_us dt = nowUs - lastUs;
    bool first = !lastUs;
    lastCount = c;
    lastUs = nowUs;
    if (first || dc < 0 || dt < 100000) return warn;
    float expect = (float) converter.getPwmFrequency() * (float) dt * 1e-6f;
    warn = ((float) dc < expect * 0.1f) ? 0x300                                   // no pulses at all
         : ((float) dc < expect * 0.8f || (float) dc > expect * 1.25f) ? 0x330    // wrong rate / noise
         : 0;
    return warn;
}

// RGB LED color from current converter state (manual / idle / sweep / MPPT / CV / topping).
static void lfUpdateLed(time_us nowUs) {
    // A follower whose wire is dead free-runs against the leader, so flag it above everything
    // else — but only every other update, so the state color below stays readable. Suppressed
    // once the idle branch turns the LED off for the night.
    uint16_t wsyncWarn = lfWsyncWarnColor(nowUs);
    static bool warnPhase = false;
    warnPhase = !warnPhase;
    if (wsyncWarn && warnPhase && (!mppt.converter.disabled() || nowUs <= 60000000ULL * 15)) {
        led.setHexShort(wsyncWarn);
        return;
    }
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

static bool selftestMaintenance();

__attribute__((cold)) void loopLF(const time_us &nowUs, bool interim) {
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

    // a sensor reset (calibration) zeroes the counter: count the samples since then
    uint32_t n = nSamples >= lastNSamples ? nSamples - lastNSamples : nSamples;
    uint32_t sps = (dt > 20000) ? (uint64_t) n * 1000000llu / dt : lastSps;
    lastSps = sps;

    lfWatchdog(nowUs, dt, sps, nSamples);
    lfStuckWatchdog();
    lfControl();
    lfMarkOtaValid();
    static bool lagReported = false;
    if (!lagReported && esp_timer_get_time() > 7200ll * 1000000) {
        lagReported = true;
        handleCommand("reset-lag");
    }
    mppt.rectCalLf();
    if (mppt.sensorPhysicalI) selftestLf(wallClockMs(), selftestMaintenance());
    mppt.scanLf(flashWriteSafe());
    heapDiagLf((uint32_t) wallClockMs());
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

    // Print pacing stays coupled to console activity (don't scribble over what the user is typing,
    // and stay quiet while idle) — but it no longer gates any of the work above.
    if (!lastTimeOutUs
        or (nowUs - lastTimeOutUs) >= (mppt.converter.disabled() ? (lfPeriod * 8) : lfPeriod)) {
        lfStatusLine(nSamples, sps, dt);
        // Unconditional: lfStatusLine also returns without printing (no sensors / coil sweep), and
        // leaving lastTimeOutUs at 0 there would make the outer !lastTimeOutUs gate spin loopLF —
        // and with it the die-sensor read — every millisecond.
        lastTimeOutUs = wallClockUs();
    }

    lastNSamples = nSamples;
    lastWindowUs = nowUs;
    bytesSent = 0;

    mppt.meter.update(); // day accumulator; needs to run WHILE converting, that is when energy flows

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

    if (haveNewSample) {
        timeLastSampler = wallClockUs();
        mppt.rectCalRt();
        mppt.hsShortRt((uint32_t) nowMs);
    }

    // auto range current sense ADC channel TODO add hysteresis
    /* float adcRangeBound = 2.0f;
    adcSampler.adc->setMaxExpectedVoltage(
            sensors.Iin->adcCh,
            (std::max(sensors.Iin->last, sensors.Iin->ewm.avg.get()) < sensors.Iin->transform.apply(adcRangeBound))
            ? adcRangeBound
            : sensors.Iin->transform.apply_inverse(30.f)
    ); */


    // an alert while not switching must not trip the next start; one while the LS clamp is pending, ramping or held
    // is left to the next hsShortRt(), which releases the clamp
    if (mppt.converter.disabled() && (!mppt.hsFaultState() || mppt.hsFaultState() == MpptController::HsOff))
        g_hwOvTrip = false;

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
        } else if (!flashOpActive() && wallClockUs() > delayStartUntil) { // start runs from flash
            if (!mppt.startBlockReasonRt()) RT_COLD(
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
                static int64_t bmsRearmAllowedUs = 0;
                static bool bmsWasFresh = false;
                constexpr int64_t BMS_BOOT_WAIT_US = 12'000'000;
                constexpr int64_t BMS_REARM_MIN_US = 10ll * 60 * 1000000;
                bool bmsFresh = mppt.charger.batSt.haveValidCellVoltage();
                // A stale BMS cell frame can leave termCond stuck true. Don't trust it: only treat
                // the pack as full when termCond is true AND the BMS data is fresh. The partial and
                // cold holds don't block the start: their pin regulates the pack current, and the
                // converter has to run to serve the loads.
                bool full = bool(mppt.charger.termCond) && bmsFresh;
                // Wait for termCond to actually be evaluated (cell voltage AND warm ibat), not just
                // for the first cell frame — otherwise the sweep could fire in the gap before ibat
                // smoothing warms up and termination latches. A fresh->stale edge re-arms the wait
                // (a brief outage must not sweep into a possibly-full pack), but at most once per
                // 10 min: re-arming on every edge let a flapping BMS block the sweep for hours
                // (flat, 2026-07-06).
                bool waitingForBms = mppt.charger.hasBmsCellSource()
                                     && (!mppt.charger.terminationDecided() || !bmsFresh);
                if (bmsWasFresh && !bmsFresh && esp_timer_get_time() > bmsRearmAllowedUs) {
                    bmsBootDeadline = 0;
                    bmsRearmAllowedUs = esp_timer_get_time() + BMS_REARM_MIN_US;
                }
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
            );
        }

        if (scope && sensors.Vout && haveNewSample)
            scope->addSample12(&mppt, 0,
                               (uint16_t) max(0.f, sensors.Vout->last / 60.0f *
                                                   2000.0f));

        if (haveNewSample) {
            auto lv = converter.boost() ? sensors.Vin : sensors.Vout;
            mppt.lvPgood.update(lv->med3.get(), lv->ewm.avg.get(), lv->numSamples, !mppt.inBackoff(),
                                (uint32_t) nowMs);
        }
    }

    if (g_app.manualPwm()) {
        if (!converter.disabled())
            converter.pwmPerturb(0); // this will increase LS duty cycle if possible
    }
}


#ifdef WITH_NETW
static NetGuard s_guard;
// Survives esp_restart(): magic | kNgHold (guard reboot, no MQTT connect since) | guard reboots since power-on.
static RTC_NOINIT_ATTR uint32_t s_ngRtc;
static constexpr uint32_t kNgMagic = 0x4e470000, kNgHold = 0x8000;

static uint32_t ngRtc() {
    if ((s_ngRtc & 0xffff0000) != kNgMagic) s_ngRtc = kNgMagic;
    return s_ngRtc;
}

void netGuardPrint() {
    const uint32_t r = ngRtc();
    UART_LOG("netguard: mqtt down %lu s, re-inits %u, guard reboots %lu since power-on%s",
             (unsigned long) (s_guard.downMs((uint32_t) wallClockMs()) / 1000), s_guard.reinits,
             (unsigned long) (r & 0x7fff), (r & kNgHold) ? ", reboot held" : "");
}

// Once a second: retry Failed/Stopped network services, re-init a disconnected MQTT client with
// growing gaps, and reboot when MQTT stays down (at most once per kRebootHoldMs without a connect).
static void netGuardTick(bool wifiUp) {
    auto &guard = s_guard;
    static uint32_t lastMs = 0;
    const auto nowMs = (uint32_t) wallClockMs();
    if (nowMs - lastMs < 1000) return;
    lastMs = nowMs;

    const auto st = MQTT.state();
    NetGuardIn in{};
    in.nowMs = nowMs;
    in.freeHeap = esp_get_free_heap_size();
    // walks every block with the heap locked: only while it can matter
    in.largest = MQTT.isConnected() ? NetGuard::kHeapUnknown
                                    : (uint32_t) heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    in.wifiUp = wifiUp;
    in.mqttActive = MQTT.enabled() && MQTT.configured() && st != ServiceState::Stopped;
    in.mqttRunning = st == ServiceState::Running;
    in.mqttConnected = MQTT.isConnected();
    in.rebootOk = esp_timer_get_time() > 300LL * 1000000 && !flashOpActive() && !g_restartBlocked
#ifdef WITH_BLE
                  && !otaBleActive()
#endif
            ;
    const uint32_t r = ngRtc();
    in.rebootHeld = r & kNgHold;
    if (in.rebootHeld && in.mqttConnected) s_ngRtc = r & ~kNgHold;
    const uint8_t a = guard.step(in);
    if (a & NG_REBOOT) {
        s_ngRtc = kNgMagic | kNgHold | ((r + 1) & 0x7fff);
        ESP_LOGE("main", "mqtt down %lu s with heap %lu B (largest %lu, min %lu): guard reboot #%lu",
                 (unsigned long) (guard.downMs(nowMs) / 1000), (unsigned long) in.freeHeap,
                 (unsigned long) in.largest, (unsigned long) esp_get_minimum_free_heap_size(),
                 (unsigned long) (s_ngRtc & 0x7fff));
        systemRestart();
        return;
    }
    if (a & NG_RESTART_MQTT) {
        ESP_LOGW("main", "mqtt disconnected %lu s, re-init #%u (heap %lu B, largest %lu)",
                 (unsigned long) (guard.downMs(nowMs) / 1000), guard.reinits, (unsigned long) in.freeHeap,
                 (unsigned long) in.largest);
        MQTT.restart();
    }
    if (a & NG_RETRY_FAILED) g_services.retryFailedNetworkServices();
}
#endif

#ifdef WITH_NETW
static int8_t s_wifiPs = -1; // -1 unset, 0 MIN_MODEM, 1 MAX_MODEM
#endif

// CPU part of the profile (core 0). InhibitIdle is set from before the clock drops until the RT loop
// has settled at the boot clock again, so the converter never starts at the idle clock.
enum : uint8_t { CpuBoot, CpuPre, CpuTest, CpuLow, CpuSettle };

static struct {
    uint16_t cpuMhz, advMs, bootMhz; // 0 = that part off
    uint32_t teleMs;
    bool on, suspended, hold;
    bool cpuOff;         // guard: the RT loop lost samples at cpuMhz, CPU part off until reboot (RAM only)
    uint8_t cpu, force;  // force: 0 auto, 1 off, 2 on (`pwr idle`)
    uint8_t tries;       // CpuPre: windows without a rate; CpuTest: disturbed windows
    bool stray;          // a stray InhibitIdle was cleared (logged once)
    float pre;           // samples/s before the switch
    int64_t swUs;        // CpuTest: clock switch time
    struct { int64_t t; uint32_t n, seq; time_us cal; } w; // rate window start
} s_idle{};

#if CONFIG_FUGU_WITH_PWR_KNOBS
static uint32_t s_idleAfterMs = 5u * 60 * 1000;
void idleProfileAfter(uint32_t s) { s_idleAfterMs = s * 1000; }
uint32_t idleProfileAfter() { return s_idleAfterMs / 1000; }
#else
static constexpr uint32_t s_idleAfterMs = 5u * 60 * 1000;
#endif

bool idleProfileOn() { return s_idle.on; }

// board.conf idle_cpu_mhz, ble.conf idle_adv_ms, tele.conf idle_interval_ms
static void idleProfileSetup(const ConfFile &boardConf) {
    const long mhz = boardConf.getLong("idle_cpu_mhz", 80);
    if (mhz && mhz != 80 && mhz != 160 && mhz != 240) {
        ESP_LOGE("main", "board.conf idle_cpu_mhz=%ld: expected 0, 80, 160 or 240", mhz);
        g_app.setupErr = true;
        return;
    }
    rtc_cpu_freq_config_t cur, idle;
    rtc_clk_cpu_freq_get_config(&cur);
    s_idle.bootMhz = cur.freq_mhz;
    s_idle.cpuMhz = mhz == cur.freq_mhz ? 0 : mhz;
    // a different PLL (classic ESP32: 240 MHz) is not a divider switch, it stops the PLL clocks
    if (s_idle.cpuMhz && (!rtc_clk_cpu_freq_mhz_to_config(mhz, &idle) || idle.source_freq_mhz != cur.source_freq_mhz)) {
        ESP_LOGW("main", "idle_cpu_mhz=%ld needs a PLL change from %lu MHz, CPU stays", mhz, (unsigned long) cur.freq_mhz);
        s_idle.cpuMhz = 0;
    }
    s_idle.advMs = std::clamp(loadConfSafe("/littlefs/conf/ble.conf", false, true).getLong("idle_adv_ms", 1000), 0L,
                              10240L);
    if (s_idle.advMs && s_idle.advMs < 20) s_idle.advMs = 20;
    s_idle.teleMs = std::clamp(loadConfSafe("/littlefs/conf/tele.conf", false, true).getLong("idle_interval_ms", 900000),
                               0L, 3600000L);
}

static bool cpuSetMhz(uint32_t mhz) {
#ifdef CONFIG_PM_ENABLE
    esp_pm_config_t c = {};
    c.max_freq_mhz = mhz;
    c.min_freq_mhz = mhz;
    return esp_pm_configure(&c) == ESP_OK;
#else
    return setCpuFrequencyMhz(mhz);
#endif
}

static uint32_t voutSamples() { return sensors.Vout ? sensors.Vout->numSamples : 0; }

static void idleWinStart(int64_t t) {
    s_idle.w = {t, voutSamples(), fugu_flash_seq, adcSampler.getTimeLastCalibrationUs()};
}

// a flash write stalls both cores, a calibration resets the sample counter: no verdict from such a window
static float idleRateNow(int64_t t) {
    const auto &w = s_idle.w;
    const uint32_t sq = fugu_flash_seq;
    const bool dist = sq != w.seq || (sq & 1) || adcSampler.isCalibrating()
                      || adcSampler.getTimeLastCalibrationUs() != w.cal;
    return idleWinRate(w.n, voutSamples(), t - w.t, dist);
}

static void idleCpuRestore(int64_t now) {
    cpuSetMhz(s_idle.bootMhz);
    s_idle.cpu = CpuSettle;
    s_idle.w.t = now;
    s_idle.w.n = voutSamples();
}

// inhibit first, then make sure no start slipped through before the clock drops
static bool idleCpuLower() {
    const uint16_t en = converter.enableCount();
    converter.inhibitStart(true, SynchronousConverter::InhibitIdle);
    vTaskDelay(pdMS_TO_TICKS(10)); // a start that passed the inhibit check has finished
    if (!converter.disabled() || converter.enableCount() != en || converter.startWantedPending()) {
        converter.inhibitStart(false, SynchronousConverter::InhibitIdle);
        return false;
    }
    cpuSetMhz(s_idle.cpuMhz);
    return true;
}

// leaving the profile: back to the boot clock (settle) or drop a pending check
static void idleCpuLeave(int64_t now) {
    if (s_idle.cpu == CpuPre) s_idle.cpu = CpuBoot;
    else if (s_idle.cpu == CpuTest || s_idle.cpu == CpuLow) idleCpuRestore(now);
}

static void idleCpuTick() {
    auto &s = s_idle;
    const int64_t now = esp_timer_get_time();
    if (!s.on) idleCpuLeave(now); // also catches a leave from another task during idleCpuLower()
    // only CpuTest, CpuLow and CpuSettle own InhibitIdle
    if (s.cpu <= CpuPre && converter.startInhibited(SynchronousConverter::InhibitIdle)) {
        converter.inhibitStart(false, SynchronousConverter::InhibitIdle);
        if (!s.stray) UART_LOG("idle profile: stray start inhibit cleared");
        s.stray = true;
    }
    switch (s.cpu) {
        case CpuBoot:
            if (s.on && s.cpuMhz && !s.cpuOff) {
                s.cpu = CpuPre;
                s.tries = 0;
                idleWinStart(now);
            }
            break;
        case CpuPre: {
            if (now - s.w.t < kIdleRateWinUs) break;
            s.pre = idleRateNow(now);
            if (idleCpuVerdict(s.pre, s.pre) == IdleCpuVerdict::NoBase) {
                if (!s.tries++)
                    UART_LOG("idle profile: cpu stays %u MHz, no sample rate (%.0f/s), retrying", s.bootMhz, s.pre);
                idleWinStart(now);
                break;
            }
            if (!idleCpuLower()) {
                idleWinStart(now);
                break;
            }
            s.cpu = CpuTest;
            s.tries = 0;
            s.swUs = now;
            s.w.t = 0;
            break;
        }
        case CpuTest: {
            if (getCpuFrequencyMhz() != s.cpuMhz) {
                if (now - s.swUs < 2000000) break;
                s.cpuOff = true;
                UART_LOG("idle profile: cpu stays %u MHz, switch to %u MHz failed", s.bootMhz, s.cpuMhz);
                idleCpuRestore(now);
                break;
            }
            if (!s.w.t) {
                if (now - s.swUs >= kIdleSkipUs) idleWinStart(now);
                break;
            }
            if (now - s.w.t < kIdleRateWinUs) break;
            const float r = idleRateNow(now);
            if (std::isnan(r) && ++s.tries < 4) {
                idleWinStart(now);
                break;
            }
            if (idleCpuVerdict(s.pre, r) == IdleCpuVerdict::Keep) {
                s.cpu = CpuLow;
                UART_LOG("idle profile: cpu %u MHz, sample rate %.0f/%.0f", s.cpuMhz, r, s.pre);
                break;
            }
            s.cpuOff = true;
            UART_LOG("idle profile: cpu stays %u MHz, sample rate %.0f/%.0f at %u MHz", s.bootMhz, r, s.pre, s.cpuMhz);
            idleCpuRestore(now);
            break;
        }
        case CpuSettle: {
            // a `pwr cpu|pm` knob owns the clock now
            if (!s.suspended && getCpuFrequencyMhz() != s.bootMhz) {
                idleCpuRestore(now);
                break;
            }
            const auto n = (int32_t) (voutSamples() - s.w.n);
            if (n < 0) s.w.n = voutSamples();
            if (!s.suspended && (n < (int32_t) kIdleSettleN || now - s.w.t < kIdleSettleUs)) break;
            converter.inhibitStart(false, SynchronousConverter::InhibitIdle);
            s.cpu = CpuBoot;
            UART_LOG("idle profile: start released, %ld samples in %lu ms at %lu MHz", (long) n,
                     (unsigned long) ((now - s.w.t) / 1000), (unsigned long) getCpuFrequencyMhz());
            break;
        }
        default:
            break;
    }
}

// CPU first: a refused start waits for it, telemetry and advertising do not matter for it
static void idleProfileSet(bool on, const char *why) {
    s_idle.on = on;
    if (!on) idleCpuLeave(esp_timer_get_time());
    if (s_idle.teleMs) g_teleMinPointUs = on ? s_idle.teleMs * 1000u : kTeleMinPointUs;
#ifdef WITH_BLE
    if (s_idle.advMs) bleConsoleSetAdvMs(on ? s_idle.advMs : 0); // with a client: retried by the tick
#endif
    const char *wifi = "-";
#ifdef WITH_NETW
    if (s_wifiPs >= 0) wifi = s_wifiPs ? "MAX_MODEM" : "MIN_MODEM";
#endif
    char adv[24] = "-";
#ifdef WITH_BLE
    if (*bleConsoleName()) {
        const uint16_t a = on ? s_idle.advMs : 0;
        snprintf(adv, sizeof adv, "%u ms%s", (unsigned) bleConsoleAdvMs(), bleConsoleAdvMs() == a ? "" : " (pending)");
    }
#endif
    UART_LOG("idle profile %s%s%s: cpu %lu MHz, adv %s, tele %lu ms, WiFi %s", on ? "on" : "off", why ? " " : "",
             why ? why : "", (unsigned long) getCpuFrequencyMhz(), adv, (unsigned long) (g_teleMinPointUs / 1000), wifi);
}

void idleProfileSuspend() {
    if (s_idle.on) idleProfileSet(false, "(pwr knob)");
    s_idle.suspended = true;
}

void idleProfileHold(bool hold) {
    s_idle.hold = hold;
    if (hold && s_idle.on) idleProfileSet(false, "(ota)");
}

#if CONFIG_FUGU_WITH_PWR_KNOBS
const char *idleProfileForce(const char *arg) {
    if (s_idle.suspended) return "suspended by a pwr knob until reboot";
    if (!strcmp(arg, "auto")) s_idle.force = 0;
    else if (!strcmp(arg, "0")) s_idle.force = 1;
    else if (!strcmp(arg, "1")) s_idle.force = 2;
    else return "expected 0, 1 or auto";
    return nullptr;
}

const char *idleProfileMode() {
    static const char *const m[] = {"auto", "off", "on"};
    return s_idle.suspended ? "susp" : m[s_idle.force];
}

const char *idleCpuState() {
    static const char *const st[] = {"boot", "pre", "test", "low", "settle"};
    if (!s_idle.cpuMhz) return "-";
    return s_idle.cpuOff && s_idle.cpu == CpuBoot ? "latched" : st[s_idle.cpu];
}
#endif

// what keeps the profile off; nullptr = idle
static const char *idleBlocker() {
    // a sweep armed for the next start keeps mppt.active() true all night
    if (!converter.disabled() || adcSampler.isCalibrating()) return "(converter)";
    if (g_app.psuMode()) return "(psu)";
    if (g_app.manualPwm() && mppt.getManualTarget()) return "(manual duty)";
    if (mppt.rectCal.cal.running()) return "(rect-cal)";
    if (bfTestBusy()) return "(selftest)";
    if (const uint8_t hs = mppt.hsFaultState(); hs && hs != MpptController::HsOff) return "(hs clamp)"; // monitored each sample
    if (converter.bsyncOwnsPeriod) return "(bsync)";
    if (s_idle.hold) return "(ota)";
#ifdef WITH_BLE
    if (otaBleActive()) return "(ota)";
#endif
#ifdef WITH_MEASURE_COIL
    if (isMeasuring()) return "(measure-coil)";
#endif
#ifdef WITH_USB_MSC
    if (usbDriveMounted()) return "(usb drive)";
#endif
    return nullptr;
}

// work the daily `selftest bflow` must not interrupt
static bool selftestMaintenance() {
    if (mppt.rectCal.cal.running() || converter.bsyncOwnsPeriod || s_idle.hold) return true;
#ifdef WITH_BLE
    if (otaBleActive()) return true;
#endif
#ifdef WITH_MEASURE_COIL
    if (isMeasuring()) return true;
#endif
#ifdef WITH_USB_MSC
    if (usbDriveMounted()) return true;
#endif
    return false;
}

static void idleProfileTick(time_ms nowMs) {
    static time_ms busyMs = 0;
    static uint32_t idleOnlyMs = 0;
    const bool rtWake = converter.takeStartWanted(); // the RT loop refused a start (InhibitIdle)
    const bool bsWake = idleBackstopWake(converter.startInhibited(SynchronousConverter::InhibitIdle) &&
                                         !g_app.manualPwm() && mppt.startBlockReason() == MpptController::kIdleBlock,
                                         (uint32_t) nowMs, idleOnlyMs);
    const bool wake = rtWake || bsWake;
    if (!s_idle.suspended) {
        const char *why = rtWake ? "(start)" : bsWake ? "(start, backstop)" : idleBlocker();
        if (why) busyMs = nowMs;
        if (wake && s_idle.force == 2) s_idle.force = 0; // a start ends a forced profile
        const bool want = s_idle.force ? s_idle.force == 2 && !why : nowMs - busyMs > s_idleAfterMs;
        if (want != s_idle.on) idleProfileSet(want, why ? why : s_idle.force ? "(pwr idle)" : nullptr);
#ifdef WITH_BLE
        else if (s_idle.advMs && bleConsoleAdvMs() != (want ? s_idle.advMs : 0))
            bleConsoleSetAdvMs(want ? s_idle.advMs : 0);
#endif
    }
    idleCpuTick();
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
        netGuardTick(false);
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
        wifiLoop(mppt.ucTemp.lastFresh() < 80);

        // self-heal: bring up enabled network services on the WiFi-up edge (they fail to start
        // at boot when WiFi isn't connected yet). _wifiConnected() has set up MDNS by now.
        static bool wifiWasUp = false;
        bool wifiUp = wifiIsConnected();
        if (wifiUp && !wifiWasUp) { g_services.startEnabledNetworkServices(); s_wifiPs = -1; } // connect reset PS to MIN
        wifiWasUp = wifiUp;
        netGuardTick(wifiUp);

        // Night = not converting for a while: drop to MAX_MODEM to cut standby battery draw. Back to
        // MIN_MODEM the instant PV power returns so daytime console/MQTT/OTA stay responsive.
        static time_ms lastActiveMs = 0;
        time_ms nowMs = wallClockMs();
        if (converting) lastActiveMs = nowMs;
        int8_t want = (nowMs - lastActiveMs > 5u * 60 * 1000) ? 1 : 0;
        if (s_idle.suspended) s_wifiPs = want;
        if (converter.bsyncOwnsPeriod) s_wifiPs = -1; // bsync holds WIFI_PS_NONE, re-assert after it stops
        else if (wifiUp && want != s_wifiPs) {
            s_wifiPs = want;
            wifiSetPowerSave(want);
            UART_LOG("WiFi power save: %s", want ? "MAX_MODEM (night/idle)" : "MIN_MODEM (active)");
        }
    }
#endif
}

// True while a TCP scope client is attached and its blocking netLoop() will provide the cooperative
// yield this iteration; otherwise loopNetwork_task must yield explicitly.
static bool scopeKeepsAwake() {
#if WITH_SCOPE
    return scopeService.state() == ServiceState::Running && scopeService.hasClient();
#else
    return false;
#endif
}

static void loopNetwork_task(void *arg) {
    //ESP_LOGI("main", "Net loop running on core %i", xPortGetCoreID());
    assert(xPortGetCoreID() == 0);

    // loopRT is the ONLY writer of loopWallClockUs_, and it is never created when setup() failed
    // (the g_app.setupErr gate on xTaskCreatePinnedToCore above). wallClockMs() then stays frozen
    // forever and every consumer pacing off it stops silently -- no error, no log. The one that
    // bites is bleTxDrain(): its TX_SETTLE_MS window can never expire, so the BLE console accepts
    // commands, dispatches them and produces the reply, then emits ZERO notifications. The console
    // dies exactly when it is most needed, on a board too broken to bring up its converter.
    // otaBleTick/teleBleTick/teleAdvTick pace off the same value and stop with it.
    // MEASURED 2026-09-10, S3 with no charger.conf: loopWallClockUs_ pinned at 224388 us across
    // 17 s of uptime; adding this line it tracks uptime again and notifications resume.
    // Guarded by setupErr so it can never race loopRT -- the guard is true only when loopRT does
    // not exist. Cheap: one esp_timer_get_time() per network-loop iteration, and only when broken.
    if (g_app.setupErr) setWallClockUs(esp_timer_get_time());

    auto nowMs(wallClockMs());
    idleProfileTick(nowMs); // first: a converter start leaves the profile before any long job below

    loopUart(nowMs);
    if (mppt.sensorPhysicalI) selftestUrgent(nowMs), bfTestTick((uint32_t) nowMs);
#ifdef WITH_USB_MSC
    switch (usbDevPoll()) {
        case UsbEvent::ConfCommitted:
            mppt.rectCal.discard();
            if (mppt.active()) UART_LOG("conf changed, restart to apply");
            else systemRestart();
            break;
        case UsbEvent::EnterDownload:
            converter.disable();
            usbEnterDownload();
        default:
            break;
    }
#endif
    flush_async_uart_log();
    process_queued_tasks();

    networkLoopTick();

    // ftp / telnet / telemetry / lcd / scope ticks (only the Running ones do work)
    g_services.tickAll();
    teleMqttTick(nowMs);


    mppt.tempPollStale(wallClockUs());

    // Control cadence is fixed and console-independent; the status line keeps its own (slower when
    // idle) pacing inside loopLF.
    // !lastTimeOutUs: telnet onConnect zeroes it to force an immediate status line.
    if ((wallClockUs() - lastLfWorkUs) >= lfPeriod or !lastLfWorkUs or !lastTimeOutUs) {
        loopLF(wallClockUs(), false);
        // HA power publish moved to MqttService::onTick (MQTT.tickHook, wired in setup()).
        lastLfWorkUs = wallClockUs();
    }

#if CONFIG_FUGU_WITH_PWR_KNOBS
    pwrTick();
#endif

    // Preserve the cooperative yield: scope's netLoop() blocks ~1 tick when a client is attached
    // and serves as the yield; otherwise we must yield explicitly.
    if (!scopeKeepsAwake())
#if CONFIG_FUGU_WITH_PWR_KNOBS
        vTaskDelay(pdMS_TO_TICKS(g_pwrNetLoopMs));
#else
        vTaskDelay(pdMS_TO_TICKS(1));
#endif
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
    converter.inhibitStart(true, SynchronousConverter::InhibitBoot);
    converter.disable();
    if (xPortGetCoreID() != RT_CORE)
        mppt.scanSave(), mppt.rectCal.saveFinal(converter, (uint32_t) (wallClockUs() / 1000000));
    if (g_restartBlocked) {
        converter.inhibitStart(false, SynchronousConverter::InhibitBoot);
        ESP_LOGE("main", "restart refused: bootloader region not verified");
        return;
    }
    mppt.meter.commit(false, true); // energy counters are otherwise only committed at sweep start / UV
    UART_LOG("Rebooting");
    restartCloseTelnet();
    g_services.stopAll(); // tear down enabled services while WiFi is still up (see mqtt_task overflow)
    delay(300);
    ESP.restart();
}


[[maybe_unused]] void esp_task_wdt_isr_user_handler() {
    //throw std::runtime_error("reboot");
    if (esp_cpu_dbgr_is_attached()) return;

    rt_defer([](void *, float, float) {
        ESP_LOGE("main", "Restart after WDT trigger");
        systemRestart();
    }, nullptr);
}
