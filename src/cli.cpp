#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

#include "cli.h"

#include <Arduino.h>

#include "tele/telemetry_service.h"
#ifdef CONFIG_FUGU_WITH_NETW
#include <WiFi.h>
#endif
#include <cmath>
#include <cinttypes>
#include <cstring>
#include <cstdio>
#include <dirent.h>
#include <sys/stat.h>
#include <esp_timer.h>
#include <esp_memory_utils.h>
#include <esp_core_dump.h>
#include <esp_partition.h>
#include <esp_flash_partitions.h>
#include <esp_system.h>
#include <nvs.h>
#include <ctime>
#include <esp_ota_ops.h>
#include <esp_flash.h>
#include <esp_chip_info.h>
#include "etc/flash_probe.h"
#if CONFIG_FUGU_WITH_RTPROF
#include "etc/rtprof.h"
extern volatile uint8_t g_rtLogTest;
#endif
#ifdef CONFIG_BOOTGUARD_ENABLE
#include "bootguard.h"
#endif
#include <esp_heap_caps.h>
#if CONFIG_HEAP_TASK_TRACKING
#include <esp_heap_task_info.h>
#endif
#include <esp_log.h>
#include <freertos/task.h>
#include <SimpleCLI.h>

#include "logging.h"
#include "console.h"
#include "conf.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#if CONFIG_IDF_TARGET_ESP32S3
#include "esp_efuse_rtc_calib.h"
#endif
#include "esp_private/esp_gpio_reserve.h"
#include "util.h"
#include "buck.h"
#include "mppt.h"
#include "tele/console_ble.h"
#ifdef CONFIG_FUGU_WITH_MEASURE_COIL
#include "selftest/measure_coil.h"
#endif
#include "selftest/bflow_test.h"
#include "etc/version.h"
#include "adc/sampling.h"
#include "service.h"
#include "viz/led.h"
#include "storage/key-value.h"
#include "conf_hash.h"
#include "etc/con_guard.h"
#include "etc/ota.h"            // doOta, otaPendingVerify
#ifdef CONFIG_FUGU_WITH_NETW
#include "tele/telemetry.h"     // connect_wifi_async, add_ap
#include "etc/network_shim.h"
#include "etc/rt_task.h"
#include "etc/idle_profile.h"
#include <lwip/netif.h>
#include <lwip/pbuf.h>
#endif
#ifdef CONFIG_FUGU_WITH_NETTOOLS
#include <esp_http_client.h>
#if CONFIG_FUGU_WITH_HTTPS
#include <esp_crt_bundle.h>
#endif
#include <ping/ping_sock.h>
#include <lwip/netdb.h>
#include <lwip/ip_addr.h>
#include <lwip/sockets.h>
#include <fcntl.h>
#include <errno.h>
#endif
#include <ota_ble.h>            // otaBleSubmitCommand (OTA push over BLE, esp-ota-ble component)
#include "tele/tele_ble.h"      // teleBleSetStreaming/teleBleStreaming (BLE telemetry stream)
#include "tele/tele_mqtt.h"
#include <sys/time.h>
#if CONFIG_FUGU_WITH_VCONV
#include "sim/vconv.h"
#endif
#if CONFIG_FUGU_WITH_PWR_KNOBS
#include "etc/pwr_knobs.h"
#endif

#include "app_state.h"

// Components owned by main.cpp. The mode flags (g_app.manualPwm/g_app.disableWifi/g_app.wifiReenableMs/g_app.maxLoopLag
// etc.) live in `g_app` (app_state.h); these here are the singleton components — the right
// granularity to keep as named externs rather than bundling further.
extern SynchronousConverter converter;
extern MpptController mppt;
extern ADC_Sampler adcSampler;
extern VIinVout<const Sensor *> sensors;
extern LedIndicator led;
extern KeyValueStorage nvs;

#define WITH_PWM_DIAGNOSTICS 1

// Defined in main.cpp (non-static so we can reach them from here).
void systemRestart();
void netGuardPrint();

void stopAndBackoff(uint32_t secondsDelay, const char *who = "stopAndBackoff");
extern std::atomic<bool> g_wireResetReq;

void loopLF(const time_us &nowUs, bool interim); // both call sites pass interim explicitly

static SimpleCLI cli;

// --- Console command callbacks ---------------------------------------------------------------
// use CMD_FAIL() in command dispatchers to flag that command as failed
// SimpleCLI's dispatch API has no return value

static bool s_cmdFailed = false;
#define CMD_FAIL_RETURN(...) do { ESP_LOGW("main", __VA_ARGS__); s_cmdFailed = true; return; } while (0)

// a running measure-coil owns the converter: refuse commands that change its mode or drive
static bool measBusy(cmd *c) {
#ifdef CONFIG_FUGU_WITH_MEASURE_COIL
    if (isMeasuring()) {
        ESP_LOGW("main", "%s: refused, measure-coil running", Command(c).getName().c_str());
        s_cmdFailed = true;
        return true;
    }
#endif
    return false;
}

static bool argInt(Command &cc, int i, int &out) {
    if (i >= cc.countArgs()) return false;
    auto s = cc.getArg(i).getValue();
    char *end;
    long v = strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end) return false;
    out = (int) v;
    return true;
}

// Whole-token float parse; toFloat() would turn "abc" into 0 and "24V" into 24.
static bool argFloat(Command &cc, int i, float &out) {
    if (i >= cc.countArgs()) return false;
    auto s = cc.getArg(i).getValue();
    char *end;
    float v = strtof(s.c_str(), &end);
    if (end == s.c_str() || *end || !std::isfinite(v)) return false;
    out = v;
    return true;
}

static bool waitPsuCommand(uint32_t ticket, uint32_t timeoutMs = 1000) {
    // not wallClockMs(): only loopRT advances it, so without a running RT loop this never times out
    const int64_t deadline = esp_timer_get_time() + (int64_t) timeoutMs * 1000;
    while (!mppt.isPsuCommandDone(ticket) && esp_timer_get_time() < deadline)
        delay(1);
    if (mppt.isPsuCommandDone(ticket)) return true;
    if (mppt.cancelPsuCommand(ticket)) return false;
    for (int i = 0; i < 10 && !mppt.isPsuCommandDone(ticket); ++i) delay(1);
    return mppt.isPsuCommandDone(ticket);
}

static const char *psuErrorText(PsuSetpointError error) {
    switch (error) {
        case PsuSetpointError::None: return "none";
        case PsuSetpointError::OutOfRange: return "out of range";
        case PsuSetpointError::TelemetryUnavailable: return "fresh Vin telemetry unavailable";
        case PsuSetpointError::BoostBelowInput: return "boost setpoint must exceed Vin by 0.5 V";
        case PsuSetpointError::OvLimitConflict: return "setpoint conflicts with explicit OV limit";
    }
    return "unknown error";
}

static void cmdSync(cmd *c) {
    auto arg = Command(c).getArg(0).getValue();
    if (arg.isEmpty()) {
        const char *fpwm = !converter.forcedPwmRequested()
                               ? "off"
                               : (!converter.forcedPwmGateEnabled()
                                      ? "on (ungated)"
                                      : (converter.forcedPwmGateArming()
                                             ? "arming"
                                             : (converter.forcedPwmGated() ? "armed" : "engaged")));
        UART_LOG("sync rect %s, forced pwm %s (D=%.3f, engage>=%.3f) pwmLS=%hu",
                 converter.syncRectEnabled_() ? "on" : "off", fpwm,
                 converter.dutyCtrlEff(), converter.forcedPwmEngageDuty(),
                 converter.getRectOnPwmCnt());
        return;
    }
    if (measBusy(c)) return;
    if (!g_app.manualPwm())
        CMD_FAIL_RETURN("sync: only in manual PWM (use 'dc N' first)");
    auto on = arg == "on" or arg == "1";
    if (on or arg == "off" or arg == "0") {
        converter.forcedPwm_(false);
        converter.enableSyncRect(on, true);
    } else if (arg == "forced" or arg == "forced!") {
        // Plain `forced` arms the bring-up gate: the LS stays diode-emulating until the duty is
        // above the voltage ratio, so a ramp from 0 cannot boost back into the source. `forced!`
        // engages immediately - only for an output that cannot sink (open output, no loop).
        const bool immediate = (arg == "forced!");
        converter.forcedPwm_(true, immediate);
        converter.enableSyncRect(true);
        if (!immediate && converter.forcedPwmGated())
            UART_LOG("forced PWM armed, engages above D=%.3f (now %.3f)",
                     converter.forcedPwmEngageDuty(), converter.dutyCtrlEff());
    } else {
        CMD_FAIL_RETURN("sync: expected on|off|forced|forced!");
    }
}

static void cmdBflow(cmd *c) {
    if (measBusy(c)) return;
    if (!g_app.manualPwm())
        CMD_FAIL_RETURN("bf: only in manual PWM (use 'dc N' first)");
    if (!mppt.bflow)
        CMD_FAIL_RETURN("backflow switch not configured");
    Command cc(c);
    int newState;
    if (!argInt(cc, 0, newState) or (newState != 0 and newState != 1))
        CMD_FAIL_RETURN("bf: expected 0|1");
    if (mppt.bflow.state() != newState)
        ESP_LOGI("main", "Set bflow state %i", (int) newState);
    mppt.bflow.enable(newState);
}

static void cmdRestart(cmd *) { systemRestart(); }

static void cmdMppt(cmd *c) {
    if (measBusy(c)) return;
    if (!g_app.psuMode() && !g_app.manualPwm() && !mppt.hasPendingPsuCommand()) {
        CMD_FAIL_RETURN("MPPT already enabled");
    }
    const auto ticket = mppt.requestPsuOff();
    if (!waitPsuCommand(ticket))
        CMD_FAIL_RETURN("mppt: RT transition timed out");
    ESP_LOGI("main", "MPPT re-enabled");
}

// dc <hs> [ls]  — manual PWM. With no [ls] the low side is automatic (diode emulation); with
// [ls] the low-side on-count is pinned to that value (bench LS-timing sweep). ls<0 -> auto.

static void cmdDc(cmd *c) {
    if (bfTestBusy())
        CMD_FAIL_RETURN("dc: selftest running (mppt aborts it)");
#ifdef CONFIG_FUGU_WITH_MEASURE_COIL
    if (isMeasuring())
        CMD_FAIL_RETURN("dc: busy measuring");
#endif
    Command cc(c);
    auto v = cc.getArg(0).getValue();
    if (v.length() == 0)
        CMD_FAIL_RETURN("dc: expected <hs> [ls]");
    int dc;
    if (!argInt(cc, 0, dc) || dc < 0 || dc > converter.pwmCtrlMax)
        CMD_FAIL_RETURN("dc: out of range [0,%i]", (int) converter.pwmCtrlMax);

    // `dc 0` is the shutdown escape hatch and must always work — a sweep runs its calibration
    // phase, so refusing it there is exactly when the user needs it (e.g. to break out of a
    // converter that keeps tripping a protection). Only powering up needs a settled sampler.
    if (dc != 0 && adcSampler.isCalibrating())
        CMD_FAIL_RETURN("dc: busy calibrating");

    int manualRect = -1;
    if (cc.countArgs() >= 2 && dc > 0 && (!argInt(cc, 1, manualRect) || manualRect < 0))
        CMD_FAIL_RETURN("dc: bad ls count");

    // dc 0 stops at once: ramping below Vout/Vin in CCM at power reverses the coil current (highD-bflowOpen)
    const auto ticket = mppt.requestPsuManual(dc, manualRect, false, dc == 0);
    if (!waitPsuCommand(ticket))
        CMD_FAIL_RETURN("dc: RT transition timed out");
    ESP_LOGI("main", "Switched to manual PWM");
    if (manualRect >= 0)
        UART_LOG("Manual LS=%i held (HS=%i); reverse-current risk, bench only",
                 manualRect, (int) dc);
}

// dt [hl_ns [lh_ns]] — MCPWM hardware dead-time, one value per transition (HS->LS / LS->HS).
// One argument sets both. No argument reports the live values. The change is applied by the RT
// core and is RAM-only; `set-config board.conf pwm_deadtime_hl_ns|pwm_deadtime_lh_ns <ns>` makes
// it survive.
static void cmdDeadTime(cmd *c) {
    Command cc(c);
    auto v = cc.getArg(0).getValue();
    if (v.length()) {
#ifdef CONFIG_FUGU_WITH_MEASURE_COIL
        // measure-coil snapshots pwmMax/pwmCtrlMax for its duty math and may persist the result
        if (isMeasuring())
            CMD_FAIL_RETURN("dt: busy measuring");
#endif
        // toFloat() stops at the first non-numeric character and reports no error, so "300,9" and
        // "80O" would silently become 300 and 80. Require the whole token to parse.
        char *end = nullptr;
        const float hlNs = strtof(v.c_str(), &end);
        if (end == v.c_str() || *end)
            CMD_FAIL_RETURN("dt: expected dead-time in ns");
        float lhNs = hlNs;
        if (cc.countArgs() >= 2) {
            auto v2 = cc.getArg(1).getValue();
            lhNs = strtof(v2.c_str(), &end);
            if (end == v2.c_str() || *end)
                CMD_FAIL_RETURN("dt: expected dead-time in ns");
        }
        // Shortening a dead-band is the direction that can short the half-bridge, so it stays a
        // bench operation -- lowering EITHER edge needs manual PWM. Lengthening only adds margin
        // and is allowed in any mode. Compare quantized ticks, not ns -- a request that lands on
        // the current tick count is a no-op.
        if ((converter.deadTimeTicksFor(hlNs) < converter.getDtHlTicks()
             || converter.deadTimeTicksFor(lhNs) < converter.getDtLhTicks())
            && !g_app.manualPwm())
            CMD_FAIL_RETURN("dt: lowering the dead-time needs manual PWM (use 'dc N' first)");
        uint16_t hlTicks = 0, lhTicks = 0;
        if (const char *err = converter.requestDeadTimeNs(hlNs, lhNs, hlTicks, lhTicks))
            CMD_FAIL_RETURN("dt: %s", err);
        // esp_timer_get_time(), not wallClockMs(): the latter is only advanced by loopRT itself, so
        // a wedged RT loop -- the one failure this deadline exists to report -- would stop the very
        // clock measuring it and spin here forever, taking the whole network task down with it.
        const int64_t deadline = esp_timer_get_time() + 1000000;
        while (!converter.deadTimeIdle() && esp_timer_get_time() < deadline) delay(1);
        // No cancel on timeout: withdrawing a request the RT core may already be applying could
        // hand it back the PREVIOUS value. Report it as unconfirmed, still queued.
        if (!converter.deadTimeIdle())
            CMD_FAIL_RETURN("dt: RT loop did not confirm within 1 s (request still queued)");
        if (converter.getDtHlTicks() != hlTicks || converter.getDtLhTicks() != lhTicks)
            CMD_FAIL_RETURN("dt: driver rejected hl=%u lh=%u ct, still hl=%u lh=%u ct",
                            (unsigned) hlTicks, (unsigned) lhTicks,
                            (unsigned) converter.getDtHlTicks(), (unsigned) converter.getDtLhTicks());
    }
    if (!converter.hasDeadTime())
        CMD_FAIL_RETURN("dt: n/a, %s has no dead-time module",
                        converter.isEnLogic() ? "an InEn gate driver" : "this gate driver");
    const unsigned hl = converter.getDtHlTicks(), lh = converter.getDtLhTicks();
    if (!hl && !lh)
        CMD_FAIL_RETURN("dt: module bypassed at boot (board.conf pwm_deadtime_hl_ns=0)");
    // realized HS->LS gap is one tick short of the configured value, see setDeadTimeTicks();
    // the LS->HS band is periodTicks - cmpLS, so with cmpLS capped at pwmMax-1 it is one tick wide
    UART_LOG("dt hl=%.0f ns (%u ct, gap %u ct) lh=%.0f ns (%u ct) pwmMax=%u minLS=%u maxHS=%u",
             converter.getDeadTimeHlNs(), hl, hl ? hl - 1 : 0u,
             converter.getDeadTimeLhNs(), lh, (unsigned) converter.pwmMaxDriver(),
             (unsigned) converter.getRectOnPwmMin(), (unsigned) converter.pwmCtrlMax);
}

// pwm-freq [hz] — MCPWM switching frequency, changed live (no reboot). No argument reports the
// REALIZED frequency. The change is applied by the RT core and is RAM-only; `set-config board.conf
// pwm_freq <hz>` makes it survive a power cycle.
//
// Duty is carried through as a ratio, so the operating point survives — that is the whole point:
// on the recirculating power-loop rig a reboot of the buck is a reversed shutdown and collapses the
// loop.
static void cmdPwmFreq(cmd *c) {
    Command cc(c);
    auto v = cc.getArg(0).getValue();
    if (v.length()) {
#ifdef CONFIG_FUGU_WITH_MEASURE_COIL
        // measure-coil snapshots pwmMax/pwmCtrlMax for its duty math and may persist the result
        if (isMeasuring())
            CMD_FAIL_RETURN("pwm-freq: busy measuring");
#endif
        // toFloat()/toInt() stop at the first non-numeric character and report no error, so "39k"
        // would silently become 39. Require the whole token to parse.
        char *end = nullptr;
        errno = 0;
        const long hz = strtol(v.c_str(), &end, 10);
        // Bound before narrowing to uint32_t. `long` is 32-bit on the target so a huge input
        // saturates and the range guard catches it, but on an LP64 host build 4295006296 would
        // truncate to 39000 and pass — a far-tail input flipping the verdict back to accept.
        if (end == v.c_str() || *end || errno == ERANGE || hz <= 0 || hz > 1000000L)
            CMD_FAIL_RETURN("pwm-freq: expected a frequency in Hz");
        uint16_t ticks = 0;
        if (const char *err = converter.requestPwmFrequency((uint32_t) hz, ticks))
            CMD_FAIL_RETURN("pwm-freq: %s", err);
        // esp_timer_get_time(), not wallClockMs(): the latter is only advanced by loopRT itself, so
        // a wedged RT loop -- the one failure this deadline exists to report -- would stop the very
        // clock measuring it and spin here forever, taking the whole network task down with it.
        const int64_t deadline = esp_timer_get_time() + 1000000;
        while (!converter.pwmFreqIdle() && esp_timer_get_time() < deadline) delay(1);
        // No cancel on timeout: withdrawing a request the RT core may already be applying could
        // hand it back the PREVIOUS period. Report it as unconfirmed, still queued.
        if (!converter.pwmFreqIdle())
            CMD_FAIL_RETURN("pwm-freq: RT loop did not confirm within 1 s (request still queued)");
        if (converter.getPeriodTicks() != ticks)
            CMD_FAIL_RETURN("pwm-freq: driver rejected %u ct, still %u ct",
                            (unsigned) ticks, (unsigned) converter.getPeriodTicks());
    }
    if (!converter.getPeriodTicks())
        CMD_FAIL_RETURN("pwm-freq: n/a, needs the MCPWM gate driver build");
    UART_LOG("pwm-freq %.2f Hz period_ticks=%u res=%lu pwmMax=%u hs_off=%u maxHS=%u nominal=%lu",
             converter.realizedFreqHz(), (unsigned) converter.getPeriodTicks(),
             (unsigned long) converter.getPwmResolutionHz(), (unsigned) converter.pwmMaxDriver(),
             (unsigned) converter.getCtrlOnPwmCnt(), (unsigned) converter.pwmCtrlMax,
             (unsigned long) converter.getPwmFrequency());
}

static void cmdShortLs(cmd *c) {
    if (measBusy(c)) return;
    if (converter.boost() && abs(sensors.Vin->ewm.avg.get()) < 0.05) {
        const auto ticket = mppt.requestPsuShortLowSide();
        if (!waitPsuCommand(ticket))
            CMD_FAIL_RETURN("short-ls: RT transition timed out");
    } else {
        CMD_FAIL_RETURN("short-ls: requires boost mode and Vin~0");
    }
}

static void cmdSpeed(cmd *c) {
    Command cc(c);
    float speedScale;
    if (argFloat(cc, 0, speedScale) && speedScale >= 0 && speedScale < 10) {
        mppt.speedScale = speedScale;
        ESP_LOGI("main", "Set tracker speed scale %.4f", speedScale);
    } else {
        CMD_FAIL_RETURN("speed: out of range [0,10)");
    }
}

static void cmdFan(cmd *c) {
    Command cc(c);
    float pct;
    if (!argFloat(cc, 0, pct) || !mppt.fan.fanSet(pct * 0.01f))
        CMD_FAIL_RETURN("fan: set failed");
}

static void cmdLed(cmd *c) { led.setRGB(Command(c).getArg(0).getValue().c_str()); }

// Pins board.conf assigns (gates, driver enable, I2C, alerts, fan, LED) are off-limits for the pin diagnostics.
static bool boardOwnsPin(int pin) {
    try {
        ConfFile board{"/littlefs/conf/board.conf", true};
        for (auto k: {"pwm_hi", "pwm_li", "pwm_in", "pwm_en", "pwm_sd", "bflow_sd", "bflow_en", "lv_pgood", "pwm_fault_pin", "pwm_sync_pin",
                      "i2c_sda", "i2c_scl", "ina226_alert", "ads_alert", "fan_pwm", "led_WS2812", "led_simple"})
            if (board.getLong(k, -1) == pin) return true;
        return false;
    } catch (...) {
        return true;
    }
}

// ADC1 pads may carry internal-ADC sense inputs (driving one blinds protection); flash/PSRAM pins are reserved.
static bool freeOutputPin(int pin) {
    if (pin < 0 || pin >= SOC_GPIO_PIN_COUNT || !GPIO_IS_VALID_OUTPUT_GPIO(pin) || esp_gpio_is_reserved(BIT64(pin)))
        return false;
    adc_unit_t unit;
    adc_channel_t ch;
    if (adc_oneshot_io_to_channel(pin, &unit, &ch) == ESP_OK && unit == ADC_UNIT_1)
        return false;
    return !boardOwnsPin(pin);
}

// Bench-only: `gpio <pin> <0|1>` -- direct digitalWrite test. Bypasses MCPWM/LEDC.
static void cmdGpio(cmd *c) {
    Command cc(c);
    int pin, val;
    if (cc.countArgs() != 2 || !argInt(cc, 0, pin) || !argInt(cc, 1, val) || (val != 0 && val != 1))
        CMD_FAIL_RETURN("gpio <pin> <0|1>");
    if (!freeOutputPin(pin))
        CMD_FAIL_RETURN("gpio: pin %d invalid or assigned in board.conf", pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, val);
    UART_LOG("gpio %u -> %u", (unsigned) pin, (unsigned) val);
}

// Bench-only: `mcpwmtest <pin>` -- minimal IDF MCPWM example pattern on a single pin.
// 1 kHz 50% duty, no dead-time, no fault. Verifies the MCPWM peripheral itself.
#include "driver/mcpwm_prelude.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/mcpwm_reg.h"

#define MT_CHECK(x) do { esp_err_t e_ = (x); if (e_ != ESP_OK) CMD_FAIL_RETURN("mcpwmtest: %s", esp_err_to_name(e_)); } while (0)

static void cmdMcpwmTest(cmd *c) {
    Command cc(c);
    int pin;
    if (!argInt(cc, 0, pin) || !freeOutputPin(pin))
        CMD_FAIL_RETURN("mcpwmtest: pin invalid or assigned in board.conf");
    static mcpwm_timer_handle_t timer = nullptr;
    static mcpwm_oper_handle_t oper = nullptr;
    static mcpwm_cmpr_handle_t cmp = nullptr;
    static mcpwm_gen_handle_t gen = nullptr;
    if (timer) {
        CMD_FAIL_RETURN("already created; reboot to re-test");
        return;
    }
    mcpwm_timer_config_t tc = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 80'000'000, .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 2048, .intr_priority = 0, .flags = {}
    };
    MT_CHECK(mcpwm_new_timer(&tc, &timer));
    mcpwm_operator_config_t oc = {.group_id = 0, .intr_priority = 0, .flags = {}};
    MT_CHECK(mcpwm_new_operator(&oc, &oper));
    MT_CHECK(mcpwm_operator_connect_timer(oper, timer));
    mcpwm_comparator_config_t cc2 = {.intr_priority = 0, .flags = {}};
    MT_CHECK(mcpwm_new_comparator(oper, &cc2, &cmp));
    MT_CHECK(mcpwm_comparator_set_compare_value(cmp, 1024));
    mcpwm_generator_config_t gc = {.gen_gpio_num = pin, .flags = {}};
    MT_CHECK(mcpwm_new_generator(oper, &gc, &gen));
    MT_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    MT_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_LOW)));
    MT_CHECK(mcpwm_timer_enable(timer));
    MT_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    UART_LOG("mcpwmtest: 1kHz 50%% on GPIO %d (group 0)", pin);
}

static void cmdSweep(cmd *c) {
    if (measBusy(c)) return;
    Command cc(c);
    if (cc.countArgs()) {
        if (cc.getArg(0).getValue() != "calfail") CMD_FAIL_RETURN("usage: sweep [calfail]");
        adcSampler.forceCalibFailure();
        UART_LOG("sweep: next calibration forced to fail");
    }
    const auto ticket = mppt.requestPsuSweep();
    if (!waitPsuCommand(ticket))
        CMD_FAIL_RETURN("sweep: RT transition timed out");
    // the reason is only valid once calibration is done and the filters refilled
    const int64_t until = esp_timer_get_time() + 3000000;
    while (converter.disabled() && esp_timer_get_time() < until) {
        if (!adcSampler.isCalibrating() && !mppt.filtersEmpty()) {
            const char *r = mppt.startBlockReason();
            if (r && r != MpptController::kIdleBlock)
                UART_LOG("sweep: queued, START blocked: %s (Vin=%.1f Vout=%.1f)", r,
                         sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get());
            break;
        }
        delay(10);
    }
}

static void cmdSweepCurve(cmd *) {
    const bool sw = mppt.isSweeping();
    const Plot &pl = mppt.sweepCurve();
    // sweeping: the points so far, else the last completed sweep
    const PlotPoints *b = sw ? pl.block : pl.last();
    const uint16_t n = b ? sw ? b->n[0] : b->kn : 0;
    if (!n) {
        UART_LOG("sc: none");
        return;
    }
    const auto &m = mppt.sweepMpp();
    const uint8_t f = sw ? 0 : mppt.sweepFlag();
    char fl[32] = "";
    if (f) snprintf(fl, sizeof fl, " flag=%s land=%.1f", f == 2 ? "light" : "0", mppt.sweepLand());
    UART_LOG("sc: t=%lu n=%u mpp=%.1f,%.1f sweeping=%d%s", sw ? 0ul : (unsigned long) b->endTimeS, (unsigned) n,
             sw ? m.power : b->kMpp[1], sw ? m.voltage : b->kMpp[0], (int) sw, fl);
    char buf[192];
    int len = 0, k = 0;
    for (uint16_t i = 0; i < n; ++i) {
        if (!len) len = snprintf(buf, sizeof(buf), "sc:");
        const auto q = sw ? b->p[0][i] : b->kept(i);
        len += snprintf(buf + len, sizeof(buf) - len, " %.2f %.1f", q.first, q.second);
        if (++k == 12 || len > 140 || i + 1 == n) {
            UART_LOG("%s", buf);
            len = k = 0;
        }
    }
    UART_LOG("sc: end");
}

// rect-cal [force|status|abort] — self-calibrate the DCM low-side timing offset (rect_cal.h)
static void cmdRectCal(cmd *c) {
    Command cc(c);
    const String sub = cc.countArgs() ? cc.getArg(0).getValue() : String("");
    auto &rc = mppt.rectCal;
    if (sub == "status") { rc.status(converter); return; }
    if (sub == "abort") {
        if (!rc.cal.running()) CMD_FAIL_RETURN("rect-cal: not running");
        rc.abortReq.store(true);
        return;
    }
    const bool force = sub == "force";
    if (sub.length() && !force) CMD_FAIL_RETURN("rect-cal: expected [force|status|abort]");
    if (measBusy(c)) return;
    if (!rc.requestStart(force)) CMD_FAIL_RETURN("rect-cal: already running");
    for (int i = 0; i < 100 && rc.cal.get() == RectCal::Idle; ++i) delay(10);
    const auto st = rc.cal.get();
    if (st != RectCal::Run)
        CMD_FAIL_RETURN("rect-cal: refused: %s", st == RectCal::Fail ? rc.cal.why : "no RT response");
    ESP_LOGI("main", "rect-cal: running at hs=%u (up to 60 s), see rect-cal status", rc.cal.hs);
}

static void cmdResetLag(cmd *c) {
    g_app.maxLoopLag = 0;
    g_flashLagUs[0] = g_flashLagUs[1] = 0;
    g_flashLagPeak = {};
#if CAPTURE_LOOP_DT
    g_app.maxLoopDT = 0;
#endif
    if (!Command(c).countArgs()) rtcount_print(true);
}

#ifdef CONFIG_FUGU_WITH_NETW
static void cmdWifi(cmd *c) {
    Command cc(c);
    auto arg = cc.getArg(0).getValue();
    if (otaBleActive())
        CMD_FAIL_RETURN("wifi: BLE OTA in progress, WiFi stays off until its reboot");
    if (arg == "on") {
        g_app.wifiReenableMs = 0;
        g_app.disableWifi = false;
        nvs.open();
        if (!nvs.readString("wifi_off", "").empty())
            nvs.writeString("wifi_off", "");
        nvs.close();
        connect_wifi_async();
    } else if (arg == "off") {
        // "off <minutes>" disables temporarily and keeps the saved ssid for reconnect;
        // bare "off" disables for good (persisted in nvs, survives reboot) and forgets the sticky ssid.
        // The actual WiFi/netif teardown is deferred to networkLoopTick's WiFi-down edge so it never
        // runs inside this console/telnet input callback: deiniting the netif under the telnet socket
        // the command arrived on frees lwip pbufs out from under the log mirror (InstrFetch UAF).
        long mins = cc.countArgs() >= 2 ? cc.getArg(1).getValue().toInt() : 0;
        g_app.disableWifi = true;
        if (mins > 0) {
            g_app.wifiReenableMs = wallClockMs() + (uint32_t) mins * 60000;
            UART_LOG("WiFi off for %ld min", mins);
        } else {
            g_app.wifiReenableMs = 0;
            nvs.open();
            if (!nvs.readString("wifi_ssid", "").empty())
                nvs.writeString("wifi_ssid", "");
            nvs.writeString("wifi_off", "1");
            nvs.close();
            UART_LOG("WiFi off (persisted, `wifi on` to re-enable)");
        }
    } else {
        CMD_FAIL_RETURN("wifi: expected on|off [minutes]");
    }
}

static void cmdWifiAdd(cmd *c) {
    auto ssidAndPw = Command(c).getArg(0).getValue();
    auto i = ssidAndPw.indexOf(':');
    if (i <= 0)
        CMD_FAIL_RETURN("wifi-add: expected ssid:password");
    std::string ssid = ssidAndPw.substring(0, i).c_str();
    auto psk = ssidAndPw.substring(i + 1);
    ESP_LOGI("main", "adding wifi network %s", ssid.c_str());
    if (!add_ap(ssid, psk.c_str()))
        CMD_FAIL_RETURN("wifi-add: not stored (bad chars or write error)");
}
#endif

static void cmdScanI2c(cmd *) { scan_i2c(); }

// ls/cat operate on the littlefs tree. A bare or relative arg is taken under /littlefs/.
static std::string lfsPath(const String &arg) {
    std::string p = arg.length() ? arg.c_str() : "/littlefs";
    if (p.empty() || p[0] != '/') p = "/littlefs/" + p;
    return p;
}

// ls [path]  — list a littlefs directory (default /littlefs), or stat a single file.
static void cmdLs(cmd *c) {
    auto path = lfsPath(Command(c).getArg(0).getValue());
    struct stat st;
    if (stat(path.c_str(), &st) != 0)
        CMD_FAIL_RETURN("ls: not found: %s", path.c_str());
    if (!S_ISDIR(st.st_mode)) {
        UART_LOG("%8lu  %s", (unsigned long) st.st_size, path.c_str());
        return;
    }
    DIR *d = opendir(path.c_str());
    if (!d)
        CMD_FAIL_RETURN("ls: cannot open %s", path.c_str());
    int n = 0;
    for (dirent *e; (e = readdir(d));) {
        std::string full = path + "/" + e->d_name;
        struct stat es;
        bool ok = stat(full.c_str(), &es) == 0;
        if (ok && S_ISDIR(es.st_mode))
            UART_LOG("    <dir>  %s/", e->d_name);
        else
            UART_LOG("%8lu  %s", (unsigned long) (ok ? es.st_size : 0), e->d_name);
        ++n;
    }
    closedir(d);
    UART_LOG("ls: %d entries in %s", n, path.c_str());
}

// cat <file>  — print a littlefs text file (capped at 16 KB).
static void cmdCat(cmd *c) {
    auto arg = Command(c).getArg(0).getValue();
    if (arg.length() == 0)
        CMD_FAIL_RETURN("cat: expected <file>");
    auto path = lfsPath(arg);
    FILE *f = fopen(path.c_str(), "r");
    if (!f)
        CMD_FAIL_RETURN("cat: cannot open %s", path.c_str());
    char line[200];
    long total = 0;
    while (fgets(line, sizeof(line), f)) {
        size_t l = strlen(line);
        while (l && (line[l - 1] == '\n' || line[l - 1] == '\r')) line[--l] = 0; // UART_LOG adds its own EOL
        const int m = secretCut(line);
        UART_LOG("%.*s%s", CG_MASKED(line, m));
        if ((total += (long) l) > 16384) { UART_LOG("cat: ...truncated"); break; }
    }
    fclose(f);
}

// tasks  — FreeRTOS task table: state, priority, pinned core, min-ever free stack (bytes). perf.h def.
extern void print_task_table();

// Max RT-loop gap while the command ran, the self-check that a diagnostic does not stall core 1.
static void cmdLoopLagBegin() { g_app.cmdLoopLag = 0; }
static void cmdLoopLagPrint() { UART_LOG("loop gap during command: max %lu us", (unsigned long) g_app.cmdLoopLag); }

static void cmdTasks(cmd *) {
    cmdLoopLagBegin();
    print_task_table();
    cmdLoopLagPrint();
}

// bootinfo  — last reset reason, running OTA slot + rollback verify state, heap headroom, uptime.
static void cmdBootinfo(cmd *) {
    const char *rs;
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON: rs = "POWERON"; break;
        case ESP_RST_SW: rs = "SW"; break;
        case ESP_RST_PANIC: rs = "PANIC"; break;
        case ESP_RST_INT_WDT: rs = "INT_WDT"; break;
        case ESP_RST_TASK_WDT: rs = "TASK_WDT"; break;
        case ESP_RST_WDT: rs = "OTHER_WDT"; break;
        case ESP_RST_BROWNOUT: rs = "BROWNOUT"; break;
        case ESP_RST_DEEPSLEEP: rs = "DEEPSLEEP"; break;
        case ESP_RST_USB: rs = "USB"; break;
        case ESP_RST_JTAG: rs = "JTAG"; break;
        default: rs = "UNKNOWN"; break;
    }
    UART_LOG("reset reason: %s", rs);
    UART_LOG("setup: %s%s", g_app.setupErr ? "FAILED: " : "ok", g_app.setupErrMsg);
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (run) {
        const char *ss = "n/a";
        esp_ota_img_states_t st;
        if (esp_ota_get_state_partition(run, &st) == ESP_OK) {
            switch (st) {
                case ESP_OTA_IMG_NEW: ss = "NEW"; break;
                case ESP_OTA_IMG_PENDING_VERIFY: ss = "PENDING_VERIFY"; break;
                case ESP_OTA_IMG_VALID: ss = "VALID"; break;
                case ESP_OTA_IMG_INVALID: ss = "INVALID"; break;
                case ESP_OTA_IMG_ABORTED: ss = "ABORTED"; break;
                default: ss = "UNDEFINED"; break;
            }
        }
        UART_LOG("running: %s @0x%06x (%uKB)  rollback=%s",
                 run->label, (unsigned) run->address, (unsigned) (run->size / 1024), ss);
    }
    UART_LOG("heap: free=%u min-ever=%u largest=%u",
             (unsigned) esp_get_free_heap_size(), (unsigned) esp_get_minimum_free_heap_size(),
             (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    rtAllocPrint();
#ifdef CONFIG_FUGU_WITH_NETW
    netGuardPrint();
#endif
    UART_LOG("uptime: %lu s, app %s", (uint32_t) (esp_timer_get_time() / 1000000), format_version());
    esp_chip_info_t ci;
    esp_chip_info(&ci);
    uint32_t fid = 0, fsz = 0;
    esp_flash_read_id(nullptr, &fid);
    esp_flash_get_size(nullptr, &fsz);
    UART_LOG("chip: model=%d rev=%u.%u  flash: jedec=%06lx %luKB auto-suspend=%s", (int) ci.model,
             ci.revision / 100, ci.revision % 100, (unsigned long) fid, (unsigned long) (fsz / 1024),
#ifdef CONFIG_SPI_FLASH_AUTO_SUSPEND
             "on");
#else
             "off");
#endif
    char fsb[256];
    flashSusProbeFmt(fsb, sizeof(fsb));
    UART_LOG("%s", fsb);
    char bl[112];
    bootloaderInfo(ESP_PRIMARY_BOOTLOADER_OFFSET, bl, sizeof(bl));
    UART_LOG("bootloader: %s", bl);
#ifdef CONFIG_BOOTGUARD_ENABLE
    bootguard_status_t bg;
    if (bootguard_get_status(&bg) == ESP_OK)
        UART_LOG("bootguard: %s crashes=%u/%d trips=%u rom=0x%02x hint=%u", bg.guarded ? "on" : "off",
                 bg.crashes, CONFIG_BOOTGUARD_MAX_CRASHES, bg.trips, bg.last_rom, bg.last_hint);
    else
        UART_LOG("bootguard: no record (stock bootloader?)");
#endif
}

// heap [check]  — free / min-ever / largest-block per capability; `check` runs an integrity scan.
// interrupt table per core (source, level, shared), to check which core serves a peripheral
static void cmdIntr(cmd *) {
    char *buf = nullptr;
    size_t len = 0;
    FILE *f = open_memstream(&buf, &len);
    if (!f) return;
    esp_intr_dump(f);
    fclose(f);
    for (char *l = buf, *e; l && *l; l = e ? e + 1 : nullptr) {
        e = strchr(l, '\n');
        if (e) *e = 0;
        if (!strstr(l, "Reserved") && !strstr(l, "Free") && !strstr(l, "CPU-internal")) UART_LOG("%s", l);
    }
    free(buf);
}

#if CONFIG_HEAP_TASK_TRACKING
// IDF 6.0.3's tracking hooks take a mutex on every malloc/free and hang the board (INT_WDT on a malloc inside a
// critical section; skipping those still hung). Keep only the per-block owner word and walk it on demand.
extern "C" {
IRAM_ATTR void __wrap_heap_caps_update_per_task_info_alloc(void *, void *, size_t, uint32_t) {}
IRAM_ATTR void __wrap_heap_caps_update_per_task_info_free(void *, void *) {}
IRAM_ATTR void __wrap_heap_caps_update_per_task_info_realloc(void *, void *, void *, size_t, TaskHandle_t, size_t,
                                                             uint32_t) {}
}

// Internal heap held per owning task (stacks count to the creating task), plus own stack size/free.
static void heapTasks() {
    constexpr size_t NT = 48;
    auto *tt = (heap_task_totals_t *) calloc(NT, sizeof(heap_task_totals_t));
    auto *ts = (TaskStatus_t *) malloc(NT * sizeof(TaskStatus_t));
    if (!tt || !ts) {
        free(tt);
        free(ts);
        UART_LOG("heap tasks: oom");
        return;
    }
    size_t n = 0;
    heap_task_info_params_t p{};
    p.caps[0] = p.mask[0] = MALLOC_CAP_INTERNAL;
    p.totals = tt;
    p.num_totals = &n;
    p.max_totals = NT;
    heap_caps_get_per_task_info(&p);
    UBaseType_t nl = uxTaskGetSystemState(ts, NT, nullptr);
    auto stack = [](TaskHandle_t h) -> unsigned {
#if ESP_IDF_VERSION_MAJOR >= 6
        auto *s = (void *) xTaskGetStackStart(h);
#else
        auto *s = (void *) pxTaskGetStackStart(h);
#endif
        return heap_caps_check_integrity_addr((intptr_t) s, false) ? heap_caps_get_allocated_size(s) : 0;
    };
    UART_LOG("%-16s %7s %5s %6s %6s", "task", "held", "blk", "stack", "stkfre");
    size_t sum = 0, blk = 0;
    for (size_t i = 0; i < n + nl; ++i) {
        unsigned sz = 0, cnt = 0, stk = 0, fre = 0;
        char nm[20];
        UBaseType_t j = 0;
        if (i < n) {
            const auto &t = tt[i];
            sz = t.size[0];
            cnt = t.count[0];
            while (t.task && j < nl && ts[j].xHandle != t.task) ++j;
            if (!t.task || j == nl) {
                snprintf(nm, sizeof nm, t.task ? "(gone %p)" : "(no task)", t.task);
                j = nl;
            }
        } else if (!ts[j = i - n].xHandle) continue;
        if (j < nl) {
            stk = stack(ts[j].xHandle);
            fre = ts[j].usStackHighWaterMark;
            snprintf(nm, sizeof nm, "%s", ts[j].pcTaskName);
            ts[j].xHandle = nullptr;
        }
        sum += sz;
        blk += cnt;
        UART_LOG("%-16s %7u %5u %6u %6u", nm, sz, cnt, stk, fre);
    }
    UART_LOG("%-16s %7u %5u  (incl. this command's %u B buffers)", "total", (unsigned) sum, (unsigned) blk,
             (unsigned) (heap_caps_get_allocated_size(tt) + heap_caps_get_allocated_size(ts)));
    free(tt);
    free(ts);
}
#endif

static void cmdHeap(cmd *c) {
    UART_LOG("heap (bytes):  free / min-ever / largest-block");
    struct { const char *n; uint32_t caps; } pools[] = {
        {"INTERNAL", MALLOC_CAP_INTERNAL}, {"DMA", MALLOC_CAP_DMA}, {"SPIRAM", MALLOC_CAP_SPIRAM}};
    for (auto &p: pools)
        UART_LOG("  %-8s %8u %8u %8u", p.n, (unsigned) heap_caps_get_free_size(p.caps),
                 (unsigned) heap_caps_get_minimum_free_size(p.caps),
                 (unsigned) heap_caps_get_largest_free_block(p.caps));
    if (Command(c).getArg(0).getValue() == "check")
        UART_LOG("integrity: %s", heap_caps_check_integrity_all(true) ? "OK" : "CORRUPT");
#if CONFIG_HEAP_TASK_TRACKING
    multi_heap_info_t mi;
    heap_caps_get_info(&mi, MALLOC_CAP_INTERNAL);
    UART_LOG("  INTERNAL allocated %u in %u blocks, free %u", (unsigned) mi.total_allocated_bytes,
             (unsigned) mi.allocated_blocks, (unsigned) mi.total_free_bytes);
    if (Command(c).getArg(0).getValue() == "tasks")
        heapTasks();
#endif
}

// log <tag> <error|warn|info|debug|verbose|none>  — runtime ESP_LOG level for any tag (`*` = all).
static void cmdLogLevel(cmd *c) {
    Command cc(c);
    if (cc.countArgs() < 2)
        CMD_FAIL_RETURN("log: expected <tag> <error|warn|info|debug|verbose|none>");
    auto tag = cc.getArg(0).getValue();
    auto lvl = cc.getArg(1).getValue();
    esp_log_level_t l;
    if (lvl == "none") l = ESP_LOG_NONE;
    else if (lvl == "error") l = ESP_LOG_ERROR;
    else if (lvl == "warn") l = ESP_LOG_WARN;
    else if (lvl == "info") l = ESP_LOG_INFO;
    else if (lvl == "debug") l = ESP_LOG_DEBUG;
    else if (lvl == "verbose") l = ESP_LOG_VERBOSE;
    else CMD_FAIL_RETURN("log: bad level '%s'", lvl.c_str());
    esp_log_level_set(tag.c_str(), l);
    UART_LOG("log: %s -> %s", tag.c_str(), lvl.c_str());
}

#ifdef CONFIG_FUGU_WITH_NETW
// without an RT loop nothing can switch, and `dc 0` cannot be applied
static bool conversionStopped() {
    return converter.disabled() && (!rtLoopRunning() || (g_app.manualPwm() && mppt.getManualTarget() == 0));
}

// ota <url> | ota bootloader <url>
static void cmdOta(cmd *c) {
    auto url = Command(c).getArg(0).getValue();
    if (url.length() == 0)
        CMD_FAIL_RETURN("ota: expected url");
    if (bfTestBusy())
        CMD_FAIL_RETURN("ota: selftest running");
    if (measBusy(c)) return;
    UART_LOG("ota: running %s, downloading %s", format_version(), url.c_str());
    if (url.startsWith("bootloader")) {
        url.remove(0, 10);
        url.trim();
        if (url.length() == 0)
            CMD_FAIL_RETURN("ota: expected bootloader <url>");
        if (!conversionStopped())
            CMD_FAIL_RETURN("ota: conversion running, stop it first (dc 0)");
        adcSampler.halted = true;
        idleProfileHold(true);
        const char *err = doOtaBootloader(url.c_str(), conversionStopped);
        idleProfileHold(false);
        adcSampler.halted = false;
        if (err)
            CMD_FAIL_RETURN("ota: bootloader: %s", err);
        return;
    }

    // Refuse before anything is stopped or released: past the BLE release, any failure reboots.
    if (otaPendingVerify())
        CMD_FAIL_RETURN("ota: ESP_ERR_OTA_ROLLBACK_INVALID_STATE, running image not yet confirmed");
    if (!wifiIsConnected())
        CMD_FAIL_RETURN("ota: WiFi not connected");

    idleProfileHold(true); // full CPU clock for the download
    auto savedMode = g_app.opMode.load();
    const float savedPsuSetpoint = mppt.getRequestedPsuSetpoint();
    const uint16_t savedManualTarget = mppt.getManualTarget();
    float savedPvIsc, savedPvVoc, savedPvK;
    PvShade savedPvShade;
    const bool savedPvActive = mppt.getPvCurve(savedPvIsc, savedPvVoc, savedPvK, &savedPvShade);
    // Stop power conversion and let the stage de-energize BEFORE downloading. OTA flash
    // writes briefly stall the RT loop and draw erase-current spikes; at full power that
    // reset the device mid-transfer (doing `dc 0` by hand first was the reliable workaround).
    // Latch manual mode (also disables the !manualPwm-gated watchdogs) and stop the converter at
    // once (a ramp-down at full power trips highD-bflowOpen), then let the supply settle.
    // Without an RT loop (failed setup) nothing acks the handshake, and only that loop ever enables the converter.
    const bool rt = rtLoopRunning();
    if (!rt) {
        if (!converter.disabled()) {
            idleProfileHold(false);
            CMD_FAIL_RETURN("ota: no RT loop but converter enabled");
        }
        UART_LOG("ota: no RT loop, converter off, skipping the conversion stop");
    } else {
        const auto stopTicket = mppt.requestPsuManual(0, -1, false, true);
        if (!waitPsuCommand(stopTicket)) {
            idleProfileHold(false);
            CMD_FAIL_RETURN("ota: RT shutdown transition timed out");
        }
        for (int i = 0; i < 100 && !converter.disabled(); ++i) delay(100);
        delay(500);                 // let the output coil/caps de-energize
    }
    adcSampler.halted = true;   // disable ADC reading

    bool ok = doOta(url.c_str()); // reboots on success and never returns

    // OTA failed — resume the prior operating mode
    idleProfileHold(false);
    adcSampler.halted = false;
    if (!rt) {
        if (!ok) s_cmdFailed = true;
        return;
    }
    bool restored = true;
    if (savedMode == OpMode::Psu) {
        // A PV curve must be restored as the curve, not as a CV pin at the saved
        // instantaneous setpoint. rebase=false: the saved Isc may be a scaled one, and a
        // restore must not turn it into the new `pv scale` reference.
        const auto ticket = savedPvActive
                                ? mppt.queuePvCurve(savedPvIsc, savedPvVoc, savedPvK,
                                                    false, false, savedPvShade)
                                : mppt.queuePsuSetpoint(savedPsuSetpoint);
        if (!ticket || !waitPsuCommand(ticket)
            || mppt.getPsuCommandError(ticket) != PsuSetpointError::None) {
            restored = false;
            ESP_LOGE("main", "ota: PSU restore: %s",
                     psuErrorText(ticket ? mppt.getPsuCommandError(ticket)
                                         : mppt.getLastPsuRequestError()));
        }
    } else if (savedMode == OpMode::Manual) {
        restored = waitPsuCommand(mppt.requestPsuManual(savedManualTarget, -1));
    } else {
        restored = waitPsuCommand(mppt.requestPsuOff());
    }
    const char *mode = savedMode == OpMode::Psu ? "PSU" : savedMode == OpMode::Manual ? "manual" : "MPPT";
    if (restored) logRetained("I (%lu) ota: %s mode restored", (unsigned long) esp_log_timestamp(), mode);
    else logRetained("E (%lu) ota: failed to restore %s mode", (unsigned long) esp_log_timestamp(), mode);
    if (!ok) s_cmdFailed = true;
}
#endif


#ifdef CONFIG_FUGU_WITH_BLE
// otab begin <size> <sha256hex> | end | abort  — OTA firmware push over BLE (no WiFi). `begin` arms the
// receiver (halts the converter, erases the passive partition); the host then streams the image to the
// NUS FW characteristic; `end` verifies the SHA-256 and reboots. ADC halt/restore lives inside otaBle*.
static void cmdOtaBle(cmd *c) {
    if (bfTestBusy())
        CMD_FAIL_RETURN("ota-ble: selftest running");
    if (measBusy(c)) return;
    Command cc(c);
    // Reassemble the line for the module's parser, which owns both the command words and the OTAB
    // status words -- keeping them in one place is what stops them drifting apart again.
    String line = cc.getArg(0).getValue();
    for (int i = 1; i < cc.countArgs(); ++i) {
        line += ' ';
        line += cc.getArg(i).getValue();
    }
    // Rejected means malformed, and is reported synchronously so the console's OK:/ERR: completion
    // marker still tells the truth. Execution happens on the network loop -- esp_ota_begin blocks on
    // flash for far too long to run from a command callback -- so real failures still arrive later
    // as OTAB FAIL lines.
    const String w = cc.getArg(0).getValue();
    if (w == "begin" || w == "resume") {
        // Refused here, before the quiesce hook stops conversion and releases Wi-Fi.
        const char *why = otaPendingVerify() ? "esp_ota_begin ESP_ERR_OTA_ROLLBACK_INVALID_STATE"
                          : !bleConsoleConnected() ? "no-ble-client"
                          : g_restartBlocked ? "restart-blocked" : nullptr;
        if (why) {
            ESP_LOGW("otab", "OTAB FAIL %s", why);
            CMD_FAIL_RETURN("otab: %s", why);
        }
    }
    if (otaBleSubmitCommand(line.c_str()) == OtaBleSubmit::Rejected)
        CMD_FAIL_RETURN("otab: expected begin <size> <sha256hex> | end | abort");
}

// ble [bonds] | ble pair [s] | ble unpair <addr|all>
static void cmdBle(cmd *c) {
    Command cc(c);
    if (const char *err = bleCommand(cc.getArg(0).getValue().c_str(), cc.getArg(1).getValue().c_str()))
        CMD_FAIL_RETURN("ble: %s", err);
}
#endif

#if defined(CONFIG_FUGU_WITH_NETW) || defined(CONFIG_FUGU_WITH_BLE_TELE)
// set-time <epoch_ms>  — set the wall clock without NTP (BLE-only telemetry). SNTP still runs
// when WiFi comes up (may step the clock); TZ is set here so localtime-based code works.
static void cmdSetTime(cmd *c) {
    uint64_t ms = strtoull(Command(c).getArg(0).getValue().c_str(), nullptr, 10);
    if (ms < 1000000000000ULL)
        CMD_FAIL_RETURN("set-time: expected epoch milliseconds");
    timeval tv{.tv_sec = (time_t) (ms / 1000), .tv_usec = (suseconds_t) ((ms % 1000) * 1000)};
    settimeofday(&tv, nullptr);
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    timeSynced = true;
    UART_LOG("time set: %lu s", (unsigned long) (ms / 1000));
}
#endif

#ifdef CONFIG_FUGU_WITH_BLE_TELE
// tele-ble [0|1]  — start/stop the BLE telemetry stream (NUS TELE notify char). Requires a set
// clock (`set-time`) and a connected client; no arg prints status.
static void cmdTeleBle(cmd *c) {
    auto v = Command(c).getArg(0).getValue();
    if (v.length()) {
        const char *err = teleBleSetStreaming(v.toInt() != 0);
        if (err)
            CMD_FAIL_RETURN("tele-ble: %s", err);
    }
    UART_LOG("tele-ble: %s (dropped %u B)", teleBleStreaming() ? "on" : "off",
             (unsigned) teleBleDropped());
}
#endif

#ifdef CONFIG_FUGU_WITH_TELE_MQTT
// tele-mqtt [seconds]  — lease the MQTT telemetry stream (0 stops); no arg prints status.
static void cmdTeleMqtt(cmd *c) {
    auto v = Command(c).getArg(0).getValue();
    if (v.length()) {
        char *end;
        unsigned long s = strtoul(v.c_str(), &end, 10);
        if (*end || s > 3600)
            CMD_FAIL_RETURN("tele-mqtt: expected 0..3600 s");
        if (const char *err = teleMqttLease(s))
            CMD_FAIL_RETURN("tele-mqtt: %s", err);
    }
    if (teleMqttActive())
        UART_LOG("tele-mqtt: on, %u s left, dropped %u B", (unsigned) teleMqttLeaseLeftS(wallClockMs()),
                 (unsigned) teleMqttDropped());
    else
        UART_LOG("tele-mqtt: off");
}
#endif

#ifdef CONFIG_FUGU_WITH_NETTOOLS
// The lwip TCP/IP thread only exists once WiFi has come up; getaddrinfo / sockets / http post to
// it, so calling them with the stack down faults in tcpip_send_msg_wait_sem. Gate the net commands.
static bool nettoolsNetUp() { return WiFi.status() == WL_CONNECTED; }

// resolve an IPv4 host/dotted-quad to an in_addr; false if it can't be resolved.
static bool resolveHost4(const char *host, in_addr &out) {
    addrinfo hint{};
    hint.ai_family = AF_INET;
    addrinfo *res = nullptr;
    if (getaddrinfo(host, nullptr, &hint, &res) != 0 || !res)
        return false;
    out = ((sockaddr_in *) res->ai_addr)->sin_addr;
    freeaddrinfo(res);
    return true;
}

// curl [-X METHOD] [-H key:val] [-d data] <url>  — blocking HTTP(S) request, prints status + body to
// the issuing console. TLS is verified against the mbedTLS certificate bundle. `-d` implies POST. The
// body is streamed (not fully buffered) and capped so a large response can't OOM the network task.
// Flag values are single whitespace-delimited tokens (use compact JSON, no spaces). Runs on core 0.
static void cmdCurl(cmd *c) {
    Command cc(c);
    int n = cc.countArgs();
    String url, method, body;
    String hdrKey[4], hdrVal[4];
    int nHdr = 0;
    for (int i = 0; i < n; ++i) {
        String a = cc.getArg(i).getValue();
        if (a.length() == 0) continue;
        if (a == "-X" && i + 1 < n) {
            method = cc.getArg(++i).getValue();
        } else if (a == "-d" && i + 1 < n) {
            body = cc.getArg(++i).getValue();
        } else if (a == "-H" && i + 1 < n && nHdr < 4) {
            String hv = cc.getArg(++i).getValue();
            int colon = hv.indexOf(':');
            if (colon > 0) {
                hdrKey[nHdr] = hv.substring(0, colon);
                hdrVal[nHdr] = hv.substring(colon + 1);
                ++nHdr;
            }
        } else if (a.startsWith("http")) {
            url = a;
        }
    }
    if (url.length() == 0)
        CMD_FAIL_RETURN("curl: expected [-X M] [-H k:v] [-d data] <url>");
    if (!nettoolsNetUp())
        CMD_FAIL_RETURN("curl: network down (WiFi not connected)");

    esp_http_client_method_t m = HTTP_METHOD_GET;
    if (method.length()) {
        if (method == "GET") m = HTTP_METHOD_GET;
        else if (method == "POST") m = HTTP_METHOD_POST;
        else if (method == "PUT") m = HTTP_METHOD_PUT;
        else if (method == "DELETE") m = HTTP_METHOD_DELETE;
        else if (method == "HEAD") m = HTTP_METHOD_HEAD;
        else if (method == "PATCH") m = HTTP_METHOD_PATCH;
        else CMD_FAIL_RETURN("curl: unknown method '%s'", method.c_str());
    } else if (body.length()) {
        m = HTTP_METHOD_POST; // -d implies POST, like curl
    }

    esp_http_client_config_t cfg{};
    cfg.url = url.c_str();
    cfg.timeout_ms = 10000;
#if CONFIG_FUGU_WITH_HTTPS
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
#endif
    cfg.method = m;
    esp_http_client_handle_t h = esp_http_client_init(&cfg);
    if (!h)
        CMD_FAIL_RETURN("curl: init failed");

    bool haveCt = false;
    for (int i = 0; i < nHdr; ++i) {
        esp_http_client_set_header(h, hdrKey[i].c_str(), hdrVal[i].c_str());
        if (hdrKey[i].equalsIgnoreCase("Content-Type")) haveCt = true;
    }
    int wlen = body.length();
    if (wlen && !haveCt)
        esp_http_client_set_header(h, "Content-Type", "application/x-www-form-urlencoded");

    esp_err_t err = esp_http_client_open(h, wlen);
    if (err != ESP_OK) {
        esp_http_client_cleanup(h);
        CMD_FAIL_RETURN("curl: open failed (%s)", esp_err_to_name(err));
    }
    if (wlen && esp_http_client_write(h, body.c_str(), wlen) < 0) {
        esp_http_client_close(h);
        esp_http_client_cleanup(h);
        CMD_FAIL_RETURN("curl: body write failed");
    }
    int64_t clen = esp_http_client_fetch_headers(h);
    UART_LOG("curl: HTTP %d, len=%ld", esp_http_client_get_status_code(h), (long) clen);

    char buf[513];
    int total = 0, nr;
    while ((nr = esp_http_client_read(h, buf, sizeof(buf) - 1)) > 0) {
        buf[nr] = 0;
        UART_LOG("%s", buf);
        total += nr;
        if (total >= 16384) { UART_LOG("curl: ...truncated at %d bytes", total); break; }
    }
    esp_http_client_close(h);
    esp_http_client_cleanup(h);
}

// ping <host> [count]  — ICMP echo to an IPv4 host/IP via the lwip ping app. Prints per-reply lines
// and a summary; blocks the console until the session ends (or times out).
static void pingOnSuccess(esp_ping_handle_t hdl, void *) {
    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed, recvLen;
    ip_addr_t target;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target, sizeof(target));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recvLen, sizeof(recvLen));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed, sizeof(elapsed));
    UART_LOG("ping: %lu bytes from %s: seq=%u ttl=%u time=%lu ms",
             (unsigned long) recvLen, ipaddr_ntoa(&target), (unsigned) seqno, (unsigned) ttl,
             (unsigned long) elapsed);
}
static void pingOnTimeout(esp_ping_handle_t hdl, void *) {
    uint16_t seqno;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    UART_LOG("ping: seq=%u timeout", (unsigned) seqno);
}
static void pingOnEnd(esp_ping_handle_t hdl, void *args) {
    uint32_t sent, recv, totalMs;
    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &sent, sizeof(sent));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &recv, sizeof(recv));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &totalMs, sizeof(totalMs));
    UART_LOG("ping: %lu sent, %lu received, %lu%% loss, time %lums",
             (unsigned long) sent, (unsigned long) recv,
             (unsigned long) (sent ? (sent - recv) * 100 / sent : 0), (unsigned long) totalMs);
    xSemaphoreGive((SemaphoreHandle_t) args);
}
static void cmdPing(cmd *c) {
    Command cc(c);
    auto host = cc.getArg(0).getValue();
    if (host.length() == 0)
        CMD_FAIL_RETURN("ping: expected <host> [count]");
    if (!nettoolsNetUp())
        CMD_FAIL_RETURN("ping: network down (WiFi not connected)");
    uint32_t count = cc.countArgs() >= 2 ? (uint32_t) cc.getArg(1).getValue().toInt() : 4;
    if (count == 0 || count > 60) count = 4; // no infinite ping from a remote console

    in_addr a4;
    if (!resolveHost4(host.c_str(), a4))
        CMD_FAIL_RETURN("ping: cannot resolve '%s'", host.c_str());
    ip_addr_t target{};
    inet_addr_to_ip4addr(ip_2_ip4(&target), &a4);

    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    if (!done)
        CMD_FAIL_RETURN("ping: out of memory");

    esp_ping_config_t pc = ESP_PING_DEFAULT_CONFIG();
    pc.target_addr = target;
    pc.count = count;
    pc.task_stack_size = 6144; // callbacks UART_LOG via vprintf_mux (300 B stack buf + telnet/mqtt/ble fan-out)
    esp_ping_callbacks_t cbs{};
    cbs.on_ping_success = pingOnSuccess;
    cbs.on_ping_timeout = pingOnTimeout;
    cbs.on_ping_end = pingOnEnd;
    cbs.cb_args = done;

    esp_ping_handle_t hdl = nullptr;
    if (esp_ping_new_session(&pc, &cbs, &hdl) != ESP_OK || !hdl) {
        vSemaphoreDelete(done);
        CMD_FAIL_RETURN("ping: session create failed");
    }
    UART_LOG("ping %s (%s): %lu packets", host.c_str(), ipaddr_ntoa(&target), (unsigned long) count);
    esp_ping_start(hdl);
    // worst case: every packet waits the full timeout, plus the inter-packet intervals.
    TickType_t wait = pdMS_TO_TICKS(count * (pc.interval_ms + pc.timeout_ms) + 2000);
    if (xSemaphoreTake(done, wait) != pdTRUE) {
        esp_ping_stop(hdl);
        UART_LOG("ping: aborted (timeout waiting for session end)");
    }
    esp_ping_delete_session(hdl);
    vSemaphoreDelete(done);
}

// nslookup <host>  (alias resolve)  — print every IPv4 address the resolver returns for <host>.
static void cmdNslookup(cmd *c) {
    auto host = Command(c).getArg(0).getValue();
    if (host.length() == 0)
        CMD_FAIL_RETURN("nslookup: expected <host>");
    if (!nettoolsNetUp())
        CMD_FAIL_RETURN("nslookup: network down (WiFi not connected)");
    addrinfo hint{};
    hint.ai_family = AF_INET;
    addrinfo *res = nullptr;
    int rc = getaddrinfo(host.c_str(), nullptr, &hint, &res);
    if (rc != 0 || !res)
        CMD_FAIL_RETURN("nslookup: '%s' not found (rc=%d)", host.c_str(), rc);
    int n = 0;
    for (addrinfo *p = res; p; p = p->ai_next) {
        if (p->ai_family != AF_INET) continue;
        auto a = ((sockaddr_in *) p->ai_addr)->sin_addr;
        UART_LOG("nslookup: %s -> %s", host.c_str(), inet_ntoa(a));
        ++n;
    }
    freeaddrinfo(res);
    if (!n)
        CMD_FAIL_RETURN("nslookup: no IPv4 address for '%s'", host.c_str());
}

// tcpconnect <host> <port>  (alias probe)  — non-blocking TCP connect with a 5 s timeout; reports
// open / refused / timeout / error so a broker or OTA endpoint can be reached-tested at the port level.
static void cmdTcpConnect(cmd *c) {
    Command cc(c);
    auto host = cc.getArg(0).getValue();
    long port = cc.getArg(1).getValue().toInt();
    if (host.length() == 0 || port <= 0 || port > 65535)
        CMD_FAIL_RETURN("tcpconnect: expected <host> <port>");
    if (!nettoolsNetUp())
        CMD_FAIL_RETURN("tcpconnect: network down (WiFi not connected)");
    in_addr a4;
    if (!resolveHost4(host.c_str(), a4))
        CMD_FAIL_RETURN("tcpconnect: cannot resolve '%s'", host.c_str());

    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0)
        CMD_FAIL_RETURN("tcpconnect: socket() failed");
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t) port);
    addr.sin_addr = a4;

    auto t0 = wallClockUs();
    const char *result;
    int r = connect(s, (sockaddr *) &addr, sizeof(addr));
    if (r == 0) {
        result = "open";
    } else if (errno == EINPROGRESS) {
        fd_set wf;
        FD_ZERO(&wf);
        FD_SET(s, &wf);
        timeval tv{};
        tv.tv_sec = 5;
        r = select(s + 1, nullptr, &wf, nullptr, &tv);
        if (r > 0) {
            int err = 0;
            socklen_t l = sizeof(err);
            getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &l);
            result = err == 0 ? "open" : (err == ECONNREFUSED ? "refused" : "error");
            if (err && err != ECONNREFUSED)
                UART_LOG("tcpconnect: connect error %d (%s)", err, strerror(err));
        } else {
            result = r == 0 ? "timeout" : "select error";
        }
    } else {
        result = "error";
    }
    float ms = (wallClockUs() - t0) * 1e-3f;
    close(s);
    UART_LOG("tcpconnect: %s:%ld %s (%.0f ms)", inet_ntoa(a4), port, result, ms);
    if (strcmp(result, "open") != 0) s_cmdFailed = true;
}

// netstat  (alias ifconfig)  — STA link + IP config snapshot (ssid/bssid/rssi, ip/gw/mask/dns, mac).
static void cmdNetstat(cmd *) {
    UART_LOG("hostname: %s", getHostname().c_str());
    bool up = WiFi.status() == WL_CONNECTED;
    UART_LOG("wifi: %s", up ? "connected" : "disconnected");
    if (up) {
        UART_LOG("  ssid=%s bssid=%s ch=%d rssi=%d dBm",
                 WiFi.SSID().c_str(), WiFi.BSSIDstr().c_str(), (int) WiFi.channel(), (int) WiFi.RSSI());
        UART_LOG("  ip=%s gw=%s mask=%s",
                 WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str(),
                 WiFi.subnetMask().toString().c_str());
        UART_LOG("  dns=%s,%s", WiFi.dnsIP(0).toString().c_str(), WiFi.dnsIP(1).toString().c_str());
    }
    UART_LOG("  mac=%s", WiFi.macAddress().c_str());
}
#endif // CONFIG_FUGU_WITH_NETTOOLS

// Runs synchronously (2s): blocks the console task so the `OK:` reply follows the table (scripted
// callers capture it) and concurrent rt-stats can't pile up. perf.h def; see note there.
extern void print_real_time_stats_blocking();

static void cmdRtStats(cmd *) {
    cmdLoopLagBegin();
    print_real_time_stats_blocking();
    rtAllocPrint();
    UART_LOG("loop-rate watchdog %hu Hz", g_app.loopRateMin);
    cmdLoopLagPrint();
}

// wired-sync diagnostic: edge rate on the sync pin (PCNT). Leader = self-check of its own
// pulse, follower = wire delivery check. Expect the converter's own pwm_freq.
static void cmdWsync(cmd *c) {
#if CONFIG_FUGU_WITH_WSYNC
    // Handled before the counter check on purpose: `arm` is only ever needed in the state that
    // has no counter (USB fallback), where the check below would have returned already.
    Command cc(c);
    if (cc.countArgs() >= 1 && cc.getArg(0).getValue() == "arm") {
        bool on = !(cc.countArgs() >= 2 && cc.getArg(1).getValue() == "off");
        nvs.open();
        nvs.writeString("wsync_arm", on ? "1" : "");
        nvs.commit();
        nvs.close();
        // One-shot, and it only skips the USB host pre-check -- the line still has to qualify,
        // so this cannot arm a follower against a leader that is not running.
        UART_LOG("wsync arm: %s. restart to apply",
                 on ? "next boot takes the USB sync pad even with a host attached" : "cleared");
        return;
    }
#else
    (void) c;
#endif
    if (!converter.wsyncHasCounter()) {
        // Keep the "no edge counter" substring: fugu.py keys role=none off it.
#if CONFIG_FUGU_WITH_WSYNC
        // The one-shot flag is the only way to tell "probed because you armed me" from "probed
        // because no host was attached" -- both reach the same mode.
        UART_LOG("wsync: no edge counter, mode=%s%s", wsyncModeStr(converter.wsyncMode),
                 converter.wsyncArmRequest ? " (armed this boot)" : "");
#else
        UART_LOG("wsync: no edge counter (sync_role=none, or CONFIG_FUGU_WITH_WSYNC off)");
#endif
        return;
    }
    // The RUNNING role, which only the driver knows: sync_role is read once at boot while
    // set-config edits the file live, and a leader counts its own outgoing pulse at exactly
    // pwm_freq -- indistinguishable from a locked follower unless the reply says which this is.
    UART_LOG("wsync role=%s%s", converter.wsyncFollower ? "follower" : "leader",
#if CONFIG_FUGU_WITH_WSYNC
             converter.wsyncArmRequest ? " (armed this boot)" : "");
#else
             "");
#endif
    // keep the window short at high pwm_freq: the count accumulates across the 16-bit wrap
    // (see drvInit), but a shorter window still bounds the wrap-ISR load
    uint32_t f = converter.getPwmFrequency();
    uint32_t ms = (f && converter.wsyncCountMax * 800ull / f < 250) ? converter.wsyncCountMax * 800u / f : 250u;
    converter.wsyncClear();
    int64_t t0 = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(ms));
    int n = converter.wsyncCount();
    // measured, not requested: a preempted console task overruns the delay, and dividing by the
    // nominal window would report a healthy wire as over-rate
    uint32_t msMeas = (uint32_t) ((esp_timer_get_time() - t0 + 500) / 1000);
    if (!msMeas) CMD_FAIL_RETURN("wsync: zero-length window, rate unknown");
    UART_LOG("wsync edges: %d in %ums (%.2f kHz)", n, (unsigned) msMeas, (float) n / (float) msMeas);
}

// monotonic seconds since boot; resets only on reboot (unlike status N, which zeroes on each sweep)
static void cmdUptime(cmd *) {
        UART_LOG("Uptime: %lu s", (uint32_t) (esp_timer_get_time() / 1000000));
    UART_LOG("App: %s", format_version());
}

// peek <addr> [len]  — read up to 256 bytes from internal RAM, DROM (flash-mapped const),
// IRAM/IROM, RTC slow memory or peripheral MMIO, and either print one typed value
// (len ∈ {1,2,4,8}) or a hex+ASCII dump (any other 1..256). Address accepts `0x…`, decimal, or
// octal (strtoul base 0). Executable and MMIO regions need 4-byte aligned addr+len so we can
// issue 32-bit bus loads. Symbol resolution lives host-side in fugu_console.py.
//
// MMIO caveats: a register whose peripheral clock is gated faults the bus, several registers are
// read-destructive (UART/I2C FIFO pop, MCPWM capture, *_INT_ST latches), and the read is not
// synchronised against the RT loop.
#if CONFIG_IDF_TARGET_ESP32S3
#define PEEK_MMIO_LOW  0x60000000UL
#define PEEK_MMIO_HIGH 0x600D2000UL
#elif CONFIG_IDF_TARGET_ESP32
#include <soc/dport_access.h>
#define PEEK_MMIO_LOW  0x3FF00000UL
#define PEEK_MMIO_HIGH 0x3FF80000UL
#else
#define PEEK_MMIO_LOW  0UL
#define PEEK_MMIO_HIGH 0UL
#endif
static inline bool peekByteOk(const void *p) {
    return esp_ptr_internal(p) || esp_ptr_in_drom(p) || esp_ptr_external_ram(p)
           || esp_ptr_in_rtc_slow(p) || esp_ptr_in_rtc_dram_fast(p);
}
static inline bool peekMmioOk(uint32_t a) { return a >= PEEK_MMIO_LOW && a < PEEK_MMIO_HIGH; }
static inline uint32_t peekWord(uint32_t a) {
#if CONFIG_IDF_TARGET_ESP32
    // classic DPORT needs the APB-interlocked read to avoid returning the other bus's data
    if (a >= DR_REG_DPORT_BASE && a <= DR_REG_DPORT_END) return DPORT_SEQUENCE_REG_READ(a);
#endif
    return *(const volatile uint32_t *) a;
}
static void cmdPeek(cmd *c) {
    Command cc(c);
    if (cc.countArgs() < 1)
        CMD_FAIL_RETURN("peek: expected <addr> [len]");
    auto sAddr = cc.getArg(0).getValue();
    char *endp = nullptr;
    unsigned long addrUl = strtoul(sAddr.c_str(), &endp, 0);
    if (!endp || endp == sAddr.c_str() || *endp != '\0')
        CMD_FAIL_RETURN("peek: invalid address '%s'", sAddr.c_str());
    uint32_t addr = (uint32_t) addrUl;
    int len = cc.countArgs() >= 2 ? cc.getArg(1).getValue().toInt() : 4;
    if (len <= 0 || len > 256)
        CMD_FAIL_RETURN("peek: len out of range (1..256)");

    const void *first = (const void *) addr;
    const void *last  = (const void *) (addr + len - 1);
    uint8_t buf[256];
    bool mmio = peekMmioOk(addr) && peekMmioOk(addr + len - 1);
    if (peekByteOk(first) && peekByteOk(last)) {
        memcpy(buf, first, (size_t) len);
    } else if (mmio || (esp_ptr_executable(first) && esp_ptr_executable(last))) {
        if ((addr & 3) || (len & 3))
            CMD_FAIL_RETURN("peek: %s region needs 4-byte aligned addr+len", mmio ? "mmio" : "executable");
        for (int i = 0; i < len; i += 4) {
            uint32_t w = peekWord(addr + i);
            memcpy(buf + i, &w, 4);
        }
    } else {
        CMD_FAIL_RETURN("peek: 0x%08lx not safely readable", (unsigned long) addr);
    }

    // Typed scalar print for the common "what's the value of X" case.
    if (len == 1) { UART_LOG("peek 0x%08lx = 0x%02x", (unsigned long) addr, buf[0]); return; }
    if (len == 2) { uint16_t v; memcpy(&v, buf, 2);
        UART_LOG("peek 0x%08lx = 0x%04x", (unsigned long) addr, (unsigned) v); return; }
    if (len == 4) { uint32_t v; memcpy(&v, buf, 4);
        UART_LOG("peek 0x%08lx = 0x%08lx", (unsigned long) addr, (unsigned long) v); return; }
    if (len == 8) { uint32_t hi, lo; memcpy(&lo, buf, 4); memcpy(&hi, buf + 4, 4);
        UART_LOG("peek 0x%08lx = 0x%08lx%08lx", (unsigned long) addr,
                 (unsigned long) hi, (unsigned long) lo); return; }

    for (int off = 0; off < len; off += 16) {
        char line[100];
        int row = std::min(16, len - off);
        int n = snprintf(line, sizeof(line), "0x%08lx:", (unsigned long) (addr + off));
        for (int i = 0; i < 16; ++i) {
            int avail = (int) sizeof(line) - n;
            if (avail <= 1) break;
            int w = (i < row) ? snprintf(line + n, avail, " %02x", buf[off + i])
                              : snprintf(line + n, avail, "   ");
            n += (w < 0 || w >= avail) ? avail - 1 : w;
        }
        if ((int) sizeof(line) - n > 4) {
            line[n++] = ' '; line[n++] = '|';
            for (int i = 0; i < row && n + 2 < (int) sizeof(line); ++i) {
                char ch = (char) buf[off + i];
                line[n++] = (ch >= 0x20 && ch < 0x7f) ? ch : '.';
            }
            line[n++] = '|';
        }
        line[n] = 0;
        UART_LOG("%s", line);
    }
}

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
// crash <null|abort|stack> — deliberately panic to exercise the coredump path. Requires an explicit
// subtype (the console is reachable over MQTT, so no bare trigger). Never returns.
static volatile int s_crashNever = 1 << 30; // volatile so the compiler can't prove infinite recursion
static void crashRecurse(volatile int x) {
    volatile char eat[160];
    eat[0] = (char) x;
    if (x > s_crashNever) return;           // never true in practice; stack overflows long before
    crashRecurse(x + (int) eat[0] + 1);
}
static void cmdCrash(cmd *c) {
    auto t = Command(c).getArg(0).getValue();
    if (t != "null" && t != "abort" && t != "stack" && t != "overrun"
#ifdef CONFIG_FUGU_WITH_NETW
        && t != "lwip" && t != "pbuf"
#endif
    )
        CMD_FAIL_RETURN("crash: expected null|abort|stack|overrun|lwip|pbuf (deliberate panic, writes a coredump)");
    UART_LOG("crash: inducing '%s' panic NOW", t.c_str());
    delay(80); // let the line flush over UART/telnet/MQTT/BLE before we die
    if (t == "abort") abort();
    if (t == "stack") crashRecurse(1);
    if (t == "overrun") { // heap poisoning catches it in free(); without poisoning this corrupts the heap
        auto *b = (volatile uint8_t *) malloc(16);
        for (int i = 0; i < 24; ++i) b[i] = 0;
        free((void *) b);
        UART_LOG("crash overrun: free() passed (heap poisoning off?), rebooting");
        delay(80);
        esp_restart();
    }
#ifdef CONFIG_FUGU_WITH_NETW
    if (t == "lwip") { // core API off the tcpip thread: aborts with CONFIG_LWIP_CHECK_THREAD_SAFETY
        netif_find("zz0");
        CMD_FAIL_RETURN("crash lwip: no abort (CONFIG_LWIP_CHECK_THREAD_SAFETY off)");
    }
    if (t == "pbuf") { // ref-0 free: lwIP aborts; CONFIG_FUGU_PBUF_PROBE snapshots it first
        auto *p = pbuf_alloc(PBUF_RAW, 64, PBUF_RAM);
        if (!p) CMD_FAIL_RETURN("crash pbuf: no heap");
        memset(p->payload, 0xA5, 64);
        p->ref = 0;
        pbuf_free(p);
        CMD_FAIL_RETURN("crash pbuf: no abort (lwIP asserts off?)");
    }
#endif
    volatile uint32_t *p = (volatile uint32_t *) (uintptr_t) (esp_timer_get_time() & 0); // null, opaque to -Wnull
    *p = 0xDEAD;
}

// coredump [info|get|erase] — inspect/extract the panic core dump saved to the `coredump` flash
// partition. `get` streams the raw partition image as base64 over the console (mirrors to MQTT/telnet),
// so a backtrace can be pulled from a converter that has no serial. Decode host-side:
//   collect lines between the BEGIN/END markers -> base64 -decode-> dump.bin
//   esp-coredump info_corefile --core-format raw -c dump.bin build/fugu-firmware.elf
static int b64enc(const uint8_t *in, size_t n, char *out) {
    static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int o = 0;
    size_t i = 0;
    for (; i + 3 <= n; i += 3) {
        uint32_t v = (in[i] << 16) | (in[i + 1] << 8) | in[i + 2];
        out[o++] = T[(v >> 18) & 63]; out[o++] = T[(v >> 12) & 63];
        out[o++] = T[(v >> 6) & 63];  out[o++] = T[v & 63];
    }
    if (i < n) {
        uint32_t v = in[i] << 16;
        if (i + 1 < n) v |= in[i + 1] << 8;
        out[o++] = T[(v >> 18) & 63];
        out[o++] = T[(v >> 12) & 63];
        out[o++] = (i + 1 < n) ? T[(v >> 6) & 63] : '=';
        out[o++] = '=';
    }
    return o;
}
// Trailing 4-byte checksum of the stored dump — unique per crash, used to detect a fresh dump.
static uint32_t coredumpSignature(size_t addr, size_t size) {
    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
    if (!p || size < 4) return 0;
    size_t off = (addr >= p->address) ? addr - p->address : 0;
    uint32_t sig = 0;
    esp_partition_read(p, off + size - 4, &sig, sizeof(sig));
    return sig;
}

// ESP coredumps carry no wall-clock. The panic reboots the device, so the first time-synced boot
// afterwards approximates the crash time. Keyed by the dump's checksum so a *new* crash re-stamps
// but reboots that keep the same dump don't. Cheap and idempotent — call from loopLF every tick.
void coredumpStampIfNew() {
    static bool done = false;
    if (done) return;
    time_t now = time(nullptr);
    if (now < 1735689600) return; // before 2025 → clock not SNTP-synced yet, retry next tick

    size_t addr = 0, size = 0;
    uint32_t sig = (esp_core_dump_image_get(&addr, &size) == ESP_OK)
                       ? coredumpSignature(addr, size) : 0;
    nvs_handle_t h;
    if (nvs_open("coredump", NVS_READWRITE, &h) != ESP_OK) { done = true; return; }
    uint32_t storedSig = 0;
    nvs_get_u32(h, "sig", &storedSig);
    if (sig == 0) {
        if (storedSig) { nvs_erase_key(h, "sig"); nvs_erase_key(h, "ts"); nvs_commit(h); }
    } else if (sig != storedSig) {
        nvs_set_u32(h, "sig", sig);
        nvs_set_u32(h, "ts", (uint32_t) now);
        nvs_commit(h);
        ESP_LOGW("main", "coredump: new dump, stamped crash ~%lu", (unsigned long) now);
    }
    nvs_close(h);
    done = true;
}

static uint32_t coredumpStoredTs() {
    nvs_handle_t h;
    uint32_t ts = 0;
    if (nvs_open("coredump", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, "ts", &ts);
        nvs_close(h);
    }
    return ts;
}

static void cmdCoredump(cmd *c) {
    Command cc(c);
    std::string sub = cc.countArgs() >= 1 ? cc.getArg(0).getValue().c_str() : "info";

    if (sub == "erase") {
        esp_err_t e = esp_core_dump_image_erase();
        if (e != ESP_OK) CMD_FAIL_RETURN("coredump: erase failed (%s)", esp_err_to_name(e));
        nvs_handle_t h;
        if (nvs_open("coredump", NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_key(h, "sig"); nvs_erase_key(h, "ts"); nvs_commit(h); nvs_close(h);
        }
        UART_LOG("coredump: erased");
        return;
    }

    size_t addr = 0, size = 0;
    esp_err_t err = esp_core_dump_image_get(&addr, &size);
    if (err != ESP_OK) {
        UART_LOG("coredump: none (%s)", esp_err_to_name(err));
        return;
    }

    if (sub == "info") {
        bool ok = (esp_core_dump_image_check() == ESP_OK);
        // crashed = epoch of first synced boot after the dump appeared (0 = unknown/not yet stamped)
        UART_LOG("coredump: present addr=0x%08x size=%u check=%s crashed=%lu",
                 (unsigned) addr, (unsigned) size, ok ? "ok" : "BAD",
                 (unsigned long) coredumpStoredTs());
        UART_LOG("coredump: 'get' to stream base64, 'erase' to clear");
        return;
    }

    if (sub == "get") {
        const esp_partition_t *p = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
        if (!p) CMD_FAIL_RETURN("coredump: partition not found");
        size_t off = (addr >= p->address) ? addr - p->address : 0;
        UART_LOG("==COREDUMP-BEGIN size=%u==", (unsigned) size);
        uint8_t raw[48];
        char b64[68];
        for (size_t done = 0; done < size;) {
            size_t n = std::min(sizeof(raw), size - done);
            if (esp_partition_read(p, off + done, raw, n) != ESP_OK)
                CMD_FAIL_RETURN("coredump: read failed @%u", (unsigned) (off + done));
            int bl = b64enc(raw, n, b64);
            b64[bl] = 0;
            UART_LOG("%s", b64);
            done += n;
#ifdef CONFIG_FUGU_WITH_BLE
            bleConsoleAwaitTxDrain(2048, 4000); // pace BLE TX FIFO (console_ble.cpp absent when BLE off)
#endif
        }
        UART_LOG("==COREDUMP-END==");
        return;
    }
    CMD_FAIL_RETURN("coredump: expected info|get|erase");
}
#endif // CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH

static void cmdMem(cmd *) {
    UART_LOG("Total heap:  %9ld", ESP.getHeapSize());
    UART_LOG("Free heap:   %9ld", ESP.getFreeHeap());
    UART_LOG("Total PSRAM: %9ld", ESP.getPsramSize());
    UART_LOG("Free PSRAM:  %9ld", ESP.getFreePsram());
}

void ina226PrintInfo();

static void cmdSensor(cmd *c) {
    if (Command(c).countArgs() >= 1) {
        // `sensor avg`: one compact line of EWM averages, for fast polling
        char line[160];
        int n = 0;
        for (auto s: adcSampler.sensors) {
            if (n < 0 || n >= (int) sizeof(line)) break;
            n += snprintf(line + n, sizeof(line) - n, "%s=%.4f ", s->params.teleName.c_str(), s->ewm.avg.get());
        }
        UART_LOG("sens: %s", line);
        return;
    }
    for (auto s: adcSampler.sensors) {
        auto u = s->params.unit;
        UART_LOG("\nSensor `%s` (ch%d, %s):", s->params.teleName.c_str(), s->params.adcCh,
                 s->isVirtual ? "virtual" : "physical");
        UART_LOG("  num=%6lu  last=%7.3f %c   prev=%7.3f %c  raw=%8.4f", s->numSamples, s->last, u, s->previous, u,
                 s->lastRaw);
        if (s->params.calibrationConstraints.calibrateOffset)
            UART_LOG("  zero=%7.4f %c  unsettled=%u (in a row)", s->calibrationAvg, u, s->zeroMiss);
        UART_LOG("  EWM(%4lu):  avg= %7.3f %c   std*=%7.4f %c  std%%=%7.3f %%", s->ewm.span(),
                 s->ewm.avg.get(), u,
                 sqrt(s->ewm.std.get()) * abs(s->ewm.avg.get()), u,
                 sqrt(s->ewm.std.get()) * 100.f);
        UART_LOG("  ANF(span=%4.0f):  Nstd= %7.3f   Sstd=%7.3f   NSR=%7.3f%s", s->anf.span,
                 sqrt(s->anf.ewmN.nvar()) * 100.0f, sqrt(s->anf.ewmS.nvar()) * 100.0f,
                 sqrt(s->anf.ewmN.nvar() / s->anf.ewmS.nvar()),
                 Sensor::anfEnabled ? "" : "  (stale — run `anf on`)");
        if (!s->isVirtual) { // racy reads of RT-core filter state, display only
            const float L = s->comb ? s->comb->length() : 0.f;
            UART_LOG("  ripple: notch %s f0=%.1fHz %.1fdB  comb %s L=%.2f fs=%.1fHz", s->notchFilter ? "on" : "off",
                     adcSampler.getNotchFreq(), 8.685890f * logf(adcSampler.notchAmp), L > 0 ? "on" : "off", L,
                     ((const PhysicalSensor *) s)->notchFs);
        }
    }
    ina226PrintInfo();
}

#ifdef CONFIG_FUGU_WITH_NETW
static void cmdIp(cmd *) { UART_LOG("Local IP Address: %s", WiFi.localIP().toString().c_str()); }
#endif

static void cmdAdcRestart(cmd *) { adcSampler.requestAdcReset(true); }

static void cmdAdcReset(cmd *) { adcSampler.requestAdcReset(false); }

// anf [on|off] — the AdaptiveNoiseFilter is diagnostics-only (read by `sensor`); it's kept out of
// the RT sample path by default. Enable it to populate the ANF/NSR stats, disable when done.
static void cmdAnf(cmd *c) {
    Command cc(c);
    auto arg = cc.getArg(0).getValue();
    if (arg == "on") Sensor::anfEnabled = true;
    else if (arg == "off") Sensor::anfEnabled = false;
    UART_LOG("anf %s", Sensor::anfEnabled ? "on" : "off");
}

// hostname [name]  — no arg prints the current hostname; an arg sets it.
static void cmdHostname(cmd *c) {
    Command cc(c);
    if (cc.countArgs() < 1 || cc.getArg(0).getValue().length() == 0) {
#ifdef CONFIG_FUGU_WITH_NETW
        UART_LOG("Hostname: %s", getHostname().c_str());
#else
        nvs.open();
        UART_LOG("Hostname: %s", nvs.readString("hostname", "fugu").c_str());
        nvs.close();
#endif
        return;
    }
    auto hn = cc.getArg(0).getValue();
    nvs.open();
    nvs.writeString("hostname", hn.c_str());
    nvs.close();
}

std::string confFile(const std::string &c) {
    auto path = "/littlefs/conf/" + c;
    if (!c.ends_with(".conf")) path += ".conf";
    return path;
}

std::string confFile(const String &c) {
    return confFile(std::string(c.c_str()));
}


// a hand edit of the rect-cal keys wins over a result not yet saved
static void confEdited(const std::string &fn, const String &key) {
    if (fn.ends_with("/coil.conf") && key.startsWith("rect_offset")) mppt.rectCal.discard();
}

// a set secret or a URI with credentials prints as *** (console output reaches every log sink)
static const char *secretVal(const char *key, const char *v) {
    return *v && (isSecretKey(key) || secretCut(v) >= 0) ? "***" : v;
}

// set-config <file> <key> <value...>  — value may contain spaces, so join the trailing tokens.
//   set-config coil.conf L0 50            set-config mqtt.conf broker_uri mqtt://192.168.1.134:1882
//   set-config limits.conf lv_i_max 35    set-config charger.conf cv_eoc 3.53
static void cmdSetConfig(cmd *c) {
    Command cc(c);
    if (cc.countArgs() < 3)
        CMD_FAIL_RETURN("set-config: expected <file> <key> <value>");
    auto fn = confFile(cc.getArg(0).getValue());
    auto key = cc.getArg(1).getValue();
    String val = cc.getArg(2).getValue();
    for (int i = 3; i < cc.countArgs(); ++i) val += " " + cc.getArg(i).getValue();
    ConfFile conf{fn.c_str()};
    auto oldVal = conf.getString(key.c_str(), "");
    if (oldVal != val.c_str() && !conf.add({{key.c_str(), val.c_str()}}, true))
        CMD_FAIL_RETURN("set-config: cannot write %s, '%s' unchanged", fn.c_str(), key.c_str());
    confEdited(fn, key);
    ESP_LOGI("main", "Set conf '%s:%s' = '%s' (was %s)", fn.c_str(), key.c_str(), secretVal(key.c_str(), val.c_str()),
             secretVal(key.c_str(), oldVal.c_str()));
}


// del-config <file> <key>  — remove a key; the whole line (incl. inline comment) is deleted.
static void cmdDelConfig(cmd *c) {
    Command cc(c);
    if (cc.countArgs() < 2)
        CMD_FAIL_RETURN("del-config: expected <file> <key>");
    auto fn = confFile(cc.getArg(0).getValue());
    auto key = cc.getArg(1).getValue();
    ConfFile conf{fn.c_str()};
    if (!conf.has(key.c_str()))
        CMD_FAIL_RETURN("del-config: key '%s' not found in %s", key.c_str(), fn.c_str());
    if (!conf.remove(key.c_str()))
        CMD_FAIL_RETURN("del-config: cannot write %s, '%s' kept", fn.c_str(), key.c_str());
    confEdited(fn, key);
    ESP_LOGI("main", "Deleted conf '%s:%s'", fn.c_str(), key.c_str());
}

// get-config <file> [key]  — print one key, or dump the whole file.
static void cmdGetConfig(cmd *c) {
    Command cc(c);
    if (cc.countArgs() < 1)
        CMD_FAIL_RETURN("get-config: expected <file> [key]");
    auto fn = confFile(cc.getArg(0).getValue());
    ConfFile conf{fn.c_str()};
    if (cc.countArgs() >= 2) {
        auto key = cc.getArg(1).getValue();
        auto val = conf.getString(key.c_str(), "");
        UART_LOG("Conf '%s:%s' = '%s'", fn.c_str(), key.c_str(), secretVal(key.c_str(), val.c_str()));
    } else {
        for (const auto &key: conf.keys())
            ESP_LOGI("main", "Conf '%s:%s' = '%s'", fn.c_str(), key.c_str(), secretVal(key.c_str(), conf.getString(key).c_str()));
    }
}

// conf-hash — fingerprint of the conf files (definition: conf_hash.h), now and as loaded at boot.
static void cmdConfHash(cmd *) {
    const uint8_t *boot = confHashBoot();
    const std::string bootHex = boot ? confHashHex(boot, 32) : "unknown";
    uint8_t sha[32];
    std::string err;
    const int files = confHash(CONF_HASH_ROOT, sha, err, [](const char *name, const uint8_t *s) {
        UART_LOG("conf-file %s sha256=%s", name, confHashHex(s, 8).c_str());
    });
    if (files < 0) {
        UART_LOG("conf sha256=unknown boot=%s error=%s", bootHex.c_str(), err.c_str());
        CMD_FAIL_RETURN("conf-hash failed");
    }
    UART_LOG("conf sha256=%s boot=%s files=%d", confHashHex(sha, 32).c_str(), bootHex.c_str(), files);
}

// conf-check — re-read the parameter confs and warn about keys no loader reads (typos / obsolete,
// e.g. cv_min when the firmware reads cv_float). Reuses the real loaders (pure: no hardware side
// effects) so the recognized-key set never drifts. Unlike the boot-time check this runs in console
// context, so the warnings reach the telnet/MQTT client that issued the command.
bool confCheck(const char *chargerPath, const char *limitsPath) {
    bool ok = true;
    auto check = [&](const char *path, auto &&load) {
        struct stat st;
        if (!path || stat(path, &st) != 0) return;
        ConfFile cf{path, true}; // an empty file still goes through the loader (missing keys throw)
        try { load(cf); }
        catch (const std::exception &e) { ok = false; UART_LOG("%s: %s", path, e.what()); }
        catch (...) { ok = false; UART_LOG("%s: load failed", path); }
        cf.warnUnknownKeys();
    };
    check(chargerPath, [](const ConfFile &cf) { BatChargerParams p; p.load(cf); });
    check(limitsPath, [](const ConfFile &cf) { Limits l{cf}; (void) l; WireGuardConf w; wireGuardConf(cf, w); });
    return ok;
}

static void cmdConfCheck(cmd *) {
    if (!confCheck("/littlefs/conf/charger.conf", "/littlefs/conf/limits.conf"))
        CMD_FAIL_RETURN("conf-check failed");
    UART_LOG("conf-check done");
}

static void cmdVset(cmd *c) {
    Command cc(c);
    float v;
    if (!argFloat(cc, 0, v))
        CMD_FAIL_RETURN("vset: expected a number");
    // a pack voltage without its cell count leaves the float fallback unset
    if (mppt.charger.params.awaitingDetect())
        CMD_FAIL_RETURN("vset: vout_max=auto, no pack detected yet");
    // 0 is not "no limit": it clamps the OV threshold to ~0 and wedges the converter (issue #58).
    if (v > 0 and v <= 999) {
        mppt.charger.params.Vbat_max = v;
        mppt.charger.params.vbatMaxExplicit = true;
    } else
        CMD_FAIL_RETURN("vset: out of range (0,999]");
}

static void cmdIset(cmd *c) {
    Command cc(c);
    float v;
    if (!argFloat(cc, 0, v))
        CMD_FAIL_RETURN("iset: expected a number");
    if (v >= 0 and v <= 999) mppt.charger.params.Ibat_lim = v;
    else
        CMD_FAIL_RETURN("iset: out of range [0,999]");
}

static void cmdOvset(cmd *c) {
    Command cc(c);
    float v;
    if (!argFloat(cc, 0, v))
        CMD_FAIL_RETURN("ovset: expected a number");
    if (v > 0 and v <= 999) {
        const float requested = mppt.getRequestedPsuSetpoint();
        if (std::isfinite(requested) && requested >= 0.98f * v)
            CMD_FAIL_RETURN("ovset: %.2fV conflicts with PSU setpoint %.2fV", v, requested);
        mppt.setExplicitOvLimit(v);
        UART_LOG("ovset: hard OV limit = %.2fV", v);
    } else if (v == 0) {
        mppt.setExplicitOvLimit(NAN);
        UART_LOG("ovset: cleared, OV threshold reverts to derived");
    } else
        CMD_FAIL_RETURN("ovset: out of range [0,999] (0 = clear)");
}

static void pvLogShade(const PvShade &sh) {
    if (!sh.on()) return;
    char b[40];
    int o = 0;
    for (uint8_t j = 0; j < sh.n; ++j)
        o += snprintf(b + o, sizeof(b) - o, "%s%.2f", j ? "," : "", sh.g[j] * 0.01f);
    UART_LOG("  shade %s", b);
}

#if CONFIG_FUGU_WITH_PSU
static void cmdPsu(cmd *c) {
    Command cc(c);
    auto arg = cc.countArgs() >= 1 ? cc.getArg(0).getValue() : String();
    if (arg.length() == 0) {
        if (g_app.psuMode() || mppt.hasPendingPsuCommand()) {
            UART_LOG("PSU: %s vset=%.2fV %s trips=%u",
                     g_app.psuMode() ? "on" : "pending", mppt.getRequestedPsuSetpoint(),
                     mppt.isPsuLatched() ? "LATCHED" : mppt.isPsuEscalated() ? "escalated" : "ok",
                     mppt.getPsuTripCount());
        } else {
            UART_LOG("PSU: off");
        }
        return;
    }
    if (measBusy(c)) return;
    if (arg == "off") {
        if (!g_app.psuMode() && !mppt.hasPendingPsuCommand())
            CMD_FAIL_RETURN("psu: not in PSU mode");
        const auto ticket = mppt.requestPsuOff();
        if (!waitPsuCommand(ticket))
            CMD_FAIL_RETURN("psu off: RT transition timed out");
        return;
    }
    float v;
    if (!argFloat(cc, 0, v) || v <= 0 || v > mppt.limits.Vout_max)
        CMD_FAIL_RETURN("psu: out of range (0,%.1f]", mppt.limits.Vout_max);
    const auto ticket = mppt.queuePsuSetpoint(v);
    if (!ticket)
        CMD_FAIL_RETURN("psu: %s", psuErrorText(mppt.getLastPsuRequestError()));
    if (!waitPsuCommand(ticket))
        CMD_FAIL_RETURN("psu: RT transition timed out");
    if (mppt.getPsuCommandError(ticket) != PsuSetpointError::None)
        CMD_FAIL_RETURN("psu: %s", psuErrorText(mppt.getPsuCommandError(ticket)));
    ESP_LOGI("main", "PSU mode, vset=%.2fV", v);
}

// Local power maxima of the curve, scanned over the current.
static void pvLogPeaks(float isc, float voc, float k, const PvShade &sh) {
    PvModel m;
    m.set(isc, voc, k);
    float pPrev = 0, vPrev = voc, pPrev2 = -1;
    constexpr int N = 600;
    for (int j = 1; j <= N; ++j) {
        const float i = isc * (float) j / N, v = sh.voltage(m, i), p = v * i;
        if (pPrev > p && pPrev >= pPrev2 && pPrev > 0.1f)
            UART_LOG("  peak %.1fV %.0fW", vPrev, pPrev);
        pPrev2 = pPrev, pPrev = p, vPrev = v;
    }
}

// pv                      — status
// pv <isc> <voc> [k]      — enter PV-sim (output follows the panel curve) / update the curve
// pv scale <s>            — irradiance knob: Isc = s * last full-command Isc
// pv shade <g1,g2,..>|off — series string of 2..6 substrings with bypass diodes, Isc factors g
// pv off                  — ramp to 0, manual mode (deliberately NOT the MPPT fallback of `psu off`)
static void cmdPv(cmd *c) {
    Command cc(c);
    int n = cc.countArgs();
    auto arg = n >= 1 ? cc.getArg(0).getValue() : String();
    PvShade sh;
    if (arg.length() == 0) {
        float isc, voc, k;
        if (mppt.getPvCurve(isc, voc, k, &sh)) {
            UART_LOG("PV: on Isc=%.2fA Voc=%.2fV k=%.2f  Vset=%.2fV Vout=%.2fV Iout=%.2fA %s trips=%u",
                     isc, voc, k, mppt.getPsuSetpoint(),
                     sensors.Vout->med3.get(), sensors.Iout->med3.get(),
                     mppt.isPsuLatched() ? "LATCHED" : mppt.isPsuEscalated() ? "escalated" : "ok",
                     mppt.getPsuTripCount());
            pvLogShade(sh);
            pvLogPeaks(isc, voc, k, sh);
        } else {
            UART_LOG("PV: off");
        }
        return;
    }
    if (measBusy(c)) return;
    if (arg == "off") {
        if (!mppt.isPvActive() && !mppt.hasPendingPsuCommand())
            CMD_FAIL_RETURN("pv: not in PV mode");
        const auto ticket = mppt.requestPsuManual(0, -1);
        if (!waitPsuCommand(ticket))
            CMD_FAIL_RETURN("pv off: RT transition timed out");
        return;
    }
    float isc, voc, k;
    bool rebase = true;
    const bool active = mppt.getPvCurve(isc, voc, k, &sh);
    if (arg == "shade") {
        if (!active)
            CMD_FAIL_RETURN("pv shade: no active curve");
        if (n < 2 || !sh.parse(cc.getArg(1).getValue().c_str()))
            CMD_FAIL_RETURN("pv shade: expected 2..6 factors in (0,1] (e.g. 1,1,0.4) or off");
        rebase = false;
    } else if (arg == "scale") {
        float s;
        if (!argFloat(cc, 1, s) || s <= 0 || s > 1.2f)
            CMD_FAIL_RETURN("pv scale: expected (0,1.2]");
        float base = mppt.getPvBaseIsc();
        if (!active || !std::isfinite(base))
            CMD_FAIL_RETURN("pv scale: no active curve");
        isc = base * s;
        rebase = false;
    } else {
        k = 0.8f;
        if (!argFloat(cc, 0, isc) || !argFloat(cc, 1, voc) || (n >= 3 && !argFloat(cc, 2, k)))
            CMD_FAIL_RETURN("pv: expected <isc> <voc> [k]");
    }
    if (!MpptController::validPvParams(isc, voc, k))
        CMD_FAIL_RETURN("pv: expected isc>0 voc>0 k in [0.5,0.95]");
    if (!active) sh = {};
    const auto ticket = mppt.queuePvCurve(isc, voc, k, false, rebase, sh);
    if (!ticket)
        CMD_FAIL_RETURN("pv: %s", psuErrorText(mppt.getLastPsuRequestError()));
    if (!waitPsuCommand(ticket))
        CMD_FAIL_RETURN("pv: RT transition timed out");
    if (mppt.getPsuCommandError(ticket) != PsuSetpointError::None)
        CMD_FAIL_RETURN("pv: %s", psuErrorText(mppt.getPsuCommandError(ticket)));
    ESP_LOGI("main", "PV-sim Isc=%.2fA Voc=%.2fV k=%.2f%s", isc, voc, k, sh.on() ? " shaded" : "");
    pvLogPeaks(isc, voc, k, sh);
}
#endif

static void cmdWire(cmd *) {
    const auto &g = mppt.charger.wireGuard;
    static const char *const lvl[]{"ok", "warn", "derate", "trip"};
    if (!g.conf.mode) {
        UART_LOG("wire: off (limits.conf wire_protect=0)");
        return;
    }
    UART_LOG("wire: %s, level %s (effective %s)%s%s, wf=%d Pw=%.2fW cap=%.1fA", g.armed() ? "armed" : "warn only",
             lvl[g.level()], lvl[g.effLevel()], g.tripped() ? " TRIPPED" : "", g.rbWarn() ? ", shared path up" : "",
             g.wf(), g.pw(), g.ioutCap());
    UART_LOG("  baseline: %u days%s, Rc %.2f a %.1fmV Rb %.2f Ro %.2f mOhm, unverified days %u",
             g.dayCount(), g.learning() ? " (learning)" : "", g.baseRc() * 1e3f, g.baseA() * 1e3f,
             g.baseRb() * 1e3f, g.baseRo() * 1e3f, g.blob().unvDays);
    const char *names[]{"fast", "Rc", "Ro", "Rb"};
    const WireGuard::PathView pv[]{g.fastPath(), g.rcPath(), g.roPath(), g.rbPath()};
    const bool ref[]{g.rcRef(), g.rcRef(), g.roRef(), g.rbRef()};
    for (int i = 0; i < 4; ++i) {
        if (ref[i]) UART_LOG("  %-4s %s dR %.2f mOhm Pw %.2f W", names[i], pv[i].fresh ? "fresh" : "stale",
                             pv[i].dr * 1e3f, pv[i].pw);
        else UART_LOG("  %-4s stale, baseline %u/%u days", names[i], i == 2 ? g.blob().nRo : g.blob().nRc,
                      WireGuard::BASE_MIN_DAYS);
    }
    UART_LOG("  offset %.1fmV slope %.2fmOhm vs baseline, day dR warn %d", g.ofs() * 1e3f, g.slope() * 1e3f,
             (int) g.dayWarn());
    UART_LOG("  Iref %.1fA, today %u Rc samples (Iout sd %.1fA), %u sweeps, counts %d", g.iRef(), g.daySamples(),
             g.dayCurrentSd(), g.daySweeps(), (int) g.dayCounts());
}

static void cmdWireReset(cmd *) {
    g_wireResetReq = true;
    UART_LOG("wire: latch and baseline cleared, learning restarts");
}

static const char *const HOLIDAY_MODES[] = {"auto", "on", "off"};

// holiday [auto|on|off] — holiday ceiling: detection, forced on, or disabled
static void cmdHoliday(cmd *c) {
    Command cc(c);
    auto &chg = mppt.charger;
    if (cc.countArgs() >= 1) {
        auto arg = cc.getArg(0).getValue();
        uint8_t o = 3;
        for (uint8_t i = 0; i < 3; ++i) if (arg == HOLIDAY_MODES[i]) o = i;
        if (o == 3) CMD_FAIL_RETURN("holiday: use auto|on|off");
        if (!(chg.params.holiday_charge > 0.f)) CMD_FAIL_RETURN("holiday: set charger.conf holiday_charge first");
        chg.setHolidayOverride(o);
    }
    UART_LOG("holiday %s (%s), ceiling %.0f%%", chg.holiday() ? "ON" : "off", HOLIDAY_MODES[chg.holidayOverride()],
             chg.ceiling() * 100.f);
}

static void balanceLine(char *b, size_t n) {
    auto &chg = mppt.charger;
    const auto &p = chg.params;
    const char *why = chg.balanceDeferWhy();
    if (!p.balance_interval_s) snprintf(b, n, "balance off");
    else if (!chg.balanceDue()) snprintf(b, n, "balance in %.1fd", (p.balance_interval_s - chg.balanceAgeS()) / 86400.f);
    else snprintf(b, n, "balance due (%s) %u/%lus", chg.balanceRunning() ? "running" : why ? why : "next charge",
                  (unsigned) chg.balanceInBandS(), (unsigned long) p.balance_hold_s);
}

// balance [now|skip] — top-balancing run state; make it due now, or mark it done
static void cmdBalance(cmd *c) {
    Command cc(c);
    auto &chg = mppt.charger;
    if (cc.countArgs() >= 1) {
        auto arg = cc.getArg(0).getValue();
        if (!chg.params.balance_interval_s || (arg != "now" && arg != "skip"))
            CMD_FAIL_RETURN("balance now|skip, needs balance_interval");
        arg == "now" ? chg.balanceNow() : chg.balanceSkip();
        return;
    }
    char b[64];
    balanceLine(b, sizeof b);
    UART_LOG("%s", b);
}

// status  — charger/battery snapshot: termination state, effective limits, termination line and BMS feed.
// latched HS-short state, short
static const char *hsStateStr(char *b, size_t n) {
    switch (mppt.hsFaultState()) {
        case MpptController::HsClamped: return "LS clamped";
        case MpptController::HsRamp: return "LS ramp";
        case MpptController::HsPending:
            snprintf(b, n, "gates off, clamp pending (Vin<%.0fV %lus)", (double) HsClampArm::VinMax,
                     (unsigned long) (mppt.hsPendingHeldMs((uint32_t) wallClockMs()) / 1000));
            return b;
        default: return "gates off";
    }
}

static void cmdStatus(cmd *) {
    if (g_app.setupErr) UART_LOG("setup: FAILED: %s", g_app.setupErrMsg);
    auto &chg = mppt.charger;
    const auto &p = chg.params;
    auto &bs = chg.batSt;
    bool vcOk = bs.haveValidCellVoltage();
    uint32_t ageS = bs.vcell_high > 0 ? coarseTicksToSec(bs.cellAgeTicks()) : 0;

    char hold[64];
    if (chg.balanceRunning())
        snprintf(hold, sizeof(hold), "CV HOLD balance (%u/%lus, ilim %.1fA)", (unsigned) chg.balanceInBandS(),
                 (unsigned long) p.balance_hold_s, chg.Iout_max());
    else
        snprintf(hold, sizeof(hold), "CV HOLD (%lus, ilim %.1fA)", (unsigned long) chg.termCond.cvHoldS(), chg.Iout_max());
    const char *const st[]{"charging", hold, hold, "PARTIAL HOLD (load-following)", "BLOCKED (pack cold)",
                           "TERMINATED (float)"};
    UART_LOG("Charger: %s", st[chg.state()]);
    UART_LOG("  Vbat_max=%.2fV%s Vout_max=%.2fV  Ibat_lim=%.1fA Iout_max=%.1fA", p.Vbat_max,
             p.awaitingDetect() ? " (auto, no pack yet)" : p.vbatAuto ? " (auto)" : "", chg.Vout_max(), p.Ibat_lim,
             chg.Iout_max());
    const float ovLimit = mppt.getExplicitOvLimit();
    if (std::isfinite(ovLimit) && ovLimit > 0)
        UART_LOG("  OV limit=%.2fV (explicit ovset%s)", ovLimit,
                 p.vbatMaxExplicit ? ", vset locked" : "");
    else if (g_app.psuMode())
        UART_LOG("  OV limit=%.2fV (derived from the PSU setpoint)", mppt.computeOvThreshold());
    else
        UART_LOG("  OV limit=derived from Vbat_max%s", p.vbatMaxExplicit ? " (vset locked)" : "");
    UART_LOG("  v_term=%.3fV cv_float=%.3f cv_eoc=%.3f  Cbat=%.1fAh recharge_dod=%.2f",
             chg.termCond.v_term(), p.cv_min, p.cv_eoc, p.Cbat, p.recharge_dod);
    UART_LOG("  BMS vcell_high=%.3fV (%s, %lus ago)  ibat=%.2fA (%s, %lus ago)  ahSinceFull=%.2fAh (%s)  vout_avg=%.2fV",
             bs.vcell_high, vcOk ? "ok" : "stale/na", ageS,
             bs.ibatSmoothed(), chg.ibatFresh() ? "ok" : "stale/na", (unsigned long) chg.ibatAgeS(),
             bs.coulombCounter.deficit(), bs.coulombCounter.bmsSource() ? "bms" : "ibat", bs.vout_avg.get());
    if ((bool) chg.termCond && std::isfinite(p.Cbat) && p.Cbat > 0.f && p.recharge_dod > 0.f)
        UART_LOG("  DoD since full: %.0f%% / %.0f%% to recharge", bs.coulombCounter.depth() / p.Cbat * 100.f,
                 p.recharge_dod * 100.f);
    if (chg.ceiling() > 0.f) {
        uint32_t age = chg.fullAgeS();
        if (age != UINT32_MAX)
            UART_LOG("  ceiling=%.0f%%  last full %.1fh ago (full charge every %.1f d)%s", chg.ceiling() * 100.f,
                     age / 3600.f, p.full_charge_interval_s / 86400.f, chg.deficitKnown() ? "" : ", Ah counter unknown");
        else
            UART_LOG("  ceiling=%.0f%%  last full unknown (charging to full first)", chg.ceiling() * 100.f);
    }
    if (p.holiday_charge > 0.f)
        UART_LOG("  holiday %s (%s)  quiet days %u/%u (gaps %u)  today: discharging %.1fh, mean %.2f/%.2fA",
                 chg.holiday() ? "ON" : "off", HOLIDAY_MODES[chg.holidayOverride()], (unsigned) chg.holidayStreak(),
                 (unsigned) p.holiday_days, (unsigned) chg.holidayGaps(), chg.holidayDischargeH(), chg.holidayMeanA(),
                 p.holiday_ibat);
    if (p.balance_interval_s) {
        char b[64];
        balanceLine(b, sizeof b);
        UART_LOG("  %s", b);
    }
    if (chg.haveTemp())
        UART_LOG("  pack temp %.1f..%.1f°C (%lus ago)  charge min %.0f derate %.0f max %.0f°C",
                 chg.tempMin(), chg.tempMax(), (unsigned long) chg.tempAgeS(),
                 p.bat_temp_min, p.bat_temp_derate, p.bat_temp_max);
    else if (chg.tempAgeS())
        UART_LOG("  pack temp expired (%lus ago)", (unsigned long) chg.tempAgeS());
    if (mppt.hsFaultState()) {
        char b[48];
        UART_LOG("HS-short: LATCHED (%c), %s (selftest clear)", mppt.hsFaultWhy(), hsStateStr(b, sizeof b));
    }
    else if (converter.startInhibited(SynchronousConverter::InhibitHs))
        UART_LOG("HS-short: start waits for a clean idle check");
    selftestPrintState();
    if (g_app.psuMode()) {
        UART_LOG("PSU: vset=%.2fV (ramp %.2fV) %s trips=%u", mppt.getRequestedPsuSetpoint(), mppt.getPsuSetpoint(),
                 mppt.isPsuLatched() ? "LATCHED" : mppt.isPsuEscalated() ? "escalated" : "ok",
                 mppt.getPsuTripCount());
        float isc, voc, k;
        PvShade sh;
        if (mppt.getPvCurve(isc, voc, k, &sh)) {
            UART_LOG("  PV curve Isc=%.2fA Voc=%.2fV k=%.2f", isc, voc, k);
            pvLogShade(sh);
        }
    }
}

// svc [list]                       svc on|off|restart|rs <name>
// svc log <name> <error|warn|info>
static void cmdService(cmd *c) {
    Command cc(c);
    int n = cc.countArgs();
    String sub = n >= 1 ? cc.getArg(0).getValue() : String("list");

    if (sub == "list") {
        UART_LOG("%-10s %-8s %-6s %-8s %s", "NAME", "STATE", "LOG", "ENABLED", "DETAIL");
        for (auto *s: g_services.all()) {
            auto detail = s->statusDetail();
            auto st = s->state();
            const char *color = st == ServiceState::Running
                                    ? "\x1b[32m" // green
                                    : st == ServiceState::Failed
                                          ? "\x1b[31m" // red
                                          : "\x1b[90m"; // gray
            char state[24];
            snprintf(state, sizeof(state), "%s%-8s\x1b[0m", color, stateStr(st));
            UART_LOG("%-10s %s %-6s %-8s %s", s->name(), state,
                     levelToStr(s->logLevel()), s->enabled() ? "yes" : "no", detail.c_str());
        }
        return;
    }

    if (n < 2)
        CMD_FAIL_RETURN("svc: expected <name>");
    auto name = cc.getArg(1).getValue();
    auto *s = g_services.findByName(name.c_str());
    if (!s)
        CMD_FAIL_RETURN("svc: unknown '%s'", name.c_str());

    if (sub == "on" || sub == "off") {
        const bool on = sub == "on";
        if (!s->setEnabledPersist(on))
            CMD_FAIL_RETURN("svc: cannot save %s state, '%s' unchanged", sub.c_str(), name.c_str());
        if (!on) s->stop();
        else if (!s->start()) UART_LOG("start failed (state=%s)", stateStr(s->state()));
    } else if (sub == "restart" || sub == "rs") {
        if (!s->enabled())
            CMD_FAIL_RETURN("svc: %s is disabled, use 'svc on %s'", s->name(), s->name());
        s->restart();
    } else if (sub == "log") {
        if (n < 3)
            CMD_FAIL_RETURN("svc log: expected <error|warn|info>");
        if (!s->setLogLevel(strToLevel(cc.getArg(2).getValue().c_str()), /*persist*/ true))
            CMD_FAIL_RETURN("svc log: cannot save the level, '%s' unchanged", name.c_str());
    } else {
        CMD_FAIL_RETURN("svc: unknown subcommand '%s'", sub.c_str());
    }
}

// selftest                 HS-short state, last backflow-switch result
// selftest bflow           backflow-switch test (buck, converter taken over, mode restored)
// selftest abort           stop a running backflow-switch test
// selftest clear           clear a latched HS short and a latched bf-fail
static void cmdSelftest(cmd *c) {
    auto sub = Command(c).getArg(0).getValue();
    if (sub == "bflow") {
#ifdef CONFIG_FUGU_WITH_MEASURE_COIL
        if (isMeasuring())
            CMD_FAIL_RETURN("selftest: measure-coil running");
#endif
#ifdef CONFIG_FUGU_WITH_BLE
        if (otaBleActive())
            CMD_FAIL_RETURN("selftest: OTA running");
#endif
        if (const char *r = bfTestRefusal())
            CMD_FAIL_RETURN("selftest bflow: %s", r);
        if (!bfTestStart())
            CMD_FAIL_RETURN("selftest bflow: refused");
        UART_LOG("selftest bflow: started, usually 10-30 s, up to ~3 min with slow sensor filters");
    } else if (sub == "abort") {
        if (!bfTestBusy())
            CMD_FAIL_RETURN("selftest abort: not running");
        bfTestAbort();
        UART_LOG("selftest bflow: aborting");
    } else if (sub == "clear") {
        const bool hs = mppt.hsFaultState(), bf = mppt.bfFail.load();
        if (!hs && !bf)
            CMD_FAIL_RETURN("selftest: nothing latched");
        if (hs) mppt.hsClear();
        if (bf) selftestClearBf();
        UART_LOG("selftest: cleared%s%s", hs ? " hs-short" : "", bf ? " bf-fail" : "");
    } else if (!sub.length()) {
        char b[48];
        UART_LOG("hs-short: %s%s, idle check %s, zero %.3f A, start %s%s", mppt.hsFaultState() ? "LATCHED, " : "ok",
                 mppt.hsFaultState() ? hsStateStr(b, sizeof b) : "", mppt.hsIdleCheck() ? "on" : "off",
                 mppt.hsZero(), converter.startInhibited(SynchronousConverter::InhibitHs) ? "blocked" : "free",
                 mppt.hsClampConf ? (mppt.hsClampAllowed(false) ? ", clamp armed" : ", clamp not armed") : "");
        selftestPrintState();
        if (bfTestBusy()) UART_LOG("selftest bflow: running");
        else bfTestPrint();
    } else {
        CMD_FAIL_RETURN("selftest: expected bflow|abort|clear");
    }
}

#ifdef CONFIG_FUGU_WITH_MEASURE_COIL
// measure-coil l0|ls [steps|hs] [dwell_ms] [f=<Hz>] [sync] [apply]  — the sweep logic lives in measure_coil.cpp.
static void cmdMeasureCoil(cmd *c) {
    if (adcSampler.isCalibrating())
        CMD_FAIL_RETURN("measure-coil: busy calibrating");
    Command cc(c);
    auto mode = cc.getArg(0).getValue();
    bool ls;
    if (mode == "l0") ls = false;
    else if (mode == "ls") ls = true;
    else
        CMD_FAIL_RETURN("measure-coil: expected l0|ls [args]");

    bool apply = false, sync = false;
    int pos[2] = {0, 0}, np = 0; // steps|hs, dwell_ms (0: per-mode default)
    long fHz = 0;
    for (int i = 1, n = cc.countArgs(); i < n; ++i) {
        auto v = cc.getArg(i).getValue();
        if (v == "apply") apply = true;
        else if (v == "sync") sync = true;
        else if (!strncmp(v.c_str(), "f=", 2)) {
            char *end = nullptr;
            errno = 0;
            fHz = strtol(v.c_str() + 2, &end, 10);
            if (end == v.c_str() + 2 || *end || errno == ERANGE || fHz <= 0)
                CMD_FAIL_RETURN("measure-coil: expected f=<Hz>");
        } else if (np < 2) pos[np++] = v.toInt();
    }
    uint32_t dwellMs = (uint32_t) pos[1];
    if (dwellMs && dwellMs < 200) dwellMs = 200;
    if (fHz && ls)
        CMD_FAIL_RETURN("measure-coil: f= is for l0 only");
    if (ls && converter.boost())
        CMD_FAIL_RETURN("measure-coil: ls is for a buck only");
    if (sync && (ls || converter.boost()))
        CMD_FAIL_RETURN("measure-coil: sync is for l0 on a buck only");
    if (fHz && !converter.getPeriodTicks())
        CMD_FAIL_RETURN("measure-coil: f= needs the MCPWM gate driver build");
    if (fHz && (fHz < (long) kMeasFswMin || fHz > (long) converter.getPwmFrequency()))
        CMD_FAIL_RETURN("measure-coil: f= expects %lu..%lu Hz", (unsigned long) kMeasFswMin,
                        (unsigned long) converter.getPwmFrequency());
    if (fHz == (long) converter.getPwmFrequency()) fHz = 0;
    // drives the half-bridge itself: only from a converter stopped with `dc 0`, where nothing else starts it
    if (!rtLoopRunning())
        CMD_FAIL_RETURN("measure-coil: no RT loop");
    if (g_app.psuMode())
        CMD_FAIL_RETURN("measure-coil: PSU/PV mode on, stop conversion first (dc 0)");
    if (!g_app.manualPwm())
        CMD_FAIL_RETURN("measure-coil: MPPT on, stop conversion first (dc 0)");
    if (mppt.getManualTarget())
        CMD_FAIL_RETURN("measure-coil: manual duty %u set, stop conversion first (dc 0)", (unsigned) mppt.getManualTarget());
    if (mppt.hasPendingPsuCommand())
        CMD_FAIL_RETURN("measure-coil: mode change pending");
    if (mppt.isSweeping())
        CMD_FAIL_RETURN("measure-coil: sweep running");
    if (mppt.rectCal.cal.running())
        CMD_FAIL_RETURN("measure-coil: rect-cal running");
    if (!converter.disabled())
        CMD_FAIL_RETURN("measure-coil: converter still on, wait for the ramp-down");
    if (converter.startInhibited((uint8_t) ~SynchronousConverter::InhibitIdle))
        CMD_FAIL_RETURN("measure-coil: start blocked (%s)", mppt.startBlockReason());
    if (mppt.inBackoff())
        CMD_FAIL_RETURN("measure-coil: protection backoff");
#ifdef CONFIG_FUGU_WITH_BLE
    if (otaBleActive())
        CMD_FAIL_RETURN("measure-coil: OTA running");
#endif
    if (!converter.deadTimeIdle())
        CMD_FAIL_RETURN("measure-coil: a dead-time change is still queued");
    if (!converter.pwmFreqIdle())
        CMD_FAIL_RETURN("measure-coil: a frequency change is still queued");
    if (bfTestBusy())
        CMD_FAIL_RETURN("measure-coil: selftest running");
    if (!measureCoilStart(ls, apply, pos[0], dwellMs, (uint32_t) fHz, sync))
        CMD_FAIL_RETURN("measure-coil: already running");
}
#endif // CONFIG_FUGU_WITH_MEASURE_COIL

#if CONFIG_FUGU_WITH_VCONV
// vconv                           dump state
// vconv pv <isc> <voc> [k]        update PV params
// vconv bat <vbat>                update battery voltage
// vconv set <key> <value>         in-memory setter (plant_c_in, plant_c_out, r_bat, l, vbat_ac_amp, vbat_ac_freq, vbat_ac_shape, diode_vf,
//                                 ring_c, ring_q, bflow_model, bflow_stuck, hs_short, i_unidir, ntc_v, sun_day_s, sun_night_s, sun_peak, cloud_rate, cloud_depth, cloud_len_s, cloud_edge_s, sun_seed)
// vconv sun                       irradiance state
static void cmdVconv(cmd *c) {
    Command cc(c);
    int n = cc.countArgs();
    if (n == 0 || cc.getArg(0).getValue().length() == 0) {
        const auto &p = g_vconv.getPwm();
        UART_LOG("vconv: Vin=%.3fV Vout=%.3fV IL=%.3fA Iin=%.3fA Iout=%.3fA %s pwm[ctrl=%u rect=%u max=%u f=%uHz]%s",
                 g_vconv.getVin(), g_vconv.getVout(), g_vconv.getIL(),
                 g_vconv.getIinAvg(), g_vconv.getIoutAvg(),
                 g_vconv.inDcm() ? "DCM" : "CCM",
                 (unsigned) p.pwmCtrl, (unsigned) p.pwmRect, (unsigned) p.pwmMax,
                 (unsigned) p.pwmFreq,
                 g_vconv.errored() ? " ERR" : "");
        return;
    }
    auto sub = cc.getArg(0).getValue();
    if (sub == "pv") {
        if (n < 3) CMD_FAIL_RETURN("vconv pv: expected <isc> <voc> [k]");
        float isc = cc.getArg(1).getValue().toFloat();
        float voc = cc.getArg(2).getValue().toFloat();
        float k   = n >= 4 ? cc.getArg(3).getValue().toFloat() : g_vconv.getPvK();
        if (isc <= 0 || voc <= 0 || k <= 0 || k >= 1)
            CMD_FAIL_RETURN("vconv pv: bad args (isc>0, voc>0, 0<k<1)");
        g_vconv.setPv(isc, voc, k);
        UART_LOG("vconv: PV set Isc=%.2f Voc=%.2f k=%.2f", isc, voc, k);
    } else if (sub == "bat") {
        if (n < 2) CMD_FAIL_RETURN("vconv bat: expected <v|open|short>");
        auto arg1 = cc.getArg(1).getValue();
        if (arg1 == "open") {
            // Open-circuit output: huge series R, V_bat=0. I_bat≈0 so V_out
            // integrates whatever the converter delivers (model caps at 2·Voc;
            // firmware OVP should trip first).
            g_vconv.setBat(0.0f, 1e9f);
            UART_LOG("vconv: bat open-circuit (v_bat=0, r_bat=1e9)");
        } else if (arg1 == "short") {
            // Hard short on the output: V_bat=0, tiny R_bat. Backward-Euler on the
            // V_out node makes this stable at arbitrary r_bat — V_out collapses to
            // ~iOutAvg·rbat, the firmware should see Iout climb and trip OCP.
            g_vconv.setBat(0.0f, 1e-3f);
            UART_LOG("vconv: bat short-circuit (v_bat=0, r_bat=1e-3)");
        } else {
            float v = arg1.toFloat();
            if (v <= 0 || v > 200) CMD_FAIL_RETURN("vconv bat: out of range");
            g_vconv.setBat(v, g_vconv.getRbat());
            UART_LOG("vconv: Vbat=%.2f", v);
        }
    } else if (sub == "sun") {
        const auto &s = g_vconv.sun();
        UART_LOG("vconv sun: G=%.4f clear=%.4f cloud=%.4f t=%.1f/%.1f+%.1fs Isc=%.3fA Voc=%.2fV",
                 g_vconv.getSunG(), s.gClear, s.cloud, s.t, s.day, s.nightS(), g_vconv.getIsc(), g_vconv.getVoc());
    } else if (sub == "set") {
        if (n < 3) CMD_FAIL_RETURN("vconv set: expected <key> <value>");
        auto key = cc.getArg(1).getValue();
        float v  = cc.getArg(2).getValue().toFloat();
        auto &sun = g_vconv.sun();
        if (!std::isfinite(v)) CMD_FAIL_RETURN("vconv set: bad value");
        if (key == "plant_c_in")   g_vconv.setPassives(v, g_vconv.getCout(), g_vconv.getL());
        else if (key == "plant_c_out") g_vconv.setPassives(g_vconv.getCin(), v, g_vconv.getL());
        else if (key == "l")        g_vconv.setPassives(g_vconv.getCin(), g_vconv.getCout(), v);
        else if (key == "r_bat")    g_vconv.setBat(g_vconv.getVbat(), v);
        else if (key == "vbat_ac_amp")  g_vconv.setBatRipple(v, g_vconv.getVbatAcFreq(), g_vconv.getVbatAcShape());
        else if (key == "vbat_ac_freq") g_vconv.setBatRipple(g_vconv.getVbatAcAmp(), v, g_vconv.getVbatAcShape());
        else if (key == "vbat_ac_shape") g_vconv.setBatRipple(g_vconv.getVbatAcAmp(), g_vconv.getVbatAcFreq(), (int) v);
        else if (key == "diode_vf") g_vconv.setDiodeVf(v);
        else if (key == "bflow_model") g_vconv.bfModel = v != 0;
        else if (key == "bflow_stuck") g_vconv.bfStuck = (uint8_t) v;
        else if (key == "hs_short") g_vconv.hsShort = (uint8_t) v;
        else if (key == "i_unidir") g_vconv.iUnidir = v != 0;
        else if (key == "ntc_v")    g_vconv.ntcV = v;
        else if (key == "ring_c")   g_vconv.setRing(v, g_vconv.getRingQ());
        else if (key == "ring_q")   g_vconv.setRing(g_vconv.getRingC(), v);
        else if (key == "sun_day_s")    sun.setDay(v);
        else if (key == "sun_night_s")  sun.night = v;
        else if (key == "sun_peak")     sun.peak = v;
        else if (key == "cloud_rate")   sun.rate = v;
        else if (key == "cloud_depth")  sun.depth = v;
        else if (key == "cloud_len_s")  sun.len = v;
        else if (key == "cloud_edge_s") sun.edge = v;
        else if (key == "sun_seed")     sun.reset((uint32_t) std::min<double>(std::fabs(v), 4294967295.0));
        else CMD_FAIL_RETURN("vconv set: unknown key '%s'", key.c_str());
        UART_LOG("vconv: %s=%.6g", key.c_str(), v);
    } else {
        CMD_FAIL_RETURN("vconv: expected pv|bat|sun|set");
    }
}
#endif

// --- Scripts ---------------------------------------------------------------------------------
#ifdef CONFIG_FUGU_WITH_SCRIPTS

static const char *SCRIPTS_DIR = "/littlefs/scripts";

static std::string scriptPath(const String &name) {
    return std::string(SCRIPTS_DIR) + "/" + name.c_str();
}

static void cmdSleep(cmd *c) {
    auto v = Command(c).getArg(0).getValue();
    if (v.length() == 0)
        CMD_FAIL_RETURN("sleep: expected <seconds>");
    float secs = v.toFloat();
    if (!std::isfinite(secs) || secs <= 0 || secs > 60)
        CMD_FAIL_RETURN("sleep: %.2f out of range (0..60)", (double) secs);
    int ms = (int) (secs * 1000);
    UART_LOG("sleep %.3f", (double) secs);
    for (int t = 0; t < ms; t += 100) {
        vTaskDelay(pdMS_TO_TICKS(std::min(100, ms - t)));
        consoleFlush();
    }
}

static int s_scriptDepth = 0;

static void cmdRun(cmd *c) {
    Command cc(c);
    auto name = cc.getArg(0).getValue();
    if (name.length() == 0)
        CMD_FAIL_RETURN("run: expected <script name>");
    if (name.indexOf('/') != -1 || name.indexOf("..") != -1)
        CMD_FAIL_RETURN("run: invalid script name '%s'", name.c_str());
    if (s_scriptDepth >= 3)
        CMD_FAIL_RETURN("run: script nesting too deep (max 3)");
    auto path = scriptPath(name);
    FILE *f = fopen(path.c_str(), "r");
    if (!f) {
        path += ".txt";
        f = fopen(path.c_str(), "r");
    }
    if (!f)
        CMD_FAIL_RETURN("run: script not found: %s", name.c_str());
    s_scriptDepth++;
    try {
        char line[200];
        int ln = 0, executed = 0;
        while (fgets(line, sizeof(line), f)) {
            ++ln;
            size_t l = strlen(line);
            while (l && (line[l - 1] == '\n' || line[l - 1] == '\r')) line[--l] = 0;
            char *p = line;
            while (*p == ' ' || *p == '\t') ++p;
            if (*p == '#' || *p == 0) continue;
            String inp(p);
            inp.trim();
            if (!inp.length()) continue;
            const int m = secretCut(p);
            UART_LOG("=== %.*s%s ===", CG_MASKED(p, m));
            bool ok = handleCommand(inp);
            consoleFlush();
            if (!ok) {
                s_scriptDepth--;
                fclose(f);
                CMD_FAIL_RETURN("run: aborted at line %d", ln);
            }
            ++executed;
        }
        fclose(f);
        s_scriptDepth--;
        UART_LOG("run: %s — %d commands, %d lines", name.c_str(), executed, ln);
    } catch (...) {
        s_scriptDepth--;
        fclose(f);
        throw;
    }
}

static void cmdScripts(cmd *) {
    DIR *d = opendir(SCRIPTS_DIR);
    if (!d) {
        UART_LOG("scripts: none (no %s directory)", SCRIPTS_DIR);
        return;
    }
    int n = 0;
    for (dirent *e; (e = readdir(d));) {
        std::string full = std::string(SCRIPTS_DIR) + "/" + e->d_name;
        struct stat st;
        bool ok = stat(full.c_str(), &st) == 0;
        UART_LOG("%8lu  %s", (unsigned long) (ok ? st.st_size : 0), e->d_name);
        ++n;
    }
    closedir(d);
    UART_LOG("scripts: %d in %s", n, SCRIPTS_DIR);
}

static void cmdScriptSet(cmd *c) {
    Command cc(c);
    if (cc.countArgs() < 2)
        CMD_FAIL_RETURN("script-set: expected <name> <cmd; cmd; ...>");
    auto name = cc.getArg(0).getValue();
    if (name.indexOf('/') != -1 || name.indexOf("..") != -1 || name.length() == 0)
        CMD_FAIL_RETURN("script-set: invalid name '%s'", name.c_str());
    std::string body;
    for (int i = 1; i < cc.countArgs(); ++i) {
        if (i > 1) body += ' ';
        body += cc.getArg(i).getValue().c_str();
    }
    mkdir(SCRIPTS_DIR, 0777);
    auto path = scriptPath(name) + ".txt";
    struct stat probe;
    bool exists = stat(path.c_str(), &probe) == 0;
    FILE *f = fopen(path.c_str(), "w");
    if (!f)
        CMD_FAIL_RETURN("script-set: cannot write %s", path.c_str());
    int lines = 0;
    size_t start = 0;
    while (start <= body.size()) {
        size_t sep = body.find(';', start);
        std::string seg = (sep == std::string::npos) ? body.substr(start) : body.substr(start, sep - start);
        size_t a = seg.find_first_not_of(" \t"), b = seg.find_last_not_of(" \t");
        if (a != std::string::npos) {
            fprintf(f, "%s\n", seg.substr(a, b - a + 1).c_str());
            ++lines;
        }
        if (sep == std::string::npos) break;
        start = sep + 1;
    }
    if (fsync(fileno(f)) != 0) {
        fclose(f);
        CMD_FAIL_RETURN("script-set: fsync failed for %s", path.c_str());
    }
    fclose(f);
    UART_LOG("script-set: %s '%s' — %d lines", exists ? "overwritten" : "created", path.c_str(), lines);
}
#endif // CONFIG_FUGU_WITH_SCRIPTS

#if CONFIG_IDF_TARGET_ESP32S3
// adc-cal: what IDF's ADC1 calibration is built from on this chip. Read-only, no reset.
// eFuse, per attenuation: the hardware offset code (loaded by the driver) and one factory point
// (raw code at 850 mV) that sets IDF's first-step gain. Then, per attenuation, IDF's full
// conversion (first-step line + the fixed family-wide curve) at every 256th code.
static void cmdAdcCal(cmd *) {
    const int ver = esp_efuse_rtc_calib_get_ver();
    if (ver < ESP_EFUSE_ADC_CALIB_VER_MIN || ver > ESP_EFUSE_ADC_CALIB_VER_MAX)
        CMD_FAIL_RETURN("adc-cal: eFuse calibration version %d not supported", ver);
    UART_LOG("ADCCAL efuse_ver=%d", ver);
    uint32_t calCode[4] = {}, calMv[4] = {};
    for (int at = 0; at < 4; ++at) {
        if (esp_efuse_rtc_calib_get_cal_voltage(ver, ADC_UNIT_1, at, &calCode[at], &calMv[at]) != ESP_OK
            || !calCode[at])
            CMD_FAIL_RETURN("adc-cal: no cal point for atten %d", at);
        UART_LOG("ADCCAL adc1 atten=%d init_code=%lu cal_mv=%lu cal_code=%lu gain_uv_per_code=%.2f", at,
                 (unsigned long) esp_efuse_rtc_calib_get_init_code(ver, ADC_UNIT_1, at),
                 (unsigned long) calMv[at], (unsigned long) calCode[at], 1e3 * calMv[at] / calCode[at]);
    }
    for (int at = 0; at < 4; ++at) {
        adc_cali_handle_t cal = nullptr;
        adc_cali_curve_fitting_config_t conf{.unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_0,
                                             .atten = (adc_atten_t) at, .bitwidth = ADC_BITWIDTH_12};
        if (adc_cali_create_scheme_curve_fitting(&conf, &cal) != ESP_OK)
            CMD_FAIL_RETURN("adc-cal: curve-fitting scheme unavailable for atten %d", at);
        UART_LOG("ADCCAL table atten=%d code mv_idf mv_first_step curve_mv", at);
        for (int code = 0; code <= 4096; code += 256) {
            const int c = std::min(code, 4095);
            int mv = 0;
            adc_cali_raw_to_voltage(cal, c, &mv);
            const float line = (float) c * calMv[at] / calCode[at];
            UART_LOG("ADCCAL %d %4d %5d %8.1f %6.1f", at, c, mv, line, mv - line);
        }
        adc_cali_delete_scheme_curve_fitting(cal);
    }
}
#endif

static void cmdHelp(cmd *) { UART_LOG("%s", cli.toString().c_str()); }

// Commands that change the PWM, rectification, backflow switch or sensors are refused while `selftest bflow` runs.
// Checked where SimpleCLI dispatches each parsed command, so `a;;b` lines are covered too.
template<void (*Handler)(cmd *)>
static void bfGuarded(cmd *c) {
    if (bfTestBusy())
        CMD_FAIL_RETURN("%s: refused, selftest bflow running (selftest abort)", Command(c).getName().c_str());
    Handler(c);
}

void setupCli() {
    cli.addCommand("help,?", cmdHelp);
    cli.addCommand("restart,reset,reboot", cmdRestart);
    cli.addCommand("mppt", cmdMppt);
    cli.addCommand("short-ls", bfGuarded<cmdShortLs>);
    cli.addBoundlessCmd("sweep", bfGuarded<cmdSweep>); // sweep [calfail]
    cli.addCommand("sweep-curve", cmdSweepCurve);
    cli.addBoundlessCmd("rect-cal", bfGuarded<cmdRectCal>); // rect-cal [force|status|abort]
    cli.addCommand("intr", cmdIntr);
    cli.addBoundlessCmd("reset-lag", cmdResetLag); // reset-lag [q]: q skips the rt-stats print
#if CONFIG_FUGU_WITH_RTPROF
    cli.addCommand("rtlog-test", [](cmd *) { g_rtLogTest = 4; }); // formatted and raw-captured RT log lines
    cli.addBoundlessCmd("rtprof", [](cmd *c) { // rtprof [clear|all|flash]
        const String a = Command(c).countArgs() ? Command(c).getArg(0).getValue() : String();
        if (a == "clear" || a == "all" || a == "flash") rtprofClear(a == "all", a == "flash");
        else rtprofDump();
    });
#endif
    cli.addCommand("scan-i2c", cmdScanI2c);
    cli.addBoundlessCmd("ls", cmdLs);   // ls [path] (littlefs)
    cli.addBoundlessCmd("cat", cmdCat); // cat <file> (littlefs)
    cli.addCommand("rt-stats", cmdRtStats);
    cli.addCommand("tasks", cmdTasks);
    cli.addCommand("bootinfo", cmdBootinfo);
#if CONFIG_IDF_TARGET_ESP32S3
    cli.addCommand("adc-cal", bfGuarded<cmdAdcCal>);
#endif
    cli.addBoundlessCmd("heap", cmdHeap);    // heap [check|tasks]
    cli.addBoundlessCmd("log", cmdLogLevel); // log <tag> <level>
    cli.addCommand("mem", cmdMem);
    cli.addBoundlessCmd("peek", cmdPeek); // peek <hex-addr> [len]; symbol resolution lives host-side
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    cli.addBoundlessCmd("coredump", cmdCoredump); // coredump [info|get|erase]; get streams base64
    cli.addBoundlessCmd("crash", cmdCrash); // crash <null|abort|stack>: deliberate panic, writes coredump
#endif
    cli.addCommand("uptime", cmdUptime);
    cli.addBoundlessCmd("wsync", bfGuarded<cmdWsync>); // `wsync` edge rate; `wsync arm [off]` one-shot USB-pad probe
    cli.addBoundlessCmd("sensor", cmdSensor); // `sensor` full dump; `sensor avg` compact EWM line
#ifdef CONFIG_FUGU_WITH_NETW
    cli.addCommand("ip", cmdIp);
#endif
    cli.addCommand("adc-restart", bfGuarded<cmdAdcRestart>);
    cli.addCommand("adc-reset", bfGuarded<cmdAdcReset>);
    cli.addCommand("anf", bfGuarded<cmdAnf>);

    cli.addBoundlessCmd("dc", bfGuarded<cmdDc>); // dc <hs> [ls]
    cli.addBoundlessCmd("dt,deadtime", bfGuarded<cmdDeadTime>); // dt [hl_ns [lh_ns]]
    cli.addBoundlessCmd("pwm-freq,fsw", bfGuarded<cmdPwmFreq>); // pwm-freq [hz]
    cli.addSingleArgCmd("sync", bfGuarded<cmdSync>);
    cli.addSingleArgCmd("bf,panel", bfGuarded<cmdBflow>);
    cli.addSingleArgCmd("speed", bfGuarded<cmdSpeed>);
    cli.addSingleArgCmd("fan", cmdFan);
    cli.addSingleArgCmd("led", cmdLed);
    cli.addBoundlessCmd("gpio", bfGuarded<cmdGpio>);

#if WITH_PWM_DIAGNOSTICS
    cli.addSingleArgCmd("mcpwmtest", bfGuarded<cmdMcpwmTest>);
    cli.addSingleArgCmd("gpiodump", [](cmd *c) {
        Command cc(c);
        int pin;
        if (!argInt(cc, 0, pin) || pin < 0 || pin >= SOC_GPIO_PIN_COUNT || !GPIO_IS_VALID_GPIO(pin))
            CMD_FAIL_RETURN("gpiodump <pin>");
        // GPIO_ENABLE_REG (or _ENABLE1 for pin>=32) bit, GPIO_OUT_SEL signal, IO_MUX function
        uint32_t en = (pin < 32) ? REG_READ(GPIO_ENABLE_REG) : REG_READ(GPIO_ENABLE1_REG);
        uint32_t out_sel = REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + (pin * 4));
        uint32_t io_mux = REG_READ(IO_MUX_GPIO0_REG + (pin * 4));
        uint32_t enable_bit = (en >> (pin & 31)) & 1;
        uint32_t signal_idx = out_sel & 0x1FF;
        uint32_t oen_sel = (out_sel >> 10) & 1;
        UART_LOG("gpio %d: enable=%u sig=%u oen_sel=%u io_mux=0x%lx",
                 pin, (unsigned) enable_bit, (unsigned) signal_idx, (unsigned) oen_sel, (unsigned long) io_mux);
    });
    cli.addSingleArgCmd("mcpwmdump", [](cmd *c) {
        Command cc(c);
        int grp;
        if (!argInt(cc, 0, grp) || grp < 0 || grp > 1) // ESP32/-S3: 2 MCPWM groups
            CMD_FAIL_RETURN("mcpwmdump <group 0|1>");
        uint32_t cfg0 = REG_READ(MCPWM_TIMER0_CFG0_REG(grp));
        uint32_t cfg1 = REG_READ(MCPWM_TIMER0_CFG1_REG(grp));
        uint32_t status = REG_READ(MCPWM_TIMER0_STATUS_REG(grp));
        UART_LOG("mcpwm grp%d T0: cfg0=0x%lx cfg1=0x%lx status=0x%lx (count=%lu dir=%lu)",
                 grp, (unsigned long) cfg0, (unsigned long) cfg1, (unsigned long) status,
                 (unsigned long) (status & 0xFFFF), (unsigned long) ((status >> 16) & 1));
    });
    cli.addBoundlessCmd("pwm-dump", [](cmd *) {
        uint32_t freq = converter.getPwmFrequency();
        uint16_t pwmMax = converter.pwmMaxDriver();
        uint16_t pwmCtrl = converter.getCtrlOnPwmCnt();
        uint16_t pwmRect = converter.getRectOnPwmCnt();
        uint16_t dtHlTicks = converter.getDtHlTicks();
        uint16_t hs_off = pwmCtrl;
        // enLogic: LS rises at TEZ (tick 0); HiLi: LS rises at cmpHS_. DT module delays the posedge.
        uint16_t ls_on_base = converter.isEnLogic() ? 0 : pwmCtrl;
        uint16_t ls_on = (pwmRect == 0) ? 0 : (uint16_t) (ls_on_base + dtHlTicks);
        uint16_t ls_off = (pwmRect == 0) ? 0 : (uint16_t) (pwmCtrl + pwmRect);
        // realized, not requested: bestTiming() rounds the period to whole ticks (39000 asked,
        // 38995.86 delivered at 4103 ct) -- period_ticks is the source of truth
        UART_LOG("freq=%.2f nominal=%u period_ticks=%u pwmMax=%u hs_off=%u ls_on=%u ls_off=%u fault=0 brake=0",
                 converter.realizedFreqHz(), (unsigned) freq,
                 (unsigned) converter.getPeriodTicks(), (unsigned) pwmMax,
                 (unsigned) hs_off, (unsigned) ls_on, (unsigned) ls_off);
    });
#if CONFIG_FUGU_WITH_LEDC
    cli.addBoundlessCmd("anaw", [](cmd *c) {
        if (bfTestBusy()) CMD_FAIL_RETURN("anaw: refused, selftest bflow running (selftest abort)");
        Command cc(c);
        int pin, val;
        if (!argInt(cc, 0, pin) || !argInt(cc, 1, val) || val < 0 || val > 255)
            CMD_FAIL_RETURN("anaw <pin> <0..255>");
        if (!freeOutputPin(pin))
            CMD_FAIL_RETURN("anaw: pin %d invalid or assigned in board.conf", pin);
        // analogWrite claims LEDC channel/timer 0, which the LEDC gate driver uses without Arduino knowing
#if HAVE_MCPWM
        if (!converter.mcpwmLeg())
#endif
            CMD_FAIL_RETURN("anaw: refused while the LEDC gate driver is active");
        analogWrite(pin, val);
        UART_LOG("anaw %d -> %d (Arduino LEDC)", pin, val);
    });
#endif
#endif
#ifdef CONFIG_FUGU_WITH_NETW
    cli.addBoundlessCmd("wifi", cmdWifi); // wifi on | off [minutes]
    cli.addSingleArgCmd("wifi-add", cmdWifiAdd);
    cli.addSingleArgCmd("ota", bfGuarded<cmdOta>);
#endif
#ifdef CONFIG_FUGU_WITH_NETTOOLS
    cli.addBoundlessCmd("curl", cmdCurl);   // curl [-X M] [-H k:v] [-d data] <url>
    cli.addBoundlessCmd("ping", cmdPing);   // ping <host> [count]
    cli.addSingleArgCmd("nslookup,resolve", cmdNslookup);
    cli.addBoundlessCmd("tcpconnect,probe", cmdTcpConnect); // tcpconnect <host> <port>
    cli.addCommand("netstat,ifconfig", cmdNetstat);
#endif
    cli.addSingleArgCmd("vset", bfGuarded<cmdVset>);
    cli.addSingleArgCmd("iset", cmdIset);
    cli.addSingleArgCmd("ovset", bfGuarded<cmdOvset>);
#if CONFIG_FUGU_WITH_PSU
    cli.addBoundlessCmd("psu", cmdPsu);
    cli.addBoundlessCmd("pv", cmdPv); // pv <isc> <voc> [k] | scale <s> | shade <g,..> | off
#endif
    cli.addCommand("status", cmdStatus);
    cli.addBoundlessCmd("holiday", cmdHoliday); // holiday [auto|on|off]
    cli.addBoundlessCmd("balance", cmdBalance); // balance [now|skip]
    cli.addCommand("wire", cmdWire);
    cli.addCommand("wire-reset", cmdWireReset);

    cli.addBoundlessCmd("hostname,hn", cmdHostname);
    cli.addBoundlessCmd("set-config,setc", cmdSetConfig);
    cli.addBoundlessCmd("del-config,delc", cmdDelConfig);
    cli.addBoundlessCmd("get-config,getc", cmdGetConfig);
    cli.addCommand("conf-hash,confhash", cmdConfHash);
    cli.addBoundlessCmd("conf-check,confcheck", cmdConfCheck);
    cli.addBoundlessCmd("service,svc", cmdService);
#ifdef CONFIG_FUGU_WITH_SCRIPTS
    cli.addSingleArgCmd("sleep", bfGuarded<cmdSleep>);
    cli.addSingleArgCmd("run", cmdRun);
    cli.addCommand("scripts", cmdScripts);
    cli.addBoundlessCmd("script-set,script", cmdScriptSet);
#endif
#ifdef CONFIG_FUGU_WITH_BLE
    cli.addBoundlessCmd("ota-ble", bfGuarded<cmdOtaBle>);
    cli.addBoundlessCmd("ble", cmdBle);
#endif
#if defined(CONFIG_FUGU_WITH_NETW) || defined(CONFIG_FUGU_WITH_BLE_TELE)
    cli.addSingleArgCmd("set-time", cmdSetTime);
#endif
#ifdef CONFIG_FUGU_WITH_BLE_TELE
    cli.addSingleArgCmd("tele-ble", cmdTeleBle);
#endif
#ifdef CONFIG_FUGU_WITH_TELE_MQTT
    cli.addSingleArgCmd("tele-mqtt", cmdTeleMqtt);
#endif
    cli.addBoundlessCmd("selftest", cmdSelftest); // selftest [bflow|abort|clear]
#ifdef CONFIG_FUGU_WITH_MEASURE_COIL
    cli.addBoundlessCmd("measure-coil", bfGuarded<cmdMeasureCoil>); // measure-coil l0|ls [steps|hs] [dwell_ms] [f=<Hz>] [sync] [apply]
#endif
#if CONFIG_FUGU_WITH_VCONV
    cli.addBoundlessCmd("vconv", bfGuarded<cmdVconv>); // vconv [pv|bat|set ...]
#endif
#if CONFIG_FUGU_WITH_PWR_KNOBS
    cli.addBoundlessCmd("pwr", [](cmd *c) {
        if (bfTestBusy()) CMD_FAIL_RETURN("pwr: refused, selftest bflow running (selftest abort)");
        Command cc(c);
        String s[5];
        const char *a[5];
        const int n = std::min(cc.countArgs(), 5);
        for (int i = 0; i < n; ++i) {
            s[i] = cc.getArg(i).getValue();
            a[i] = s[i].c_str();
        }
        if (!pwrCommand(n, a)) s_cmdFailed = true;
    });
#endif
}

// Parses without running (paused SimpleCLI queues the commands) and checks each command against kRoCmds.
static bool roLine(const String &inp) {
    cli.pause();
    cli.parse(inp);
    bool ok = cli.countCmdQueue() < 10; // the queue drops its oldest entry beyond that
    while (cli.available()) {
        Command c = cli.getCmd();
        const int n = c.countArgs();
        if (!roCmdAllowed(kRoCmds, std::size(kRoCmds), c.getName().c_str(), n, n ? c.getArg(0).getValue().c_str() : ""))
            ok = false;
    }
    while (cli.errored()) cli.getError();
    cli.unpause();
    return ok;
}

bool handleCommand(const String &inp, const char *ro) {
    const int m = secretCut(inp.c_str());
    ESP_LOGI("main", "received serial command: '%.*s%s'", CG_MASKED(inp.c_str(), m));

    if (ro && (inp[0] == '+' || inp[0] == '-' || !roLine(inp))) {
        ESP_LOGW("main", "refused: read-only console (%s)", ro);
        return false;
    }

    // +N / -N PWM step is a signed numeric token, not a named command -> handle before SimpleCLI.
    // does not enable manual pwm!
    if ((inp[0] == '+' or inp[0] == '-') && !adcSampler.isCalibrating() && inp.length() < 6 &&
        inp.toInt() != 0 && std::abs(inp.toInt()) < converter.pwmCtrlMax) {
        if (bfTestBusy()) {
            ESP_LOGW("main", "+-N: selftest bflow running");
            return false;
        }
#ifdef CONFIG_FUGU_WITH_MEASURE_COIL
        if (isMeasuring()) {
            ESP_LOGW("main", "+-N: refused, measure-coil running");
            return false;
        }
#endif
        int pwmStep = inp.toInt();
        int target = (int) converter.getCtrlOnPwmCnt() + pwmStep;
        if (target < 0) target = 0;
        if (target > converter.pwmCtrlMax) target = converter.pwmCtrlMax;
        if (g_app.manualPwm())
            mppt.setManualTarget((uint16_t) target);
        else if (g_app.psuMode()) {
            ESP_LOGW("main", "+-N: not in PSU mode (use 'psu off' first)");
            s_cmdFailed = true;
            return false;
        }
        else
            mppt.setAutoRampTarget((uint16_t) target);
        ESP_LOGI("main", "PWM step %i -> target %i", pwmStep, target);
        loopLF(wallClockUs(), true);
        return true;
    }

    s_cmdFailed = false; // a handler may set this via CMD_FAIL() to report ERR despite a clean parse
    try {
        cli.parse(inp); // runs the matched command callback inline
    } catch (const std::exception &e) {
        // a throwing command must not take down the console/network task
        ESP_LOGE("main", "command '%.*s%s' failed: %s", CG_MASKED(inp.c_str(), m), e.what());
        return false;
    } catch (...) {
        ESP_LOGE("main", "command '%.*s%s' failed: unknown exception", CG_MASKED(inp.c_str(), m));
        return false;
    }
    if (cli.errored()) {
        ESP_LOGE("main", "%s", cli.getError().toString().c_str()); // unknown command / parse error
        return false;
    }
    if (s_cmdFailed) return false; // handler rejected its input (CMD_FAIL)

    loopLF(wallClockUs(), true); // interim status snapshot; must not disturb the sps window
    return true;
}
