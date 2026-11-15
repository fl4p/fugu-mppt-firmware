#include "pwr_knobs.h"

#include <Arduino.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <esp_pm.h>

#include "logging.h"
#include "mppt.h"
#include "adc/mock.h"
#include "app_state.h"
#ifdef WITH_NETW
#include <WiFi.h>
#include <esp_wifi.h>
#endif
#ifdef WITH_BLE
#include "tele/console_ble.h"
#endif
#include "idle_profile.h"

extern MpptController mppt;
extern SynchronousConverter converter;
extern VIinVout<const Sensor *> sensors;
ADC_Fake *g_pwrFakeAdc = nullptr;
#ifdef CONFIG_PM_ENABLE
extern esp_pm_lock_handle_t s_cli_pm_lock;
static bool s_lockReleased = false;
#endif
static bool argNum(const char *s, long &out) {
    char *end;
    out = strtol(s, &end, 10);
    return end != s && !*end;
}

static void pwrStatus() {
    char w[48] = "wifi=off ps=- li=- wtx=-";
#ifdef WITH_NETW
    wifi_ps_type_t ps;
    wifi_config_t wc = {};
    int8_t tx = 0;
    if (esp_wifi_get_ps(&ps) == ESP_OK && esp_wifi_get_config(WIFI_IF_STA, &wc) == ESP_OK
        && esp_wifi_get_max_tx_power(&tx) == ESP_OK)
        snprintf(w, sizeof w, "wifi=%s ps=%s li=%u wtx=%.2f", WiFi.isConnected() ? "up" : "down",
                 ps == WIFI_PS_NONE ? "none" : ps == WIFI_PS_MIN_MODEM ? "min" : "max",
                 (unsigned) wc.sta.listen_interval, tx * .25f);
#endif
    char adv[8] = "-";
#ifdef WITH_BLE
    snprintf(adv, sizeof adv, "%u", (unsigned) bleConsoleAdvMs());
#endif
    char pm[24] = "-/-/- lock=-";
#ifdef CONFIG_PM_ENABLE
    esp_pm_config_t pc = {};
    esp_pm_get_configuration(&pc);
    snprintf(pm, sizeof pm, "%d/%d/%d lock=%d", pc.max_freq_mhz, pc.min_freq_mhz, (int) pc.light_sleep_enable,
             (int) (s_cli_pm_lock && !s_lockReleased));
#endif
#if CONFIG_IDF_TARGET_ESP32S3
    const float t = mppt.ucTemp.readRaw();
#else
    const float t = mppt.ucTemp.last();
#endif
    char pwm = '-';
#if HAVE_MCPWM
    if (auto *leg = converter.mcpwmLeg()) pwm = leg->timerOff() ? '0' : '1';
#endif
    auto *vo = (const PhysicalSensor *) sensors.Vout;
    const float rt = vo && !vo->isVirtual ? vo->adc->getSamplingRate(vo->params.adcCh) : NAN;
    UART_LOG("pwr: cpu=%lu apb=%lu %s adv=%s tele=%lu idle=%d pm=%s pwm=%c rt=%.0f T=%.2f", (unsigned long) getCpuFrequencyMhz(),
             (unsigned long) getApbFrequency(), w, adv, (unsigned long) (g_teleMinPointUs / 1000), (int) idleProfileOn(), pm,
             pwm, rt, t);
}

#ifdef CONFIG_PM_ENABLE
static bool pmSet(int maxMhz, int minMhz, bool ls) {
    esp_pm_config_t c = {};
    c.max_freq_mhz = maxMhz;
    c.min_freq_mhz = minMhz;
    c.light_sleep_enable = ls;
    esp_err_t e = esp_pm_configure(&c);
    if (e != ESP_OK) UART_LOG("pwr: esp_pm_configure: %s", esp_err_to_name(e));
    return e == ESP_OK;
}
#endif

#if HAVE_MCPWM
static int pwmSet(bool on) {
    auto *leg = converter.mcpwmLeg();
    if (!leg) { UART_LOG("pwr: pwm not supported"); return 1; }
    if (on != leg->timerOff()) return 0;
    if (converter.startInhibited()) { UART_LOG("pwr: pwm refused, converter start inhibited (rect-cal)"); return 1; }
    if (!on && converter.bsyncOwnsPeriod) { UART_LOG("pwr: pwm 0 refused while bsync runs"); return 1; }
    const uint16_t en = converter.enableCount();
    converter.inhibitStart(true);
    vTaskDelay(pdMS_TO_TICKS(10)); // a start that passed the inhibit check has finished
    const bool idle = converter.disabled() && converter.enableCount() == en;
    const esp_err_t e = !idle ? ESP_OK : on ? leg->timerResume() : leg->timerSuspend();
    converter.inhibitStart(false);
    if (!idle) UART_LOG("pwr: pwm refused, converter not disabled");
    else if (e != ESP_OK) UART_LOG("pwr: pwm: %s", esp_err_to_name(e));
    return idle && e == ESP_OK ? 0 : 1;
}
#endif

static int rtSet(long hz) {
    auto *vo = (const PhysicalSensor *) sensors.Vout;
    if (!g_pwrFakeAdc || !vo || vo->isVirtual || vo->adc != (AsyncADC<float> *) g_pwrFakeAdc) {
        UART_LOG("pwr: rt not supported (adc is not fake)");
        return 1;
    }
    // 10 Hz: 2x margin on loopRT's 200 ms stale-sample window (ADC stall reset, restart after 60 s)
    const long lo = std::max<long>(10, g_app.loopRateMin);
    if (hz < lo || hz > 3000) {
        UART_LOG("pwr: rt %ld refused, range %ld..3000 Hz (floor: %s)", hz, lo,
                 lo > 10 ? "sensor.conf expected_hz, loop latency watchdog" : "ADC stall window 200 ms");
        return 1;
    }
    const esp_err_t e = g_pwrFakeAdc->setRate(hz);
    if (e == ESP_OK) return 0;
    UART_LOG("pwr: rt: %s", esp_err_to_name(e));
    return 1;
}

#ifdef CONFIG_PM_ENABLE
static int pmLocks() {
    char *buf = nullptr;
    size_t len = 0;
    FILE *f = open_memstream(&buf, &len);
    if (!f) { UART_LOG("pwr: locks: no memory"); return 1; }
    esp_pm_dump_locks(f);
    fclose(f);
    for (char *l = buf, *n; l && *l; l = n) {
        n = strchr(l, '\n');
        if (n) *n++ = 0;
        if (*l) UART_LOG("%s", l);
    }
    free(buf);
    return 0;
}
#endif

// 0 ok, 1 failed (reported), -1 usage
static int pwrSet(int argc, const char *const *argv) {
    const char *k = argv[0];
    long v = 0;
    if (!strcmp(k, "cpu")) {
        if (argc != 2 || !argNum(argv[1], v) || (v != 80 && v != 160 && v != 240)) return -1;
        idleProfileSuspend();
#ifdef CONFIG_PM_ENABLE
        return pmSet(v, v, false) ? 0 : 1;
#else
        return setCpuFrequencyMhz(v) ? 0 : 1;
#endif
    }
#ifdef CONFIG_PM_ENABLE
    if (!strcmp(k, "pm")) {
        if (argc == 2 && !strcmp(argv[1], "off")) {
            if (s_lockReleased && s_cli_pm_lock && esp_pm_lock_acquire(s_cli_pm_lock) == ESP_OK)
                s_lockReleased = false;
            return s_lockReleased;
        }
        long mn, ls;
        if (argc != 4 || !argNum(argv[1], v) || !argNum(argv[2], mn) || !argNum(argv[3], ls) || ls < 0 || ls > 1)
            return -1;
        idleProfileSuspend();
        if (!pmSet(v, mn, ls)) return 1;
        if (!s_lockReleased && s_cli_pm_lock && esp_pm_lock_release(s_cli_pm_lock) == ESP_OK)
            s_lockReleased = true;
        return 0;
    }
#endif
    if (!strcmp(k, "pwm")) {
        if (argc != 2 || !argNum(argv[1], v) || v < 0 || v > 1) return -1;
#if HAVE_MCPWM
        return pwmSet(v);
#else
        UART_LOG("pwr: pwm not supported");
        return 1;
#endif
    }
    if (!strcmp(k, "rt")) {
        if (argc != 2 || !argNum(argv[1], v)) return -1;
        return rtSet(v);
    }
#ifdef CONFIG_PM_ENABLE
    if (!strcmp(k, "locks")) return argc == 1 ? pmLocks() : -1;
#endif
    if (!strcmp(k, "tele")) {
        if (argc != 2 || !argNum(argv[1], v) || v < 20 || v > 60000) return -1;
        idleProfileSuspend();
        g_teleMinPointUs = v * 1000;
        return 0;
    }
#ifdef WITH_NETW
    if (!strcmp(k, "wifi-ps")) {
        if (argc < 2 || argc > 3) return -1;
        const char *m = argv[1];
        wifi_ps_type_t ps = !strcmp(m, "none") ? WIFI_PS_NONE : !strcmp(m, "min") ? WIFI_PS_MIN_MODEM
                          : !strcmp(m, "max") ? WIFI_PS_MAX_MODEM : (wifi_ps_type_t) -1;
        if (ps == (wifi_ps_type_t) -1 || (argc == 3 && (!argNum(argv[2], v) || v < 0 || v > 1000))) return -1;
        esp_err_t e = ESP_OK;
        if (argc == 3) {
            wifi_config_t wc = {};
            e = esp_wifi_get_config(WIFI_IF_STA, &wc);
            if (e == ESP_OK) {
                wc.sta.listen_interval = v;
                esp_wifi_set_storage(WIFI_STORAGE_RAM); // no NVS write (flash stalls the RT loop)
                e = esp_wifi_set_config(WIFI_IF_STA, &wc);
                esp_wifi_set_storage(WIFI_STORAGE_FLASH);
            }
            UART_LOG("pwr: listen_interval=%ld %s, takes effect after reconnect (a fresh connect resets it)",
                     v, esp_err_to_name(e));
        }
        idleProfileSuspend();
        if (WiFi.setSleep(ps) && e == ESP_OK) return 0;
        UART_LOG("pwr: wifi-ps failed");
        return 1;
    }
    if (!strcmp(k, "wifi-tx")) {
        float dbm;
        char *end;
        if (argc != 2 || (dbm = strtof(argv[1], &end), end == argv[1] || *end)) return -1;
        long q = lroundf(dbm * 4);
        if (q < 8 || q > 84) return -1;
        esp_err_t e = esp_wifi_set_max_tx_power(q);
        if (e == ESP_OK) return 0;
        UART_LOG("pwr: esp_wifi_set_max_tx_power: %s", esp_err_to_name(e));
        return 1;
    }
#endif
#ifdef WITH_BLE
    if (!strcmp(k, "ble-adv")) {
        if (argc != 2 || !argNum(argv[1], v) || (v && (v < 20 || v > 10240))) return -1;
        if (!*bleConsoleName()) { UART_LOG("pwr: BLE not started"); return 1; }
        if (bleConsoleConnected()) { UART_LOG("pwr: refused while a BLE client is connected"); return 1; }
        idleProfileSuspend();
        if (bleConsoleSetAdvMs(v)) return 0;
        UART_LOG("pwr: ble-adv refused");
        return 1;
    }
#endif
    return -1;
}

bool pwrCommand(int argc, const char *const *argv) {
    const int rc = argc ? pwrSet(argc, argv) : 0;
    if (rc < 0) {
        UART_LOG("usage: pwr [cpu 80|160|240 | pm <max> <min> <ls 0|1> | pm off | wifi-ps none|min|max [li]"
                 " | wifi-tx <dBm 2..21> | ble-adv <ms 20..10240, 0=default> | tele <ms 20..60000> | pwm 0|1 | rt <hz>"
#ifdef CONFIG_PM_ENABLE
                 " | locks"
#endif
                 "]");
        return false;
    }
    pwrStatus();
    return rc == 0;
}
