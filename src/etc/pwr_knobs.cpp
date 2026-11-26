#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

#include "pwr_knobs.h"

#include <Arduino.h>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <esp_pm.h>
#include <esp_timer.h>

#include "logging.h"
#include "mppt.h"
#include "adc/mock.h"
#include "app_state.h"
#ifdef CONFIG_FUGU_WITH_NETW
#include <WiFi.h>
#include <esp_wifi.h>
#endif
#ifdef CONFIG_FUGU_WITH_BLE
#include "tele/console_ble.h"
#endif
#include "idle_profile.h"

extern MpptController mppt;
extern SynchronousConverter converter;
extern VIinVout<const Sensor *> sensors;
ADC_Fake *g_pwrFakeAdc = nullptr;
uint16_t g_pwrNetLoopMs = 1;
#ifdef CONFIG_PM_ENABLE
extern esp_pm_lock_handle_t s_cli_pm_lock;
static bool s_lockReleased = false;

// `pwr pm ... <secs>` window. The esp_timer alarm also bounds every automatic light sleep
// (vApplicationSleep sleeps at most until esp_timer_get_next_alarm_for_wake_up()).
enum : uint8_t { WIN_IDLE, WIN_RUN, WIN_DUE, WIN_TIMER_REVERTED };
static struct {
    esp_timer_handle_t timer;
    esp_pm_config_t prev, cfg;
    bool prevLockReleased;
    std::atomic<uint8_t> st;
    uint16_t secs, n;
    int64_t t0Us, lastUs;
    float sum, mn, mx, ring[12]; // ring: last 60 s at 5 s
} s_win;
static char s_winLine[160];
#endif
static bool argNum(const char *s, long &out) {
    char *end;
    out = strtol(s, &end, 10);
    return end != s && !*end;
}

static void pwrStatus() {
    char w[48] = "wifi=off ps=- li=- wtx=-";
#ifdef CONFIG_FUGU_WITH_NETW
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
#ifdef CONFIG_FUGU_WITH_BLE
    snprintf(adv, sizeof adv, "%u", (unsigned) bleConsoleAdvMs());
#endif
    char pm[40] = "-/-/- lock=-";
#ifdef CONFIG_PM_ENABLE
    esp_pm_config_t pc = {};
    esp_pm_get_configuration(&pc);
    const long left = s_win.st == WIN_RUN ? s_win.secs - (long) ((esp_timer_get_time() - s_win.t0Us) / 1000000) : -1;
    snprintf(pm, sizeof pm, "%d/%d/%d lock=%d win=%ld", pc.max_freq_mhz, pc.min_freq_mhz, (int) pc.light_sleep_enable,
             (int) (s_cli_pm_lock && !s_lockReleased), left);
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
    UART_LOG("pwr: cpu=%lu apb=%lu %s adv=%s tele=%lu idle=%d/%s/%lus icpu=%s pm=%s pwm=%c rt=%.0f rtsrc=%s vin=%.2f nl=%u T=%.2f",
             (unsigned long) getCpuFrequencyMhz(), (unsigned long) getApbFrequency(), w, adv,
             (unsigned long) (g_teleMinPointUs / 1000), (int) idleProfileOn(), idleProfileMode(),
             (unsigned long) idleProfileAfter(), idleCpuState(), pm, pwm, rt,
             !g_pwrFakeAdc ? "-" : g_pwrFakeAdc->espSource() ? "esp" : "gp", g_pwrFakeAdc ? g_pwrFakeAdc->getScale() : NAN,
             (unsigned) g_pwrNetLoopMs, t);
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
    if (converter.startInhibited(SynchronousConverter::InhibitCal)) {
        UART_LOG("pwr: pwm refused, converter start inhibited (rect-cal)");
        return 1;
    }
    if (!on && converter.bsyncOwnsPeriod) { UART_LOG("pwr: pwm 0 refused while bsync runs"); return 1; }
    // the LS clamp, its ramp and its pending start drive the LS on a disabled converter
    const auto hsBusy = [] { const uint8_t s = mppt.hsFaultState(); return converter.lsClamped() || (s && s != MpptController::HsOff); };
    if (!on && hsBusy()) { UART_LOG("pwr: pwm 0 refused while the LS clamp is held or pending (hs-short)"); return 1; }
    const uint16_t en = converter.enableCount();
    converter.inhibitStart(true);
    vTaskDelay(pdMS_TO_TICKS(10)); // a start that passed the inhibit check has finished
    const bool idle = converter.disabled() && converter.enableCount() == en && (on || !hsBusy());
    const esp_err_t e = !idle ? ESP_OK : on ? leg->timerResume() : leg->timerSuspend();
    converter.inhibitStart(false);
    if (!idle) UART_LOG("pwr: pwm refused, converter not disabled or LS clamp pending");
    else if (e != ESP_OK) UART_LOG("pwr: pwm: %s", esp_err_to_name(e));
    return idle && e == ESP_OK ? 0 : 1;
}
#endif

static bool onFakeAdc(const Sensor *s) {
    auto *p = (const PhysicalSensor *) s;
    return g_pwrFakeAdc && p && !p->isVirtual && p->adc == (AsyncADC<float> *) g_pwrFakeAdc;
}

static int rtSet(long hz) {
    if (!onFakeAdc(sensors.Vout)) {
        UART_LOG("pwr: rt not supported (adc is not fake)");
        return 1;
    }
    // 10 Hz: 2x margin on loopRT's 200 ms stale-sample window (ADC stall reset, restart after 60 s)
    const long lo = std::max<long>(10, g_app.loopRateMin);
    if (hz < lo || hz > 10000) {
        UART_LOG("pwr: rt %ld refused, range %ld..10000 Hz (floor: %s)", hz, lo,
                 lo > 10 ? "loop latency watchdog, half the boot Vout rate" : "ADC stall window 200 ms");
        return 1;
    }
    const esp_err_t e = g_pwrFakeAdc->setRate(hz);
    if (e == ESP_OK) return 0;
    UART_LOG("pwr: rt: %s", esp_err_to_name(e));
    return 1;
}

#ifdef CONFIG_PM_ENABLE
// lock + mode stats; the SLEEP mode row and sleep counts only appear while light sleep is enabled
static char *locksDump() {
    char *buf = nullptr;
    size_t len = 0;
    FILE *f = open_memstream(&buf, &len);
    if (!f) return nullptr;
    esp_pm_dump_locks(f);
    fclose(f);
    return buf;
}

static int locksPrint(char *buf) {
    if (!buf) { UART_LOG("pwr: locks: no memory"); return 1; }
    for (char *l = buf, *n; *l; l = n) {
        n = strchr(l, '\n');
        if (n) *n++ = 0;
        if (*l) UART_LOG("%s", l);
        if (!n) break;
    }
    free(buf);
    return 0;
}

static void pmRevert() {
    esp_pm_config_t c = s_win.prev;
    c.light_sleep_enable = false;
    pmSet(c.max_freq_mhz, c.min_freq_mhz, false);
    if (!s_win.prevLockReleased && s_lockReleased && s_cli_pm_lock && esp_pm_lock_acquire(s_cli_pm_lock) == ESP_OK)
        s_lockReleased = false;
}

// esp_timer task: flag the end for pwrTick(); 10 s later revert here if the network loop did not
static void winTimerCb(void *) {
    uint8_t e = WIN_RUN;
    if (s_win.st.compare_exchange_strong(e, WIN_DUE)) esp_timer_start_once(s_win.timer, 10000000);
    else if (e == WIN_DUE && s_win.st.compare_exchange_strong(e, WIN_TIMER_REVERTED)) pmRevert();
}

static void winSample(int64_t now) {
    s_win.lastUs = now;
    const float t = mppt.ucTemp.readRaw();
    if (!std::isfinite(t)) return;
    s_win.ring[s_win.n % 12] = t;
    s_win.sum += t;
    s_win.mn = s_win.n ? std::min(s_win.mn, t) : t;
    s_win.mx = s_win.n ? std::max(s_win.mx, t) : t;
    ++s_win.n;
}

static int winStart(long secs) {
    if (!s_win.timer) {
        esp_timer_create_args_t a = {};
        a.callback = winTimerCb;
        a.name = "pwrWin";
        if (esp_timer_create(&a, &s_win.timer) != ESP_OK) return 1;
    }
    s_win.secs = secs;
    s_win.n = 0;
    s_win.sum = 0;
    s_win.t0Us = esp_timer_get_time();
    esp_pm_get_configuration(&s_win.cfg);
    winSample(s_win.t0Us);
    s_win.st = WIN_RUN;
    if (esp_timer_start_once(s_win.timer, secs * 1000000ULL) != ESP_OK) {
        s_win.st = WIN_IDLE;
        pmRevert();
        return 1;
    }
    UART_LOG("pwr: pmwin start secs=%ld cfg=%d/%d/%d", secs, s_win.cfg.max_freq_mhz, s_win.cfg.min_freq_mhz,
             (int) s_win.cfg.light_sleep_enable);
    return locksPrint(locksDump());
}

static void winEnd(uint8_t st) {
    char *dump = st == WIN_DUE ? locksDump() : nullptr;
    uint8_t e = WIN_DUE;
    const bool timerReverted = !(st == WIN_DUE && s_win.st.compare_exchange_strong(e, WIN_IDLE));
    if (!timerReverted) pmRevert();
    esp_timer_stop(s_win.timer);
    if (!dump) dump = locksDump();
    const int64_t now = esp_timer_get_time();
    const uint16_t k = std::min<uint16_t>(s_win.n, 12);
    float last = 0;
    for (uint16_t i = 0; i < k; ++i) last += s_win.ring[i];
    snprintf(s_winLine, sizeof s_winLine,
             "pwr: pmwin secs=%u dt=%.1f n=%u Tmean=%.2f Tlast60=%.2f Tmin=%.2f Tmax=%.2f cfg=%d/%d/%d rev=%s",
             (unsigned) s_win.secs, (now - s_win.t0Us) * 1e-6f, (unsigned) s_win.n,
             s_win.n ? s_win.sum / s_win.n : NAN, k ? last / k : NAN, s_win.n ? s_win.mn : NAN,
             s_win.n ? s_win.mx : NAN, s_win.cfg.max_freq_mhz, s_win.cfg.min_freq_mhz,
             (int) s_win.cfg.light_sleep_enable, timerReverted ? "timer" : "loop");
    s_win.st = WIN_IDLE;
    UART_LOG("%s", s_winLine);
    locksPrint(dump);
    pwrStatus();
}
#endif

void pwrTick() {
#ifdef CONFIG_PM_ENABLE
    const uint8_t st = s_win.st;
    if (st == WIN_RUN) {
        const int64_t now = esp_timer_get_time();
        if (now - s_win.lastUs >= 5000000) winSample(now);
    } else if (st != WIN_IDLE) winEnd(st);
#endif
}

// max RT loop period (converter on or off) and the Vout sample rate since the last call
static int gapPrint() {
    static int64_t t0 = 0;
    static uint32_t n0 = 0;
    const uint32_t g = g_app.cmdLoopLag;
    g_app.cmdLoopLag = 0;
    const int64_t t = esp_timer_get_time();
    const uint32_t n = sensors.Vout ? sensors.Vout->numSamples : 0;
    const float sps = t0 && (int32_t) (n - n0) >= 0 ? (float) (n - n0) * 1e6f / (float) (t - t0) : NAN;
    UART_LOG("pwr: gap max=%lu us dt=%lu ms sps=%.0f cpu=%lu", (unsigned long) g, (unsigned long) ((t - t0) / 1000),
             sps, (unsigned long) getCpuFrequencyMhz());
    t0 = t;
    n0 = n;
    return 0;
}

// 0 ok, 1 failed (reported), -1 usage
static int pwrSet(int argc, const char *const *argv) {
    const char *k = argv[0];
    long v = 0;
#ifdef CONFIG_PM_ENABLE
    const bool winBusy = s_win.st != WIN_IDLE;
    if (winBusy && (!strcmp(k, "cpu") || (!strcmp(k, "pm") && !(argc == 2 && !strcmp(argv[1], "off"))))) {
        UART_LOG("pwr: pm window running, `pwr pm off` ends it");
        return 1;
    }
#endif
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
            if (winBusy) {
                uint8_t e = WIN_RUN;
                s_win.st.compare_exchange_strong(e, WIN_DUE);
                winEnd(s_win.st);
                return 0;
            }
            if (s_lockReleased && s_cli_pm_lock && esp_pm_lock_acquire(s_cli_pm_lock) == ESP_OK)
                s_lockReleased = false;
            return s_lockReleased;
        }
        long mn, ls, secs = 0;
        if ((argc != 4 && argc != 5) || !argNum(argv[1], v) || !argNum(argv[2], mn) || !argNum(argv[3], ls) || ls < 0
            || ls > 1 || (argc == 5 && (!argNum(argv[4], secs) || secs < 10 || secs > 3600)))
            return -1;
#if !CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION
        if (ls && !secs) { UART_LOG("pwr: light sleep drops USB here, give a window (secs)"); return 1; }
#endif
        idleProfileSuspend();
        esp_pm_get_configuration(&s_win.prev);
        s_win.prevLockReleased = s_lockReleased;
        if (!pmSet(v, mn, ls)) return 1;
        if (!s_lockReleased && s_cli_pm_lock && esp_pm_lock_release(s_cli_pm_lock) == ESP_OK)
            s_lockReleased = true;
        return secs ? winStart(secs) : 0;
    }
    if (!strcmp(k, "pmwin")) {
        if (argc != 1) return -1;
        if (s_win.st == WIN_RUN)
            UART_LOG("pwr: pmwin running secs=%u elapsed=%ld n=%u", (unsigned) s_win.secs,
                     (long) ((esp_timer_get_time() - s_win.t0Us) / 1000000), (unsigned) s_win.n);
        UART_LOG("%s", *s_winLine ? s_winLine : "pwr: pmwin none");
        return 0;
    }
#endif
    if (!strcmp(k, "netloop")) {
        if (argc != 2 || !argNum(argv[1], v) || v < 1 || v > 1000) return -1;
        g_pwrNetLoopMs = v;
        return 0;
    }
    if (!strcmp(k, "rt-src")) {
        if (argc != 2 || (strcmp(argv[1], "gp") && strcmp(argv[1], "esp"))) return -1;
        if (!g_pwrFakeAdc) { UART_LOG("pwr: rt-src not supported (adc is not fake)"); return 1; }
        const esp_err_t e = g_pwrFakeAdc->setEspSource(argv[1][0] == 'e');
        if (e == ESP_OK) return 0;
        UART_LOG("pwr: rt-src: %s", esp_err_to_name(e));
        return 1;
    }
    if (!strcmp(k, "pwm")) {
        if (argc != 2 || !argNum(argv[1], v) || v < 0 || v > 1) return -1;
#if HAVE_MCPWM
        return pwmSet(v);
#else
        UART_LOG("pwr: pwm not supported");
        return 1;
#endif
    }
    if (!strcmp(k, "gap")) return argc == 1 ? gapPrint() : -1;
    if (!strcmp(k, "idle")) {
        if (argc != 2) return -1;
        const char *e = idleProfileForce(argv[1]);
        if (!e) return 0;
        UART_LOG("pwr: idle: %s", e);
        return 1;
    }
    if (!strcmp(k, "rt")) {
        if (argc != 2 || !argNum(argv[1], v)) return -1;
        return rtSet(v);
    }
    if (!strcmp(k, "vin")) {
        char *end;
        float f;
        if (argc != 2 || (f = strtof(argv[1], &end), end == argv[1] || *end) || !(f >= 0 && f <= 2)) return -1;
        if (!onFakeAdc(sensors.Vin)) { UART_LOG("pwr: vin not supported (adc is not fake)"); return 1; }
        g_pwrFakeAdc->setScale(((const PhysicalSensor *) sensors.Vin)->params.adcCh, f);
        return 0;
    }
    if (!strcmp(k, "idle-after")) {
        if (argc != 2 || !argNum(argv[1], v) || v < 5 || v > 3600) return -1;
        idleProfileAfter(v);
        return 0;
    }
#ifdef CONFIG_PM_ENABLE
    if (!strcmp(k, "locks")) return argc == 1 ? locksPrint(locksDump()) : -1;
#endif
    if (!strcmp(k, "tele")) {
        if (argc != 2 || !argNum(argv[1], v) || v < 20 || v > 3600000) return -1;
        idleProfileSuspend();
        g_teleMinPointUs = (uint32_t) v * 1000u;
        return 0;
    }
#ifdef CONFIG_FUGU_WITH_NETW
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
#ifdef CONFIG_FUGU_WITH_BLE
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
        UART_LOG("usage: pwr [cpu 80|160|240 | pm <max> <min> <ls 0|1> [secs 10..3600] | pm off | wifi-ps none|min|max [li]"
                 " | wifi-tx <dBm 2..21> | ble-adv <ms 20..10240, 0=default> | tele <ms 20..3600000> | pwm 0|1 | rt <hz>"
                 " | rt-src gp|esp | netloop <ms 1..1000> | idle 0|1|auto | idle-after <s 5..3600> | vin <0..2> | gap"
#ifdef CONFIG_PM_ENABLE
                 " | locks | pmwin"
#endif
                 "]");
        return false;
    }
    pwrStatus();
    return rc == 0;
}
