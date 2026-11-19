// Host simulator: the real MpptController (mppt.cpp/h, tracker, charger, PD controllers, protect()),
// ADC_Sampler + Sensor filter chain and SynchronousConverter (MOCK driver at MCPWM resolution) closed
// over the VirtualConverter plant (src/sim/vconv.cpp), on a deterministic sim clock.
// The RT/LF glue mirrors src/main.cpp (loopRT, loopRTNewData, stopAndBackoff, lfControl,
// lfStuckWatchdog); see README.md for what is simplified. Build: test/host-stub/sim/build.sh

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <new>
#include <random>
#include <string>

#include "mppt.h"
#include "../../../src/sim/vconv.h" // plant always from this tree, firmware from $SRC
#include "viz/lcd.h"

// ---- sim clock + log -------------------------------------------------------------------------
time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;
static time_us g_simUs = 0;
int64_t esp_timer_get_time() { return (int64_t) g_simUs; }
uint32_t millis() { return (uint32_t) (g_simUs / 1000); }
uint32_t micros() { return (uint32_t) g_simUs; }

static int g_verbose = 1; // 0: summary only, 1: E/W + selected I, 2: all
static std::map<std::string, int> g_backoffs, g_events;

static void countLine(const char *s) {
    if (const char *b = strstr(s, "backoff ")) {
        if (const char *l = strchr(b, '[')) {
            if (const char *r = strchr(l, ']')) {
                std::string why(l + 1, r);
                if (strstr(r, "mid-sweep")) why += " (mid-sweep)";
                ++g_backoffs[why];
            }
        }
    }
    static const char *keys[] = {"Start sweep", "Stop sweep", "Stop sweep: no MPP", "with open backflow switch, ending it",
                                 "High duty cycle with open backflow", "periodic sweep", "Low current", "stuck watchdog",
                                 "restarting", "Calibration failed", "Back-flow switch enabled"};
    for (auto k: keys) if (strstr(s, k)) ++g_events[k];
}

static void emit(char lvl, const char *tag, const char *msg) {
    countLine(msg);
    if (g_verbose >= 2 || (g_verbose == 1 && lvl != 'I' && lvl != 'U')
        || (g_verbose == 1 && (strstr(msg, "sweep") || strstr(msg, "Sweep") || strstr(msg, "backoff"))))
        printf("%10.3f %c %s: %s\n", (double) g_simUs * 1e-6, lvl, tag, msg);
}

void simLog(char lvl, const char *tag, const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    emit(lvl, tag, buf);
}

void UART_LOG(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    size_t n = strlen(buf);
    while (n && buf[n - 1] == '\n') buf[--n] = 0;
    emit('U', "uart", buf);
}

void printf_mux(const char *fmt, ...) {
    if (g_verbose < 2) return;
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

// core-0 task queue, drained after each RT pass
static std::deque<std::function<void()>> g_tasks;
void enqueue_task(std::function<void(void)> &&fn) { g_tasks.push_back(std::move(fn)); }
bool rt_defer(RtDeferredFn fn, void *ctx, float a, float b) {
    g_tasks.push_back([=] { fn(ctx, a, b); });
    return true;
}
void process_queued_tasks() {
    while (!g_tasks.empty()) {
        auto f = std::move(g_tasks.front());
        g_tasks.pop_front();
        f();
    }
}
void loggingEnableDefer() {}
void flush_async_uart_log() {}

// ---- firmware link stubs ---------------------------------------------------------------------
rtcount_entry rtcount_table[RTCOUNT_MAX];
std::atomic<int> rtcount_count{0};
volatile bool rtcount_en = false;
volatile bool g_hwOvTrip = false;
void rtcount(const char *) {}
void assertPinState(uint8_t, bool, const char *, bool) {}
bool LCD::init(uint8_t) { return false; }
void LCD::periodicInit() {}
void LCD::displayMessage(const std::string &, uint16_t) {}
void LCD::displayMessageF(const std::string &, uint16_t, ...) {}

// ---- scenario --------------------------------------------------------------------------------
struct Scn {
    float isc = 0.1f, voc = 66.f, pvK = 0.7f, vbat = 25.6f, rbat = 0.05f, L = 80e-6f;
    float nIout = 0.f, nVin = 0.f, nVout = 0.f, ioutOff = 0.f, vinK = 1.f, ioutK = 1.f;
    float hours = 1.f, sps = 500.f;
    float iscEnd = NAN; // linear irradiance ramp to this Isc over the run
    uint32_t seed = 1;
} S;

// ADC backend reading the plant, one snapshot of every channel per tick (like ADC_VConv).
class SimADC : public AsyncADC<float> {
    uint8_t ch_ = 0;
    std::mt19937 rng_{1};
    std::normal_distribution<float> nd_{0.f, 1.f};
public:
    bool fresh = false;
    void seed(uint32_t s) { rng_.seed(s); }
    [[nodiscard]] AdcReadMode readMode() const override { return AdcReadMode::SnapshotAllChannels; }
    bool init(const ConfFile &) override { return true; }
    void deinit() override {}
    float getSamplingRate(uint8_t) override { return S.sps; }
    void startReading(uint8_t c) override { ch_ = c; }
    bool hasData() override {
        bool f = fresh;
        fresh = false;
        return f;
    }
    void setMaxExpectedVoltage(uint8_t, float) override {}
    float getInputImpedance(uint8_t) override { return 1e9f; }
    float getSample() override {
        switch (ch_) {
            case 0: return g_vconv.getVin() * S.vinK + S.nVin * nd_(rng_);
            case 1: return g_vconv.getVout() + S.nVout * nd_(rng_);
            case 2: return g_vconv.getIoutAvg() * S.ioutK + S.ioutOff + S.nIout * nd_(rng_);
            case 4: return 1.65f; // NTC 25 C
            default: return 0.f;
        }
    }
};

// ---- firmware globals (as in main.cpp) ---------------------------------------------------------
AppState g_app{};
ADC_Sampler adcSampler{};
VIinVout<const Sensor *> sensors{nullptr, nullptr, nullptr, nullptr};
SynchronousConverter converter;
static LCD g_lcd;
MpptController mppt{adcSampler, sensors, converter, g_lcd};
float conversionEfficiency = 0.97f;
static time_us delayStartUntil = 0, timeLastSampler = 0;
static uint32_t lastMpptUpdateNumSamples = 0;
static SimADC *g_adc = nullptr;
static std::vector<std::pair<std::string, std::string>> g_confOverrides; // file.key=value

static ConfFile mkConf(const char *file, std::initializer_list<std::pair<const char *, const char *>> kv) {
    ConfFile c{};
    for (auto &p: kv) c.set(p.first, p.second);
    for (auto &o: g_confOverrides) {
        auto dot = o.first.find('.');
        if (o.first.substr(0, dot) == file) c.set(o.first.substr(dot + 1), o.second);
    }
    return c;
}

// fry-like profile (config/fmetal + coil L0, MCPWM resolution)
static void setupFw() {
    char lbuf[24];
    snprintf(lbuf, sizeof lbuf, "%g", S.L);
    auto board = mkConf("board", {{"pwm_freq", "39000"}, {"pwm_driver_logic", "HiLi"}, {"pwm_hi", "21"},
                                  {"pwm_li", "14"}, {"pwm_sd", "47"}, {"panel_sd", "40"}, {"skip_assert", "1"}});
    auto limitsC = mkConf("limits", {{"vin_min", "10.5"}, {"hv_max", "85"}, {"lv_max", "60"}, {"hv_i_max", "30"},
                                     {"lv_i_max", "32"}, {"iout_short", "6"}, {"p_max", "1000"},
                                     {"temp_max", "90"}, {"temp_derate", "70"}});
    auto charger = mkConf("charger", {{"vout_max", "29"}, {"ibat_max", "100"}, {"bat_c", "280"},
                                      {"tail_c_rate", "0.05"}, {"recharge_dod", "0.2"}, {"cv_eoc", "3.57"},
                                      {"cv_float", "3.37"}});
    auto coil = mkConf("coil", {{"L0", lbuf}, {"rect_cal_auto", "0"}});
    auto conv = mkConf("converter", {{"topo", "buck"}, {"forced_pwm", "0"}});
    auto tracker = mkConf("tracker", {});
    auto sens = mkConf("sensor", {{"filt_len", "20"}, {"expected_hz", "200"}});
    TeleConf tele{};

    setWallClockUs(g_simUs);
    Limits lim{limitsC, false};
    g_app.loopRateMin = (uint16_t) sens.getLong("expected_hz", 200);
    const auto fl = (uint16_t) sens.getLong("filt_len", 20);

    // as setupSensors() for adc=vconv (channel 0=Vin, 1=Vout, 2=Iout, 4=NTC, Iin virtual)
    g_adc = new SimADC();
    g_adc->seed(S.seed);
    auto vt = LinearTransform{1.f, 0.f};
    sensors.Vin = adcSampler.addSensor(g_adc, {0, vt, {lim.Vin_max, 1.8f, false}, "vin", 'V', true}, lim.Vin_max, fl);
    sensors.Iin = adcSampler.addVirtualSensor([] {
        if (!std::isfinite(sensors.Iout->last) || !std::isfinite(sensors.Vin->last) || !std::isfinite(sensors.Vout->last))
            return 0.f;
        if (std::abs(sensors.Iout->last) < .01f or sensors.Vin->last < 0.1f) return 0.f;
        return sensors.Iout->last * sensors.Vout->last / sensors.Vin->last / conversionEfficiency;
    }, fl, "iin", 'A');
    sensors.Iout = adcSampler.addSensor(g_adc, {2, {1.f, 0.f}, {lim.Iout_max * 0.05f, .1f, true}, "iout", 'A', true},
                                        lim.Iout_max, fl);
    auto ntcS = adcSampler.addSensor(g_adc, {4, vt, {2.8f, .1f, false}, "ntc", 'C', false}, 2.8f, 50);
    mppt.ntc.setValueRef(ntcS->ewm.avg.get());
    sensors.Vout = adcSampler.addSensor(g_adc, {1, vt, {lim.Vout_max, .7f, false}, "vout", 'V', false}, lim.Vout_max, fl);
    adcSampler.configureNotch(sens.getByte("notch_adaptive", 1) != 0, sens.f("notch_freq", 100.f),
                              sens.f("notch_q", 20.f), -30.f);
    adcSampler.setRippleSource(sensors.Vout);
    adcSampler.configureDespike(sens.f("despike", 0.f));

    mppt.initSensors(board);
    mppt.charger.begin(charger);
    converter.init(conv, board, coil);
    mppt.rectCal.autoRun = coil.getByte("rect_cal_auto", 1);
    mppt.rectCal.persist = [](int, uint8_t) { return true; };
    mppt.rectCal.init(converter.getRectOnOffset(), !converter.rectOffsetNeedsCal());
    mppt.begin(tracker, board, conv, lim, tele);
    adcSampler.begin();
}

// esp_restart(): rebuild the firmware objects in place (RAM state lost, plant keeps running)
static int g_reboots = 0;
static void simRestart() {
    ++g_reboots;
    emit('W', "sim", "esp_restart() -> reboot");
    adcSampler.~ADC_Sampler();
    new(&adcSampler) ADC_Sampler{};
    converter.~SynchronousConverter();
    new(&converter) SynchronousConverter{};
    mppt.~MpptController();
    new(&mppt) MpptController{adcSampler, sensors, converter, g_lcd};
    g_app.maxLoopLag = 0;
    delayStartUntil = timeLastSampler = 0;
    lastMpptUpdateNumSamples = 0;
    setupFw();
}

// ---- main.cpp RT glue (non-PSU, non-manual paths) ----------------------------------------------
void stopAndBackoff(uint32_t secondsDelay) {
    if (!converter.disabled()) mppt.shutdownDcdc("stopAndBackoff", 5);
    delayStartUntil = wallClockUs() + static_cast<time_us>(secondsDelay) * 1000000ULL;
}

static void loopRTNewData(time_ms nowMs) {
    auto nSamples = sensors.Vout->numSamples;
    bool haveNewSample = (nSamples - lastMpptUpdateNumSamples) > 0;
    if (haveNewSample) {
        timeLastSampler = wallClockUs();
        mppt.rectCalRt();
    }
    if (unlikely(adcSampler.isCalibrating())) {
        mppt.shutdownDcdc("calib", 0);
    } else {
        if (mppt.active()) {
            bool mppt_ok = true;
            if (!mppt.converter.disabled()) {
                mppt_ok &= mppt.protect(false);
                mppt_ok &= mppt.protectLf(false);
            }
            if (mppt_ok) {
                if (haveNewSample) {
                    mppt.update();
                    lastMpptUpdateNumSamples = nSamples;
                }
            } else {
                stopAndBackoff(4);
            }
        } else if (wallClockUs() > delayStartUntil && mppt.startCondition()) {
            // no BMS cell source in the sim: the BMS wait block reduces to !full
            if (!(bool(mppt.charger.termCond) && mppt.charger.batSt.haveValidCellVoltage())) {
                mppt.startSweep();
                delayStartUntil = wallClockUs() + 4 * 1000000ULL;
            }
        }
        if (haveNewSample) {
            auto lv = sensors.Vout;
            mppt.lvPgood.update(lv->med3.get(), lv->ewm.avg.get(), lv->numSamples, !mppt.inBackoff(), (uint32_t) nowMs);
        }
    }
}

static void loopRTOnce() {
    setWallClockUs(g_simUs);
    auto nowUs = wallClockUs();
    auto samplerRet = adcSampler.update();
    converter.applyPendingDeadTimeRt();
    mppt.applyPendingPwmFreqRt();
    if (samplerRet == ADC_Sampler::UpdateRet::CalibFailure) {
        // a sweep's converter is still off here, which stopAndBackoff() skips; the sweep would restart it
        if (mppt.isSweeping()) mppt.shutdownDcdc("calib-fail", 4);
        stopAndBackoff(4);
    }
    if (samplerRet == ADC_Sampler::UpdateRet::NewData) loopRTNewData(nowUs / 1000ULL);
}

// lfStuckWatchdog() (main.cpp), esp_restart -> simRestart
static void lfStuckWatchdog() {
    static int64_t stuckSinceUs = 0;
    static bool triedRelease = false;
    constexpr int64_t RELEASE_US = 30ll * 1000000ll, TIMEOUT_US = 2ll * 3600ll * 1000000ll;
    const int64_t nowUs = esp_timer_get_time();
    bool headroom = sensors.Vin->ewm.avg.get() > sensors.Vout->ewm.avg.get() + 8.0f;
    bool noPower = fabsf(sensors.Iout->ewm.avg.get()) < 0.3f;
    bool stuck = headroom && noPower && !mppt.charger.chargeHold();
    if (!stuck) { stuckSinceUs = 0; triedRelease = false; return; }
    if (!stuckSinceUs) { stuckSinceUs = nowUs; return; }
    if (!triedRelease && (nowUs - stuckSinceUs) >= RELEASE_US) {
        triedRelease = true;
        if (mppt.releaseCvFloorLatch("stuck watchdog")) {
            ESP_LOGW("main", "stuck watchdog: released CV-floor latch in place, retrying");
            return;
        }
    }
    if ((nowUs - stuckSinceUs) < TIMEOUT_US) return;
    ESP_LOGE("main", "No output %us despite headroom (D=%u Vin=%.1f Vout=%.1f Iout=%.2f), restarting",
             (unsigned) ((nowUs - stuckSinceUs) / 1000000ul), converter.getCtrlOnPwmCnt(),
             sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(), sensors.Iout->ewm.avg.get());
    stuckSinceUs = 0;
    triedRelease = false;
    simRestart();
}

// lfControl() (main.cpp), charger part
static void loopLF() {
    lfStuckWatchdog();
    mppt.ntc.read();
    mppt.ucTemp.read();
    const float vout = sensors.Vout->ewm.avg.get(), iout = sensors.Iout->ewm.avg.get();
    uint16_t pwmMargin = std::max<uint16_t>(8, mppt.converter.pwmCtrlMax / 256);
    const uint16_t minAuthorityPwm = mppt.converter.getCtrlOnPwmMin() + pwmMargin;
    const bool currentShowsAuthority = std::isfinite(iout) && (iout > 0.5f || mppt.charger.chargeHold());
    const bool voutAuthority = !mppt.converter.disabled() && mppt.converter.getCtrlOnPwmCnt() > minAuthorityPwm
                               && currentShowsAuthority;
    mppt.charger.update(vout, iout, voutAuthority);
    mppt.charger.updateWire(vout, iout, !mppt.converter.disabled());
    if (g_verbose >= 2)
        UART_LOG("V=%.2f/%.2f I=%.3f/%.3fA %.2fW D=%u|%u bf=%d st=%s", sensors.Vin->ewm.avg.get(), vout,
                 sensors.Iin->ewm.avg.get(), iout, sensors.Vin->ewm.avg.get() * sensors.Iin->ewm.avg.get(),
                 converter.getCtrlOnPwmCnt(), converter.getRectOnPwmCnt(), (int) mppt.bflow.state(),
                 MpptState2String[(uint8_t) mppt.getState()]);
}

static float argf(const char *a, const char *k, float &dst) {
    size_t n = strlen(k);
    if (!strncmp(a, k, n) && a[n] == '=') dst = (float) atof(a + n + 1);
    return dst;
}

int main(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        float seed = (float) S.seed, verb = (float) g_verbose;
        argf(a, "isc", S.isc); argf(a, "voc", S.voc); argf(a, "pv_k", S.pvK); argf(a, "vbat", S.vbat);
        argf(a, "r_bat", S.rbat); argf(a, "L", S.L); argf(a, "noise_iout", S.nIout); argf(a, "noise_vin", S.nVin);
        argf(a, "noise_vout", S.nVout); argf(a, "vin_k", S.vinK); argf(a, "iout_k", S.ioutK); argf(a, "iout_off", S.ioutOff); argf(a, "hours", S.hours);
        argf(a, "sps", S.sps); argf(a, "isc_end", S.iscEnd); argf(a, "seed", seed); argf(a, "v", verb);
        S.seed = (uint32_t) seed;
        g_verbose = (int) verb;
        if (const char *eq = strchr(a, '='); eq && strchr(a, '.') && strchr(a, '.') < eq)
            g_confOverrides.emplace_back(std::string(a, eq), eq + 1);
    }
    PWM_Mock::pwmMaxOverride = 4103; // MCPWM resolution at 39 kHz

    g_vconv.setPv(S.isc, S.voc, S.pvK);
    g_vconv.setBat(S.vbat, S.rbat);
    g_vconv.setPassives(470e-6f, 470e-6f, S.L);
    g_vconv.setVin(S.voc);
    g_vconv.setVout(S.vbat);
    g_vconv.setDiodeVf(0.7f);

    setupFw();

    const uint32_t fsw = 39000;
    const time_us dtUs = (time_us) (1e6f / S.sps), endUs = (time_us) (S.hours * 3600e6);
    time_us lfAt = 0;
    double tConv = 0, energy = 0, sumVin = 0, sumP = 0, sumIsc = 0;
    uint64_t nConv = 0, nSweeping = 0;
    float vinHist[8] = {}; // Vin while converting outside sweeps: <30, 30-35, 35-40, 40-45, 45-50, 50-55, 55-60, >=60

    while (g_simUs < endUs) {
        g_simUs += dtUs;
        const float frac = (float) ((double) g_simUs / (double) endUs);
        const float isc = std::isfinite(S.iscEnd) ? S.isc + (S.iscEnd - S.isc) * frac : S.isc;
        g_vconv.setPv(isc, S.voc, S.pvK);
        g_vconv.setPvBlocking(!mppt.bflow.state());
        const bool on = !converter.disabled();
        g_vconv.setPwm({4103, on ? converter.getCtrlOnPwmCnt() : (uint16_t) 0,
                        on ? converter.getRectOnPwmCnt() : (uint16_t) 0, fsw});
        g_vconv.stepSeconds((float) dtUs * 1e-6f, fsw);
        if (on) {
            const float dt = (float) dtUs * 1e-6f, p = g_vconv.getVout() * g_vconv.getIoutAvg();
            tConv += dt;
            energy += p * dt;
            if (!mppt.isSweeping()) {
                sumVin += g_vconv.getVin();
                sumP += p;
                sumIsc += isc;
                ++nConv;
                int b = (int) ((g_vconv.getVin() - 25.f) / 5.f);
                ++vinHist[std::clamp(b, 0, 7)];
            } else ++nSweeping;
        }
        g_adc->fresh = true;
        loopRTOnce();
        process_queued_tasks();
        if (g_simUs >= lfAt) {
            lfAt = g_simUs + 3000000ULL;
            loopLF();
        }
    }

    const double h = (double) endUs / 3600e6;
    printf("\n--- summary: isc=%.3f%s voc=%.1f pv_k=%.2f vbat=%.2f L=%.0fuH noise_iout=%.3f iout_off=%.2f sps=%.0f %.2fh\n",
           S.isc, std::isfinite(S.iscEnd) ? ("->" + std::to_string(S.iscEnd)).c_str() : "", S.voc, S.pvK, S.vbat,
           S.L * 1e6, S.nIout, S.ioutOff, S.sps, h);
    PvModel pm;
    pm.set(S.isc, S.voc, S.pvK);
    float mppP = 0, mppV = 0;
    for (float v = 0; v < S.voc; v += 0.05f)
        if (v * pm.current(v) > mppP) mppP = v * pm.current(v), mppV = v;
    printf("plant MPP (initial isc) %.2f W @ %.1f V\n", mppP, mppV);
    printf("converting %.1f%% of time, energy %.2f Wh (%.2f W avg over run)\n", 100 * tConv / (h * 3600),
           energy / 3600, energy / (h * 3600));
    if (nConv)
        printf("tracking (not sweeping): mean Vin %.2f V, mean P %.3f W, %.1f%% of converting time sweeping\n",
               sumVin / nConv, sumP / nConv, 100.0 * nSweeping / (nSweeping + nConv));
    if (nConv) {
        printf("Vin while tracking:");
        const char *lbl[] = {"<30", "30-35", "35-40", "40-45", "45-50", "50-55", "55-60", ">=60"};
        for (int i = 0; i < 8; ++i) if (vinHist[i]) printf(" %s:%.0f%%", lbl[i], 100.0 * vinHist[i] / nConv);
        printf("\n");
    }
    int trips = 0;
    for (auto &b: g_backoffs) {
        printf("backoff [%s]: %d (%.1f/h)\n", b.first.c_str(), b.second, b.second / h);
        if (b.first.rfind("highD-bflowOpen", 0) == 0) trips += b.second;
    }
    for (auto &e: g_events) printf("event \"%s\": %d (%.1f/h)\n", e.first.c_str(), e.second, e.second / h);
    printf("reboots: %d\nhighD-bflowOpen trips: %d (%.1f/h)\n", g_reboots, trips, trips / h);
    const double trk = nConv ? sumVin / nConv : NAN;
    printf("RESULT trips/h=%.1f mid/h=%.1f stopAndBackoff/h=%.1f sweeps/h=%.1f W=%.2f Vin=%.1f tracking=%.0f%% reboots=%d\n",
           trips / h, g_backoffs["highD-bflowOpen (mid-sweep)"] / h,
           (g_backoffs["stopAndBackoff"] + g_backoffs["stopAndBackoff (mid-sweep)"]) / h, g_events["Start sweep"] / h,
           energy / (h * 3600), trk, 100.0 * (double) nConv * dtUs * 1e-6 / (h * 3600), g_reboots);
    return 0;
}
