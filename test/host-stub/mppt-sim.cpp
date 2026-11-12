// Host closed-loop simulator: the real RT control path (ADC_Sampler + Sensor filters, MpptController
// protect()/update(), PD controllers, tracker, SynchronousConverter) driving the VirtualConverter
// plant through the firmware's own vconv ADC backend (adc/vconv.h) and PWM driver (pwm/vconv.h), as
// on a CONFIG_FUGU_WITH_VCONV build. loopRT/loopRTNewData/stopAndBackoff and the low-frequency
// control work of loopLF (lfControl, lfWatchdog, lfStuckWatchdog) are replicated from main.cpp.
// Time is simulated: every ADC timer tick (1 / vconv.conf::adc_freq, quantized to 1 MHz like the
// gptimer) advances esp_timer_get_time() and steps the plant once (ADC_VConv::hasData).
// Config: config/lab/vconv_mock/conf is copied to /tmp/mppt-sim-XXXXXX/littlefs/conf and read from
// there through the /littlefs fopen redirect (esp/sim_prelude.h); converter.conf mode psu -> mppt,
// per-scenario keys appended to vconv.conf / limits.conf. Stubs: test/host-stub/esp/.
//
//   test/host-stub/mppt-sim.sh [scenario ...]   # builds /tmp/mppt-sim, runs the scenarios (default all)
//   /tmp/mppt-sim <scenario>   # sanity | a | b | c | d-cloud | d-cloud-ramp | d-bus04 | d-bus08 | d-bus12 |
//                              # d-noise | e | e-bleed; suffix -p0 = limits.conf reverse_current_paranoia=0
//   SIM_FINE=t0,t1 /tmp/mppt-sim c              # per-tick sensor trace between t0 and t1 [s]
//
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <filesystem>
#include <functional>
#include <unistd.h>

#include "adc/sampling.h"
#include "adc/sensor_setup.h"
#include "buck.h"
#include "mppt.h"
#include "app_state.h"
#include "sim/vconv.h"

// ---- globals owned by main.cpp on target ----
ADC_Sampler adcSampler{};
VIinVout<const Sensor *> sensors{nullptr, nullptr, nullptr, nullptr};
SynchronousConverter converter;
static LCD simLcd;
MpptController mppt{adcSampler, sensors, converter, simLcd};
time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;
time_us lastLoopTime = 0, timeLastSampler = 0, delayStartUntil = 0, lastTimeOutUs = 0, lastLfWorkUs = 0;
uint32_t lastNSamples = 0;
unsigned long lastMpptUpdateNumSamples = 0;
float conversionEfficiency;
AppState g_app{};
static const auto lfPeriod = 3000000ULL;

extern std::string g_simFsRoot;
extern uint32_t g_simTickUs;
extern void (*g_simLogHook)(char lvl, const char *tag, const char *msg);

// ---- trace / event capture ----
struct Trip {
    double t;
    std::string who;
    unsigned sec;
};
static std::vector<Trip> g_trips;
static std::vector<std::string> g_logSince; // log lines after the scenario event
static double g_eventT = -1;
static double g_fineT0 = 0, g_fineT1 = 0; // env SIM_FINE=t0,t1: per-tick sensor trace window
static double g_rampSec = 0, g_rampT0 = 0, g_rampFrom = 0, g_rampTo = 0; // linear Isc ramp
static bool g_verbose = true;

static double tNow() { return (double) esp_timer_get_time() * 1e-6; }

static void logHook(char lvl, const char *tag, const char *msg) {
    // backoff log of MpptController::shutdownDcdc: "backoff %lus [%s]..."
    unsigned sec;
    char who[64];
    if (sscanf(msg, "backoff %us [%63[^]]]", &sec, who) == 2) g_trips.push_back({tNow(), who, sec});
    char line[640];
    snprintf(line, sizeof line, "%9.4f %c %-8s %s", tNow(), lvl, tag, msg);
    if (g_eventT >= 0 && g_logSince.size() < 400) g_logSince.emplace_back(line);
    // rate-limit the per-tick chatter ("Limiting!", "Current above/below threshold")
    static double winT = -1;
    static int winN = 0;
    if (tNow() - winT > 1.0) { winT = tNow(); winN = 0; }
    if (g_verbose && (lvl != 'U' || ++winN <= 6)) printf("%s\n", line);
}

// ---- replicated main.cpp RT path ----
void systemRestart() {
    printf("%9.4f !!! systemRestart() requested (not modelled, sim continues)\n", tNow());
}

void stopAndBackoff(uint32_t secondsDelay) {
    const bool psuFast = g_app.psuMode() && secondsDelay <= 5;
    if (!converter.disabled())
        mppt.shutdownDcdc("stopAndBackoff", psuFast ? 0 : 5);
    delayStartUntil = wallClockUs() + (psuFast ? 100000ULL : static_cast<time_us>(secondsDelay) * 1000000ULL);
}

static void loopRTNewData(time_ms nowMs) {
    auto nSamples = sensors.Vout->numSamples;
    bool haveNewSample = (nSamples - lastMpptUpdateNumSamples) > 0;
    mppt.applyPendingPsuCommandRt(haveNewSample);
    if (haveNewSample)
        timeLastSampler = wallClockUs();

    if (unlikely(adcSampler.isCalibrating())) {
        mppt.shutdownDcdc("calib", 0);
    } else {
        if (mppt.active() or g_app.manualPwm()) {
            bool mppt_ok = true;
            if (!mppt.converter.disabled()) {
                mppt_ok &= mppt.protect(g_app.manualPwm());
                mppt_ok &= mppt.protectLf(g_app.manualPwm());
            }
            if (mppt_ok) {
                if (haveNewSample) {
                    if (!g_app.manualPwm()) mppt.update();
                    else mppt.updateManual();
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
                static int64_t bmsBootDeadline = 0;
                static int64_t bmsRearmAllowedUs = 0;
                static bool bmsWasFresh = false;
                constexpr int64_t BMS_BOOT_WAIT_US = 12'000'000;
                constexpr int64_t BMS_REARM_MIN_US = 10ll * 60 * 1000000;
                bool bmsFresh = mppt.charger.batSt.haveValidCellVoltage();
                bool full = bool(mppt.charger.termCond) && bmsFresh;
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
                    mppt.startSweep();
                    delayStartUntil = wallClockUs() + 4 * 1000000ULL;
                }
            }
        }
        if (haveNewSample) {
            auto lv = converter.boost() ? sensors.Vin : sensors.Vout;
            mppt.lvPgood.update(lv->med3.get(), lv->ewm.avg.get(), lv->numSamples, !mppt.inBackoff(),
                                (uint32_t) nowMs);
        }
    }
    if (g_app.manualPwm()) {
        if (!converter.disabled()) converter.pwmPerturb(0);
    }
}

// one iteration of loopRT's while(true) body
static void loopRTIter() {
    setWallClockUs(esp_timer_get_time());
    auto &nowUs(loopWallClockUs_);
    time_ms nowMs = nowUs / 1000ULL;

    if (unlikely(adcSampler.adcResetPending())) {
        if (adcSampler.halted) {
            adcSampler.dropAdcReset();
        } else {
            stopAndBackoff(6);
            bool ok = adcSampler.applyAdcResetRt();
            setWallClockUs(esp_timer_get_time());
            stopAndBackoff(ok ? 6 : 30);
            if (timeLastSampler) timeLastSampler = nowUs;
        }
    }

    auto samplerRet = adcSampler.update();
    converter.applyPendingDeadTimeRt();
    mppt.applyPendingPwmFreqRt();
    if (adcSampler.halted) return;
    if (samplerRet == ADC_Sampler::UpdateRet::CalibFailure) stopAndBackoff(4);

    constexpr time_ms kAdcResetThrottleMs = 300, kAdcSustainedMs = 800, kAdcRestartMs = 60000;
    static time_ms adcStalledSinceMs = 0, lastAdcResetMs = 0;
    bool adcStalled = samplerRet == ADC_Sampler::UpdateRet::AdcError ||
                      (samplerRet != ADC_Sampler::UpdateRet::NewData && !adcSampler.isCalibrating() &&
                       timeLastSampler && nowUs - timeLastSampler > 200000);
    if (adcStalled) {
        if (!adcStalledSinceMs) adcStalledSinceMs = nowMs;
        mppt.lvPgood.release();
        if (nowMs - lastAdcResetMs > kAdcResetThrottleMs) {
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
        mppt.applyPendingPsuCommandRt(false);
        if (adcSampler.isCalibrating() && mppt.boardPowerSupplyUnderVoltage()) {
            ESP_LOGW("main", "Board power supply UV %.2f!", mppt.boardPowerSupplyVoltage());
            adcSampler.cancelCalibration();
        }
        if (!timeLastSampler and nowMs > 20000) {
            converter.disable();
            ESP_LOGE("main", "Never got a sample! Please check ADC");
        }
    } else {
        loopRTNewData(nowMs);
    }

    auto lag = nowUs - lastLoopTime;
    if (lastLoopTime && lag > g_app.maxLoopLag && !converter.disabled()) g_app.maxLoopLag = lag;
    lastLoopTime = nowUs;
}

// ---- replicated loopLF control work (core 0 on target, every lfPeriod) ----
static void lfWatchdog(time_us nowUs, uint32_t dt, uint32_t sps, uint32_t nSamples) {
    static uint8_t starvedWindows = 0;
    constexpr uint8_t TRIP_WINDOWS = 3;
    bool starved = (dt > (lfPeriod * 0.9f)) && sps < g_app.loopRateMin && !converter.disabled() &&
                   nSamples > max(g_app.loopRateMin * 5, 200) &&
                   !g_app.manualPwm() && lastTimeOutUs && (nowUs - adcSampler.getTimeLastCalibrationUs()) > 2000000;
    if (!starved) { starvedWindows = 0; return; }
    if (++starvedWindows < TRIP_WINDOWS) {
        ESP_LOGW("main", "Loop latency high (%lu<%hu Hz), window %d/%d", (unsigned long) sps, g_app.loopRateMin,
                 (int) starvedWindows, (int) TRIP_WINDOWS);
        return;
    }
    starvedWindows = 0;
    ESP_LOGE("main", "Loop latency high (%lu<%hu Hz), shutdown!", (unsigned long) sps, g_app.loopRateMin);
    stopAndBackoff(4);
}

static void lfStuckWatchdog() {
    static int64_t stuckSinceUs = 0;
    static bool triedRelease = false;
    constexpr int64_t RELEASE_US = 30ll * 1000000ll;
    constexpr int64_t TIMEOUT_US = 2ll * 3600ll * 1000000ll;
    const int64_t nowUs = esp_timer_get_time();
    bool headroom = sensors.Vin && sensors.Vout
                    && (converter.boost()
                            ? sensors.Vin->ewm.avg.get() > mppt.limits.Vin_min + 2.0f
                            : sensors.Vin->ewm.avg.get() > sensors.Vout->ewm.avg.get() + 8.0f);
    bool noPower = sensors.Iout && fabsf(sensors.Iout->ewm.avg.get()) < 0.3f;
    const float psuSetpoint = mppt.getPsuSetpoint();
    bool stuck = headroom && noPower && !g_app.manualPwm()
                 && (g_app.psuMode()
                         ? (std::isfinite(psuSetpoint) && sensors.Vout->ewm.avg.get() < psuSetpoint - 2.0f)
                         : !mppt.charger.chargeHold());
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
    ESP_LOGE("main", "No output despite headroom, restarting");
    systemRestart();
}

static void lfControl() {
    mppt.ntc.read();
    mppt.ucTemp.read();
    if (sensors.Vout && sensors.Iout) {
        const float vout = sensors.Vout->ewm.avg.get();
        const float iout = sensors.Iout->ewm.avg.get();
        uint16_t pwmMargin = mppt.converter.pwmCtrlMax / 256;
        if (pwmMargin < 8) pwmMargin = 8;
        const uint16_t minAuthorityPwm = mppt.converter.getCtrlOnPwmMin() + pwmMargin;
        const bool currentShowsAuthority = std::isfinite(iout) && (iout > 0.5f || mppt.charger.chargeHold());
        const bool voutAuthority = !mppt.converter.disabled()
                                   && mppt.converter.getCtrlOnPwmCnt() > minAuthorityPwm
                                   && currentShowsAuthority;
        mppt.charger.update(vout, iout, voutAuthority);
    }
}

static void loopLF(time_us nowUs) {
    auto &nSamples(sensors.Vout ? sensors.Vout->numSamples : lastNSamples);
    static time_us lastWindowUs = 0;
    auto dt = nowUs - lastWindowUs;
    uint32_t sps = (dt > 20000) ? (uint64_t) (nSamples - lastNSamples) * 1000000llu / dt : 0;
    lfWatchdog(nowUs, dt, sps, nSamples);
    lfStuckWatchdog();
    lfControl();
    lastTimeOutUs = wallClockUs();
    lastNSamples = nSamples;
    lastWindowUs = nowUs;
}

// ---- harness ----
static const char *modeStr() {
    if (mppt.converter.disabled()) return mppt.inBackoff() ? "OFF/backoff" : "OFF";
    return MpptState2String[(int) mppt.getState()];
}

static void traceLine(const char *pfx = "") {
    auto s = mppt.teleSnap();
    printf("%9.4f %sVin=%5.1f Vout=%6.3f Iout=%6.2f P=%6.1f D=%4u %-11s lim=%2u | plant Vin=%5.1f Vout=%6.3f Iout=%6.2f\n",
           tNow(), pfx, sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(), sensors.Iout->ewm.avg.get(),
           sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get(), converter.getCtrlOnPwmCnt(), modeStr(),
           (unsigned) s.limIdx, g_vconv.getVin(), g_vconv.getVout(), g_vconv.getIoutAvg());
}

static void writeConf(const std::string &dir, const char *file,
                      const std::vector<std::pair<std::string, std::string>> &kv) {
    std::string p = dir + "/" + file;
    FILE *f = ::fopen(p.c_str(), "a");
    for (auto &[k, v]: kv) fprintf(f, "\n%s=%s\n", k.c_str(), v.c_str());
    fclose(f);
}

static void boot(const std::vector<std::pair<std::string, std::string>> &vconvOverrides, bool paranoia0) {
    namespace fs = std::filesystem;
    char tmpl[] = "/tmp/mppt-sim-XXXXXX";
    g_simFsRoot = mkdtemp(tmpl);
    std::string confDir = g_simFsRoot + "/littlefs/conf";
    fs::create_directories(confDir);
    std::string src = std::string(SIM_REPO_ROOT) + "/config/lab/vconv_mock/conf";
    for (auto &e: fs::directory_iterator(src)) fs::copy_file(e.path(), confDir + "/" + e.path().filename().string());
    // vconv_mock ships mode=psu; BMS-cutoff detection is off in PSU mode, so run it as an MPPT charger
    {
        std::string p = confDir + "/converter.conf";
        std::string s;
        FILE *f = ::fopen(p.c_str(), "r");
        char buf[256];
        while (fgets(buf, sizeof buf, f)) {
            if (!strncmp(buf, "mode=", 5) || !strncmp(buf, "psu_vout=", 9)) continue;
            s += buf;
        }
        fclose(f);
        f = ::fopen(p.c_str(), "w");
        fputs(s.c_str(), f);
        fputs("\nmode=mppt\n", f);
        fclose(f);
    }
    if (!vconvOverrides.empty()) writeConf(confDir, "vconv.conf", vconvOverrides);
    if (paranoia0) writeConf(confDir, "limits.conf", {{"reverse_current_paranoia", "0"}});

    // setup(): the parts of setupConverterAndMppt() that the RT path needs
    ConfFile boardConf{"/littlefs/conf/board.conf"};
    ConfFile limitsConf{"/littlefs/conf/limits.conf"};
    Limits lim{limitsConf};
    setupSensors(boardConf, lim);
    mppt.initSensors(boardConf);
    ConfFile coilConf{"/littlefs/conf/coil.conf"};
    ConfFile chargerConf{"/littlefs/conf/charger.conf"};
    ConfFile converterConf{"/littlefs/conf/converter.conf"};
    mppt.charger.begin(chargerConf);
    converter.init(converterConf, boardConf, coilConf);
    ConfFile trackerConf{"/littlefs/conf/tracker.conf", true};
    mppt.begin(trackerConf, boardConf, converterConf, lim, TeleConf{});
    if (g_app.setupErr) { printf("setupErr\n"); exit(2); }
    adcSampler.begin(); // loopRT prologue
    printf("# sim tick %u us, pwm_freq %lu, Vbat_max %.2f, Iout_max %.1f, p_max %.0f\n", g_simTickUs,
           (unsigned long) converter.getPwmFrequency(), mppt.charger.params.Vbat_max, lim.Iout_max, lim.P_max);
}

struct Probe {
    double t0;
    float voutPeakSensor = -1, voutPeakPlant = -1;
    float ioutBefore, voutBefore, vinBefore, pBefore;
    size_t tripIdx0;
    double firstOffT = -1;
    // bmsCutoffSignature() terms, evaluated while the converter runs (bmsCutoff() is gated on that)
    uint32_t nOn = 0, nC[4] = {}, nThree = 0, nAll = 0;
    float minIRatio = INFINITY, maxDvRatio = -INFINITY;
};

// call after a tick that started with the converter on: sensors hold what protect() saw
static void probeDetector(Probe &p) {
    const float ioutMin = std::max(1.f, mppt.limits.Iout_max * 0.05f);
    const float dvMin = mppt.charger.params.Vbat_max * 0.02f;
    const float i = sensors.Iout->med3.get(), ia = sensors.Iout->ewm.avg.get();
    const float v = sensors.Vout->med3.get(), va = sensors.Vout->ewm.avg.get();
    const float vi = sensors.Vin->med3.get(), via = sensors.Vin->ewm.avg.get();
    bool c[4] = {ia > ioutMin, i < ia * 0.25f, v > va + dvMin, vi > via};
    int n = 0;
    for (int k = 0; k < 4; ++k) if (c[k]) { ++p.nC[k]; ++n; }
    ++p.nOn;
    if (n == 3) ++p.nThree;
    if (n == 4) ++p.nAll;
    if (c[0]) p.minIRatio = std::min(p.minIRatio, i / ia);
    p.maxDvRatio = std::max(p.maxDvRatio, (v - va) / dvMin);
}

static Probe probeStart() {
    Probe p{};
    p.t0 = tNow();
    p.ioutBefore = sensors.Iout->ewm.avg.get();
    p.voutBefore = sensors.Vout->ewm.avg.get();
    p.vinBefore = sensors.Vin->ewm.avg.get();
    p.pBefore = p.ioutBefore * p.voutBefore;
    p.tripIdx0 = g_trips.size();
    g_eventT = p.t0;
    g_logSince.clear();
    return p;
}

// run for `sec`, tracing every `traceEvery` s; per-tick trace for the first `fineMs` after start
static void run(double sec, double traceEvery, Probe *probe = nullptr, double fineMs = 0) {
    double tEnd = tNow() + sec, nextTrace = tNow();
    bool wasOn = !converter.disabled();
    while (tNow() < tEnd) {
        if (g_rampSec > 0) {
            double f = std::min(1.0, (tNow() - g_rampT0) / g_rampSec);
            g_vconv.setPv((float) (g_rampFrom + (g_rampTo - g_rampFrom) * f), g_vconv.getVoc(), g_vconv.getPvK());
            if (f >= 1.0) g_rampSec = 0;
        }
        bool onBefore = !converter.disabled();
        loopRTIter();
        if (probe && onBefore) probeDetector(*probe);
        if ((wallClockUs() - lastLfWorkUs) >= lfPeriod or !lastLfWorkUs) {
            loopLF(wallClockUs());
            lastLfWorkUs = wallClockUs();
        }
        bool on = !converter.disabled();
        if (g_fineT1 > 0 && tNow() >= g_fineT0 && tNow() <= g_fineT1) {
            printf("%9.4f fine: Vout last=%.3f med=%.3f ewm=%.3f n=%lu | Iout last=%.2f med=%.2f ewm=%.2f | D=%u %s cal=%d\n",
                   tNow(), sensors.Vout->last, sensors.Vout->med3.get(), sensors.Vout->ewm.avg.get(),
                   (unsigned long) sensors.Vout->numSamples, sensors.Iout->last, sensors.Iout->med3.get(),
                   sensors.Iout->ewm.avg.get(), converter.getCtrlOnPwmCnt(), modeStr(), (int) adcSampler.isCalibrating());
        }
        if (probe) {
            probe->voutPeakSensor = std::max(probe->voutPeakSensor, sensors.Vout->last);
            probe->voutPeakPlant = std::max(probe->voutPeakPlant, g_vconv.getVout());
            if (wasOn && !on && probe->firstOffT < 0) probe->firstOffT = tNow();
            if ((tNow() - probe->t0) * 1e3 < fineMs) traceLine("  ");
        }
        if (on != wasOn && g_verbose) printf("%9.4f --- converter %s\n", tNow(), on ? "ON" : "OFF");
        wasOn = on;
        if (traceEvery > 0 && tNow() >= nextTrace) {
            traceLine();
            nextTrace += traceEvery;
        }
    }
}

static void report(const char *name, const Probe &p) {
    printf("\n== %s: event at t=%.4f s, before: Vin=%.2f Vout=%.3f Iout=%.2f P=%.1f\n", name, p.t0, p.vinBefore,
           p.voutBefore, p.ioutBefore, p.pBefore);
    if (g_trips.size() > p.tripIdx0) {
        auto &t = g_trips[p.tripIdx0];
        printf("== first protection: [%s] backoff %us after %.1f ms\n", t.who.c_str(), t.sec, (t.t - p.t0) * 1e3);
        for (size_t i = p.tripIdx0; i < g_trips.size() && i < p.tripIdx0 + 12; ++i)
            printf("==   trip %zu: t=%.4f (+%.1f ms) [%s] %us\n", i - p.tripIdx0, g_trips[i].t,
                   (g_trips[i].t - p.t0) * 1e3, g_trips[i].who.c_str(), g_trips[i].sec);
        if (g_trips.size() - p.tripIdx0 > 12) printf("==   ... %zu trips total\n", g_trips.size() - p.tripIdx0);
    } else {
        printf("== no protection trip\n");
    }
    printf("== detector terms over %u on-ticks: ioutAvg>min %u, iout<avg/4 %u, dVout>2%%Vbat_max %u, Vin>avg %u;"
           " 3-of-4 %u, all %u; min iout/avg %.3f, max dVout/dvMin %.2f\n",
           p.nOn, p.nC[0], p.nC[1], p.nC[2], p.nC[3], p.nThree, p.nAll, p.minIRatio, p.maxDvRatio);
    if (p.firstOffT >= 0) printf("== converter first off at +%.1f ms\n", (p.firstOffT - p.t0) * 1e3);
    printf("== peak Vout: sensor(last) %.3f V, plant %.3f V\n", p.voutPeakSensor, p.voutPeakPlant);
}

static void settle(double sec) {
    run(sec, 2.0);
    traceLine("settled ");
}

int main(int argc, char **argv) {
    std::string sc = argc > 1 ? argv[1] : "sanity";
    g_simLogHook = logHook;
    if (const char *f = getenv("SIM_FINE")) sscanf(f, "%lf,%lf", &g_fineT0, &g_fineT1);
    // suffix -p0: limits.conf reverse_current_paranoia=0 (Vout-OV threshold 1.5x instead of 1.03x Vbat_max)
    const bool p0 = sc.size() > 3 && sc.compare(sc.size() - 3, 3, "-p0") == 0;
    const std::string name = sc;
    if (p0) sc.resize(sc.size() - 3);
    std::vector<std::pair<std::string, std::string>> ov;
    if (sc == "b") ov = {{"isc", "1.6"}};
    if (sc == "c") ov = {{"v_bat", "28.7"}};
    if (sc == "d-noise") ov = {{"noise_vout", "0.03"}, {"vbat_ac_amp", "0.26"}, {"vbat_ac_freq", "100"}};
    boot(ov, p0);

    if (sc == "sanity") {
        Probe p = probeStart();
        g_eventT = -1;
        run(90, 1.0, &p);
        printf("\n== sanity: %zu trips in 90 s\n", g_trips.size());
        for (auto &t: g_trips) printf("==   t=%.3f [%s] %us\n", t.t, t.who.c_str(), t.sec);
        traceLine("final ");
        return 0;
    }

    settle(sc == "d-noise" ? 60 : 60);
    size_t settleTrips = g_trips.size();
    printf("# trips during settle: %zu\n", settleTrips);

    if (sc == "a" || sc == "b" || sc == "c" || sc == "e" || sc == "e-bleed") {
        const bool e = sc[0] == 'e';
        Probe p = probeStart();
        // e-bleed: open battery but a 1 kOhm residual output load (hypothetical divider/aux drain)
        g_vconv.setBat(0, sc == "e-bleed" ? 1e3f : 1e9f);
        printf("%9.4f >>> vconv bat open (r=%g)\n", tNow(), g_vconv.getRbat());
        run(e ? 120 : 6, e ? 2.0 : 0.5, &p, 25);
        report(name.c_str(), p);
        if (e) {
            Probe q = probeStart();
            g_vconv.setBat(27.f, 0.05f);
            printf("%9.4f >>> vconv bat reconnect 27 V / 0.05 ohm\n", tNow());
            run(60, 2.0, &q, 0);
            report("e-reconnect", q);
            traceLine("final ");
        }
    } else if (sc == "d-cloud" || sc == "d-cloud-ramp") {
        Probe p = probeStart();
        if (sc == "d-cloud") {
            g_vconv.setPv(3.f, g_vconv.getVoc(), g_vconv.getPvK());
            printf("%9.4f >>> Isc 13 -> 3 A step\n", tNow());
        } else {
            g_rampT0 = tNow(), g_rampSec = 1.0, g_rampFrom = g_vconv.getIsc(), g_rampTo = 3.0;
            printf("%9.4f >>> Isc 13 -> 3 A linear over 1 s\n", tNow());
        }
        run(60, 1.0, &p, 20);
        report(name.c_str(), p);
    } else if (sc.rfind("d-bus", 0) == 0) {
        float dv = std::stof(sc.substr(5)) / 10.f;
        Probe p = probeStart();
        g_vconv.setBat(g_vconv.getVbat() + dv, g_vconv.getRbat());
        printf("%9.4f >>> Vbat +%.1f V -> %.2f V\n", tNow(), dv, g_vconv.getVbat());
        run(30, 1.0, &p, 25);
        report(name.c_str(), p);
    } else if (sc == "d-noise") {
        Probe p = probeStart();
        run(300, 10.0, &p);
        report(name.c_str(), p);
    } else {
        printf("unknown scenario %s\n", sc.c_str());
        return 1;
    }

    printf("\n== log after event (first %zu lines):\n", g_logSince.size());
    for (size_t i = 0; i < g_logSince.size() && i < 60; ++i) printf("   %s\n", g_logSince[i].c_str());
    return 0;
}
