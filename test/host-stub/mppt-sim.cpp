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
//                              # shade | shade-cv: half-shaded string (two peaks)
//   weather scenarios (PV string of 1..3 substrings, irradiance played from a time series; "== SUMMARY" line):
//     wx-cloud          recorded cloudy quarter hour (flat, 2026-11-14 11:20 UTC), full sun <-> cloud
//     wx-cloud-cv       same with the battery at the Vout cap in full sun; -ripple adds 100 Hz bus ripple
//     wx-shade2, wx-shade3            static partial shade, 2 / 3 power peaks
//     wx-shade2-move, wx-shade3-move  shade moves, the global maximum changes peak after the boot sweep
//     wx-shade-cloud    shaded substring under the cloud series (second peak comes and goes)
//     wx-dawn, wx-dusk  irradiance through the start threshold
//     wx-sweep-fall, wx-sweep-rise, wx-sweep-shade   cloud edge during a sweep (-shade: two peaks)
//   SIM_T=600 wx run length [s], SIM_TR=5 wx trace interval [s], SIM_V=1 firmware log in wx/shade runs
//   SIM_FINE=t0,t1 /tmp/mppt-sim c              # per-tick sensor trace between t0 and t1 [s]
//   SIM_ADC_HZ=3000 /tmp/mppt-sim a             # ADC tick rate [Hz], default 450
//   SIM_CONF="coil.conf:L0=80e-6,vconv.conf:c_out=470e-6" /tmp/mppt-sim a   # board overrides
//
#include <algorithm>
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
static std::vector<double> g_sweepT; // "Start sweep" log times
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
    if (!strcmp(msg, "Start sweep")) g_sweepT.push_back(tNow());
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
    // pin the ADC tick to a board rate (SIM_ADC_HZ, default ~fmetal's 451 Hz) instead of the mock's
    const char *hzEnv = getenv("SIM_ADC_HZ");
    const int hz = hzEnv ? atoi(hzEnv) : 450;
    writeConf(confDir, "vconv.conf", {{"adc_freq", std::to_string(hz)}});
    writeConf(confDir, "sensor.conf", {{"expected_hz", std::to_string(hz * 4 / 5)}});
    // SIM_CONF="coil.conf:L0=80e-6,vconv.conf:c_out=470e-6" appends board overrides
    if (const char *c = getenv("SIM_CONF")) {
        std::string all = c;
        for (size_t i = 0, j; i < all.size(); i = j + 1) {
            j = all.find(',', i);
            if (j == std::string::npos) j = all.size();
            std::string kv = all.substr(i, j - i);
            size_t colon = kv.find(':'), eq = kv.find('=');
            if (colon != std::string::npos && eq != std::string::npos && eq > colon)
                writeConf(confDir, kv.substr(0, colon).c_str(), {{kv.substr(colon + 1, eq - colon - 1), kv.substr(eq + 1)}});
        }
    }
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
    g_vconv.takeVoutMax();
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
            float vMax = g_vconv.takeVoutMax();
            if (tNow() - probe->t0 < 1.0) probe->voutPeakPlant = std::max(probe->voutPeakPlant, vMax);
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
    printf("== peak Vout: sensor(last) %.3f V, plant (per PWM cycle, 1 s) %.3f V\n", p.voutPeakSensor, p.voutPeakPlant);
}

static void settle(double sec) {
    run(sec, 2.0);
    traceLine("settled ");
}

// ---- weather scenarios (wx-*) ----
using Track = std::vector<std::pair<float, float>>; // (t [s], value): piecewise linear, held outside

static float trackAt(const Track &tr, double t, float dflt) {
    if (tr.empty()) return dflt;
    if (t <= tr.front().first) return tr.front().second;
    if (t >= tr.back().first) return tr.back().second;
    size_t i = 1;
    while (tr[i].first < t) ++i;
    return tr[i - 1].second + (tr[i].second - tr[i - 1].second) * ((float) t - tr[i - 1].first) / (tr[i].first - tr[i - 1].first);
}

// flat, 2026-11-14 11:20:00..11:35:00 UTC: P / 776 W (1 s means, reduced to 2 % max error)
static const Track kCloud1120 = {
    {0, 0.948f}, {10, 0.943f}, {12, 0.890f}, {14, 0.922f}, {16, 0.630f}, {17, 0.540f}, {18, 0.521f}, {20, 0.387f},
    {25, 0.285f}, {36, 0.249f}, {42, 0.248f}, {48, 0.294f}, {49, 0.328f}, {50, 0.452f}, {52, 0.571f}, {54, 0.784f},
    {56, 0.703f}, {57, 0.712f}, {59, 0.953f}, {61, 0.946f}, {63, 0.834f}, {64, 0.894f}, {65, 0.892f}, {66, 0.815f},
    {67, 0.794f}, {68, 0.889f}, {69, 0.768f}, {70, 0.563f}, {73, 0.522f}, {75, 0.441f}, {77, 0.505f}, {79, 0.504f},
    {81, 0.435f}, {85, 0.397f}, {89, 0.438f}, {91, 0.539f}, {93, 0.762f}, {94, 0.807f}, {95, 0.911f}, {96, 0.940f},
    {101, 0.892f}, {110, 0.901f}, {113, 0.870f}, {114, 0.902f}, {119, 0.883f}, {187, 0.924f}, {189, 0.917f}, {191,
    0.807f}, {192, 0.799f}, {194, 0.880f}, {196, 0.814f}, {197, 0.863f}, {198, 0.826f}, {205, 0.932f}, {211,
    0.955f}, {212, 0.940f}, {213, 0.876f}, {215, 0.892f}, {217, 0.652f}, {218, 0.615f}, {222, 0.369f}, {223,
    0.337f}, {225, 0.392f}, {226, 0.346f}, {228, 0.461f}, {229, 0.354f}, {230, 0.328f}, {232, 0.365f}, {233,
    0.423f}, {234, 0.513f}, {235, 0.659f}, {236, 0.669f}, {237, 0.557f}, {240, 0.521f}, {241, 0.549f}, {243,
    0.879f}, {244, 0.974f}, {251, 0.947f}, {253, 0.900f}, {254, 0.932f}, {258, 0.940f}, {293, 0.927f}, {296,
    0.909f}, {298, 0.830f}, {299, 0.830f}, {301, 0.890f}, {303, 0.904f}, {304, 0.877f}, {306, 0.920f}, {314,
    0.892f}, {315, 0.916f}, {319, 0.909f}, {323, 0.869f}, {324, 0.907f}, {329, 0.913f}, {332, 0.877f}, {336,
    0.937f}, {342, 0.920f}, {347, 0.942f}, {349, 0.888f}, {351, 0.926f}, {353, 0.817f}, {355, 0.883f}, {356,
    0.850f}, {357, 0.740f}, {358, 0.672f}, {360, 0.688f}, {362, 0.574f}, {363, 0.558f}, {364, 0.669f}, {367,
    0.569f}, {369, 0.577f}, {370, 0.548f}, {372, 0.419f}, {375, 0.344f}, {384, 0.302f}, {389, 0.340f}, {390,
    0.380f}, {391, 0.465f}, {393, 0.548f}, {396, 0.524f}, {397, 0.607f}, {398, 0.801f}, {399, 0.937f}, {401,
    0.982f}, {406, 0.979f}, {408, 0.948f}, {415, 0.969f}, {418, 0.920f}, {420, 0.971f}, {429, 0.947f}, {431,
    0.983f}, {438, 0.991f}, {441, 0.978f}, {442, 0.939f}, {443, 0.978f}, {447, 0.989f}, {450, 0.956f}, {451,
    0.983f}, {457, 0.983f}, {458, 0.944f}, {463, 0.987f}, {468, 0.942f}, {471, 0.999f}, {472, 0.968f}, {474,
    1.000f}, {478, 0.985f}, {480, 0.932f}, {482, 0.991f}, {483, 0.956f}, {486, 0.684f}, {488, 0.570f}, {490,
    0.572f}, {491, 0.640f}, {492, 0.628f}, {495, 0.446f}, {496, 0.463f}, {501, 0.382f}, {508, 0.399f}, {512,
    0.502f}, {513, 0.497f}, {514, 0.553f}, {516, 0.828f}, {518, 0.963f}, {519, 0.963f}, {521, 0.890f}, {522,
    0.964f}, {523, 0.972f}, {525, 0.864f}, {526, 0.662f}, {527, 0.531f}, {528, 0.550f}, {530, 0.688f}, {532,
    0.661f}, {533, 0.737f}, {534, 0.904f}, {535, 0.962f}, {536, 0.956f}, {537, 0.926f}, {538, 0.809f}, {539,
    0.782f}, {540, 0.839f}, {542, 0.810f}, {544, 0.626f}, {545, 0.606f}, {547, 0.830f}, {548, 0.842f}, {549,
    0.662f}, {550, 0.432f}, {551, 0.379f}, {563, 0.311f}, {569, 0.325f}, {570, 0.350f}, {572, 0.480f}, {573,
    0.647f}, {574, 0.738f}, {576, 0.645f}, {578, 0.617f}, {586, 0.722f}, {587, 0.760f}, {589, 0.734f}, {590,
    0.812f}, {591, 0.799f}, {594, 0.533f}, {597, 0.378f}, {602, 0.272f}, {606, 0.239f}, {620, 0.247f}, {630,
    0.302f}, {635, 0.368f}, {638, 0.455f}, {642, 0.469f}, {646, 0.284f}, {656, 0.229f}, {808, 0.111f}, {899, 0.116f}
};

struct Wx {
    float voc = 70, k = 0.78f, pClear = 776; // clear-sky string: Voc, Vmpp/Voc, MPP power
    uint8_t n = 1;                           // series substrings
    float w[3] = {1, 0, 0};                  // their Voc shares
    Track cloud, sub[3];                     // irradiance factor of the whole string / per substring
    bool beamShade = false;                  // shade blocks the beam only: substring gets min(cloud, sub)
    float vocLog = 0;                        // Voc(t) = voc + vocLog * ln(irradiance)
    float vbat = 26.6f, rbat = 0.036f;
    Track vbatT;                             // battery EMF profile
    double sweepAt = -1, statsFrom = 0, T = 600;
    float isc = 0, g[3] = {1, 1, 1}, vocNow = 0;

    void init() {
        PvModel m;
        m.set(1, voc, k);
        float best = 0;
        for (float v = 1; v < voc; v += 0.05f) best = std::max(best, v * m.current(v));
        isc = pClear / best;
        vocNow = voc;
        g_vconv.setPv(isc, voc, k);
        g_vconv.setBat(vbat, rbat);
    }

    void apply(double t) {
        const float c = trackAt(cloud, t, 1);
        float gMax = 0;
        for (uint8_t i = 0; i < n; ++i) {
            const float s = trackAt(sub[i], t, 1);
            g[i] = beamShade ? std::min(c, s) : c * s;
            gMax = std::max(gMax, g[i]);
        }
        g_vconv.setPvString(n, g, n > 1 ? w : nullptr);
        if (vocLog > 0) {
            const float v = std::max(1.f, voc + vocLog * std::log(std::max(gMax, 1e-30f)));
            if (std::fabs(v - vocNow) > 1e-3f) g_vconv.setPv(isc, vocNow = v, k);
        }
        if (!vbatT.empty()) g_vconv.setBat(trackAt(vbatT, t, vbat), rbat);
    }
};

struct Peak {
    float v, p;
};

// local maxima of the plant's P(V) in ascending V; valley[i] separates peak i and i+1
static int pvPeaks(Peak pk[4], float valley[4]) {
    int n = 0;
    float p0 = 0;
    bool rising = true;
    for (float v = 1; v < g_vconv.getVoc() + 0.2f && n < 4; v += 0.1f) {
        const float p = v * g_vconv.pvCurrent(v);
        if (rising && p < p0) pk[n++] = {v - 0.1f, p0}, rising = false;
        else if (!rising && p > p0) valley[n - 1] = v - 0.1f, rising = true;
        p0 = p;
    }
    return n;
}

static int runWx(const std::string &name, Wx &wx) {
    if (const char *e = getenv("SIM_T")) wx.T = atof(e);
    g_verbose = getenv("SIM_V") != nullptr;
    const double trEvery = getenv("SIM_TR") ? atof(getenv("SIM_TR")) : wx.T / 60;
    wx.init();
    wx.apply(0);
    printf("# %s: clear-sky MPP %.0f W, Isc %.2f A, Voc %.0f V, %u substring(s), bat %.2f V / %.3f ohm, %.0f s\n",
           name.c_str(), wx.pClear, wx.isc, wx.voc, (unsigned) wx.n, wx.vbat, wx.rbat, wx.T);
    printf("#        t  g(substrings)     | PLANT Vin   Ppv | GMPP V     P  peaks | CTRL state D lim\n");

    enum { OFF, SWEEP, LIMIT, WRONG, TRACK, NCAT };
    double eG = 0, eUse = 0, ePv = 0, eOut = 0, eCtl = 0, tCat[NCAT] = {}, loss[NCAT] = {}, tStat = 0;
    struct Span { // vOp..pG: at the largest gap
        double t0, t1, lost;
        float vOp, pOp, vG, pG;
    };
    std::vector<Span> wrong;
    bool inWrong = false;
    Peak pk[4];
    float valley[4], key[4] = {-1, -1, -1, -1};
    int nPk = 0, maxPk = 0;

    for (double nextTr = 0; tNow() < wx.T;) {
        const double tb = tNow();
        wx.apply(tb);
        if (wx.sweepAt >= 0 && tb >= wx.sweepAt) {
            printf("%9.2f >>> sweep requested\n", tb);
            mppt.startSweep();
            wx.sweepAt = -1;
        }
        run(0.02, 0);
        const double dt = tNow() - tb;

        const float now[4] = {wx.g[0], wx.g[1], wx.g[2], wx.vocNow};
        if (memcmp(now, key, sizeof key)) nPk = pvPeaks(pk, valley), memcpy(key, now, sizeof key);
        // reference: highest peak a buck can reach (above the output voltage)
        const float vReach = g_vconv.getVout() * 1.05f, vin = g_vconv.getVin();
        int iG = -1, iOp = 0, nReach = 0;
        for (int i = 0; i < nPk; ++i) {
            if (pk[i].v < vReach) continue;
            ++nReach;
            if (iG < 0 || pk[i].p > pk[iG].p) iG = i;
        }
        while (iOp + 1 < nPk && vin > valley[iOp]) ++iOp;
        const float pG = iG < 0 ? 0 : pk[iG].p, pPv = vin * g_vconv.pvCurrent(vin);
        const bool on = !converter.disabled();
        const bool sweeping = mppt.getState() == MpptControlMode::Sweep;
        const bool isWrong = on && !sweeping && iG >= 0 && iOp != iG && pG > pk[iOp].p * 1.02f;

        if (tb >= nextTr) {
            printf("%9.2f  %.3f %.3f %.3f | %5.1f %6.1f | %5.1f %6.1f  %d(%d) | %-11s %4u %2u%s\n", tNow(), wx.g[0],
                   wx.n > 1 ? wx.g[1] : 0.f, wx.n > 2 ? wx.g[2] : 0.f, vin, pPv, iG < 0 ? 0.f : pk[iG].v, pG, nPk, nReach,
                   modeStr(), converter.getCtrlOnPwmCnt(), (unsigned) mppt.teleSnap().limIdx, isWrong ? "  WRONG PEAK" : "");
            nextTr += trEvery;
        }
        if (tb < wx.statsFrom) continue;

        maxPk = std::max(maxPk, nReach);
        tStat += dt;
        eG += pG * dt;
        // what the battery takes at the controller's Vout cap
        const float vCap = mppt.charger.Vout_max();
        eUse += std::min(pG, std::max(0.f, vCap * (vCap - g_vconv.getVbat()) / g_vconv.getRbat())) * dt;
        eOut += g_vconv.getVout() * g_vconv.getIoutAvg() * dt;
        ePv += pPv * dt;
        const float pCtl = sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get();
        if (std::isfinite(pCtl)) eCtl += pCtl * dt;
        const int cat = !on ? OFF : sweeping ? SWEEP
                        : mppt.teleSnap().limIdx != 15 ? LIMIT : isWrong ? WRONG : TRACK;
        tCat[cat] += dt;
        loss[cat] += (pG - pPv) * dt;
        if (isWrong) {
            if (!inWrong) wrong.push_back({tb, 0, 0, 0, 0, 0, 0});
            auto &w = wrong.back();
            if (w.pOp <= 0 || pG * w.pOp > w.pG * pk[iOp].p) w.vOp = pk[iOp].v, w.pOp = pk[iOp].p, w.vG = pk[iG].v, w.pG = pG;
            wrong.back().t1 = tNow();
            wrong.back().lost += (pG - pPv) * dt;
        }
        inWrong = isWrong;
    }

    std::string trips;
    size_t nTrips = 0;
    {
        std::vector<std::pair<std::string, int>> byName;
        for (auto &t: g_trips) {
            if (t.t < wx.statsFrom) continue;
            ++nTrips;
            auto it = std::find_if(byName.begin(), byName.end(), [&](auto &p) { return p.first == t.who; });
            if (it == byName.end()) byName.emplace_back(t.who, 1);
            else ++it->second;
        }
        for (auto &[who, cnt]: byName) trips += (trips.empty() ? "" : ", ") + who + " x" + std::to_string(cnt);
    }
    size_t nSweeps = 0;
    for (double t: g_sweepT) nSweeps += t >= wx.statsFrom;
    double tWrong = 0;
    for (auto &s: wrong) tWrong += s.t1 - s.t0;
    const double pc = eG > 0 ? 100 / eG : 0;
    const char *hz = getenv("SIM_ADC_HZ");
    printf("\n== %s @%s Hz, %.0f s, up to %d reachable peak(s)\n", name.c_str(), hz ? hz : "450", tStat, maxPk);
    printf("== PLANT TRUTH: PV energy %.2f Wh = %.1f %% of the reachable GMPP (%.2f Wh), %.1f %% of what the battery"
           " takes below the Vout cap (%.2f Wh); lost %% of GMPP while off %.1f / sweep %.1f / limiter %.1f /"
           " wrong peak %.1f / tracking %.1f; on a non-global peak %.1f s, off %.1f s; output energy %.2f Wh\n",
           ePv / 3600, ePv * pc, eG / 3600, eUse > 0 ? 100 * ePv / eUse : 0, eUse / 3600, loss[OFF] * pc, loss[SWEEP] * pc,
           loss[LIMIT] * pc, loss[WRONG] * pc, loss[TRACK] * pc, tWrong, tCat[OFF], eOut / 3600);
    printf("== CONTROLLER: %zu sweeps, limiter active %.1f %% of the time, sweeping %.1f %%, %zu trips [%s],"
           " measured output energy %.2f Wh\n", nSweeps, 100 * tCat[LIMIT] / tStat, 100 * tCat[SWEEP] / tStat, nTrips,
           trips.c_str(), eCtl / 3600);
    // plant truth: harvest, wrong-peak, off; controller: sweeps, limiter, trips
    printf("== SUMMARY %-18s %5s Hz | harvest %5.1f %% of GMPP, %5.1f %% of cap-limited | wrong-peak %6.1f s | off %6.1f s |"
           " sweeps %2zu | limiter %4.1f %% | trips %zu [%s]\n", name.c_str(), hz ? hz : "450", ePv * pc,
           eUse > 0 ? 100 * ePv / eUse : 0, tWrong, tCat[OFF], nSweeps, 100 * tCat[LIMIT] / tStat, nTrips, trips.c_str());
    size_t shown = 0;
    for (auto &s: wrong) {
        if (s.t1 - s.t0 < 2 || ++shown > 12) continue;
        printf("==   wrong peak t=%.1f..%.1f s: on the %.1f V peak (%.1f W) while the %.1f V peak gives %.1f W (up to +%.0f %%),"
               " %.2f Wh lost\n", s.t0, s.t1, s.vOp, s.pOp, s.vG, s.pG, 100 * (s.pG / s.pOp - 1), s.lost / 3600);
    }
    shown = 0;
    for (double t: g_sweepT) if (t >= wx.statsFrom && ++shown <= 12) printf("==   sweep at t=%.1f s\n", t);
    shown = 0;
    for (auto &t: g_trips)
        if (t.t >= wx.statsFrom && ++shown <= 12) printf("==   trip at t=%.1f s [%s] %us\n", t.t, t.who.c_str(), t.sec);
    if (nTrips > 12) printf("==   ... %zu trips total\n", nTrips);
    return 0;
}

// wx-* scenario table; false = unknown name
static bool wxScenario(const std::string &sc, Wx &wx) {
    // fry-like string: Voc 80 V, peaks near 68 V (all), 57 V and 30 V (substrings bypassed)
    auto fry = [&](float pClear) { wx.voc = 80, wx.k = 0.86f, wx.pClear = pClear; };
    if (sc == "wx-cloud") {
        wx.cloud = kCloud1120, wx.T = 900;
    } else if (sc == "wx-cloud-cv" || sc == "wx-cloud-cv-ripple") {
        // battery high enough that full sun runs into the Vout cap and every cloud releases it
        wx.cloud = kCloud1120, wx.T = 900, wx.vbat = 28.2f;
    } else if (sc == "wx-shade2" || sc == "wx-shade2-move") {
        // 57 V peak 19 % above the 68 V peak (fry, mornings); -move: starts unshaded
        fry(215), wx.n = 2, wx.w[0] = 0.79f, wx.w[1] = 0.21f;
        if (sc == "wx-shade2") wx.sub[1] = {{0, 0.664f}};
        else wx.sub[1] = {{120, 1}, {420, 0.664f}}, wx.T = 2400;
    } else if (sc == "wx-shade3" || sc == "wx-shade3-move") {
        fry(300), wx.n = 3, wx.w[0] = 0.44f, wx.w[1] = 0.40f, wx.w[2] = 0.16f;
        if (sc == "wx-shade3") {
            wx.sub[1] = {{0, 0.75f}}, wx.sub[2] = {{0, 0.5f}}; // global = middle peak
        } else {
            // global 30 V peak -> 68 V peak 64 % above it
            wx.sub[1] = wx.sub[2] = {{120, 0.3f}, {420, 0.72f}}, wx.T = 2400;
        }
    } else if (sc == "wx-shade-cloud") {
        // beam shade on one substring under the cloud series: every cloud removes the second peak
        fry(615), wx.n = 2, wx.w[0] = 0.79f, wx.w[1] = 0.21f;
        wx.cloud = kCloud1120, wx.sub[1] = {{0, 0.664f}}, wx.beamShade = true, wx.T = 900;
    } else if (sc == "wx-dawn" || sc == "wx-dusk") {
        // irradiance through the start threshold, doubling every 350 s (flat 2026-11-14 06:59..07:34 UTC)
        // ln(irradiance): below the start threshold, 500 s up to a 1.8 W MPP, then doubling
        wx.vocLog = 2.6f, wx.T = 2200;
        for (float t = 0; t <= wx.T; t += 10) {
            const float lg = t < 120 ? -17.7f + t / 120 * 5.7f : t < 620 ? -12 + (t - 120) / 500 * 6.2f
                                                                 : -5.8f + (t - 620) / 350 * 0.6931f;
            wx.cloud.emplace_back(sc == "wx-dawn" ? t : (float) wx.T - t, std::exp(lg));
        }
        if (sc == "wx-dusk") std::reverse(wx.cloud.begin(), wx.cloud.end());
    } else if (sc == "wx-sweep-fall" || sc == "wx-sweep-rise") {
        // cloud edge (73 % in 10 s) starting 2 s into a sweep
        const bool fall = sc == "wx-sweep-fall";
        wx.cloud = {{62, fall ? 1 : 0.27f}, {72, fall ? 0.27f : 1}};
        wx.sweepAt = wx.statsFrom = 60, wx.T = 240;
    } else if (sc == "wx-sweep-shade") {
        // two peaks, the 68 V one 10 % higher; a rising cloud edge during the sweep favours the later 54 V one
        fry(615), wx.n = 2, wx.w[0] = 0.79f, wx.w[1] = 0.21f, wx.sub[1] = {{0, 0.87f}};
        wx.cloud = {{62, 0.27f}, {72, 1}};
        wx.sweepAt = wx.statsFrom = 60, wx.T = 660;
    } else {
        return false;
    }
    return true;
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
    if (sc.rfind("shade", 0) == 0) {
        ov = {{"isc", "6.5"}, {"voc", "78"}};
        // shade-cv: battery near the Vout cap with inverter ripple, so the Vout limiter touches now and then
        if (sc == "shade-cv") {
            ov.insert(ov.end(), {{"noise_vout", "0.03"}, {"vbat_ac_amp", "0.26"}, {"vbat_ac_freq", "100"}});
        }
    }
    Wx wx;
    const bool isWx = sc.rfind("wx-", 0) == 0;
    if (isWx && !wxScenario(sc, wx)) {
        printf("unknown scenario %s\n", sc.c_str());
        return 1;
    }
    if (sc == "wx-cloud-cv-ripple") ov = {{"noise_vout", "0.03"}, {"vbat_ac_amp", "0.26"}, {"vbat_ac_freq", "100"}};
    boot(ov, p0);
    if (isWx) return runWx(name, wx);
    if (sc.rfind("shade", 0) == 0) g_vconv.setPvShade(0.68f);

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
    } else if (sc.rfind("shade", 0) == 0) {
        // two-peak curve; report where the tracker sits and the power against the higher peak
        float pk[2] = {0, 0}, vk[2] = {0, 0};
        for (float v = 5; v < g_vconv.getVoc(); v += 0.1f) {
            int k = v > g_vconv.getVoc() * 0.6f;
            if (v * g_vconv.pvCurrent(v) > pk[k]) pk[k] = v * g_vconv.pvCurrent(v), vk[k] = v;
        }
        printf("== peaks: %.0f W at %.1f V, %.0f W at %.1f V\n", pk[0], vk[0], pk[1], vk[1]);
        const double T = getenv("SIM_T") ? atof(getenv("SIM_T")) : 600;
        double eW = 0, tLow = 0, t0 = tNow(), tLeft = -1;
        uint32_t nLim = 0, n = 0;
        g_verbose = getenv("SIM_V") != nullptr;
        for (double tEnd = tNow() + T, nextTr = tNow(); tNow() < tEnd;) {
            double tb = tNow();
            run(0.02, 0);
            double dt = tNow() - tb;
            eW += g_vconv.getVin() * g_vconv.pvCurrent(g_vconv.getVin()) * dt;
            if (g_vconv.getVin() < g_vconv.getVoc() * 0.6f) { tLow += dt; if (tLeft < 0) tLeft = tNow() - t0; }
            ++n; if (mppt.teleSnap().limIdx != 15) ++nLim;
            if (tNow() >= nextTr) { traceLine(); nextTr += T / 40; }
        }
        float best = std::max(pk[0], pk[1]);
        printf("\n== %s: mean PV power %.1f W = %.1f %% of the higher peak; %.0f s of %.0f s on the low-voltage side"
               " (first at +%.0f s); limiter active in %.1f %% of 20 ms slices; %zu trips\n", name.c_str(), eW / T,
               100 * eW / T / best, tLow, T, tLeft, 100.0 * nLim / n, g_trips.size());
        return 0;
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
