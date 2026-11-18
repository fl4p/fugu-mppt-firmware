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
//                              # idle-wake: start refused only by InhibitIdle while Vin rises (armed sweep and not)
//   weather scenarios (PV string of 1..3 substrings, irradiance played from a time series; "== SUMMARY" line):
//     wx-cloud          recorded cloudy quarter hour (flat, 2026-11-14 11:20 UTC), full sun <-> cloud
//     wx-cloud-cv       same with the battery at the Vout cap in full sun; -ripple adds 100 Hz bus ripple
//     wx-shade2, wx-shade3            static partial shade on a 6-substring string (2 modules x 3 bypass diodes)
//     wx-shade2-move, wx-shade3-move  shade moves, the global maximum changes peak after the boot sweep
//     wx-shade-walk                   a shadow walks across the substrings, the global peak moves several times
//     wx-shade-cloud    shaded substring under the cloud series (second peak comes and goes)
//     wx-shade-learn    wx-shade2 shade for two sweeps (learns the substrings), clear at 1900 s, back at 2000..2300 s
//     wx-dawn, wx-dusk  irradiance through the start threshold
//     wx-low, wx-mid    steady light rising slowly, MPP 2 -> 20 W in 30 min / 55 -> 110 W in 10 min, sensor noise
//     wx-low-slow       wx-low with power-sensor filter length 60 (slow sensor profile; SIM_ADC_HZ=200 for ADS-like rates)
//     wx-sweep-fall, wx-sweep-rise, wx-sweep-shade   cloud edge during a sweep (-shade: two peaks)
//     wx-mp-ramp, wx-mp-psu, wx-mp-psu-nosink  source Voc slowly 1.25 -> 1.02 -> 1.25 x Vbat (LS min pulse ratio
//                       limit): weak PV / stiff sinking source into a full battery / non-sinking bench PSU; the
//                       == REVERSE line has the plant's reverse output current and the starts
//     wx-mp-manual      the -psu source with manual duty 0.8 (stop, then the manual restart waits for the start ratio)
//     wx-mp-ccm         stiff source into a charging battery, Voc 1.3 -> 1.02 x Vbat: CCM until the duty limit, then DCM
//     wx-sun-fry, wx-sun-flat   steady sun, MPP SIM_P W (default 70), the board's L, string and Vin (fry ~64 V, flat ~55 V)
//     wx-ramp-fry, wx-ramp-flat wx-sun with irradiance changing SIM_RAMP %/min (default 2, exponential) from 120 s
//     wx-wob-fry, wx-wob-flat   wx-sun with irradiance x (1 + a sin 2 pi f t), SIM_WOB=a,f (default 0.005,0.2)
//   SIM_NOISE=fry|flat|<file.fnz>  replay sensor noise measured on a converter (test/host-stub/noise/, made by
//     noise_profile.py; looped): residuals added to Vin/Vout/Iout, Iout's scaled by Iout/Iout(capture)^SIM_NOISE_EXP (default 1),
//     x SIM_NOISE_GAIN, starting SIM_NOISE_OFS s into the profile; INA226 LSB quantisation, the boards' filt_len 20, SIM_ADC_HZ defaults to the capture rate
//     fry-wander, flat-wander, fry3-wander: Iout keeps its slow wander below 0.3 Hz (noise_profile.py --keep-slow;
//     fry3: a capture with 3 % p-p at ~0.2 Hz). The == TRACKER line ends with the tracking loss in J, the part in slow
//     mode and the largest Vin offset from the MPP in slow mode
//   SIM_T=600 wx run length [s], SIM_TR=5 wx trace interval [s], SIM_V=1 firmware log in wx/shade runs
//   SIM_FINE=t0,t1 /tmp/mppt-sim c              # per-tick sensor trace between t0 and t1 [s]
//   SIM_PDUMP=<file>  per RT tick float32 t, Iout ewm, Vout ewm, HS count, slow mode, PV power; SIM_HOLD=t: wx runs hold the duty from t [s]
//   SIM_ADC_HZ=3000 /tmp/mppt-sim a             # ADC tick rate [Hz], default 450
//   SIM_PHASE_US=500 /tmp/mppt-sim a          # a/b/c/e: plant runs n us past the event before the next ADC tick
//   SIM_CONF="coil.conf:L0=80e-6,vconv.conf:c_out=470e-6" /tmp/mppt-sim a   # board overrides
//   rect-cal (rectCalRt/rectCalLf wired as in main.cpp; the automatic start is off, SIM_RCAL_AUTO=1 enables it):
//     rcal         `rect-cal force` at full power (CCM), LS body-diode drop 0.7 V, sensor noise
//     rcal-bus     same, battery +2.5 % linear over 30 s from the start (shared bus)
//     rcal-cloud   same, Isc x0.7 and Voc -1.8 % step 12 s after the start
//     rcal-ov      same, Vin trip (hv_max) 2 % above Voc: the run must stop short of it
//     rcal-low     same, a cloud 0.2 s after the start: Isc 2 A, Voc -3 % (soft panel, knee lifts Vin)
//     rcal-sun     80 uH board, Isc 5.6 A / Voc 79.6 V, 5 nF switch node ringing at Q 40 (non-monotonic late side)
//     rcal-flat    flat (51 uH, 326 kHz ring, 1470 sps), ~390 W at Vin ~55 V, Vout +0.6 V for 2 ms every 1-3 s:
//                  D-only Vout limiter touches below the cap must not abort the run
//     rcal-pv      plain rect-cal force on the SIM_RING board plant (default fry), panel SIM_PV="isc,voc,k" (default
//                  5.6,79.6,0.78): k moves the start Vin, isc the power, voc the calibration duty
//     rcal*: SIM_NOISE_IOUT=<A> Iout noise sigma (default 0.01); the plant's zero crossing at the calibration point
//     ("truth": LS count where the coil current at LS turn-off crosses zero, stiff sources at the run's mean plant
//     Vin/Vout, so dead time, diode drop and the ring current at HS turn-on are in it) is printed against ideal
//     rcal-low-bus rcal-low on a swinging bus (+-0.4 % / 23 s, +-0.2 % / 7.3 s, 0.15 % steps every 2-6 s)
//   switch-node ring (DCM Iout ripple vs duty; vconv.conf ring_c/ring_q), board = fry | flat (L0, L, ring f, Q
//   fitted to field sweeps), suffix -off = same board with the ring disabled, -damped = ring at Q 0.5 (no
//   oscillation, but the ring branch: node charge, and the HS body diode returns clamp charge to the input):
//     ring-sweep-<board>   ~150-180 W morning sun, global sweep at 60 s: ripple seen in the sweep, chosen peak
//     ring-weak-<board>    steady ~45 W, the whole operating range in DCM: does P&O sit on a ripple maximum
//     ring-coil-<board>    `measure-coil l0` flow (old 10-step median, new 48-step ring fit) against the plant,
//                          then the MPPT restore; -weak: at ~40 W (soft source)
//   SIM_RING=fry|flat|fry-hq|fry-off|flat-damped|... applies a board plant (test/host-stub/plant_boards.h: ring, diode
//     drop, dead time, conduction and switching loss, fitted to the LTspice bench) to any other scenario; wx-* default
//     to fry, SIM_RING=ideal = the lossless vconv_mock plant without ring. -hq: Q of the undamped bench (more ripple)
//   SIM_PLANT="fry:ring_q=40,..." overrides board fields
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
#include <random>
#include <unistd.h>

#include "adc/sampling.h"
#include "adc/sensor_setup.h"
#include "buck.h"
#include "mppt.h"
#include "app_state.h"
#include "sim/vconv.h"
#include "sim_alloc.h"
#include "selftest/coil_fit.h"
#include "plant_boards.h"

#if __has_include("peak_scan.h")
#define SCAN_ACTIVE() mppt.scan.active()
#define SCAN_ACTIVE_DEFINED 1
#else
#define SCAN_ACTIVE() false // a tree without the partial scan
#define SCAN_ACTIVE_DEFINED 0
#endif

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
static std::vector<double> g_slowDownT; // "Slow mode, duty" log times (slow-mode entries)
static std::vector<std::pair<double, char>> g_scanT; // "scan:" log times; m = moved, l = light changed, - = stayed
static double g_fineT0 = 0, g_fineT1 = 0; // env SIM_FINE=t0,t1: per-tick sensor trace window
static double g_rampSec = 0, g_rampT0 = 0, g_rampFrom = 0, g_rampTo = 0; // linear Isc ramp
static bool g_verbose = true;
static double g_qOut = 0; // plant output charge [C], per RT iteration (cycle-average Iout x dt)
static unsigned g_nStarts = 0; // converter off -> on edges

// ---- board plant (ring-* scenarios, SIM_RING; wx-* default to fry) ----
static PlantBoard g_board;
static const PlantBoard *g_ring = nullptr;
static bool g_ringOn = false, g_ringDamped = false; // damped: Q 0.5, ring branch without oscillation
static void (*g_tickHook)() = nullptr; // called after every RT iteration of run()
static FILE *g_pDump = nullptr; // SIM_PDUMP=<file>: per tick t, Iout ewm, Vout ewm, HS count (float32)

static bool cutSuffix(std::string &t, const char *sfx) {
    const size_t n = strlen(sfx);
    if (t.size() <= n || t.compare(t.size() - n, n, sfx)) return false;
    t.resize(t.size() - n);
    return true;
}

// "...-fry[-hq][-off]" / "...-flat[-damped]" or SIM_RING: pick the board; returns the name without the suffix
static std::string ringBoardFrom(std::string sc) {
    plantBoardsFromEnv();
    const char *env = getenv("SIM_RING");
    std::string tag = sc;
    if (sc.rfind("ring-", 0) != 0) {
        if (!env || !strcmp(env, "ideal")) return sc;
        tag = std::string("x-") + env;
    }
    g_ringOn = !cutSuffix(tag, "-off");
    g_ringDamped = cutSuffix(tag, "-damped");
    const bool hq = cutSuffix(tag, "-hq");
    for (auto &b: kPlantBoards) {
        const std::string sfx = std::string("-") + b.name;
        if (cutSuffix(tag, sfx.c_str())) {
            g_board = hq ? highQ(b) : b;
            if (g_ringDamped) g_board.ringQ = 0.5f;
            g_ring = &g_board;
            if (sc.rfind("ring-", 0) == 0) sc = tag;
            return sc;
        }
    }
    printf("ring board? %s\n", tag.c_str());
    exit(1);
}

static double tNow() { return (double) esp_timer_get_time() * 1e-6; }

// ---- measured sensor noise (SIM_NOISE) ----
float (*g_simSenseHook)(uint8_t ch, float x) = nullptr;
static struct {
    float fs[3], lsb[3], iRef = 1, gain = 1, exp = 1;
    uint64_t ofs = 0; // ticks
    std::vector<int16_t> v[3]; // vin, vout, iout (ADC_VConv channels 0..2)
} g_noise;

static float noiseHook(uint8_t ch, float x) {
    if (ch > 2) return x;
    const auto &v = g_noise.v[ch];
    // by ADC tick: a channel at the tick rate replays sample for sample (a skipped or repeated sample is a
    // phase step in the ripple, which the notch passes)
    const uint64_t tick = esp_timer_get_time() / g_simTickUs + g_noise.ofs;
    const float k = g_noise.fs[ch] * g_simTickUs * 1e-6f;
    const size_t i = (size_t) (std::fabs(k - 1) < 0.01f ? tick : (uint64_t) ((double) tick * k)) % v.size();
    float r = (float) v[i] * g_noise.lsb[ch] * g_noise.gain;
    if (ch == 0) return x + r;
    if (ch == 2) r *= std::pow(std::max(x, 0.f) / g_noise.iRef, g_noise.exp);
    const float q = ch == 1 ? 0.0025f : 35.f / 32768; // INA226 bus (x2 divider) and current LSB
    return std::round((x + r) / q) * q;
}

static void noiseLoad(const char *name) {
    std::string p = name;
    if (p.find('/') == std::string::npos) p = std::string(SIM_REPO_ROOT) + "/test/host-stub/noise/" + p + ".fnz";
    FILE *f = ::fopen(p.c_str(), "rb");
    char magic[4];
    if (!f || fread(magic, 1, 4, f) != 4 || memcmp(magic, "FNZ1", 4) || fread(&g_noise.iRef, 4, 1, f) != 1) {
        printf("SIM_NOISE: cannot read %s\n", p.c_str());
        exit(1);
    }
    for (int c = 0; c < 3; ++c) {
        uint32_t n = 0;
        if (fread(&g_noise.fs[c], 4, 1, f) != 1 || fread(&g_noise.lsb[c], 4, 1, f) != 1 || fread(&n, 4, 1, f) != 1 || !n) exit(1);
        g_noise.v[c].resize(n);
        if (fread(g_noise.v[c].data(), 2, n, f) != n) exit(1);
    }
    fclose(f);
    if (const char *e = getenv("SIM_NOISE_GAIN")) g_noise.gain = atof(e);
    if (const char *e = getenv("SIM_NOISE_EXP")) g_noise.exp = atof(e);
    if (const char *e = getenv("SIM_NOISE_OFS")) g_noise.ofs = (uint64_t) (atof(e) * g_noise.fs[2]);
    if (!getenv("SIM_ADC_HZ")) setenv("SIM_ADC_HZ", std::to_string(std::lround(g_noise.fs[2])).c_str(), 1);
    g_simSenseHook = noiseHook;
    printf("# noise %s: %.0f s at %.0f Hz, Iout ref %.2f A, gain %.2f, exp %.2f\n", p.c_str(),
           g_noise.v[2].size() / g_noise.fs[2], g_noise.fs[2], g_noise.iRef, g_noise.gain, g_noise.exp);
}

static void logHook(char lvl, const char *tag, const char *msg) {
    // backoff log of MpptController::shutdownDcdc: "backoff %lus [%s]..."
    unsigned sec;
    char who[64];
    if (sscanf(msg, "backoff %us [%63[^]]]", &sec, who) == 2) g_trips.push_back({tNow(), who, sec});
    if (!strcmp(msg, "Start sweep")) g_sweepT.push_back(tNow());
    if (!strncmp(msg, "Slow mode, duty", 15)) g_slowDownT.push_back(tNow());
    if (!strncmp(msg, "scan:", 5))
        g_scanT.emplace_back(tNow(), strstr(msg, "moving") ? 'm' : strstr(msg, "light changed") ? 'l' : '-');
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
    if (haveNewSample) {
        timeLastSampler = wallClockUs();
        mppt.rectCalRt();
    }

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
        } else if (wallClockUs() > delayStartUntil && !mppt.startBlockReasonRt()) {
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
static void loopRTIterBody() {
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

// SIM_NOALLOC=<s>: heap use inside the RT iteration after <s> simulated seconds fails the run
static double g_noAllocAfter = -1;

static void loopRTIter() {
    g_simRtAlloc.armed = g_noAllocAfter >= 0 && tNow() >= g_noAllocAfter;
    g_simRtAlloc.active = true;
    loopRTIterBody();
    g_simRtAlloc.active = false;
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
    mppt.rectCalLf();
    mppt.scanLf(false); // flash writes not simulated; saved and reloaded at the end of the run
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
    if (g_simSenseHook)
        writeConf(confDir, "sensor.conf", {{"hv_v_filt_len", "20"}, {"lv_v_filt_len", "20"}, {"lv_i_filt_len", "20"}, {"hv_i_filt_len", "20"}});
    if (g_ring) writeConf(confDir, "vconv.conf", g_ring->conf(g_ringOn)); // ring_c is at the plant L, set after boot
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
    if (g_ring) {
        char l0[24];
        snprintf(l0, sizeof l0, "%g", g_ring->l0Conf);
        writeConf(confDir, "coil.conf", {{"L0", l0}});
    }

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
    mppt.rectCal.autoRun = getenv("SIM_RCAL_AUTO") && coilConf.getByte("rect_cal_auto", 1);
    mppt.rectCal.persist = rectCalPersist;
    mppt.rectCal.init(converter.getRectOnOffset(), !converter.rectOffsetNeedsCal());
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
        const double tIt = tNow();
        loopRTIter();
        g_qOut += g_vconv.getIoutAvg() * (tNow() - tIt);
        if (g_tickHook) g_tickHook();
        if (g_pDump) {
            const float r[6] = {(float) tNow(), sensors.Iout->ewm.avg.get(), sensors.Vout->ewm.avg.get(),
                                (float) converter.getCtrlOnPwmCnt(), (float) mppt.tracker.slowMode,
                                g_vconv.getVin() * g_vconv.pvCurrent(g_vconv.getVin())};
            fwrite(r, 4, 6, g_pDump);
        }
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
        g_nStarts += on && !wasOn;
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
    uint8_t n = 1;                           // series substrings (bypass-diode sections), up to 6
    float w[6] = {1, 0, 0, 0, 0, 0};         // their Voc shares (all 0 = equal)
    float vd = 0;                            // bypass-diode drop [V]
    Track cloud, sub[6];                     // irradiance factor of the whole string / per substring
    bool beamShade = false;                  // shade blocks the beam only: substring gets min(cloud, sub)
    float vocLog = 0;                        // Voc(t) = voc + vocLog * ln(irradiance)
    float vbat = 26.6f, rbat = 0.036f;
    Track vbatT;                             // battery EMF profile
    double sweepAt = -1, statsFrom = 0, T = 600;
    float isc = 0, g[6] = {1, 1, 1, 1, 1, 1}, vocNow = 0;
    float half = 0, vocDrift = 0;           // half-substring shade, Voc drift over T (fraction)
    bool exact = false;                      // exact series-string plant
    float gm[6] = {1, 1, 1, 1, 1, 1};        // per-substring Isc mismatch
    float ramp = 0, wobA = 0, wobF = 0;      // irradiance x exp(ramp (t - 120 s)), x (1 + wobA sin(2 pi wobF t))
    Track vocT;                              // source Voc profile (overrides vocLog)
    float iscFix = 0;                        // Isc [A] instead of the one from pClear
    bool noSink = false;                     // source does not sink current (bench PSU); default: sinks above Voc
    float manual = 0;                        // > 0: manual duty (fraction of pwmMax) instead of MPPT

    void init() {
        PvModel m;
        m.set(1, voc, k);
        float best = 0;
        for (float v = 1; v < voc; v += 0.05f) best = std::max(best, v * m.current(v));
        isc = iscFix > 0 ? iscFix : pClear / best;
        vocNow = voc;
        g_vconv.setPv(isc, voc, k);
        g_vconv.setPvBlocking(noSink);
        g_vconv.setBat(vbat, rbat);
        if (manual > 0)
            g_app.opMode = OpMode::Manual,
            mppt.manualTarget = (uint16_t) std::lround(manual * (float) converter.pwmMaxDriver());
    }

    void apply(double t) {
        const float c = trackAt(cloud, t, 1) * std::exp(ramp * (float) std::max(0.0, t - 120)) *
                        (1 + wobA * (float) std::sin(6.283185 * wobF * t));
        float gMax = 0;
        for (uint8_t i = 0; i < n; ++i) {
            const float s = trackAt(sub[i], t, 1);
            if (half > 0) g_vconv.setPvHalf(i, s < 1 ? 1.f / s : 1.f); // half the cells shaded, the other half in full light
            g[i] = (beamShade ? std::min(c, s) : c * s) * gm[i];
            gMax = std::max(gMax, g[i]);
        }
        g_vconv.setPvString(n, g, n > 1 && w[1] > 0 ? w : nullptr, vd);
        if (vocDrift != 0) {
            const float v = voc * (1 + vocDrift * (float) (t / T));
            if (std::fabs(v - vocNow) > 1e-3f) g_vconv.setPv(isc, vocNow = v, k);
        }
        if (vocLog > 0 || !vocT.empty()) {
            const float v = !vocT.empty() ? trackAt(vocT, t, voc)
                                          : std::max(1.f, voc + vocLog * std::log(std::max(gMax, 1e-30f)));
            if (std::fabs(v - vocNow) > 1e-3f) g_vconv.setPv(isc, vocNow = v, k);
        }
        if (!vbatT.empty()) g_vconv.setBat(trackAt(vbatT, t, vbat), rbat);
    }
};

struct Peak {
    float v, p;
};

// local maxima of the plant's P(V) in ascending V; valley[i] separates peak i and i+1
static int pvPeaks(Peak pk[6], float valley[6]) {
    int n = 0;
    float p0 = 0;
    bool rising = true;
    for (float v = 1; v < g_vconv.getVoc() + 0.2f && n < 6; v += 0.1f) {
        const float p = v * g_vconv.pvCurrent(v);
        if (rising && p < p0) pk[n++] = {v - 0.1f, p0}, rising = false;
        else if (!rising && p > p0) valley[n - 1] = v - 0.1f, rising = true;
        p0 = p;
    }
    return n;
}

// ---- switch-node ring analysis (ring-* scenarios) ----
struct SweepPt {
    uint16_t h;
    float vin, vout, iout, ioutSensor;
};
static std::vector<SweepPt> g_sweepPts;
static uint16_t g_sweepEndH = 0;
static double g_sweepEndT = -1, g_ringRecFrom = 0; // first sweep starting after g_ringRecFrom

static void ringTick() {
    static bool was = false;
    static uint16_t last = 0;
    const bool sw = mppt.getState() == MpptControlMode::Sweep && !converter.disabled();
    const uint16_t h = converter.getCtrlOnPwmCnt();
    if (sw && h > last && g_sweepEndT < 0 && tNow() >= g_ringRecFrom)
        g_sweepPts.push_back({h, g_vconv.getVin(), g_vconv.getVout(), g_vconv.getIoutAvg(), sensors.Iout->ewm.avg.get()});
    if (was && !sw && g_sweepEndT < 0 && !g_sweepPts.empty() && tNow() >= g_ringRecFrom) g_sweepEndT = tNow();
    if (g_sweepEndT >= 0 && !g_sweepEndH && tNow() >= g_sweepEndT + 2.0) g_sweepEndH = h; // after the slew to the MPP
    if (!sw) last = 0;
    else last = std::max(last, h);
    was = sw;
}

// steady PV power versus HS count at the present PV/battery: a copy of the plant stepped to equilibrium at
// each count, LS as the firmware sets it in DCM (voltage ratio with its 1 % worst-case margins)
static std::vector<std::pair<uint16_t, float>> staticCurve(uint16_t hi, uint16_t step) {
    VirtualConverter v = g_vconv;
    const uint16_t pmax = v.getPwm().pwmMax, lsMin = converter.getRectOnPwmMin();
    std::vector<std::pair<uint16_t, float>> out;
    for (uint16_t h = step; h <= hi; h += step) {
        for (int k = 0; k < 3; ++k) {
            const float r = v.getVin() / v.getVout() * (0.99f / 1.01f) - 1.f;
            long ls = std::max<long>(lsMin, std::lround(h * std::max(0.f, r)));
            v.setPwmCount(0, h), v.setPwmCount(1, (uint32_t) std::min<long>(ls, pmax - h - 1));
            v.stepSeconds(0.08f, 39000);
        }
        out.emplace_back(h, v.getVin() * v.pvCurrent(v.getVin()));
    }
    return out;
}

// local maxima of P(count) within +-w points above 50 % of the maximum
static std::vector<std::pair<uint16_t, float>> localMaxima(const std::vector<std::pair<uint16_t, float>> &c, int w) {
    std::vector<std::pair<uint16_t, float>> m;
    float pMax = 0;
    for (auto &p: c) pMax = std::max(pMax, p.second);
    for (int i = 0; i < (int) c.size(); ++i) {
        bool top = c[i].second > 0.5f * pMax;
        for (int j = std::max(0, i - w); top && j <= std::min((int) c.size() - 1, i + w); ++j)
            if (c[j].second > c[i].second) top = false;
        if (top) m.push_back(c[i]);
    }
    return m;
}

static float curveAt(const std::vector<std::pair<uint16_t, float>> &c, uint16_t h) {
    float best = 0;
    int d = 1 << 30;
    for (auto &p: c) if (std::abs((int) p.first - (int) h) < d) d = std::abs((int) p.first - (int) h), best = p.second;
    return best;
}

static void ringSweepReport() {
    const uint16_t pmax = g_vconv.getPwm().pwmMax;
    const float fsw = (float) converter.getPwmFrequency(), tick = 1.f / (fsw * (float) pmax), L = g_vconv.getL();
    float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
    std::vector<float> rLo, rHi, sLo, sHi;
    int n = 0;
    const size_t stride = std::max<size_t>(1, g_sweepPts.size() / 200);
    std::vector<SweepPt> dcm;
    for (auto &p: g_sweepPts) {
        const float D = (float) p.h / (float) pmax, cf = D * p.vin / p.vout;
        if (cf > 0.85f || cf < 0.1f || p.iout < 0.05f) continue;
        dcm.push_back(p);
        const float ideal = (p.vin - p.vout) * p.vin * D * D / (2 * p.vout * fsw * L);
        (cf < 0.5f ? rLo : rHi).push_back(p.iout / ideal - 1);
        (cf < 0.5f ? sLo : sHi).push_back(p.ioutSensor / ideal - 1);
    }
    // thin to <= kMaxPts evenly for the ring fit
    for (size_t k = 0; k < dcm.size() && n < coilfit::kMaxPts; k += std::max<size_t>(1, dcm.size() / coilfit::kMaxPts)) {
        auto &p = dcm[k];
        d[n] = (float) p.h / (float) pmax, vi[n] = p.vin, vo[n] = p.vout, io[n] = p.iout, ++n;
    }
    (void) stride;
    auto pp = [](std::vector<float> v) {
        if (v.size() < 5) return 0.f;
        std::sort(v.begin(), v.end());
        return v[(size_t) (0.95 * (double) (v.size() - 1))] - v[(size_t) (0.05 * (double) (v.size() - 1))];
    };
    const auto f = coilfit::fit(d, vi, vo, io, n, fsw);
    float ratio = 0;
    for (int i = 0; i < n; ++i) ratio += vi[i] / vo[i] / (float) n;
    const float N = f.f > 0 ? 1.f / (f.f * tick * ratio) : 0;
    printf("== RING sweep: %zu rising-duty ticks, %zu in DCM; plant Iout vs ideal DCM, p-p (5-95 %%): conduction <0.5"
           " %.0f %%, 0.5..0.85 %.0f %% (sensor ewm %.0f / %.0f %%)\n", g_sweepPts.size(), dcm.size(), 100 * pp(rLo),
           100 * pp(rHi), 100 * pp(sLo), 100 * pp(sHi));
    printf("== RING sweep fit: ring %.0f kHz (plant %.0f), period %.0f counts here = %.0f at 6.25 ns/count, Vin/Vout %.2f,"
           " ripple +-%.0f %%, %.1f periods; L from the sweep %.1f uH (plant %.1f)\n", f.f * 1e-3f,
           g_vconv.getRingF() * 1e-3f, N, N * tick / 6.25e-9f, ratio, 100 * f.amp, f.periods, f.L * 1e6f, L * 1e6f);
}

static void ringStaticReport(const char *what, const std::vector<double> &dutyHist) {
    const uint16_t hi = converter.pwmCtrlMax;
    const auto c = staticCurve(hi, 2);
    float pMax = 0;
    uint16_t hMax = 0;
    for (auto &p: c) if (p.second > pMax) pMax = p.second, hMax = p.first;
    const auto lm = localMaxima(c, 3);
    printf("== RING static P(count) now: max %.1f W at %u; %zu local maxima above 50 %%:", pMax, hMax, lm.size());
    for (size_t i = 0; i < lm.size() && i < 12; ++i) printf(" %u:%.1f%%", lm[i].first, 100 * lm[i].second / pMax);
    printf("\n");
    if (g_sweepEndT >= 0)
        printf("== RING sweep chose count %u (t=%.1f + 2 s): static P there %.1f %% of the max (max at %u)\n", g_sweepEndH,
               g_sweepEndT, 100 * curveAt(c, g_sweepEndH) / pMax, hMax);
    if (!dutyHist.empty()) {
        double tot = 0;
        for (double x: dutyHist) tot += x;
        std::vector<std::pair<double, int>> top;
        for (int b = 0; b < (int) dutyHist.size(); ++b) if (dutyHist[b] > 0) top.emplace_back(dutyHist[b], b);
        std::sort(top.rbegin(), top.rend());
        printf("== RING %s: time at count (4-count bins):", what);
        for (size_t i = 0; i < top.size() && i < 6; ++i)
            printf(" %d: %.0f%% (static P %.1f%%)", top[i].second * 4 + 2, 100 * top[i].first / tot,
                   100 * curveAt(c, (uint16_t) (top[i].second * 4 + 2)) / pMax);
        printf("\n");
    }
}

#if SCAN_ACTIVE_DEFINED
// integration checks around a running partial peak scan (SIM_ACT=psu|manual|limit on a scan scenario)
struct ItState {
    int act = 0, step = 0;
    double t1 = 0;
    float vMax0 = 0;
    bool done = false, fail = false;
    const char *why = "";
};

static void itFail(ItState &it, const char *why) {
    if (!it.fail) it.fail = true, it.why = why;
}

static void itStep(ItState &it, double t) {
    auto &sc = mppt.scan;
    if (it.act == 1 && it.step >= 1 && g_app.psuMode() && mppt.targetDutyCycle) itFail(it, "PSU mode with a duty fade target");
    switch (it.step) {
        case 0: // wait for a probe
            if (sc.phase != PeakScan::Probe) return;
            it.t1 = t, it.step = 1;
            printf("%9.2f >>> IT action during the probe\n", t);
            if (it.act == 1) {
                if (!mppt.requestPsuSetpoint(g_vconv.getVbat() + 0.3f)) itFail(it, "psu request refused");
            } else if (it.act == 2) {
                (void) mppt.requestPsuManual(converter.getCtrlOnPwmCnt(), -1);
            } else {
                it.vMax0 = mppt.charger.params.Vbat_max;
                mppt.charger.params.Vbat_max = sensors.Vout->ewm.avg.get() - 0.2f;
            }
            return;
        case 1:
            if (t - it.t1 < 0.5) return;
            if (sc.active()) itFail(it, "scan still active 0.5 s after the action");
            if (it.act == 1 && mppt.targetDutyCycle) itFail(it, "PSU mode with a duty fade target");
            if (it.act == 2 && !g_app.manualPwm()) itFail(it, "not in manual mode");
            if (it.act == 3 && mppt.teleSnap().limIdx == 15) itFail(it, "no limiter after the cap drop");
            it.step = 2, it.t1 = t;
            return;
        case 2:
            if (it.act == 1 && (sc.active() || mppt.targetDutyCycle)) itFail(it, "PSU: scan or fade target came back");
            if (t - it.t1 < 5) return;
            if (it.act == 2) (void) mppt.requestPsuOff(); // manual -> MPPT, no sweep
            if (it.act == 3) mppt.charger.params.Vbat_max = it.vMax0;
            it.step = 3, it.t1 = t;
            return;
        case 3:
            // back in MPPT: no scan from the old reference before a new one is taken (5 s) and the period passed
            if (it.act == 2 && sc.active() && t - it.t1 < 6) itFail(it, "stale scan resumed after manual -> MPPT");
            if (t - it.t1 > 10) it.done = true, it.step = 4;
            return;
        default:
            return;
    }
}
#endif

static int runWx(const std::string &name, Wx &wx) {
    if (const char *e = getenv("SIM_T")) wx.T = atof(e);
    g_verbose = getenv("SIM_V") != nullptr;
    const double trEvery = getenv("SIM_TR") ? atof(getenv("SIM_TR")) : wx.T / 60;
    // exact series string by default on the bypass-diode scenarios; SIM_PV_EXACT=0/1 overrides
    g_vconv.setPvExact(getenv("SIM_PV_EXACT") ? atoi(getenv("SIM_PV_EXACT")) != 0 : wx.exact);
    wx.init();
    wx.apply(0);
    std::vector<double> dutyHist;
    if (g_ring) g_tickHook = ringTick, dutyHist.assign(1024, 0.0), g_ringRecFrom = std::max(0.0, wx.sweepAt);
    printf("# %s: clear-sky MPP %.0f W, Isc %.2f A, Voc %.0f V, %u substring(s), bat %.2f V / %.3f ohm, %.0f s\n",
           name.c_str(), wx.pClear, wx.isc, wx.voc, (unsigned) wx.n, wx.vbat, wx.rbat, wx.T);
    printf("#        t  g(substrings)     | PLANT Vin   Ppv | GMPP V     P  peaks | CTRL state D lim\n");

    enum { OFF, SWEEP, LIMIT, WRONG, TRACK, SCAN, NCAT };
    double eG = 0, eUse = 0, ePv = 0, eOut = 0, eCtl = 0, tCat[NCAT] = {}, loss[NCAT] = {}, tStat = 0;
    struct Span { // vOp..pG: at the largest gap
        double t0, t1, lost;
        float vOp, pOp, vG, pG;
    };
    std::vector<Span> wrong;
    bool inWrong = false;
    Peak pk[6];
    float valley[6], key[7] = {-1, -1, -1, -1, -1, -1, -1};
    int nPk = 0, maxPk = 0;
    // tracker steady state: duty and Vin while tracking, slow-mode time, reversals, slow-mode entries
    double dN = 0, dS = 0, dS2 = 0, vS = 0, vS2 = 0, tSlow = 0, lossSlow = 0;
    float slowDvMax = 0; // largest |Vin - Vmpp| / Vmpp in slow mode
    float dMin = 1e9f, dMax = 0, vMin = 1e9f, vMax = 0;
    uint32_t nRev = 0, nSlowIn = 0;
    time_ms lastRev = 0;
    bool wasSlow = false;
    double holdAt = getenv("SIM_HOLD") ? atof(getenv("SIM_HOLD")) : -1; // tracker output x0 from then on
    // reverse output current per 20 ms slice (plant truth); time on above the LS min pulse's carry-over ratio
    double iMin = 0, tNeg = 0, qNeg = 0, tAbove = 0, tAboveDcm = 0;
    float mOnMax = 0;
    const unsigned starts0 = g_nStarts;
    const float mCrit = 1.f - (float) converter.getRectOnPwmMin() /
                              ((float) converter.pwmMaxDriver() + (float) converter.getDtLhTicks());

#if SCAN_ACTIVE_DEFINED
    ItState it{};
    if (const char *e = getenv("SIM_ACT")) it.act = !strcmp(e, "psu") ? 1 : !strcmp(e, "manual") ? 2 : !strcmp(e, "limit") ? 3 : 0;
#endif
    for (double nextTr = 0; tNow() < wx.T;) {
        const double tb = tNow();
        wx.apply(tb);
        if (holdAt >= 0 && tb >= holdAt) mppt.speedScale = 0, holdAt = -1;
        if (wx.sweepAt >= 0 && tb >= wx.sweepAt) {
            printf("%9.2f >>> sweep requested\n", tb);
            mppt.startSweep();
            wx.sweepAt = -1;
        }
        const double q0 = g_qOut;
        run(0.02, 0);
        const double dt = tNow() - tb;
#if SCAN_ACTIVE_DEFINED
        if (it.act) itStep(it, tb);
#endif
        if (tb >= wx.statsFrom && dt > 0) {
            const double im = (g_qOut - q0) / dt;
            iMin = std::min(iMin, im);
            if (im < -0.05) tNeg += dt;
            if (im < 0) qNeg -= im * dt;
            if (!converter.disabled() && g_vconv.getVin() > 1) {
                const float m = g_vconv.getVout() / g_vconv.getVin();
                mOnMax = std::max(mOnMax, m);
                if (m > mCrit) tAbove += dt, tAboveDcm += converter.inDCM() ? dt : 0;
            }
        }

        const float now[7] = {wx.g[0], wx.g[1], wx.g[2], wx.g[3], wx.g[4], wx.g[5], wx.vocNow};
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
        const bool sweeping = mppt.isSweeping() || (mppt.getState() == MpptControlMode::Sweep && !SCAN_ACTIVE());
        const bool isWrong = on && !sweeping && !SCAN_ACTIVE() && iG >= 0 && iOp != iG && pG > pk[iOp].p * 1.02f;

        if (tb >= nextTr) {
            printf("%9.2f  %.3f %.3f %.3f | %5.1f %6.1f | %5.1f %6.1f  %d(%d) | %-11s %4u %2u%s\n", tNow(), wx.g[0],
                   wx.n > 1 ? *std::min_element(wx.g + 1, wx.g + wx.n) : 0.f,
                   wx.n > 2 ? *std::max_element(wx.g + 1, wx.g + wx.n) : 0.f, vin, pPv, iG < 0 ? 0.f : pk[iG].v, pG, nPk, nReach,
                   modeStr(), converter.getCtrlOnPwmCnt(), (unsigned) mppt.teleSnap().limIdx, isWrong ? "  WRONG PEAK" : "");
            nextTr += trEvery;
        }
        if (tb < wx.statsFrom) continue;
        if (g_ring && on && !sweeping) dutyHist[std::min<size_t>(1023, converter.getCtrlOnPwmCnt() / 4)] += dt;

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
        const int cat = !on ? OFF : sweeping ? SWEEP : SCAN_ACTIVE() ? SCAN
                        : mppt.teleSnap().limIdx != 15 ? LIMIT : isWrong ? WRONG : TRACK;
        tCat[cat] += dt;
        loss[cat] += (pG - pPv) * dt;
        const auto &trk = mppt.tracker;
        if (cat == TRACK) {
            const float d = converter.getCtrlOnPwmCnt();
            dN += dt, dS += d * dt, dS2 += d * d * dt, vS += vin * dt, vS2 += vin * vin * dt;
            dMin = std::min(dMin, d), dMax = std::max(dMax, d), vMin = std::min(vMin, vin), vMax = std::max(vMax, vin);
        }
        if (trk.slowMode) {
            tSlow += dt;
            if (cat == TRACK && iG >= 0) lossSlow += (pG - pPv) * dt, slowDvMax = std::max(slowDvMax, std::fabs(vin / pk[iG].v - 1));
        }
        nSlowIn += trk.slowMode && !wasSlow, wasSlow = trk.slowMode;
        nRev += trk._timeLastReverse != lastRev, lastRev = trk._timeLastReverse;
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
    size_t nScan = 0, nMoved = 0, nLight = 0;
    for (auto &[t, c]: g_scanT) if (t >= wx.statsFrom) ++nScan, nMoved += c == 'm', nLight += c == 'l';
    printf("== SUMMARY %-18s %5s Hz | harvest %5.1f %% of GMPP, %5.1f %% of cap-limited | wrong-peak %6.1f s | off %6.1f s |"
           " sweeps %2zu | limiter %4.1f %% | trips %zu [%s]\n", name.c_str(), hz ? hz : "450", ePv * pc,
           eUse > 0 ? 100 * ePv / eUse : 0, tWrong, tCat[OFF], nSweeps, 100 * tCat[LIMIT] / tStat, nTrips, trips.c_str());
    printf("== SCAN %-18s %zu scans (%zu moved, %zu light changed), scanning %.1f s, lost while scanning %.3f Wh = %.2f %% of"
           " GMPP, %.1f J per scan; wrong-peak loss %.2f %%\n", name.c_str(), nScan, nMoved, nLight, tCat[SCAN],
           loss[SCAN] / 3600, loss[SCAN] * pc, nScan ? loss[SCAN] / (double) nScan : 0.0, loss[WRONG] * pc);
    for (auto &[t, c]: g_scanT) if (t >= wx.statsFrom && nScan <= 40) printf("==   scan at t=%.1f s %c\n", t, c);
    if (mppt.scan.unsaved()) { // learned in this run: the save and the reload
        mppt.scanSave();
        ConfFile tc{"/littlefs/conf/tracker.conf"};
        printf("== SCAN learned %u substrings, tracker.conf pv_substrings=%u\n", mppt.scan.n, tc.getByte("pv_substrings", 0));
    }
    {
        size_t nMp = 0;
        for (auto &t: g_trips) nMp += t.t >= wx.statsFrom && t.who == "minpulse-M";
        printf("== REVERSE %-18s plant Iout 20 ms mean min %.3f A, %.1f s below -0.05 A, %.3f C back | starts %u,"
               " minpulse-M stops %zu | on at plant M > %.3f (1 - t_min/T) %.1f s (%.1f s DCM), max on M %.3f\n",
               name.c_str(), iMin, tNeg, qNeg, g_nStarts - starts0, nMp, mCrit, tAbove, tAboveDcm, mOnMax);
    }
    {
        size_t nLogged = 0;
        for (double t: g_slowDownT) nLogged += t >= wx.statsFrom;
        const double dm = dN > 0 ? dS / dN : 0, vm = dN > 0 ? vS / dN : 0, k = 1000.0 / converter.pwmCtrlMax;
        printf("== TRACKER %-18s tracking %.0f s: duty mean %.0f rms %.1f p-p %.0f counts of %u (%.2f / %.1f permille),"
               " Vin mean %.2f rms %.3f p-p %.2f V | tracking loss %.2f %% | slow %.1f %% of the time, %u entries"
               " (%zu logged, every %.0f s) | %.1f reversals/min | harvest %.2f %% | tracking loss %.0f J, in slow mode"
               " %.0f J, slow-mode Vin up to %.1f %% off the MPP\n", name.c_str(), dN, dm,
               std::sqrt(std::max(0.0, dS2 / std::max(dN, 1e-9) - dm * dm)), dMax - dMin, converter.pwmCtrlMax,
               std::sqrt(std::max(0.0, dS2 / std::max(dN, 1e-9) - dm * dm)) * k, (dMax - dMin) * k, vm,
               std::sqrt(std::max(0.0, vS2 / std::max(dN, 1e-9) - vm * vm)), vMax - vMin, loss[TRACK] * pc,
               100 * tSlow / tStat, nSlowIn, nLogged, nLogged ? tStat / nLogged : 0.0, nRev * 60 / tStat, ePv * pc,
               loss[TRACK], lossSlow, 100 * slowDvMax);
    }
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
    if (g_ring) {
        g_tickHook = nullptr;
        if (!g_sweepPts.empty()) ringSweepReport();
        ringStaticReport("tracking", dutyHist);
    }
#if SCAN_ACTIVE_DEFINED
    if (it.act) {
        printf("== IT %s: %s%s\n", getenv("SIM_ACT"), it.fail ? "FAIL " : it.done ? "PASS" : "FAIL scan never active",
               it.fail ? it.why : "");
        if (it.fail || !it.done) return 1;
    }
#endif
    return 0;
}

// wx-* scenario table; false = unknown name
static bool wxScenario(const std::string &sc, Wx &wx) {
    // fry-like string: Voc 80 V, peaks near 68 V (all), 57 V and 30 V (substrings bypassed)
    auto fry = [&](float pClear) { wx.voc = 80, wx.k = 0.86f, wx.pClear = pClear; };
    // fry/flat geometry: 2 modules x 3 bypass diodes, local peaks near m/6 of the string's Vmpp
    auto fry6 = [&](float pClear) {
        fry(pClear), wx.n = 6, wx.vd = 0.45f, wx.exact = true;
        if (const char *e = getenv("SIM_K")) wx.k = atof(e);
        if (const char *e = getenv("SIM_W")) sscanf(e, "%f,%f,%f,%f,%f,%f", wx.w, wx.w + 1, wx.w + 2, wx.w + 3, wx.w + 4, wx.w + 5);
        if (const char *e = getenv("SIM_HALF")) wx.half = atof(e);
        if (const char *e = getenv("SIM_G")) // per-substring Isc mismatch factors
            sscanf(e, "%f,%f,%f,%f,%f,%f", wx.gm, wx.gm + 1, wx.gm + 2, wx.gm + 3, wx.gm + 4, wx.gm + 5);
        if (const char *e = getenv("SIM_VOCDRIFT")) wx.vocDrift = atof(e);
    };
    if (sc == "wx-cloud") {
        wx.cloud = kCloud1120, wx.T = 900;
    } else if (sc == "wx-cloud-cv" || sc == "wx-cloud-cv-ripple") {
        // battery high enough that full sun runs into the Vout cap and every cloud releases it
        wx.cloud = kCloud1120, wx.T = 900, wx.vbat = 28.2f;
    } else if (sc == "wx-shade2" || sc == "wx-shade2-move") {
        // one of 6 substrings at 0.7: 57 V peak 18 % above the 68.8 V peak (fry, mornings); -move: starts unshaded
        fry6(215);
        if (sc == "wx-shade2") wx.sub[5] = {{0, 0.70f}};
        else wx.sub[5] = {{120, 1}, {420, 0.70f}}, wx.T = 2400;
    } else if (sc == "wx-shade3" || sc == "wx-shade3-move") {
        fry6(300);
        if (sc == "wx-shade3") {
            wx.sub[3] = wx.sub[4] = {{0, 0.75f}}, wx.sub[5] = {{0, 0.5f}}; // peaks 34 / 57 / 68.8 V, global = 57 V
        } else {
            // one module shaded 0.3 -> 0.72: global 34 V peak -> 68.8 V peak 50 % above it
            wx.sub[3] = wx.sub[4] = wx.sub[5] = {{120, 0.3f}, {420, 0.72f}}, wx.T = 2400;
        }
    } else if (sc == "wx-shade-learn") {
        fry6(215), wx.T = 3400;
        wx.sub[5] = {{0, 0.70f}, {1900, 0.70f}, {1960, 1}, {2000, 1}, {2300, 0.70f}};
    } else if (sc == "wx-shade-walk") {
        // a shadow (0.35) walks across the substrings, about 2.5 of them at a time: the global peak moves
        fry6(400), wx.T = 2400;
        for (int i = 0; i < 6; ++i) {
            const float t0 = 200 + 150.f * (float) i;
            wx.sub[i] = {{t0, 1}, {t0 + 60, 0.35f}, {t0 + 400, 0.35f}, {t0 + 460, 1}};
        }
    } else if (sc == "wx-shade-cloud") {
        // beam shade on one substring under the cloud series: every cloud removes the second peak
        fry6(615);
        wx.cloud = kCloud1120, wx.sub[5] = {{0, 0.664f}}, wx.beamShade = true, wx.T = 900;
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
    } else if (sc == "wx-low" || sc == "wx-mid" || sc == "wx-low-slow") {
        // steady light rising slowly after the boot sweep, with sensor noise: MPP 2 -> 20 W in 30 min (x2 per 9 min),
        // 55 -> 110 W in 10 min (x2 per 10 min, dawn rate) then 10 min level
        const bool low = sc != "wx-mid";
        const float g0 = (low ? 2.5f : 61.f) / wx.pClear, r = low ? 10.f : 2.f, tr = low ? 1800 : 600;
        wx.vocLog = 2.6f, wx.T = low ? 1860 : 1260;
        for (float t = 0; t <= wx.T; t += 10) wx.cloud.emplace_back(t, g0 * std::pow(r, std::clamp(t - 60, 0.f, tr) / tr));
    } else if (sc.rfind("wx-sun-", 0) == 0 || sc.rfind("wx-ramp-", 0) == 0 || sc.rfind("wx-wob-", 0) == 0) {
        // steady sun, MPP SIM_P W; fry: Vin ~64 V at 70 W, flat: ~55 V
        const float p = getenv("SIM_P") ? atof(getenv("SIM_P")) : 70;
        if (sc.compare(sc.size() - 4, 4, "-fry") == 0) fry(615), wx.vocLog = 2.6f;
        if (const char *e = getenv("SIM_VOC")) wx.voc = atof(e);
        if (sc.rfind("wx-ramp-", 0) == 0) wx.ramp = (getenv("SIM_RAMP") ? atof(getenv("SIM_RAMP")) : 2) / 6000;
        if (sc.rfind("wx-wob-", 0) == 0) {
            wx.wobA = 0.005f, wx.wobF = 0.2f;
            if (const char *e = getenv("SIM_WOB")) sscanf(e, "%f,%f", &wx.wobA, &wx.wobF);
        }
        float g = p / wx.pClear;
        for (int i = 0; i < 6; ++i) g = p / wx.pClear * wx.voc / (wx.voc + wx.vocLog * std::log(g));
        wx.cloud = {{0, g}}, wx.vbat = getenv("SIM_VBAT") ? atof(getenv("SIM_VBAT")) : 26.1f, wx.statsFrom = 120, wx.T = 900;
    } else if (sc == "wx-mp-manual") {
        // manual duty 0.8 from a stiff sinking source, Voc 1.25 -> 1.02 x Vbat and back: DCM up through the stop ratio;
        // after the stop the manual restart must wait for the start ratio
        wx.vbat = 28.95f, wx.voc = 1.25f * wx.vbat, wx.T = 1500, wx.k = 0.95f, wx.iscFix = 20, wx.manual = 0.8f;
        wx.vocT = {{120, wx.voc}, {720, 1.02f * wx.vbat}, {780, 1.02f * wx.vbat}, {1380, wx.voc}};
        wx.statsFrom = 60;
    } else if (sc == "wx-mp-ccm") {
        // stiff sinking source into a battery below the cap, Voc 1.3 -> 1.02 x Vbat: MPPT runs in CCM (no floor there)
        // until the duty limit, then the current falls and the converter enters DCM near or above the stop ratio
        wx.vbat = 26.6f, wx.voc = 1.3f * wx.vbat, wx.T = 700, wx.k = 0.95f, wx.iscFix = 20;
        wx.vocT = {{150, wx.voc}, {500, 1.02f * wx.vbat}};
        wx.statsFrom = 60;
    } else if (sc == "wx-mp-ramp" || sc == "wx-mp-psu" || sc == "wx-mp-psu-nosink") {
        // operating point from 1.25 x Vbat down to ~1.02 x and back over 2 x 600 s from 120 s. -ramp: weak PV (k 0.78,
        // Voc 1.6 -> 1.3 x Vbat, 20 W at the start) in DCM near the ratio limit. -psu: stiff source (k 0.95, Isc 20 A,
        // sinks above Voc like a battery bus; Voc 1.25 -> 1.02 x) into a battery at the Vout cap, so the converter runs
        // light in DCM; -nosink: a bench PSU that cannot sink
        const bool pv = sc == "wx-mp-ramp";
        wx.vbat = pv ? 26.6f : 28.95f, wx.voc = (pv ? 1.6f : 1.25f) * wx.vbat, wx.T = 1500;
        if (pv) wx.k = 0.78f, wx.pClear = 20;
        else wx.k = 0.95f, wx.iscFix = 20, wx.noSink = sc == "wx-mp-psu-nosink";
        const float lo = (pv ? 1.3f : 1.02f) * wx.vbat;
        wx.vocT = {{120, wx.voc}, {720, lo}, {780, lo}, {1380, wx.voc}};
        wx.statsFrom = 60;
    } else if (sc == "wx-sweep-fall" || sc == "wx-sweep-rise") {
        // cloud edge (73 % in 10 s) starting 2 s into a sweep
        const bool fall = sc == "wx-sweep-fall";
        wx.cloud = {{62, fall ? 1 : 0.27f}, {72, fall ? 0.27f : 1}};
        wx.sweepAt = wx.statsFrom = 60, wx.T = 240;
    } else if (sc == "ring-sweep" || sc == "ring-weak") {
        // field-like string at the board's morning Voc; -sweep: a global sweep at 60 s in ~150-180 W sun
        const bool fry = !strcmp(g_ring->name, "fry");
        wx.voc = fry ? 78 : 70, wx.k = 0.86f, wx.vbat = fry ? 26.8f : 26.3f;
        if (sc == "ring-sweep") wx.pClear = fry ? 150 : 180, wx.sweepAt = wx.statsFrom = 60, wx.T = 150;
        else wx.pClear = 45, wx.statsFrom = 150, wx.T = 900;
    } else if (sc == "wx-sweep-shade") {
        // two peaks, the 68.8 V one 10 % higher; a rising cloud edge during the sweep favours the later 57 V one
        fry6(615), wx.sub[5] = {{0, 0.91f}};
        wx.cloud = {{62, 0.27f}, {72, 1}};
        wx.sweepAt = wx.statsFrom = 60, wx.T = 660;
    } else {
        return false;
    }
    return true;
}

// LS count (fractional) where the plant's coil current at LS turn-off first turns negative as LS grows, HS fixed,
// stiff Vin/Vout; i0: coil current at HS turn-on there. NAN: none below the period.
static double plantZc(float vin, float vout, int hs, double &i0) {
    const uint32_t f = converter.getPwmFrequency();
    const auto at = [&](int ls, double &a) {
        VirtualConverter p = g_vconv;
        p.setPvBlocking(true), p.setBat(vout, 1e-6f), p.setBatRipple(0, 100);
        p.setPwmCount(0, hs), p.setPwmCount(1, ls);
        double s = 0, s0 = 0;
        for (int k = 0; k < 600; ++k) {
            p.setVin(vin), p.setVout(vout);
            const float il = p.getIL();
            p.stepSeconds(1.f / (float) f, f);
            if (k >= 300) s += p.getILsOff(), s0 += il;
        }
        a = s0 / 300;
        return s / 300;
    };
    const double id = (vin / vout - 1) * hs;
    double a, prev = at((int) (0.6 * id), a);
    for (int ls = (int) (0.6 * id) + 1; ls < converter.pwmMaxDriver() - hs - 1; ++ls) {
        const double c = at(ls, a);
        if (prev > 0 && c <= 0) return i0 = a, ls - c / (c - prev);
        prev = c;
    }
    return NAN;
}

// rect-cal force at full power: ramp-down, calibration, ramp back, against the plant. Reports trips,
// the learned offset (no gate delay in the plant: the truth is 0), the ramp and the power after.
static int runRectCal(const std::string &name, const std::string &sc) {
    settle(60);
    printf("# trips during settle: %zu\n", g_trips.size());
    auto &rc = mppt.rectCal;
    const uint16_t hs0 = converter.getCtrlOnPwmCnt();
    const float p0 = sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get();
    const bool dcm0 = converter.inDCM();
    Probe p = probeStart();
    const float vbat0 = g_vconv.getVbat(), isc0 = g_vconv.getIsc(), voc0 = g_vconv.getVoc();
    if (!rc.requestStart(true)) { printf("requestStart refused\n"); return 1; }
    printf("%9.4f >>> rect-cal force at HS %u, %s, P %.0f W\n", tNow(), hs0, dcm0 ? "DCM" : "CCM", p0);
    double tCal = -1, tEnd = -1;
    float iMin = INFINITY, ilMin = INFINITY, vinMax = 0, iRunMin = INFINITY, ilAfter = INFINITY;
    int stepMax = 0, lsOffMin = 0, nPv = 0;
    double pvSum = 0, poSum = 0;
    uint16_t hsPrev = hs0;
    const uint8_t clamps0 = converter.lsClamps();
    std::mt19937 busRng{7};
    std::uniform_real_distribution<float> busU{0.f, 1.f};
    double busNext = 0;
    float busStep = 0;
    while (tNow() - p.t0 < 60) {
        const double t = tNow() - p.t0;
        if (sc == "rcal-bus") g_vconv.setBat(vbat0 * (1.f + 0.025f * (float) std::min(1.0, t / 30.0)), g_vconv.getRbat());
        if (sc == "rcal-cloud" && t >= 12.0) g_vconv.setPv(isc0 * 0.7f, voc0 * 0.982f, g_vconv.getPvK());
        if (sc.rfind("rcal-low", 0) == 0 && t >= 0.2) g_vconv.setPv(2.f, voc0 * 0.97f, g_vconv.getPvK());
        // bus transients: Vout +0.6 V for 2 ms every 1-3 s, D-only Vout limiter touches far below the 29 V cap
        // as logged on flat 2026-11-15 (ctrl -92, act 26.7, tgt 29); too short to move a window average
        if (sc == "rcal-flat") {
            busStep = t >= busNext ? 0.6f : 0.f;
            while (t >= busNext) busNext += 1.0 + 2.0 * busU(busRng);
            g_vconv.setBat(vbat0 + busStep, g_vconv.getRbat());
        }
        if (sc == "rcal-low-bus") {
            while (t >= busNext) busNext += 2.0 + 4.0 * busU(busRng), busStep = (busU(busRng) - 0.5f) * 0.003f;
            g_vconv.setBat(vbat0 * (1.f + 0.004f * (float) std::sin(6.2832 * t / 23.) + 0.002f * (float) std::sin(6.2832 * t / 7.3)
                                    + busStep), g_vconv.getRbat());
        }
        run(0.0022, 0, &p);
        const auto st = rc.cal.get();
        if (st == RectCal::Run) {
            const uint16_t hs = converter.getCtrlOnPwmCnt();
            stepMax = std::max(stepMax, std::abs((int) hs - (int) hsPrev));
            hsPrev = hs;
            if (hs > rc.cal.hs && converter.manualRect_() && converter.getRectOnPwmCnt() != converter.getRectOnPwmMin())
                ++lsOffMin;
            if (tCal < 0 && hs == rc.cal.hs) tCal = t;
            iRunMin = std::min(iRunMin, g_vconv.getIoutAvg());
            if (hs == rc.cal.hs && converter.manualRect_()) pvSum += g_vconv.getVin(), poSum += g_vconv.getVout(), ++nPv;
        }
        iMin = std::min(iMin, g_vconv.getIoutAvg());
        ilMin = std::min(ilMin, g_vconv.getIL());
        vinMax = std::max(vinMax, g_vconv.getVin());
        if (tEnd < 0 && (st == RectCal::Done || st == RectCal::Fail)) tEnd = t;
        if (tEnd >= 0 && t > tEnd + 3) ilAfter = std::min(ilAfter, g_vconv.getIL());
        if (tEnd >= 0 && t > tEnd + 15) break;
    }
    report(name.c_str(), p);
    const float p1 = sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get();
    printf("== rect-cal: %s%s%s after %.1f s; cal HS %u reached at +%.2f s (from %u, %s), max HS step %d/sample, "
           "LS off its minimum during the ramp: %d samples, LS clamps %u\n",
           rc.cal.get() == RectCal::Done ? "done" : "failed", rc.cal.get() == RectCal::Fail ? ": " : "",
           rc.cal.get() == RectCal::Fail ? rc.cal.why : "", tEnd, rc.cal.hs, tCal, hs0, dcm0 ? "DCM" : "CCM", stepMax,
           lsOffMin, (unsigned) (uint8_t) (converter.lsClamps() - clamps0));
    printf("== rect-cal: ideal %.0f peak %.0f learned %+d applied %+d ct%s%s (plant timing error 0), %u pts, %u redone, dV %.2f%%, Iout rise "
           "%.1f mA (bound %.1f), offset now %d ct, rect_offset_cal %u\n", rc.cal.ideal, rc.cal.peak, rc.cal.learned,
           rc.cal.applied, rc.cal.clamped ? " (bound)" : "", rc.cal.cut ? " (fit ended at a dip)" : "", rc.cal.n,
           rc.cal.retries, 100 * rc.cal.dv, 1e3f * rc.cal.rise, 1e3f * rc.cal.iLs, rc.offset(), converter.rectOffsetCal());
    if (nPv && rc.cal.hs) {
        const float tr = converter.getPwmTickRate(), vi = (float) (pvSum / nPv), vo = (float) (poSum / nPv);
        double i0 = 0;
        const double zc = plantZc(vi, vo, rc.cal.hs, i0), t = zc - rc.cal.ideal;
        const float zq = [](const auto &c) { if constexpr (requires { c.zc; }) return (float) c.zc; else return NAN; }(rc.cal);
        printf("== truth: plant Vin %.2f Vout %.2f at HS %u: zero crossing %.1f ct, %+.1f ct (%+.0f ns) off ideal %.1f "
               "(plant voltages: %+.1f ct), coil current at HS turn-on %+.3f A; charge zc %+.0f ct; learned %+.0f, applied "
               "%+.0f ct off the truth\n", vi, vo, rc.cal.hs, zc, t, t / tr * 1e9, rc.cal.ideal, zc - (vi / vo - 1) * rc.cal.hs,
               i0, zq - rc.cal.ideal, rc.cal.learned - t, rc.cal.applied - t);
    }
    printf("== rect-cal points (LS - ideal : dI A):");
    for (int i = 0; i < rc.cal.n; ++i) printf(" %.0f:%+.3f", rc.cal.xs[i] - rc.cal.ideal, rc.cal.ys[i]);
    printf("\n");
    printf("== plant: min Iout avg %.2f A (during the run %.2f A), min coil current at cycle end %.3f A (from 3 s after "
           "the run %.3f A), max Vin %.2f V\n", iMin, iRunMin, ilMin, ilAfter, vinMax);
    printf("== after: HS %u (before %u), P %.0f W (before %.0f W), %s, manual LS %d\n", converter.getCtrlOnPwmCnt(), hs0,
           p1, p0, modeStr(), converter.manualRect_());
    printf("\n== log after event (first %zu lines):\n", g_logSince.size());
    for (size_t i = 0; i < g_logSince.size() && i < 60; ++i) printf("   %s\n", g_logSince[i].c_str());
    // a late offset eats the 2 % margin of the LS ratio near D = M: never past LateMaxNs
    const int hi = pwmCountsFromNs(RectCal::LateMaxNs, converter.getPwmTickRate());
    if (rc.offset() > hi || (rc.cal.get() == RectCal::Done && (rc.cal.applied > hi || (rc.cal.clamped && rc.cal.applied < rc.offset())))) {
        printf("FAIL: offset past LateMaxNs applied (%+d ct, active %+d ct)\n", rc.cal.applied, rc.offset());
        return 1;
    }
    return 0;
}

// `measure-coil l0` against the plant: mirrors measureCoilTask()/sweepL0() of src/selftest/measure_coil.cpp
// (the console task's waits become simulated time), estimator shared through selftest/coil_fit.h.
static bool simPsuWait(uint32_t ticket) {
    for (int i = 0; i < 1000 && !mppt.isPsuCommandDone(ticket); ++i) run(0.001, 0);
    return mppt.isPsuCommandDone(ticket);
}

static float med5(float *a) { return coilfit::detail::median(a, 5); }

static coilfit::Result simMeasureL0(int steps, uint32_t dwellMs, double &secs) {
    const double t0 = tNow();
    coilfit::Result res;
    if (!simPsuWait(mppt.requestPsuManual(0, -1))) printf("  measure: RT transition timed out\n");
    const float fsw = (float) converter.getPwmFrequency();
    const uint16_t pwmMax = converter.pwmMaxDriver();
    const float iMaxLim = std::min(2.0f, mppt.limits.Iout_max);
    const float vin0 = sensors.Vin->ewm.avg.get(), vout0 = sensors.Vout->ewm.avg.get();
    int lo, hi, step;
    if (coilfit::band(vin0, vout0, pwmMax, converter.pwmCtrlMax, steps, lo, hi, step)) {
        float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
        int n = 0;
        float iMaxSeen = 0;
        for (int H = lo; H <= hi && n < coilfit::kMaxPts; H += step) {
            mppt.setManualTarget((uint16_t) H);
            const double dl = tNow() + std::max<uint32_t>(dwellMs, 3000) * 1e-3 + H * 4e-3;
            while (converter.getCtrlOnPwmCnt() != H && tNow() < dl) run(0.02, 0);
            run(dwellMs * 1e-3, 0);
            float a[5], b[5], c[5];
            for (int k = 0; k < 5; ++k) {
                a[k] = sensors.Vin->ewm.avg.get(), b[k] = sensors.Vout->ewm.avg.get(), c[k] = sensors.Iout->ewm.avg.get();
                run(0.05, 0);
            }
            if (converter.disabled()) { printf("  measure: converter disabled; aborting\n"); break; }
            const float vin = med5(a), vout = med5(b), iout = med5(c), D = (float) converter.getCtrlOnPwmCnt() / pwmMax;
            const bool dcm = converter.inDCM(), ok = iout > 0.02f && vin > vout;
            if (dcm && ok) d[n] = D, vi[n] = vin, vo[n] = vout, io[n] = iout, ++n, iMaxSeen = std::max(iMaxSeen, iout);
            if (iout > iMaxLim || !dcm) break;
        }
        const float floor = std::max(0.15f, 0.2f * iMaxSeen);
        int nk = 0;
        for (int i = 0; i < n; ++i)
            if (io[i] >= floor) d[nk] = d[i], vi[nk] = vi[i], vo[nk] = vo[i], io[nk] = io[i], ++nk;
        res = coilfit::fit(d, vi, vo, io, nk, fsw);
    }
    // restore as the task does: ramp down, wait for the disable, back to MPPT
    mppt.setManualTarget(0);
    for (double dl = tNow() + 10.0; !converter.disabled() && tNow() < dl;) run(0.01, 0);
    const bool off = converter.disabled();
    if (!simPsuWait(off ? mppt.requestPsuOff() : mppt.requestPsuSweep())) printf("  measure: MPPT restore timed out\n");
    secs = tNow() - t0;
    return res;
}

static int runRingCoil(const std::string &name, bool weak) {
    Wx wx;
    const bool fry = !strcmp(g_ring->name, "fry");
    wx.voc = fry ? 78 : 70, wx.k = 0.86f, wx.vbat = fry ? 26.8f : 26.3f, wx.pClear = weak ? 40 : 150;
    wx.init();
    g_verbose = getenv("SIM_V") != nullptr;
    settle(45);
    const float Lt = g_vconv.getL();
    const char *tags[2] = {"old (10 steps, 3 s, median)", "new (48 steps, 1 s, ring fit)"};
    for (int m = 0; m < 2; ++m) {
        double secs = 0;
        const auto r = simMeasureL0(m ? 48 : 10, m ? 1000 : 3000, secs);
        const float L = m ? r.L : r.Lmed;
        printf("== RING coil %s: L %.2f uH = %+.1f %% vs plant %.2f uH (%d DCM pts, %.0f s); fit: ring %.0f kHz"
               " (plant %.0f), ripple +-%.0f %%, %.1f periods, median of points %.2f uH (%+.1f %%)\n", tags[m], L * 1e6f,
               100 * (L / Lt - 1), Lt * 1e6f, r.n, secs, r.f * 1e-3f, g_vconv.getRingF() * 1e-3f, 100 * r.amp, r.periods,
               r.Lmed * 1e6f, 100 * (r.Lmed / Lt - 1));
        // does MPPT come back on its own?
        const double tr = tNow();
        double tOn = -1;
        while (tNow() - tr < 40) {
            run(0.05, 0);
            if (tOn < 0 && !converter.disabled()) tOn = tNow() - tr;
        }
        const float v = g_vconv.getVin();
        printf("== RING coil restore: converter %s (first on after %.1f s), %s, PV %.1f W\n",
               converter.disabled() ? "OFF" : "running", tOn, modeStr(), v * g_vconv.pvCurrent(v));
    }
    printf("== SUMMARY %s: see RING coil lines\n", name.c_str());
    return 0;
}

// Start refused only by the idle profile's InhibitIdle while Vin rises, with a sweep armed (st=START,1: fry
// 2026-11-18, update() returned before any wake) and without (START,0: the start branch): the RT loop must raise
// the wake, not start, and start once the bit is released.
static int runIdleWake() {
    g_vconv.setPvBlocking(true); // the night's panel would sink the battery through the HS body diode
    settle(20);
    const float isc = g_vconv.getIsc(), voc = g_vconv.getVoc(), k = g_vconv.getPvK();
    int fails = 0;
    auto check = [&](bool c, const char *what, int armed) {
        printf("== idle-wake START,%d %s: %s\n", armed, c ? "ok" : "FAIL", what);
        fails += !c;
    };
    for (int armed = 1; armed >= 0; --armed) {
        stopAndBackoff(1);
        g_vconv.setPv(isc, 0.3f * g_vconv.getVbat(), k);
        run(10, 0);
        if (armed) mppt.startSweep();
        run(10, 0);
        const char *r = mppt.startBlockReason();
        traceLine("night ");
        check(converter.disabled() && mppt.isSweeping() == (bool) armed && r && !strcmp(r, "Vin-Vout"),
              "disabled, Vin-Vout", armed);
        converter.inhibitStart(true, SynchronousConverter::InhibitIdle);
        converter.takeStartWanted();
        g_vconv.setPv(isc, voc, k);
        run(3, 0);
        traceLine("dawn  ");
        check(mppt.startBlockReason() == MpptController::kIdleBlock, "only the idle bit blocks", armed);
        check(converter.disabled(), "no start while inhibited", armed);
        check(converter.takeStartWanted(), "wake raised", armed);
        const uint16_t en = converter.enableCount();
        converter.inhibitStart(false, SynchronousConverter::InhibitIdle);
        run(6, 0);
        traceLine("start ");
        check(converter.enableCount() != en, "started after release", armed);
    }
    printf("== SUMMARY idle-wake: %s (%d failures)\n", fails ? "FAILED" : "OK", fails);
    return fails != 0;
}

static int simMain(int argc, char **argv) {
    std::string sc = argc > 1 ? argv[1] : "sanity";
    g_simLogHook = logHook;
    if (const char *f = getenv("SIM_FINE")) sscanf(f, "%lf,%lf", &g_fineT0, &g_fineT1);
    // suffix -p0: limits.conf reverse_current_paranoia=0 (Vout-OV threshold 1.5x instead of 1.03x Vbat_max)
    const bool p0 = sc.size() > 3 && sc.compare(sc.size() - 3, 3, "-p0") == 0;
    const std::string name = sc;
    if (p0) sc.resize(sc.size() - 3);
    if (sc == "rcal-flat") setenv("SIM_RING", "flat", 0), setenv("SIM_ADC_HZ", "1470", 0);
    if (sc == "rcal-pv") setenv("SIM_RING", "fry", 0);
    if (sc.rfind("wx-", 0) == 0 && sc.size() > 4 && sc.compare(sc.size() - 4, 4, "-fry") == 0)
        setenv("SIM_RING", "fry-off", 0), setenv("SIM_ADC_HZ", "513", 0);
    if (sc.rfind("wx-", 0) == 0 && sc.size() > 5 && sc.compare(sc.size() - 5, 5, "-flat") == 0)
        setenv("SIM_RING", "flat-off", 0), setenv("SIM_ADC_HZ", "1469", 0);
    if (sc.rfind("wx-", 0) == 0) setenv("SIM_RING", "fry", 0); // tracker scenarios: a field board's plant
    if (const char *n = getenv("SIM_NOISE"); n && *n) noiseLoad(n);
    if (const char *f = getenv("SIM_PDUMP")) g_pDump = ::fopen(f, "wb");
    sc = ringBoardFrom(sc);
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
    const bool isWx = sc.rfind("wx-", 0) == 0 || sc == "ring-sweep" || sc == "ring-weak";
    if (isWx && !wxScenario(sc, wx)) {
        printf("unknown scenario %s\n", sc.c_str());
        return 1;
    }
    if (sc == "wx-low" || sc == "wx-mid" || sc == "wx-low-slow")
        ov = {{"noise_iout", sc != "wx-mid" ? "0.02" : "0.04"}, {"noise_vout", "0.03"}};
    if (sc == "wx-low-slow") { // filter length 60 on the power sensors (buck_bench, boost_*)
        std::string c = "sensor.conf:lv_i_filt_len=60,sensor.conf:lv_v_filt_len=60";
        if (const char *e = getenv("SIM_CONF")) c = c + "," + e;
        setenv("SIM_CONF", c.c_str(), 1);
    }
    if (sc == "wx-cloud-cv-ripple") ov = {{"noise_vout", "0.03"}, {"vbat_ac_amp", "0.26"}, {"vbat_ac_freq", "100"}};
    const bool rcal = sc.rfind("rcal", 0) == 0;
    if (rcal) ov = {{"noise_iout", "0.01"}, {"noise_vout", "0.005"}, {"noise_vin", "0.05"}};
    if (sc == "rcal-flat") ov.insert(ov.end(), {{"isc", "7.5"}, {"voc", "66"}, {"v_bat", "26.7"}});
    if (sc == "rcal-ov") setenv("SIM_CONF", "limits.conf:hv_max=77.5", 1);
    if (sc == "rcal-sun") setenv("SIM_CONF", "coil.conf:L0=80e-6", 1);
    if (const char *n = getenv("SIM_NOISE_IOUT"); rcal && n) ov[0].second = n;
    boot(ov, p0);
    if (rcal) g_vconv.setDiodeVf(0.7f);
    if (sc == "rcal-sun" && !g_ring) g_vconv.setRing(5e-9f, 40.f);
    if (sc == "rcal-sun") g_vconv.setPv(5.6f, 79.6f, 0.78f);
    if (sc == "rcal-pv") {
        float isc = 5.6f, voc = 79.6f, k = 0.78f;
        if (const char *e = getenv("SIM_PV")) sscanf(e, "%f,%f,%f", &isc, &voc, &k);
        g_vconv.setPv(isc, voc, k);
    }
    if (g_ring) {
        g_vconv.setPassives(g_vconv.getCin(), g_vconv.getCout(), g_ring->lPlant);
        printf("# board plant %s: coil.conf L0 %.0f uH, plant L %.0f uH, ring %s (%.0f kHz at %.0f V, Q %.1f, node C %.2f nF),"
               " Vf %.2f V, dead %.0f ns, R %.0f/%.0f mOhm, t_sw %.0f ns\n", g_ring->name, g_ring->l0Conf * 1e6f,
               g_ring->lPlant * 1e6f, g_ringOn ? (g_ringDamped ? "damped" : "on") : "OFF", g_ring->ringF * 1e-3f, g_ring->ringV,
               g_ring->ringQ, g_ring->ringC() * 1e9f, g_ring->vf, g_ring->tDead * 1e9f, g_ring->rHs * 1e3f, g_ring->rLs * 1e3f,
               g_ring->tSw * 1e9f);
    }
    if (sc == "ring-coil" || sc == "ring-coil-weak") return runRingCoil(name, sc == "ring-coil-weak");
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

    if (sc == "idle-wake") return runIdleWake();
    settle(sc == "d-noise" ? 60 : 60);
    size_t settleTrips = g_trips.size();
    printf("# trips during settle: %zu\n", settleTrips);

    if (sc == "a" || sc == "b" || sc == "c" || sc == "e" || sc == "e-bleed") {
        const bool e = sc[0] == 'e';
        Probe p = probeStart();
        // e-bleed: open battery but a 1 kOhm residual output load (hypothetical divider/aux drain)
        g_vconv.setBat(0, sc == "e-bleed" ? 1e3f : 1e9f);
        printf("%9.4f >>> vconv bat open (r=%g)\n", tNow(), g_vconv.getRbat());
        if (const char *ph = getenv("SIM_PHASE_US")) g_vconv.stepSeconds(atof(ph) * 1e-6f, 39000);
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
    } else if (rcal) {
        return runRectCal(name, sc);
    } else {
        printf("unknown scenario %s\n", sc.c_str());
        return 1;
    }

    printf("\n== log after event (first %zu lines):\n", g_logSince.size());
    for (size_t i = 0; i < g_logSince.size() && i < 60; ++i) printf("   %s\n", g_logSince[i].c_str());
    return 0;
}

int main(int argc, char **argv) {
    g_simRtAlloc.now = tNow;
    if (const char *e = getenv("SIM_NOALLOC")) g_noAllocAfter = atof(e);
    int rc = simMain(argc, argv);
    auto &a = g_simRtAlloc;
    printf("== rt-alloc: %llu allocs, %llu frees in the RT iteration (%s)", (unsigned long long) a.allocs,
           (unsigned long long) a.frees, a.mallocHooked ? "new/delete + malloc zone" : "new/delete only");
    if (g_noAllocAfter >= 0)
        printf("; after %.0f s: %llu allocs, %llu frees, %d sites", g_noAllocAfter, (unsigned long long) a.armedAllocs,
               (unsigned long long) a.armedFrees, a.nSites);
    printf("\n");
    fflush(stdout);
    if (g_noAllocAfter >= 0 && (a.armedAllocs || a.armedFrees)) return rc ? rc : 3;
    return rc;
}
