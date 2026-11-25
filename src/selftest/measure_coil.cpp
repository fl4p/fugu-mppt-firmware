#include "measure_coil.h"

#include <Arduino.h>
#include <atomic>
#include <cmath>

#include "../logging.h"
#include "../conf.h"
#include "../util.h"
#include "../buck.h"
#include "../mppt.h"
#include "../adc/sampling.h"
#include "../math/asym_peak_fit.h"
#include "coil_fit.h"

// Components owned by main.cpp (same extern coupling as cli.cpp; this TU is built only with the
// real main, never the test mains).
extern SynchronousConverter converter;
extern MpptController mppt;
extern VIinVout<const Sensor *> sensors;

static constexpr int MEAS_MAX_PTS = coilfit::kMaxPts;

struct MeasArgs {
    bool ls;        // false = L0 duty sweep, true = LS-timing sweep
    bool apply;
    int arg1;       // L0: steps; LS: hs (0 = auto)
    uint32_t dwellMs;
    uint32_t fHz;   // L0: switching frequency for the sweep, 0 = keep
};

static std::atomic<bool> s_measureBusy{false};
static MeasArgs s_measArgs{};
// boost: output (HV) and input (coil) current trips of the run, 0 = none; s_tripped: one fired
static constexpr float kBoostVoutMax = 80, kBoostIinMax = 8;
static float s_hvTrip = 0, s_iTrip = 0;
static bool s_tripped = false;

// the coil current: the output's in a buck, the input's in a boost
static const Sensor *coilI() { return converter.boost() ? sensors.Iin : sensors.Iout; }

// boost trip check, then sleep ~ms; false: tripped (PWM stopped by the RT loop at once)
static bool waitMs(uint32_t ms) {
    for (const time_ms dl = wallClockMs() + ms;;) {
        if (!s_tripped && s_hvTrip > 0 &&
            (sensors.Vout->med3.get() > s_hvTrip || coilI()->med3.get() > s_iTrip)) {
            s_tripped = true;
            (void) mppt.requestPsuManual(0, -1, false, true);
            UART_LOG("measure-coil: TRIP Vout %.2f (max %.1f) Iin %.2f A (max %.1f), stopped", sensors.Vout->med3.get(),
                     s_hvTrip, coilI()->med3.get(), s_iTrip);
        }
        if (s_tripped) return false;
        if (wallClockMs() >= dl) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool waitPsuCommand(uint32_t ticket, uint32_t timeoutMs = 1000) {
    const auto deadline = wallClockMs() + timeoutMs;
    while (!mppt.isPsuCommandDone(ticket) && wallClockMs() < deadline)
        vTaskDelay(pdMS_TO_TICKS(1));
    if (mppt.isPsuCommandDone(ticket)) return true;
    if (mppt.cancelPsuCommand(ticket)) return false;
    for (int i = 0; i < 10 && !mppt.isPsuCommandDone(ticket); ++i)
        vTaskDelay(pdMS_TO_TICKS(1));
    return mppt.isPsuCommandDone(ticket);
}

struct Reading { float vin, vout, iout; uint16_t H; bool dcm; };

// Insertion sort then return the middle element (sorts the array in place).
static float medianN(float *a, int n) {
    for (int i = 1; i < n; ++i) {
        float v = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; --j; }
        a[j + 1] = v;
    }
    return n ? a[n / 2] : NAN;
}

// Set the duty target, wait for the RT fade to arrive (or stall), settle, then median 5 EWM reads.
static void settleAndRead(uint16_t H, uint32_t dwellMs, Reading &out) {
    mppt.setManualTarget(H);
    time_ms deadline = wallClockMs() + max<uint32_t>(dwellMs, 3000) + (uint32_t) H * 4;
    while (converter.getCtrlOnPwmCnt() != H && wallClockMs() < deadline && waitMs(20)) {}
    waitMs(dwellMs);
    float vi[5], vo[5], io[5];
    for (int k = 0; k < 5; ++k) {
        vi[k] = sensors.Vin->ewm.avg.get();
        vo[k] = sensors.Vout->ewm.avg.get();
        io[k] = coilI()->ewm.avg.get();
        waitMs(50);
    }
    out = {medianN(vi, 5), medianN(vo, 5), medianN(io, 5), converter.getCtrlOnPwmCnt(), converter.inDCM()};
}

// coil current with the converter off: ramp down, settle, median of 5 EWM reads (NAN: still on)
static float readZero(uint32_t dwellMs) {
    mppt.setManualTarget(0);
    const time_ms deadline = wallClockMs() + 10000;
    while (!converter.disabled() && wallClockMs() < deadline)
        vTaskDelay(pdMS_TO_TICKS(10));
    if (!converter.disabled()) return NAN;
    vTaskDelay(pdMS_TO_TICKS(dwellMs));
    float io[5];
    for (int k = 0; k < 5; ++k) {
        io[k] = coilI()->ewm.avg.get();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return medianN(io, 5);
}

// tag: "" or " <which fit>", inserted after "L(Ipk)"
static void logSplit(const coilfit::Result &f, const char *tag) {
    if (f.k != f.k) return;
    char b[240];
    int o = snprintf(b, sizeof(b), "measure-coil: L(Ipk)%s, +-noise/offsets:", tag);
    for (int i = 0; i < f.nb && o < (int) sizeof(b); ++i)
        o += snprintf(b + o, sizeof(b) - o, "%s %.1f-%.1fA %.2fuH+-%.1f/%.1f%%", i ? " |" : "", f.lo[i], f.hi[i],
                      f.Lb[i] * 1e6f, f.seb[i] * 100.0f, f.sysb[i] * 100.0f);
    UART_LOG("%s", b);
    const float w = f.ksys + 2 * f.kse, lo = f.k - w;
    snprintf(b, sizeof(b), lo < 0 ? "drop <= %.1f %%/A" : "no drop resolved", -lo * 100.0f);
    UART_LOG("measure-coil: L(Ipk)%s slope %+.2f +-%.2f %%/A over %.1f-%.1f A, Iout/on-time offsets add up to +-%.2f: "
             "coil %+.2f..%+.2f %%/A, %s", tag, f.k * 100.0f, f.kse * 100.0f, f.lo[0], f.hi[f.nb - 1], f.ksys * 100.0f,
             lo * 100.0f, (f.k + w) * 100.0f, b);
}

static bool measWriteConf(const char *key, const char *val) {
    ConfFile conf{"/littlefs/conf/coil.conf"};
    return conf.add({{key, val}}, true);
}

// switching frequency through the console's runtime path (RT core applies it, duty counts rescaled)
static bool setFsw(uint32_t hz) {
    auto idle = [] {
        const int64_t dl = esp_timer_get_time() + 2000000;
        while (!converter.pwmFreqIdle() && esp_timer_get_time() < dl) vTaskDelay(1);
        return converter.pwmFreqIdle();
    };
    uint16_t ticks = 0;
    // ripple floor 0.5 ohm: the sweep stays in DCM and stops its rows at the peak-current limit
    const char *e = idle() ? converter.requestPwmFrequency(hz, ticks, 0.5f) : "a frequency change is still being applied";
    if (!e && !(idle() && converter.getPeriodTicks() == ticks && converter.getPwmFrequency() == hz))
        e = "not confirmed by the RT loop";
    if (e) UART_LOG("measure-coil: fsw %lu Hz: %s", (unsigned long) hz, e);
    return !e;
}

// returns the inductance to persist (NAN: none). Boost: the LV input (battery) drives the HV output, whose load sets the
// operating point (a dark PV string at night); rows map onto the buck relations through coilfit::asBuck().
static float sweepL0(const MeasArgs &a) {
    const bool boost = converter.boost();
    float vin0 = sensors.Vin->ewm.avg.get(), vout0 = sensors.Vout->ewm.avg.get();
    // boost: the band is the buck one at the output trip, the live Vout ends it (no fixed voltage target)
    if (boost ? !(vin0 > 5.0f && vin0 < 0.8f * s_hvTrip && vout0 < s_hvTrip - 2.0f) : vin0 <= vout0 + 1.0f) {
        UART_LOG(boost ? "measure-coil: need 5<Vin<%.1f, Vout<%.1f (Vin=%.2f Vout=%.2f)" : "measure-coil: need Vin>Vout+1 "
                 "(Vin=%.2f Vout=%.2f)", boost ? 0.8f * s_hvTrip : vin0, boost ? s_hvTrip - 2.0f : vout0, vin0, vout0);
        return NAN;
    }
    if (a.fHz && !setFsw(a.fHz)) return NAN;
    // after the frequency change: pwmMax, pwmCtrlMax and the CCM/DCM basis fL follow it
    const float fsw = (float) converter.realizedFreqHz();
    const uint16_t pwmMax = converter.pwmMaxDriver();
    // boost: under the 6 A trip without sync rect (protect(): highI-noSyncRect)
    const float iMaxLim = boost ? min(5.0f, 0.75f * s_iTrip) : min(2.0f, mppt.limits.Iout_max);
    const float ipkMax = boost ? mppt.limits.Iin_max : mppt.limits.Iout_max;
    int steps = a.arg1 > 0 ? min(a.arg1, MEAS_MAX_PTS) : 48;
    int h_lo, h_hi, step;
    float bvi = boost ? s_hvTrip : vin0, bvo = boost ? s_hvTrip - vin0 : vout0;
    if (!coilfit::band(bvi, bvo, pwmMax, converter.pwmCtrlMax, steps, h_lo, h_hi, step)) {
        UART_LOG("measure-coil: empty duty band");
        return NAN;
    }

    UART_LOG("measure-coil L0: fsw=%.0f pwmMax=%u band %d..%d step %d  Vin=%.2f Vout=%.2f M=%.3f%s",
             fsw, pwmMax, h_lo, h_hi, step, vin0, vout0, vout0 / vin0, boost ? " boost" : "");
    const uint32_t zDwell = max<uint32_t>(a.dwellMs, 2000);
    if (boost) { // the battery feeds the coil through the backflow switch; the output charges to it through the HS diode
        // the switch's gate pump runs off the bootstrap supply: switch at one refresh pulse (2 us LS) per period
        mppt.bflow.enable(true);
        const auto hPre = (uint16_t) max(2, min((int) ceilf(2e-6f * fsw * pwmMax), converter.pwmCtrlMax / 8));
        auto up = [] { return sensors.Vout->ewm.avg.get() > sensors.Vin->ewm.avg.get() - 3.0f; };
        if (!up()) mppt.setManualTarget(hPre);
        for (const time_ms dl = wallClockMs() + 3000; !up() && wallClockMs() < dl && waitMs(20);) {}
        vout0 = sensors.Vout->ewm.avg.get();
        if (!up() || s_tripped) {
            UART_LOG("measure-coil: Vout %.2f did not follow Vin %.2f with the backflow switch on, 3 s at duty %u; "
                     "switch not conducting?", vout0, sensors.Vin->ewm.avg.get(), (unsigned) hPre);
            return NAN;
        }
    }
    const float z0 = readZero(zDwell);
    if (z0 != z0) { UART_LOG("measure-coil: converter did not stop for the current zero"); return NAN; }
    if (boost) { // a low duty lifts Vout above Vin first: below it the coil does not reset and the duty must not climb
        mppt.setManualTarget((uint16_t) max(2, h_lo / 3));
        for (const time_ms dl = wallClockMs() + 5000; !(sensors.Vout->ewm.avg.get() > sensors.Vin->ewm.avg.get() + 2.0f)
                 && wallClockMs() < dl && waitMs(20);) {}
        waitMs(max<uint32_t>(a.dwellMs, 2000)); // settled: the near-CCM test below judges the next duty by this Vout
        if (!(sensors.Vout->ewm.avg.get() > sensors.Vin->ewm.avg.get() + 2.0f)) {
            UART_LOG("measure-coil: Vout %.2f not above Vin %.2f + 2 at duty %d; stopping", sensors.Vout->ewm.avg.get(),
                     sensors.Vin->ewm.avg.get(), max(2, h_lo / 3));
            h_hi = -1;
        }
    }
    const char *iName = boost ? "Iin" : "Iout";
    UART_LOG("      H      D    Vin   Vout   %4s mode    L_uH", iName);
    float d[MEAS_MAX_PTS], vi[MEAS_MAX_PTS], vo[MEAS_MAX_PTS], io[MEAS_MAX_PTS];
    uint8_t row[MEAS_MAX_PTS]; // sweep position of each point, for the zero interpolation
    int n = 0, rows = 0;
    for (int H = h_lo; H <= h_hi && n < MEAS_MAX_PTS; H += step) {
        const float vin = sensors.Vin->ewm.avg.get(), vout = sensors.Vout->ewm.avg.get();
        if (boost ? !(vout > vin + 2.0f) : !(vin > vout + 1.0f)) {
            UART_LOG(boost ? "  Vout no longer above Vin+2; stopping" : "  Vin no longer above Vout+1; stopping");
            break;
        }
        // DCM ends at D = 1 - Vin/Vout, which moves with the load: stop a step short of 0.9 x it
        if (boost && (float) H / pwmMax > 0.9f * (1 - vin / vout)) { UART_LOG("  near CCM; stopping"); break; }
        // buck: BF closed once forward current flows (output node above the battery, as MPPT closes it): open, its body
        // diode carries the sweep's ~2 A (highD-bflowOpen trips above 1 A) and its drop biases Vout and so L
        if (!boost && sensors.Iout->ewm.avg.get() > 0.1f) mppt.bflow.enable(true);
        Reading r{};
        settleAndRead((uint16_t) H, a.dwellMs, r);
        if (s_tripped || converter.disabled()) { UART_LOG("  converter disabled (protection?); aborting"); break; }
        float D = (float) r.H / converter.periodCounts(), bi = r.vin, bo = r.vout; // period basis, as dcmCurrent()
        if (boost) coilfit::asBuck(bi, bo);
        float L = (r.iout <= 0.02f || bi <= bo || bo <= 0) ? NAN : (bi - bo) * bi * D * D / (2.0f * bo * fsw * r.iout);
        UART_LOG("  %5u %6.3f %6.2f %6.2f %6.3f  %3s %7.2f", r.H, D, r.vin, r.vout, r.iout,
                 r.dcm ? "DCM" : "CCM", L * 1e6f);
        if (r.dcm && L == L) {
            d[n] = D, vi[n] = bi, vo[n] = bo, io[n] = r.iout, row[n] = (uint8_t) rows;
            ++n;
        }
        ++rows;
        if (r.iout > iMaxLim) { UART_LOG("  %s %.2f > %.2f; stopping", iName, r.iout, iMaxLim); break; }
        const float ipk = coilfit::ipkDcm(D, bi, bo, r.iout);
        if (!(ipk <= ipkMax)) { UART_LOG("  Ipk %.2f > %.2f; stopping", ipk, ipkMax); break; }
        if (!r.dcm) { UART_LOG("  entered CCM; stopping"); break; }
        if (boost && r.vout > s_hvTrip - 2.0f) { UART_LOG("  Vout %.2f near %.1f; stopping", r.vout, s_hvTrip); break; }
    }
    const float z1 = readZero(zDwell);
    const bool disturbed = !(fabsf(z1 - z0) <= coilfit::kZeroTol); // also: z1 unread
    const float zsig = coilfit::zeroCorrect(io, n, z0, z1 == z1 ? z1 : z0, row, rows);
    float iMaxSeen = 0;
    for (int i = 0; i < n; ++i) iMaxSeen = max(iMaxSeen, io[i]);
    float ifloor = max(0.15f, 0.2f * iMaxSeen);
    int nk = 0;
    for (int i = 0; i < n; ++i)
        if (io[i] >= ifloor) d[nk] = d[i], vi[nk] = vi[i], vo[nk] = vo[i], io[nk] = io[i], ++nk;
    if (nk < 3) { UART_LOG("measure-coil: only %d DCM pts above %.2f A; widen band or wait for sun", nk, ifloor); return NAN; }
    const float ton = max(coilfit::kOnTimeMin, max(converter.getDeadTimeHlNs(), converter.getDeadTimeLhNs()) * 1e-9f);
    const auto f0 = coilfit::fit(d, vi, vo, io, nk, fsw, zsig, ton);
    int st;
    const auto ft = coilfit::fitOnTime(d, vi, vo, io, nk, fsw, zsig, f0, st);
    const auto &f = st ? f0 : ft;
    if (f.f > 0)
        UART_LOG("measure-coil: L = %.2f uH %s (ring fit, %d DCM pts, %s>=%.2fA, +-%.1f%% noise +-%.1f%% syst; ring %.0f kHz"
                 " = %.1f nF node C, ripple +-%.0f%%, %.1f periods; median of points %.2f uH; uncorrected %.2f uH)", f.L * 1e6f,
                 st ? "uncorrected" : "on-time corrected", f.n, iName, ifloor, f.se * 100.0f, f.sys * 100.0f, f.f * 1e-3f,
                 1e9f / (39.478418f * f.f * f.f * f.L), f.amp * 100.0f, f.periods, f.Lmed * 1e6f, f0.L * 1e6f);
    else
        UART_LOG("measure-coil: L = %.2f uH %s (median of %d DCM pts, %s>=%.2fA; ring not resolved: %s, expect up to +-5%%;"
                 " uncorrected %.2f uH)", f.L * 1e6f, st ? "uncorrected" : "on-time corrected", f.n, iName, ifloor,
                 f.n < coilfit::kMinFitPts ? "too few points" : "no periodic term", f0.L * 1e6f);
    static const char *const why[] = {"applied", "", "not applied: duty span under 2x", "not applied: error over 80 ns",
                                      "offset or error beyond +-400 ns: FAIL, the DCM model does not fit, L not for coil.conf"};
    if (st == 1) UART_LOG("measure-coil: on-time offset not estimated (too few points), L uncorrected");
    else UART_LOG("measure-coil: on-time offset %+.0f ns +-%.0f noise +-%.0f syst (L(Ipk) slope 0 +-%.1f %%/A assumed), "
                  "duty span %.1fx: %s", ft.dl * 1e9f, ft.dlse * 1e9f, ft.dlsys * 1e9f, coilfit::kSlopeTol * 100.0f,
                  ft.dspan, why[st]);
    UART_LOG("measure-coil: %s zero %+.3f/%+.3f A (off, before/after), subtracted%s", iName, z0, z1,
             disturbed ? "; DISTURBED: moved > 0.02 A, L not for coil.conf" : "");
    if (!st) logSplit(ft, " at the fitted on-time offset (slope 0 assumed)");
    logSplit(f0, st ? "" : " uncorrected");
    const float l = coilfit::persistL(f0, ft, st);
    if (st != 4 && l != l) UART_LOG("measure-coil: L implausible, not for coil.conf");
    if (disturbed || l != l) return NAN;
    UART_LOG("  -> coil.conf L0=%.6e   note: scales with Iout cal & period%s", l, st ? "; on-time offset not corrected" : "");
    return l;
}

static void sweepLs(const MeasArgs &a) {
    float fsw = (float) converter.getPwmFrequency();
    uint16_t pwmMax = converter.pwmMaxDriver();
    uint16_t pwmRectMin = converter.getRectOnPwmMin();
    float iMaxLim = min(2.0f, mppt.limits.Iout_max);

    float vin0 = sensors.Vin->ewm.avg.get(), vout0 = sensors.Vout->ewm.avg.get();
    if (vin0 <= vout0 + 1.0f) { UART_LOG("measure-coil: need Vin>Vout+1 (Vin=%.2f Vout=%.2f)", vin0, vout0); return; }
    int hs = a.arg1 > 0 ? a.arg1 : max(2, (int) lroundf(0.25f * (vout0 / vin0) * pwmMax));

    converter.enableSyncRect(true, true);        // LS sweep needs sync rect on regardless of paranoia
    mppt.bflow.enable(true);

    Reading r{};
    settleAndRead((uint16_t) hs, max<uint32_t>(a.dwellMs, 4000), r);
    if (converter.disabled()) { UART_LOG("  converter disabled (protection?); aborting"); return; }
    float vi = r.vin, vo = r.vout;
    if (vi <= vo || !r.dcm) {
        UART_LOG("measure-coil: need DCM with Vin>Vout (got %s Vin=%.2f Vout=%.2f); lower hs",
                 r.dcm ? "DCM" : "CCM", vi, vo);
        return;
    }
    uint16_t auto_ls = converter.getRectOnPwmCnt();
    float ideal_ls = (vi / vo - 1.0f) * hs;
    int ls_lo = max((int) pwmRectMin, (int) lroundf(0.5f * ideal_ls));
    int ls_hi = min((int) (pwmMax - hs), (int) lroundf(1.4f * ideal_ls));
    if (ls_hi <= ls_lo) { UART_LOG("measure-coil: empty LS band (%d..%d)", ls_lo, ls_hi); return; }
    int step = max(1, (ls_hi - ls_lo) / 23);     // 24 steps

    UART_LOG("measure-coil LS: HS=%d D=%.3f Vin=%.2f Vout=%.2f ideal_LS=%.0f auto_LS=%u sweep %d..%d",
             hs, (float) hs / pwmMax, vi, vo, ideal_ls, auto_ls, ls_lo, ls_hi);
    UART_LOG("     LS    Iout    Vin   Vout mode");

    float Xls[MEAS_MAX_PTS], Yio[MEAS_MAX_PTS];
    int n = 0;
    float peak_io = -1e9f;
    for (int ls = ls_lo; ls <= ls_hi && n < MEAS_MAX_PTS; ls += step) {
        converter.setManualRect(ls);
        vTaskDelay(pdMS_TO_TICKS(a.dwellMs));
        if (converter.disabled()) { UART_LOG("  converter disabled (protection?); aborting"); break; }
        float ii[5], vv[5], oo[5];
        for (int k = 0; k < 5; ++k) {
            ii[k] = sensors.Iout->ewm.avg.get();
            vv[k] = sensors.Vin->ewm.avg.get();
            oo[k] = sensors.Vout->ewm.avg.get();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        float io = medianN(ii, 5);
        uint16_t rect = converter.getRectOnPwmCnt();
        Xls[n] = rect; Yio[n] = io; ++n;
        UART_LOG("  %5u %7.3f %6.2f %6.2f  %3s", rect, io, medianN(vv, 5), medianN(oo, 5),
                 converter.inDCM() ? "DCM" : "CCM");
        if (io > peak_io) peak_io = io;
        if (io > iMaxLim) { UART_LOG("  Iout %.2f > %.2f; stopping", io, iMaxLim); break; }
        if (peak_io > 0.05f && io < 0.8f * peak_io && rect > ideal_ls) { UART_LOG("  passed peak; stopping"); break; }
    }
    if (n < 4) { UART_LOG("measure-coil: too few points to locate peak"); return; }
    int pk = 0;
    for (int i = 1; i < n; ++i) if (Yio[i] > Yio[pk]) pk = i;
    // The peak is strongly asymmetric (shallow body-diode side, steep reverse-current side); a single
    // parabola is dragged toward the flat side.
    float ls_peak = Xls[pk], Lc = NAN;
    double bq = 0;
    bool fitted = asymPeakFit(Xls, Yio, n, ls_peak, bq);
    if (fitted && bq > 0)  // steep side: reverse charge ~ Vo*Vin/(2L(Vin-Vo)) * t^2 per period (V_f neglected)
        Lc = vo * vi / (2.0f * (float) bq * fsw * (float) pwmMax * (float) pwmMax * (vi - vo));
    UART_LOG("measure-coil: peak LS %.0f (Iout_peak %.3f A at raw argmax %.0f%s)", ls_peak, Yio[pk], Xls[pk],
             fitted ? ", two-sided fit" : ", fit failed: raw argmax");
    UART_LOG("  ideal LS %.0f  auto LS %u  offset peak-ideal %+.0f ct (%+.1f%% HS)  peak-auto %+.0f ct",
             ideal_ls, auto_ls, ls_peak - ideal_ls, (ls_peak - ideal_ls) / hs * 100.0f, ls_peak - auto_ls);
    if (Lc == Lc) UART_LOG("  L (peak curvature) = %.1f uH (cross-check)", Lc * 1e6f);
    if (a.apply) {
        if (!(pk > 0 && pk < n - 1)) { UART_LOG("  apply skipped: peak at sweep edge; widen sweep"); return; }
        const int lim = pwmMax / 8, hi = rectOffsetCountsFromNs(RectOffsetLateMaxNs, converter.getPwmTickRate());
        const int raw = (int) lroundf(ls_peak - ideal_ls - 12.0f), new_off = constrain(raw, -lim, hi);
        mppt.rectCal.post(new_off, true); // applied by the RT loop
        mppt.rectCal.requestSave();       // coil.conf once the converter is off
        converter.setRectOffsetCal(raw > hi ? 2 : 1);
        UART_LOG("  applied rect_offset %.0f ns (%d ct, %s); saved to coil.conf once the converter is off",
                 rectOffsetNsFromCounts(new_off, converter.getPwmTickRate()), new_off, converter.drvName());
    }
}

static void measureCoilTask(void *arg) {
    auto a = *(const MeasArgs *) arg;
    if (!a.dwellMs) a.dwellMs = a.ls ? 3000 : 1000; // L0: 48 x ~1.3 s, about a minute
    const uint32_t fNom = converter.getPwmFrequency();
    float l0 = NAN;
    const bool boost = converter.boost();
    const float ov0 = mppt.getExplicitOvLimit();
    s_tripped = false;
    s_iTrip = min(kBoostIinMax, mppt.limits.Iin_max);
    s_hvTrip = boost ? min(kBoostVoutMax, 0.95f * mppt.limits.Vout_max) : 0;
    if (boost) mppt.setExplicitOvLimit(fminf(ov0, s_hvTrip)); // the RT loop's per-sample Vout-OV trip
    // started from `dc 0` only (console); manual mode keeps MPPT, its sweeps and starts off until `mppt`
    const auto enterTicket = mppt.requestPsuManual(0, -1, false, true);
    const bool entered = waitPsuCommand(enterTicket);
    for (const time_ms dl = wallClockMs() + 10000; entered && wallClockMs() < dl;) { // let the filters drain
        if (converter.disabled() && coilI() && fabsf(coilI()->ewm.avg.get()) < 1.f) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!entered) {
        UART_LOG("measure-coil: RT transition timed out");
    } else if (!sensors.Vin || !sensors.Vout || !coilI()) {
        UART_LOG("measure-coil: missing Vin/Vout/I sensor");
    } else {
        if (a.ls) sweepLs(a); else l0 = sweepL0(a);
    }
    converter.setManualRect(-1);
    mppt.setManualTarget(0); // RT core ramps down and disables (avoids cross-core LEDC race)
    // pwmMax/512 counts per sample: ~0.4 s from the top of the band at 450 sps, longer on a slow sampler
    time_ms doneDeadline = wallClockMs() + 10000;
    while (!converter.disabled() && wallClockMs() < doneDeadline)
        vTaskDelay(pdMS_TO_TICKS(10));
    const bool off = converter.disabled();
    if (boost) mppt.setExplicitOvLimit(ov0);
    s_hvTrip = 0;
    if (a.fHz && !(converter.pwmFreqIdle() && converter.getPwmFrequency() == fNom)) {
        bool ok = false;
        for (int i = 0; i < 3 && !ok; ++i) ok = setFsw(fNom);
        if (ok) UART_LOG("measure-coil: fsw restored to %lu Hz", (unsigned long) fNom);
        else UART_LOG("measure-coil: fsw %lu Hz restore failed, now %lu Hz: run `pwm-freq %lu`", (unsigned long) fNom,
                      (unsigned long) converter.getPwmFrequency(), (unsigned long) fNom);
    }
    if (off && mppt.limits.reverse_current_paranoia) mppt.bflow.enable(false);
    if (!off) UART_LOG("measure-coil: converter still on after the ramp-down");
    if (a.apply && l0 == l0 && !off) UART_LOG("  measure-coil: L0 not applied (converter still on)");
    else if (a.apply && l0 == l0) { // flash write with the converter off (it stalls the RT loop)
        char buf[24];
        snprintf(buf, sizeof(buf), "%.6e", l0);
        if (measWriteConf("L0", buf)) UART_LOG("  applied coil.conf L0=%s (effective next boot)", buf);
        else UART_LOG("  measure-coil: FAILED to write coil.conf L0=%s", buf);
    }
    s_measureBusy.store(false);
    UART_LOG("measure-coil: done, manual PWM at duty 0 ('mppt' resumes tracking)");
    vTaskDelete(nullptr);
}

bool measureCoilStart(bool ls, bool apply, int arg1, uint32_t dwellMs, uint32_t fHz) {
    // The sweeps below snapshot pwmMax/pwmCtrlMax/pwmRectMin once and run for seconds. A dead-time
    // request that is still queued (console `dt` timed out but did not withdraw it) would land
    // mid-sweep and move all three under those copies, silently corrupting an L0 we may persist.
    if (!converter.deadTimeIdle() || !converter.pwmFreqIdle()) return false;
    if (s_measureBusy.exchange(true)) return false;
    s_measArgs = {ls, apply, arg1, dwellMs, ls ? 0 : fHz};
    xTaskCreatePinnedToCore(measureCoilTask, "meas-coil", 6144, &s_measArgs, 1, nullptr, NON_RT_CORE);
    return true;
}

bool isMeasuring() { return s_measureBusy; }
