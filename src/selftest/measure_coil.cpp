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
#include "../app_state.h"

// Components owned by main.cpp (same extern coupling as cli.cpp; this TU is built only with the
// real main, never the test mains). Mode flags live in g_app (app_state.h).
extern SynchronousConverter converter;
extern MpptController mppt;
extern VIinVout<const Sensor *> sensors;

static constexpr int MEAS_MAX_PTS = 64;

struct MeasArgs {
    bool ls;        // false = L0 duty sweep, true = LS-timing sweep
    bool apply;
    int arg1;       // L0: steps; LS: hs (0 = auto)
    uint32_t dwellMs;
};

static std::atomic<bool> s_measureBusy{false};
static MeasArgs s_measArgs{};

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
    while (converter.getCtrlOnPwmCnt() != H && wallClockMs() < deadline)
        vTaskDelay(pdMS_TO_TICKS(20));
    vTaskDelay(pdMS_TO_TICKS(dwellMs));
    float vi[5], vo[5], io[5];
    for (int k = 0; k < 5; ++k) {
        vi[k] = sensors.Vin->ewm.avg.get();
        vo[k] = sensors.Vout->ewm.avg.get();
        io[k] = sensors.Iout->ewm.avg.get();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    out = {medianN(vi, 5), medianN(vo, 5), medianN(io, 5), converter.getCtrlOnPwmCnt(), converter.inDCM()};
}

// Least-squares y = a + b*x + c*x^2 via 3x3 Gaussian elimination; false if singular.
static bool quadfit3(const float *xs, const float *ys, int n, double &a, double &b, double &c) {
    if (n < 3) return false;
    double s[5] = {0, 0, 0, 0, 0}, rhs[3] = {0, 0, 0};
    for (int i = 0; i < n; ++i) {
        double x = xs[i], y = ys[i], xp = 1;
        for (int k = 0; k < 5; ++k) { s[k] += xp; xp *= x; }
        rhs[0] += y; rhs[1] += x * y; rhs[2] += x * x * y;
    }
    double M[3][4] = {{s[0], s[1], s[2], rhs[0]}, {s[1], s[2], s[3], rhs[1]}, {s[2], s[3], s[4], rhs[2]}};
    for (int i = 0; i < 3; ++i) {
        double p = M[i][i];
        if (fabs(p) < 1e-12) return false;
        for (int j = 0; j < 4; ++j) M[i][j] /= p;
        for (int k = 0; k < 3; ++k)
            if (k != i) {
                double f = M[k][i];
                for (int j = 0; j < 4; ++j) M[k][j] -= f * M[i][j];
            }
    }
    a = M[0][3]; b = M[1][3]; c = M[2][3];
    return true;
}

static void measWriteConf(const char *key, const char *val) {
    ConfFile conf{"/littlefs/conf/coil.conf"};
    conf.add({{key, val}}, true);
}

static void sweepL0(const MeasArgs &a) {
    float fsw = (float) converter.getPwmFrequency();
    uint16_t pwmMax = converter.pwmMaxDriver();
    uint16_t pwmCtrlMax = converter.pwmCtrlMax;
    float iMaxLim = min(2.0f, mppt.limits.Iout_max);

    float vin0 = sensors.Vin->ewm.avg.get(), vout0 = sensors.Vout->ewm.avg.get();
    if (vin0 <= vout0 + 1.0f) { UART_LOG("measure-coil: need Vin>Vout+1 (Vin=%.2f Vout=%.2f)", vin0, vout0); return; }
    float M = vout0 / vin0;
    int steps = a.arg1 > 0 ? a.arg1 : 10;
    if (steps > MEAS_MAX_PTS) steps = MEAS_MAX_PTS;
    int h_lo = max(2, (int) lroundf(0.25f * M * pwmMax));
    int h_hi = min((int) pwmCtrlMax, (int) lroundf(0.9f * M * pwmMax));
    if (h_hi <= h_lo) { UART_LOG("measure-coil: empty duty band (%d..%d)", h_lo, h_hi); return; }
    int step = max(1, (h_hi - h_lo) / max(1, steps - 1));

    UART_LOG("measure-coil L0: fsw=%.0f pwmMax=%u band %d..%d step %d  Vin=%.2f Vout=%.2f M=%.3f",
             fsw, pwmMax, h_lo, h_hi, step, vin0, vout0, M);
    UART_LOG("      H      D    Vin   Vout   Iout mode    L_uH");

    float Ls[MEAS_MAX_PTS], Io[MEAS_MAX_PTS];
    int n = 0;
    float iMaxSeen = 0;
    for (int H = h_lo; H <= h_hi && n < MEAS_MAX_PTS; H += step) {
        Reading r{};
        settleAndRead((uint16_t) H, a.dwellMs, r);
        if (converter.disabled()) { UART_LOG("  converter disabled (protection?); aborting"); break; }
        float D = (float) r.H / pwmMax;
        float L = (r.iout <= 0.02f || r.vin <= r.vout) ? NAN
                  : (r.vin - r.vout) * r.vin * D * D / (2.0f * r.vout * fsw * r.iout);
        UART_LOG("  %5u %6.3f %6.2f %6.2f %6.3f  %3s %7.2f", r.H, D, r.vin, r.vout, r.iout,
                 r.dcm ? "DCM" : "CCM", L * 1e6f);
        if (r.dcm && L == L) { Ls[n] = L; Io[n] = r.iout; ++n; if (r.iout > iMaxSeen) iMaxSeen = r.iout; }
        if (r.iout > iMaxLim) { UART_LOG("  Iout %.2f > %.2f; stopping", r.iout, iMaxLim); break; }
        if (!r.dcm) { UART_LOG("  entered CCM; stopping"); break; }
    }
    float ifloor = max(0.15f, 0.2f * iMaxSeen);
    float kept[MEAS_MAX_PTS];
    int nk = 0;
    for (int i = 0; i < n; ++i) if (Io[i] >= ifloor) kept[nk++] = Ls[i];
    if (nk < 3) { UART_LOG("measure-coil: only %d DCM pts above %.2f A; widen band or wait for sun", nk, ifloor); return; }
    float med = medianN(kept, nk);              // sorts kept[]
    float q1 = kept[nk / 4], q3 = kept[(3 * nk) / 4];
    UART_LOG("measure-coil: L = %.2f uH  (%d DCM pts, Iout>=%.2fA, IQR %.0f%%)", med * 1e6f, nk, ifloor,
             (q3 - q1) / med * 100.0f);
    UART_LOG("  -> coil.conf L0=%.6e   note: scales with Iout cal & pwmMax; dead-time/DCR ~few %% low", med);
    if (a.apply) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%.6e", med);
        measWriteConf("L0", buf);
        UART_LOG("  applied coil.conf L0=%s (effective next boot)", buf);
    }
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
    float ls_peak = Xls[pk], Lc = NAN;
    float thr = 0.9f * Yio[pk];
    int lo = pk; while (lo > 0 && Yio[lo - 1] >= thr) --lo;
    int hi = pk; while (hi < n - 1 && Yio[hi + 1] >= thr) ++hi;
    int wn = hi - lo + 1;
    if (wn >= 5) {
        float mx = 0;
        for (int i = lo; i <= hi; ++i) mx += Xls[i];
        mx /= wn;
        float wx[MEAS_MAX_PTS];
        for (int i = 0; i < wn; ++i) wx[i] = Xls[lo + i] - mx;
        double aa, bb, cc;
        if (quadfit3(wx, &Yio[lo], wn, aa, bb, cc) && cc < 0) {
            double vtx = mx - bb / (2 * cc);
            if (Xls[lo] <= vtx && vtx <= Xls[hi]) {
                ls_peak = (float) vtx;
                Lc = vo / (2.0f * (float) (-cc) * fsw * (float) pwmMax * (float) pwmMax);
            }
        }
    }
    UART_LOG("measure-coil: peak LS %.0f (Iout_peak %.3f A%s)", ls_peak, Yio[pk], Lc == Lc ? ", parabola" : ", raw argmax");
    UART_LOG("  ideal LS %.0f  auto LS %u  offset peak-ideal %+.0f ct (%+.1f%% HS)  peak-auto %+.0f ct",
             ideal_ls, auto_ls, ls_peak - ideal_ls, (ls_peak - ideal_ls) / hs * 100.0f, ls_peak - auto_ls);
    if (Lc == Lc) UART_LOG("  L (peak curvature) = %.1f uH (cross-check)", Lc * 1e6f);
    if (a.apply) {
        if (!(pk > 0 && pk < n - 1)) { UART_LOG("  apply skipped: peak at sweep edge; widen sweep"); return; }
        int lim = pwmMax / 8;
        int new_off = constrain((int) lroundf(ls_peak - ideal_ls - 12.0f), -lim, lim);
        converter.setRectOnOffset(new_off);   // live (counts)
        // Persist as a time so the calibration survives a driver-resolution change.
        float ns = rectOffsetNsFromCounts(new_off, converter.getPwmTickRate());
        char buf[16];
        snprintf(buf, sizeof(buf), "%.0f", ns);
        ConfFile conf{"/littlefs/conf/coil.conf"};
        conf.remove("rect_offset");           // drop any legacy count-based key
        conf.add({{"rect_offset_ns", buf}}, true);
        UART_LOG("  applied coil.conf rect_offset_ns=%s (%d ct, effective now + next boot)", buf, new_off);
    }
}

static void measureCoilTask(void *arg) {
    auto a = *(const MeasArgs *) arg;
    auto savedMode = g_app.opMode;
    if (!sensors.Vin || !sensors.Vout || !sensors.Iout) {
        UART_LOG("measure-coil: missing Vin/Vout/Iout sensor");
    } else {
        g_app.opMode = OpMode::Manual;
        if (!mppt.limits.reverse_current_paranoia) {
            converter.enableSyncRect(true);
            mppt.bflow.enable(true);
        }
        if (a.ls) sweepLs(a); else sweepL0(a);
    }
    converter.setManualRect(-1);
    mppt.setManualTarget(0); // RT core ramps down and disables (avoids cross-core LEDC race)
    time_ms doneDeadline = wallClockMs() + 2000;
    while (!converter.disabled() && wallClockMs() < doneDeadline)
        vTaskDelay(pdMS_TO_TICKS(10));
    mppt.clearBootTarget();
    if (savedMode == OpMode::Psu)
        mppt.setPsuSetpoint(mppt.psuVsetpoint);
    g_app.opMode = savedMode;
    s_measureBusy.store(false);
    UART_LOG("measure-coil: done, mode restored");
    vTaskDelete(nullptr);
}

bool measureCoilStart(bool ls, bool apply, int arg1, uint32_t dwellMs) {
    if (s_measureBusy.exchange(true)) return false;
    s_measArgs = {ls, apply, arg1, dwellMs};
    xTaskCreatePinnedToCore(measureCoilTask, "meas-coil", 6144, &s_measArgs, 1, nullptr, NON_RT_CORE);
    return true;
}

bool isMeasuring() { return s_measureBusy; }