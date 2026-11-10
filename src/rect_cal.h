#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <cstdio>

#include "esp_log.h"
#include "math/asym_peak_fit.h"
#include "pwm/mcpwm_timing.h"

/**
 * Self-calibration of the buck DCM low-side timing offset (rect_offset).
 *
 * HS is lowered to HsFrac*M of the period and held there: a low DCM point draws well below the
 * panel's MPP current, so Vin stays on the flat, voltage-source part of the PV curve and does not
 * move with the LS window (at an MPPT operating point it does, and the peak shifts by hundreds of
 * counts). Then the LS count is stepped from 0.5x to 1.4x the ideal DCM freewheel count
 * ideal = (Vin/Vout - 1) * HS. Output current peaks where the LS turns off at the inductor zero
 * crossing: shorter leaves the tail to the body diode, longer draws reverse current. Each step is
 * measured against a reference LS (0.6x ideal) sampled before and after it, which cancels source
 * drift, and is placed relative to the ideal count of its own Vin/Vout, so a voltage change does
 * not read as a timing error. Vin and Vout must stay within StiffV of their Init values.
 * learned = peak - ideal (counts) is the net timing error of driver, gate and sensing.
 *
 * Ownership: the RT core writes the payload in start()/refuse() (from Idle) and during Run, core 0
 * in Fit. Each hand-off is a release store of `state`; the other side reads it with acquire before
 * touching the payload. Only core 0 leaves Done/Fail (requestStart), so a result stays stable while
 * core 0 reads it.
 */
class RectCal {
public:
    enum State : uint8_t { Idle, Run, Fit, Done, Fail };

    static constexpr int Steps = 24;
    static constexpr uint32_t SettleUs = 150000, InitUs = 500000, RefUs = 250000, MeasUs = 500000;
    static constexpr uint32_t TimeoutUs = 32000000;
    static constexpr float LsLo = 0.5f, LsHi = 1.4f, LsRef = 0.6f, HsFrac = 0.55f;
    static constexpr float PlausibleNs = 2500.f; // |learned| above this is rejected (400 ct at 160 MHz)
    static constexpr float MinMarginNs = 125.f;  // applied margin floor (20 ct at 160 MHz)
    static constexpr float StiffV = 0.02f;       // max Vin and Vout deviation from Init (fraction)

    std::atomic<uint8_t> state{Idle};
    const char *why = ""; // failure reason
    uint16_t hs = 0;      // 0 after a refused start
    uint8_t n = 0, seq = 0; // seq: counts starts and refusals
    int16_t learned = 0, applied = 0;
    float ideal = 0, peak = 0; // peak: fitted LS count of the current maximum
    float dv = 0; // largest relative Vin/Vout deviation from Init
    float xs[Steps]{}, ys[Steps]{};

    [[nodiscard]] State get() const { return (State) state.load(std::memory_order_acquire); }
    [[nodiscard]] bool running() const { return get() == Run; }

    // HS count of the calibration point for buck ratio M = vout/vin.
    static uint16_t calHs(float vin, float vout, uint16_t pwmMax) {
        return (uint16_t) std::lround(HsFrac * vout / vin * (float) pwmMax);
    }

    // Lowest offset fit() can apply: -(plausible + its margin). Also the backstop floor.
    static int floorCounts(float tickRate) {
        const int p = pwmCountsFromNs(PlausibleNs, tickRate);
        return -(int) std::lround((float) p + std::max((float) pwmCountsFromNs(MinMarginNs, tickRate), 0.3f * (float) p));
    }

    // HS step towards the calibration point, limited to maxStep; 0 once there.
    [[nodiscard]] int hsStep(uint16_t hsNow, int maxStep) const {
        return running() ? std::clamp((int) hs - (int) hsNow, -maxStep, maxStep) : 0;
    }

    void start(uint32_t nowUs, uint16_t hs_, uint16_t lsMin_, uint16_t lsCap_) {
        hs = hs_; lsMin = lsMin_; lsCap = lsCap_;
        n = 0; k = 0; ideal = peak = dv = 0; ymax = -INFINITY; learned = applied = 0; why = "";
        phase = Init; ls = -1;
        t0 = tPh = nowUs;
        clearAcc();
        ++seq;
        state.store(Run, std::memory_order_release);
    }

    void refuse(const char *reason) {
        hs = 0; n = 0; ++seq;
        fail(reason);
    }

    void abort(const char *reason) {
        if (running()) fail(reason);
    }

    // Returns the LS count to hold, -1 to leave LS to diode emulation. The sweep starts once HS
    // has reached the calibration point.
    int feed(uint32_t nowUs, uint16_t hsNow, float vin, float vout, float iout) {
        if (!running()) return -1;
        if (nowUs - t0 > TimeoutUs) return fail("timeout");
        if (hsNow != hs) {
            if (phase != Init) return fail("HS moved");
            tPh = nowUs;
            clearAcc();
            return ls;
        }
        const uint32_t dt = nowUs - tPh;
        if (dt < SettleUs) return ls;
        si += iout; sv += vin; so += vout; ++ns;
        if (dt < SettleUs + (phase == Init ? InitUs : phase == Ref ? RefUs : MeasUs)) return ls;

        const float i = si / (float) ns, vi = sv / (float) ns, vo = so / (float) ns;
        if (!std::isfinite(i + vi + vo)) return fail("bad sample");
        if (phase == Init) {
            if (!(vo > 0.5f && vi > vo + 1.f)) return fail("Vin<=Vout");
            if (!(i > 0.15f)) return fail("low I");
            vi0 = vi; vo0 = vo;
            ideal = (vi / vo - 1.f) * (float) hs;
            if ((float) lsCap < 1.15f * ideal) return fail("LS headroom");
            ref = (uint16_t) std::max<float>(lsMin, LsRef * ideal);
            nextPhase(nowUs, Ref, ref);
            return ls;
        }
        const float d = std::max(std::fabs(vi / vi0 - 1.f), std::fabs(vo / vo0 - 1.f));
        dv = std::max(dv, d);
        if (!(d <= StiffV)) return fail("V shift");
        if (phase == Meas) {
            meas = i;
            measX = (float) ls_ - ((vi / vo - 1.f) * (float) hs - ideal);
            nextPhase(nowUs, Ref, ref);
            return ls;
        }
        if (k) {
            const float y = meas - 0.5f * (refPrev + i);
            xs[n] = measX; ys[n] = y; ++n;
            ymax = std::max(ymax, y);
            if (meas < -0.05f || (measX > ideal && y < ymax - 0.2f * std::fabs(meas)))
                return finish();
        }
        refPrev = i;
        for (;;) {
            if (k >= Steps) return finish();
            const float x = (LsLo + (LsHi - LsLo) * (float) k++ / (float) (Steps - 1)) * ideal;
            if (x > (float) lsCap) return finish();
            if (x < (float) lsMin) continue;
            ls_ = (uint16_t) std::lround(x);
            break;
        }
        nextPhase(nowUs, Meas, ls_);
        return ls;
    }

    // Peak fit and margin, not RT-safe (double-precision grid search). tickRate converts the ns limits.
    bool fit(float tickRate) {
        if (get() != Fit) return false;
        int pk = 0;
        for (int j = 1; j < n; ++j) if (ys[j] > ys[pk]) pk = j;
        float x0 = n ? xs[pk] : 0;
        double b = 0;
        const char *e = n < 5 ? "few points"
                        : (pk == 0 || pk == n - 1) ? "peak at edge"
                        : !asymPeakFit(xs, ys, n, x0, b) ? "fit"
                        : !(std::fabs(x0 - ideal) <= (float) pwmCountsFromNs(PlausibleNs, tickRate)) ? "implausible"
                        : nullptr;
        if (e) {
            fail(e);
            return false;
        }
        const float l = x0 - ideal;
        learned = (int16_t) std::lround(l);
        applied = (int16_t) std::lround(l - std::max((float) pwmCountsFromNs(MinMarginNs, tickRate), 0.3f * std::fabs(l)));
        peak = x0;
        state.store(Done, std::memory_order_release);
        return true;
    }

private:
    enum Phase : uint8_t { Init, Ref, Meas };
    Phase phase = Init;
    int ls = -1;
    uint16_t lsMin = 0, lsCap = 0, ref = 0, ls_ = 0;
    uint8_t k = 0;
    uint32_t t0 = 0, tPh = 0;
    uint32_t ns = 0;
    float si = 0, sv = 0, so = 0, refPrev = 0, meas = 0, measX = 0, ymax = 0, vi0 = 0, vo0 = 0;

    void clearAcc() { si = sv = so = 0; ns = 0; }

    void nextPhase(uint32_t nowUs, Phase p, int lsCount) {
        phase = p; ls = lsCount; tPh = nowUs;
        clearAcc();
    }

    int fail(const char *reason) {
        why = reason;
        state.store(Fail, std::memory_order_release);
        return -1;
    }

    int finish() {
        state.store(Fit, std::memory_order_release);
        return -1;
    }
};

/**
 * Runtime backstop for a too-long DCM LS window: output current collapsing while HS rises towards
 * D = M and Vin rises (less power drawn from the panel). A legit PV sweep past the MPP does not do
 * this: the panel current rises towards Isc as Vin falls; a source drop (cloud) lowers Vin.
 * update() returns true at most once per HoldUs when the offset should shrink.
 */
struct RectBackstop {
    static constexpr float MinPeak = 0.5f, CollapseFrac = 0.25f, NearM = 0.8f, VinRise = 0.005f;
    static constexpr uint32_t HoldUs = 2000000;
    static constexpr float StepNs = 125.f; // offset decrement per trigger (20 ct at 160 MHz)
    float peak = 0, vinPeak = 0;
    uint16_t peakHs = 0;
    uint32_t lastFire = 0; // 0: never

    void reset() { peak = 0, peakHs = 0; }

    // armed: eligible mode, DCM diode emulation with sync rect on; hsZero: HS count at D = M; hsDrop:
    // HS retreat that ends a rising run; iSmooth tracks the peak, iFast (protect()'s currentFilt) the
    // collapse, which outruns any EWM: it reaches the reverse-current trip within a few samples.
    bool update(uint32_t nowUs, bool armed, uint16_t hs, float hsZero, uint16_t hsDrop, float iSmooth, float iFast,
                float vin) {
        if (!armed || hs + hsDrop < peakHs) reset();
        if (!armed) return false;
        if (iSmooth > peak) { peak = iSmooth; peakHs = hs; vinPeak = vin; }
        if (peak < MinPeak || hs <= peakHs || (float) hs < NearM * hsZero || iFast > CollapseFrac * peak
            || !(vin > (1.f + VinRise) * vinPeak))
            return false;
        if (lastFire && nowUs - lastFire < HoldUs) return false;
        lastFire = nowUs | 1;
        reset();
        return true;
    }

    // Never above off: a positive offset (LS later than ideal) drops to 0 at once, below that by step,
    // not below floor (an offset already below floor stays).
    static int shrink(int off, int step, int floor) { return std::min(off, std::max(floor, off > 0 ? 0 : off - step)); }
};

// Controller state the rect-cal glue needs, per new sample. Physical output current is iS/iF.
struct RectCalIn {
    uint32_t nowUs;
    float vin, vout, iout;          // last samples (Iout sensor)
    float vinAvg, voutAvg, ioutAvg; // EWM
    float iS, iF;                   // physical output current: EWM and protect()'s fast filter
    bool mppt;     // MPPT mode (not PSU/PV-sim/manual duty), no ADC calibration
    bool sweeping; // global MPPT sweep
    bool bflow;    // backflow switch closed
    bool limiting; // a PD limiter is active
    bool iSensor;  // physical Iout sensor (not virtual)
};

/**
 * RT/core-0 orchestration of RectCal and RectBackstop around a converter (SynchronousConverter).
 * RT: rt() before protect() each new sample, hold() inside the MPPT update after protect() and the
 * limiters, drop() on a mode change that takes LS/HS over. Core 0: lf(), requestStart(), post(),
 * status(). Commands go core 0 -> RT in atomics; the RT owns the converter and the active offset,
 * and publishes it with its calibrated flag in one word (offset(), activeCalibrated()). coil.conf
 * always gets that published pair, never a value core 0 kept aside, and only while the converter
 * has been off for IdleS: a flash erase stalls both cores for tens of ms.
 */
template<class Conv>
class RectCalCtl {
public:
    static constexpr uint32_t StartupUs = 2000000; // backstop disarmed after converter start
    static constexpr uint32_t BootDelayS = 600, RetryS = 600, DayS = 24 * 3600;
    static constexpr uint32_t IdleS = 10, SaveRetryS = 60, SaveMinS = 600; // coil.conf save gating
    static constexpr float IdleW = 5.f; // auto start above, saves need |pOut| below IdleW / 5
    static constexpr uint8_t Confirm = 2; // backstop triggers before a reduced offset is saved

    RectCal cal{};
    bool autoRun = true; // coil.conf rect_cal_auto
    bool (*persist)(int counts, bool calibrated) = nullptr; // core 0, writes coil.conf
    std::atomic<bool> startReq{false}, abortReq{false};
    std::atomic<uint16_t> fires{0};    // backstop triggers that shrank the offset

    [[nodiscard]] int offset() const { return (int16_t) (act.load(std::memory_order_acquire) & 0xffff); }
    [[nodiscard]] bool activeCalibrated() const { return act.load(std::memory_order_acquire) & CalBit; }
    [[nodiscard]] bool converterOn() const { return convOn.load(std::memory_order_acquire); }

    // setup, before the RT loop runs
    void init(int off, bool calibrated) { act.store(word(off, calibrated), std::memory_order_release); }

    // core 0: an offset for RT to apply. firesSeen: backstop count the value was derived after; if RT
    // has shrunk the offset since, it applies the shorter of both, uncalibrated.
    void post(int off, bool calibrated, uint16_t firesSeen) {
        pend.store(word(off, calibrated) | PendBit | ((uint32_t) (firesSeen & 0x3fff) << 18),
                   std::memory_order_release);
    }
    void post(int off, bool calibrated) { post(off, calibrated, fires.load(std::memory_order_acquire)); }

    // any core-0 task: save the active offset at the next opportunity (after post())
    void requestSave() { saveReq.store(true, std::memory_order_release); }

    // core 0: request a run; false while one is running or being fitted.
    bool requestStart() {
        uint8_t s = cal.state.load(std::memory_order_acquire);
        if (s == RectCal::Run || s == RectCal::Fit) return false;
        if (s != RectCal::Idle && !cal.state.compare_exchange_strong(s, RectCal::Idle, std::memory_order_acq_rel))
            return false;
        startReq.store(true, std::memory_order_release);
        return true;
    }

    // RT, each new sample before protect(). Returns an HS count to ramp back to after a run, 0: none.
    uint16_t rt(Conv &c, const RectCalIn &in) {
        if (uint32_t p = pend.load(std::memory_order_acquire)) {
            const bool stale = ((p >> 18) & 0x3fff) != (fires.load(std::memory_order_relaxed) & 0x3fffu);
            const int off = (int16_t) (p & 0xffff);
            if (stale) setOff(c, std::min<int>(off, c.getRectOnOffset()), false);
            else setOff(c, off, p & CalBit);
            pend.compare_exchange_strong(p, 0, std::memory_order_acq_rel);
        }
        if (cal.running()) {
            const char *why = abortReq.load(std::memory_order_relaxed) ? "aborted"
                              : protectedSince(c, in) ? "protection" : block(c, in, false);
            if (why) cal.abort(why);
        }
        abortReq.store(false, std::memory_order_relaxed);
        const uint16_t back = cal.running() ? 0 : end(c, in);
        if (startReq.exchange(false, std::memory_order_acq_rel) && cal.get() == RectCal::Idle) {
            const uint16_t pm = c.pwmMaxDriver();
            const char *why = block(c, in, true);
            const uint16_t hs = why ? 0 : RectCal::calHs(in.vinAvg, in.voutAvg, pm);
            if (!why && (hs > c.getCtrlOnPwmCnt() || hs <= c.getCtrlOnPwmMin())) why = "HS below calibration point";
            if (why) cal.refuse(why);
            else {
                hsPrev = c.getCtrlOnPwmCnt();
                clamps = c.lsClamps();
                bs.reset();
                cal.start(in.nowUs, hs, c.getRectOnPwmMin(), pm - hs - 1);
            }
        }
        backstop(c, in);
        return back;
    }

    // RT, in the MPPT update: ramps HS to the calibration point, holds it and pins LS. Returns true
    // while update() must stop here; back: HS count to ramp back to after the run (0: none).
    bool hold(Conv &c, const RectCalIn &in, float limitingControlValue, uint16_t &back) {
        back = 0;
        if (!cal.running()) return false;
        if (protectedSince(c, in)) {
            cal.abort("protection");
        } else if (limitingControlValue < 0) {
            cal.abort("limiting");
        } else {
            const int step = cal.hsStep(c.getCtrlOnPwmCnt(), std::max(1, c.pwmMaxDriver() / 512));
            if (step) c.pwmPerturb((int16_t) step);
            const int ls = cal.feed(in.nowUs, c.getCtrlOnPwmCnt(), in.vin, in.vout, in.iout);
            if (ls >= 0 && !protectedSince(c, in)) {
                if (!c.manualRect_() || ls != c.getRectOnPwmCnt()) c.setManualRect(ls);
                owns = true;
            }
            if (cal.running()) return true;
        }
        back = end(c, in);
        return false;
    }

    // RT: a mode change that sets manualRect and the duty itself takes over without end().
    void drop() {
        cal.abort("mode change");
        owns = false;
        hsPrev = 0;
        bs.reset();
    }

    // core 0, periodic: fit, apply, log, deferred save, auto schedule. nowS: seconds since boot,
    // pOut: output power (W).
    void lf(Conv &c, uint32_t nowS, float pOut) {
        const float tr = c.getPwmTickRate();
        if (cal.get() == RectCal::Fit && cal.fit(tr)) {
            ESP_LOGI("rectcal", "done: hs=%u ideal=%.0f peak=%.0f learned=%+d ct, applied %+d ct (%.0f ns, %s, %u pts)",
                     cal.hs, cal.ideal, cal.peak, cal.learned, cal.applied, rectOffsetNsFromCounts(cal.applied, tr),
                     c.drvName(), cal.n);
            post(cal.applied, true, seenFires);
            c.setRectOffsetNeedsCal(false);
            dirty = true;
            confirms = 0;
        }
        const uint8_t s = cal.get();
        if ((s == RectCal::Done || s == RectCal::Fail) && cal.seq != seenSeq) {
            seenSeq = cal.seq;
            if (s == RectCal::Fail) {
                ESP_LOGW("rectcal", "%s: %s (%u pts), offset stays %d ct", cal.hs ? "failed" : "refused", cal.why,
                         cal.n, offset());
                if (!cal.hs && (int32_t) (nextAuto - nowS) > (int32_t) RetryS) nextAuto = nowS + RetryS;
            }
        }

        const uint16_t f = fires.load(std::memory_order_acquire);
        if (f != seenFires) {
            confirms += (uint16_t) (f - seenFires);
            seenFires = f;
            c.setRectOffsetNeedsCal(true);
            const int o = offset();
            ESP_LOGW("rectcal", "backstop: DCM current collapse near D=M, offset now %d ct (%.0f ns)", o,
                     rectOffsetNsFromCounts(o, tr));
            if (confirms >= Confirm) dirty = true;
        }

        // Save the RT-published pair once any posted value is applied, the converter has been off for
        // IdleS and the power reading confirms it (NaN does not).
        if (saveReq.exchange(false, std::memory_order_acq_rel)) dirty = true;
        const bool off = !converterOn() && std::fabs(pOut) < IdleW / 5;
        if (!off) offSince = 0;
        else if (!offSince) offSince = nowS | 1;
        if (dirty && offSince && (int32_t) (nowS - offSince) >= (int32_t) IdleS && !pend.load(std::memory_order_acquire)
            && (int32_t) (nowS - saveAt) >= 0) {
            const uint32_t a = act.load(std::memory_order_acquire);
            const int o = (int16_t) (a & 0xffff);
            const bool cl = a & CalBit;
            if (persist && persist(o, cl)) {
                ESP_LOGI("rectcal", "saved rect_offset %d ct%s", o, cl ? "" : " (uncalibrated)");
                dirty = false;
                confirms = 0;
                saveAt = nowS + SaveMinS;
            } else {
                saveAt = nowS + SaveRetryS;
            }
        }

        const bool steady = pOut > IdleW && std::fabs(pOut - pPrev) < 0.1f * pOut;
        pPrev = pOut;
        if (autoRun && c.rectOffsetNeedsCal() && steady && (int32_t) (nowS - nextAuto) >= 0 && requestStart()) {
            nextAuto = nowS + DayS;
            ESP_LOGI("rectcal", "auto calibration (rect_offset uncalibrated on %s)", c.drvName());
        }
    }

    // core 0
    void status(const Conv &c) const {
        static const char *const names[] = {"idle", "running", "fitting", "done", "failed"};
        const uint8_t s = cal.get();
        const int o = offset();
        ESP_LOGI("rectcal", "state=%s%s%s auto=%d needed=%d drv=%s offset=%d ct (%.0f ns%s) backstop=%u unsaved=%d",
                 names[s], s == RectCal::Fail ? ": " : "", s == RectCal::Fail ? cal.why : "", autoRun,
                 c.rectOffsetNeedsCal(), c.drvName(), o, rectOffsetNsFromCounts(o, c.getPwmTickRate()),
                 activeCalibrated() ? "" : ", uncalibrated", fires.load(std::memory_order_relaxed), dirty);
        if (s == RectCal::Run) ESP_LOGI("rectcal", "hs=%u", cal.hs);
        if ((s != RectCal::Done && s != RectCal::Fail) || !cal.hs) return;
        ESP_LOGI("rectcal", "last: hs=%u ideal=%.0f peak=%.0f learned=%+d applied=%+d ct, dV %.1f%%, %u pts", cal.hs,
                 cal.ideal, cal.peak, cal.learned, cal.applied, 100.f * cal.dv, cal.n);
        char buf[128];
        int len = 0;
        for (int i = 0; i < cal.n; ++i) {
            len += snprintf(buf + len, sizeof(buf) - len, " %.0f:%+.3f", cal.xs[i], cal.ys[i]);
            if (len > (int) sizeof(buf) - 20 || i == cal.n - 1) {
                ESP_LOGI("rectcal", "LS:dI%s", buf);
                len = 0;
            }
        }
    }

    [[nodiscard]] bool unsaved() const { return dirty; }

private:
    static constexpr uint32_t CalBit = 1u << 16, PendBit = 1u << 17;
    static uint32_t word(int off, bool calibrated) { return (uint16_t) (int16_t) off | (calibrated ? CalBit : 0); }

    RectBackstop bs{};
    std::atomic<uint32_t> act{0};  // RT: active offset (low 16 bits) | CalBit
    std::atomic<uint32_t> pend{0}; // core 0 -> RT: word() | PendBit | firesSeen << 18, 0: none
    std::atomic<bool> convOn{true}, saveReq{false};
    // RT
    bool owns = false;    // manualRect is held by the run
    uint16_t hsPrev = 0;  // HS before the run (0: none)
    uint8_t clamps = 0;   // converter LS-clamp count at start
    uint32_t upSince = 0; // converter enabled since (0: off)
    // core 0
    bool dirty = false; // act differs from coil.conf
    uint8_t seenSeq = 0;
    uint16_t seenFires = 0, confirms = 0;
    uint32_t nextAuto = BootDelayS, saveAt = 0, offSince = 0;
    float pPrev = 0;

    void setOff(Conv &c, int off, bool calibrated) {
        c.setRectOnOffset(off);
        act.store(word(off, calibrated), std::memory_order_release);
    }

    // A protection clamped LS or opened the backflow switch / sync rect since the start.
    bool protectedSince(const Conv &c, const RectCalIn &in) const {
        return c.lsClamps() != clamps || c.disabled() || !in.bflow || !c.syncRectEnabled_();
    }

    uint16_t end(Conv &c, const RectCalIn &in) {
        if (owns) c.setManualRect(-1);
        owns = false;
        const uint16_t h = (!c.disabled() && in.mppt && !in.sweeping) ? hsPrev : 0;
        hsPrev = 0;
        return h;
    }

    void backstop(Conv &c, const RectCalIn &in) {
        const bool on = !c.disabled();
        if (on != convOn.load(std::memory_order_relaxed)) convOn.store(on, std::memory_order_release);
        if (!on) upSince = 0;
        else if (!upSince) upSince = in.nowUs | 1;
        const bool armed = on && in.mppt && !cal.running() && (int32_t) (in.nowUs - upSince) >= (int32_t) StartupUs && !c.boost()
                           && c.inDCM() && c.syncRectEnabled_() && !c.manualRect_() && !c.forcedPwm_()
                           && in.vin > in.vout + 1.f;
        const uint16_t pm = c.pwmMaxDriver();
        if (!bs.update(in.nowUs, armed, c.getCtrlOnPwmCnt(), in.vout / in.vin * (float) pm, pm / 64, in.iS, in.iF,
                       in.vin))
            return;
        const float tr = c.getPwmTickRate();
        const int o = c.getRectOnOffset();
        const int n = RectBackstop::shrink(o, pwmCountsFromNs(RectBackstop::StepNs, tr), RectCal::floorCounts(tr));
        if (n == o) return;
        setOff(c, n, false);
        fires.fetch_add(1, std::memory_order_release);
    }

    static const char *block(const Conv &c, const RectCalIn &in, bool starting) {
        if (c.disabled()) return "converter off";
        if (c.boost()) return "boost";
        if (!in.mppt) return "not in MPPT mode";
        if (in.sweeping) return "sweeping";
        if (c.forcedPwm_() || c.forcedPwmRequested()) return "forced PWM";
        if (!c.syncRectEnabled_() || !in.bflow) return "sync rect/backflow off";
        if (!in.iSensor) return "no Iout sensor";
        if (!starting) return nullptr;
        if (c.manualRect_()) return "manual LS";
        if (!c.deadTimeIdle()) return "dead-time change queued";
        if (in.limiting) return "limiting";
        if (!c.inDCM()) return "not in DCM";
        if (!(in.ioutAvg >= 0.3f && in.ioutAvg <= 4.f)) return "Iout outside 0.3..4 A";
        if (!(in.vinAvg > in.voutAvg + 1.f)) return "Vin<=Vout+1";
        return nullptr;
    }
};
