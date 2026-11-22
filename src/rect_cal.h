#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <cstdio>

#include "esp_log.h"
#include "math/asym_peak_fit.h"
#include "pwm/mcpwm_timing.h"
#include "etc/mailbox.h"

/**
 * Self-calibration of the buck DCM low-side timing offset (rect_offset).
 *
 * HS is ramped to HsFrac*M of the period with the LS pinned at its minimum (body-diode freewheel,
 * no reverse current whatever the CCM/DCM state) and held there: a low DCM point draws well below
 * the panel's MPP current, so Vin stays on the flat, voltage-source part of the PV curve and does
 * not move with the LS window. Init waits for DCM and two consecutive windows that agree. Then
 * the LS count is stepped up from 0.5x the ideal DCM freewheel count ideal = (Vin/Vout - 1) * HS.
 * Output current is flat until the LS outlasts the inductor zero crossing, then bends down (reverse
 * current). Past the bend the switch-node ring makes the curve non-monotonic, so the sweep stops one
 * step after the first clear drop (if that step is back up, the drop is measured again: an outlier
 * is replaced, a ring dip stays) and then fills the four coarse intervals before the drop with Fine
 * points; the fit ends at the floor of the first dip, which can put the knee late on a partial recovery:
 * such a result gets the larger margin floor DipMarginNs. The LS never exceeds ideal + lateMax, nor
 * CcmFrac of the off-time limit (1 - Vo/Vi) of the period, beyond which the reverse current no longer
 * returns to zero within a period. Each step is
 * measured against a reference LS (0.6x ideal) sampled before and after it, which cancels linear
 * drift, and is placed relative to the ideal count of its own Vin/Vout. Each window current is
 * scaled to the Init voltages with the DCM current model I ~ Vin*(Vin-Vout)/Vout (D fixed), so a
 * bus voltage change does not read as a current step. Vin of a reference stays within StiffV of Init (panel
 * stiffness), Vout within VoutDrift (a shared battery bus moves); a step's voltages must lie on
 * the line through its two references within JumpV/JumpVo, which rejects a voltage step that the model
 * scaling cannot fully remove; a step that fails is measured again (MaxRetry). Vin may rise above
 * that by what the step's own measured (unscaled) current drop explains on a panel below its MPP
 * (SpvMax). A sense path that follows the LS window ends the run: a Vout reading off its references by
 * the same amount on both tries at one LS, or an Iout reading that rises by more than IoutLs (+ IoutLsA
 * per A) before the ideal count (the diode-to-channel gain is a few mA, reverse current only lowers it).
 * The fit places the zero crossing where the current bends, independent of a sloped baseline.
 * The coil current the switch-node ring leaves when HS turns on moves the zero crossing off ideal, by up to
 * sqrt(L C) either way depending on the ring phase, which differs at every operating point. The zero crossing
 * is therefore also taken from the charge per period (zc, needs the coil L). learned = peak - max(zc, ideal)
 * (counts) is the timing error of driver, gate and sensing, rounded early. A late (positive) result is applied up
 * to LateMaxNs from a calibration point below Vin = 2 Vout; a later one, or any late one above, is a bound (clamped).
 *
 * Ownership: the RT core writes the payload in start()/refuse() (from Idle) and during Run, core 0
 * in Fit. Each hand-off is a release store of `state`; the other side reads it with acquire before
 * touching the payload. Only core 0 leaves Done/Fail (requestStart), so a result stays stable while
 * core 0 reads it.
 */
class RectCal {
public:
    enum State : uint8_t { Idle, Run, Fit, Done, Fail };

    static constexpr int Steps = 24, Fine = 10, MaxPts = Steps + Fine; // coarse grid, fine pass at the knee
    static constexpr uint32_t SettleUs = 150000, InitUs = 500000, RefUs = 250000, MeasUs = 500000;
    static constexpr uint32_t TimeoutUs = 60000000, SettleMaxUs = 12000000; // run, ramp + Init (from start)
    static constexpr float LsLo = 0.5f, LsHi = 1.4f, LsRef = 0.6f, HsFrac = 0.55f;
    static constexpr float PlausibleNs = 2500.f; // |learned| above this is rejected (400 ct at 160 MHz)
    static constexpr float MinMarginNs = 125.f;  // applied margin floor (20 ct at 160 MHz)
    static constexpr float DipMarginNs = 250.f;  // same when the fit ended at a dip floor (40 ct)
    static constexpr float LateMaxNs = RectOffsetLateMaxNs; // latest applied offset (20 ct), below Vin = 2 Vout only
    static constexpr float StiffV = 0.02f;       // max Vin deviation from Init (fraction)
    static constexpr float VoutDrift = 0.05f;    // max Vout deviation from Init (fraction)
    static constexpr float JumpV = 0.003f;       // max step Vin off its references' mean (fraction)
    static constexpr float JumpVo = 0.002f;      // same for Vout (a bus can swing in step with the run)
    static constexpr float SpvMax = 0.25f;       // PV slope bound |dV/dI| * I/V below the MPP (1 at the MPP)
    static constexpr float KneeMinA = 0.01f;     // fitted bend at the last point, at least
    static constexpr uint8_t MaxRetry = 6;       // steps re-measured after a voltage step
    // knee: 2 consecutive steps each StepDrop below the previous and KneeDrop below the best (A)
    static constexpr float KneeDrop = 0.02f, StepDrop = 0.01f;
    static constexpr float CcmFrac = 0.9f; // LS cap: this fraction of the off-time volt-second limit (1 - Vo/Vi)
    static constexpr float SettleI = 0.02f;      // Init windows agree: |dI| <= SettleI A + SettleI * I
    static constexpr float IoutLs = 0.02f, IoutLsA = 0.004f; // max Iout rise before ideal: A + per A of Init current
    static constexpr float ZcLo = 0.8f;          // zc from the steps at ZcLo..1 x ideal (below the knee)

    std::atomic<uint8_t> state{Idle};
    const char *why = ""; // failure reason
    uint16_t hs = 0;      // 0 after a refused start
    uint8_t n = 0, seq = 0; // seq: counts starts and refusals
    int16_t learned = 0, applied = 0;
    bool clamped = false; // the fit asked for more than LateMaxNs: a bound, applied as min(LateMaxNs, active offset)
    bool cut = false;     // the fit ended at the floor of a dip
    float ideal = 0, peak = 0; // peak: fitted LS count of the zero crossing (knee)
    float zc = 0; // zero crossing from the charge per period (counts), ideal without a coil L
    float dv = 0; // largest relative Vin/Vout deviation from Init
    float rise = 0, iLs = 0; // Iout rise before ideal on two points in a row, its bound (A)
    float xs[MaxPts]{}, ys[MaxPts]{}; // fit() sorts them by LS
    int8_t jvi[MaxPts]{}, jvo[MaxPts]{}; // step Vin/Vout off its references' mean (0.01 %)
    uint8_t retries = 0;
    // last voltage check: deviation (fraction), Vin allowance (fraction), reference (V), LS count
    float jdi = 0, jdo = 0, jallow = 0, jrefVi = 0, jrefVo = 0;
    uint16_t jls = 0;

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

    // lateMax: farthest LS past ideal the sweep may go (counts, the plausible |learned|); lt: coil L times the
    // PWM period (H s), 0: zero crossing at ideal
    void start(uint32_t nowUs, uint16_t hs_, uint16_t lsMin_, uint16_t lsCap_, uint16_t lateMax_ = 400, float lt = 0) {
        hs = hs_; lsMin = lsMin_; lsCap = lsCap_; lateMax = lateMax_; fine = false; drops = 0; kf = 0; coilLT = lt;
        n = 0; k = 0; ideal = peak = zc = dv = rise = iLs = 0; ymax = -INFINITY; learned = applied = 0; clamped = cut = false; why = "";
        retries = 0; jdi = jdo = jallow = jrefVi = jrefVo = voHi = 0; jls = 0;
        phase = Init; ls = lsMin; prevOk = false;
        t0 = tPh = nowUs;
        clearAcc();
        ++seq;
        state.store(Run, std::memory_order_release);
    }

    void refuse(const char *reason) {
        hs = 0; n = 0; ++seq;
        fail(reason);
    }

    // RT: feed(), abort() and RectCalCtl::hold() are inlined into IRAM callers (rt(), update())
    __attribute__((always_inline)) void abort(const char *reason) {
        if (running()) fail(reason);
    }

    // LS count the run holds now (lsMin until Init is done), -1: none
    [[nodiscard]] int lsHold() const { return running() ? ls : -1; }

    // dcm: the converter is in DCM. Returns the LS count to hold, -1 to leave LS to diode emulation.
    // The sweep starts once HS has reached the calibration point, DCM holds and Init has settled.
    __attribute__((always_inline)) int feed(uint32_t nowUs, uint16_t hsNow, float vin, float vout, float iout, bool dcm) {
        if (!running()) return -1;
        if (nowUs - t0 > TimeoutUs) return fail("timeout");
        if (phase == Init) {
            if (hsNow != hs || !dcm) {
                if (nowUs - t0 > SettleMaxUs) return fail(hsNow != hs ? "HS ramp" : "not in DCM");
                tPh = nowUs;
                clearAcc();
                prevOk = false;
                return ls;
            }
        } else if (hsNow != hs) {
            return fail("HS moved");
        } else if (!dcm) {
            return fail("left DCM");
        }
        const uint32_t dt = nowUs - tPh;
        if (dt < SettleUs) return ls;
        si += iout; sv += vin; so += vout; ++ns;
        if (dt < SettleUs + (phase == Init ? InitUs : phase == Ref ? RefUs : MeasUs)) return ls;

        float i = si / (float) ns;
        const float vi = sv / (float) ns, vo = so / (float) ns;
        if (!std::isfinite(i + vi + vo)) return fail("bad sample");
        if (phase == Init) {
            if (!(vo > 0.5f && vi > vo + 1.f)) return fail("Vin<=Vout");
            // settled: two consecutive windows agree (refPrev/vi0/vo0 hold the previous one)
            if (!(prevOk && std::fabs(i - refPrev) <= SettleI * (1.f + std::fabs(i))
                  && std::fabs(vi / vi0 - 1.f) <= JumpV && std::fabs(vo / vo0 - 1.f) <= JumpV)) {
                if (nowUs - t0 > SettleMaxUs) return fail("unsettled");
                refPrev = i; vi0 = vi; vo0 = vo; prevOk = true;
                tPh = nowUs;
                clearAcc();
                return ls;
            }
            if (!(i > 0.15f)) return fail("low I");
            vi0 = vi; vo0 = vo; iLs = IoutLs + IoutLsA * i;
            g0 = dcmShape(vi, vo);
            ideal = (vi / vo - 1.f) * (float) hs;
            // past (1 - Vo/Vi) of the period the reverse current cannot return to zero before HS turns on:
            // it grows every cycle (synchronous buck run backwards from the battery)
            capX = std::min({(float) lsCap, CcmFrac * (1.f - vo / vi) * (float) (lsCap + hs + 1), ideal + (float) lateMax});
            if (capX < ideal + 0.5f * (float) lateMax) return fail("LS headroom");
            ref = (uint16_t) std::max<float>(lsMin, LsRef * ideal);
            nextPhase(nowUs, Ref, ref);
            return ls;
        }
        const float dIn = std::fabs(vi / vi0 - 1.f), dOut = std::fabs(vo / vo0 - 1.f);
        dv = std::max(dv, std::max(dIn, dOut));
        // a step window's Vin is judged against its references (a knee raises Vin on a soft panel)
        if (!(dIn <= StiffV) && phase == Ref) return vfail("Vin shift", vi, vo, vi0, vo0);
        if (!(dOut <= VoutDrift)) return vfail("Vout shift", vi, vo, vi0, vo0);
        const float r = g0 / dcmShape(vi, vo); // current at the Init voltages
        if (!(r > 0.8f && r < 1.25f)) return vfail("V shift", vi, vo, vi0, vo0);
        const float raw = i;
        i *= r;
        if (phase == Meas) {
            meas = i; measRaw = raw; measVi = vi; measVo = vo;
            measX = (float) ls_ - ((vi / vo - 1.f) * (float) hs - ideal);
            nextPhase(nowUs, Ref, ref);
            return ls;
        }
        bool redo = false;
        if (k) {
            // windows are equally spaced: a linear drift puts the step on its references' mean. The measured
            // (unscaled) current drop lowers the input power by dRaw/mid and raises Vin by up to SpvMax times that.
            const float mid = 0.5f * (refPrev + i), y = meas - mid, dRaw = 0.5f * (rawPrev + raw) - measRaw;
            jrefVi = 0.5f * (refVi + vi); jrefVo = 0.5f * (refVo + vo); jls = ls_;
            jdi = measVi / jrefVi - 1.f; jdo = measVo / jrefVo - 1.f;
            jallow = JumpV + (dRaw > 0 && mid > 0.1f ? SpvMax * dRaw / mid : 0.f);
            const bool viOk = jdi >= -JumpV && jdi <= jallow, voOff = std::fabs(jdo) > JumpVo;
            // Vout off by the same amount on both tries at this LS: the reading follows the LS window (the bus
            // does not), and so may the Iout reading of the same sense path
            if (viOk && voOff && jdo * voHi > 0 && std::fabs(jdo - voHi) <= JumpVo) return fail("sense follows LS");
            voHi = 0;
            if (!viOk || voOff) {
                if (retries >= MaxRetry) return fail("V step");
                ++retries; redo = true;
                if (viOk) voHi = jdo;
            } else {
                const bool low = y <= ymax - 0.5f * KneeDrop;
                bool go = false;
                if (drops == 2) {
                    // the drop measured again: still low (a ring dip): the lower value, else it was an outlier
                    const int d = n - 2;
                    if (low) ys[d] = std::min(ys[d], y);
                    else xs[d] = measX, ys[d] = y, jvi[d] = pct100(jdi), jvo[d] = pct100(jdo), ymax = std::max(ymax, y);
                    go = low; drops = 0;
                } else {
                    xs[n] = measX; ys[n] = y;
                    jvi[n] = pct100(jdi); jvo[n] = pct100(jdo);
                    ++n;
                    // a reading that climbs before the ideal count on two points in a row
                    if (!fine && n > 3 && measX <= ideal
                        && (rise = std::max(rise, std::min(y, ys[n - 2]) - std::max(ys[0], ys[1]))) > iLs)
                        return fail("Iout follows LS");
                    const bool down = n > 1 && y <= std::min(ys[n - 2] - StepDrop, ymax - KneeDrop);
                    ymax = std::max(ymax, y);
                    if (meas < -0.05f) return finish();
                    if (!fine) {
                        if (!drops) drops = down, lsD = ls_;
                        else go = low, drops = low ? 0 : 2; // back up after the drop: measure it again
                    }
                }
                if (go) {
                    // one step past a clear drop (the ring can turn the current back up): the knot lies within
                    // 4 coarse steps before the drop (sharp knee: the last one, quadratic: 2-3): Fine steps
                    // across them, never past the drop
                    if (n < 6 || n + Fine > MaxPts) return finish();
                    fg = (xs[n - 2] - xs[n - 6]) / (float) Fine;
                    f0 = xs[n - 6] + 0.5f * fg;
                    fine = true;
                }
            }
        }
        refPrev = i; rawPrev = raw; refVi = vi; refVo = vo;
        if (redo || drops == 2) {
            nextPhase(nowUs, Meas, ls_ = redo ? ls_ : lsD);
            return ls;
        }
        for (;;) {
            if (fine ? kf >= Fine : k >= Steps) return finish();
            const float x = fine ? f0 + fg * (float) kf++ : (LsLo + (LsHi - LsLo) * (float) k++ / (float) (Steps - 1)) * ideal;
            if (x > capX) return finish();
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
        for (int j = 1; j < n; ++j) // insertion sort by LS, the fine pass interleaves
            for (int m = j; m > 0 && xs[m] < xs[m - 1]; --m) {
                std::swap(xs[m], xs[m - 1]); std::swap(ys[m], ys[m - 1]);
                std::swap(jvi[m], jvi[m - 1]); std::swap(jvo[m], jvo[m - 1]);
            }
        // the ring can turn the current back up past the bend: once two points in a row were KneeDrop/2 below the
        // highest and one rises StepDrop above the lowest since, the fit ends at that lowest point (the dip floor
        // after it is no quadratic: it pulled the knee early). An up-bend at the crossing comes before any dip.
        int m = 1, low = 0, at = 0;
        bool dip = cut = false;
        for (float hi = ys[0], lo = ys[0]; m < n; ++m) {
            if (dip && ys[m] > lo + StepDrop) { m = at + 1; cut = true; break; }
            low = ys[m] <= hi - 0.5f * KneeDrop ? low + 1 : 0;
            dip |= low >= 2;
            if (ys[m] > hi) hi = lo = ys[m], dip = false;
            else if (ys[m] < lo) lo = ys[m], at = m;
        }
        // A DCM triangle of peak p carries p^2 L Vi / (2 Vo (Vi - Vo)) per period, whatever current HS turned on
        // into (second order) and however long it conducted, and ends p L / Vo after HS turns off
        float iz = 0;
        int nz = 0, kmax = 0;
        for (int j = 0; j < n; ++j)
            if (xs[j] >= ZcLo * ideal && xs[j] <= ideal) iz += ys[j], ++nz;
        for (int j = 1; j < m; ++j)
            if (ys[j] > ys[kmax]) kmax = j;
        zc = ideal;
        if (coilLT > 0 && nz)
            zc = tickRate * std::sqrt(std::max(0.f, 2.f * (refPrev + iz / (float) nz) * coilLT * (vi0 - vo0) / (vo0 * vi0)));
        const float plaus = (float) pwmCountsFromNs(PlausibleNs, tickRate);
        // Holding the LS past the crossing changes the ring current at HS turn-on: with a negative one the current
        // bends up there, else it stays flat until the node clamps at Vin and bends down. The crossing is the
        // earlier of an up-bend before the maximum and a down-bend before the dip floor.
        float x0 = INFINITY;
        const auto knee = [&](int j, bool up) {
            float x, drop, rms;
            if (j >= 6 && kneeFit(xs, ys, j, x, drop, rms, up) && drop >= std::max(KneeMinA, 4.f * rms)) x0 = std::min(x0, x);
        };
        if (xs[kmax] > zc) knee(kmax + 1, true);
        knee(m, false);
        const char *e = m < 6 ? "few points"
                        : coilLT > 0 && !nz ? "no points below ideal"
                        : !(std::fabs(zc - ideal) <= plaus) ? "charge model"
                        : !(x0 < INFINITY) ? "no knee"
                        : !(std::fabs(x0 - std::max(zc, ideal)) <= plaus) ? "implausible"
                        : nullptr;
        if (e) {
            fail(e);
            return false;
        }
        // zc - ideal is the ring current at HS turn-on, but also HS edge delays and half a voltage sensing error,
        // which the offset must keep: against the later of both the result errs early (body diode)
        const float l = x0 - std::max(zc, ideal);
        const float a = l - std::max((float) pwmCountsFromNs(cut ? DipMarginNs : MinMarginNs, tickRate), 0.3f * std::fabs(l));
        learned = (int16_t) std::lround(l);
        // Driver and gate delays explain tens of ns. Past Vin = 2 Vout the node lift to Vin can put the knee later
        // still, so a late result there only bounds the offset at 0; below, beyond LateMaxNs (lf(): min(hi, active)).
        const float hi = vi0 < 2.f * vo0 ? (float) pwmCountsFromNs(LateMaxNs, tickRate) : 0.f;
        clamped = a > hi;
        applied = (int16_t) std::lround(std::min(a, hi));
        peak = x0;
        state.store(Done, std::memory_order_release);
        return true;
    }

private:
    enum Phase : uint8_t { Init, Ref, Meas };
    Phase phase = Init;
    int ls = -1;
    uint16_t lsMin = 0, lsCap = 0, ref = 0, ls_ = 0, lateMax = 400;
    bool fine = false;      // fine pass around the knee
    uint8_t drops = 0, kf = 0; // drops: 1 the last coarse step was a clear drop (at lsD), 2 measuring it again
    uint16_t lsD = 0;
    float capX = 0, f0 = 0, fg = 0; // LS cap, fine pass start and step
    float coilLT = 0; // coil L times the PWM period (H s)
    uint8_t k = 0;
    uint32_t t0 = 0, tPh = 0;
    uint32_t ns = 0;
    bool prevOk = false; // Init: a previous window is stored
    float si = 0, sv = 0, so = 0, refPrev = 0, meas = 0, measX = 0, ymax = 0, vi0 = 0, vo0 = 0, g0 = 0;
    float refVi = 0, refVo = 0, measVi = 0, measVo = 0, rawPrev = 0, measRaw = 0; // raw: unscaled current
    float voHi = 0; // Vout offset of the last attempt at this LS, if only Vout failed

    void clearAcc() { si = sv = so = 0; ns = 0; }

    // DCM output current at fixed D over its constant factor D^2/(2 fsw L): Vin (Vin - Vout) / Vout [V]
    static float dcmShape(float vi, float vo) { return vi * (vi - vo) / vo; }

    void nextPhase(uint32_t nowUs, Phase p, int lsCount) {
        phase = p; ls = lsCount; tPh = nowUs;
        clearAcc();
    }

    static int8_t pct100(float f) { return (int8_t) std::clamp(std::lround(1e4f * f), -127L, 127L); }

    int vfail(const char *reason, float vi, float vo, float rvi, float rvo) {
        jdi = vi / rvi - 1.f; jdo = vo / rvo - 1.f; jallow = 0; jrefVi = rvi; jrefVo = rvo; jls = ls;
        return fail(reason);
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
    bool limiting; // any PD limiter is past its limit (P term), not a D-only touch
    bool iSensor;  // physical Iout sensor (not virtual)
    float vinMax;  // input over-voltage trip (V)
};

/**
 * RT/core-0 orchestration of RectCal and RectBackstop around a converter (SynchronousConverter).
 * RT: rt() before protect() each new sample, hold() inside the MPPT update after protect() and the
 * limiters, drop() on a mode change that takes LS/HS over. Core 0: lf(), requestStart(), post(),
 * status(). Commands go core 0 -> RT in atomics; the RT owns the converter and the active offset,
 * and publishes it with its calibrated flag in one word (offset(), activeCalibrated()). coil.conf
 * always gets that published pair, never a value core 0 kept aside, and only under a persist lock:
 * a flash erase stalls both cores for tens of ms. Lock states (lk): core 0 requests (Req); RT grants
 * (Granted, starts inhibited) once the converter has been off without any start for IdleUs; core 0
 * admits the write (Admitted) and releases (Idle) when it is done. RT expires a grant that is not
 * admitted within LockMaxUs (Expired), never an admitted write. Only starts are inhibited.
 */
template<class Conv>
class RectCalCtl {
public:
    static constexpr uint32_t StartupUs = 2000000; // backstop disarmed after converter start
    static constexpr uint32_t BootDelayS = 600, RetryS = 600, DayS = 24 * 3600, BoundRetryS = 7 * DayS;
    static constexpr uint32_t IdleUs = 10000000, LockMaxUs = 8000000; // persist lock: off time, grant limit
    enum : uint8_t { Idle, Req, Granted, Admitted, Expired };
    static constexpr uint32_t SaveRetryS = 60, SaveMinS = 600;
    static constexpr float IdleW = 5.f; // auto start above, saves need |pOut| below IdleW / 5
    static constexpr float VinMaxFrac = 0.97f; // run aborts above this fraction of the Vin trip
    static constexpr uint8_t Confirm = 2; // backstop triggers before a reduced offset is saved

    RectCal cal{};
    bool autoRun = true; // coil.conf rect_cal_auto
    bool (*persist)(int counts, uint8_t cal) = nullptr; // core 0, writes coil.conf (cal: rect_offset_cal)
    std::atomic<bool> startReq{false}, abortReq{false}, forceReq{false};
    std::atomic<uint16_t> fires{0};    // backstop triggers that shrank the offset

    [[nodiscard]] int offset() const { return (int16_t) (act.load(std::memory_order_acquire) & 0xffff); }
    [[nodiscard]] bool activeCalibrated() const { return act.load(std::memory_order_acquire) & CalBit; }
    // a coil.conf write is admitted: converter starts stay inhibited until it is done
    [[nodiscard]] bool persistLocked() const { return lk.load(std::memory_order_acquire) == Admitted; }
    // core 0: a save waits for the RT grant
    [[nodiscard]] bool saveRequested() const { return lk.load(std::memory_order_acquire) == Req; }

    // setup, before the RT loop runs
    void init(int off, bool calibrated) { act.store(word(off, calibrated), std::memory_order_release); }

    // core 0: an offset for RT to apply. firesSeen: backstop count the value was derived after; if RT
    // has shrunk the offset since, it applies the shorter of both, uncalibrated.
    void post(int off, bool calibrated, uint16_t firesSeen) {
        pend.post(word(off, calibrated) | ((uint32_t) (firesSeen & 0x3fff) << 17));
    }
    void post(int off, bool calibrated) { post(off, calibrated, fires.load(std::memory_order_acquire)); }

    // any core-0 task: save the active offset at the next opportunity (after post())
    void requestSave() { saveReq.store(true, std::memory_order_release); }

    // core 0, after lf(): finish a save request in this pass. Waits (wait(): RT gets to sample) while
    // the request is pending, then admits a grant, whenever RT gave it, and withdraws what is left.
    template<class Wait>
    void saveNow(const Conv &c, uint32_t nowS, Wait &&wait) {
        for (int i = 0; i < 10 && c.disabled() && saveRequested(); ++i) wait();
        trySave(c, nowS);
        cancelSave();
    }

    // core 0: withdraw a request or an unadmitted grant
    void cancelSave() {
        uint8_t s = lk.load(std::memory_order_acquire);
        while ((s == Req || s == Granted) && !lk.compare_exchange_weak(s, Idle, std::memory_order_acq_rel)) {}
    }

    // core 0: write the RT-published pair if RT granted the lock. RT cannot revoke the admitted write.
    // A calibrated value keeps the converter's measured/bound state (set on core 0 with its post()).
    void trySave(const Conv &c, uint32_t nowS) {
        uint8_t s = Granted;
        if (lk.compare_exchange_strong(s, Admitted, std::memory_order_acq_rel)) write(c, nowS);
    }

    // core 0, before a reboot with the converter off and its starts inhibited: save an applied offset without the
    // RT grant (RT may not sample any more). Skipped while another task's write is admitted.
    void saveFinal(const Conv &c) {
        uint8_t s = lk.load(std::memory_order_acquire);
        while (dirty && pend.idle() && c.disabled() && s != Admitted)
            if (lk.compare_exchange_weak(s, Admitted, std::memory_order_acq_rel)) return write(c, 0);
    }

private:
    void write(const Conv &c, uint32_t nowS) {
        const uint32_t a = act.load(std::memory_order_acquire);
        const int o = (int16_t) (a & 0xffff);
        const uint8_t cl = a & CalBit ? std::max<uint8_t>(1, c.rectOffsetCal()) : 0;
        const bool ok = persist && persist(o, cl);
        lk.store(Idle, std::memory_order_release);
        if (ok) {
            ESP_LOGI("rectcal", "saved rect_offset %d ct%s", o, cl == 1 ? "" : cl ? " (bound)" : " (uncalibrated)");
            dirty = false;
            confirms = 0;
            saveAt = nowS + SaveMinS;
        } else {
            saveAt = nowS + SaveRetryS;
        }
    }

public:
    // core 0: request a run; false while one is running or being fitted. force: start from any MPPT
    // operating point above the calibration HS, also CCM at high current.
    bool requestStart(bool force = false) {
        uint8_t s = cal.state.load(std::memory_order_acquire);
        if (s == RectCal::Run || s == RectCal::Fit) return false;
        if (s != RectCal::Idle && !cal.state.compare_exchange_strong(s, RectCal::Idle, std::memory_order_acq_rel))
            return false;
        forceReq.store(force, std::memory_order_relaxed);
        startReq.store(true, std::memory_order_release);
        return true;
    }

    // RT, each new sample before protect(). Returns an HS count to ramp back to after a run, 0: none.
    uint16_t rt(Conv &c, const RectCalIn &in) {
        if (const uint32_t p = pend.peek()) {
            const bool stale = ((p >> 17) & 0x3fff) != (fires.load(std::memory_order_relaxed) & 0x3fffu);
            const int off = (int16_t) (p & 0xffff);
            if (stale) setOff(c, std::min<int>(off, c.getRectOnOffset()), false);
            else setOff(c, off, p & CalBit);
            pend.clear(p);
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
            const char *why = block(c, in, true, forceReq.load(std::memory_order_relaxed));
            const uint16_t hs = why ? 0 : RectCal::calHs(in.vinAvg, in.voutAvg, pm);
            if (!why && (hs > c.getCtrlOnPwmCnt() || hs <= c.getCtrlOnPwmMin())) why = "HS below calibration point";
            if (why) cal.refuse(why);
            else {
                hsPrev = c.getCtrlOnPwmCnt();
                clamps = c.lsClamps();
                bs.reset();
                cal.start(in.nowUs, hs, c.getRectOnPwmMin(), pm - hs - 1,
                          (uint16_t) pwmCountsFromNs(RectCal::PlausibleNs, c.getPwmTickRate()),
                          c.getCoilL() / (float) c.getPwmFrequency());
            }
        }
        persistLock(c, in.nowUs);
        backstop(c, in);
        return back;
    }

    // RT, in the MPPT update after the limiters: pins LS (at its minimum until Init is done), ramps HS to the
    // calibration point by pwmMax/512 per sample and holds it. Returns true while update() must stop
    // here; back: HS count to ramp back to after the run (0: none).
    __attribute__((always_inline)) bool hold(Conv &c, const RectCalIn &in, uint16_t &back) {
        back = 0;
        if (!cal.running()) return false;
        if (protectedSince(c, in)) {
            cal.abort("protection");
        } else if (in.limiting) {
            cal.abort("limiting");
        } else if (in.vin > VinMaxFrac * in.vinMax) {
            cal.abort("Vin near max");
        } else {
            pin(c, cal.lsHold()); // before the first HS step: no complementary LS window on the way down
            const int step = cal.hsStep(c.getCtrlOnPwmCnt(), std::max(1, c.pwmMaxDriver() / 512));
            if (step) c.pwmPerturb((int16_t) step);
            pin(c, cal.feed(in.nowUs, c.getCtrlOnPwmCnt(), in.vin, in.vout, in.iout, c.inDCM()));
            if (cal.running()) return true;
        }
        back = end(c, in);
        return false;
    }

    // RT: a mode change that sets manualRect and the duty itself takes over without end().
    __attribute__((always_inline)) void drop() {
        cal.abort("mode change");
        owns = false;
        hsPrev = 0;
        bs.reset();
    }

    // core 0, periodic: fit, apply, log, deferred save, auto schedule. nowS: seconds since boot,
    // pOut: output power (W), vr: Vin/Vout.
    void lf(Conv &c, uint32_t nowS, float pOut, float vr = 0) {
        const float tr = c.getPwmTickRate();
        if (cal.get() == RectCal::Fit && cal.fit(tr)) {
            // a clamped fit is a bound: the active offset if shorter, else LateMaxNs; a measured shorter one stays
            // measured
            const int o = offset();
            const bool keep = cal.clamped && o < cal.applied && activeCalibrated() && c.rectOffsetCal() == 1;
            if (cal.clamped) cal.applied = (int16_t) std::min<int>(cal.applied, o), nextAuto = nowS + BoundRetryS;
            ESP_LOGI("rectcal", "done: hs=%u ideal=%.0f zc=%.0f peak=%.0f learned=%+d ct, applied %+d ct%s (%.0f ns, %s, %u pts)",
                     cal.hs, cal.ideal, cal.zc, cal.peak, cal.learned, cal.applied,
                     keep ? " (bound, measured offset kept)" : cal.clamped ? " (bound)" : "",
                     rectOffsetNsFromCounts(cal.applied, tr), c.drvName(), cal.n);
            if (!keep) {
                post(cal.applied, true, seenFires);
                c.setRectOffsetCal(cal.clamped ? 2 : 1);
                dirty = true;
                confirms = 0;
            }
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

        // Save the RT-published pair once any posted value is applied, the power reading confirms the
        // converter is off (NaN does not) and RT has granted the persist lock.
        if (saveReq.exchange(false, std::memory_order_acq_rel)) dirty = true;
        const bool want = dirty && std::fabs(pOut) < IdleW / 5 && pend.idle()
                          && (int32_t) (nowS - saveAt) >= 0;
        const uint8_t l = lk.load(std::memory_order_acquire);
        if (l == Expired) {
            lk.store(Idle, std::memory_order_release);
            saveAt = nowS + SaveRetryS;
        } else if (want && l == Idle) {
            lk.store(Req, std::memory_order_release);
        } else if (!want) {
            cancelSave();
        }
        trySave(c, nowS);

        const bool steady = pOut > IdleW && std::fabs(pOut - pPrev) < 0.1f * pOut;
        pPrev = pOut;
        // a bound (knee past LateMaxNs, as the node lift can put it above Vin = 2 Vout) is retried only below that
        const uint8_t lv = c.rectOffsetCal();
        if (autoRun && (!lv || (lv == 2 && vr > 0 && vr < 2.f)) && steady && (int32_t) (nowS - nextAuto) >= 0
            && requestStart()) {
            nextAuto = nowS + DayS;
            ESP_LOGI("rectcal", "auto calibration (rect_offset %s on %s)", lv ? "bound" : "uncalibrated", c.drvName());
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
                 !activeCalibrated() ? ", uncalibrated" : c.rectOffsetCal() == 2 ? ", bound" : "",
                 fires.load(std::memory_order_relaxed), dirty);
        if (s == RectCal::Run) ESP_LOGI("rectcal", "hs=%u", cal.hs);
        if ((s != RectCal::Done && s != RectCal::Fail) || !cal.hs) return;
        ESP_LOGI("rectcal", "last: hs=%u ideal=%.0f zc=%.0f peak=%.0f learned=%+d applied=%+d ct%s%s, dV %.1f%%, Iout rise "
                 "%.0f mA (max %.0f), %u pts, %u redone", cal.hs, cal.ideal, cal.zc, cal.peak, cal.learned, cal.applied,
                 cal.clamped ? " (bound)" : "", cal.cut ? " (fit to dip floor)" : "", 100.f * cal.dv, 1e3f * cal.rise,
                 1e3f * cal.iLs, cal.n, cal.retries);
        if (s == RectCal::Fail && cal.jrefVi > 0) {
            const bool st = cal.jallow > 0; // step vs its references, else window vs Init
            ESP_LOGI("rectcal", "at LS %u: Vin %+.2f%% (%+.2f..%+.2f) Vout %+.2f%% (+-%.2f) vs %s %.2f/%.2f V", cal.jls,
                     100.f * cal.jdi, -100.f * (st ? RectCal::JumpV : RectCal::StiffV),
                     100.f * (st ? cal.jallow : RectCal::StiffV), 100.f * cal.jdo,
                     100.f * (st ? RectCal::JumpVo : RectCal::VoutDrift), st ? "refs" : "Init", cal.jrefVi, cal.jrefVo);
        }
        char buf[128];
        int len = 0;
        for (int i = 0; i < cal.n; ++i) {
            len += snprintf(buf + len, sizeof(buf) - len, " %.0f:%+.3f/%d/%d", cal.xs[i], cal.ys[i], cal.jvi[i], cal.jvo[i]);
            if (len > (int) sizeof(buf) - 28 || i == cal.n - 1) {
                ESP_LOGI("rectcal", "LS:dI/Vin/Vout%s", buf);
                len = 0;
            }
        }
    }

    [[nodiscard]] bool unsaved() const { return dirty; }

private:
    static constexpr uint32_t CalBit = 1u << 16;
    static uint32_t word(int off, bool calibrated) { return (uint16_t) (int16_t) off | (calibrated ? CalBit : 0); }

    RectBackstop bs{};
    std::atomic<uint32_t> act{0};  // RT: active offset (low 16 bits) | CalBit
    ReqSlot<31> pend; // core 0 -> RT: word() | firesSeen << 17
    std::atomic<bool> saveReq{false};
    std::atomic<uint8_t> lk{Idle}; // persist lock state
    // RT
    bool owns = false;    // manualRect is held by the run
    uint16_t hsPrev = 0;  // HS before the run (0: none)
    uint8_t clamps = 0;   // converter LS-clamp count at start
    uint32_t upT0 = 0, offT0 = 0, lockT0 = 0; // start of: start-up hold, off time, lock
    uint16_t enSeen = 0;                        // converter enable count at offT0
    bool upPending = false, upDone = false;     // start-up hold running / over until the next stop
    bool offDone = false, lockHeld = false;
    // core 0
    bool dirty = false; // act differs from coil.conf
    uint8_t seenSeq = 0;
    uint16_t seenFires = 0, confirms = 0;
    uint32_t nextAuto = BootDelayS, saveAt = 0;
    float pPrev = 0;

    void setOff(Conv &c, int off, bool calibrated) {
        c.setRectOnOffset(off);
        act.store(word(off, calibrated), std::memory_order_release);
    }

    // A protection clamped LS or opened the backflow switch / sync rect since the start.
    bool protectedSince(const Conv &c, const RectCalIn &in) const {
        return c.lsClamps() != clamps || c.disabled() || !in.bflow || !c.syncRectEnabled_();
    }

    void pin(Conv &c, int ls) {
        if (ls < 0) return;
        if (!c.manualRect_() || ls != c.getRectOnPwmCnt()) c.setManualRect(ls);
        owns = true;
    }

    // HS ramps back unless a mode change or the reverse-current guard (backflow switch open) intervened
    uint16_t end(Conv &c, const RectCalIn &in) {
        if (owns) c.setManualRect(-1);
        owns = false;
        const uint16_t h = (!c.disabled() && in.mppt && !in.sweeping && in.bflow) ? hsPrev : 0;
        hsPrev = 0;
        return h;
    }

    // RT: off time must be continuous (any start, even one between two samples, restarts it).
    void persistLock(Conv &c, uint32_t nowUs) {
        const uint16_t en = c.enableCount();
        if (!c.disabled() || en != enSeen) enSeen = en, offT0 = nowUs, offDone = false;
        else if (!offDone && nowUs - offT0 >= IdleUs) offDone = true;
        uint8_t s = lk.load(std::memory_order_acquire);
        if (lockHeld && s != Granted && s != Admitted) {
            c.inhibitStart(false);
            lockHeld = false;
        }
        if (s == Req && offDone) {
            c.inhibitStart(true);
            if (c.disabled() && c.enableCount() == en
                && lk.compare_exchange_strong(s, Granted, std::memory_order_acq_rel)) {
                lockHeld = true;
                lockT0 = nowUs;
            } else {
                c.inhibitStart(false);
            }
        } else if (s == Granted && nowUs - lockT0 >= LockMaxUs
                   && lk.compare_exchange_strong(s, Expired, std::memory_order_acq_rel)) {
            c.inhibitStart(false);
            lockHeld = false;
        }
    }

    void backstop(Conv &c, const RectCalIn &in) {
        const bool on = !c.disabled();
        if (!on) upPending = upDone = false;
        else if (!upPending) upPending = true, upT0 = in.nowUs;
        else if (!upDone && in.nowUs - upT0 >= StartupUs) upDone = true;
        const bool armed = on && upDone && in.mppt && !cal.running() && !c.boost()
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

    static const char *block(const Conv &c, const RectCalIn &in, bool starting, bool force = false) {
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
        if (!force && !c.inDCM()) return "not in DCM";
        if (!(in.ioutAvg >= 0.3f && (force || in.ioutAvg <= 4.f))) return force ? "Iout below 0.3 A" : "Iout outside 0.3..4 A";
        if (!(in.vinAvg > in.voutAvg + 1.f)) return "Vin<=Vout+1";
        return nullptr;
    }
};
