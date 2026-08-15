#include "vconv.h"

#include <cstdio>

VirtualConverter g_vconv;

namespace {
inline float trapArea(float y0, float y1, float t) {
    return 0.5f * (y0 + y1) * t;
}
constexpr float kTwoPi = 6.28318530717958647692f;
}

void VirtualConverter::stepOneCycle(float T) {
    const float L = l_;
    const uint16_t pmax = pwm_.pwmMax;
    const uint16_t ctrl = pwm_.pwmCtrl;
    const uint16_t rect = pwm_.pwmRect;

    // Rebuild reciprocals only when an input changes; the steady N-cycle loop then divides nothing
    // (LX7 has no hw float divide, so each `/` was a software __divsf3). 6 compares << 18 divisions.
    if (T != cycT_ || pmax != cycPmax_ || l_ != cycL_ || cIn_ != cycCin_ ||
        cOut_ != cycCout_ || rbat_ != cycRbat_) {
        cycT_ = T; cycPmax_ = pmax; cycL_ = l_; cycCin_ = cIn_; cycCout_ = cOut_; cycRbat_ = rbat_;
        invL_    = (l_   > 0.0f) ? 1.0f / l_   : 0.0f;
        invCin_  = (cIn_ > 0.0f) ? 1.0f / cIn_ : 0.0f;
        invCout_ = (cOut_> 0.0f) ? 1.0f / cOut_: 0.0f;
        invT_    = 1.0f / T;
        aOut_    = T / (rbat_ * cOut_);
        inv1pAout_ = 1.0f / (1.0f + aOut_);
        invPmax_ = pmax ? 1.0f / (float) pmax : 0.0f;
    }

    // Spec invariant: pwmCtrl + pwmRect <= pwmMax. Idle while violated; recover
    // as soon as a subsequent update brings the counts back in range.
    if (pmax > 0 && (uint32_t) ctrl + (uint32_t) rect > pmax) {
        if (!errored_) {
            std::printf("vconv: pwmCtrl(%u)+pwmRect(%u) > pwmMax(%u)\n",
                        (unsigned) ctrl, (unsigned) rect, (unsigned) pmax);
            errored_ = true;
        }
        iInAvg_ = 0.0f;
        iOutAvg_ = 0.0f;
        return;
    }
    if (errored_) {
        std::printf("vconv: pwm counts back in range, resuming\n");
        errored_ = false;
    }

    // Mains ripple advances regardless of regime so disturbance is steady. Skip
    // when v_bat == 0 (open-circuit preset): a bipolar swing around zero is unphysical.
    float vBatEff = vbat_;
    if (vbatAcAmp_ != 0.0f && vbat_ > 0.0f) {
        const float step = kTwoPi * vbatAcFreq_ * T;
        if (vbatAcShape_ == 0) {
            // Built-in sine via recursive oscillator — no per-cycle sinf. (Re)seed on freq change.
            if (step != oscStep_) {
                oscStep_ = step;
                rotC_ = std::cos(step); rotS_ = std::sin(step);
                oscS_ = std::sin(vbatAcPhase_); oscC_ = std::cos(vbatAcPhase_);
            }
            vBatEff += vbatAcAmp_ * oscS_;
            const float ns = oscS_ * rotC_ + oscC_ * rotS_; // rotate (sin,cos) by +step
            const float nc = oscC_ * rotC_ - oscS_ * rotS_;
            const float corr = 1.5f - 0.5f * (ns * ns + nc * nc); // cheap renorm toward unit circle
            oscS_ = ns * corr; oscC_ = nc * corr;
        } else {
            vBatEff += vbatAcAmp_ * vbatAcShapeFn_(vbatAcPhase_); // |sin| / custom: keep the call
        }
        vbatAcPhase_ += step;
        if (vbatAcPhase_ > kTwoPi) vbatAcPhase_ -= kTwoPi;
        else if (vbatAcPhase_ < -kTwoPi) vbatAcPhase_ += kTwoPi;
    }

    // V_in below this means the PV is dead (no source) — phase-1 math becomes
    // degenerate, relax caps only. V_out is NOT guarded: a real short pulls
    // V_out near zero while the converter still delivers large I_L, and the
    // phase math handles V_out → 0 (phase-2/3 slopes go to zero, coil idles).
    constexpr float kVMin = 1e-4f;
    if (vIn_ < kVMin || pmax == 0 || L <= 0.0f) {
        const float Ipv = pvCurrent(vIn_);
        vIn_  += Ipv * T * invCin_;
        vOut_ = (vOut_ + aOut_ * vBatEff) * inv1pAout_;
        if (vIn_  < 0.0f) vIn_  = 0.0f;
        if (vOut_ < 0.0f) vOut_ = 0.0f;
        iInAvg_  = 0.0f;
        iOutAvg_ = 0.0f;
        iLEnd_   = 0.0f;
        dcm_     = true;
        return;
    }

    if (boost_) {
        stepOneCycleBoost(T);
    } else {
        stepOneCycleBuck(T);
    }

    // Source/sink + cap dynamics. V_in: forward-Euler (PV+C_in time constant is
    // bounded). V_out: backward-Euler — unconditionally stable, lets r_bat go
    // arbitrarily small without crossing the T/(R·C)=2 cliff (short-circuit case).
    //   V_out_new = (V_out + T·I_out_avg/C_out + a·V_bat) / (1 + a),   a = T/(R_bat·C_out)
    const float Ipv = pvCurrent(vIn_);

    vIn_ += (Ipv - iInAvg_) * T * invCin_;
    vOut_ = (vOut_ + (T * iOutAvg_) * invCout_ + aOut_ * vBatEff) * inv1pAout_;

    if (vIn_ < 0.0f) vIn_ = 0.0f;
    const float vInMax = pv_.voc * 1.05f;
    if (vIn_ > vInMax) vIn_ = vInMax;
    // KNOWN SOLVER LIMITATION (pre-existing, affects buck and boost alike). iL is advanced using
    // the OLD vOut_, so the L–C_out loop is forward-Euler and grows energy at (w0*T)^2/4 per cycle
    // against damping zeta*w0*T. It is unstable whenever
    //     zeta < w0*T/4,      w0 = 1/sqrt(L*C_out),  zeta = (1/2)*sqrt(L/C_out)/r_bat
    // i.e. at LIGHT LOAD / low damping. The clamps below do not cause it — with them removed the
    // trajectory diverges to NaN rather than ringing; they merely bound the divergence into a
    // limit cycle (~1 Hz, unrelated to the LC frequency). Measured: boost D=0.3 C_out=470uF grows
    // +0.30%/cycle at r_bat=100 ohm, is marginal at 12 ohm, decays at 2 ohm.
    //
    // This IS firmware-reachable: buck.h commands rect = driverPwmMax - pwmCtrl - 1 in CCM, and the
    // limit cycle appears at 1, 2 and 4 counts of slack. So a lightly loaded vconv rig can show a
    // wild Vout/iL oscillation that is a model defect, not a controller bug — check the criterion
    // above before blaming the loop. Conversion-ratio tests pick a damped operating point (see
    // vconv-test W, and A for buck).
    if (vOut_ < 0.0f) vOut_ = 0.0f;
    // Defensive ceiling: 2·V_bat for normal operation. When V_bat ≈ 0 (open-circuit
    // preset: r_bat≫1, v_bat=0) fall back to 2·Voc so V_out can charge up to the
    // firmware's OVP trip instead of being pinned at zero. Boost adds the boost-ratio
    // headroom: V_out legitimately sits well above both on this topology.
    float vOutMax = std::max(vbat_ * 2.0f, pv_.voc * 2.0f);
    // Boost: buck.h caps duty at pwmCtrlMax = 0.9*driverPwmMax, so the ideal ratio reaches 10, and
    // vIn_ is itself allowed up to 1.05*voc — a LEGITIMATE operating point is therefore 10.5*voc.
    // Anything tighter silently pins the plant so it stops responding to duty, which makes a Vout
    // loop look stable when it is not. 12x leaves margin over that; keep it in step with
    // MaxBoostRatio / pwmCtrlMax in buck.h.
    if (boost_) vOutMax = std::max(vOutMax, pv_.voc * 12.0f);
    if (vOut_ > vOutMax) vOut_ = vOutMax;
}

// Buck: inductor in series with the OUTPUT. Ctrl = HS, Rect = LS.
void VirtualConverter::stepOneCycleBuck(float T) {
    const uint16_t ctrl = pwm_.pwmCtrl;
    const uint16_t rect = pwm_.pwmRect;

    const float dHS = (float) ctrl * invPmax_;
    const float dLS = (float) rect * invPmax_;
    const float tHS = dHS * T;
    const float tLS = dLS * T;
    const float tOff = T - tHS - tLS;

    // Phase 1: HS on. dI/dt = (Vin - Vout) / L.
    const float a = iLEnd_;
    const float b = (tHS > 0.0f) ? (a + (vIn_ - vOut_) * invL_ * tHS) : a;
    const float areaHS = trapArea(a, b, tHS);

    // Phase 2: LS on. Signed trapezoid; no mode decision -- firmware picks
    // pwmRect to land near zero, forced-PWM commands past it.
    const float c = (tLS > 0.0f) ? (b - vOut_ * invL_ * tLS) : b;
    const float areaLS = trapArea(b, c, tLS);

    // Phase 3: both off. c > 0: LS body diode. c < 0: HS body diode (rev pump).
    float cEnd = c;
    float areaOff = 0.0f;
    if (tOff > 0.0f) {
        if (c > 0.0f) {
            const float slope = -vOut_ * invL_;     // <= 0; zero iff vOut_ == 0
            if (slope < 0.0f) {
                const float tZero = -c / slope;
                if (tZero < tOff) {
                    areaOff = 0.5f * c * tZero;
                    cEnd = 0.0f;
                } else {
                    cEnd = c + slope * tOff;
                    areaOff = trapArea(c, cEnd, tOff);
                }
            } else {
                // V_out == 0 (short): no back-EMF, body diode passes c through unchanged.
                cEnd = c;
                areaOff = c * tOff;
            }
        } else if (c < 0.0f) {
            const float slope = (vIn_ - vOut_) * invL_;
            if (slope > 0.0f) {
                const float tZero = -c / slope;
                if (tZero < tOff) {
                    areaOff = 0.5f * c * tZero;
                    cEnd = 0.0f;
                } else {
                    cEnd = c + slope * tOff;
                    areaOff = trapArea(c, cEnd, tOff);
                }
            } else {
                cEnd = c;
                areaOff = c * tOff;
            }
        }
    }

    iInAvg_  = areaHS * invT_;
    iOutAvg_ = (areaHS + areaLS + areaOff) * invT_;
    iLEnd_ = cEnd;
    // DCM := "coil is at zero at cycle end". c!=0 means phase 3 drove it down;
    // a==0 covers the staying-idle case (prior cycle ended at zero too).
    dcm_ = (cEnd == 0.0f) && (c != 0.0f || a == 0.0f);
}

// Boost: inductor in series with the INPUT. Ctrl = LS (charges the coil to ground),
// Rect = HS (delivers it to the output) — the role swap buck.h does via isBoost.
// So iL is the INPUT current in every phase, and only the Rect / HS-body-diode path
// reaches the output. That asymmetry is the whole difference from the buck model.
void VirtualConverter::stepOneCycleBoost(float T) {
    const float tCtrl = (float) pwm_.pwmCtrl * invPmax_ * T; // LS on
    const float tRect = (float) pwm_.pwmRect * invPmax_ * T; // HS on
    const float tOff = T - tCtrl - tRect;

    // Phase 1: Ctrl (LS) on — coil shorted to ground, dI/dt = Vin/L. The HS body
    // diode is reverse-biased here (SW at 0, V_out above it), so nothing reaches
    // the output.
    const float a = iLEnd_;
    const float b = (tCtrl > 0.0f) ? (a + vIn_ * invL_ * tCtrl) : a;
    const float areaCtrl = trapArea(a, b, tCtrl);

    // Phase 2: Rect (HS) on — dI/dt = (Vin - Vout)/L. Synchronous, so it conducts
    // both ways: negative iL here is reverse power flow, which the signed area keeps.
    const float c = (tRect > 0.0f) ? (b + (vIn_ - vOut_) * invL_ * tRect) : b;
    const float areaRect = trapArea(b, c, tRect);

    // Phase 3: both off. iL>0 → HS body diode to the output. iL<0 → LS body diode
    // to ground (input current, but NOT output current). iL==0 → the HS body diode
    // still conducts whenever V_out < V_in: that is the boost passthrough that sets the
    // V_out ≈ V_in floor.
    //
    // In steady state the floor holds for any commanded duty: the synchronous equilibrium is
    // V_out = V_in/(1-D) >= V_in for D < 1, verified in vconv-test X2. Reverse coil current does
    // NOT break it — an over-driven rect just bucks V_out back down toward that same ratio.
    // (A sub-V_in excursion with iL very negative is reachable, but only in the light-load regime
    // the forward-Euler criterion at the vOut_ clamp flags as unstable, i.e. it is a solver
    // artifact rather than a property of the topology.)
    //
    // The one real escape: pwmCtrl == pwmMax leaves tOff=0 and rect=0, so nothing reaches the
    // output and V_out collapses to 0 while the coil shorts the input. Out of reach only because
    // buck.h caps duty at pwmCtrlMax = 0.9*driverPwmMax — so the floor is duty-clamp enforced at
    // the top end, and should not be leaned on as a safety property there.
    float cEnd = c;
    float areaOff = 0.0f;    // inductor charge in phase 3 (all of it is input current)
    float areaOffOut = 0.0f; // the part that reaches the output
    if (tOff > 0.0f) {
        if (c > 0.0f) {
            const float slope = (vIn_ - vOut_) * invL_;
            if (slope < 0.0f) {
                const float tZero = -c / slope;
                if (tZero < tOff) {
                    areaOff = 0.5f * c * tZero;
                    cEnd = 0.0f;
                } else {
                    cEnd = c + slope * tOff;
                    areaOff = trapArea(c, cEnd, tOff);
                }
            } else {
                // V_out <= V_in: current RISES through the body diode and keeps
                // delivering — the converter cannot stop pumping V_out up to V_in.
                cEnd = c + slope * tOff;
                areaOff = trapArea(c, cEnd, tOff);
            }
            areaOffOut = areaOff;
        } else if (c < 0.0f) {
            const float slope = vIn_ * invL_; // >0 while Vin>0: rises back toward zero
            if (slope > 0.0f) {
                const float tZero = -c / slope;
                if (tZero < tOff) {
                    areaOff = 0.5f * c * tZero;
                    cEnd = 0.0f;
                } else {
                    cEnd = c + slope * tOff;
                    areaOff = trapArea(c, cEnd, tOff);
                }
            } else {
                cEnd = c;
                areaOff = c * tOff;
            }
            areaOffOut = 0.0f; // routed to ground, not the output
        } else {
            // Idle coil: passthrough only, and only while V_out < V_in.
            const float slope = (vIn_ - vOut_) * invL_;
            if (slope > 0.0f) {
                cEnd = slope * tOff;
                areaOff = 0.5f * cEnd * tOff;
                areaOffOut = areaOff;
            }
        }
    }

    iInAvg_ = (areaCtrl + areaRect + areaOff) * invT_;
    iOutAvg_ = (areaRect + areaOffOut) * invT_;
    iLEnd_ = cEnd;
    dcm_ = (cEnd == 0.0f) && (c != 0.0f || a == 0.0f);
}

void VirtualConverter::stepSeconds(float dt_s, uint32_t pwmFreqFallback) {
    if (dt_s <= 0.0f) return;
    uint32_t freq = pwm_.pwmFreq ? pwm_.pwmFreq : pwmFreqFallback;
    if (freq < 1000) freq = 39000;
    const float T = 1.0f / (float) freq;

    long n = std::lround(dt_s / T);
    if (n < 1) n = 1;
    for (long i = 0; i < n; ++i) stepOneCycle(T);
}
