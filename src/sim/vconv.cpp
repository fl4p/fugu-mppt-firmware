#include "vconv.h"

#include <cstdio>

VirtualConverter g_vconv;

namespace {
inline float trapArea(float y0, float y1, float t) {
    return 0.5f * (y0 + y1) * t;
}
constexpr float kTwoPi = 6.28318530717958647692f;

// free ring segment x(s) = e^(-a s) (x0 cos + B sin)(wd s), i(s) = e^(-a s) (i0 cos + Bi sin)(wd s)
struct RingSeg {
    float a, wd, x0, B, i0, Bi;
    __attribute__((noinline)) void at(float s, float &x, float &i) const {
        const float e = std::exp(-a * s), cs = std::cos(wd * s), sn = std::sin(wd * s);
        x = e * (x0 * cs + B * sn), i = e * (i0 * cs + Bi * sn);
    }
};

// coil current i through a body diode for t at slope di/dt, stopping at zero; area = its charge
__attribute__((noinline)) float diodeRun(float i, float t, float slope, float &area) {
    if (i * slope >= 0.0f) return area = i * t, i;
    const float tZero = -i / slope;
    if (tZero < t) return area = 0.5f * i * tZero, 0.0f;
    const float e = i + slope * t;
    return area = trapArea(i, e, t), e;
}

// first phase in (0, pi] where y cos + z sin... = 0, i.e. atan2(y, z) mod pi
__attribute__((noinline)) float firstZero(float y, float z) {
    constexpr float kPi = kTwoPi * 0.5f;
    float th = std::atan2(y, z);
    th -= kPi * std::floor(th / kPi);
    return th < 1e-4f ? th + kPi : th;
}
}

void VirtualConverter::stepOneCycle(float T) {
    const float L = l_;
    const uint16_t pmax = pwm_.pwmMax;
    const uint16_t ctrl = pwm_.pwmCtrl;
    const uint16_t rect = pwm_.pwmRect;

    // Rebuild reciprocals only when an input changes; the steady N-cycle loop then divides nothing
    // (LX7 has no hw float divide, so each `/` was a software __divsf3). 6 compares << 18 divisions.
    // Vin-dependent terms (node C, 1/Vin of the loss charges) follow Vin in 1 % steps
    const bool nl = ringV_ > 0.0f && vIn_ > 1.0f;
    const bool vt = vIn_ > 1.0f && (ringV_ > 0.0f || tSw_ > 0.0f || rHs_ > 0.0f || rLs_ > 0.0f);
    if (T != cycT_ || pmax != cycPmax_ || l_ != cycL_ || cIn_ != cycCin_ || cOut_ != cycCout_ || rbat_ != cycRbat_ ||
        ringC_ != cycRingC_ || (vt && std::fabs(vIn_ - cycRingV_) > 0.01f * vIn_)) {
        cycT_ = T; cycPmax_ = pmax; cycL_ = l_; cycCin_ = cIn_; cycCout_ = cOut_; cycRbat_ = rbat_;
        cycRingC_ = ringC_, cycRingV_ = vIn_;
        invVin_ = vIn_ > 1e-3f ? 1.0f / vIn_ : 0.0f;
        ringCe_ = nl && ringC_ > 0.0f ? ringC_ * std::pow(ringV_ / vIn_, ringExp_) : ringC_;
        cn_ = ringCt_ * ringCe_;
        if (ringCe_ > 0.0f && l_ > 0.0f) {
            const float w0 = 1.0f / std::sqrt(l_ * ringCe_), r = 0.25f / (ringQ_ * ringQ_);
            ringA_ = 0.5f * w0 / ringQ_;
            ringWd_ = w0 * std::sqrt(std::max(1.0f - r, 1e-4f));
            ringInvWd_ = 1.0f / ringWd_;
            ringInvC_ = 1.0f / ringCe_;
        }
        if (cn_ > 0.0f && l_ > 0.0f) {
            // undamped edge swing over the HS->LS dead time
            const float z = std::sqrt(l_ / cn_), wt = std::max(0.0f, tDead_ - tHsExt_) / std::sqrt(l_ * cn_);
            edgeZ_ = z, edgeInvZ_ = 1.0f / z, edgeCos_ = std::cos(wt), edgeSin_ = std::sin(wt);
            swK_ = tSw_ * tSw_ / (24.0f * cn_);
        }
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
    vbNow_ = vBatEff;
    struct HsRestore { uint16_t &r; uint16_t v; ~HsRestore() { r = v; } } hsRestore{pwm_.pwmCtrl, ctrl}, lsRestore{pwm_.pwmRect, rect};
    if (hsShort && !boost_ && ctrl < pmax && pmax) {
        if (rect && hsShort != 2) { // shorted HS + LS on: C_in dumps, the node sees ground
            vIn_ = 0.0f;
            iLEnd_ -= (vOut_ + 0.01f * iLEnd_) * T * invL_; // 10 mOhm loop
            iInAvg_ = 0.0f;
            iOutAvg_ = iLEnd_;
            vOut_ = std::max(0.0f, outNext(vOut_, iOutAvg_, T));
            return;
        }
        pwm_.pwmCtrl = pmax;
        if (hsShort == 2) pwm_.pwmRect = 0;
    }

    constexpr float kVMin = 1e-4f;
    if (vIn_ < kVMin || pmax == 0 || L <= 0.0f) {
        const float Ipv = pvBlocking_ ? std::max(0.f, pvCurrent(vIn_)) : pvCurrent(vIn_);
        vIn_  += Ipv * T * invCin_;
        vOut_ = outNext(vOut_, 0.0f, T);
        if (vIn_  < 0.0f) vIn_  = 0.0f;
        if (vOut_ < 0.0f) vOut_ = 0.0f;
        iInAvg_  = 0.0f;
        iOutAvg_ = 0.0f;
        iLEnd_   = 0.0f;
        dcm_     = true;
        return;
    }

    // L-C_out and L-C_in loops: pass 1 at the old V_out/V_in predicts their end values, pass 2 reruns the cycle
    // from the same coil state at the mid-cycle V_out/V_in. The charges are weighted (q1 + 2 q2)/3, which makes
    // V_out exact to (w0 T)^3 and removes the midpoint's (w0 T)^2/12 frequency error. One pass at the old voltages
    // is forward Euler there and gains energy every cycle (open battery + synchronous LS: runaway LC ring).
    const float v0 = vOut_, vi0 = vIn_, iL0 = iLEnd_, iPk0 = iPk_;
    // V_out: backward-Euler in r_bat, unconditionally stable for any r_bat (short-circuit case):
    //   V_out_new = (V_out + T·I_out_avg/C_out + a·V_bat) / (1 + a),   a = T/(R_bat·C_out)
    // V_in: the PV current is linearized at the cycle start and taken implicit (backward Euler); explicit, a stiff
    // source (bench PSU, k -> 1: T G / C_in > 2) rings or settles where I_pv(V_in) != I_in. Above Voc the panel
    // diode sinks what the HS body diode pumps back (curve slope at Voc), implicit for the same reason.
    const float ipv0 = pvBlocking_ ? std::max(0.f, pvCurrent(vi0)) : pvCurrent(vi0);
    const float g = std::max(0.0f, (ipv0 - pvCurrent(vi0 + 0.01f)) * 100.0f); // -dI_pv/dV
    const float hIn = T * invCin_ / (1.0f + g * T * invCin_);
    auto vinNext = [&](float iIn) {
        float v = vi0 + (ipv0 - iIn) * hIn;
        if (!pvBlocking_ && v > pv_.voc) {
            const float k = T * invCin_ * pv_.isc * pv_.alpha / pv_.voc;
            v = (v + k * pv_.voc) / (1.0f + k);
        }
        return std::max(0.0f, v);
    };
    float qOut1 = 0.0f, qIn1 = 0.0f;
    for (int pass = 0; pass < 2; ++pass) {
        if (boost_) stepOneCycleBoost(T);
        else stepOneCycleBuck(T);
        if (pass) break;
        qOut1 = iOutAvg_, qIn1 = iInAvg_;
        const float vPred = std::max(0.0f, outNext(v0, iOutAvg_, T));
        vOut_ = 0.5f * (v0 + vPred), vIn_ = 0.5f * (vi0 + vinNext(iInAvg_)), iLEnd_ = iL0, iPk_ = iPk0;
    }
    vOut_ = v0;
    iOutAvg_ = (qOut1 + 2.0f * iOutAvg_) * (1.0f / 3.0f);
    iInAvg_ = (qIn1 + 2.0f * iInAvg_) * (1.0f / 3.0f);

    vIn_ = vinNext(iInAvg_);
    vOut_ = outNext(vOut_, iOutAvg_, T);
    if (vOut_ > vOutMax_) vOutMax_ = vOut_;

    // Safety bound only: a sinking panel stays near Voc on its own; a blocking source (open backflow switch)
    // has C_in integrate the pumped-back charge until the firmware's Vin OVP trips
    const float vInMax = pvBlocking_ ? std::max(200.0f, 2.0f * pv_.voc) : pv_.voc * 1.05f;
    if (vIn_ > vInMax) vIn_ = vInMax, ++vInClamps_;
    // The two-pass step above leaves a slight numerical damping of the undamped L-C_out ring, ~(w0 T)^4/24 per
    // cycle (w0 = 1/sqrt(L C_out)); the single-pass forward-Euler step it replaced grew it instead.
    if (vOut_ < 0.0f) vOut_ = 0.0f;
    // Defensive ceiling: 2·V_bat for normal operation. When V_bat ≈ 0 (open-circuit
    // preset: r_bat≫1, v_bat=0) fall back to 2·Voc so V_out can charge up to the
    // firmware's OVP trip instead of being pinned at zero. Boost adds the boost-ratio
    // headroom: V_out legitimately sits well above both on this topology.
    float vOutMax = std::max(vbat_ * 2.0f, pv_.voc * 2.0f);
    // Boost: buck.h caps duty at pwmCtrlMax = 0.9*driverPwmMax, so the ideal ratio reaches 10, and
    // vIn_ is itself allowed up to 1.05*voc (blocking source: above) — a LEGITIMATE operating point is therefore 10.5*voc.
    // Anything tighter silently pins the plant so it stops responding to duty, which makes a Vout
    // loop look stable when it is not. 12x leaves margin over that; keep it in step with
    // MaxBoostRatio / pwmCtrlMax in buck.h.
    if (boost_) vOutMax = std::max(vOutMax, std::max(pv_.voc, vIn_) * 12.0f);
    if (vOut_ > vOutMax) vOut_ = vOutMax;
}

// Buck: inductor in series with the OUTPUT. Ctrl = HS, Rect = LS.
void VirtualConverter::stepOneCycleBuck(float T) {
    float invL = invL_;
    if (lK_ != 0.0f) invL /= std::max(0.1f, 1.0f - lK_ * iPk_);

    // realised gate windows: tDead_ before each rising edge (an LS pulse shorter than that is swallowed), the HS
    // turns off tHsExt_ late; from HS turn-on to the next one is T
    const float tH = (float) pwm_.pwmCtrl * invPmax_ * T, tL = (float) pwm_.pwmRect * invPmax_ * T;
    const bool lsOn = tL > std::max(tDead_, tHsExt_);
    const float hsDelay = lsOn ? std::max(0.0f, tDead_ - (T - tH - tL)) : 0.0f;
    const float tHS = tH > 0.0f ? std::min(T, std::max(0.0f, tH + tHsExt_ - hsDelay)) : 0.0f;
    const float tDd = lsOn ? std::max(0.0f, tDead_ - tHsExt_) : 0.0f;
    const float tLS = lsOn ? tL - std::max(tDead_, tHsExt_) : 0.0f;
    const float tOff = std::max(0.0f, T - tHS - tDd - tLS);

    // body-diode freewheel: i > 0 LS diode (node -Vf), i < 0 HS diode (node Vin + Vf)
    const float slP = -(vOut_ + diodeVf_) * invL, slN = (vIn_ + diodeVf_ - vOut_) * invL;
    auto diode = [slP, slN](float i, float t, float &area) { return diodeRun(i, t, i > 0.0f ? slP : slN, area); };

    // Phase 1: HS on. dI/dt = (Vin - Vout - rHs*i) / L. The current drop uses the mean current; the ripple part of
    // R*int(i^2), R*t*(b-a)^2/12, is paid from the input. Switch losses below are charges at Vin.
    const float a = iLEnd_;
    const float dv = vIn_ - vOut_;
    float b = a + dv * invL * tHS;
    float qIn = 0.0f;
    if (rHs_ > 0.0f) {
        b = a + (dv - rHs_ * 0.5f * (a + b)) * invL * tHS;
        qIn += rHs_ * tHS * (b - a) * (b - a) * (1.0f / 12.0f) * invVin_;
    }
    iPk_ = std::fabs(b);
    const float areaHS = trapArea(a, b, tHS);
    // HS turn-off: the channel current falls linearly in tSw_, the edge C takes the rest until the node reaches
    // 0 V at t1 = sqrt(2 cn Vin tSw / b). E/Vin = b^2 tSw^2 / (24 cn Vin) for t1 >= tSw, else
    // b tSw / 2 - 2/3 b t1 + cn Vin / 2
    if (tSw_ > 0.0f && tHS > 0.0f && tHS < T && b > 0.0f) {
        const float h = 2.0f * cn_ * vIn_;
        qIn += b * tSw_ <= h ? b * b * swK_ * invVin_
                             : b * (0.5f * tSw_) - (2.0f / 3.0f) * std::sqrt(h * tSw_ * b) + 0.25f * h;
    }

    // dead time HS off -> LS on. A positive current swings the node down along the undamped L-cn resonance; it
    // clamps at the LS body diode (-Vf) or, if it gets there late, the LS turns on hard and the rest of the node
    // energy is lost
    float areaDd = 0.0f, b1 = b;
    if (tDd > 0.0f) {
        if (cn_ > 0.0f && b > 0.0f) {
            const float xLo = -(vOut_ + diodeVf_), x1 = dv * edgeCos_ - b * edgeZ_ * edgeSin_;
            if (x1 > xLo) {
                b1 = b * edgeCos_ + dv * edgeInvZ_ * edgeSin_;
                areaDd = cn_ * (dv - x1);
            } else {
                // clamped: current from the swing energy, swing time from its mean current
                b1 = std::sqrt(b * b + cn_ * (dv * dv - xLo * xLo) * invL_);
                const float q = cn_ * (dv - xLo), tHit = std::min(tDd, 2.0f * q / (b + b1));
                b1 = diode(b1, tDd - tHit, areaDd);
                areaDd += q;
            }
        } else {
            b1 = diode(b, tDd, areaDd);
            if (b < 0.0f) qIn += areaDd;
        }
    }

    // Phase 2: LS on. Signed trapezoid; no mode decision -- firmware picks
    // pwmRect to land near zero, forced-PWM commands past it.
    float c = b1 - vOut_ * invL * tLS;
    if (rLs_ > 0.0f) {
        c = b1 - (vOut_ + rLs_ * 0.5f * (b1 + c)) * invL * tLS;
        qIn += rLs_ * tLS * (c - b1) * (c - b1) * (1.0f / 12.0f) * invVin_;
    }
    const float areaLS = trapArea(b1, c, tLS);
    iLsOff_ = c;

    // Phase 3: both off. c > 0: LS body diode. c < 0: HS body diode (rev pump).
    float cEnd = c;
    float areaOff = 0.0f;
    bool ring = false, zero = false;
    // node voltage when the HS turns on next
    float vNode = lsOn ? 0.0f : vIn_;
    if (ringCe_ > 0.0f && tOff > 0.0f && tHS > 0.0f && vOut_ > 0.0f && vIn_ > vOut_) {
        // switch-node ring from the node voltage the last switch left (LS: 0, HS: Vin)
        float x = (tLS > 0.0f ? 0.0f : vIn_) - vOut_, i = c;
        areaOff = ringOff(x, i, tOff, qIn, zero);
        vNode = x + vOut_;
        cEnd = i;
        ring = true;
    } else if (tOff > 0.0f) {
        cEnd = diode(c, tOff, areaOff);
        if (c < 0.0f) qIn += areaOff; // reverse current through the HS body diode returns to the input
        vNode = cEnd > 0.0f ? -diodeVf_ : cEnd < 0.0f ? vIn_ + diodeVf_ : vOut_;
    }
    // the HS turn-on charges the node back to Vin from the input: the edge C, or the ring C when the free ring
    // started at the HS turn-off (no LS phase) and so discharged the full ring C
    if (tHS > 0.0f) qIn += (ring && tLS <= 0.0f ? ringCe_ : cn_) * std::max(0.0f, vIn_ - vNode);

    iInAvg_  = (areaHS + qIn) * invT_;
    iOutAvg_ = (areaHS + areaDd + areaLS + areaOff) * invT_;
    iLEnd_ = cEnd;
    // DCM := "coil is at zero at cycle end". c!=0 means phase 3 drove it down;
    // a==0 covers the staying-idle case (prior cycle ended at zero too).
    dcm_ = ring ? zero : (cEnd == 0.0f) && (c != 0.0f || a == 0.0f);
}

// Switch-node ring while both buck switches are off: coil L, node capacitance C (FET Coss, winding,
// layout), damping conductance G = 1/(Q Z0) from the node to the output. x = Vnode - Vout, i = coil
// current toward the output:
//   C dx/dt = -i - G x,  L di/dt = x
//   x(s) = e^(-a s) (x0 cos(wd s) + B sin(wd s)),  B = -(a x0 + i0/C) / wd,
//   a = w0/(2Q),  wd = w0 sqrt(1 - 1/(4Q^2))
// so a free segment moves C (x0 - x1) to the output (coil and G). The body diodes clamp x at
// -(Vout + Vf) (LS) and Vin + Vf - Vout (HS); clamped, the coil current ramps to zero at that
// voltage and the ring restarts from rest. x is monotonic between its extrema, so each half-period is tested for a clamp
// while the envelope can still reach one. Advances x, i by t; returns the charge to the output, adds
// the charge the HS body diode returns to the input to qIn (<= 0); zero: the coil current reached zero.
float VirtualConverter::ringOff(float &x, float &i, float t, float &qIn, bool &zero) const {
    constexpr float kPi = kTwoPi * 0.5f;
    const float xLo = -(vOut_ + diodeVf_), xHi = vIn_ + diodeVf_ - vOut_, lim = std::min(-xLo, xHi);
    const float C = ringCe_, a = ringA_, wd = ringWd_, iwd = ringInvWd_;
    float q = 0.0f;
    for (int seg = 0; seg < 8; ++seg) {
        const float x0 = x, i0 = i;
        const float B = -(a * x0 + i0 * ringInvC_) * iwd;
        const RingSeg g{a, wd, x0, B, i0, C * (wd * x0 - a * B)};
        // phases of the first zero of i and of dx/ds (x extremum)
        const float thI = firstZero(-i0, g.Bi), th = firstZero(a * x0 - wd * B, -a * B - wd * x0);
        const float env = std::sqrt(x0 * x0 + B * B);
        float s0 = 0.0f, xs0 = x0, sHit = -1.0f, xc = 0.0f;
        for (int k = 0; seg < 7 && k < 64 && s0 < t && env * std::exp(-a * s0) > lim; ++k) {
            const float s1 = std::min(t, (th + kPi * (float) k) * iwd);
            float xs1, is1;
            g.at(s1, xs1, is1);
            if (xs1 > xHi || xs1 < xLo) {
                xc = xs1 > xHi ? xHi : xLo;
                // monotonic on [s0, s1]: secant start, safeguarded Newton (dx/ds = -(i + G x)/C)
                float lo = s0, hi = s1, s = s0 + (s1 - s0) * (xc - xs0) / (xs1 - xs0);
                for (int n = 0; n < 4; ++n) {
                    float xs, is;
                    g.at(s, xs, is);
                    if ((xs - xc) * (xs1 - xc) > 0.0f) hi = s; else lo = s;
                    const float ig = is + 2.0f * a * C * xs;
                    const float sn = ig != 0.0f ? s + (xs - xc) * C / ig : 0.5f * (lo + hi);
                    s = sn > lo && sn < hi ? sn : 0.5f * (lo + hi);
                }
                sHit = s;
                break;
            }
            s0 = s1, xs0 = xs1;
        }
        if (sHit < 0.0f) {
            g.at(t, x, i);
            if (thI * iwd < t) zero = true;
            return q + C * (x0 - x);
        }
        float i1;
        g.at(sHit, x, i1);
        q += C * (x0 - xc);
        x = xc, t -= sHit;
        const float tz = std::fabs(i1) * l_ / std::fabs(xc);
        if (tz >= t) {
            i = i1 + xc * invL_ * t;
            const float dq = 0.5f * (i1 + i) * t;
            if (xc > 0.0f) qIn += dq;
            return q + dq;
        }
        const float dq = 0.5f * i1 * tz;
        if (xc > 0.0f) qIn += dq;
        q += dq, t -= tz, i = 0.0f, zero = true;
    }
    return q;
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

// Isc ~ G; Voc falls 4 % per e-fold of G, and below G = 1e-3 linearly toward a dead panel
void VirtualConverter::applySun(float g) {
    constexpr float kGKnee = 1e-3f, kVocKnee = 0.7236897f; // 1 + 0.04 ln(kGKnee)
    sunG_ = g;
    const float voc = g >= kGKnee ? voc0_ * (1.0f + 0.04f * std::log(g)) : voc0_ * kVocKnee * g * (1.0f / kGKnee);
    pv_.set(std::max(isc0_ * g, 1e-4f), std::max(voc, 0.5f), pv_.k);
    xDirty_ = true;
}

void VirtualConverter::stepSeconds(float dt_s, uint32_t pwmFreqFallback) {
    if (dt_s <= 0.0f) return;
    if ((sun_.day > 0.0f || sunG_ != 1.0f) && (sunAcc_ += dt_s) >= 0.05f) {
        const float g = sun_.day > 0.0f ? sun_.step(sunAcc_) : 1.0f;
        sunAcc_ = 0.0f;
        if (std::fabs(g - sunG_) > 0.002f * sunG_ || (g == 1.0f && sunG_ != 1.0f)) applySun(g);
    }
    uint32_t freq = pwm_.pwmFreq ? pwm_.pwmFreq : pwmFreqFallback;
    if (freq < 1000) freq = 39000;
    const float T = 1.0f / (float) freq;

    // carry the sub-cycle remainder so plant time tracks the caller's clock
    cycAcc_ += dt_s * (float) freq;
    const long n = std::lround(cycAcc_);
    cycAcc_ -= (float) n;
    for (long i = 0; i < n; ++i) stepOneCycle(T);
    if (bfModel) {
        const float vb = vbNow_ + (bfConducts() ? 0.0f : bfVf);
        ibat_ = vOut_ > vb || bfConducts() ? (vOut_ - vb) / rbat_ : 0.0f;
        vSense_ = vbNow_ + ibat_ * rbat_;
    }
}

// output node: C_out, and the battery behind r_bat (backward Euler); a blocking BF (bflow_model) passes only forward
float VirtualConverter::outNext(float v, float iOut, float T) const {
    const float vn = v + (T * iOut) * invCout_;
    if (!bfModel) return (vn + aOut_ * vbNow_) * inv1pAout_;
    const bool on = bfConducts();
    const float vb = vbNow_ + (on ? 0.0f : bfVf);
    const float vc = (vn + aOut_ * vb) * inv1pAout_;
    return on || vc > vb ? vc : vn;
}
