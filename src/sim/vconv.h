#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <vector>

#include "math/pv_model.h"

// Irradiance G in [0, 1]: a clear-sky day (flattened half-sine) and night, repeating, times a Markov
// cloud factor with soft edges. Deterministic for a seed. day = 0: off.
struct SunModel {
    float day = 0.0f;    // s, 0 = off (min 1 s)
    float night = -1.0f; // s, < 0: day / 4
    float peak = 1.0f;   // clear-sky G at noon
    float rate = 0.0f;   // cloud starts per minute
    float depth = 0.7f;  // max cloud dip; a cloud dips uniform in [0.3, 1] x depth
    float len = 20.0f;   // mean cloud duration, s (1..600)
    float edge = 3.0f;   // cloud edge time constant, s
    float t = 0.0f, gClear = 1.0f, cloud = 1.0f, g = 1.0f;
    uint32_t seed = 1;

    void setDay(float d) {
        if (!(day > 0.0f) && d > 0.0f) t = 0.0f;
        day = d > 0.0f ? std::max(d, 1.0f) : 0.0f;
    }
    void reset(uint32_t s) { seed = rng_ = s ? s : 1; t = 0.0f; cloud = target_ = 1.0f; left_ = 0.0f; }
    [[nodiscard]] float nightS() const { return night >= 0.0f ? night : 0.25f * day; }

    float step(float dt) {
        constexpr float kPi = 3.14159265f;
        const float period = day + nightS();
        t += dt;
        if (t >= period) t = std::fmod(t, period);
        gClear = t < day ? std::fmin(std::fmax(peak, 0.0f), 1.0f) *
                           std::pow(std::fmax(std::sin(kPi * t / day), 0.0f), 1.3f) : 0.0f;
        if (left_ > 0.0f) left_ -= dt;
        else if (uni() < std::fmax(rate, 0.0f) * (1.0f / 60.0f) * dt) {
            target_ = 1.0f - std::fmin(std::fmax(depth, 0.0f), 1.0f) * (0.3f + 0.7f * uni());
            left_ = -std::fmin(std::fmax(len, 1.0f), 600.0f) * std::log(1.0f - uni());
        }
        if (left_ <= 0.0f) target_ = 1.0f;
        cloud += (target_ - cloud) * dt / (std::fmax(edge, 0.0f) + dt);
        return g = gClear * cloud;
    }

private:
    uint32_t rng_ = 1;
    float left_ = 0.0f, target_ = 1.0f;
    float uni() { // [0, 1)
        rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
        return (float) (rng_ >> 8) * (1.0f / 16777216.0f);
    }
};

// Pure-C++ model of a synchronous buck OR boost converter (setBoost()). No Arduino, no IDF,
// no FreeRTOS.
// Wire-up (PWM input, ADC output, conf parsing) lives in src/pwm/vconv.h,
// src/adc/vconv.h, and src/sensor_setup.cpp. Spec:
// doc/superpowers/specs/2026-05-23-virtual-converter-design.md
class VirtualConverter {
public:
    struct PwmState {
        uint16_t pwmMax = 0;
        uint16_t pwmCtrl = 0;   // on-count of the control switch: HS in buck, LS in boost
        uint16_t pwmRect = 0;   // on-count of the rectifier switch: LS in buck, HS in boost
        uint32_t pwmFreq = 0;   // Hz
    };

    // Topology. Mirrors converter.conf::topo — buck.h swaps the Ctrl/Rect gate roles the same
    // way, so the shim's ch0->Ctrl / ch1->Rect mapping needs no change.
    void setBoost(bool b) { boost_ = b; }
    [[nodiscard]] bool isBoost() const { return boost_; }

    // PV source curve (math/pv_model.h).
    // Base PV parameters; the sun model scales them.
    void setPv(float isc, float voc, float k) {
        isc0_ = isc, voc0_ = voc;
        if (sunG_ == 1.0f) pv_.set(isc, voc, k), xDirty_ = true;
        else pv_.k = k, applySun(sunG_);
    }
    SunModel &sun() { return sun_; }
    [[nodiscard]] float getSunG() const { return sunG_; } // irradiance applied to the PV curve

    // PV string of n (1..6) series substrings, each with a bypass diode: g[i] scales the Isc of
    // substring i, w[i] is its share of Voc (null = equal shares), vd the forward drop of a conducting
    // bypass diode. Gives up to n power peaks.
    // Approximation: the conducting substrings share the voltage by w at the current of the weakest of them.
    void setPvString(uint8_t n, const float *g, const float *w = nullptr, float vd = 0) {
        pvN_ = n < 1 ? 1 : n > kPvSubMax ? kPvSubMax : n;
        pvVd_ = vd;
        for (uint8_t i = 0; i < pvN_; ++i) {
            const float wi = w ? w[i] : 1.f / (float) pvN_;
            if (xg_[i] != g[i] || xw_[i] != wi) xg_[i] = g[i], xw_[i] = wi, xDirty_ = true;
        }
        uint8_t o[kPvSubMax] = {0, 1, 2, 3, 4, 5};
        for (uint8_t i = 1; i < pvN_; ++i) // insertion sort, as std::sort does at this size; GCC 15 -Warray-bounds
            for (uint8_t j = i; j > 0 && g[o[j]] > g[o[j - 1]]; --j) std::swap(o[j], o[j - 1]);
        float sum = 0;
        for (uint8_t m = 0; m < pvN_; ++m) {
            sum += w ? w[o[m]] : 1.f / (float) pvN_;
            pvG_[m] = g[o[m]];
            pvInvW_[m] = m + 1 == pvN_ ? 1.f : 1.f / sum;
        }
    }

    // Partial shade: half of the string delivers `shade` x Isc. 1 = unshaded.
    void setPvShade(float shade) {
        const float g[2] = {1.f, shade};
        setPvString(shade >= 1.f ? 1 : 2, g);
    }

    // String current at voltage v: best of "the m strongest substrings conduct, the rest bypassed".
    // exact series string: V(I) = sum over substrings of max(sum over halves w/2 V_pv(I/g), -vd), tabulated
    bool xExact_ = false; mutable bool xDirty_ = true;
    float xg_[6]{}, xw_[6]{}, xHalf_[6] = {1, 1, 1, 1, 1, 1};
    static constexpr int kTbl = 4096;
    mutable std::vector<float> xV_; // V(I) table, allocated only for the exact model (host sim)
    mutable float xImax_ = 0;
    void setPvExact(bool e) { xExact_ = e, xDirty_ = true; }
    void setPvHalf(int i, float f) { xHalf_[i] = f, xDirty_ = true; }
    float xVsub(int k, float I) const {
        float v = 0;
        for (int h = 0; h < 2; ++h) {
            const float gh = xg_[k] * (h ? xHalf_[k] : 1.f);
            if (I >= gh * pv_.isc * 0.99999f) { v = -1e4f; break; }
            v += 0.5f * xw_[k] * pv_.voltage(I / gh);
        }
        return std::max(v, -pvVd_);
    }
    void xBuild() const {
        float im = 0;
        for (int k = 0; k < pvN_; ++k) im = std::max(im, xg_[k] * pv_.isc * std::max(1.f, xHalf_[k]));
        xImax_ = im;
        xV_.resize(kTbl + 1);
        for (int j = 0; j <= kTbl; ++j) {
            const float I = im * (float) j / (float) kTbl * 0.99998f;
            float v = 0;
            for (int k = 0; k < pvN_; ++k) v += xVsub(k, I);
            xV_[j] = v;
        }
        xDirty_ = false;
    }
    [[nodiscard]] float pvCurrent(float v) const {
        if (xExact_) {
            if (xDirty_) xBuild();
            if (v >= xV_[0]) return 0;
            if (v <= xV_[kTbl]) return xImax_;
            int lo = 0, hi = kTbl; // xV_ decreasing
            while (hi - lo > 1) { int m = (lo + hi) / 2; if (xV_[m] > v) lo = m; else hi = m; }
            const float f = (xV_[lo] - v) / std::max(1e-9f, xV_[lo] - xV_[hi]);
            return xImax_ * ((float) lo + f) / (float) kTbl * 0.99998f;
        }
        float i = pvG_[pvN_ - 1] * pv_.current(v);
        for (uint8_t m = 0; m + 1 < pvN_; ++m)
            i = std::max(i, pvG_[m] * pv_.current((v + (float) (pvN_ - 1 - m) * pvVd_) * pvInvW_[m]));
        return i;
    }

    // Battery-side backflow switch between the output node and the battery (opt-in): on conducts both ways,
    // off is a diode (bfVf) toward the battery. Then Vout/Iout sense the battery side (BT+, battery current).
    bool bfModel = false, bfOn = false;
    uint8_t bfStuck = 0; // 1 stuck open, 2 stuck closed
    float bfVf = 0.7f;
    uint8_t hsShort = 0; // HS conducts permanently (buck); 2: and the LS never turns on
    bool iUnidir = false; // the Iout channel clips at 0 (one-direction current amp)
    [[nodiscard]] bool bfConducts() const { return bfStuck ? bfStuck == 2 : bfOn; }
    [[nodiscard]] float getVoutSense() const { return bfModel ? vSense_ : vOut_; }
    [[nodiscard]] float getIoutSense() const { return bfModel ? ibat_ : iOutAvg_; }

    // Open backflow switch at the PV input: its body diode passes only forward PV current.
    void setPvBlocking(bool b) { pvBlocking_ = b; }

    void setBat(float vbat, float rbat) {
        vbat_ = vbat;
        // Guard against r_bat → 0: divides in stepOneCycle (BE coefficient and
        // kVMin branch) would produce inf/NaN. 1 µΩ floor is well below any
        // realistic short impedance and keeps the math finite.
        rbat_ = (rbat < 1e-6f) ? 1e-6f : rbat;
    }
    // Pluggable V_bat ripple shape: maps phase [0,2pi) -> a zero-mean waveform in ~[-1,1], scaled
    // by vbat_ac_amp. Add a new noise model by writing one of these (anywhere) and passing it to
    // setBatRippleShape(); for conf/runtime selection by integer id, also list it in shapeFromId().
    using RippleShape = float (*)(float phase);
    static float shapeSine(float phase) { return std::sin(phase); }                 // inverter (cos 2*line)
    static float shapeAbsSin(float phase) {                                          // rectifier load |sin|
        return 2.0f * (std::fabs(std::sin(phase * 0.5f)) - 0.63661977f);            // zero-mean, ~20% 2nd harm
    }
    // Cheap-inverter spike train: a narrow current pulse once per cycle (crest ~3, ~20% duty),
    // harmonic-rich so it aliases past Nyquist when sampled — models the nasty China-inverter load
    // whose pulses corrupt the current estimate. ((1+cos)/2)^8 via 3 squarings (no exp), zero-mean.
    static float shapeSpiky(float phase) {
        float u = 0.5f * (1.0f + std::cos(phase));
        u *= u; u *= u; u *= u;                          // u^8 -> narrow peak
        return (u - 0.196381f) * 1.244370f;              // zero-mean, peak +1, trough ~-0.244
    }
    static RippleShape shapeFromId(int id) {
        switch (id) {
            case 1:  return &VirtualConverter::shapeAbsSin;
            case 2:  return &VirtualConverter::shapeSpiky;
            default: return &VirtualConverter::shapeSine;
        }
    }

    // shape: built-in id (0 = sine inverter, 1 = |sin| rectifier, 2 = spiky China-inverter pulse).
    // For a custom model use setBatRippleShape() instead.
    void setBatRipple(float amp, float freq, int shape = 0) {
        vbatAcAmp_ = amp; vbatAcFreq_ = freq; vbatAcShape_ = shape;
        vbatAcShapeFn_ = shapeFromId(shape);
        oscStep_ = -1.0f; // force sine-oscillator re-seed (freq/shape may have changed)
    }
    // Plug in an arbitrary ripple model (overrides the built-in shape; getVbatAcShape() -> -1).
    void setBatRippleShape(RippleShape fn) {
        if (fn) { vbatAcShapeFn_ = fn; vbatAcShape_ = -1; oscStep_ = -1.0f; }
    }
    void setPassives(float c_in, float c_out, float l) { cIn_ = c_in; cOut_ = c_out; l_ = l; }
    // Diode across the output, I = i0 (exp(V / vt) - 1): a dark PV string as a boost load (vt = n Ns kT/q). i0 <= 0: off
    void setOutDiode(float i0, float vt) { outDi0_ = i0 > 0.0f && vt > 0.0f ? i0 : 0.0f, outInvVt_ = vt > 0.0f ? 1.0f / vt : 0.0f; }
    [[nodiscard]] float outDiodeI(float v) const { return outDi0_ * (std::exp(std::min(v * outInvVt_, 80.0f)) - 1.0f); }
    void setVin(float v)  { vIn_ = v; }
    void setVout(float v) { vOut_ = vSense_ = v; }
    void setDiodeVf(float v) { diodeVf_ = v; } // body-diode drop (LS and HS) in the dead/off phases
    float ntcV = 0.9f; // NTC sense voltage, ~1.3 °C (25 °C is 1.65 V)
    // Buck DC bias: L = L0 * (1 - k*|Ipk|), Ipk the previous cycle's peak (k in 1/A, 0 = off). Only the HS/LS/diode slopes use the
    // biased L; the dead-time swing and its clamp energy, the off-phase ring and the boost branch keep L0.
    void setLBias(float k) { lK_ = k; }
    // Buck switch-node ring while both switches are off: node capacitance c (F) against the coil,
    // quality factor q; ring frequency 1/(2*pi*sqrt(L*c)). c <= 0 disables it (default).
    void setRing(float c, float q) { ringC_ = c > 0.0f ? c : 0.0f; ringQ_ = q > 0.5f ? q : 0.5f; cycRingC_ = -1.0f; }
    // Coss-like node C: c(Vin) = ring_c * (vRef / Vin)^e (vRef <= 0: constant); the hard edges move ct * c
    void setRingNode(float vRef, float e, float ct) { ringV_ = vRef; ringExp_ = e; ringCt_ = ct; cycRingC_ = -1.0f; }
    // Buck switch timing and losses (0 = ideal): dead time before each rising gate edge, HS on-time extension
    // (turn-off minus turn-on delay), conduction resistance while HS / LS conducts, HS turn-off current fall time
    void setSwitching(float tDead, float tHsExt, float rHs, float rLs, float tSw) {
        tDead_ = tDead, tHsExt_ = tHsExt, rHs_ = rHs, rLs_ = rLs, tSw_ = tSw, cycRingC_ = -1.0f;
    }
    [[nodiscard]] float getRingC() const { return ringC_; }
    [[nodiscard]] float getRingF() const { return ringCe_ > 0.0f ? 1.0f / (6.2831853f * std::sqrt(l_ * ringCe_)) : 0.0f; }
    [[nodiscard]] float getRingQ() const { return ringQ_; }
    [[nodiscard]] float getVin()  const { return vIn_; }
    [[nodiscard]] float getVout() const { return vOut_; }
    // per-cycle Vout maximum since the last call
    float takeVoutMax() { float m = vOutMax_; vOutMax_ = vOut_; return m; }
    // per-cycle minima since the last call: coil current at LS turn-off, cycle-mean output current
    void takeIMin(float &iLsOff, float &iOut) {
        iLsOff = iLsOffMin_, iOut = iOutMin_;
        iLsOffMin_ = iLsOff_, iOutMin_ = iOutAvg_;
    }
    [[nodiscard]] uint32_t vinClamps() const { return vInClamps_; } // cycles Vin hit its safety bound
    [[nodiscard]] float getIL()   const { return iLEnd_; }
    [[nodiscard]] float getILsOff() const { return iLsOff_; } // buck: coil current at LS turn-off (HS off: at its end)
    [[nodiscard]] float getIsc()  const { return pv_.isc; }
    [[nodiscard]] float getVoc()  const { return pv_.voc; }
    [[nodiscard]] float getPvK()  const { return pv_.k; }
    [[nodiscard]] float getVbat() const { return vbat_; }
    [[nodiscard]] float getRbat() const { return rbat_; }
    [[nodiscard]] float getCin()  const { return cIn_; }
    [[nodiscard]] float getCout() const { return cOut_; }
    [[nodiscard]] float getL()    const { return l_; }
    [[nodiscard]] float getVbatAcAmp()  const { return vbatAcAmp_; }
    [[nodiscard]] float getVbatAcFreq() const { return vbatAcFreq_; }
    [[nodiscard]] int   getVbatAcShape() const { return vbatAcShape_; }

    void setPwm(const PwmState &s) { pwm_ = s; }
    [[nodiscard]] const PwmState &getPwm() const { return pwm_; }
    [[nodiscard]] float getIinAvg()  const { return iInAvg_; }
    [[nodiscard]] float getIoutAvg() const { return iOutAvg_; }
    [[nodiscard]] bool  inDcm() const     { return dcm_; }
    [[nodiscard]] bool  errored() const   { return errored_; }

    // Set HS/LS on-counts from PwmDriver shim. ch=0 -> Ctrl (HS), ch=1 -> Rect (LS).
    // The shim is responsible for translating LEDC's hpoint/duty pair (and EnLogic's
    // cumulative single-arg form) into an absolute on-count before calling this.
    void setPwmCount(uint8_t ch, uint32_t on) {
        if (ch == 0) pwm_.pwmCtrl = (uint16_t) on;
        else if (ch == 1) pwm_.pwmRect = (uint16_t) on;
    }

    void setPwmMax(uint16_t m)   { pwm_.pwmMax = m; }
    void setPwmFreq(uint32_t hz) { pwm_.pwmFreq = hz; }
    // gate-driver timer clock the PWM shim emulates: 0 = LEDC (80 MHz, power-of-two period), else MCPWM bestTiming()
    void setPwmClk(uint32_t hz) { pwmClkHz_ = hz; }
    [[nodiscard]] uint32_t pwmClk() const { return pwmClkHz_; }

    void stopChannel(uint8_t ch) {
        if (ch == 0) pwm_.pwmCtrl = 0;
        else if (ch == 1) pwm_.pwmRect = 0;
    }

    // Advance the model by dt_s seconds, carrying the sub-cycle remainder. Without prior pwmFreq, falls back.
    void stepSeconds(float dt_s, uint32_t pwmFreqFallback);

private:
    bool boost_ = false;
    uint32_t pwmClkHz_ = 0;

    PvModel pv_;
    SunModel sun_;
    float isc0_ = 8.0f, voc0_ = 40.0f, sunG_ = 1.0f, sunAcc_ = 0.0f, cycAcc_ = 0.0f;
    void applySun(float g);
    static constexpr uint8_t kPvSubMax = 6;
public:
    static constexpr uint8_t kPvSubMaxX = 6;
private:
    uint8_t pvN_ = 1;
    float pvVd_ = 0;
    float pvG_[kPvSubMax] = {1.f, 1.f, 1.f, 1.f, 1.f, 1.f};    // substring factors, strongest first
    float pvInvW_[kPvSubMax] = {1.f, 1.f, 1.f, 1.f, 1.f, 1.f}; // 1 / Voc share of the m+1 strongest
    bool pvBlocking_ = false;

    // Battery + passives
    float vbat_ = 28.0f;
    float rbat_ = 0.05f;
    float vbatAcAmp_  = 0.0f;
    float vbatAcFreq_ = 100.0f;
    int   vbatAcShape_ = 0; // built-in id (0=sine, 1=|sin|); -1 = custom plugged via setBatRippleShape
    RippleShape vbatAcShapeFn_ = &VirtualConverter::shapeSine;
    float vbatAcPhase_ = 0.0f;
    float cIn_  = 470e-6f;
    float cOut_ = 470e-6f;
    float l_    = 50e-6f;
    float diodeVf_ = 0.0f;
    float lK_ = 0.0f, iPk_ = 0.0f;
    float outDi0_ = 0.0f, outInvVt_ = 0.0f;

    // State
    float vIn_   = 0.0f;
    float vOut_  = 0.0f;
    float vOutMax_ = 0.0f;
    uint32_t vInClamps_ = 0;
    float iLEnd_ = 0.0f, iLsOff_ = 0.0f;
    float iLsOffMin_ = 0.0f, iOutMin_ = 0.0f;
    PwmState pwm_{};
    float iInAvg_  = 0.0f;
    float iOutAvg_ = 0.0f;
    bool  dcm_     = false;
    bool  errored_ = false;
    float vbNow_ = 0.0f, ibat_ = 0.0f, vSense_ = 0.0f;
    float outNext(float v, float iOut, float T) const;

    // Cached reciprocals — the LX7 FPU has no hardware float divide, so every `/` is a software
    // __divsf3 call (~tens of cycles). stepOneCycle runs ~pwmFreq/adc_freq times per ADC sample, so
    // recomputing per cycle was the dominant cost. These are rebuilt only when an input changes
    // (see the guard in stepOneCycle); across the steady N-cycle loop every divide becomes a mul.
    float cycT_ = 0.0f, cycL_ = 0.0f, cycCin_ = 0.0f, cycCout_ = 0.0f, cycRbat_ = 0.0f;
    uint16_t cycPmax_ = 0xFFFFu;
    float invL_ = 0.0f, invCin_ = 0.0f, invCout_ = 0.0f, invT_ = 0.0f;
    float aOut_ = 0.0f, inv1pAout_ = 1.0f, invPmax_ = 0.0f;

    // Recursive sine oscillator for the built-in inverter ripple: advances (sin,cos) of the ripple
    // phase by a fixed per-cycle rotation (4 mul + 2 add), avoiding a per-cycle sinf. Re-seeded when
    // the rotation step changes; |sin| / custom shapes still go through the function pointer.
    float oscS_ = 0.0f, oscC_ = 1.0f, rotS_ = 0.0f, rotC_ = 1.0f, oscStep_ = -1.0f;

    // switch-node ring (setRing): C, Q; cached decay rate, damped angular frequency and reciprocals
    float ringC_ = 0.0f, ringQ_ = 20.0f, cycRingC_ = -1.0f, ringV_ = 0.0f, ringExp_ = 0.0f, ringCe_ = 0.0f, cycRingV_ = 0.0f, ringCt_ = 1.0f;
    float ringA_ = 0.0f, ringWd_ = 0.0f, ringInvWd_ = 0.0f, ringInvC_ = 0.0f;
    float tDead_ = 0.0f, tHsExt_ = 0.0f, rHs_ = 0.0f, rLs_ = 0.0f, tSw_ = 0.0f;
    // cached: 1/Vin, edge C, its impedance and cos/sin of the dead-time swing angle, tSw^2/(24 edge C)
    float invVin_ = 0.0f, cn_ = 0.0f, edgeZ_ = 0.0f, edgeInvZ_ = 0.0f, edgeCos_ = 1.0f, edgeSin_ = 0.0f, swK_ = 0.0f;

    void stepOneCycle(float T);
    void stepOneCycleBuck(float T);
    void stepOneCycleBoost(float T);
    float ringOff(float &x, float &i, float t, float &qIn, bool &zero) const;
};

// Singleton. Defined in src/sim/vconv.cpp.
extern VirtualConverter g_vconv;
