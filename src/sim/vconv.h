#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

#include "math/pv_model.h"

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
    void setPv(float isc, float voc, float k) { pv_.set(isc, voc, k); }

    // PV string of n (1..3) series substrings, each with a bypass diode: g[i] scales the Isc of
    // substring i, w[i] is its share of Voc (null = equal shares). Gives up to n power peaks.
    // Approximation: the conducting substrings share the voltage by w, no bypass-diode drop.
    void setPvString(uint8_t n, const float *g, const float *w = nullptr) {
        pvN_ = n < 1 ? 1 : n > kPvSubMax ? kPvSubMax : n;
        uint8_t o[kPvSubMax] = {0, 1, 2};
        std::sort(o, o + pvN_, [g](uint8_t a, uint8_t b) { return g[a] > g[b]; });
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
    [[nodiscard]] float pvCurrent(float v) const {
        float i = pvG_[pvN_ - 1] * pv_.current(v);
        for (uint8_t m = 0; m + 1 < pvN_; ++m) i = std::max(i, pvG_[m] * pv_.current(v * pvInvW_[m]));
        return i;
    }

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
    void setVin(float v)  { vIn_ = v; }
    void setVout(float v) { vOut_ = v; }
    void setDiodeVf(float v) { diodeVf_ = v; } // body-diode drop (LS and HS) in the dead/off phases
    // Buck DC bias: L = L0 * (1 - k*|Ipk|), Ipk the previous cycle's peak (k in 1/A, 0 = off). The ring keeps L0.
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

    void stopChannel(uint8_t ch) {
        if (ch == 0) pwm_.pwmCtrl = 0;
        else if (ch == 1) pwm_.pwmRect = 0;
    }

    // Advance the model by dt_s seconds. Without prior pwmFreq, falls back.
    void stepSeconds(float dt_s, uint32_t pwmFreqFallback);

private:
    bool boost_ = false;

    PvModel pv_;
    static constexpr uint8_t kPvSubMax = 3;
    uint8_t pvN_ = 1;
    float pvG_[kPvSubMax] = {1.f, 1.f, 1.f};    // substring factors, strongest first
    float pvInvW_[kPvSubMax] = {1.f, 1.f, 1.f}; // 1 / Voc share of the m+1 strongest
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

    // State
    float vIn_   = 0.0f;
    float vOut_  = 0.0f;
    float vOutMax_ = 0.0f;
    float iLEnd_ = 0.0f, iLsOff_ = 0.0f;
    PwmState pwm_{};
    float iInAvg_  = 0.0f;
    float iOutAvg_ = 0.0f;
    bool  dcm_     = false;
    bool  errored_ = false;

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
