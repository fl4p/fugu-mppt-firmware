// Cell-level LFP pack and a BMS publisher, shared by the host tests (charger-term-test.cpp, mppt-sim.cpp).
#pragma once

#include <algorithm>
#include <cmath>

// N series cells. Cell k: ocv[k] + vp + i r0, one RC branch (rp, tauP) carrying the same current in every cell, so one
// vp. ocv is double: a 2 ms sim tick at a few A moves it by less than a float ulp. OCV slope dV/dAh: dvdah on the
// plateau plus knee, ramped in over kneeV..kneeV + kneeW (kneeW 0: a step at kneeV; the default kneeV 0 applies the
// knee everywhere). Cell 0 runs high: set() puts it imb above the mean (the others imb / (N - 1) below) and it gains
// kImb V/Ah more while charging. i > 0 charges [A].
struct LfpPack {
    static constexpr int N = 8;
    double ocv[N]{};
    float dvdah = 0.0032f, knee = 0.f, kneeV = 0.f, kneeW = 0.f;
    float r0 = 0.004f, rp = 0.f, tauP = 30.f, vp = 0.f, kImb = 0.f;

    LfpPack() { set(3.38f, 0.f); }

    void set(float mean, float imb) {
        for (int k = 0; k < N; ++k) ocv[k] = mean + (k ? -imb / (N - 1) : imb);
    }
    void setMean(float mean) {
        const double d = mean - this->mean();
        for (double &v: ocv) v += d;
    }
    [[nodiscard]] float sum() const {
        double s = 0;
        for (double v: ocv) s += v;
        return (float) s;
    }
    [[nodiscard]] float mean() const { return sum() / N; }
    [[nodiscard]] float vcellHigh(float i) const { return (float) *std::max_element(ocv, ocv + N) + vp + i * r0; }
    // pack voltage at 0 A with the present polarization; terminal = emf() + i * N * r0
    [[nodiscard]] float emf() const { return sum() + N * vp; }
    [[nodiscard]] float slope(float v) const {
        const float f = kneeW > 0 ? std::clamp((v - kneeV) / kneeW, 0.f, 1.f) : v >= kneeV ? 1.f : 0.f;
        return dvdah + knee * f;
    }

    void step(float i, float dt) {
        const double ah = (double) i * dt / 3600.0;
        for (int k = 0; k < N; ++k) ocv[k] += ah * (slope((float) ocv[k]) + (k == 0 && i > 0 ? kImb : 0.f));
        if (rp > 0) vp -= (i * rp - vp) * expm1f(-dt / tauP);
    }
};

// Top of the 8s 280 Ah pack fit to fry/flat 2026-11-23 13:44 (both rebooted at float, highest cell 3.358 V at rest,
// then 40 A: crossed 3.50 V ~75 s later, peaked 3.528 V in the hold): 1 mOhm + 2 mOhm / 30 s, OCV +0.05 V/Ah at the
// top, 40 mV imbalance.
inline void topOfCharge(LfpPack &p) {
    p.r0 = 0.001f, p.rp = 0.002f, p.tauP = 30.f, p.knee = 0.05f;
    p.set(3.318f, 0.04f);
}

// BMS publisher: an ibat frame every periodI, a highest-cell frame every periodC [s], the first at phase + period.
// Values are rounded to qI / qV (the BMS text frames; 0 = exact); lagI: an ibat frame carries the sample taken at the
// frame before. cells / ibatOn false: those frames stop (a gap). poll() once per step, then deliver() to every charger
// on the BMS: the same BatteryState calls as the MQTT callbacks in BatteryCharger::beginMqtt().
struct BmsFeed {
    float periodI = 4.f, periodC = 12.f, phaseI = 0.f, phaseC = 0.f, qI = 0.f, qV = 0.f;
    bool lagI = false, cells = true, ibatOn = true;
    bool dueI = false, dueC = false;
    float ibat = 0.f, vcell = 0.f;

    void poll(double t, float iNow, float vNow) {
        if (nextI < 0) nextI = phaseI + periodI, nextC = phaseC + periodC;
        dueI = dueC = false;
        if (t >= nextI) {
            nextI += periodI;
            dueI = ibatOn;
            ibat = q(lagI ? iPrev : iNow, qI);
            iPrev = iNow;
        }
        if (t >= nextC) {
            nextC += periodC;
            dueC = cells;
            vcell = q(vNow, qV);
        }
    }

    template<class Charger>
    void deliver(Charger &c) const {
        if (dueI) c.batSt.updateBatCurrent(ibat);
        if (dueC) c.batSt.setVcellHigh(vcell);
    }

private:
    double nextI = -1, nextC = -1;
    float iPrev = 0.f;
    static float q(float v, float step) { return step > 0 ? std::round(v / step) * step : v; }
};
