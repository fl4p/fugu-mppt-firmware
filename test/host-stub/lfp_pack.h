// Cell-level LFP pack and a BMS publisher, shared by the host tests (charger-term-test.cpp, mppt-sim.cpp).
#pragma once

#include <algorithm>
#include <cmath>

// N series cells. Cell k: ocv[k] + vp + i r0, one RC branch (rp, tauP) carrying the same current in every cell, so one
// vp. ocv is double: a 2 ms sim tick at a few A moves it by less than a float ulp. OCV slope dV/dAh: dvdah on the
// plateau plus knee, ramped in over kneeV..kneeV + kneeW (kneeW 0: a step at kneeV; the default kneeV 0 applies the
// knee everywhere). Cell 0 runs high: set() puts it imb above the mean (the others imb / (N - 1) below) and it gains
// kImb V/Ah more while charging. i > 0 charges [A].
// kd > 0: two tanks per cell (kinetic battery model). The current goes into a surface tank, a fraction c of the cell,
// whose OCV is ocv[k]; kd A flow to the bulk (OCV ob[k]) per Ah of level difference (level: the cell's charge if it all
// sat at that OCV). A hold at a fixed surface OCV takes a current decaying with ~3600 (1 - c) / kd s; without current
// the cell relaxes to ob[k] (~3600 c / kd s).
struct LfpPack {
    static constexpr int N = 8;
    double ocv[N]{}, ob[N]{};
    float dvdah = 0.0032f, knee = 0.f, kneeV = 0.f, kneeW = 0.f;
    float r0 = 0.004f, rp = 0.f, tauP = 30.f, vp = 0.f, kImb = 0.f;
    float c = 1.f, kd = 0.f;

    LfpPack() { set(3.38f, 0.f); }

    void set(float mean, float imb) {
        for (int k = 0; k < N; ++k) ob[k] = ocv[k] = mean + (k ? -imb / (N - 1) : imb);
    }
    void setMean(float mean) {
        const double d = mean - this->mean();
        for (int k = 0; k < N; ++k) ocv[k] += d, ob[k] = ocv[k];
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
    // level [Ah] at OCV v: integral of dv / slope(v) from kneeV
    [[nodiscard]] double level(double v) const {
        const double x = v - kneeV, a = dvdah, b = knee, w = kneeW;
        if (x <= 0 || b <= 0) return x / a;
        if (x < w) return w / b * std::log1p(b * x / (w * a));
        return (w > 0 ? w / b * std::log1p(b / a) : 0) + (x - w) / (a + b);
    }

    void step(float i, float dt) {
        const double ah = (double) i * dt / 3600.0;
        for (int k = 0; k < N; ++k) {
            const double s = slope((float) ocv[k]) + (k == 0 && i > 0 ? kImb : 0.f);
            if (kd <= 0) {
                ocv[k] += ah * s;
                continue;
            }
            const double f = kd * (level(ocv[k]) - level(ob[k])) * dt / 3600.0; // Ah to the bulk
            ocv[k] += (ah - f) * s / c;
            ob[k] += f * slope((float) ob[k]) / (1 - c);
        }
        if (rp > 0) vp -= (i * rp - vp) * expm1f(-dt / tauP);
    }
};

// Top of the 8s 280 Ah pack, fit to the highest cell of fry/flat 2026-11-23 09:50-14:08 (BMS ibat replayed, 2 s steps):
// R0 1.5 mOhm + 1.3 mOhm / 65 s, plateau 0.5 mV/Ah, 6 mV/Ah from 3.363 V (surface), surface tank 5.3 %, 0.68 A/Ah to
// the bulk. A 3.50 V hold from 29 A decays with ~110 min (field 92-145 min) and takes ~36 Ah in 130 min (field ~30).
// The high cell rests at 3.352 V, 40 mV above the mean: ~20-25 A into a 3.50 V hold after the approach.
inline void topOfCharge(LfpPack &p) {
    p.r0 = 0.0015f, p.rp = 0.0013f, p.tauP = 65.f;
    p.dvdah = 0.0005f, p.knee = 0.0055f, p.kneeV = 3.363f, p.kneeW = 0.006f, p.c = 0.053f, p.kd = 0.68f;
    p.set(3.312f, 0.04f);
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
