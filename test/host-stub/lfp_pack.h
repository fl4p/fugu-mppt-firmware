// Cell-level LFP pack and a BMS publisher, shared by the host tests (charger-term-test.cpp, mppt-sim.cpp).
#pragma once

#include <algorithm>
#include <cmath>

// N series cells, i > 0 charges [A]. Cell k: ocv[k] + vp[k] + i r0. ocv is double: a 2 ms sim tick at a few A moves it
// by less than a float ulp. ah[k] is the cell's charge [Ah] on the axis of the polarization and the knee; it only
// differs between cells by its start (place()). OCV slope dV/dAh: dvdah plus knee, ramped in over kneeAh..kneeAh +
// kneeW (kneeW 0: a step); cell 0 gains kImb V/Ah more while charging. Polarization: y follows i with tauP (one value,
// the cells carry the same current); vp[k] = rp exp(ah[k] / qR) y (y / iRef)^(n - 1) for y > 0 (qR 0: no charge
// dependence), rpd y for y <= 0. n 1, qR 0, rpd = rp: one linear RC branch (rp, tauP).
struct LfpPack {
    static constexpr int N = 8;
    double ocv[N]{}, ah[N]{};
    float vp[N]{}, y = 0.f;
    float dvdah = 0.0032f, knee = 0.f, kneeAh = 0.f, kneeW = 0.f;
    float r0 = 0.004f, rp = 0.f, rpd = 0.f, tauP = 30.f, qR = 0.f, iRef = 10.f, n = 1.f, kImb = 0.f;

    LfpPack() { set(3.38f, 0.f); }

    void set(float mean, float imb) {
        for (int k = 0; k < N; ++k) ocv[k] = mean + (k ? -imb / (N - 1) : imb);
    }
    void setMean(float mean) {
        const double d = mean - this->mean();
        for (double &v: ocv) v += d;
    }
    // cell 0 at ah = a, the others dA behind, all on the OCV curve ocv0 at ah 0 (knee included), relaxed
    void place(float ocv0, float a, float dA) {
        for (int k = 0; k < N; ++k) {
            const double x = k ? a - dA : a, u = x - kneeAh;
            const double kn = u <= 0 ? 0 : kneeW <= 0 || u >= kneeW ? u - kneeW / 2 : u * u / (2 * kneeW);
            ah[k] = x, ocv[k] = ocv0 + dvdah * x + knee * kn, vp[k] = 0;
        }
        y = 0;
    }
    [[nodiscard]] float sum() const {
        double s = 0;
        for (double v: ocv) s += v;
        return (float) s;
    }
    [[nodiscard]] float mean() const { return sum() / N; }
    [[nodiscard]] float vcellHigh(float i) const {
        double m = -1e9;
        for (int k = 0; k < N; ++k) m = std::max(m, ocv[k] + vp[k]);
        return (float) m + i * r0;
    }
    // pack voltage at 0 A with the present polarization; terminal = emf() + i * N * r0
    [[nodiscard]] float emf() const {
        float s = 0;
        for (float v: vp) s += v;
        return sum() + s;
    }
    [[nodiscard]] float slope(double a) const {
        const float u = (float) (a - kneeAh), f = kneeW > 0 ? std::clamp(u / kneeW, 0.f, 1.f) : u >= 0 ? 1.f : 0.f;
        return dvdah + knee * f;
    }

    void step(float i, float dt) {
        const double dah = (double) i * dt / 3600.0;
        y -= (i - y) * expm1f(-dt / tauP);
        const float g = y > 0 ? rp * y * (n != 1 ? powf(y / iRef, n - 1) : 1.f) : rpd * y;
        for (int k = 0; k < N; ++k) {
            ocv[k] += dah * (slope(ah[k]) + (k == 0 && i > 0 ? kImb : 0.f));
            ah[k] += dah;
            vp[k] = y > 0 && qR > 0 ? g * expf((float) ah[k] / qR) : g;
        }
    }
};

// Top of the 8s 245 Ah pack, fit to the BMS highest cell and pack voltage of 2026-11-23 10:00..16:20 (fry + flat,
// ~6-33 A, holds at 3.500 V, rests at ~0 and -2..-10 A; 10 s data, rms 12 mV cell / 8 mV pack per cell). ah 0 = the
// 13:44 rest (both rebooted at float, then 40 A; the old fit's start). The field numbers it reproduces:
// - rest-to-rest the high cell stayed 3.344..3.357 V across +18 Ah: plateau 0.3 mV/Ah, OCV 3.348 V at ah 0; the
//   others rest 5 mV lower, as 17.4 Ah behind on the same curve (they polarize ~0.45x the high cell in the holds);
// - holding 3.500 V took 20 -> 16.6, 13.2 -> 11.8, 10.3 -> 9.4, 9.0 -> 7.8 A (model 16.8, 11.3, 9.1, 7.5 A at the
//   ends): the hold current follows the charge in, not the time (10-20 min rests do not reset it), hence
//   rp exp(ah / qR) with qR 17.7 Ah and no slow RC branch; the taper tau grows as the current falls (1.2 h at 20 A,
//   2.5 h at 9 A; model 1.9 h at 15 A, 2.8 h at 9 A); steady hold current 25 A at ah -30, 7.4 A at 0, 4.7 A at +10;
// - after the 13:33 hold (7.1 -> -2.2 A): -55 mV in 60 s, then 93 mV with tau 228 s (model -51 mV, 118 mV / 265 s):
//   r0 2.1 mOhm, y over tauP 410 s, n 1.19 (the voltage relaxes with tauP / n); discharge side rpd 3.9 mOhm;
// - 0.5 A net: the cell settles ~10 mV above its OCV, no upward creep.
// The balancer (from 3.40 V and 20 mV spread, current unknown) is inside the fit: the pack current is taken as the
// high cell's. Valid from ah ~-50 (the fit's start, ~80 % SoC); lower, the polarization is underestimated. The upper
// knee is an assumption: the pack never rested above ~3.36 V and the last field hold ends at ah ~+1.3. It is put at
// ah +10 (where the 3.500 V hold current is down to 4.7 A, ~0.02 C), 0.02 V/Ah ramped in over 5 Ah.
inline void topOfCharge(LfpPack &p, float a = 0.f) {
    p.r0 = 0.0021f, p.rp = 0.0194f, p.rpd = 0.0039f, p.tauP = 410.f, p.qR = 17.7f, p.iRef = 10.f, p.n = 1.19f;
    p.dvdah = 0.0003f, p.knee = 0.02f, p.kneeAh = 10.f, p.kneeW = 5.f;
    p.place(3.348f, a, 17.4f);
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
