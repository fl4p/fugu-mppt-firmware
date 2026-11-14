#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <random>

#include "rect_cal.h"

// DCM plant for RectCal::feed() held at its calibration point: current shape around the zero crossing,
// a baseline slope over LS, a soft PV source (a current drop raises Vin), bus resistance, sensor noise
// and external voltage disturbances. Sampled at 434 sps.
struct RcPlant {
    float vi0 = 72.8f, vo0 = 26.f, i0 = 1.6f; // undisturbed operating point (V, V, A)
    uint16_t hs = 1111;
    float zcErr = 0;    // true zero crossing - ideal (ct)
    float kE = 6.5e-9f; // body-diode side: -kE * (zc - LS)^2 (A/ct^2)
    float kL = 6.5e-7f; // reverse-current side: -kL * (LS - zc)^2 (A/ct^2)
    float slope = 0;    // baseline slope over LS from the reference count (A/ct)
    float rpv = 0;      // panel |dV/dI| at the operating point (ohm), 0: stiff
    float rbus = 0;     // bus resistance seen by the output current (ohm)
    float vExp = 1;     // plant DCM voltage sensitivity relative to the RectCal model
    float iN = 0.01f, viN = 0.05f, voN = 0.01f; // per-sample noise, 1 sigma (A, V, V)
    std::function<void(double t, int ls, float &vi, float &vo)> ext; // panel/bus disturbance, t: s from start
};

inline void runPlant(RectCal &cal, const RcPlant &p, float tickRate, uint32_t seed = 1) {
    std::mt19937 rng{seed};
    std::normal_distribution<float> nd{0.f, 1.f};
    cal.start(0, p.hs, 16, 4103 - p.hs - 1);
    int ls = cal.lsHold();
    bool init = true;
    const float g0 = p.vi0 * (p.vi0 - p.vo0) / p.vo0;
    for (uint32_t us = 0; cal.running() && us < 60000000; us += 2304) {
        if (ls != 16) init = false;
        float vi = p.vi0, vo = p.vo0;
        if (p.ext) p.ext(us * 1e-6, init ? -1 : ls, vi, vo);
        const float viX = vi, voX = vo;
        float dI = 0;
        for (int it = 0; it < 30; ++it) { // knee <-> Vin (soft panel) <-> Vout (bus), fixed point
            const float zc = (vi / vo - 1.f) * p.hs + p.zcErr, ref = 0.6f * zc, x = init ? ref : (float) ls;
            dI = (x < zc ? -p.kE * (zc - x) * (zc - x) : -p.kL * (x - zc) * (x - zc)) + p.slope * (x - ref);
            const float iIn = p.i0 * vo / vi;
            vi = viX - p.rpv * dI * vo / vi / (1.f + p.rpv * iIn / (vi - vo));
            vo = voX + p.rbus * dI;
        }
        const float i = p.i0 * std::pow(vi * (vi - vo) / vo / g0, p.vExp) + dI;
        ls = cal.feed(us, p.hs, vi + p.viN * nd(rng), vo + p.voN * nd(rng), i + p.iN * nd(rng), true);
    }
    cal.fit(tickRate);
}

// The three field runs on an 80 uH buck at low light, 2026-11-14 (shared bus, 1 kW load, a second
// charger 90-590 W): measured knee 1.3e-5 A/ct^2, baseline -15 mA over 1000 ct.
inline RcPlant fieldPlant(float vi, float vo, uint16_t hs, float rpv, uint32_t seed) {
    RcPlant p{};
    p.vi0 = vi; p.vo0 = vo; p.hs = hs; p.i0 = 1.62f;
    p.kL = 1.3e-5f; p.slope = -1.5e-5f; p.zcErr = 16; p.rpv = rpv; p.rbus = 0.04f; p.iN = 0.02f;
    // bus: other charger and load, smooth swings plus a 0.15 % step every ~4 s
    auto st = std::make_shared<std::pair<double, float>>(0.0, 0.f);
    auto rng = std::make_shared<std::mt19937>(seed);
    p.ext = [st, rng, vo](double t, int, float &, float &v) {
        std::uniform_real_distribution<float> u{0.f, 1.f};
        while (t >= st->first) { st->first += 2.0 + 4.0 * u(*rng); st->second = (u(*rng) - 0.5f) * 0.003f; }
        v = vo * (1.f + 0.004f * std::sin(6.2832 * t / 23.) + 0.002f * std::sin(6.2832 * t / 7.3) + st->second);
    };
    return p;
}
