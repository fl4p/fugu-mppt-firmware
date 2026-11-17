#pragma once

#include <algorithm>
#include <cmath>
#include "math/statmath.h"
#include "util.h"

/**
 * Partial global-peak scan for a string of n bypass-diode substrings: its local power peaks sit near m/n of the
 * unshaded MPP voltage (m conducting substrings). Measures the operating point, probes those voltages for a moment
 * each, returns, and moves to the best probe if it beats the operating point (drift-corrected) by `margin`.
 * Runs periodically and on a sustained Vin or power change since the last scan or sweep.
 */
struct PeakScan {
    static constexpr float kVmpOfVoc = 0.82f; // unshaded Vmpp / Voc (field fry/flat 0.79..0.85)
    static constexpr float kProbeLow = 0.97f; // probe on the flat low-voltage side of a peak
    static constexpr uint16_t kMeasMs = 100, kTravelMs = 1500, kSteadyMs = 5000;

    uint8_t n = 0;          // tracker.conf::pv_substrings, 0 = off
    uint16_t interval = 300; // tracker.conf::scan_interval [s], 0 = triggered only
    bool trigger = true;     // tracker.conf::scan_trigger
    float margin = 0.03f;

    enum Phase : uint8_t { Idle, Ref, Probe, Back, Go };
    Phase phase = Idle;
    uint8_t k = 0, nc = 0;
    bool settled = false;
    uint8_t ended = 0; // 1 back at the start point, 2 moved to another peak
    uint16_t d0 = 0;
    float vc[6]{}, pc[6]{};
    uint32_t tc[6]{};
    float voc = NAN, iEnv = 0, vs = 0;  // Voc at the sweep start, max PV-side current [W/V], substring Vmpp share
    float v0 = 0, p0 = 0, pRef = 0, vRef = 0, pS = 0;
    uint32_t t0 = 0, tPhase = 0, tLast = 0, tOff = 0, tS = 0;
    MeanAccumulator acc{};

    [[nodiscard]] bool active() const { return phase != Idle; }

    // first sweep tick (converter still off, Vin ~ Voc)
    void onSweep(float vin, uint32_t now) {
        voc = vin, iEnv = 0, pRef = -1, phase = Idle, tLast = tOff = now;
    }

    void sweepSample(float i) { if (i > iEnv) iEnv = i; }

    void abort() { phase = Idle; }

    // NAN while idle; else the duty count to steer to. vinFast: med3, vin/pw: the tracker's (ewm) signals,
    // settleMs(): power filter settling time [ms]
    template<class SettleMs>
    __attribute__((always_inline)) float update(uint32_t now, float vinFast, float vin, float pw, uint16_t d,
                                                float vMin, float pMin, SettleMs settleMs) {
        if (phase == Idle) {
            if (!n || !(voc > 0)) return NAN;
            if (vin > 0 && pw / vin > iEnv) iEnv = pw / vin;
            if (pRef <= 0) { // tracking after a sweep: reference 5 s later
                if (pRef < 0) pRef = 0, tPhase = now;
                else if (now - tPhase > 5000) pRef = pw, vRef = vin, tOff = now;
                return NAN;
            }
            const float vs0 = kVmpOfVoc * voc / (float) n;
            // trigger: off the reference Vin or power for 15 s, the power steady (+-10 %) for the last 5 s
            if (!trigger || (std::fabs(vin - vRef) < 0.5f * vs0 && pw > 0.7f * pRef && pw < 1.4f * pRef)) tOff = now;
            if (std::fabs(pw - pS) > 0.1f * pS) pS = pw, tS = now;
            const bool due = (interval && now - tLast > interval * 1000u) ||
                             (now - tOff > 15000 && now - tS > kSteadyMs && now - tLast > 60000);
            if (!due || pw < pMin) return NAN;
            tLast = now;
            // anchor the substring share on the peak we are on (m0 of n conducting)
            const int m0 = std::clamp((int) std::lround(vin / vs0), 1, (int) n);
            vs = vin / (float) m0, nc = 0;
            for (int m = 1; m <= n; ++m) {
                const float v = kProbeLow * (float) m * vs;
                // unreachable, at our own peak, or a lower peak that cannot beat us even at the best current seen
                if (m == m0 || v < vMin || v > 0.95f * voc || (m < m0 && iEnv > 0 && (float) m * vs * iEnv < pw * (1 + margin)))
                    continue;
                vc[nc++] = v;
            }
            if (!nc) {
                pRef = pw, vRef = vin;
                return NAN;
            }
            phase = Ref, v0 = vin, d0 = d, t0 = tPhase = now, settled = false, k = 0;
            acc.clear();
        }
        const float vt = phase == Probe || phase == Go ? vc[k] : v0;
        if (!settled && (std::fabs(vinFast - vt) < 0.01f * vt || now - tPhase > kTravelMs)) settled = true, tPhase = now;
        if (settled) {
            const float ts = settleMs(), dt = (float) (now - tPhase);
            if (dt > ts) acc.add(pw);
            if (dt > ts + kMeasMs) step(now, vin);
        }
        return (float) d * std::sqrt(vinFast / vt);
    }

    __attribute__((always_inline)) void step(uint32_t now, float vin) {
        const float p = acc.pop();
        settled = false, tPhase = now;
        if (phase == Ref) {
            p0 = p, phase = Probe;
        } else if (phase == Probe) {
            pc[k] = p, tc[k] = now - t0;
            if (++k == nc) phase = Back;
        } else if (phase == Back) {
            // power at the operating point drifted p0 -> p over the scan: compare each probe at its time
            const float drift = (p - p0) / (float) (now - t0);
            int best = 0;
            float g = 0;
            for (int i = 0; i < nc; ++i) {
                const float r = pc[i] / (p0 + drift * (float) tc[i]);
                if (r > g) g = r, best = i;
            }
            const bool steady = std::fabs(p - p0) < 0.15f * p0, move = steady && g > 1 + margin;
            ESP_LOGI("mppt", "scan: P %.1f->%.1f W at %.1fV, %u probes, best x%.3f at %.1fV%s", p0, p, v0, (unsigned) nc, g,
                     vc[best], !steady ? " (light changed)" : move ? ", moving" : "");
            if (move) {
                k = (uint8_t) best, phase = Go, pRef = pc[best], vRef = vc[best];
            } else {
                phase = Idle, pRef = p, vRef = vin, ended = 1;
            }
        } else {
            phase = Idle, vRef = vin, ended = 2;
        }
    }
};
