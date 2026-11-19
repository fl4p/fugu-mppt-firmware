#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include "esp_attr.h"
#include "math/statmath.h"
#include "util.h"

/**
 * Partial global-peak scan for a string of n bypass-diode substrings, one local power peak per number m of conducting
 * substrings, at or above m substring MPP voltages. Measures the operating point, probes the candidates for a moment
 * each, measures the operating point again and moves to the best probe if it beats the larger of the two by `margin`.
 * Runs periodically and on a sustained Vin or power change since the last scan or sweep.
 */
struct PeakScan {
    static constexpr float kProbeLow = 0.97f; // probe on the flat low-voltage side of a peak
    static constexpr uint16_t kMeasMs = 100, kTravelMs = 1500, kSteadyMs = 5000;

    uint8_t n = 0;           // bypass-diode substrings learned from the sweeps (tracker.conf::pv_substrings), 0 = off
    std::atomic<bool> pending{false}; // RT -> core 0: a sweep's maxima wait for learn()
    std::atomic<uint8_t> gen{0};      // RT: bumped before vc/pc/nc/voc/pk are rewritten (sweep start, scan start)
    uint8_t saved = 0;       // core 0: n as read from or written to tracker.conf
    uint16_t interval = 300; // tracker.conf::scan_interval [s], 0 = triggered only
    bool trigger = true;     // tracker.conf::scan_trigger
    bool force = false;      // RT: scan once referenced, also without a trigger
    bool held = false;       // RT: maxima for learn() held until release()
    float margin = 0.03f;

    enum Phase : uint8_t { Idle, Ref, Probe, Back, Go };
    Phase phase = Idle;
    uint8_t k = 0, nc = 0;
    bool settled = false;
    uint8_t ended = 0; // back at the start point: 1, 3 light changed; 2 moved to another peak; 4 timed out (return to d0);
                       // 6 return pass done
    uint16_t d0 = 0;
    float vc[6]{}, pc[6]{};
    // return pass: probe Vin and the down-pass power there (ascending), probe index of the best, largest probe/down - 1
    bool ret = false;
    uint8_t nr = 0, rb = 0;
    float rv[7]{}, rp[7]{}, rDev = 0, rBest = 0, rLand = 0;
    float voc = NAN, iEnv = 0, kVmp = 0.82f; // Voc at the sweep start, max PV-side current [W/V], Vmpp / Voc
    float v0 = 0, p0 = 0, pRef = 0, vRef = 0, pS = 0;
    float pk = 0, vPk = 0, pLo = 0; // sweep: running maximum (after the sweep: the MPP), its Vin, minimum after it
    bool fall = false;
    uint32_t t0 = 0, tPhase = 0, tLast = 0, tOff = 0, tS = 0;
    MeanAccumulator acc{};

    [[nodiscard]] bool active() const { return phase != Idle; }
    [[nodiscard]] bool unsaved() const { return n != saved; }

    // RT, before vc/pc/nc/voc/pk change: a learn() in progress drops the sweep
    __attribute__((noinline)) IRAM_ATTR void claim() {
        pending.store(false, std::memory_order_relaxed);
        held = false;
        gen.store(gen.load(std::memory_order_relaxed) + 1, std::memory_order_release);
        std::atomic_thread_fence(std::memory_order_release);
    }

    // first sweep tick (converter still off, Vin ~ Voc)
    void onSweep(float vin, uint32_t now) {
        claim();
        voc = vin, iEnv = 0, pRef = -1, phase = Idle, tLast = tOff = now, nc = 0, pLo = 0, fall = true, ret = force = false;
    }

    // sweep tick, Vin falling: local power maxima below 0.93 Voc with an 8 % dip on both sides go to vc/pc (idle
    // during a sweep), nc = 7 on overflow
    __attribute__((always_inline)) void sweepSample(float v, float p) {
        if (!(v > 1) || std::isnan(p)) return;
        if (p / v > iEnv) iEnv = p / v;
        if (fall) {
            if (p < pLo) pLo = p;
            else if (p > 1.08f * pLo) fall = false, pk = p, vPk = v;
        } else if (p > pk) {
            pk = p, vPk = v;
        } else if (p < 0.92f * pk) {
            fall = true, pLo = p;
            if (vPk < 0.93f * voc && nc < 7) {
                if (nc < 6) vc[nc] = vPk, pc[nc] = pk;
                ++nc;
            }
        }
    }

    // sweep MPP: an unshaded string's Vmpp / Voc lies in 0.74..0.86 (field 0.79..0.85); a shaded global maximum
    // falls outside (m < n substrings: lower, weaker substrings conducting: higher) and keeps the last value.
    // Hands the maxima to learn() unless the sweep was dim (MPP below pMin) or noisy (more than 6 maxima); hold: only
    // on release()
    __attribute__((always_inline)) void onSweepMpp(float v, float p, float pMin, bool hold = false) {
        const float r = v / voc;
        if (r > 0.74f && r < 0.86f) kVmp = r;
        if (nc <= 6 && p >= pMin && voc > 0) {
            pk = p;
            if (hold) held = true;
            else pending.store(true, std::memory_order_release);
        }
    }

    void release() {
        if (held) held = false, pending.store(true, std::memory_order_release);
    }

    // core 0, after onSweepMpp() (vc/pc stay untouched for 5 s): the first maximum of at least 60 % of the MPP is the
    // full string (a start transient is weaker); n from the maxima below it of at least 30 % of the MPP, the largest
    // n in 2..6 that puts each at m - 0.25 .. m + 0.5 shares of 0.82 Voc / n (m < n; shaded maxima sit high).
    // Nothing if the MPP is not a recorded maximum. Works on a copy; drops it if RT rewrote the arrays meanwhile
    // (mid: test hook, runs after the copy). True if n grew.
    template<class Mid = decltype([](PeakScan &) {})>
    bool learn(Mid mid = {}) {
        const uint8_t g0 = gen.load(std::memory_order_acquire);
        if (!pending.exchange(false, std::memory_order_acquire)) return false;
        const uint8_t nc = std::min<uint8_t>(this->nc, 6);
        float vc[6], pc[6];
        std::copy_n(this->vc, 6, vc), std::copy_n(this->pc, 6, pc);
        const float voc = this->voc, pk = this->pk;
        mid(*this);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (gen.load(std::memory_order_relaxed) != g0) return false;
        int g = -1;
        bool mpp = false;
        for (int i = 0; i < nc; ++i) {
            if (g < 0 && pc[i] >= 0.6f * pk) g = i;
            mpp = mpp || pc[i] >= 0.98f * pk;
        }
        if (!mpp) return false;
        int c = 6, q = 0;
        for (; c > 1; --c) {
            bool ok = true;
            q = 0;
            for (int i = g + 1; i < nc; ++i) {
                if (pc[i] < 0.3f * pk) continue;
                const float y = vc[i] / (0.82f * voc) * (float) c;
                const int m = (int) (y + 0.25f);
                ok = ok && m >= 1 && m < c && y <= (float) m + 0.5f, ++q;
            }
            if (!q || ok) break;
        }
        if (!q || c < 2 || c <= n) return false;
        n = (uint8_t) c;
        ESP_LOGI("mppt", "scan: %u bypass-diode substrings learned from the sweep", (unsigned) n);
        return true;
    }

    void abort() { phase = Idle, ret = false; }

    // mode change: drop a running scan and the reference, re-reference 5 s into tracking, next scan after the period
    void reset(uint32_t now) { phase = Idle, ended = 0, pRef = -1, tLast = tOff = now, ret = false; }

    // RT, at the bottom of a sweep: the way back up probes the recorded maxima and the MPP (vm, pm), lowest Vin first,
    // each against its down-pass power, then goes to the best. vc/pc stay for learn(). False: fewer than 2 above vMin,
    // or a noisy sweep. d: fallback duty on a timeout.
    bool startReturn(float vm, float pm, float vMin, uint16_t d, uint32_t now) {
        if (nc > 6) return false;
        nr = 0;
        bool mpp = false;
        const auto add = [&](float v, float p) { if (v >= vMin) rv[nr] = v, rp[nr] = p, ++nr; };
        for (int i = nc - 1; i >= 0; --i) {
            if (!mpp && vm < vc[i] * 0.98f) add(vm, pm), mpp = true;
            if (std::fabs(vc[i] - vm) <= 0.02f * vm) mpp = true;
            add(vc[i], pc[i]);
        }
        if (!mpp) add(vm, pm);
        if (nr < 2) return false;
        phase = Probe, ret = true, k = rb = 0, d0 = d, t0 = tPhase = now, settled = false, rDev = rBest = 0;
        acc.clear();
        return true;
    }

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
            const float vs0 = kVmp * voc / (float) n;
            // trigger: off the reference Vin or power for 15 s, the power steady (+-10 %) for the last 5 s
            if (!trigger || (std::fabs(vin - vRef) < 0.5f * vs0 && pw > 0.7f * pRef && pw < 1.4f * pRef)) tOff = now;
            if (std::fabs(pw - pS) > 0.1f * pS) pS = pw, tS = now;
            const bool due = force || (interval && now - tLast > interval * 1000u) ||
                             (now - tOff > 15000 && now - tS > kSteadyMs && now - tLast > 60000);
            if (!due || pw < pMin) return NAN;
            tLast = now, force = false;
            // m0 of n substrings conduct at our peak. With weaker substrings conducting a peak sits above m unshaded
            // shares, so Vin / m0 can be high; the share from Voc misses heating and the Vmpp/Voc spread: take the mean
            const int m0 = std::clamp((int) std::lround(vin / vs0), 1, (int) n);
            const float sh = 0.5f * (vin / (float) m0 + vs0);
            claim();
            nc = 0;
            for (int m = 1; m <= n; ++m) {
                const float v = kProbeLow * (float) m * sh;
                // unreachable, at our own peak, or a lower peak that cannot beat us even at the best current seen
                if (m == m0 || v < vMin || v > 0.95f * voc || (m < m0 && iEnv > 0 && (float) m * sh * iEnv < pw * (1 + margin)))
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
        const float vt = phase == Probe || phase == Go ? (ret ? rv[k] : vc[k]) : v0;
        if (!settled && (std::fabs(vinFast - vt) < 0.01f * vt || now - tPhase > kTravelMs)) settled = true, tPhase = now;
        if (settled) {
            const float ts = std::fmin(settleMs(), 1000.f), dt = (float) (now - tPhase); // fmin: NAN -> 1 s
            if (dt > ts) acc.add(pw);
            if (dt > ts + kMeasMs) step(now, vin);
        }
        if (phase != Idle && now - t0 > 30000) phase = Idle, pRef = -1, ended = 4, ret = false;
        return phase == Idle ? NAN : (float) d * std::sqrt(vinFast / vt);
    }

    __attribute__((always_inline)) void step(uint32_t now, float vin) {
        const float p = acc.pop();
        settled = false, tPhase = now;
        if (phase == Ref) {
            p0 = p, phase = Probe;
        } else if (phase == Probe && ret) {
            const float r = p / rp[k] - 1;
            ESP_LOGI("mppt", "sweep return: %.1f V %.1f W, down %.1f W (%+.0f %%)", rv[k], p, rp[k], 100 * r);
            if (std::fabs(r) > std::fabs(rDev)) rDev = r;
            if (p > rBest) rBest = p, rb = k;
            if (++k == nr) k = rb, phase = Go;
        } else if (phase == Probe) {
            pc[k] = p;
            if (++k == nc) phase = Back;
        } else if (phase == Back) {
            // the start point measured before and after: the larger one, so a light change cannot fake a gain
            int best = 0;
            float g = 0;
            for (int i = 0; i < nc; ++i) {
                const float r = pc[i] / std::max(p0, p);
                if (r > g) g = r, best = i;
            }
            const bool steady = std::fabs(p - p0) < 0.15f * p0, move = steady && g > 1 + margin;
            ESP_LOGI("mppt", "scan: P %.1f->%.1f W at %.1fV, %u probes, best x%.3f at %.1fV%s", p0, p, v0, (unsigned) nc, g,
                     vc[best], !steady ? " (light changed)" : move ? ", moving" : "");
            if (move) {
                k = (uint8_t) best, phase = Go, pRef = pc[best], vRef = vc[best];
            } else {
                phase = Idle, pRef = p, vRef = vin, ended = steady ? 1 : 3;
                // a probe within 2 margins (clouds change the ratio): again in 60 s
                if (g > 1 - 2 * margin && interval > 60) tLast = now - (interval - 60) * 1000u;
            }
        } else {
            phase = Idle, vRef = vin, ended = ret ? 6 : 2, rLand = p, ret = false;
        }
    }
};
