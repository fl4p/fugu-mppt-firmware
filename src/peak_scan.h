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
 * Runs periodically and on a sustained Vin or power change since the last scan or sweep. Log only: before that,
 * two probes at the start Vin -+ nearDv, bracketed by the start power measured before and after, show whether the
 * operating point sits off the local peak; with no candidate they run alone, at most once per interval.
 */
struct PeakScan {
    static constexpr uint16_t kMeasMs = 100, kTravelMs = 1500, kSteadyMs = 5000;
    // probe m at m shares times: below m0 on the flat low-voltage side of a peak; above m0 weaker substrings conduct
    // and lift the peak over m shares. Not const: the host sim varies them
    static inline float probeLow = 0.97f, probeHigh = 1.015f;
    // with Vmpp/Voc learned, Vin / m0 below heatCut x the sweep's share: heated since the sweep, probes above m0
    // take Vin / m0 alone (with the fallback the gap is the Vmpp/Voc spread)
    static inline float heatCut = 0.96f;
    static inline float riseK = 1.15f; // rising light: power over the reference by this factor
    static constexpr float kVmpFallback = 0.84f;

    uint8_t n = 0;           // bypass-diode substrings learned from the sweeps (tracker.conf::pv_substrings), 0 = learn,
                             // 1 = single peak, nothing to scan
    std::atomic<uint8_t> pending{0}; // RT -> core 0: a sweep's maxima wait for learn(); 2 verified, 3 flagged (no maxima)
    uint8_t one = 0;         // core 0: verified single-maximum sweeps in a row
    std::atomic<uint8_t> gen{0};      // RT: bumped before vc/pc/nc/voc/pk are rewritten (sweep start, scan start)
    uint8_t saved = 0;       // core 0: n as read from or written to tracker.conf
    uint8_t nearDv = 20;     // tracker.conf::scan_near_v [0.1 V], 0 = off
    uint16_t interval = 300; // tracker.conf::scan_interval [s], 0 = triggered only
    bool trigger = true;     // tracker.conf::scan_trigger
    bool force = false;      // RT: scan once referenced, also without a trigger
    bool held = false;       // RT: maxima for learn() held until release()
    float margin = 0.03f;

    enum Phase : uint8_t { Idle, Ref, Probe, Back, Go, Near };
    Phase phase = Idle;
    uint8_t k = 0, nc = 0;
    bool settled = false, band = false; // band: settled within 1 % of the target Vin, not by the travel timeout
    uint8_t ended = 0; // back at the start point: 1, 3 light changed; 2 moved to another peak; 4 timed out (return to d0);
                       // 6 return pass done
    uint16_t d0 = 0;
    float vc[6]{}, pc[6]{};
    // return pass: probe Vin and the down-pass power there (ascending), probe index of the best, largest probe/down - 1;
    // a scan: its near probes in rv/rp 0 (start point, before), 1, 2; rv[0] NAN: none
    bool ret = false, heat = false, up = false, rt = false; // heat: this scan's upward probes used Vin / m0 (heatCut);
                                                              // up: in rising light; rt: its one close retry is due
    uint8_t nr = 0, rb = 0;
    float rv[7]{}, rp[7]{}, rDev = 0, rBest = 0, rLand = 0;
    float voc = NAN, iEnv = 0, kVmp = kVmpFallback; // Voc at the sweep start, max PV-side current seen [W/V], Vmpp / Voc
    float v0 = 0, p0 = 0, pRef = 0, vRef = 0, pS = 0;
    // sweep: running maximum (after the sweep: the MPP), its Vin, minimum after it. After the sweep pLo is the
    // rising-light reference: the MPP, raised to the power 5 s later, at a scan's move and when a scan in rising light
    // ends steady without a probe within 2 margins
    float pk = 0, vPk = 0, pLo = 0;
    bool fall = false;
    uint32_t t0 = 0, tPhase = 0, tLast = 0, tOff = 0, tS = 0;
    MeanAccumulator acc{};

    [[nodiscard]] bool active() const { return phase != Idle; }
    [[nodiscard]] bool unsaved() const { return n != saved; }

    // RT, before vc/pc/nc/voc/pk change: a learn() in progress drops the sweep
    __attribute__((noinline)) IRAM_ATTR void claim() {
        pending.store(0, std::memory_order_relaxed);
        held = false;
        gen.store(gen.load(std::memory_order_relaxed) + 1, std::memory_order_release);
        std::atomic_thread_fence(std::memory_order_release);
    }

    // first sweep tick (converter still off, Vin ~ Voc)
    void onSweep(float vin, uint32_t now) {
        claim();
        voc = vin, iEnv = 0, pRef = -1, phase = Idle, tLast = tOff = now, nc = 0, pLo = 0, fall = true, ret = force = rt = false;
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
        pLo = p;
        if (nc <= 6 && p >= pMin && voc > 0) {
            pk = p;
            if (hold) held = true;
            else pending.store(1, std::memory_order_release);
        }
    }

    // verified: the sweep passed its verdict; else not evaluable
    void release(bool verified = false) {
        if (held) held = false, pending.store(verified ? 2 : 1, std::memory_order_release);
    }

    // a flagged sweep: its maxima do not teach and it breaks a run of single-maximum sweeps
    void drop() {
        if (held) held = false, pending.store(3, std::memory_order_release);
    }

    // core 0, after onSweepMpp() (vc/pc stay untouched for 5 s): the first maximum of at least 60 % of the MPP is the
    // full string (a start transient is weaker); n from the maxima below it of at least 30 % of the MPP, the largest
    // n in 2..6 that puts each at m - 0.25 .. m + 0.5 shares of 0.82 Voc / n (m < n; shaded maxima sit high).
    // Nothing if the MPP is not a recorded maximum. Works on a copy; drops it if RT rewrote the arrays meanwhile
    // (mid: test hook, runs after the copy). Two verified sweeps in a row with one maximum each and n unknown: n = 1.
    // True if n grew.
    template<class Mid = decltype([](PeakScan &) {})>
    bool learn(Mid mid = {}) {
        const uint8_t g0 = gen.load(std::memory_order_acquire);
        const uint8_t pd = pending.exchange(0, std::memory_order_acquire);
        if (!pd) return false;
        if (pd == 3) return one = 0, false;
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
        if (!mpp || nc != 1) one = 0;
        else if (pd == 2 && !n && ++one >= 2) {
            n = 1;
            ESP_LOGI("mppt", "scan: single peak learned from 2 verified sweeps");
            return true;
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
    // settleMs(): power filter settling time [ms], hold: pw reads low (LS fading in), restart the settle time
    template<class SettleMs>
    __attribute__((always_inline)) float update(uint32_t now, float vinFast, float vin, float pw, uint16_t d,
                                                float vMin, float pMin, SettleMs settleMs, bool hold = false) {
        if (phase == Idle) {
            if (n < 2 || !(voc > 0)) return NAN;
            if (vin > 0 && pw / vin > iEnv) iEnv = pw / vin;
            if (pRef <= 0) { // tracking after a sweep: reference 5 s later
                if (pRef < 0) pRef = 0, tPhase = now;
                else if (now - tPhase > 5000) pRef = pw, vRef = vin, tOff = now, pLo = std::max(pLo, pw);
                return NAN;
            }
            const float vs0 = kVmp * voc / (float) n;
            // trigger: off the reference Vin or power for 15 s, the power steady (+-10 %) for the last 5 s
            if (!trigger || (std::fabs(vin - vRef) < 0.5f * vs0 && pw > 0.7f * pRef && pw < 1.4f * pRef)) tOff = now;
            if (std::fabs(pw - pS) > 0.1f * pS) pS = pw, tS = now;
            const bool per = interval && now - tLast > interval * 1000u;
            const bool due = force || per || (now - tOff > 15000 && now - tS > kSteadyMs && now - tLast > 60000);
            // riseK above pLo the light rose, iEnv may miss the current of brighter substrings (shade on the others):
            // no current bound, down to pMin / 5
            const bool rise = pw > riseK * pLo;
            if (!due || pw < (rise ? 0.2f : 1.f) * pMin || !begin(vin, pw, d, vMin, now, per, rise, vs0)) return NAN;
        }
        const float vt = phase == Near ? rv[k] : phase == Probe || phase == Go ? (ret ? rv[k] : vc[k]) : v0;
        const float dv = std::fabs(vinFast - vt);
        if (!settled && (dv < 0.01f * vt || now - tPhase > kTravelMs)) settled = true, band = dv < 0.01f * vt, tPhase = now;
        // settled in the band, then out of 2 % (Vin overshoots at low power): travel again
        if (settled && band && dv > 0.02f * vt) settled = false, tPhase = now, acc.clear();
        if (settled && hold) tPhase = now, acc.clear();
        if (settled) {
            const float ts = std::fmin(settleMs(), 1000.f), dt = (float) (now - tPhase); // fmin: NAN -> 1 s
            if (dt > ts) acc.add(pw);
            if (dt > ts + kMeasMs) step(now, vin);
        }
        if (phase != Idle && now - t0 > 30000) phase = Idle, pRef = -1, ended = 4, ret = false;
        return phase == Idle ? NAN : (float) d * std::sqrt(vinFast / vt);
    }

    // a due scan: candidates, then the start; false: nothing to probe, back to tracking
    __attribute__((noinline)) bool begin(float vin, float pw, uint16_t d, float vMin, uint32_t now, bool per, bool rise,
                                         float vs0) {
        tLast = now, force = false;
        // m0 of n substrings conduct at our peak. With weaker substrings conducting a peak sits above m unshaded
        // shares, so Vin / m0 can be high; the share from Voc misses heating and the Vmpp/Voc spread: take the mean
        const int m0 = std::clamp((int) std::lround(vin / vs0), 1, (int) n);
        const float sv = vin / (float) m0, sh = 0.5f * (sv + vs0);
        heat = kVmp != kVmpFallback && sv < heatCut * vs0;
        const float shUp = heat ? sv : sh;
        claim();
        nc = 0;
        for (int m = 1; m <= n; ++m) {
            const float v = m > m0 ? probeHigh * (float) m * shUp : probeLow * (float) m * sh;
            // unreachable, at our own peak, or a lower peak that cannot beat us even at the best current seen
            if (m == m0 || v < vMin || v > 0.95f * voc ||
                (m < m0 && !rise && iEnv > 0 && (float) m * sh * iEnv < pw * (1 + margin)))
                continue;
            vc[nc++] = v;
        }
        up = rise, phase = Ref, v0 = vin, d0 = d, t0 = tPhase = now, settled = false, k = 0;
        if (!armNear(nearDv && (nc || per), vMin, now) && !nc) {
            phase = Idle, pRef = pw, vRef = vin;
            if (rise) pLo = pw;
            return false;
        }
        acc.clear();
        return true;
    }

    // on: v0 and the near probes at v0 -+ nearDv in rv 0..2 (NAN: out of range, skipped), starts them. False: off or
    // both out of range (rv[0] NAN)
    __attribute__((noinline)) bool armNear(bool on, float vMin, uint32_t now) {
        bool any = false;
        for (int i = 1; on && i < 3; ++i) {
            const float v = v0 + (i == 2 ? 0.1f : -0.1f) * (float) nearDv;
            rv[i] = v >= vMin && v <= 0.95f * voc ? v : NAN;
            any = any || !std::isnan(rv[i]);
        }
        rv[0] = any ? v0 : NAN;
        if (any) nearStep(now, 0);
        return any;
    }

    // scan start (k 0) or a near point measured: the next in-range one, then Ref with the timeout started afresh
    __attribute__((noinline)) void nearStep(uint32_t now, float p) {
        if (phase == Near) rp[k++] = p;
        else phase = Near;
        while (k < 3 && std::isnan(rv[k])) rp[k++] = NAN;
        if (k == 3) phase = Ref, k = 0, t0 = now;
    }

    // start point measured after the near probes: on to the candidates; without any, back to tracking
    __attribute__((noinline)) void refDone(float p, float vin) {
        if (!std::isnan(rv[0])) logNear(p);
        p0 = p, phase = Probe;
        if (!nc) phase = Idle, pRef = p, vRef = vin, ended = 1, pLo = up ? std::max(pLo, p) : pLo;
    }

    // ratios against the start power interpolated between before (rp[0]) and after (p) in measurement order; drift:
    // the two differ by more than 1 %
    __attribute__((noinline)) void logNear(float p) const {
        char s[2][12];
        const int m = !std::isnan(rp[1]) + !std::isnan(rp[2]);
        for (int i = 0, j = 0; i < 2; ++i)
            if (std::isnan(rp[i + 1])) strcpy(s[i], "n/a");
            else snprintf(s[i], sizeof s[i], "x%.3f", rp[i + 1] / (rp[0] + (p - rp[0]) * (float) ++j / (float) (m + 1)));
        const float dv = 0.1f * (float) nearDv;
        ESP_LOGI("mppt", "scan near: v0 %.1f V, -%.1fV %s, +%.1fV %s, ref %.1f->%.1f W%s%s", v0, dv, s[0], dv, s[1], rp[0],
                 p, std::fabs(p - rp[0]) > 0.005f * (p + rp[0]) ? ", drift" : "", nc ? "" : ", near only");
    }

    __attribute__((always_inline)) void step(uint32_t now, float vin) {
        const float p = acc.pop();
        settled = false, tPhase = now;
        if (phase == Ref) {
            refDone(p, vin);
        } else if (phase == Near) {
            nearStep(now, p);
        } else if (phase == Probe && ret) {
            const float r = p / rp[k] - 1;
            ESP_LOGI("mppt", "sweep return: %.1f V %.1f W, down %.1f W (%+.0f %%)", rv[k], p, rp[k], 100 * r);
            if (std::fabs(r) > std::fabs(rDev)) rDev = r;
            if (p > rBest) rBest = p, rb = k;
            if (++k == nr) k = rb, phase = Go;
        } else if (phase == Probe) {
            pc[k] = p, iEnv = std::max(iEnv, p / vin);
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
            ESP_LOGI("mppt", "scan: P %.1f->%.1f W at %.1fV, %u probes, best x%.3f at %.1fV%s%s", p0, p, v0, (unsigned) nc, g,
                     vc[best], !steady ? " (light changed)" : move ? ", moving" : "", heat ? ", heat" : "");
            if (move) {
                k = (uint8_t) best, phase = Go, pRef = pc[best], vRef = vc[best], pLo = std::max(pLo, pRef);
            } else {
                phase = Idle, pRef = p, vRef = vin, ended = steady ? 1 : 3;
                // a probe within 2 margins (clouds change the ratio): again in 60 s, in rising light still as such once
                const bool again = g > 1 - 2 * margin;
                if (up && steady && (!again || rt)) pLo = std::max(pLo, p);
                rt = up && steady && again && !rt;
                if (again && interval > 60) tLast = now - (interval - 60) * 1000u;
            }
        } else {
            phase = Idle, vRef = vin, ended = ret ? 6 : 2, rLand = p, ret = false;
        }
    }
};
