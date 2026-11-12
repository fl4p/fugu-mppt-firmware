#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>

struct WireGuardConf {
    uint8_t mode = 1;                          // 0 off, 1 warn only, 2 warn + derate + trip
    float pWarn = 2.f, pDerate = 4.f, pTrip = 8.f; // [W] excess power
    float drMin = 0.004f;                      // [Ω] minimum excess resistance for any level
    float rMax = 0;                            // [Ω] absolute cap on the Rc baseline, 0 = none
};

/**
 * Output wire-fault guard: compares the output-path resistance against a per-board baseline and
 * grades the excess power ΔR·I² into ok/warn/derate/trip. Three paths: per-frame excess drop
 * against the frozen baseline fit (frame(), judged on 1-min means at Iout >= I_MIN), the slow
 * fit Rc and the per-sweep Ro (tick()). The slow paths are graded at the peak current of the last
 * 1-2 charging hours. The baseline is the median of up to DAYS accepted day medians; a day's
 * median above baseline + drMin is rejected. Freshness ages in charging minutes, so a night
 * neither releases nor invalidates the last verdict. Trip latches until reset() or reboot.
 * ioutCap() and tripped() may be read from another core; everything else is single-core.
 */
class WireGuard {
public:
    enum Level : uint8_t { Ok = 0, Warn, Derate, Trip };
    static constexpr int8_t WF_UNVERIFIED = -1;
    static constexpr uint8_t DAYS = 14, LEARN_DAYS = 7, DAY_N = 16, VER = 1;
    static constexpr float I_MIN = 5.f;        // [A]
    static constexpr float HYST = 0.75f;       // release thresholds = HYST · raise thresholds
    static constexpr float OVER_DERATE = 1.25f; // derated but Pw still above this · pDerate -> trip
    static constexpr uint8_t N_FAST = 5, N_RC = 30, N_RO = 3, MIN_FRAMES = 5;
    static constexpr uint16_t STALE_FAST = 15, STALE_RC = 60, STALE_RO = 360; // [charging minutes]
    static constexpr uint32_t LOWER_S = 1800, DAY_SAMPLE_S = 900, NIGHT_S = 3600;
    static constexpr uint32_t DAY_MIN_S = 12 * 3600, DAY_MAX_S = 36 * 3600;
    static constexpr uint16_t UNV_CHARGE_MIN = 30; // a charging day: this much time at Iout >= I_MIN
    static constexpr uint8_t UNV_DAYS = 3;

    static constexpr int16_t NONE = INT16_MIN;

    // persisted baseline, ring buffers of day medians: R in 10 µΩ, a in 0.1 mV. anc*: lowest
    // baseline since learning ended; later days must stay within drMin/2 of it (no creep-in).
    struct Blob {
        uint8_t ver, hRc, nRc, hRo, nRo, days, unvDays, _r;
        int16_t ancRc, ancRo;
        int16_t rc[DAYS], a[DAYS], rb[DAYS], ro[DAYS];
    };

    WireGuardConf conf{};
    uint8_t paths = 7; // graded paths: 1 fast, 2 Rc, 4 Ro (diagnostics)

    void reset() {
        const auto c = conf;
        this->~WireGuard();
        new(this) WireGuard();
        conf = c;
        dirty = true;
    }

    bool load(const void *p, size_t n) {
        Blob b;
        if (n != sizeof b) return false;
        memcpy(&b, p, n);
        if (b.ver != VER || b.nRc > DAYS || b.nRo > DAYS || b.hRc >= DAYS || b.hRo >= DAYS) return false;
        h = b;
        updateBase();
        return true;
    }

    [[nodiscard]] const Blob &blob() const { return h; }
    // a day closed or reset(): the baseline wants writing, but only while the converter is off
    [[nodiscard]] bool saveDue(bool running) const { return dirty && !running && conf.mode; }
    void saved() { dirty = false; }

    // per steady BMS frame: dv = Vout − Vpack
    void frame(float dv, float iout, float ibat) {
        if (!conf.mode || !std::isfinite(dv) || !std::isfinite(iout) || !std::isfinite(ibat)) return;
        if (std::fabs(ibat) > ibMinute) ibMinute = std::fabs(ibat);
        if (iout < I_MIN) return;
        se += dv - (bA + rcBase() * iout + bRb * ibat);
        si += iout;
        sib += ibat;
        ++nf;
    }

    /**
     * Every loop tick. rc/a/rb: the slow fit (NAN = none), ro + roStamp: the latest sweep value and
     * its time stamp (a new stamp = a new sweep). Returns true when wf() or alertLevel() may have
     * changed.
     */
    bool tick(uint32_t nowS, float iout, bool running, float rc, float a, float rb, float ro, uint32_t roStamp) {
        if (!conf.mode) return false;
        if (!winS) winS = dayS = runS = nowS;
        if (running) runS = nowS;
        if (std::isfinite(iout)) { iSum += iout; ++iN; }
        if (std::isfinite(ro) && roStamp && roStamp != roSeen) {
            roSeen = roStamp;
            onRo(ro);
        }
        if (std::isfinite(rc) && std::isfinite(a) && std::isfinite(rb)) {
            rcIn = rc; aIn = a; rbIn = rb;
        } else rcIn = NAN;
        if (std::isfinite(rcIn) && nowS - dLastS >= dStep) {
            dLastS = nowS;
            daySample(dRc, dA, dRb, nD, &dStep);
        }
        const auto prevAl = alertLevel();
        const auto prevWf = wf();
        if (nowS - winS >= 60) minute(nowS);
        if ((!running && nowS - runS >= NIGHT_S && nowS - dayS >= DAY_MIN_S) || nowS - dayS >= DAY_MAX_S)
            closeDay(nowS);
        return prevAl != alertLevel() || prevWf != wf();
    }

    // [A] output current cap, NAN = none
    [[nodiscard]] float ioutCap() const { return cap.load(std::memory_order_relaxed); }
    [[nodiscard]] bool tripped() const { return trip.load(std::memory_order_relaxed); }

    [[nodiscard]] bool learning() const { return h.days < LEARN_DAYS; }
    [[nodiscard]] bool armed() const { return conf.mode == 2 && !learning(); }
    // level the guard reached, regardless of mode and learning
    [[nodiscard]] uint8_t level() const { return lvl; }
    [[nodiscard]] uint8_t effLevel() const { return armed() ? lvl : std::min<uint8_t>(lvl, Warn); }
    [[nodiscard]] bool rbWarn() const { return rbLvl; }
    [[nodiscard]] bool anyFresh() const { return fresh(fast) || fresh(rcP) || fresh(roP); }
    [[nodiscard]] bool unverifiedLong() const { return h.unvDays > UNV_DAYS; }

    // telemetry: 0..3 level, WF_UNVERIFIED while no path can judge and nothing is held
    [[nodiscard]] int8_t wf() const {
        const uint8_t l = std::max<uint8_t>(effLevel(), rbLvl);
        return !l && !anyFresh() ? WF_UNVERIFIED : (int8_t) l;
    }

    // alert state: the held level, or "unverified" after UNV_DAYS charging days without data
    [[nodiscard]] int8_t alertLevel() const {
        const uint8_t l = std::max<uint8_t>(effLevel(), rbLvl);
        return !l && unverifiedLong() ? WF_UNVERIFIED : (int8_t) l;
    }

    // [W] excess power of the worst fresh path, NAN when none
    [[nodiscard]] float pw() const {
        float p = NAN;
        for (auto *q: {&fast, &rcP, &roP})
            if (fresh(*q) && !(q->pw <= p)) p = q->pw;
        return p;
    }

    struct PathView { float dr, pw; bool fresh; };
    [[nodiscard]] PathView fastPath() const { return {fast.dr, fast.pw, fresh(fast)}; }
    [[nodiscard]] PathView rcPath() const { return {rcP.dr, rcP.pw, fresh(rcP)}; }
    [[nodiscard]] PathView roPath() const { return {roP.dr, roP.pw, fresh(roP)}; }
    [[nodiscard]] PathView rbPath() const { return {rbP.dr, rbP.pw, fresh(rbP)}; }
    [[nodiscard]] float baseRc() const { return rcBase(); }
    [[nodiscard]] float baseA() const { return bA; }
    [[nodiscard]] float baseRb() const { return bRb; }
    [[nodiscard]] float baseRo() const { return bRo; }
    [[nodiscard]] float iRef() const { return std::max(ioPk[0], ioPk[1]); }
    [[nodiscard]] uint8_t dayCount() const { return h.days; }
    [[nodiscard]] uint8_t daySamples() const { return nD; }
    [[nodiscard]] uint8_t daySweeps() const { return nDRo; }

private:
    struct Hist { // 2-bit levels, newest in the low bits
        uint64_t b = 0;
        uint8_t n = 0;
        void push(uint8_t l) { b = b << 2 | l; if (n < 32) ++n; }
        [[nodiscard]] uint8_t minOf(uint8_t k) const {
            if (n < k) return 0;
            uint8_t m = Trip;
            for (uint8_t i = 0; i < k; ++i) m = std::min<uint8_t>(m, (b >> (2 * i)) & 3);
            return m;
        }
    };
    struct Path {
        uint16_t stale; // [charging minutes]
        float dr = NAN, pw = NAN;
        uint32_t cm = 0; // charging minute of the last value
        bool has = false;
        Hist hist{};
        explicit Path(uint16_t s) : stale(s) {}
    };

    Blob h{VER, 0, 0, 0, 0, 0, 0, 0, NONE, NONE, {}, {}, {}, {}};
    bool dirty = false;
    float bRc = NAN, bA = NAN, bRb = NAN, bRo = NAN;

    std::atomic<float> cap{NAN};
    std::atomic<bool> trip{false};
    uint8_t lvl = Ok, rbLvl = 0, overDerate = 0;
    uint32_t belowS = 0, rbBelowS = 0;

    Path fast{STALE_FAST}, rcP{STALE_RC}, roP{STALE_RO}, rbP{STALE_RC};
    float rcIn = NAN, aIn = NAN, rbIn = NAN, roIn = NAN;
    uint32_t roSeen = 0;
    bool roNew = false;

    float se = 0, si = 0, sib = 0, iSum = 0, ibMinute = 0;
    uint8_t nf = 0;
    uint16_t iN = 0;
    uint32_t winS = 0, cm = 0;
    float ioPk[2]{}, ibPk[2]{}; // peak minute current: current / previous charging hour
    uint8_t pkMin = 0;

    int16_t dRc[DAY_N]{}, dA[DAY_N]{}, dRb[DAY_N]{}, dRo[DAY_N]{};
    uint8_t nD = 0, nDRo = 0;
    uint16_t dChargeMin = 0, dFastN = 0, dFastHi = 0; // fast-path minutes today, of which dR >= drMin/2
    bool dFast = false;
    uint32_t dStep = DAY_SAMPLE_S, dLastS = 0, dayS = 0, runS = 0;

    static int16_t q(float x, float k) {
        const float v = std::round(x * k);
        return (int16_t) std::max(-32767.f, std::min(32767.f, v));
    }

    static float median(const int16_t *x, uint8_t n) {
        if (!n) return NAN;
        int16_t t[DAY_N > DAYS ? DAY_N : DAYS];
        std::copy(x, x + n, t);
        std::sort(t, t + n);
        return n & 1 ? t[n / 2] : 0.5f * ((float) t[n / 2 - 1] + (float) t[n / 2]);
    }

    [[nodiscard]] float rcBase() const {
        if (conf.rMax > 0) return std::isfinite(bRc) ? std::min(bRc, conf.rMax) : conf.rMax;
        return bRc;
    }

    [[nodiscard]] bool fresh(const Path &p) const { return p.has && cm - p.cm <= p.stale; }

    [[nodiscard]] uint8_t gradeOf(float pw, float dr, float k) const {
        if (!(dr >= conf.drMin * k)) return Ok;
        return pw >= conf.pTrip * k ? Trip : pw >= conf.pDerate * k ? Derate : pw >= conf.pWarn * k ? Warn : Ok;
    }

    void updateBase() {
        auto ring = [](const int16_t *x, uint8_t n, float k) { return n ? median(x, n) / k : NAN; };
        bRc = ring(h.rc, h.nRc, 1e5f);
        bA = ring(h.a, h.nRc, 1e4f);
        bRb = ring(h.rb, h.nRc, 1e5f);
        bRo = ring(h.ro, h.nRo, 1e5f);
    }

    // decimating day buffer: when full keep every other sample and halve the rate
    static void compact(int16_t *x, uint8_t n) {
        for (uint8_t i = 0; i < n / 2; ++i) x[i] = x[2 * i];
    }

    void daySample(int16_t *rc, int16_t *a, int16_t *rb, uint8_t &n, uint32_t *step) {
        if (n == DAY_N) {
            compact(rc, n); compact(a, n); compact(rb, n);
            n /= 2;
            *step *= 2;
        }
        rc[n] = q(rcIn, 1e5f);
        a[n] = q(aIn, 1e4f);
        rb[n] = q(rbIn, 1e5f);
        ++n;
    }

    void onRo(float ro) {
        roIn = ro;
        if (nDRo == DAY_N) { compact(dRo, nDRo); nDRo /= 2; }
        dRo[nDRo++] = q(ro, 1e5f);
        roNew = true;
    }

    static void peakAdd(float *pk, float x) { pk[0] = std::max(pk[0], x); }

    void slow(Path &p, float dr, float iref) {
        if (!std::isfinite(dr) || iref < I_MIN) return;
        p.dr = dr;
        p.pw = dr * iref * iref;
        p.cm = cm;
        p.has = true;
        p.hist.push(gradeOf(p.pw, p.dr, 1));
    }

    void minute(uint32_t nowS) {
        winS = nowS;
        const float im = iN ? iSum / iN : NAN;
        iSum = 0;
        iN = 0;
        if (im >= I_MIN) {
            ++cm;
            ++dChargeMin;
            peakAdd(ioPk, im);
            peakAdd(ibPk, ibMinute);
            if (++pkMin >= 60) {
                pkMin = 0;
                ioPk[1] = ioPk[0]; ioPk[0] = 0;
                ibPk[1] = ibPk[0]; ibPk[0] = 0;
            }
        }
        ibMinute = 0;

        const bool op = nf >= MIN_FRAMES;
        const float i = op ? si / nf : NAN, ib = op ? sib / nf : NAN, e = op ? se / nf : NAN;
        if (op && std::isfinite(e) && (paths & 1)) {
            fast.dr = e / i;
            fast.pw = e * i;
            fast.cm = cm;
            fast.has = true;
            dFast = true;
            fast.hist.push(gradeOf(fast.pw, fast.dr, 1));
            ++dFastN;
            if (fast.dr >= 0.5f * conf.drMin) ++dFastHi;
            overDerate = armed() && lvl == Derate && fast.pw >= conf.pDerate * OVER_DERATE ? overDerate + 1 : 0;
        }
        se = si = sib = 0;
        nf = 0;

        // a sweep before the first charging minute (boot sweep) waits for a reference current
        if (roNew && (!std::isfinite(bRo) || iRef() >= I_MIN)) {
            roNew = false;
            if (paths & 4) slow(roP, roIn - bRo, iRef());
        }
        if (std::isfinite(rcIn)) {
            // R1 and R2 trade off when output and battery current move together, so judge the whole
            // fit against the baseline at this minute's operating point instead of R1 alone
            const float ex = op ? (aIn - bA) + (rcIn - rcBase()) * i + (rbIn - bRb) * ib : NAN; // [V]
            if (paths & 2) slow(rcP, ex / i, iRef());
            // Rb only up to what the total excess drop supports, so an R1/R2 trade-off alone can't warn
            if (std::fabs(ib) >= I_MIN) slow(rbP, std::fmin(rbIn - bRb, ex / std::fabs(ib)), std::max(ibPk[0], ibPk[1]));
        }

        for (auto *p: {&fast, &rcP, &roP, &rbP})
            if (p->has && !fresh(*p)) *p = Path{p->stale};
        grade(nowS);
    }

    void grade(uint32_t nowS) {
        uint8_t up = std::max({fast.hist.minOf(N_FAST), rcP.hist.minOf(N_RC), roP.hist.minOf(N_RO)});
        if (overDerate >= N_FAST) up = Trip;
        if (learning()) up = std::min<uint8_t>(up, Warn);
        if (up > lvl) {
            lvl = up;
            belowS = 0;
        } else if (lvl > Ok && lvl < Trip) {
            uint8_t down = Ok;
            bool any = false;
            for (auto *p: {&fast, &rcP, &roP})
                if (fresh(*p)) { any = true; down = std::max(down, gradeOf(p->pw, p->dr, HYST)); }
            if (!any || down >= lvl) belowS = 0;
            else if (!belowS) belowS = nowS;
            else if (nowS - belowS >= LOWER_S) { lvl = down; belowS = 0; }
        }

        if (rbP.hist.minOf(N_RC) >= Warn) { rbLvl = 1; rbBelowS = 0; }
        else if (rbLvl) {
            if (!fresh(rbP) || gradeOf(rbP.pw, rbP.dr, HYST) >= Warn) rbBelowS = 0;
            else if (!rbBelowS) rbBelowS = nowS;
            else if (nowS - rbBelowS >= LOWER_S) { rbLvl = 0; rbBelowS = 0; }
        }

        // cap follows the fast path; without it, it holds, or starts from the worst fresh slow path
        float c = NAN;
        if (effLevel() >= Derate) {
            c = cap.load(std::memory_order_relaxed);
            float dr = fresh(fast) ? fast.dr : NAN;
            if (!std::isfinite(dr) && !std::isfinite(c))
                for (auto *p: {&rcP, &roP})
                    if (fresh(*p) && !(p->dr <= dr)) dr = p->dr;
            if (dr > 0) c = std::sqrt(conf.pDerate / dr);
        }
        cap.store(c, std::memory_order_relaxed);
        if (effLevel() == Trip) trip.store(true, std::memory_order_relaxed);
    }

    void closeDay(uint32_t nowS) {
        dayS = nowS;
        bool data = false;
        if (nD >= 4) {
            data = true;
            const float m = median(dRc, nD) / 1e5f;
            // the fit can absorb a fault into its offset or Rb, so the fast path vetoes too
            if (!(m > bRc + conf.drMin) && accept(m, h.ancRc) && 2 * dFastHi <= dFastN) {
                h.rc[h.hRc] = q(m, 1e5f);
                h.a[h.hRc] = q(median(dA, nD) / 1e4f, 1e4f);
                h.rb[h.hRc] = q(median(dRb, nD) / 1e5f, 1e5f);
                h.hRc = (h.hRc + 1) % DAYS;
                if (h.nRc < DAYS) ++h.nRc;
            }
        }
        if (nDRo >= 2) {
            data = true;
            const float m = median(dRo, nDRo) / 1e5f;
            if (!(m > bRo + conf.drMin) && accept(m, h.ancRo)) {
                h.ro[h.hRo] = q(m, 1e5f);
                h.hRo = (h.hRo + 1) % DAYS;
                if (h.nRo < DAYS) ++h.nRo;
            }
        }
        data = data || dFast;
        if (data) {
            if (h.days < 255) ++h.days;
            h.unvDays = 0;
        } else if (dChargeMin >= UNV_CHARGE_MIN && h.unvDays < 255) ++h.unvDays;
        nD = nDRo = 0;
        dStep = DAY_SAMPLE_S;
        dChargeMin = dFastN = dFastHi = 0;
        dFast = false;
        updateBase();
        if (!learning()) {
            anchor(h.ancRc, bRc);
            anchor(h.ancRo, bRo);
        }
        dirty = true;
    }

    [[nodiscard]] bool accept(float m, int16_t anc) const {
        return learning() || anc == NONE || !(m > (float) anc / 1e5f + 0.5f * conf.drMin);
    }

    static void anchor(int16_t &anc, float base) {
        if (!std::isfinite(base)) return;
        const int16_t b = q(base, 1e5f);
        if (anc == NONE || b < anc) anc = b;
    }
};
