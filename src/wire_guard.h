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
 * grades the excess power ΔR·I² into ok/warn/derate/trip. Three paths: per-frame drop against the
 * baseline Rc/Rb (frame(), judged on 1-min means at Iout >= I_MIN), the slope of those drops
 * against Iout, and the sweep Ro (tick()). The voltage offset between converter and BMS drifts,
 * so the fast path takes it from a slow regression of the frame drops against Iout instead of the
 * baseline: an offset carries no current, a resistance shows as slope. Minutes off that regression
 * line by pWarn/2 (steps) don't enter it. The Ro path grades the median of the last RO_MED sweeps
 * (nothing below RO_MED_MIN), at most once per RO_GAP charging minutes, so a burst of noisy sweeps
 * can't fill its N_RO history; it releases a level only when all of those sweeps are below. The
 * slope and Ro paths are graded at the peak current of the last 1-2 charging hours. A day whose
 * median fast-path ΔR (without BMS: Ro) is 2·drMin above baseline warns regardless of current,
 * never more. The baseline is the median of up to DAYS accepted day medians. Only a good day enters
 * (rcGood: slow-fit samples taken while charging, enough frames and current spread; roGood: sweeps).
 * A path is graded only against a baseline of BASE_MIN_DAYS stored days (Rc: or a configured rMax).
 * While learning (fewer than LEARN_DAYS counted days) every good day enters, so the median outvotes
 * an odd first day; after that a day's median above baseline + drMin is rejected. A day closes at
 * the first hour without conversion DAY_MIN_S after the last close, or at the first one after a cold
 * start if it counts, or after DAY_MAX_S. The open day and the offset regression survive a restart
 * through saveDay()/restoreDay() and saveFit()/restoreFit() (RTC RAM). Freshness ages in charging
 * minutes, so a night neither releases nor invalidates the last verdict. Trip latches until reset() or reboot.
 * ioutCap() and tripped() may be read from another core; everything else is single-core.
 */
class WireGuard {
public:
    enum Level : uint8_t { Ok = 0, Warn, Derate, Trip };
    static constexpr int8_t WF_UNVERIFIED = -1;
    static constexpr uint8_t DAYS = 14, LEARN_DAYS = 7, BASE_MIN_DAYS = 3, DAY_N = 16, VER = 1;
    static constexpr float I_MIN = 5.f;        // [A]
    static constexpr float HYST = 0.75f;       // release thresholds = HYST · raise thresholds
    static constexpr float OVER_DERATE = 1.25f; // derated but Pw still above this · pDerate -> trip
    static constexpr uint8_t N_FAST = 5, N_RC = 30, N_RO = 3, MIN_FRAMES = 5;
    static constexpr uint16_t STALE_FAST = 15, STALE_RC = 60, STALE_RO = 360; // [charging minutes]
    // Ro: median window [sweeps], fewest sweeps graded, [charging minutes] between graded values.
    // RO_GAP > RO_MED sweeps at the fastest rate seen (~2 min), so graded values share no sweeps.
    static constexpr uint8_t RO_MED = 5, RO_MED_MIN = 3;
    static constexpr uint16_t RO_GAP = 20;
    static constexpr uint32_t LOWER_S = 1800, DAY_SAMPLE_S = 900, NIGHT_S = 3600;
    static constexpr uint32_t DAY_MIN_S = 12 * 3600, DAY_MAX_S = 36 * 3600;
    static constexpr uint16_t UNV_CHARGE_MIN = 30; // a charging day: this much time at Iout >= I_MIN
    static constexpr uint8_t UNV_DAYS = 3;
    static constexpr uint16_t OFS_TAU = 120;   // [fast minutes] memory of the offset regression
    static constexpr float OFS_W_MIN = 150;    // [frames] in that regression before it is used
    static constexpr float OFS_VAR_MIN = 4.f;  // [A²] current spread needed to tell offset from slope
    static constexpr float OFS_MAX = 0.15f;    // [V] |offset − baseline a| beyond this: no verdict
    static constexpr uint32_t PW_AGE_S = 120;  // pw() reports values at most this old
    // a day enters the baseline with >= DAY_RC_N slow-fit samples taken while charging, >= DAY_W_MIN
    // frames at Iout >= I_MIN and a current variance >= DAY_VAR_MIN [A²] among them; Ro: >= DAY_RO_N sweeps
    static constexpr uint8_t DAY_RC_N = 4, DAY_RO_N = 2;
    static constexpr float DAY_W_MIN = 300, DAY_VAR_MIN = OFS_VAR_MIN;

    static constexpr int16_t NONE = INT16_MIN;

    // persisted baseline, ring buffers of day medians: R in 10 µΩ, a in 0.1 mV. anc*: lowest
    // baseline since learning ended; later days must stay within drMin/2 of it (no creep-in).
    // flags bit 0: day warning, from the median excess resistance of the last judged day.
    struct Blob {
        uint8_t ver, hRc, nRc, hRo, nRo, days, unvDays, flags;
        int16_t ancRc, ancRo;
        int16_t rc[DAYS], a[DAYS], rb[DAYS], ro[DAYS];
    };

    // the open day: decimating sample buffers (rc/a/rb slow fit, ro sweeps, dr fast-path ΔR) and
    // the frame sums of Iout >= I_MIN (count, Σi, Σi²) for the quality gate
    struct Day {
        int16_t rc[DAY_N], a[DAY_N], rb[DAY_N], ro[DAY_N], dr[DAY_N];
        float w, i, i2;
        uint32_t step, lastS, startS; // in a DayRtc: ages [s]
        uint16_t chargeMin, fastN, fastHi; // charging minutes, fast-path minutes, of which dR >= drMin/2
        uint8_t n, nRo, nDr, drStep, drSkip;
        bool fast, bms; // a fast-path minute, a BMS frame
    };
    // a copy of the open day in RAM that survives a restart (RTC_NOINIT on the device)
    struct DayRtc {
        uint32_t magic;
        Day d;
        uint32_t sum;
    };
    static constexpr uint32_t DAY_MAGIC = 0x57440000u | sizeof(Day);
    // the offset regression sums in RTC RAM; float rounding (~1e-7) is far below the fit's noise
    struct FitRtc {
        float rg[7];
        uint32_t sum;
    };
    static constexpr uint32_t FIT_MAGIC = 0x57460000u | sizeof(FitRtc);

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

    void saveDay(DayRtc &r) const {
        r.magic = DAY_MAGIC;
        r.d = d;
        if (started) {
            r.d.lastS = tS - d.lastS;
            r.d.startS = tS - d.startS;
        }
        r.sum = sum(&r.d, sizeof r.d);
    }

    void saveFit(FitRtc &r) const {
        for (uint8_t k = 0; k < 7; ++k) r.rg[k] = (float) rg[k];
        r.sum = sum(r.rg, sizeof r.rg, FIT_MAGIC);
    }

    // before the first tick(); false (cold regression) unless r holds consistent sums
    bool restoreFit(const FitRtc &r) {
        if (r.sum != sum(r.rg, sizeof r.rg, FIT_MAGIC) || !(r.rg[0] >= 0) || !(r.rg[2] >= 0)) return false;
        for (float x: r.rg)
            if (!std::isfinite(x)) return false;
        std::copy(r.rg, r.rg + 7, rg);
        return true;
    }

    // before the first tick(); false (fresh day) unless r holds a consistent day
    bool restoreDay(const DayRtc &r) {
        const auto &x = r.d;
        if (r.magic != DAY_MAGIC || r.sum != sum(&r.d, sizeof r.d) || x.n > DAY_N || x.nRo > DAY_N || x.nDr > DAY_N ||
            !x.drStep || x.drSkip >= x.drStep || x.step < DAY_SAMPLE_S || x.startS > 2 * DAY_MAX_S || !(x.w >= 0))
            return false;
        d = x;
        coldDay = false;
        return true;
    }
    // a day closed or reset(): the baseline wants writing, but only while the converter is off
    [[nodiscard]] bool saveDue(bool running) const { return dirty && !running && conf.mode; }
    void saved() { dirty = false; }

    // per steady BMS frame: dv = Vout − Vpack
    void frame(float dv, float iout, float ibat) {
        if (!conf.mode || !std::isfinite(dv) || !std::isfinite(iout) || !std::isfinite(ibat)) return;
        if (std::fabs(ibat) > ibMinute) ibMinute = std::fabs(ibat);
        d.bms = true;
        if (iout < I_MIN) return;
        const float x[7]{1, iout, iout * iout, dv, iout * dv, ibat, iout * ibat};
        for (uint8_t k = 0; k < 7; ++k) ms[k] += x[k];
        d.w += 1;
        d.i += iout;
        d.i2 += x[2];
    }

    /**
     * Every loop tick. rc/a/rb: the slow fit (NAN = none), ro + roStamp: the latest sweep value and
     * its time stamp (a new stamp = a new sweep). Returns true when wf() changed.
     */
    bool tick(uint32_t nowS, float iout, bool running, float rc, float a, float rb, float ro, uint32_t roStamp) {
        if (!conf.mode) return false;
        tS = nowS;
        if (!started) {
            started = true;
            winS = runS = nowS;
            d.startS = nowS - d.startS;
            d.lastS = nowS - d.lastS;
        }
        if (running) runS = nowS;
        if (std::isfinite(iout)) { iSum += iout; ++iN; }
        if (std::isfinite(ro) && roStamp && roStamp != roSeen) {
            roSeen = roStamp;
            onRo(ro);
        }
        if (std::isfinite(rc) && std::isfinite(a) && std::isfinite(rb)) {
            rcIn = rc; aIn = a; rbIn = rb;
        } else rcIn = NAN;
        if (std::isfinite(rcIn) && iout >= I_MIN && nowS - d.lastS >= d.step) {
            d.lastS = nowS;
            daySample();
        }
        const auto prevWf = wf();
        if (nowS - winS >= 60) minute(nowS);
        // the first day after a cold start closes at the first night if its data is good enough
        const uint32_t age = nowS - d.startS;
        if ((!running && nowS - runS >= NIGHT_S && (age >= DAY_MIN_S || (coldDay && dayCounts()))) ||
            age >= DAY_MAX_S)
            closeDay(nowS);
        return prevWf != wf();
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
    [[nodiscard]] bool dayWarn() const { return h.flags & 1; }
    [[nodiscard]] bool anyFresh() const { return fresh(fast) || fresh(rcP) || fresh(roP); }
    [[nodiscard]] bool unverifiedLong() const { return h.unvDays > UNV_DAYS; }

    // telemetry and alert: 0..3 level, WF_UNVERIFIED while nothing is held and no path can judge
    // or there was no data on the last UNV_DAYS charging days
    [[nodiscard]] int8_t wf() const {
        const uint8_t l = std::max({effLevel(), rbLvl, (uint8_t) dayWarn()});
        return !l && (!anyFresh() || unverifiedLong()) ? WF_UNVERIFIED : (int8_t) l;
    }

    // [W] excess power of the worst path that produced a value within PW_AGE_S, NAN when none
    [[nodiscard]] float pw() const {
        float p = NAN;
        for (auto *q: {&fast, &rcP, &roP})
            if (fresh(*q) && tS - q->at <= PW_AGE_S && !(q->pw <= p)) p = q->pw;
        return p;
    }

    struct PathView { float dr, pw; bool fresh; };
    [[nodiscard]] PathView fastPath() const { return {fast.dr, fast.pw, fresh(fast)}; }
    [[nodiscard]] PathView rcPath() const { return {rcP.dr, rcP.pw, fresh(rcP)}; }
    [[nodiscard]] PathView roPath() const { return {roP.dr, roP.pw, fresh(roP)}; }
    [[nodiscard]] PathView rbPath() const { return {rbP.dr, rbP.pw, fresh(rbP)}; }
    [[nodiscard]] float baseRc() const { return capRc(bRc); }
    [[nodiscard]] float baseA() const { return bA; }
    [[nodiscard]] float baseRb() const { return bRb; }
    [[nodiscard]] float baseRo() const { return bRo; }
    // offset regression: [V] offset against baseline a, [Ω] slope against baseline Rc; NAN = not separable
    [[nodiscard]] float ofs() const { return ofsA - bA; }
    [[nodiscard]] float slope() const { return ofsR - rcBase(); }
    [[nodiscard]] float iRef() const { return std::max(ioPk[0], ioPk[1]); }
    // enough stored days to grade against: fast and Rc paths (or rMax), Rb path, Ro path
    [[nodiscard]] bool rcRef() const { return rbRef() || conf.rMax > 0; }
    [[nodiscard]] bool rbRef() const { return h.nRc >= BASE_MIN_DAYS; }
    [[nodiscard]] bool roRef() const { return h.nRo >= BASE_MIN_DAYS; }
    [[nodiscard]] uint8_t dayCount() const { return h.days; }
    [[nodiscard]] uint8_t daySamples() const { return d.n; }
    [[nodiscard]] uint8_t daySweeps() const { return d.nRo; }
    [[nodiscard]] float dayCurrentSd() const { // [A] spread of today's frames at Iout >= I_MIN
        const float m = d.i / d.w;
        return std::sqrt(std::fmax(0.f, d.i2 / d.w - m * m));
    }
    // the open day counts toward learning: its slow fit was good, or on a board without BMS data its sweeps
    [[nodiscard]] bool dayCounts() const { return rcGood() || (roGood() && !h.nRc && !d.bms); }

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
        uint32_t cm = 0, at = 0; // charging minute and time of the last value
        bool has = false;
        Hist hist{};
        explicit Path(uint16_t s) : stale(s) {}
    };

    Blob h{VER, 0, 0, 0, 0, 0, 0, 0, NONE, NONE, {}, {}, {}, {}};
    bool dirty = false, roNew = false;
    float bRc = NAN, bA = NAN, bRb = NAN, bRo = NAN;

    std::atomic<float> cap{NAN};
    std::atomic<bool> trip{false};
    uint8_t lvl = Ok, rbLvl = 0, overDerate = 0;
    int16_t roBuf[RO_MED]{}; // last sweeps, 10 µΩ
    uint8_t roDown = Ok;     // release grade of the Ro path, from the highest sweep in roBuf
    uint8_t roK = 0;         // sweeps seen, wraps RO_MED..2·RO_MED-1
    uint32_t belowS = 0;

    Path fast{STALE_FAST}, rcP{STALE_RC}, roP{STALE_RO}, rbP{STALE_RC};
    float rcIn = NAN, aIn = NAN, rbIn = NAN;
    uint32_t roSeen = 0, rbBelowS = 0;

    // frame sums of this minute, and their forgetting sum over minutes: 1, i, i², dv, i·dv, ib, i·ib
    float ms[7]{};
    double rg[7]{};
    float ofsA = NAN, ofsR = NAN; // [V], [Ω] last separable intercept and slope of dv − bRb·ib over Iout
    float iSum = 0, ibMinute = 0;
    uint16_t iN = 0;
    uint32_t winS = 0, cm = 0, tS = 0;
    float ioPk[2]{}, ibPk[2]{}; // peak minute current: current / previous charging hour
    uint8_t pkMin = 0;

    // times in d are ages until the first tick
    static constexpr Day newDay(uint32_t lastS, uint32_t startS) {
        return {{}, {}, {}, {}, {}, 0, 0, 0, DAY_SAMPLE_S, lastS, startS, 0, 0, 0, 0, 0, 0, 1, 0, false, false};
    }
    Day d = newDay(DAY_SAMPLE_S, 0);
    bool started = false, coldDay = true;
    uint32_t runS = 0;

    static uint32_t sum(const void *p, size_t n, uint32_t h = 0x811c9dc5) {
        for (size_t i = 0; i + 4 <= n; i += 4) {
            uint32_t x;
            memcpy(&x, (const uint8_t *) p + i, 4);
            h = (h ^ x) * 16777619u;
        }
        return h;
    }

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

    [[nodiscard]] float capRc(float b) const {
        if (conf.rMax > 0) return std::isfinite(b) ? std::min(b, conf.rMax) : conf.rMax;
        return b;
    }
    [[nodiscard]] float rcBase() const { return capRc(rbRef() ? bRc : NAN); }
    [[nodiscard]] float roBase() const { return roRef() ? bRo : NAN; }

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

    void daySample() {
        if (d.n == DAY_N) {
            compact(d.rc, d.n); compact(d.a, d.n); compact(d.rb, d.n);
            d.n /= 2;
            d.step *= 2;
        }
        d.rc[d.n] = q(rcIn, 1e5f);
        d.a[d.n] = q(aIn, 1e4f);
        d.rb[d.n] = q(rbIn, 1e5f);
        ++d.n;
    }

    void onRo(float ro) {
        const int16_t v = q(ro, 1e5f);
        if (d.nRo == DAY_N) { compact(d.ro, d.nRo); d.nRo /= 2; }
        d.ro[d.nRo++] = v;
        roBuf[roK % RO_MED] = v;
        if (++roK == 2 * RO_MED) roK = RO_MED;
        roNew = true;
    }

    static void peakAdd(float *pk, float x) { pk[0] = std::max(pk[0], x); }

    void slow(Path &p, float dr, float iref) {
        if (!std::isfinite(dr) || iref < I_MIN) return;
        p.dr = dr;
        p.pw = dr * iref * iref;
        p.cm = cm;
        p.at = tS;
        p.has = true;
        p.hist.push(gradeOf(p.pw, p.dr, 1));
    }

    // intercept and slope of (dv − bRb·ib) over Iout; false while the currents don't separate them
    [[nodiscard]] bool ofsFit(float &a, float &r) const {
        const double w = rg[0];
        if (!(w >= OFS_W_MIN) || !std::isfinite(bRb)) return false;
        const double mi = rg[1] / w, var = rg[2] / w - mi * mi;
        if (!(var >= OFS_VAR_MIN)) return false;
        const double my = (rg[3] - bRb * rg[5]) / w, miy = (rg[4] - bRb * rg[6]) / w;
        r = (float) ((miy - mi * my) / var);
        a = (float) (my - r * mi);
        return std::fabs(a - bA) <= OFS_MAX;
    }

    void dayDr(float dr) {
        if (++d.drSkip < d.drStep) return;
        d.drSkip = 0;
        if (d.nDr == DAY_N) { compact(d.dr, d.nDr); d.nDr /= 2; d.drStep *= 2; }
        d.dr[d.nDr++] = q(dr, 1e5f);
    }

    void minute(uint32_t nowS) {
        winS = nowS;
        const float im = iN ? iSum / iN : NAN;
        iSum = 0;
        iN = 0;
        if (im >= I_MIN) {
            ++cm;
            ++d.chargeMin;
            peakAdd(ioPk, im);
            peakAdd(ibPk, ibMinute);
            if (++pkMin >= 60) {
                pkMin = 0;
                ioPk[1] = ioPk[0]; ioPk[0] = 0;
                ibPk[1] = ibPk[0]; ibPk[0] = 0;
            }
        }
        ibMinute = 0;

        const float n = ms[0];
        const bool op = n >= MIN_FRAMES;
        const float i = op ? ms[1] / n : NAN, ib = op ? ms[5] / n : NAN;
        float a = NAN, r = NAN;
        const bool sep = ofsFit(a, r);
        ofsA = sep ? a : NAN;
        ofsR = sep ? r : NAN;
        // [V] excess drop of this minute: the offset comes from the regression, its slope stays in
        const float y = op && sep ? (ms[3] - bRb * ms[5]) / n - a : NAN, e = y - rcBase() * i;
        if (std::isfinite(e)) dayDr(e / i);
        if (std::isfinite(e) && (paths & 1)) {
            fast.dr = e / i;
            fast.pw = e * i;
            fast.cm = cm;
            fast.at = nowS;
            fast.has = true;
            d.fast = true;
            fast.hist.push(gradeOf(fast.pw, fast.dr, 1));
            ++d.fastN;
            if (fast.dr >= 0.5f * conf.drMin) ++d.fastHi;
            overDerate = armed() && lvl == Derate && fast.pw >= conf.pDerate * OVER_DERATE ? overDerate + 1 : 0;
        }
        // a minute off the regression line by pWarn/2 is a step, not drift: it must not pull the line
        // along (a fault step would tilt it and inflate or hide itself), slower changes are learned.
        // Nothing is learned while a level is held, so a found fault can't fade into the offset.
        if (op && lvl == Ok && !(std::fabs(y - r * i) * i >= 0.5f * conf.pWarn)) {
            constexpr double l = 1. - 1. / OFS_TAU;
            for (uint8_t k = 0; k < 7; ++k) rg[k] = rg[k] * l + ms[k];
        }
        std::fill(ms, ms + 7, 0.f);

        // a sweep before the first charging minute (boot sweep) waits for a reference current
        if (roNew && roK >= RO_MED_MIN && (!roP.has || cm - roP.cm >= RO_GAP) &&
            (!roRef() || iRef() >= I_MIN)) {
            roNew = false;
            const uint8_t n = std::min(roK, RO_MED);
            if (paths & 4) slow(roP, median(roBuf, n) / 1e5f - roBase(), iRef());
            const float hi = *std::max_element(roBuf, roBuf + n) / 1e5f - roBase();
            roDown = gradeOf(hi * iRef() * iRef(), hi, HYST);
        }
        if (op && sep && (paths & 2)) slow(rcP, r - rcBase(), iRef());
        // Rb only up to what the excess drop supports, so an R1/R2 trade-off alone can't warn
        if (std::isfinite(rcIn) && std::isfinite(e) && rbRef() && std::fabs(ib) >= I_MIN)
            slow(rbP, std::fmin(rbIn - bRb, e / std::fabs(ib)), std::max(ibPk[0], ibPk[1]));

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
                if (fresh(*p)) {
                    any = true;
                    down = std::max(down, p == &roP ? roDown : gradeOf(p->pw, p->dr, HYST));
                }
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

    [[nodiscard]] bool rcGood() const {
        if (d.n < DAY_RC_N || !(d.w >= DAY_W_MIN)) return false;
        const float m = d.i / d.w;
        return d.i2 / d.w - m * m >= DAY_VAR_MIN;
    }
    [[nodiscard]] bool roGood() const { return d.nRo >= DAY_RO_N; }

    void closeDay(uint32_t nowS) {
        const bool learn = learning();
        // current-independent check on the day's median excess resistance: the fast path's, else Ro.
        // Not while learning, the baseline may still be one odd day.
        const float dr = learn ? NAN
                         : d.nDr >= 4 ? median(d.dr, d.nDr) / 1e5f
                         : d.nRo >= 2 ? median(d.ro, d.nRo) / 1e5f - roBase() : NAN;
        if (dr >= 2 * conf.drMin) h.flags |= 1;
        else if (dr < 2 * conf.drMin * HYST) h.flags &= ~1;
        // while learning every good day enters, the median of them outvotes an odd first day.
        // After learning the fit can absorb a fault into its offset or Rb, so the fast path vetoes too.
        const bool rc = rcGood(), ro = roGood(), cnt = dayCounts();
        if (rc) {
            const float m = median(d.rc, d.n) / 1e5f;
            if (learn || (!(m > bRc + conf.drMin) && accept(m, h.ancRc) && 2 * d.fastHi <= d.fastN)) {
                h.rc[h.hRc] = q(m, 1e5f);
                h.a[h.hRc] = q(median(d.a, d.n) / 1e4f, 1e4f);
                h.rb[h.hRc] = q(median(d.rb, d.n) / 1e5f, 1e5f);
                h.hRc = (h.hRc + 1) % DAYS;
                if (h.nRc < DAYS) ++h.nRc;
            }
        }
        if (ro) {
            const float m = median(d.ro, d.nRo) / 1e5f;
            if (learn || (!(m > bRo + conf.drMin) && accept(m, h.ancRo))) {
                h.ro[h.hRo] = q(m, 1e5f);
                h.hRo = (h.hRo + 1) % DAYS;
                if (h.nRo < DAYS) ++h.nRo;
            }
        }
        if (cnt && h.days < 255) ++h.days;
        if (d.n >= DAY_RC_N || ro || d.fast) h.unvDays = 0;
        else if (d.chargeMin >= UNV_CHARGE_MIN && h.unvDays < 255) ++h.unvDays;
        d = newDay(d.lastS, nowS);
        coldDay = false;
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
