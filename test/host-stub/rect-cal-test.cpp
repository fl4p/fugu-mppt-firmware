// Host test for src/rect_cal.h: rect_offset self-calibration, its RT/core-0 glue (RectCalCtl) and the
// runtime backstop, driven with the real SynchronousConverter (MOCK driver at the MCPWM resolution,
// 4103 counts) against the vconv cycle model with a hidden LS gate delay, LS body-diode drop, biased
// Vout reading, sensor noise and irradiance drift. Host mirrors MpptController's per-sample order:
// rectCal.rt() -> protect() (reverse-current branches + the open-backflow high-duty guard, which uses
// getDutyCycle() = pwmCtrl/pwmCtrlMax as in production) -> rectCal.hold() -> tracker; lf() as loopLF.
//
// Build & run (repo root):
//   clang++ -std=gnu++20 -O2 -fexceptions -DMOCK=1 -I test/host-stub -I src \
//       test/host-stub/rect-cal-test.cpp src/sim/vconv.cpp -o /tmp/rect-cal-test && /tmp/rect-cal-test
// Address/undefined-behaviour sanitizers: the same with -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=undefined
// Cross-core hand-off under ThreadSanitizer:
//   clang++ -std=gnu++20 -O1 -g -fsanitize=thread -fexceptions -DMOCK=1 -I test/host-stub -I src \
//       test/host-stub/rect-cal-test.cpp src/sim/vconv.cpp -o /tmp/rect-cal-tsan && /tmp/rect-cal-tsan threads

#include <cstdarg>
#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "esp_log.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define unlikely(x) (x)
#define OUTPUT 1
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t, int) {}

#include "../../src/buck.h"
#include "../../src/rect_cal.h"
#include "../../src/sim/vconv.h"
#include "rect_cal_plant.h"

void assertPinState(uint8_t, bool, const char *, bool) {}
static bool g_quiet = true, g_trace = false, g_verbose = false;
static float g_inoise = 0.01f; // Iout sensor noise rms (A)
void UART_LOG(const char *fmt, ...) {
    if (g_quiet) return;
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap); putchar('\n');
}

static int g_fail = 0;

// largest Iout rise before ideal (RectCal::rise) per test family, against its refusal bound
struct RiseLog { std::string name; float rise, frac; int runs, refused; };
static std::vector<RiseLog> g_rise;
static void noteRise(const char *what, const RectCal &c) {
    if (!(c.iLs > 0)) return;
    auto it = std::find_if(g_rise.begin(), g_rise.end(), [&](const RiseLog &l) { return l.name == what; });
    if (it == g_rise.end()) it = g_rise.insert(g_rise.end(), {what, -1, -1, 0, 0});
    it->rise = std::max(it->rise, c.rise); it->frac = std::max(it->frac, c.rise / c.iLs); ++it->runs;
    it->refused += c.get() == RectCal::Fail && !strcmp(c.why, "Iout follows LS");
}
#define CHECK(c, ...) do { if (!(c)) { ++g_fail; printf("FAIL %s:%d %s  ", __FILE__, __LINE__, #c); printf(__VA_ARGS__); putchar('\n'); } } while (0)

static constexpr uint16_t PM = 4103;
static constexpr float SPS = 434.f, VOC = 69.4f, K = 0.78f, VBAT = 26.7f, LREAL = 50.9e-6f;
static constexpr uint32_t FSW = 39000;
using Ctl = RectCalCtl<SynchronousConverter>;

// Plant + sensors. The LS conducts `hidden` counts longer than commanded (gate/driver delay).
struct Rig {
    VirtualConverter p{};
    int hidden;
    float voutBias, isc0, drift, voc = VOC, k = K;
    double t = 0;
    std::mt19937 rng{1};
    std::normal_distribution<float> nd{0.f, 1.f};

    Rig(float isc, int hidden, float voutBias, float drift) : hidden(hidden), voutBias(voutBias), isc0(isc), drift(drift) {
        p.setPv(isc, VOC, K); p.setBat(VBAT, 0.03f); p.setPassives(470e-6f, 470e-6f, LREAL);
        p.setVin(VOC); p.setVout(VBAT); p.setDiodeVf(0.7f);
    }
    void apply(uint16_t hs, uint16_t ls) {
        int eff = (hs && ls) ? std::min<int>(ls + hidden, PM - hs - 1) : 0;
        p.setPwm({PM, hs, (uint16_t) std::max(0, eff), FSW});
    }
    void step() { t += 1.0 / SPS; p.setPv(isc0 * (1.f + drift * (float) t), voc, k); p.stepSeconds(1.f / SPS, FSW); }
    uint32_t us() const { return (uint32_t) (uint64_t) (t * 1e6); } // wraps like the firmware clock
    float iout() { return p.getIoutAvg() + g_inoise * nd(rng); }
    float vin() { return p.getVin() + 0.05f * nd(rng); }
    float vout() { return p.getVout() + voutBias + 0.01f * nd(rng); }
};

static const char *g_L0 = "53.6e-6"; // coil.conf L0: the model uses 0.95 L0 = the plant's 50.9 uH
static void initConv(SynchronousConverter &c, const char *offNs = nullptr, const char *offDrv = nullptr,
                     const char *offCal = nullptr) {
    ConfFile conv{}; conv.set("topo", "buck");
    ConfFile board{}; board.set("pwm_freq", "39000"); board.set("pwm_driver_logic", "HiLi");
    board.set("pwm_hi", "21"); board.set("pwm_li", "14"); board.set("pwm_sd", "47"); board.set("skip_assert", "1");
    ConfFile coil{}; coil.set("L0", g_L0);
    if (offNs) coil.set("rect_offset_ns", offNs);
    if (offDrv) coil.set("rect_offset_drv", offDrv);
    if (offCal) coil.set("rect_offset_cal", offCal);
    c.init(conv, board, coil);
}

static bool g_saveOk = true;
static int g_saveCalls = 0;
static std::vector<std::pair<int, int>> g_saved; // offset, rect_offset_cal
// Like rectCalPersist: one coil.conf rewrite. g_writeHook(0) runs before it, g_writeHook(1) after it,
// standing in for RT activity while core 0 is inside the write (start attempts, rt() calls, delays).
static Ctl *g_ctl = nullptr;
static SynchronousConverter *g_conv = nullptr;
static std::function<void(int)> g_writeHook;
static int g_writes = 0, g_poweredWrites = 0, g_unadmitted = 0;
static bool fakePersist(int o, uint8_t cal) {
    ++g_saveCalls;
    if (g_writeHook) g_writeHook(0);
    ++g_writes;
    if (g_ctl && !g_ctl->persistLocked()) ++g_unadmitted;
    if (g_conv && !g_conv->disabled()) ++g_poweredWrites;
    if (g_writeHook) g_writeHook(1);
    if (g_conv && !g_conv->disabled()) ++g_poweredWrites;
    if (!g_saveOk) return false;
    g_saved.emplace_back(o, cal);
    return true;
}

// MpptController wiring around Ctl, per sample: rt() -> protect() -> hold() -> tracker / update() tail.
struct Host {
    Rig r;
    SynchronousConverter c{};
    Ctl own{};
    Ctl &ctl;
    float iEwm = 0, viEwm = VOC, voEwm = VBAT, last = 0, prev = 0, vi = VOC, vo = VBAT, io = 0, vinMax = 90.f;
    bool bflow = false, sr = false, mppt = true, sweeping = false, tripped = false, held = false, lim = false;
    uint16_t op = 0, rampTo = 0;
    int lsAfterHold = -1, lsRaw = -1; // pinned LS / LS count right after hold()
    int guard = 0; // samples on which the reverse-current guard acted
    double lfAt = 0;
    std::function<void(Host &)> fault; // runs inside protect(), before hold()

    Host(float isc, int hidden, float vb, float drift, int offset = 0, Ctl *shared = nullptr)
        : r(isc, hidden, vb, drift), ctl(shared ? *shared : own) {
        initConv(c);
        c.setRectOnOffset(offset);
        if (!shared) {
            ctl.init(offset, true);
            ctl.persist = fakePersist;
            ctl.autoRun = false;
        }
    }
    RectCalIn in() const {
        return {r.us(), vi, vo, io, viEwm, voEwm, iEwm, iEwm, std::min(iEwm, std::max(last, prev)), mppt, sweeping,
                bflow, lim, true, vinMax};
    }
    float pOut() const { return iEwm * voEwm; }
    void tick() {
        if (tripped) return;
        r.apply(c.getCtrlOnPwmCnt(), c.getRectOnPwmCnt()); r.step();
        vi = r.vin(); vo = r.vout(); io = r.iout();
        prev = last; last = io;
        const float a = 2.f / 21.f;
        iEwm += a * (io - iEwm); viEwm += a * (vi - viEwm); voEwm += a * (vo - voEwm);
        c.updateSyncRectMaxDuty(vi, vo, iEwm);
        if (uint16_t b = ctl.rt(c, in())) rampTo = b;

        const float iF = std::min(iEwm, std::max(last, prev));
        if ((last < -1 && prev < -1) || (sr && iF < -0.05f)) { bflow = false; c.syncRectMinDuty(); ++guard; }
        if (fault) fault(*this);
        if (sr && !bflow && c.getDutyCycle() > 0.33f && !(c.dcmCurrent(viEwm, voEwm) < 1.f)) { tripped = true; c.disable(); return; }

        uint16_t b = 0;
        held = ctl.hold(c, in(), b);
        lsAfterHold = c.manualRect_() ? c.getRectOnPwmCnt() : -1;
        lsRaw = c.getRectOnPwmCnt();
        if (!held) {
            if (b) rampTo = b;
            if (sweeping) c.pwmPerturbFractional(340.f / SPS);
            else {
                const int d = std::clamp((rampTo ? rampTo : op) - (int) c.getCtrlOnPwmCnt(), -(int) PM / 512, (int) PM / 512);
                if (d) c.pwmPerturb((int16_t) d);
                else rampTo = 0;
            }
            if (!sr && iEwm > 0.2f) { c.enableSyncRect(true); sr = true; }
            if (sr) bflow = !(iEwm < -0.05f);
        }
        if (r.t >= lfAt) {
            lfAt = r.t + 0.1;
            ctl.lf(c, (uint32_t) r.t, pOut());
        }
    }
    void run(double s) { for (double t1 = r.t + s; r.t < t1 && !tripped;) tick(); }
    RectCal::State state() const { return ctl.cal.get(); }
};

// Settle at opFrac*M in diode emulation, start a run through requestStart() and drive it to Done/Fail.
// truth: the offset the run should learn, plant zero crossing minus the hidden delay minus the ideal
// count the firmware computes from its (biased) Vout reading, at the calibration HS when the sweep ends.
static bool calibrate(Host &h, float &truth, float opFrac = 0.6f) {
    h.op = (uint16_t) (opFrac * VBAT / VOC * PM);
    h.run(6);
    CHECK(h.c.inDCM(), "not in DCM before cal");
    const uint16_t hsOp = h.c.getCtrlOnPwmCnt();
    CHECK(h.ctl.requestStart(), "requestStart");
    truth = NAN;
    for (double t1 = h.r.t + 60; h.r.t < t1;) {
        const auto s0 = h.state();
        h.tick();
        if ((s0 == RectCal::Run || s0 == RectCal::Idle) && h.state() != RectCal::Run && h.state() != RectCal::Idle)
            truth = h.r.p.getVin() * (1.f / h.r.p.getVout() - 1.f / (h.r.p.getVout() + h.r.voutBias)) * (float) h.ctl.cal.hs
                    - (float) h.r.hidden;
        if (h.state() == RectCal::Done || h.state() == RectCal::Fail) break;
    }
    const bool ok = h.state() == RectCal::Done;
    h.run(2);
    CHECK(!h.c.manualRect_(), "LS still pinned after the run");
    CHECK(std::abs((int) h.c.getCtrlOnPwmCnt() - (int) hsOp) <= 8, "HS %u not restored to %u (%s)", h.c.getCtrlOnPwmCnt(),
          hsOp, ok ? "done" : h.ctl.cal.why);
    if (ok)
        CHECK(h.c.getRectOnOffset() == h.ctl.cal.applied && h.ctl.offset() == h.ctl.cal.applied && h.ctl.activeCalibrated(),
              "offset not applied: %d / %d want %d", h.c.getRectOnOffset(), h.ctl.offset(), h.ctl.cal.applied);
    return ok;
}

// Global MPPT sweep in diode emulation (duty 0 -> past D=M). Returns true when the highD-bflowOpen
// guard trips. withBackstop=false runs the sweep as non-MPPT mode (backstop disarmed).
static bool sweepTrips(float isc, int &offset, int hidden, float voutBias, bool withBackstop, int &fires) {
    Host h(isc, hidden, voutBias, 0.f, offset);
    h.sweeping = true;
    h.mppt = withBackstop;
    while (h.r.t < 8 && !h.tripped) {
        h.tick();
        if (g_trace && ((int) (h.r.t * SPS) % 10 == 0 || h.iEwm < 0.5f))
            printf("t=%.3f hs=%u ls=%u hsZero=%.0f i=%.3f vin=%.2f dcm=%d sr=%d off=%d\n", h.r.t, h.c.getCtrlOnPwmCnt(),
                   h.c.getRectOnPwmCnt(), h.vo / h.vi * PM, h.iEwm, h.vi, h.c.inDCM(), h.c.syncRectEnabled_(),
                   h.c.getRectOnOffset());
        if ((h.c.getCtrlOnPwmCnt() / (float) PM) > 1.08f * (h.r.p.getVout() / h.r.p.getVin())) break;
    }
    fires = h.ctl.fires.load();
    offset = h.c.getRectOnOffset();
    return h.tripped;
}

// Near D = M a low Vout reading eats the WCE ratio margin; a late offset must then not reach the complementary clamp
// (forced CCM below D = M): it keeps the LS off for T/32 before the next HS, else falls back to the offset-0 window.
static void testLateGap() {
    const float vi = 60.f, vo = 30.f * 0.983f; // Vout read 1.7 % low, true M = 0.5
    const int ctMax = pwmCountsFromNs(RectOffsetLateMaxNs, 39000.f * PM), gap = PM >> 5;
    auto lsAt = [&](int off, int hs) {
        SynchronousConverter c{}; initConv(c);
        c.setRectOnOffset(off);
        c.enableSyncRect(true);
        c.pwmPerturb((int16_t) hs);
        for (int k = 0; k < 400; ++k) { c.updateSyncRectMaxDuty(vi, vo, 1.f); c.pwmPerturb(0); }
        return (int) c.getRectOnPwmCnt();
    };
    int n = 0, clamp = 0, tight = 0, minGap = PM;
    for (int hs = PM / 2 - 150; hs <= PM / 2; hs += 3) {
        const int ls0 = lsAt(0, hs), ls = lsAt(ctMax, hs), comp = PM - hs - 1;
        ++n;
        if (ls >= comp && ls0 < comp) ++clamp;
        if (ls > ls0 + 1 && comp - ls < gap - 1) ++tight;
        if (ls > ls0 + 1) minGap = std::min(minGap, comp - ls);
    }
    CHECK(clamp == 0 && tight == 0, "late gap: %d of %d points into the clamp, %d closer than T/32 (min gap %d ct)", clamp, n,
          tight, minGap);
    printf("late gap: %d points, %d into the complementary clamp, min idle after a late LS %d ct (T/32 = %d)\n", n, clamp,
           minGap, gap);
}

static void testDriverRule() {
    SynchronousConverter c{};
    const int ct875 = (int) std::lround(875e-9 * 39000.0 * PM);
    initConv(c, "875", "mcpwm");
    CHECK(c.getRectOnOffset() == 0 && c.rectOffsetNeedsCal(), "foreign driver: off=%d", c.getRectOnOffset());
    initConv(c, "875", nullptr);
    CHECK(c.getRectOnOffset() == 0 && c.rectOffsetNeedsCal(), "no driver key: off=%d", c.getRectOnOffset());
    const int ct100 = (int) std::lround(100e-9 * 39000.0 * PM), ctMax = pwmCountsFromNs(RectOffsetLateMaxNs, 39000.f * PM);
    initConv(c, "100", "mock");
    CHECK(std::abs(c.getRectOnOffset() - ct100) <= 1 && c.rectOffsetCal() == 1, "same driver: off=%d want %d",
          c.getRectOnOffset(), ct100);
    initConv(c, "875", "mock"); // past the late limit: loaded as a bound there
    CHECK(c.getRectOnOffset() == ctMax && c.rectOffsetCal() == 2, "late 875 ns: off=%d level %u want %d level 2",
          c.getRectOnOffset(), c.rectOffsetCal(), ctMax);
    initConv(c, "-500", "mock");
    CHECK(c.getRectOnOffset() < -70 && !c.rectOffsetNeedsCal(), "negative offset: off=%d", c.getRectOnOffset());
    initConv(c, "-500", "mock", "0");
    CHECK(c.getRectOnOffset() < -70 && c.rectOffsetNeedsCal(), "backstop value: off=%d needs=%d", c.getRectOnOffset(),
          c.rectOffsetNeedsCal());
    initConv(c);
    CHECK(c.getRectOnOffset() == 0 && c.rectOffsetNeedsCal(), "never calibrated");
    printf("driver rule: ok\n");
}

// Finding 3: shrink() never lengthens LS; fit()'s lowest applied value is the backstop floor.
// fry field runs (160 MHz), points in run order. 113 W: coarse, then the fine pass; full sun 446/300 W: the
// old sweep ran to the cap, the current turns back up past the bend (ring-back). bend: last point before the
// visible drop, the fit must not learn later than it.
struct FieldRun { const char *name; uint16_t hs; float ideal, bend; std::vector<float> xs, ys; };
static const FieldRun g_fieldRuns[] = {
    {"113 W", 905, 1732, 1646,
     {866, 933, 1001, 1068, 1136, 1204, 1272, 1340, 1406, 1474, 1542, 1611, 1676, 1743, 1811,
      1486, 1512, 1539, 1566, 1593, 1619, 1646, 1673, 1701, 1726},
     {.002f, -.001f, .001f, 0, .001f, .001f, .001f, -.001f, -.002f, -.003f, -.005f, -.004f, -.012f, -.046f, -.058f,
      -.002f, -.002f, -.005f, -.002f, -.005f, -.004f, -.005f, -.011f, -.020f, -.031f}},
    {"446 W", 896, 1773, 1651,
     {885, 955, 1021, 1090, 1160, 1230, 1300, 1371, 1441, 1512, 1582, 1651, 1722, 1793, 1865, 1939, 2014, 2089, 2164},
     {.001f, -.001f, -.001f, -.000f, -.002f, -.002f, -.002f, -.003f, -.003f, -.004f, -.004f, -.004f, -.012f, -.049f,
      -.055f, .008f, .102f, .179f, .199f}},
    {"300 W", 912, 1746, 1640,
     {873, 941, 1008, 1077, 1147, 1217, 1290, 1360, 1431, 1501, 1570, 1640, 1709, 1779, 1850, 1921, 1994, 2064, 2132},
     {.001f, -.000f, .002f, -.000f, .001f, -.000f, .001f, .000f, -.001f, -.003f, -.004f, -.002f, -.015f, -.066f, -.072f,
      -.013f, .080f, .166f, .201f}},
};

// The recorded points through fit(), then the run replayed through feed() against the piecewise-linear curve of
// those points (+-noise A, uniform): the sweep stops one coarse step past the first clear drop, does the fine
// pass and learns no later than the visible bend.
static void testFieldData() {
    for (const auto &f: g_fieldRuns) {
        RectCal c;
        c.n = (uint8_t) f.xs.size();
        std::copy(f.xs.begin(), f.xs.end(), c.xs);
        std::copy(f.ys.begin(), f.ys.end(), c.ys);
        c.ideal = f.ideal;
        c.state.store(RectCal::Fit);
        c.fit(160e6f);
        // two-sided: the fit learns 74-90 ct before the last flat point (the onset of the bend)
        CHECK(c.get() == RectCal::Done && c.peak <= f.bend && c.peak >= f.bend - 100, "field %s recorded: '%s' peak %.0f",
              f.name, c.why, c.peak);
        if (f.ideal == 1732) CHECK(c.learned <= -158, "field %s recorded: learned %d, was -160", f.name, c.learned);
        printf("field %s recorded (%u pts): %s%s peak %.0f learned %+d applied %+d\n", f.name, c.n,
               c.get() == RectCal::Done ? "done" : "failed: ", c.why, c.peak, c.learned, c.applied);
        std::vector<std::pair<float, float>> pts;
        for (size_t j = 0; j < f.xs.size(); ++j) pts.emplace_back(f.xs[j], f.ys[j]);
        std::sort(pts.begin(), pts.end());
        auto curve = [&](float x) {
            if (x <= pts.front().first) return pts.front().second;
            for (size_t j = 1; j < pts.size(); ++j)
                if (x <= pts[j].first)
                    return pts[j - 1].second + (pts[j].second - pts[j - 1].second) * (x - pts[j - 1].first) / (pts[j].first - pts[j - 1].first);
            return pts.back().second;
        };
        float pLo = INFINITY, pHi = -INFINITY;
        for (int seed = 0; seed <= 8; ++seed) {
            const float noise = seed ? 0.003f : 0.f;
            std::mt19937 rng(seed);
            std::uniform_real_distribution<float> u(-noise, noise);
            const float vo = 26.7f, vi = vo * (1.f + f.ideal / (float) f.hs);
            RectCal r;
            r.start(0, f.hs, 16, PM - f.hs - 1);
            int ls = r.lsHold(), lsMax = 0, lsPrev = -1;
            float e = 0;
            for (uint32_t us = 0; r.running() && us < 60000000; us += 2304) {
                if (ls != lsPrev) e = u(rng), lsPrev = ls; // one noise value per window
                lsMax = std::max(lsMax, ls);
                const float i = 2.f + (ls == 16 ? 0.f : curve((float) ls) + e);
                ls = r.feed(us, f.hs, vi, vo, i, true);
            }
            r.fit(160e6f);
            noteRise("76 uH field replays", r);
            CHECK(r.get() == RectCal::Done && r.peak <= f.bend + 10 && r.peak >= f.bend - 100 && lsMax <= f.ideal + 120 && r.n >= 20,
                  "field %s replay seed %d: '%s' peak %.0f, LS max ideal%+.0f, %u pts", f.name, seed, r.why, r.peak,
                  lsMax - f.ideal, r.n);
            if (seed <= 2)
                printf("field %s replay seed %d: %s%s peak %.0f learned %+d, LS max ideal%+.0f, %u pts\n", f.name, seed,
                       r.get() == RectCal::Done ? "done" : "failed: ", r.why, r.peak, r.learned, lsMax - f.ideal, r.n);
            pLo = std::min(pLo, r.peak), pHi = std::max(pHi, r.peak);
        }
        printf("field %s replay seeds 0-8: peak bend%+.0f..%+.0f\n", f.name, pLo - f.bend, pHi - f.bend);
    }
}

// 51 uH board (1470 sps), 2026-11-15: a shared INA226 without an input filter. Its Iout and Vout readings both
// follow the LS window: Iout rises steadily with LS and never drops, Vout jumps past ideal on every retry at that
// LS. Runs 1 and 2 ended with "V step" right at ideal (Vout +0.40/+0.26 %, Vin -0.12/-0.10 % from ideal-10 on),
// run 3 rose to +63 mA at ideal+82 and failed at 1797 (Vout +1.28 %). Replay: recorded points (LS, dI, Vin/Vout
// off their references, 0.01 %), flat past the last one, plus a lossless 51 uH knee 45 ct past ideal. The voltage
// offsets are reading-only. Each run must be refused on the sense path, never fitted; "Iout only" (Vout clean)
// and "Vout only" (Iout flat) isolate the two rules.
struct FlatRun { const char *name; uint16_t hs; float vi, vo, dvi, dvo; std::vector<float> ls, y, lsV, jvi, jvo; };
static const FlatRun g_flatRuns[] = {
    {"run 1", 1026, 67.18f, 25.95f, -0.0012f, 0.0040f,
     {818, 882, 946, 1010, 1074, 1139, 1201, 1266, 1329, 1395, 1458, 1521, 1585},
     {-.014f, -.011f, 0, -.001f, .008f, .011f, .015f, .015f, .021f, .030f, .031f, .036f, .036f},
     {818, 882, 946, 1010, 1074, 1139, 1201, 1266, 1329, 1395, 1458, 1521, 1585},
     {1, 1, 1, 1, 0, -2, -1, -2, -4, -4, -4, -4, -5}, {-2, -3, -1, 0, 2, 3, -2, 2, 3, 9, 3, 1, 2}},
    {"run 2", 1025, 67.30f, 26.00f, -0.0010f, 0.0026f,
     {817, 880, 946, 1008, 1074, 1137, 1200, 1265, 1328, 1394, 1457, 1521, 1583},
     {-.016f, -.011f, .001f, .001f, .009f, .011f, .016f, .015f, .021f, .029f, .031f, .036f, .036f},
     {817, 880, 946, 1008, 1074, 1137, 1200, 1265, 1328, 1394, 1457, 1521, 1583},
     {2, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -5}, {-2, -4, -1, -2, 3, 1, -2, 2, 2, 8, 3, 1, -1}},
    {"run 3", 1026, 67.1f, 25.73f, NAN, NAN,
     {829, 895, 959, 1023, 1088, 1155, 1220, 1284, 1349, 1415, 1481, 1546, 1607, 1670, 1735},
     {-.012f, -.009f, -.002f, .003f, -.002f, .011f, .011f, .018f, .019f, .016f, .026f, .023f, .024f, .045f, .063f},
     {829, 895, 959, 1023, 1088, 1155, 1220, 1284, 1349, 1415, 1481, 1546, 1607, 1670, 1735, 1797},
     {}, {-4, 6, 0, -1, 0, 5, 13, 6, 5, 13, 18, 19, 24, 32, 58, 128}},
};

static float interp(const std::vector<float> &xs, const std::vector<float> &ys, float x) {
    if (ys.empty()) return 0;
    if (x <= xs.front()) return ys.front();
    for (size_t j = 1; j < xs.size(); ++j)
        if (x <= xs[j]) return ys[j - 1] + (ys[j] - ys[j - 1]) * (x - xs[j - 1]) / (xs[j] - xs[j - 1]);
    return ys.back();
}

static void testFlatField() {
    for (const auto &f: g_flatRuns)
        for (int mode = 0; mode < 3; ++mode) { // 0: as recorded, 1: Iout only, 2: Vout only
            const float ideal = (f.vi / f.vo - 1.f) * (float) f.hs, art = ideal - 10;
            const char *want = mode == 2 ? "sense follows LS" : "Iout follows LS";
            int iRef = 0, vRef = 0, lsMax = 0, lsLo = 9999, lsHi = 0;
            for (uint32_t seed = 0; seed <= 8; ++seed) {
                std::mt19937 rng(seed);
                std::uniform_real_distribution<float> u(-1.f, 1.f);
                std::normal_distribution<float> nd(0.f, 1.f);
                RectCal r;
                r.start(0, f.hs, 16, PM - f.hs - 1);
                int ls = r.lsHold(), lsPrev = -1, ref = -2, runMax = 0;
                float e = 0, eo = 0;
                for (uint32_t us = 0; r.running() && us < 60000000; us += 680) {
                    if (ls != lsPrev) e = (seed ? 0.003f : 0.f) * u(rng), eo = 0.0003f * u(rng), lsPrev = ls;
                    if (ref == -2 && ls != 16) ref = ls; // the first held LS after Init is the reference
                    const bool step = ls != 16 && ls != ref;
                    runMax = std::max(runMax, ls);
                    const float x = (float) ls, d = x - ideal - 45;
                    float y = ls == 16 || mode == 2 ? 0.f : interp(f.ls, f.y, x);
                    if (d > 0) y -= 3.2e-7f * d * d;
                    float vi = f.vi, vo = f.vo;
                    if (step && mode != 1) {
                        const bool jump = !std::isnan(f.dvo) && x >= art;
                        vi *= 1.f + (jump ? f.dvi : 1e-4f * interp(f.lsV, f.jvi, x));
                        vo *= 1.f + (jump ? f.dvo : 1e-4f * interp(f.lsV, f.jvo, x)) + eo;
                    }
                    ls = r.feed(us, f.hs, vi + 0.03f * nd(rng), vo + 0.005f * nd(rng), 1.6f + y + e + 0.01f * nd(rng), true);
                }
                r.fit(160e6f);
                noteRise("51 uH field replays (refused)", r);
                const bool iRf = r.get() == RectCal::Fail && !strcmp(r.why, "Iout follows LS");
                const bool vRf = r.get() == RectCal::Fail && !strcmp(r.why, "sense follows LS");
                iRef += iRf, vRef += vRf, lsMax = std::max(lsMax, runMax);
                lsLo = std::min(lsLo, (int) r.jls), lsHi = std::max(lsHi, (int) r.jls);
                CHECK(mode ? !strcmp(r.why, want) : iRf || vRf, "flat %s mode %d seed %u: %s '%s' learned %d, %u pts",
                      f.name, mode, seed, r.get() == RectCal::Done ? "done" : "failed", r.why, r.learned, r.n);
                if (g_verbose) {
                    for (int i = 0; i < r.n; ++i) printf(" %.0f:%+.3f/%d", r.xs[i], r.ys[i], r.jvo[i]);
                    printf("\n");
                }
            }
            printf("flat %s %-11s: 9 seeds, 'Iout follows LS' %d, 'sense follows LS' %d, last step LS ideal%+d..%+d, "
                   "LS max ideal%+.0f\n", f.name, mode == 0 ? "as recorded" : mode == 1 ? "Iout only" : "Vout only", iRef,
                   vRef, lsLo - (int) ideal, lsHi - (int) ideal, lsMax - ideal);
        }
}

// One coarse window 25 mA low (not repeatable), flat curve until ideal-35, then -0.5 mA/ct. A glitch before the
// knee must not end the sweep: its step back up has it measured again. At the step just before the knee (12) the
// real drop follows and confirms it, the fit learns up to 75 ct early; on the drop step (13) the drop is only deeper.
static void testGlitch() {
    const uint16_t hs = 900;
    const float ideal = 1700, vo = 26.7f, vi = vo * (1.f + ideal / hs);
    int clean = 0;
    for (int g = -1; g < 14; ++g) {
        RectCal r;
        r.start(0, hs, 16, PM - hs - 1);
        int ls = r.lsHold(), step = -1, prev = -1, lsMax = 0;
        for (uint32_t us = 0; r.running() && us < 60000000; us += 2304) {
            if (ls != prev) { prev = ls; if (ls != 16 && ls != (int) (0.6f * ideal)) ++step; }
            lsMax = std::max(lsMax, ls);
            float y = ls > ideal - 35 ? -0.0005f * ((float) ls - (ideal - 35)) : 0.f;
            if (step == g && ls != (int) (0.6f * ideal)) y -= 0.025f;
            ls = r.feed(us, hs, vi, vo, 2.f + (ls == 16 ? 0.f : y), true);
        }
        r.fit(160e6f);
        noteRise("glitch", r);
        const bool ok = r.get() == RectCal::Done;
        if (g < 0) clean = r.learned;
        // step 0 (first point, no drop rule) tilts the baseline: 14 ct later, still before the knee
        CHECK(ok && (g >= 12 ? r.learned >= clean - 80 && r.learned <= clean + 5
                             : g == 0 ? r.learned <= clean + 20 : std::abs(r.learned - clean) <= 5),
              "glitch at step %d: '%s' learned %d (clean %d)", g, r.why, r.learned, clean);
        printf("glitch at coarse step %2d: %s%s learned %+d, LS max ideal%+.0f, %u pts\n", g, ok ? "done" : "failed: ", r.why,
               r.learned, lsMax - ideal, r.n);
    }
}

static void testShrinkMonotone() {
    SynchronousConverter c{}; initConv(c);
    const float tr = c.getPwmTickRate();
    const int step = pwmCountsFromNs(RectBackstop::StepNs, tr), floor = RectCal::floorCounts(tr);
    for (int x = -3000; x <= 3000; ++x) {
        const int y = RectBackstop::shrink(x, step, floor);
        if (y > x) { CHECK(false, "shrink(%d)=%d > input", x, y); break; }
        if (x > floor && !(y < x)) { CHECK(false, "shrink(%d)=%d not reduced above floor %d", x, y, floor); break; }
    }
    CHECK(RectBackstop::shrink(-520, 20, -400) == -520, "reviewer case shrink(-520,20,-400)=%d",
          RectBackstop::shrink(-520, 20, -400));
    printf("shrink: monotone over -3000..3000, floor %d ct, step %d ct\n", floor, step);
}

// Deterministic feed fixture: exact asymmetric current peak at the physical zero crossing of the
// CURRENT Vin/Vout (no timing error), Init at (vi0, vo0), every later phase at (vi1, vo1).
static void fixture(RectCal &cal, float vi0, float vo0, float vi1, float vo1, float timingErr = 0.f) {
    const uint16_t hs = 1200;
    cal.start(0, hs, 16, PM - hs - 1);
    int ls = cal.lsHold();
    bool init = true;
    for (uint32_t us = 0; cal.running() && us < 60000000; us += 2304) {
        if (ls != 16) init = false;
        const float vi = init ? vi0 : vi1, vo = init ? vo0 : vo1;
        const float zc = (vi / vo - 1.f) * hs + timingErr;
        const float x = init ? 0.6f * zc : (float) ls;
        const float i = 2.f - (x < zc ? 9.5e-9f : 6e-6f) * (x - zc) * (x - zc); // asymmetric parabola, physical early side
        ls = cal.feed(us, hs, vi, vo, i, true);
    }
    SynchronousConverter c{}; initConv(c);
    cal.fit(c.getPwmTickRate());
}

// Finding 1: a voltage step after Init must not become a timing correction. Vin is held to StiffV
// (panel stiffness), Vout only to VoutDrift: a shared battery bus moves, and the coordinate follows it.
static void testVoltageBaseline() {
    RectCal a, b, b2, c, d;
    fixture(a, 60, 26.7f, 66, 26.7f);
    CHECK(a.get() == RectCal::Fail && !strcmp(a.why, "Vin shift"), "Vin 60->66: state %d why '%s' learned %d", a.get(),
          a.why, a.learned);
    RectCal c0;
    fixture(c0, 60, 26.7f, 60, 26.7f);
    fixture(b, 60, 26.7f, 60, 27.5f); // +3 %: accepted, coordinate follows the step's own ideal
    CHECK(b.get() == RectCal::Done && std::abs(b.learned - c0.learned) <= 5, "Vout 26.7->27.5: state %d why '%s' learned %d (%d)",
          b.get(), b.why, b.learned, c0.learned);
    fixture(b2, 60, 26.7f, 60, 28.2f); // +5.6 %
    CHECK(b2.get() == RectCal::Fail && !strcmp(b2.why, "Vout shift"), "Vout 26.7->28.2: state %d why '%s'", b2.get(), b2.why);
    fixture(c, 60, 26.7f, 60.9f, 26.7f); // 1.5 %: accepted, coordinate follows the step's own ideal (+40 ct)
    CHECK(c0.get() == RectCal::Done && c.get() == RectCal::Done && std::abs(c.learned - c0.learned) <= 5,
          "Vin +1.5%%: state %d learned %d, without step %d (%s)", c.get(), c.learned, c0.learned, c.why);
    fixture(d, 60, 26.7f, 60, 26.7f, -398.f); // far negative tail
    SynchronousConverter cv{}; initConv(cv);
    const int floor = RectCal::floorCounts(cv.getPwmTickRate());
    CHECK(d.get() == RectCal::Done && d.applied >= floor && RectBackstop::shrink(d.applied, 20, floor) <= d.applied,
          "learned %d applied %d floor %d", d.learned, d.applied, floor);
    // below Vin = 2 Vout a knee 35 ct late (HS-side delay) is applied less the margin, 80 ct late is a bound at
    // LateMaxNs; above, any late result is a bound at 0 (node lag)
    const int hi = pwmCountsFromNs(RectCal::LateMaxNs, cv.getPwmTickRate());
    RectCal late, late35, lateHi;
    fixture(lateHi, 60, 26.7f, 60, 26.7f, 35.f);
    CHECK(lateHi.get() == RectCal::Done && lateHi.learned >= 25 && lateHi.applied == 0 && lateHi.clamped,
          "late knee +35 at 2.25 Vout: state %d learned %d applied %d clamped %d", lateHi.get(), lateHi.learned,
          lateHi.applied, lateHi.clamped);
    fixture(late35, 45, 26.7f, 45, 26.7f, 35.f);
    CHECK(late35.get() == RectCal::Done && late35.learned >= 25 && late35.applied > 0 && late35.applied <= hi
          && !late35.clamped, "late knee +35: state %d learned %d applied %d clamped %d", late35.get(), late35.learned,
          late35.applied, late35.clamped);
    fixture(late, 45, 26.7f, 45, 26.7f, 80.f);
    CHECK(late.get() == RectCal::Done && late.learned >= 60 && late.applied == hi && late.clamped,
          "late knee: state %d learned %d applied %d clamped %d", late.get(), late.learned, late.applied, late.clamped);
    printf("late knee +35: learned %+d applied %+d; +80: learned %+d applied %+d%s\n", late35.learned, late35.applied,
           late.learned, late.applied, late.clamped ? " (clamped)" : "");
    printf("voltage baseline: Vin+10%% '%s', Vout+3%% learned %d, Vout+5.6%% '%s', Vin+1.5%% learned %d (no step %d), "
           "tail learned %d applied %d floor %d\n", a.why, b.learned, b2.why, c.learned, c0.learned, d.learned, d.applied, floor);
}

// DCM fixture with the physical current shape of a flat-like point (56 uH, Vin 60, Vout 26.7, HS
// 1200 of 4103: I0 1.47 A, ideal 1497 ct): body-diode side 9.5e-9 A/ct^2, reverse-current side
// 6.5e-7 A/ct^2 (see the self-calibration section of diode-emulation.md). The plant current scales
// with the DCM model to the power vExp (1.3: 30 % more V sensitivity than the run normalises with).
// v(us, ls, init, vi, vo) sets the voltages; zero crossing at the true ideal + timingErr.
using VFn = std::function<void(uint32_t, int, bool, float &, float &)>;
static void fixtureDcm(RectCal &cal, const VFn &v, float timingErr = 0.f, float vExp = 1.3f) {
    const uint16_t hs = 1200;
    cal.start(0, hs, 16, PM - hs - 1);
    int ls = cal.lsHold();
    bool init = true;
    const float g0 = 60.f * (60.f - 26.7f) / 26.7f;
    for (uint32_t us = 0; cal.running() && us < 60000000; us += 2304) {
        if (ls != 16) init = false;
        float vi = 60.f, vo = 26.7f;
        v(us, ls, init, vi, vo);
        const float zc = (vi / vo - 1.f) * hs + timingErr;
        const float x = init ? 0.6f * zc : (float) ls;
        const float i = 1.47f * std::pow(vi * (vi - vo) / vo / g0, vExp) - (x < zc ? 9.5e-9f : 6.5e-7f) * (x - zc) * (x - zc);
        ls = cal.feed(us, hs, vi, vo, i, true);
    }
    SynchronousConverter c{}; initConv(c);
    cal.fit(c.getPwmTickRate());
    noteRise("DCM fixture (bus disturbances)", cal);
}

// Known-bad: a bus-voltage dip during the steps 100..200 ct past the zero crossing raises the DCM
// current there above the true peak. Without the step check (old code, see the report) the fit puts
// the peak inside the dip and learns a positive offset, i.e. a later LS turn-off and reverse current.
// Must be rejected; a dip below JumpV must not move the result beyond the test tolerance, and a slow
// linear drift of the same total size must be accepted.
static void testDisturbance() {
    const float zc = (60.f / 26.7f - 1.f) * 1200;
    struct Case { const char *name; float dipVo, dipVi; bool reject; };
    const Case cases[] = {{"Vout dip 1%", -0.01f, 0, true}, {"Vout dip 0.5%", -0.005f, 0, true},
                          {"Vin dip 0.5%", 0, -0.005f, true}, {"Vout dip 0.3%", -0.003f, 0, true}, {"Vout dip 0.15%", -0.0015f, 0, false},
                          {"Vin dip 0.2%", 0, -0.002f, false}};
    RectCal ref;
    fixtureDcm(ref, [](uint32_t, int, bool, float &, float &) {});
    CHECK(ref.get() == RectCal::Done && std::abs(ref.learned) <= 20, "clean: state %d why '%s' learned %d", ref.get(),
          ref.why, ref.learned);
    for (const auto &k: cases) {
        RectCal cal;
        fixtureDcm(cal, [&](uint32_t, int ls, bool init, float &vi, float &vo) {
            if (!init && ls >= zc + 100 && ls <= zc + 200) vi *= 1.f + k.dipVi, vo *= 1.f + k.dipVo;
        });
        if (k.reject)
            // a Vout dip tied to the LS repeats on the retry: the reading follows the LS
            CHECK(cal.get() == RectCal::Fail && !strcmp(cal.why, k.dipVo ? "sense follows LS" : "V step"),
                  "%s: state %d why '%s' learned %d", k.name, cal.get(), cal.why, cal.learned);
        else
            // a dip past the crossing reads as an earlier knee: up to -35 ct, the safe side
            CHECK(cal.get() == RectCal::Done && cal.learned <= 20 && cal.learned >= -35, "%s: state %d why '%s' learned %d",
                  k.name, cal.get(), cal.why, cal.learned);
        printf("disturbance '%s': %s %s learned %d\n", k.name, cal.get() == RectCal::Done ? "done" : "failed", cal.why,
               cal.learned);
    }
    // slow linear Vout drift +2.5 % over the sweep (and a -2.5 % one), with the true offset at -80 ct
    for (float tot: {0.025f, -0.025f}) {
        RectCal cal;
        fixtureDcm(cal, [&](uint32_t us, int, bool init, float &, float &vo) {
            if (!init) vo *= 1.f + tot * std::min(1.f, (float) us * 1e-6f / 30.f);
        }, -80.f);
        CHECK(cal.get() == RectCal::Done && std::abs(cal.learned + 80) <= 20, "Vout drift %+.1f%%: state %d why '%s' learned %d",
              100 * tot, cal.get(), cal.why, cal.learned);
        printf("disturbance 'Vout drift %+.1f%%': %s %s learned %d (true -80), dV %.2f%%\n", 100 * tot,
               cal.get() == RectCal::Done ? "done" : "failed", cal.why, cal.learned, 100 * cal.dv);
    }
}

// Field runs (80 uH, low light, shared bus): knee 30x steeper than the lossless model, baseline falling
// 15 mA over the sweep, the knee lifts Vin on a soft panel, the bus swings. Runs 1 and 3 must calibrate,
// run 2 (cloud) must not. Then bounds for disturbances that pass the voltage checks: a bus square wave
// below JumpVo near the step cycle (plant 0x and 2x the model's voltage sensitivity), and a Vin rise
// confined to the steps past the crossing, which the knee allowance admits: it may only make the
// learned offset earlier (shorter LS).
static void testField() {
    const float tr = 160e6f;
    struct F { const char *name; float vi; uint16_t hs; float rpv; bool cloud; } fs[] = {
        {"run 1", 72.8f, 1111, 5.4f, false}, {"run 3", 71.9f, 1125, 8.4f, false}, {"run 2 cloud", 71.7f, 1178, 5.4f, true},
        {"run 1 stiff", 72.8f, 1111, 0.f, false}};
    for (const auto &f: fs) {
        int done = 0, lo = 999, hi = -999;
        const char *why = "";
        for (uint32_t seed = 1; seed <= 10; ++seed) {
            RcPlant p = fieldPlant(f.vi, 26.f, f.hs, f.rpv, seed);
            if (f.cloud) {
                auto e = p.ext;
                p.ext = [e](double t, int ls, float &vi, float &vo) { e(t, ls, vi, vo); if (t > 3) vi *= 1.f - 0.002f * (float) (t - 3); };
            }
            RectCal c;
            runPlant(c, p, tr, seed);
            noteRise("80 uH plant (low light, bus)", c);
            if (c.get() == RectCal::Done) ++done, lo = std::min(lo, c.learned - 16), hi = std::max(hi, c.learned - 16);
            else why = c.why;
        }
        if (f.cloud) CHECK(done == 0 && !strcmp(why, "Vin shift"), "%s: %d/10 learned, '%s'", f.name, done, why);
        else CHECK(done == 10 && lo >= -10 && hi <= 10, "%s: %d/10 done, err %d..%d ('%s')", f.name, done, lo, hi, why);
        printf("field %-12s %2d/10 done, err %+d..%+d ct %s\n", f.name, done, done ? lo : 0, done ? hi : 0, why);
    }
    int lo = 999, hi = -999, done = 0, tot = 0;
    for (float vexp: {0.f, 2.f})
        for (double P = 0.9; P <= 2.3001; P += 0.1)
            for (int ph = 0; ph < 8; ++ph) {
                RcPlant p{};
                p.vi0 = 60; p.vo0 = 26.7f; p.hs = 1200; p.i0 = 1.47f; p.kE = 9.5e-9f; p.vExp = vexp;
                p.ext = [=](double t, int, float &, float &vo) { vo *= 1.f + 0.0019f * (std::sin(6.2832 * t / P + ph * 0.7854) >= 0 ? 1.f : -1.f); };
                RectCal c;
                runPlant(c, p, tr, 1 + ph);
                noteRise("80 uH plant (low light, bus)", c);
                ++tot;
                if (c.get() == RectCal::Done) ++done, lo = std::min<int>(lo, c.learned), hi = std::max<int>(hi, c.learned);
            }
    // the sweep stops 2 steps past the knee: fewer points on its far side than a full sweep, so
    // more spread; after the margin the LS ends at most 20 ct (125 ns) past the crossing
    CHECK(lo >= -45 && hi <= 40, "bus square 0.19%%: err %d..%d", lo, hi);
    printf("field bus square 0.19%% 0.9..2.3 s, plant 0x/2x: %d/%d done, err %+d..%+d ct\n", done, tot, lo, hi);
    const float zc = (60.f / 26.7f - 1.f) * 1200;
    for (float rise: {0.002f, 0.005f, 0.007f, 0.01f, 0.015f, 0.02f}) {
        RcPlant p{};
        p.vi0 = 60; p.vo0 = 26.7f; p.hs = 1200; p.i0 = 1.47f; p.kE = 9.5e-9f; p.vExp = 0;
        p.ext = [=](double, int ls, float &vi, float &) { if (ls >= zc + 100 && ls <= zc + 300) vi *= 1.f + rise; };
        RectCal c;
        runPlant(c, p, tr);
        noteRise("80 uH plant (low light, bus)", c);
        CHECK(c.get() != RectCal::Done || c.learned <= 5, "Vin +%.1f%% past the crossing: learned %d", 100 * rise, c.learned);
        printf("field Vin +%.1f%% on steps past the crossing (plant 0x): %s %s learned %d\n", 100 * rise,
               c.get() == RectCal::Done ? "done" : "failed", c.why, c.learned);
    }
}

// rect-cal force from MPPT in CCM near the MPP (~12 A): the LS is pinned at its minimum before the
// first HS step and stays there through the ramp-down and Init; no protection acts; the run
// calibrates and HS ramps back. dist(h, t): disturbance per tick (t: s since the start).
struct ForceRun { bool ok; float truth, iMin; int lsBad, stepMax; uint16_t hsOp, hsEnd; uint8_t clamps; double tCal; };
static ForceRun forceRun(Host &h, const std::function<void(Host &, double)> &dist = nullptr) {
    ForceRun f{false, NAN, INFINITY, 0, 0, 0, 0, 0, -1};
    h.op = (uint16_t) (VBAT / (0.80f * VOC) * PM);
    h.run(8);
    f.hsOp = h.c.getCtrlOnPwmCnt();
    CHECK(!h.c.inDCM() && h.iEwm > 6.f, "not CCM at high current before the run: dcm %d I %.2f", h.c.inDCM(), h.iEwm);
    CHECK(h.ctl.requestStart(), "requestStart");
    h.tick();
    CHECK(h.state() == RectCal::Fail && !strcmp(h.ctl.cal.why, "not in DCM"), "plain start from CCM: '%s'", h.ctl.cal.why);
    const uint8_t clamps0 = h.c.lsClamps();
    CHECK(h.ctl.requestStart(true), "requestStart(force)");
    const double t0 = h.r.t;
    h.fault = [&](Host &x) { if (dist) dist(x, x.r.t - t0); };
    uint16_t hsPrev = h.c.getCtrlOnPwmCnt();
    for (double t1 = h.r.t + 60; h.r.t < t1 && !h.tripped;) {
        const auto s0 = h.state();
        h.tick();
        const uint16_t hsNow = h.c.getCtrlOnPwmCnt();
        if (h.state() == RectCal::Run) {
            f.stepMax = std::max(f.stepMax, std::abs((int) hsNow - (int) hsPrev));
            if (hsNow > h.ctl.cal.hs && h.lsAfterHold != h.c.getRectOnPwmMin()) ++f.lsBad; // ramp: LS at min
            f.iMin = std::min(f.iMin, h.r.p.getIoutAvg());
            if (f.tCal < 0 && hsNow == h.ctl.cal.hs) f.tCal = h.r.t - t0;
        }
        hsPrev = hsNow;
        if (s0 == RectCal::Run && h.state() != RectCal::Run)
            f.truth = h.r.p.getVin() * (1.f / h.r.p.getVout() - 1.f / (h.r.p.getVout() + h.r.voutBias)) * (float) h.ctl.cal.hs
                      - (float) h.r.hidden;
        if (h.state() == RectCal::Done || h.state() == RectCal::Fail) break;
    }
    h.fault = nullptr;
    f.ok = h.state() == RectCal::Done;
    f.clamps = (uint8_t) (h.c.lsClamps() - clamps0);
    h.run(3);
    f.hsEnd = h.c.getCtrlOnPwmCnt();
    CHECK(!h.c.manualRect_() && !h.tripped, "LS pinned %d / tripped %d after the run", h.c.manualRect_(), h.tripped);
    return f;
}

static void testForce() {
    for (int hidden: {0, 80, 160})
        for (float vb: {0.f, -0.8f}) {
            Host h(11.5f, hidden, vb, 0.f);
            const ForceRun f = forceRun(h);
            const RectCal &cal = h.ctl.cal;
            noteRise("force from CCM", cal);
            CHECK(f.ok && std::fabs(cal.learned - f.truth) <= 20.f, "force hidden %d vb %.1f: %s learned %d truth %.0f", hidden,
                  vb, f.ok ? "done" : cal.why, cal.learned, f.truth);
            CHECK(f.lsBad == 0 && f.stepMax <= (int) PM / 512 && f.clamps == 0 && f.iMin > 0.f,
                  "force ramp: LS off min %d ticks, max HS step %d, LS clamps %u, min Iout %.2f A", f.lsBad, f.stepMax, f.clamps,
                  f.iMin);
            CHECK(std::abs((int) f.hsEnd - (int) f.hsOp) <= 8, "HS %u not restored to %u", f.hsEnd, f.hsOp);
            printf("force from CCM: hidden %3d vb %4.1f: HS %u -> %u at +%.2f s, %s learned %d truth %.0f (%d pts, dV %.2f%%), "
                   "min Iout %.2f A, back to %u\n", hidden, vb, f.hsOp, cal.hs, f.tCal, f.ok ? "done" : cal.why, cal.learned,
                   f.truth, cal.n, 100 * cal.dv, f.iMin, f.hsEnd);
        }

    // coil.conf L0 off by +-10 %: the charge-based zero crossing moves by half of that
    for (const char *l0: {"59e-6", "48.2e-6"}) {
        g_L0 = l0;
        Host h(11.5f, 80, 0.f, 0.f);
        const ForceRun f = forceRun(h);
        const RectCal &cal = h.ctl.cal;
        CHECK(f.ok && cal.learned <= f.truth + 20.f && cal.learned >= f.truth - 0.06f * cal.ideal,
              "L0 %s: %s learned %d truth %.0f", l0, f.ok ? "done" : cal.why, cal.learned, f.truth);
        printf("force, coil L0 %s (model 0.95 L0, plant 50.9e-6): %s zc-ideal %+.0f learned %d truth %.0f\n", l0, f.ok ? "done" : cal.why,
               cal.zc - cal.ideal, cal.learned, f.truth);
    }
    g_L0 = "53.6e-6";

    // battery bus: Vout +2.5 % / -2.5 % linear over 30 s from the start (second charger, loads): accepted
    for (float tot: {0.025f, -0.025f}) {
        Host h(11.5f, 80, 0.f, 0.f);
        const ForceRun f = forceRun(h, [tot](Host &x, double t) {
            x.r.p.setBat(VBAT * (1.f + tot * (float) std::min(1.0, t / 30.0)), 0.03f);
        });
        const RectCal &cal = h.ctl.cal;
        CHECK(f.ok && std::fabs(cal.learned - f.truth) <= 20.f, "Vout drift %+.1f%%: %s learned %d truth %.0f", 100 * tot,
              f.ok ? "done" : cal.why, cal.learned, f.truth);
        printf("force, Vout drift %+.1f%% over 30 s: %s learned %d truth %.0f (dV %.2f%%)\n", 100 * tot,
               f.ok ? "done" : cal.why, cal.learned, f.truth, 100 * cal.dv);
    }

    // irradiance step at the stiff point (Isc x0.7, Voc kept): Vin hardly moves, the scaling covers it.
    // cloud (Isc x0.7, Voc -1.8 %, i.e. the cell-voltage drop of 30 % less light): rejected.
    for (bool vocDrop: {false, true}) {
        Host h(11.5f, 80, 0.f, 0.f);
        const ForceRun f = forceRun(h, [vocDrop](Host &x, double t) {
            if (t >= 12.0) x.r.isc0 = 11.5f * 0.7f, x.r.voc = vocDrop ? VOC * 0.982f : VOC;
        });
        if (vocDrop)
            CHECK(!f.ok && (!strcmp(h.ctl.cal.why, "V step") || !strcmp(h.ctl.cal.why, "Vin shift")), "cloud: %s",
                  f.ok ? "learned" : h.ctl.cal.why);
        else
            CHECK(f.ok ? std::fabs(h.ctl.cal.learned - f.truth) <= 20.f : !strcmp(h.ctl.cal.why, "V step"),
                  "irradiance step: %s learned %d truth %.0f", f.ok ? "done" : h.ctl.cal.why, h.ctl.cal.learned, f.truth);
        printf("force, %s at +12 s: %s learned %d truth %.0f (%d pts, dV %.2f%%)\n", vocDrop ? "cloud step" : "Isc step",
               f.ok ? "done" : h.ctl.cal.why, f.ok ? h.ctl.cal.learned : 0, f.truth, h.ctl.cal.n, 100 * h.ctl.cal.dv);
        if (g_verbose) h.ctl.status(h.c);
    }
}

// Full sun on the 80 uH board (field run 4): CCM at 57 V / 10 A, the hold sits near Voc (78 V). The
// plant has a 5 nF switch node ringing at Q 40 (fitted to the four field runs) and the LS conducts 35 ct
// longer than commanded. Past the zero crossing the output current is flat until the reverse current
// lifts the node to Vin (~170 ct here), then drops steeply, recovers and turns positive near
// LS = 1.25x ideal; at (1 - Vo/Vi) of the period the reverse current no longer returns to zero before HS
// turns on and grows every cycle (the field guard trip). The run must stop after the knee and never reach
// the guard. The knee fit lands between the zero crossing (-35) and the clamp onset (~+50); the applied
// offset must not be later than the crossing by more than 20 ct (40 mA reverse at 80 uH).
static void testFullSun() {
    g_L0 = "84.2e-6";
    int lo = 999, hi = -999, alo = 999, ahi = -999, done = 0, guards = 0, trips = 0, lsMaxPast = -999;
    for (uint32_t seed = 1; seed <= 6; ++seed) {
        Host h(5.6f, 35, 0.f, 0.f);
        h.r.rng.seed(seed);
        h.r.voc = 79.6f;
        h.r.p.setPv(5.6f, 79.6f, K); h.r.p.setBat(26.45f, 0.04f); h.r.p.setPassives(470e-6f, 470e-6f, 80e-6f);
        h.r.p.setVin(79.6f); h.r.p.setVout(26.45f); h.r.p.setRing(5e-9f, 40.f);
        h.vi = h.viEwm = 79.6f; h.vo = h.voEwm = 26.45f;
        h.op = (uint16_t) (26.45f / 57.f * PM);
        h.run(8);
        CHECK(!h.c.inDCM() && h.iEwm > 8.f, "full sun: not CCM before the run (I %.2f, Vin %.1f)", h.iEwm, h.viEwm);
        const int g0 = h.guard;
        CHECK(h.ctl.requestStart(true), "requestStart(force)");
        for (double t1 = h.r.t + 70; h.r.t < t1 && !h.tripped;) {
            h.tick();
            if (h.state() == RectCal::Run && h.c.manualRect_() && h.ctl.cal.ideal > 0)
                lsMaxPast = std::max(lsMaxPast, (int) h.c.getRectOnPwmCnt() - (int) lroundf(h.ctl.cal.ideal));
            if (h.state() == RectCal::Done || h.state() == RectCal::Fail) break;
        }
        const RectCal &cal = h.ctl.cal;
        noteRise("full sun 80 uH ring Q40", cal);
        guards += h.guard - g0; trips += h.tripped;
        if (cal.get() == RectCal::Done) {
            ++done; lo = std::min<int>(lo, cal.learned); hi = std::max<int>(hi, cal.learned);
            alo = std::min<int>(alo, cal.applied); ahi = std::max<int>(ahi, cal.applied);
        }
        printf("full sun seed %u: %s%s hs=%u ideal=%.0f learned %+d applied %+d%s (%u pts, %u redone), guard %d\n", seed,
               cal.get() == RectCal::Done ? "done" : "failed: ", cal.get() == RectCal::Done ? "" : cal.why, cal.hs, cal.ideal,
               cal.learned, cal.applied, cal.cut ? " (fit ended at a dip)" : "", cal.n, cal.retries, h.guard - g0);
        if (seed == 1 && g_verbose) {
            for (int i = 0; i < cal.n; ++i) printf(" %.0f:%+.3f", cal.xs[i] - cal.ideal, cal.ys[i]);
            printf("\n");
        }
    }
    g_L0 = "53.6e-6";
    CHECK(done == 6 && guards == 0 && trips == 0, "full sun: %d/6 done, guard %d, trips %d", done, guards, trips);
    CHECK(done && alo >= -35 - 45 && ahi <= -35 + 20 && hi - lo <= 10, "full sun: applied %d..%d, learned %d..%d", alo, ahi,
          lo, hi);
    CHECK(lsMaxPast <= 200, "full sun: LS reached ideal%+d", lsMaxPast);
    printf("full sun: %d/6 done, learned %+d..%+d ct, applied %+d..%+d ct (zero crossing -35 ct), LS at most %+d ct past "
           "ideal, guard %d\n", done, lo, hi, alo, ahi, lsMaxPast, guards);
}

// Switch-node ring of fry/flat (230 ct period: 50.9 uH, 1.03 nF) at Q 3..40: past the crossing the current
// dips, rings back above the baseline and drops again. The old sweep never saw two drops in a row, ran to
// the cap and learned up to +252 ct (truth -35). Must fail or apply at most 20 ct late (as full sun) and learn
// at most 25 ct early.
static void testRingBack() {
    g_L0 = "53.6e-6";
    for (float q: {3.f, 5.f, 10.f, 40.f})
        for (uint32_t seed = 1; seed <= 2; ++seed) {
            Host h(5.6f, 35, 0.f, 0.f);
            h.r.rng.seed(seed);
            h.r.voc = 79.6f;
            h.r.p.setPv(5.6f, 79.6f, K); h.r.p.setBat(26.7f, 0.04f); h.r.p.setPassives(470e-6f, 470e-6f, 50.9e-6f);
            h.r.p.setVin(79.6f); h.r.p.setVout(26.7f); h.r.p.setRing(1.03e-9f, q);
            h.vi = h.viEwm = 79.6f; h.vo = h.voEwm = 26.7f;
            h.op = (uint16_t) (26.7f / 57.f * PM);
            h.run(8);
            CHECK(h.ctl.requestStart(true), "requestStart(force)");
            for (double t1 = h.r.t + 70; h.r.t < t1 && !h.tripped;) {
                h.tick();
                if (h.state() == RectCal::Done || h.state() == RectCal::Fail) break;
            }
            const RectCal &cal = h.ctl.cal;
            noteRise("ring 51 uH Q3-40", cal);
            // two-sided: Q 3 learns up to 33 ct late (applied 13 late), Q 40 up to 14 ct early; Q 5 has a 6 mA first
            // dip below the 10 mA knee bound and may fail
            CHECK(!h.tripped && (cal.get() == RectCal::Fail || (cal.get() == RectCal::Done && cal.applied <= -35 + 20
                                                                  && cal.learned >= -35 - 25)),
                  "ring Q %.0f seed %u: state %d '%s' applied %d", q, seed, cal.get(), cal.why, cal.applied);
            printf("ring 230 ct Q %2.0f seed %u: %s%s learned %+d applied %+d%s (truth -35, %u pts)\n", q, seed,
                   cal.get() == RectCal::Done ? "done" : "failed: ", cal.why, cal.learned, cal.applied,
                   cal.cut ? " (fit ended at a dip)" : "", cal.n);
            if (g_verbose) {
                for (int i = 0; i < cal.n; ++i) printf(" %.0f:%+.3f", cal.xs[i] - cal.ideal, cal.ys[i]);
                printf("\n");
            }
        }
    g_L0 = "53.6e-6";
}

// Knee at c, quadratic to a dip floor 60 ct later, a partial recovery below StepDrop, then the ring rise. The fit
// ends at the dip floor and can place the knee up to ~17 ct late: the dip margin keeps the applied offset at least
// 20 ct before the knee.
static void testDipRecovery() {
    const uint16_t hs = 905;
    const float vo = 26.7f, vi = vo * (1.f + 1732.f / hs), ideal = (vi / vo - 1.f) * hs;
    for (float c: {-50.f, 0.f})
        for (float R: {0.004f, 0.008f}) {
            int done = 0, cuts = 0, lo = 999, hi = -999, aHi = -999;
            for (uint32_t seed = 1; seed <= 8; ++seed) {
                std::mt19937 rng(seed);
                std::normal_distribution<float> nd(0, 0.002f);
                auto f = [&](float x) {
                    const float d = x - c, k = 6e-6f, W = 60;
                    return d <= 0 ? 0.f : d <= W ? -k * d * d : -k * W * W + R * std::min(1.f, (d - W) / 40.f) + (d > W + 150 ? 0.03f : 0.f);
                };
                RectCal r;
                r.start(0, hs, 16, PM - hs - 1);
                int ls = r.lsHold(), lp = -1;
                float e = 0;
                for (uint32_t us = 0; r.running() && us < 60000000; us += 2304) {
                    if (ls != lp) e = nd(rng), lp = ls;
                    ls = r.feed(us, hs, vi, vo, 2.f + (ls == 16 ? 0.f : f((float) ls - ideal) + e), true);
                }
                r.fit(160e6f);
                if (r.get() != RectCal::Done) continue;
                ++done, cuts += r.cut, lo = std::min<int>(lo, r.learned), hi = std::max<int>(hi, r.learned);
                aHi = std::max<int>(aHi, r.applied);
                CHECK(r.applied <= c - 20, "dip recovery c %.0f R %.3f seed %u: learned %d applied %d%s", c, R, seed, r.learned,
                      r.applied, r.cut ? " (dip)" : "");
            }
            printf("dip + partial recovery %.0f mA, knee %+.0f: %d/8 done (%d ended at a dip), learned %+d..%+d, applied at "
                   "most %+d\n", 1e3f * R, c, done, cuts, lo, hi, aHi);
        }
}

// Protection during the ramp-down from CCM: aborted, LS back to diode emulation, no ramp back when the
// reverse-current guard opened the backflow switch; shutdown; Vin close to its trip. At HS -200 (D 0.47)
// an open backflow switch trips highD-bflowOpen (shutdown), near the calibration point (D < 0.33) not.
static void testForceProtection() {
    struct Case { const char *name; int below; std::function<void(Host &)> act; bool rampBack; };
    const Case cases[] = {
        {"revI: LS clamp + bflow open", 200, [](Host &x) { x.bflow = false; x.c.syncRectMinDuty(); }, false},
        {"revI: LS clamp + bflow open", 820, [](Host &x) { x.bflow = false; x.c.syncRectMinDuty(); }, false},
        {"LS clamp only", 200, [](Host &x) { x.c.syncRectMinDuty(); }, true},
        {"shutdown", 200, [](Host &x) { x.c.disable(); }, false},
    };
    for (const auto &k: cases) {
        Host h(11.5f, 80, 0.f, 0.f);
        h.op = (uint16_t) (VBAT / (0.80f * VOC) * PM);
        h.run(8);
        const uint16_t hsOp = h.c.getCtrlOnPwmCnt();
        h.ctl.requestStart(true);
        h.tick();
        while (h.state() == RectCal::Run && h.c.getCtrlOnPwmCnt() > hsOp - k.below) h.tick();
        const uint16_t hsAt = h.c.getCtrlOnPwmCnt();
        bool fired = false;
        h.fault = [&](Host &x) { if (!fired) { fired = true; k.act(x); } };
        h.tick();
        h.fault = nullptr;
        // the RT loop calls rt() on every new sample, also with the converter off
        if (h.tripped || h.c.disabled()) { uint16_t b = h.ctl.rt(h.c, h.in()); if (b) h.rampTo = b; }
        CHECK(h.state() == RectCal::Fail && !strcmp(h.ctl.cal.why, "protection") && !h.c.manualRect_(),
              "%s: state %d why '%s' manualRect %d", k.name, h.state(), h.ctl.cal.why, h.c.manualRect_());
        CHECK(k.rampBack ? h.rampTo == hsOp : !h.rampTo, "%s: rampTo %u (op %u)", k.name, h.rampTo, hsOp);
        printf("force protection '%s' at HS %u (from %u, cal %u): %s%s, LS %u, rampTo %u\n", k.name, hsAt, hsOp,
               h.ctl.cal.hs, h.ctl.cal.why, h.tripped ? " (highD-bflowOpen shutdown)" : "", h.c.getRectOnPwmCnt(), h.rampTo);
    }
    Host h(11.5f, 80, 0.f, 0.f);
    h.vinMax = VOC * 1.0f; // trip at Voc: the run must stop at 0.97 Voc
    h.op = (uint16_t) (VBAT / (0.80f * VOC) * PM);
    h.run(8);
    h.ctl.requestStart(true);
    h.tick();
    float vMax = 0;
    while (h.state() == RectCal::Run && h.r.t < 120) { h.tick(); vMax = std::max(vMax, h.vi); }
    CHECK(h.state() == RectCal::Fail && !strcmp(h.ctl.cal.why, "Vin near max") && !h.c.manualRect_(), "Vin max: '%s'",
          h.ctl.cal.why);
    printf("force near Vin trip (%.1f V): %s at Vin %.1f V\n", h.vinMax, h.ctl.cal.why, vMax);
}

// Finding 2: protection during a run aborts it and the run never re-raises LS.
static void testProtectionWins() {
    struct Case { const char *name; std::function<void(Host &)> act; };
    const Case cases[] = {
        {"revI clamp + bflow open", [](Host &h) { h.bflow = false; h.c.syncRectMinDuty(); }},
        {"LS clamp only", [](Host &h) { h.c.syncRectMinDuty(); }},
        {"bflow open only", [](Host &h) { h.bflow = false; }},
    };
    for (const auto &k: cases) {
        Host h(3.f, 80, 0.f, 0.f);
        h.op = (uint16_t) (0.6f * VBAT / VOC * PM);
        h.run(6);
        h.ctl.requestStart();
        // wait for a measurement step with LS well above the reference
        for (double t1 = h.r.t + 40; h.r.t < t1 && !(h.lsAfterHold > 1.1f * h.ctl.cal.ideal && h.ctl.cal.n >= 3);) h.tick();
        const int lsBefore = h.lsAfterHold;
        bool fired = false;
        h.fault = [&](Host &x) { if (!fired) { fired = true; k.act(x); } };
        h.tick();
        const bool lsMin = h.lsRaw == h.c.getRectOnPwmMin() || !strcmp(k.name, "bflow open only");
        CHECK(h.state() == RectCal::Fail && !strcmp(h.ctl.cal.why, "protection") && !h.held && h.lsAfterHold < 0 && lsMin,
              "%s: state %d why '%s' held %d lsAfterHold %d LS %u (before %d)", k.name, h.state(), h.ctl.cal.why, h.held,
              h.lsAfterHold, h.lsRaw, lsBefore);
        h.fault = nullptr;
        bool pinned = false;
        for (int n = 0; n < 2000; ++n) { h.tick(); pinned |= h.c.manualRect_(); }
        CHECK(!pinned, "%s: LS pinned again after protection", k.name);
        printf("protection '%s': LS %d -> %d after hold(), run %s\n", k.name, lsBefore, h.lsRaw, h.ctl.cal.why);
    }
    // shutdown: next rt() aborts
    Host h(3.f, 80, 0.f, 0.f);
    h.op = (uint16_t) (0.6f * VBAT / VOC * PM);
    h.run(6);
    h.ctl.requestStart();
    h.run(5);
    h.c.disable();
    h.tick();
    CHECK(h.state() == RectCal::Fail && !h.c.manualRect_(), "shutdown: state %d why '%s'", h.state(), h.ctl.cal.why);
}

// Abort in each phase, mode change, PSU switch: LS released, HS restored only in MPPT mode.
static void testAbortAndModes() {
    for (double at: {0.3, 1.0, 3.0, 12.0}) {
        Host h(3.f, 40, 0.f, 0.f);
        h.op = (uint16_t) (0.6f * VBAT / VOC * PM);
        h.run(6);
        const uint16_t hsOp = h.c.getCtrlOnPwmCnt();
        h.ctl.requestStart();
        h.run(at);
        h.ctl.abortReq.store(true);
        h.tick();
        CHECK(h.state() == RectCal::Fail && !strcmp(h.ctl.cal.why, "aborted") && !h.c.manualRect_(),
              "abort at %.1fs: state %d why '%s'", at, h.state(), h.ctl.cal.why);
        h.run(2);
        CHECK(std::abs((int) h.c.getCtrlOnPwmCnt() - (int) hsOp) <= 8, "abort at %.1fs: HS %u, op %u", at,
              h.c.getCtrlOnPwmCnt(), hsOp);
    }
    Host h(3.f, 40, 0.f, 0.f);
    h.op = (uint16_t) (0.6f * VBAT / VOC * PM);
    h.run(6);
    h.ctl.requestStart();
    h.run(4);
    h.mppt = false; // PSU / PV-sim / manual duty
    h.tick();
    CHECK(h.state() == RectCal::Fail && !strcmp(h.ctl.cal.why, "not in MPPT mode") && !h.c.manualRect_() && !h.rampTo,
          "mode: state %d why '%s' rampTo %u", h.state(), h.ctl.cal.why, h.rampTo);
    h.mppt = true;
    h.ctl.requestStart();
    h.run(4);
    h.lim = true; // a limiter past its limit (D-only touches never reach RectCalIn::limiting)
    h.tick();
    CHECK(h.state() == RectCal::Fail && !strcmp(h.ctl.cal.why, "limiting") && !h.c.manualRect_(), "limit: '%s'",
          h.ctl.cal.why);
    CHECK(h.ctl.requestStart() && (h.tick(), h.state() == RectCal::Fail && !strcmp(h.ctl.cal.why, "limiting")),
          "start while limiting: '%s'", h.ctl.cal.why);
    h.lim = false;
    h.ctl.requestStart();
    h.run(4);
    h.ctl.drop();
    CHECK(h.state() == RectCal::Fail && !strcmp(h.ctl.cal.why, "mode change"), "drop: '%s'", h.ctl.cal.why);
    CHECK(h.ctl.requestStart(), "restart after drop refused");
    printf("abort/modes: ok\n");
}

// Auto schedule: boot delay, two refusals each retried after 10 min, a run at most once a day.
static void testSchedule() {
    SynchronousConverter c{}; initConv(c); // never calibrated
    c.pwmPerturb(400);
    Ctl ctl{};
    ctl.persist = fakePersist;
    RectCalIn in{1000, 60, 26.7f, 1, 60, 26.7f, 1, 1, 1, false, false, true, false, true, 90.f}; // not MPPT: refused
    auto lf = [&](uint32_t s) { ctl.lf(c, s, 30.f); ctl.rt(c, in); ctl.lf(c, s, 30.f); };
    lf(599);
    CHECK(ctl.cal.seq == 0, "started in the boot delay");
    lf(600);
    CHECK(ctl.cal.seq == 1 && ctl.cal.get() == RectCal::Fail, "no auto start at 600 s (seq %u)", ctl.cal.seq);
    lf(1199);
    CHECK(ctl.cal.seq == 1, "retried before 10 min");
    lf(1201);
    CHECK(ctl.cal.seq == 2, "first refusal not retried (seq %u)", ctl.cal.seq);
    lf(1802);
    CHECK(ctl.cal.seq == 3, "second refusal not retried (seq %u)", ctl.cal.seq);
    c.setRectOffsetNeedsCal(false);
    lf(2500);
    CHECK(ctl.cal.seq == 3, "auto start while calibrated");
    printf("schedule: ok (seq %u)\n", ctl.cal.seq);
}

// Stop the converter and let rt() see it, as the RT loop does at night.
static void stopConv(Host &h) {
    h.c.disable();
    h.ctl.rt(h.c, h.in());
}

// MpptController::rectCalLf: lf(), then saveNow() whose wait lets RT sample (2 ms on target).
static void lfPass(Host &h, float pOut) {
    const auto nowS = (uint32_t) h.r.t;
    h.ctl.lf(h.c, nowS, pOut);
    h.ctl.saveNow(h.c, nowS, [&] {
        h.r.t += 0.002;
        h.ctl.rt(h.c, h.in());
    });
}

// RT (rt() at 10 Hz) and LF (lfPass() every lfS) for `seconds` without the plant, converter as is.
static void idle(Host &h, double seconds, float pOut, double lfS = 1.0) {
    for (double t1 = h.r.t + seconds; h.r.t < t1;) {
        const auto k0 = (int64_t) (h.r.t / lfS);
        h.r.t += 0.1;
        h.ctl.rt(h.c, h.in());
        if ((int64_t) (h.r.t / lfS) != k0) lfPass(h, pOut);
    }
}

// What a reboot reads back from the last save.
static void rebootCheck(const std::pair<int, int> &sv, const char *what) {
    SynchronousConverter rb{};
    char ns[16], cs[4];
    initConv(rb);
    snprintf(ns, sizeof(ns), "%.0f", rectOffsetNsFromCounts(sv.first, rb.getPwmTickRate()));
    snprintf(cs, sizeof(cs), "%d", sv.second);
    initConv(rb, ns, "mock", cs);
    CHECK(std::abs(rb.getRectOnOffset() - sv.first) <= 1 && rb.rectOffsetCal() == sv.second,
          "%s reboot: off %d (saved %d) needs %d", what, rb.getRectOnOffset(), sv.first, rb.rectOffsetNeedsCal());
}

// A deterministic backstop trigger through rt(): peak, then a collapse with Vin rising, HS above
// 0.8*M. Returns true when the offset shrank.
static bool fireBackstop(Host &h, uint32_t afterUs = 3000000) {
    const uint16_t f0 = h.ctl.fires.load();
    const uint16_t hs = h.c.getCtrlOnPwmCnt();
    const float vout = 60.f * (float) hs / 0.9f / (float) PM; // hs = 0.9*M
    RectCalIn in = h.in();
    in.vin = in.vinAvg = 60.f; in.vout = in.voutAvg = vout; in.mppt = true; in.sweeping = false;
    in.iS = in.iF = 2.f;
    in.nowUs += afterUs;
    h.ctl.rt(h.c, in); // arms (start-up hold over), records the peak
    h.c.pwmPerturb(20);
    in.nowUs += 10000; in.vin = 61.f; in.iS = 1.9f; in.iF = 0.1f;
    h.ctl.rt(h.c, in);
    return h.ctl.fires.load() != f0;
}

// Findings 5/6 and R2-1/R2-2: coil.conf gets the RT-published (offset, calibrated) pair, only after the
// converter has been off for IdleS with a finite power reading, at most once per SaveMinS; failed
// saves are retried; a backstop reduction is never overwritten by an older calibration.
static void testPersistence() {
    g_saved.clear(); g_saveCalls = 0; g_saveOk = true;
    Host h(3.f, 80, 0.f, 0.f);
    float truth;
    g_ctl = &h.ctl; g_conv = &h.c; g_writeHook = nullptr; g_poweredWrites = 0;
    CHECK(calibrate(h, truth), "cal failed: %s", h.ctl.cal.why);
    CHECK(g_saveCalls == 0 && h.ctl.unsaved(), "saved while converting (%d calls)", g_saveCalls);
    idle(h, 20, 0.5f);
    CHECK(g_saveCalls == 0, "saved with the converter on at 0.5 W");
    stopConv(h);
    idle(h, 5, 0.f);
    CHECK(g_saveCalls == 0, "saved after 5 s off");
    idle(h, 30, NAN);
    CHECK(g_saveCalls == 0, "saved with power NaN");
    g_saveOk = false;
    idle(h, 2, 0.f);
    CHECK(g_saveCalls == 1 && h.ctl.unsaved(), "failed save dropped (%d calls)", g_saveCalls);
    idle(h, 30, 0.f);
    CHECK(g_saveCalls == 1, "retry before 60 s");
    g_saveOk = true;
    idle(h, 40, 0.f);
    CHECK(g_saved.size() == 1 && g_saved[0].first == h.ctl.cal.applied && g_saved[0].second && !h.ctl.unsaved(),
          "calibrated save: %zu", g_saved.size());
    if (!g_saved.empty()) rebootCheck(g_saved[0], "calibrated");
    h.ctl.requestSave();
    idle(h, 100, 0.f);
    CHECK(g_saved.size() == 1, "second save within SaveMinS");
    idle(h, Ctl::SaveMinS, 0.f);
    CHECK(g_saved.size() == 2 && !h.ctl.unsaved(), "rate-limited save never came (%zu)", g_saved.size());

    // R2-1: fit -> one backstop reduction -> save -> reboot.
    g_saved.clear(); g_saveCalls = 0;
    Host m(3.f, 80, 0.f, 0.f);
    g_ctl = &m.ctl; g_conv = &m.c;
    CHECK(calibrate(m, truth), "cal failed: %s", m.ctl.cal.why);
    const int fitted = m.ctl.cal.applied;
    CHECK(fireBackstop(m), "fixture backstop did not fire");
    m.ctl.lf(m.c, (uint32_t) m.r.t + 1, 30.f);
    stopConv(m);
    idle(m, 12, 0.f);
    CHECK(g_saved.size() == 1 && g_saved[0].first < fitted && g_saved[0].first == m.ctl.offset() && !g_saved[0].second
          && m.c.rectOffsetNeedsCal(), "fit+backstop save: fitted %d active %d saved %d cal %d", fitted, m.ctl.offset(),
          g_saved.empty() ? 0 : g_saved[0].first, g_saved.empty() ? -1 : g_saved[0].second);
    if (!g_saved.empty()) rebootCheck(g_saved[0], "fit+backstop");
    printf("persistence: fitted %d, one backstop -> saved %d ct uncalibrated\n", fitted,
           g_saved.empty() ? 0 : g_saved[0].first);

    // A calibration posted before RT saw a newer backstop reduction: applied as the shorter, uncalibrated.
    {
        Host k(3.f, 80, 0.f, 0.f, -60);
        k.c.pwmPerturb(500);
        k.ctl.post(-30, true, (uint16_t) (k.ctl.fires.load() - 1));
        k.ctl.rt(k.c, k.in());
        CHECK(k.ctl.offset() == -60 && !k.ctl.activeCalibrated(), "stale post applied: %d cal %d", k.ctl.offset(),
              k.ctl.activeCalibrated());
        k.ctl.post(-80, true);
        k.ctl.rt(k.c, k.in());
        CHECK(k.ctl.offset() == -80 && k.ctl.activeCalibrated(), "current post: %d cal %d", k.ctl.offset(),
              k.ctl.activeCalibrated());
    }

    // Backstop only: repeated sweeps on one controller; saved after Confirm triggers, uncalibrated,
    // at most once per SaveMinS.
    g_saved.clear(); g_saveCalls = 0;
    Ctl shared{};
    shared.init(140, true);
    shared.persist = fakePersist;
    shared.autoRun = false;
    bool needs = false;
    int fires = 0, off = 140;
    for (int sw = 0; sw < 6; ++sw) {
        Host s(11.5f, 160, -0.8f, 0.f, off, &shared);
        g_ctl = &shared; g_conv = &s.c;
        s.c.setRectOffsetNeedsCal(needs);
        s.sweeping = true;
        s.r.t = s.lfAt = 300.0 * sw; // monotonic time across sweeps
        while (s.r.t < 300.0 * sw + 8 && !s.tripped && s.c.getCtrlOnPwmCnt() / (float) PM < 1.08f * s.r.p.getVout() / s.r.p.getVin())
            s.tick();
        fires = shared.fires.load();
        if (fires == 1) CHECK(!shared.unsaved() && g_saved.empty(), "saved after one trigger");
        stopConv(s);
        s.r.t = 300.0 * sw + 100;
        idle(s, 12, 0.f);
        needs = s.c.rectOffsetNeedsCal();
        off = shared.offset();
    }
    bool allUncal = true;
    for (auto &v: g_saved) allUncal &= !v.second;
    CHECK(fires >= 2 && !g_saved.empty() && allUncal && needs, "backstop save: fires %d saves %zu", fires, g_saved.size());
    if (!g_saved.empty()) rebootCheck(g_saved[0], "backstop");
    printf("persistence: backstop %d triggers -> saved %d ct uncalibrated (%zu save(s))\n", fires,
           g_saved.empty() ? 0 : g_saved[0].first, g_saved.size());
    CHECK(g_poweredWrites == 0, "%d powered writes", g_poweredWrites);
    g_ctl = nullptr; g_conv = nullptr;
}

// R3-1/R4-1: the persist lock. Granted only after continuous off time (a start/stop pulse between two
// samples restarts it); starts are refused from the grant until the admitted write is done, however
// long it takes; a grant that is not admitted expires after LockMaxUs and the save is retried.
static void testPersistLock() {
    g_saved.clear(); g_saveCalls = 0; g_saveOk = true; g_writes = g_poweredWrites = g_unadmitted = 0;
    Host h(3.f, 80, 0.f, 0.f, -40);
    g_ctl = &h.ctl; g_conv = &h.c;
    h.op = (uint16_t) (0.6f * VBAT / VOC * PM);
    h.run(6);
    h.ctl.requestSave();
    stopConv(h);
    idle(h, 9, 0.f);
    h.c.pwmPerturb(1); // start/stop pulse with no rt() in between
    h.c.disable();
    const double pulse = h.r.t;
    idle(h, 9, 0.f);
    CHECK(g_saveCalls == 0, "saved %.1f s after an on/off pulse", h.r.t - pulse);

    // start attempts before and after the write: refused
    int blocked = 0;
    g_writeHook = [&](int) {
        h.c.pwmPerturb(10);
        blocked += h.c.disabled();
        h.r.t += 0.05;
        h.ctl.rt(h.c, h.in());
    };
    idle(h, 3, 0.f);
    CHECK(g_saved.size() == 1 && g_poweredWrites == 0 && blocked == 2 && h.r.t - pulse >= 10.0,
          "lock: saves %zu powered %d blocked %d at %.1f s after pulse", g_saved.size(), g_poweredWrites, blocked,
          h.r.t - pulse);
    g_writeHook = nullptr;
    // after the final write: released at the next sample, saved state truthful
    h.ctl.rt(h.c, h.in());
    h.c.pwmPerturb(10);
    CHECK(!h.c.disabled() && !h.ctl.unsaved(), "start refused / unsaved after the save");

    // an admitted write that outlasts LockMaxUs keeps starts blocked to its end and reports success
    h.c.disable();
    idle(h, Ctl::SaveMinS, 0.f); // rate limit
    h.ctl.requestSave();
    g_saved.clear();
    int slowBlocked = 0, hooks = 0;
    g_writeHook = [&](int) {
        h.r.t += 9.0; // > LockMaxUs inside the write
        h.ctl.rt(h.c, h.in());
        h.c.pwmPerturb(10);
        slowBlocked += h.c.disabled();
        ++hooks;
    };
    idle(h, 2, 0.f);
    g_writeHook = nullptr;
    CHECK(hooks == 2 && slowBlocked == 2 && g_saved.size() == 1 && !h.ctl.unsaved() && g_poweredWrites == 0,
          "slow write: hooks %d blocked %d saves %zu unsaved %d powered %d", hooks, slowBlocked, g_saved.size(),
          h.ctl.unsaved(), g_poweredWrites);
    h.ctl.rt(h.c, h.in());
    h.c.pwmPerturb(10);
    CHECK(!h.c.disabled(), "start refused after the slow write");

    // a grant core 0 never admits (core 0 stalled): expires after LockMaxUs, no write, retried
    h.c.disable();
    idle(h, Ctl::SaveMinS, 0.f);
    h.ctl.requestSave();
    const int calls0 = g_saveCalls;
    h.ctl.lf(h.c, (uint32_t) h.r.t, 0.f); // request without the wrapper's admit
    h.r.t += 0.001;
    h.ctl.rt(h.c, h.in());
    CHECK(h.c.startInhibited(), "not granted");
    h.r.t += Ctl::LockMaxUs * 1e-6 - 0.01;
    h.ctl.rt(h.c, h.in());
    CHECK(h.c.startInhibited(), "grant expired early");
    h.r.t += 0.01;
    h.ctl.rt(h.c, h.in());
    h.c.pwmPerturb(10);
    const bool startedAfterExpiry = !h.c.disabled();
    h.ctl.trySave(h.c, (uint32_t) h.r.t); // the stalled core 0 resumes: nothing to admit
    CHECK(startedAfterExpiry && g_saveCalls == calls0 && h.ctl.unsaved(),
          "expiry: started %d calls %d unsaved %d", startedAfterExpiry, g_saveCalls - calls0, h.ctl.unsaved());
    h.c.disable();
    idle(h, Ctl::SaveRetryS - 5, 0.f);
    CHECK(g_saveCalls == calls0, "retried before SaveRetryS");
    idle(h, 20, 0.f);
    CHECK(g_saveCalls == calls0 + 1 && !h.ctl.unsaved(), "not saved after the expired grant");
    CHECK(g_unadmitted == 0 && g_poweredWrites == 0, "unadmitted %d powered %d", g_unadmitted, g_poweredWrites);
    // R5-2: RT grants between lf() and saveNow(): admitted in the same pass, no wait
    h.c.disable();
    idle(h, Ctl::SaveMinS, 0.f);
    h.ctl.requestSave();
    const int calls1 = g_saveCalls;
    int waits = 0;
    h.ctl.lf(h.c, (uint32_t) h.r.t, 0.f);
    h.ctl.rt(h.c, h.in()); // the grant lands before saveNow() checks the state
    h.ctl.saveNow(h.c, (uint32_t) h.r.t, [&] { ++waits; });
    h.ctl.rt(h.c, h.in());
    CHECK(g_saveCalls == calls1 + 1 && !h.ctl.unsaved() && !h.c.startInhibited() && waits == 0,
          "early grant: calls %d unsaved %d inhibited %d waits %d", g_saveCalls - calls1, h.ctl.unsaved(),
          h.c.startInhibited(), waits);
    // a request RT does not grant in the wait (off time not over): withdrawn, nothing inhibited after
    idle(h, Ctl::SaveMinS, 0.f);
    h.ctl.requestSave();
    h.c.pwmPerturb(1);
    h.c.disable();
    h.ctl.rt(h.c, h.in());
    h.ctl.lf(h.c, (uint32_t) h.r.t, 0.f);
    h.ctl.saveNow(h.c, (uint32_t) h.r.t, [&] { ++waits; h.r.t += 0.002; h.ctl.rt(h.c, h.in()); });
    h.ctl.rt(h.c, h.in());
    CHECK(waits == 10 && !h.ctl.saveRequested() && !h.c.startInhibited() && h.ctl.unsaved(),
          "ungranted: waits %d requested %d inhibited %d", waits, h.ctl.saveRequested(), h.c.startInhibited());
    g_ctl = nullptr; g_conv = nullptr;
    printf("persist lock: ok (pulse resets off time, starts refused through an admitted 18 s write, "
           "unadmitted grant expires and retries)\n");
}

// A knee 80 ct past ideal (LS conducting 80 ct shorter than commanded) from above Vin = 2 Vout is a bound, not a
// measurement: the active offset when shorter, else 0; a shorter measured offset stays measured; saved as rect_offset_cal=2, which a
// reboot reads back. Auto retries a bound only below Vin = 2 Vout and not within a week of the last one.
static void testBound() {
    struct Case { const char *name; int off; uint8_t lv; int want; uint8_t wantLv; bool saved; };
    const Case cases[] = {{"never calibrated", 0, 0, 0, 2, true}, {"backstop -60", -60, 0, -60, 2, true},
                          {"measured -60", -60, 1, -60, 1, false}, {"manual +40", 40, 1, 0, 2, true},
                          {"bound -60", -60, 2, -60, 2, true}};
    for (const auto &k: cases) {
        g_saved.clear(); g_saveCalls = 0; g_saveOk = true;
        Host h(3.f, -80, 0.f, 0.f, k.off);
        h.ctl.init(k.off, k.lv != 0);
        h.c.setRectOffsetCal(k.lv);
        g_ctl = &h.ctl; g_conv = &h.c;
        float truth;
        calibrate(h, truth);
        const RectCal &cal = h.ctl.cal;
        stopConv(h);
        idle(h, 15, 0.f);
        CHECK(cal.get() == RectCal::Done && cal.clamped && cal.learned > 20 && h.ctl.offset() == k.want
              && h.c.rectOffsetCal() == k.wantLv && h.ctl.activeCalibrated() && g_saved.size() == (size_t) k.saved
              && (!k.saved || (g_saved[0].first == k.want && g_saved[0].second == k.wantLv)),
              "bound, %s: %s learned %d, offset %d level %u, %zu saves", k.name, cal.get() == RectCal::Done ? "done" : cal.why,
              cal.learned, h.ctl.offset(), h.c.rectOffsetCal(), g_saved.size());
        if (!g_saved.empty()) rebootCheck(g_saved[0], k.name);
        printf("bound, %-16s: learned %+d (truth %+.0f), applied %+d, offset %+d, rect_offset_cal %u, %zu save(s)\n", k.name,
               cal.learned, truth, cal.applied, h.ctl.offset(), h.c.rectOffsetCal(), g_saved.size());
    }
    g_ctl = nullptr; g_conv = nullptr;

    SynchronousConverter c{}; initConv(c);
    c.setRectOffsetCal(2);
    Ctl ctl{};
    ctl.persist = fakePersist;
    for (float vr: {2.6f, 0.f, NAN}) {
        ctl.lf(c, 700, 30.f, vr); ctl.lf(c, 701, 30.f, vr);
        CHECK(ctl.cal.get() == RectCal::Idle && !ctl.startReq.load(), "bound: auto start at Vin/Vout %.1f", vr);
    }
    ctl.lf(c, 702, 30.f, 1.8f);
    CHECK(ctl.startReq.load(), "bound: no auto start at Vin/Vout 1.8");
    // a clamped fit pushes the next automatic start a week out
    ctl.startReq.store(false);
    RectCal late;
    fixture(late, 60, 26.7f, 60, 26.7f, 80.f);
    ctl.cal.n = late.n; ctl.cal.ideal = late.ideal; ctl.cal.hs = late.hs;
    std::copy(late.xs, late.xs + late.n, ctl.cal.xs); std::copy(late.ys, late.ys + late.n, ctl.cal.ys);
    ctl.cal.state.store(RectCal::Fit);
    ctl.lf(c, 800, 30.f, 1.8f);
    CHECK(ctl.cal.get() == RectCal::Done && ctl.cal.clamped, "bound fixture not clamped");
    ctl.lf(c, 801, 30.f, 1.8f); ctl.lf(c, 802 + Ctl::DayS, 30.f, 1.8f);
    CHECK(!ctl.startReq.load(), "bound: retried within a week");
    ctl.lf(c, 801 + Ctl::BoundRetryS, 30.f, 1.8f); ctl.lf(c, 802 + Ctl::BoundRetryS, 30.f, 1.8f);
    CHECK(ctl.startReq.load(), "bound: no retry after a week");
    printf("bound: auto retry only below Vin = 2 Vout, a week after a bound result\n");
}

// rectCalPersist's single rewrite: runtime values, legacy key dropped, other lines and comments kept.
static void testConfUpdate() {
    const char *path = "/tmp/rect-cal-test-coil.conf";
    FILE *f = fopen(path, "w");
    fputs("# coil\nL0=50e-6\nrect_offset=12\nrect_offset_ns=-200  # old\nrect_cal_auto=0\n", f);
    fclose(f);
    std::string ns = "-138", drv = "vconv";
    ConfFile w{path};
    CHECK(w.update({{"rect_offset_ns", ns.c_str()}, {"rect_offset_drv", drv.c_str()}, {"rect_offset_cal", "1"}},
                   "rect_offset"), "update failed");
    ConfFile r{path};
    CHECK(r.getString("rect_offset_ns", "") == "-138" && r.getString("rect_offset_drv", "") == "vconv"
          && r.getByte("rect_offset_cal", 0) == 1 && r.getString("rect_offset", "gone") == "gone"
          && r.getString("L0", "") == "50e-6" && r.getByte("rect_cal_auto", 1) == 0, "conf update content");
    f = fopen(path, "r");
    char buf[512] = {};
    fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    CHECK(strstr(buf, "# coil\n") && strstr(buf, "rect_offset_ns=-138  # old\n"), "conf update layout: %s", buf);
    remove(path);
}

// On-target sequence: rect-cal while converting, `dc 0` (manual ramp to pwmCtrlMin, then disable),
// LF every 3 s; saved within IdleUs plus one LF period, MPPT can start again right after. Also with
// core 0 stalled (console `sleep`) through the whole off time: saved in the first LF pass after it.
static void testDc0Save() {
    for (const double stall: {0.0, 25.0}) {
        g_saved.clear(); g_saveCalls = 0; g_saveOk = true; g_writes = g_poweredWrites = g_unadmitted = 0;
        Host h(3.f, 80, 0.f, 0.f);
        g_ctl = &h.ctl; g_conv = &h.c; g_writeHook = nullptr;
        float truth;
        CHECK(calibrate(h, truth), "cal failed: %s", h.ctl.cal.why);
        h.lfAt = 1e9;
        const int16_t step = (int16_t) std::max(1, (int) std::lround((float) h.c.pwmMaxDriver() / 512.f));
        while (!h.c.disabled()) { // updateManual() with target 0, one sample per ms
            h.r.t += 0.001;
            h.ctl.rt(h.c, h.in());
            if (h.c.getCtrlOnPwmCnt() <= h.c.pwmCtrlMin) h.c.disable();
            else h.c.pwmPerturb((int16_t) -step);
        }
        const double off = h.r.t;
        for (double t1 = h.r.t + stall; h.r.t < t1;) { h.r.t += 0.1; h.ctl.rt(h.c, h.in()); }
        while (h.ctl.unsaved() && h.r.t - off < 60) idle(h, 0.1, 0.f, 3.0);
        const double took = h.r.t - off;
        CHECK(!h.ctl.unsaved() && g_saved.size() == 1 && g_saved[0].first == h.ctl.offset() && g_saved[0].second,
              "dc 0 (stall %.0f s): unsaved after %.1f s (%zu saves)", stall, took, g_saved.size());
        CHECK(took <= std::max(stall, 10.0) + 3.0 + 0.2, "dc 0 (stall %.0f s): saved after %.1f s", stall, took);
        h.ctl.rt(h.c, h.in());
        CHECK(!h.c.startInhibited(), "dc 0: start still inhibited after the save");
        h.c.pwmPerturb(10);
        CHECK(!h.c.disabled() && g_poweredWrites == 0 && g_unadmitted == 0, "dc 0: restart %d powered %d unadmitted %d",
              !h.c.disabled(), g_poweredWrites, g_unadmitted);
        printf("dc 0 save: stall %.0f s -> saved %.1f s after the stop, restart ok\n", stall, took);
    }
    g_ctl = nullptr; g_conv = nullptr;
}

// R3-2: the backstop stays armed over long uptimes (2^31 and 2^32 us) and re-holds after a restart.
static void testBackstopUptime() {
    Host h(3.f, 80, 0.f, 0.f);
    h.op = (uint16_t) (0.6f * VBAT / VOC * PM);
    h.run(6);
    int fired = 0, n = 0;
    for (double at: {100.0, 2100.0, 2200.0, 4300.0, 4400.0, 8700.0}) {
        h.r.t = at;
        h.ctl.rt(h.c, h.in());
        ++n;
        fired += fireBackstop(h, 0);
    }
    CHECK(fired == n, "backstop fired %d of %d times over 8700 s uptime", fired, n);
    h.c.disable();
    h.ctl.rt(h.c, h.in());
    h.c.pwmPerturb(400);
    h.c.enableSyncRect(true);
    h.ctl.rt(h.c, h.in());
    CHECK(!fireBackstop(h, 1000000), "fired 1 s after a restart");
    CHECK(fireBackstop(h, 3000000), "not fired 3 s after a restart");
    printf("backstop uptime: %d/%d over 2^31/2^32 us, start-up hold after restart ok\n", fired, n);
}

// Finding 7: eligibility and the Vin-rise signature.
static void testBackstopEligibility() {
    RectBackstop bs{};
    bs.update(1000000, true, 1200, 1500, 64, 2.f, 2.f, 60.f);
    CHECK(!bs.update(1100000, true, 1201, 1500, 64, 1.9f, 0.1f, 60.f), "cloud-like drop (Vin flat) fired");
    CHECK(!bs.update(1200000, true, 1202, 1500, 64, 1.8f, 0.1f, 59.f), "cloud-like drop (Vin falling) fired");
    CHECK(bs.update(1300000, true, 1203, 1500, 64, 1.7f, 0.1f, 61.f), "collapse with Vin rise did not fire");
    bs.update(5000000, true, 1200, 1500, 64, 2.f, 2.f, 60.f);
    bs.update(5100000, false, 1200, 1500, 64, 2.f, 2.f, 60.f);
    CHECK(bs.peak == 0 && bs.peakHs == 0, "history kept across disarm");
    int f, off = 140, nonMppt = 0;
    for (float isc: {11.5f, 8.f, 6.f}) {
        sweepTrips(isc, off = 140, 160, 0.f, false, f);
        nonMppt += f;
    }
    CHECK(nonMppt == 0, "backstop fired outside MPPT mode (%d)", nonMppt);
    // start-up hold: no trigger within StartupUs of the converter starting (both nowUs parities)
    for (uint32_t odd: {0u, 1u}) {
        Host h(3.f, 80, 0.f, 0.f);
        h.op = (uint16_t) (0.6f * VBAT / VOC * PM);
        h.run(6);
        stopConv(h);
        h.c.pwmPerturb((int16_t) h.op);
        RectCalIn in = h.in();
        in.nowUs = h.r.us() + odd;
        h.ctl.rt(h.c, in); // converter seen on: start-up hold begins
        const bool early = fireBackstop(h, 1000000 - odd);
        CHECK(!early, "backstop fired %.1f s after start (parity %u)", 1.0, odd);
    }
    printf("backstop eligibility: ok\n");
}

// Finding 4: RT and core-0 threads overlapping start/abort/status/fit/apply/backstop (run under TSan).
static void testThreads(double seconds) {
    Host h(3.f, 80, 0.f, 0.f, 140);
    h.op = (uint16_t) (0.6f * VBAT / VOC * PM);
    h.ctl.autoRun = true;
    h.lfAt = 1e9; // LF runs on the other thread
    std::atomic<bool> stop{false};
    std::atomic<float> p{0};
    std::atomic<uint32_t> nowS{0};
    std::thread rt([&] {
        uint32_t cycles = 0;
        bool fired = false;
        while (!stop.load()) {
            if (h.r.t < 55) {
                h.tick();
                p.store(h.pOut());
                if (h.r.t > 45 && !fired && !h.ctl.cal.running()) fired = fireBackstop(h) || true; // RT shrink + publish
            } else { // "night": converter off, LF may save
                if (!h.c.disabled()) { h.c.disable(); h.sr = h.bflow = false; }
                h.ctl.rt(h.c, h.in());
                h.r.t += 1.0 / SPS;
                p.store(0.f);
            }
            nowS.store(cycles * 70 + (uint32_t) h.r.t);
            if (h.r.t > 70) { h.r.t = 0; ++cycles; fired = false; h.ctl.abortReq.store(true); }
        }
    });
    int starts = 0, n = 0, done = 0, failed = 0, fits = 0, saves = 0;
    uint8_t seen = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::duration<double>(seconds)) {
        fits += h.ctl.cal.get() == RectCal::Fit;
        const uint32_t s = nowS.load() + 600;
        h.ctl.lf(h.c, s, p.load());
        h.ctl.saveNow(h.c, s, [] { std::this_thread::sleep_for(std::chrono::microseconds(100)); });
        saves = g_saveCalls;
        const auto st = h.ctl.cal.get();
        if ((st == RectCal::Done || st == RectCal::Fail) && h.ctl.cal.seq != seen) {
            seen = h.ctl.cal.seq;
            (st == RectCal::Done ? done : failed)++;
        }
        if (++n % 50 == 0) starts += h.ctl.requestStart();
        if (n % 20 == 0) h.ctl.status(h.c);
        if (n % 30 == 0) h.ctl.post(-20, false);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    stop.store(true);
    rt.join();
    printf("threads: %d LF passes, %d requests, %d fits, %d done, %d failed/refused, backstop %u, saves %d\n", n,
           starts, fits, done, failed, h.ctl.fires.load(), saves);
}

int main(int argc, char **argv) {
    PWM_Mock::pwmMaxOverride = PM;
    if (argc == 7 && !strcmp(argv[1], "trace")) { // trace <isc> <hidden> <vbias> <offset> <backstop>
        g_trace = true; g_quiet = false;
        int f, off = atoi(argv[5]);
        bool t = sweepTrips(atof(argv[2]), off, atoi(argv[3]), atof(argv[4]), atoi(argv[6]), f);
        printf("%s fires=%d\n", t ? "TRIP" : "no trip", f);
        return 0;
    }
    if (argc == 5 && !strcmp(argv[1], "fixture")) { // fixture <vin0> <vin1> <timingErr>
        RectCal c;
        fixture(c, atof(argv[2]), 26.7f, atof(argv[3]), 26.7f, atof(argv[4]));
        printf("state %d why '%s' learned %d applied %d ideal %.1f peak %.1f:", c.get(), c.why, c.learned, c.applied,
               c.ideal, c.peak);
        for (int j = 0; j < c.n; ++j) printf(" %.0f:%+.3f", c.xs[j], c.ys[j]);
        printf("\n");
        return 0;
    }
    if (argc >= 2 && !strcmp(argv[1], "threads")) {
        testThreads(argc > 2 ? atof(argv[2]) : 5.0);
        return 0;
    }
    g_verbose = argc == 2 && !strcmp(argv[1], "-v");
    testDriverRule();
    testLateGap();
    testShrinkMonotone();
    testFieldData();
    testFlatField();
    testGlitch();
    testVoltageBaseline();
    testDisturbance();
    testField();
    testForce();
    testForceProtection();
    testFullSun();
    testRingBack();
    testDipRecovery();
    testProtectionWins();
    testAbortAndModes();
    testSchedule();
    testPersistence();
    testPersistLock();
    testBound();
    testDc0Save();
    testConfUpdate();
    testBackstopUptime();
    testBackstopEligibility();

    printf("hidden vBias calIsc drift | learned truth  err applied | stale140 applied (Isc 11.5/8/6) | backstop fires stale/applied\n");
    int scen = 0, staleTrips = 0, staleFires = 0, staleTripNoFire = 0, maxSweeps = 0;
    for (int hidden: {0, 40, 80, 120, 160})
        for (float vb: {0.f, -0.4f, -0.8f})
            for (float calIsc: {1.5f, 3.f})
                for (float drift: {0.f, 0.003f}) {
                    ++scen;
                    Host h(calIsc, hidden, vb, drift);
                    float truth = 0;
                    bool ok = calibrate(h, truth);
                    const RectCal &cal = h.ctl.cal;
                    noteRise("plant table 0.6M", cal);
                    CHECK(ok, "cal failed: %s (n=%d)", cal.why, cal.n);
                    if (!ok) continue;
                    const float err = (float) cal.learned - truth;
                    CHECK(std::fabs(err) <= 20.f, "learned %d truth %.0f", cal.learned, truth);
                    CHECK(cal.applied <= cal.learned - 20, "margin: applied %d learned %d", cal.applied, cal.learned);
                    char s1[4], s2[4];
                    int fs = 0, fa = 0;
                    for (int k = 0; k < 3; ++k) {
                        const float isc = k == 0 ? 11.5f : k == 1 ? 8.f : 6.f;
                        int f, off = 140;
                        bool st = sweepTrips(isc, off, hidden, vb, false, f);
                        s1[k] = st ? 'T' : '.';
                        staleTrips += st;
                        off = cal.applied;
                        bool ap = sweepTrips(isc, off, hidden, vb, false, f);
                        s2[k] = ap ? 'T' : '.';
                        CHECK(!ap, "applied %d trips at Isc %.1f", cal.applied, isc);
                        // Stale offset with the backstop, carried across repeated sweeps (field loop).
                        off = 140;
                        int sw = 0;
                        for (bool t = true; t && sw < 20; ++sw) {
                            t = sweepTrips(isc, off, hidden, vb, true, f);
                            fs += f; staleFires += f;
                            if (t && !f) ++staleTripNoFire;
                        }
                        maxSweeps = std::max(maxSweeps, sw);
                        CHECK(sw < 20, "backstop did not converge (off %d)", off);
                        off = cal.applied;
                        sweepTrips(isc, off, hidden, vb, true, f);
                        fa += f;
                        CHECK(f == 0, "backstop false positive (applied %d, Isc %.1f)", cal.applied, isc);
                    }
                    s1[3] = s2[3] = 0;
                    printf("%6d %5.1f %6.1f %5.3f | %7d %5.0f %+4.0f %7d | %s      %s | %d/%d\n", hidden, vb, calIsc, drift,
                           cal.learned, truth, err, cal.applied, s1, s2, fs, fa);
                    if (g_verbose) {
                        printf("   ideal %.0f peak %.0f hs %u dV %.2f%%:", cal.ideal, cal.peak, cal.hs, 100 * cal.dv);
                        for (int j = 0; j < cal.n; ++j) printf(" %.0f:%+.3f", cal.xs[j], cal.ys[j]);
                        printf("\n");
                    }
                }

    // Higher operating points (MPPT in DCM, near the CCM boundary) and weaker panels: the run ramps HS
    // down to its calibration point and back. A refusal keeps the old value; a completed run must be
    // accurate.
    for (float opFrac: {0.8f, 0.95f})
        for (float isc: {1.f, 3.f, 8.f})
            for (int hidden: {0, 80, 160})
                for (float vb: {0.f, -0.8f}) {
                    Host h(isc, hidden, vb, 0.003f);
                    float truth = 0;
                    const bool ok = calibrate(h, truth, opFrac);
                    const RectCal &cal = h.ctl.cal;
                    noteRise("plant 0.8-0.95M", cal);
                    // Isc 1 A: the knee lifts Vin 2-3.5 % (energy pumped back to the input); the knot fit then
                    // lands up to ~35 ct early, which shortens the LS (safe side)
                    if (ok)
                        CHECK(cal.learned - truth <= 20.f && cal.learned - truth >= (isc < 2 ? -35.f : -20.f),
                              "op %.2fM Isc %.0f: learned %d truth %.0f", opFrac, isc, cal.learned, truth);
                    printf("op %.2fM Isc %3.0f hidden %3d vb %4.1f: %-26s learned %4d truth %4.0f err %+4.0f (%d pts, "
                           "dV %.2f%%)\n", opFrac, isc, hidden, vb, ok ? "ok" : cal.why, cal.learned, truth,
                           ok ? cal.learned - truth : 0.f, cal.n, 100.f * cal.dv);
                }

    // Clean plant, zero offset: the backstop must stay quiet across sweeps.
    for (float isc: {11.5f, 8.f, 6.f, 4.f, 2.f}) {
        int f, off = 0;
        sweepTrips(isc, off, 0, 0.f, true, f);
        CHECK(f == 0, "backstop fired on clean sweep at Isc %.1f", isc);
    }
    CHECK(staleTrips > 0, "harness: stale offset never trips (known-bad not reproduced)");
    CHECK(staleTripNoFire == 0, "backstop missed %d tripping stale sweeps", staleTripNoFire);
    printf("scenarios=%d staleTrips=%d staleBackstopFires=%d staleTripsWithoutFire=%d maxSweepsToClear=%d\n",
           scen, staleTrips, staleFires, staleTripNoFire, maxSweeps);
    for (const auto &l: g_rise)
        printf("Iout rise before ideal, %-32s %3d runs: max %+5.1f mA, %3.0f%% of the bound, refused %d\n", l.name.c_str(),
               l.runs, 1e3f * l.rise, 100.f * l.frac, l.refused);
    printf("%s (fails=%d)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail != 0;
}
