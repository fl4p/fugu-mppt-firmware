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

void assertPinState(uint8_t, bool, const char *, bool) {}
static bool g_quiet = true, g_trace = false, g_verbose = false;
void UART_LOG(const char *fmt, ...) {
    if (g_quiet) return;
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap); putchar('\n');
}

static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { ++g_fail; printf("FAIL %s:%d %s  ", __FILE__, __LINE__, #c); printf(__VA_ARGS__); putchar('\n'); } } while (0)

static constexpr uint16_t PM = 4103;
static constexpr float SPS = 434.f, VOC = 69.4f, K = 0.78f, VBAT = 26.7f, LREAL = 50.9e-6f;
static constexpr uint32_t FSW = 39000;
using Ctl = RectCalCtl<SynchronousConverter>;

// Plant + sensors. The LS conducts `hidden` counts longer than commanded (gate/driver delay).
struct Rig {
    VirtualConverter p{};
    int hidden;
    float voutBias, isc0, drift;
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
    void step() { t += 1.0 / SPS; p.setPv(isc0 * (1.f + drift * (float) t), VOC, K); p.stepSeconds(1.f / SPS, FSW); }
    uint32_t us() const { return (uint32_t) (t * 1e6); }
    float iout() { return p.getIoutAvg() + 0.01f * nd(rng); }
    float vin() { return p.getVin() + 0.05f * nd(rng); }
    float vout() { return p.getVout() + voutBias + 0.01f * nd(rng); }
};

static void initConv(SynchronousConverter &c, const char *offNs = nullptr, const char *offDrv = nullptr,
                     const char *offCal = nullptr) {
    ConfFile conv{}; conv.set("topo", "buck");
    ConfFile board{}; board.set("pwm_freq", "39000"); board.set("pwm_driver_logic", "HiLi");
    board.set("pwm_hi", "21"); board.set("pwm_li", "14"); board.set("pwm_sd", "47"); board.set("skip_assert", "1");
    ConfFile coil{}; coil.set("L0", "56e-6");
    if (offNs) coil.set("rect_offset_ns", offNs);
    if (offDrv) coil.set("rect_offset_drv", offDrv);
    if (offCal) coil.set("rect_offset_cal", offCal);
    c.init(conv, board, coil);
}

static bool g_saveOk = true;
static int g_saveCalls = 0;
static std::vector<std::pair<int, bool>> g_saved;
static bool fakePersist(int o, bool cal) {
    ++g_saveCalls;
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
    float iEwm = 0, viEwm = VOC, voEwm = VBAT, last = 0, prev = 0, vi = VOC, vo = VBAT, io = 0;
    bool bflow = false, sr = false, mppt = true, sweeping = false, tripped = false, held = false;
    uint16_t op = 0, rampTo = 0;
    int lsAfterHold = -1, lsRaw = -1; // pinned LS / LS count right after hold()
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
                bflow, false, true};
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
        if ((last < -1 && prev < -1) || (sr && iF < -0.05f)) { bflow = false; c.syncRectMinDuty(); }
        if (fault) fault(*this);
        if (sr && !bflow && c.getDutyCycle() > 0.33f) { tripped = true; c.disable(); return; }

        uint16_t b = 0;
        held = ctl.hold(c, in(), 0.f, b);
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

static void testDriverRule() {
    SynchronousConverter c{};
    const int ct875 = (int) std::lround(875e-9 * 39000.0 * PM);
    initConv(c, "875", "mcpwm");
    CHECK(c.getRectOnOffset() == 0 && c.rectOffsetNeedsCal(), "foreign driver: off=%d", c.getRectOnOffset());
    initConv(c, "875", nullptr);
    CHECK(c.getRectOnOffset() == 0 && c.rectOffsetNeedsCal(), "no driver key: off=%d", c.getRectOnOffset());
    initConv(c, "875", "mock");
    CHECK(std::abs(c.getRectOnOffset() - ct875) <= 1 && !c.rectOffsetNeedsCal(), "same driver: off=%d want %d",
          c.getRectOnOffset(), ct875);
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
    int ls = -1;
    bool init = true;
    for (uint32_t us = 0; cal.running() && us < 40000000; us += 2304) {
        if (ls >= 0) init = false;
        const float vi = init ? vi0 : vi1, vo = init ? vo0 : vo1;
        const float zc = (vi / vo - 1.f) * hs + timingErr;
        const float x = ls < 0 ? 0.6f * zc : (float) ls;
        const float i = 2.f - (x < zc ? 1.5e-6f : 6e-6f) * (x - zc) * (x - zc); // asymmetric parabola
        ls = cal.feed(us, hs, vi, vo, i);
    }
    SynchronousConverter c{}; initConv(c);
    cal.fit(c.getPwmTickRate());
}

// Finding 1: a voltage step after Init must not become a timing correction.
static void testVoltageBaseline() {
    RectCal a, b, c, d;
    fixture(a, 60, 26.7f, 66, 26.7f);
    CHECK(a.get() == RectCal::Fail && !strcmp(a.why, "V shift"), "Vin 60->66: state %d why '%s' learned %d", a.get(),
          a.why, a.learned);
    fixture(b, 60, 26.7f, 60, 27.5f);
    CHECK(b.get() == RectCal::Fail && !strcmp(b.why, "V shift"), "Vout 26.7->27.5: state %d why '%s'", b.get(), b.why);
    RectCal c0;
    fixture(c0, 60, 26.7f, 60, 26.7f);
    fixture(c, 60, 26.7f, 60.9f, 26.7f); // 1.5 %: accepted, coordinate follows the step's own ideal (+40 ct)
    CHECK(c0.get() == RectCal::Done && c.get() == RectCal::Done && std::abs(c.learned - c0.learned) <= 5,
          "Vin +1.5%%: state %d learned %d, without step %d (%s)", c.get(), c.learned, c0.learned, c.why);
    fixture(d, 60, 26.7f, 60, 26.7f, -398.f); // far negative tail
    SynchronousConverter cv{}; initConv(cv);
    const int floor = RectCal::floorCounts(cv.getPwmTickRate());
    CHECK(d.get() == RectCal::Done && d.applied >= floor && RectBackstop::shrink(d.applied, 20, floor) <= d.applied,
          "learned %d applied %d floor %d", d.learned, d.applied, floor);
    printf("voltage baseline: Vin+10%% '%s', Vout+3%% '%s', Vin+1.5%% learned %d (no step %d), tail learned %d applied %d floor %d\n",
           a.why, b.why, c.learned, c0.learned, d.learned, d.applied, floor);
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
    RectCalIn in{1000, 60, 26.7f, 1, 60, 26.7f, 1, 1, 1, false, false, true, false, true}; // not MPPT: refused
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

// Stop the converter and let rt() publish it, as the RT loop does at night.
static void stopConv(Host &h) {
    h.c.disable();
    h.ctl.rt(h.c, h.in());
}

// What a reboot reads back from the last save.
static void rebootCheck(const std::pair<int, bool> &sv, const char *what) {
    SynchronousConverter rb{};
    char ns[16];
    initConv(rb);
    snprintf(ns, sizeof(ns), "%.0f", rectOffsetNsFromCounts(sv.first, rb.getPwmTickRate()));
    initConv(rb, ns, "mock", sv.second ? "1" : "0");
    CHECK(std::abs(rb.getRectOnOffset() - sv.first) <= 1 && rb.rectOffsetNeedsCal() == !sv.second,
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
    CHECK(calibrate(h, truth), "cal failed: %s", h.ctl.cal.why);
    CHECK(g_saveCalls == 0 && h.ctl.unsaved(), "saved while converting (%d calls)", g_saveCalls);
    uint32_t t = (uint32_t) h.r.t + 1;
    h.ctl.lf(h.c, t, 0.5f);
    h.ctl.lf(h.c, t + 20, 0.5f);
    CHECK(g_saveCalls == 0, "saved with the converter on at 0.5 W");
    stopConv(h);
    h.ctl.lf(h.c, t + 30, NAN);
    h.ctl.lf(h.c, t + 50, NAN);
    CHECK(g_saveCalls == 0, "saved with power NaN");
    h.ctl.lf(h.c, t + 60, 0.f);
    h.ctl.lf(h.c, t + 65, 0.f);
    CHECK(g_saveCalls == 0, "saved before IdleS off");
    g_saveOk = false;
    h.ctl.lf(h.c, t + 71, 0.f);
    CHECK(g_saveCalls == 1 && h.ctl.unsaved(), "failed save dropped (%d calls)", g_saveCalls);
    h.ctl.lf(h.c, t + 100, 0.f);
    CHECK(g_saveCalls == 1, "retry before 60 s");
    g_saveOk = true;
    h.ctl.lf(h.c, t + 132, 0.f);
    CHECK(g_saved.size() == 1 && g_saved[0].first == h.ctl.cal.applied && g_saved[0].second && !h.ctl.unsaved(),
          "calibrated save: %zu", g_saved.size());
    if (!g_saved.empty()) rebootCheck(g_saved[0], "calibrated");
    h.ctl.requestSave();
    h.ctl.lf(h.c, t + 200, 0.f);
    CHECK(g_saved.size() == 1, "second save within SaveMinS");
    h.ctl.lf(h.c, t + 132 + Ctl::SaveMinS, 0.f);
    CHECK(g_saved.size() == 2 && !h.ctl.unsaved(), "rate-limited save never came (%zu)", g_saved.size());

    // R2-1: fit -> one backstop reduction -> save -> reboot.
    g_saved.clear(); g_saveCalls = 0;
    Host m(3.f, 80, 0.f, 0.f);
    CHECK(calibrate(m, truth), "cal failed: %s", m.ctl.cal.why);
    const int fitted = m.ctl.cal.applied;
    CHECK(fireBackstop(m), "fixture backstop did not fire");
    t = (uint32_t) m.r.t + 1;
    m.ctl.lf(m.c, t, 30.f);
    stopConv(m);
    m.ctl.lf(m.c, t + 1, 0.f);
    m.ctl.lf(m.c, t + 12, 0.f);
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
        s.c.setRectOffsetNeedsCal(needs);
        s.sweeping = true;
        s.r.t = s.lfAt = 300.0 * sw; // monotonic time across sweeps
        while (s.r.t < 300.0 * sw + 8 && !s.tripped && s.c.getCtrlOnPwmCnt() / (float) PM < 1.08f * s.r.p.getVout() / s.r.p.getVin())
            s.tick();
        fires = shared.fires.load();
        if (fires == 1) CHECK(!shared.unsaved() && g_saved.empty(), "saved after one trigger");
        stopConv(s);
        shared.lf(s.c, 300 * sw + 100, 0.f);
        shared.lf(s.c, 300 * sw + 111, 0.f);
        needs = s.c.rectOffsetNeedsCal();
        off = shared.offset();
    }
    bool allUncal = true;
    for (auto &v: g_saved) allUncal &= !v.second;
    CHECK(fires >= 2 && !g_saved.empty() && allUncal && needs, "backstop save: fires %d saves %zu", fires, g_saved.size());
    if (!g_saved.empty()) rebootCheck(g_saved[0], "backstop");
    printf("persistence: backstop %d triggers -> saved %d ct uncalibrated (%zu save(s))\n", fires,
           g_saved.empty() ? 0 : g_saved[0].first, g_saved.size());
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
        h.ctl.lf(h.c, nowS.load() + 600, p.load());
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
    testShrinkMonotone();
    testVoltageBaseline();
    testProtectionWins();
    testAbortAndModes();
    testSchedule();
    testPersistence();
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
                    if (ok)
                        CHECK(std::fabs(cal.learned - truth) <= 20.f, "op %.2fM Isc %.0f: learned %d truth %.0f", opFrac,
                              isc, cal.learned, truth);
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
    printf("%s (fails=%d)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail != 0;
}
