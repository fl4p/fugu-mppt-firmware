// Host test for the Iout zero (offset) calibration in src/adc/sampling.h::handleSensorCalib.
// Replays a post-stop Iout trace at the INA226 rate and checks which zero is adopted.
//   test/host-stub/izero-test.sh                        (working tree)
//   IZERO_OLD=1 test/host-stub/izero-test.sh            (same traces against ca1a1da^, before the settle check)
//   IZERO_OLD=1 IZERO_REF=<rev> test/host-stub/izero-test.sh  (another revision, results printed only)

#include "adc/sampling.h"

#include "sim_alloc.h"

#include <cstdio>
#include <cstring>
#include <random>
#include <string>

time_us loopWallClockUs_ = 0;
SimRtAlloc g_simRtAlloc{};
extern void (*g_simLogHook)(char lvl, const char *tag, const char *msg);
static std::string g_lastWarn;

namespace {
int g_run = 0, g_fail = 0;
#define EXPECT(c) do{ ++g_run; if(!(c)){ ++g_fail; std::printf("  FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);} }while(0)

struct NullADC : public AsyncADC<float> {
    [[nodiscard]] AdcReadMode readMode() const override { return AdcReadMode::SnapshotAllChannels; }
    bool init(const ConfFile &) override { return true; }
    void deinit() override {}
    void startReading(uint8_t) override {}
    bool hasData() override { return false; }
    float getSample() override { return NAN; }
    void setMaxExpectedVoltage(uint8_t, float) override {}
    float getInputImpedance(uint8_t) override { return 1e6f; }
    float getSamplingRate(uint8_t) override { return 1470.f; }
};

constexpr float kFs = 1470.f, kBase = 0.012f, kIdleSd = 0.011f; // fits the 11 A log line (ewm.std 0.0067)
float kMaxAbs = 0.05f * 32; // limits Iout_max 32 A

// raw Iout t s after the stop: base + A e^(-t/tau) + noise of sd idle + B e^(-t/tauN), rounded to q
struct Trace {
    float A, tau, B, tauN, drift = 0, sd = kIdleSd, q = 0, base = kBase; // drift: A/s ramp that never settles
    float at(float t, std::mt19937 &r) const {
        std::normal_distribution<float> n(0.f, 1.f);
        float x = base + A * expf(-t / tau) + drift * t + n(r) * (sd + B * expf(-t / tauN));
        return q > 0 ? roundf(x / q) * q : x;
    }
};

struct Result {
    float offset; // zero in use afterwards
    float ms;     // time from the stop until this sensor finished
    bool failed, done;
    float ewmStd;
    int rejected; // samples refused at the sampler input
};

// one sensor + sampler kept across calibrations (zero, unsettled count)
struct Rig {
    NullADC adc;
    ADC_Sampler sampler{};
    Sensor *s;
    time_us t0 = 1000000;

    // prevOffset: zero adopted before (NAN = none, as at boot)
    explicit Rig(float prevOffset = NAN) {
        s = const_cast<Sensor *>(sampler.addSensor(
                &adc, SensorParams{0, {1.f, 0.f}, {kMaxAbs, .1f, true}, "iout", 'A', false}, 32.f, 20));
#ifdef IZERO_OLD
        s->calibrationAvg = std::isnan(prevOffset) ? 0.f : prevOffset; // 0 meant "none" there
        s->calibrationComplete = s->calibrationAvg != 0;
#else
        s->calibrationAvg = prevOffset;
        s->calibrationComplete = !std::isnan(prevOffset);
#endif
    }

    void start() {
        loopWallClockUs_ = t0;
        sampler.startCalibration();
        sampler.update();
    }

    // preI: Iout before the stop (NAN = no samples yet); restartMs: a second startCalibration that long after
    Result calibrate(const Trace &tr, float preI, int seed = 1, float maxS = 3.f, float restartMs = -1,
                     float restartEveryMs = 0, float restartUntilMs = 0) {
        if (!std::isnan(preI))
            for (int i = 0; i < 50; ++i) s->add_sample(preI + kBase);
        start();
        std::mt19937 rng(seed);
        Result res{NAN, NAN, false, false, NAN, 0};
        float nextRestart = restartMs;
        for (int i = 0; i < (int) (maxS * kFs); ++i) {
            const float t = (float) i / kFs;
            loopWallClockUs_ = t0 + (time_us) (t * 1e6f);
            if (nextRestart >= 0 && t * 1e3f >= nextRestart) {
                sampler.startCalibration();
                sampler.update();
                nextRestart = restartEveryMs > 0 && nextRestart + restartEveryMs <= restartUntilMs
                              ? nextRestart + restartEveryMs : -1;
            }
            const float std = s->ewm.std.get();
            auto r = sampler._addSensorSample(s, tr.at(t, rng));
            if (r == ADC_Sampler::UpdateRet::CalibFailure) {
                if (sampler.isCalibrating()) { ++res.rejected; continue; }
                res.failed = true;
                res.ms = t * 1e3f;
                break;
            }
            if (s->calibrationComplete && !sampler.isCalibrating()) {
                res.done = true;
                res.ms = t * 1e3f;
                res.ewmStd = std;
                break;
            }
        }
        res.offset = s->calibrationAvg;
        t0 += (time_us) (maxS * 1e6f) + 10000000;
        if (sampler.isCalibrating()) { // leave no calibration open for the next one
            sampler.cancelCalibration();
            sampler.update();
        }
        return res;
    }
};

Result run(const Trace &tr, float prevOffset, float preI, int seed = 1, float maxS = 3.f, float restartMs = -1) {
    Rig rig(prevOffset);
    return rig.calibrate(tr, preI, seed, maxS, restartMs);
}

void show(const char *name, const Result &r) {
    std::printf("%-40s offset %+.4f A  after %6.1f ms  ewm.std %.4f%s", name, r.offset, r.ms, r.ewmStd,
                r.failed ? "  CalibFailure" : r.done ? "" : "  NOT DONE");
    if (r.rejected) std::printf("  (%d samples rejected)", r.rejected);
    std::printf("\n");
}
} // namespace

int main() {
    const Trace idle{0, 1, 0, 1};
    // fit to the 10:33 log line (from 22 A: avg 0.1198, std 0.0635 at 1470 sps)
    const Trace from22{1.24f, .040f, 1.f, .040f};
    // 10:03 (from 11 A: avg 0.068, std 0.0067 = idle noise)
    const Trace from11{.62f, .040f, 0, 1};
    const Trace ramp{0, 1, 0, 1, 1.f};             // never settles
    const Trace flat{.056f, 1000.f, 0, 1};         // load-dependent offset, flat within 1 s (e.g. thermal)
    const float kOldMs = 200 * 1e3f / kFs + 1;     // before the settle check: 100 samples settle + 101 averaged
    const float errMax = .008f;                    // before the settle check: 0.12 A

    auto rb = run(idle, NAN, NAN);
    show("boot (no samples, no previous zero)", rb);
    auto rd = run(idle, kBase, .5f);
    show("dawn restart from 0.5 A", rd);
    auto r22 = run(from22, kBase, 22);
    show("from 22 A, previous zero 0.012", r22);
    auto r22b = run(from22, NAN, 22);
    show("from 22 A, no previous zero", r22b);
    auto r11 = run(from11, kBase, 11);
    show("from 11 A, previous zero 0.012", r11);
    auto rr = run(ramp, kBase, 22);
    show("never settles, previous 0.012", rr);
    auto rr0 = run(ramp, NAN, 22);
    show("never settles, no previous", rr0);
    auto rf = run(flat, kBase, 11);
    show("flat +0.056 (not detectable)", rf);

    // (a) restart 50 ms after a loaded stop: the EWM holds only tail samples then
    float eRe = 0, msRe = 1e9f;
    for (int seed = 1; seed <= 20; ++seed) {
        auto r = run(from22, kBase, 22, seed, 3.f, 50.f);
        eRe = fmaxf(eRe, fabsf(r.offset - kBase)), msRe = fminf(msRe, r.done ? r.ms : 1e9f);
    }
    std::printf("(a) restart 50 ms after a 22 A stop, 20 seeds: err %.4f A, first done %.0f ms after the stop\n",
                eRe, msRe);
    // restart cascade from a loaded stop (every 100 ms for 2 s): must still finish after the last one
    Rig rc(kBase);
    auto rcas = rc.calibrate(from22, 22, 1, 4.f, 100.f, 100.f, 2000.f);
    show("(a) restart every 100 ms until 2 s", rcas);
    // an unloaded restart well after a loaded calibration does not wait
    Rig rl(kBase);
    rl.calibrate(from22, 22);
    auto rlate = rl.calibrate(idle, .5f);
    show("(a) unloaded stop after a loaded one", rlate);

    // (b) a zero of exactly 0 is a zero, not "none"
    auto rz = run(ramp, 0.f, 22);
    show("(b) never settles, previous zero 0.0000", rz);

    // (c) a stage of huge (finite, squares overflow) samples, cancelled, then a clean calibration
    Trace huge{0, 1, 0, 1, 0, 0, 0, 3e19f};
    Rig rh(kBase);
    auto rh1 = rh.calibrate(huge, NAN, 1, .2f);
    show("(c) 3e19 samples, cancelled after 0.2 s", rh1);
    auto rh2 = rh.calibrate(idle, .5f);
    show("(c) then idle", rh2);
    Trace infT{INFINITY, 1, 0, 1};
    auto ri = run(infT, kBase, 22, 1, 1.f);
    show("(c) +inf samples, previous 0.012", ri);

#ifdef IZERO_OLD
#ifdef IZERO_PRE
    EXPECT(r22.done && r22.offset > .1f); // known-bad: before the settle check the 22 A tail is adopted
    EXPECT(r11.done && r11.offset > .05f);
#endif
#else
    EXPECT(rb.done && fabsf(rb.offset - kBase) < .004f && rb.ms < kOldMs);
    EXPECT(rd.done && fabsf(rd.offset - kBase) < .004f);
    EXPECT(rd.ms < kOldMs); // no wait after a low-current stop
    EXPECT(r22.done && fabsf(r22.offset - kBase) < errMax && r22.ms < 600.f);
    EXPECT(r22b.done && fabsf(r22b.offset - kBase) < errMax);
    EXPECT(r11.done && fabsf(r11.offset - kBase) < errMax);
    EXPECT(rr.done && rr.offset == kBase); // kept
    EXPECT(rr.ms >= 1200.f && rr.ms < 1200.f + 50 * 1e3f / kFs + 1);
    EXPECT(rr0.done && rr0.offset > kBase + .5f); // nothing to keep: latest block, logged

    EXPECT(eRe < errMax && msRe >= 400.f);
    EXPECT(rcas.done && fabsf(rcas.offset - kBase) < errMax && rcas.ms < 2000.f + 1250.f);
    EXPECT(rlate.done && rlate.ms < kOldMs);
    EXPECT(rz.done && rz.offset == 0.f);
    EXPECT(!rh1.done && rh1.offset == kBase);
    EXPECT(rh2.done && fabsf(rh2.offset - kBase) < .004f && rh2.ms < kOldMs);
    EXPECT(!ri.done && ri.rejected > 0 && ri.offset == kBase);

    std::printf("far tail, 20 seeds each (A: tail amplitude, B: tail noise):\n");
    for (float k: {1.f, 4.f, 16.f, 64.f, 256.f, 1024.f}) {
        Trace a{1.24f * k, .040f, 1.f, .040f}, b{1.24f * k, .040f, 1.f * k, .040f}, c{1.24f * k, .3f, 1.f * k, .3f};
        float e[3] = {}, ms[3] = {};
        int kept[3] = {}, fail[3] = {};
        for (int seed = 1; seed <= 20; ++seed) {
            const Result r[3] = {run(a, kBase, 22, seed), run(b, kBase, 22, seed), run(c, kBase, 22, seed)};
            for (int i = 0; i < 3; ++i) {
                EXPECT(r[i].done || r[i].failed);
                e[i] = fmaxf(e[i], fabsf(r[i].offset - kBase)), ms[i] = fmaxf(ms[i], r[i].ms);
                kept[i] += r[i].offset == kBase, fail[i] += r[i].failed;
            }
        }
        std::printf("  A x%-5.0f| tau 40 ms: err %.4f A %4.0f ms | +B x%-5.0f: err %.4f A %4.0f ms"
                    " | tau 300 ms: err %.4f A %4.0f ms kept %d/20\n",
                    k, e[0], ms[0], k, e[1], ms[1], e[2], ms[2], kept[2]);
        EXPECT(e[0] < errMax);
        EXPECT(e[2] < errMax);
        if (k <= 64) EXPECT(e[1] < errMax);
    }
    // unevaluable input: non-finite samples are refused at the input, never become the zero
    const Trace nanT{NAN, 1, 0, 1};
    auto rn = run(nanT, kBase, 22, 1, 1.f);
    show("NaN samples, previous 0.012", rn);
    EXPECT(!rn.done && rn.rejected > 0 && rn.offset == kBase);
    auto rn0 = run(nanT, NAN, 22, 1, 1.f);
    show("NaN samples, no previous", rn0);
    EXPECT(!rn0.done && std::isnan(rn0.offset));
    // too few samples: nothing adopted, calibration still running
    auto rs = run(from22, kBase, 22, 1, .5f);
    show("only 0.5 s of samples", rs);
    EXPECT(!rs.done && !rs.failed && rs.offset == kBase);
    // stationary idle noise: no extra time at a low-current stop (a rare false reject costs one block)
    int firstPair = 0;
    for (int seed = 1; seed <= 100; ++seed) {
        auto q = run(idle, kBase, .5f, seed);
        EXPECT(q.done && q.ms < 300.f && fabsf(q.offset - kBase) < .004f);
        firstPair += q.ms < kOldMs;
    }
    std::printf("idle noise, low-current stop: %d/100 done within the old sample count\n", firstPair);
    EXPECT(firstPair >= 95);
#endif

    // (d) quantised low noise: 5 mA ADC step, small Iout_max (noise-ratio floor maxAbs/1000 below the step):
    // block sd reads 0 in one block and a few mA in the next
    for (float imax: {10.f, 5.f})
    for (float sd: {.0003f, .0005f, .0007f, .001f}) {
        kMaxAbs = .05f * imax;
        Trace qt = idle;
        qt.sd = sd, qt.q = .005f, qt.base = .011f;
        int kept = 0;
        float e = 0, msMax = 0;
        for (int seed = 1; seed <= 200; ++seed) {
            auto r = run(qt, .1f, .1f, seed);
            msMax = fmaxf(msMax, r.ms);
            kept += r.offset == .1f;
            if (r.offset != .1f) e = fmaxf(e, fabsf(r.offset - qt.base));
        }
        std::printf("(d) Iout_max %2.0f A, 5 mA step, %.1f mA noise, 200 seeds: slowest %4.0f ms, %d kept the old zero,"
                    " err %.4f A\n", imax, sd * 1e3f, msMax, kept, e);
        kMaxAbs = .05f * 32;
#ifndef IZERO_OLD
        EXPECT(kept == 0 && e < .004f && msMax < kOldMs + 50 * 1e3f / kFs);
#endif
    }
#ifndef IZERO_OLD
    // (d) noise above the cap (0.08 A here): kept, counted, then the mean is taken without the noise checks
    {
        Trace noisy = idle;
        noisy.sd = .12f;
        g_simLogHook = [](char lvl, const char *, const char *msg) {
            if (lvl == 'W' && strstr(msg, " zero ")) std::printf("    log: %s\n", (g_lastWarn = msg).c_str());
        };
        Rig rn(kBase);
        int i = 0;
        Result r{};
        uint8_t miss[5] = {};
        for (; i < 5; ++i) {
            r = rn.calibrate(noisy, .5f, i + 1);
            miss[i] = rn.s->zeroMiss;
            std::printf("(d) noise 0.12 A, calibration %d: offset %+.4f A after %.0f ms, unsettled in a row %u%s\n",
                        i + 1, r.offset, r.ms, miss[i], r.failed ? "  CalibFailure" : "");
        }
        g_simLogHook = nullptr;
        EXPECT(miss[0] == 1 && miss[1] == 2 && miss[2] == 3 && miss[3] == 0 && miss[4] == 1);
        EXPECT(g_lastWarn.find("not settled #1: noise") != std::string::npos);
        // the 4th took the mean of ~1.2 s (sd/sqrt(1700) = 3 mA)
        Rig r4(kBase);
        for (int k = 0; k < 3; ++k) r4.calibrate(noisy, .5f, k + 1);
        auto ra = r4.calibrate(noisy, .5f, 4);
        EXPECT(ra.done && ra.offset != kBase && fabsf(ra.offset - kBase) < .012f);
        // a drifting mean is still refused after the misses
        Rig rd2(kBase);
        for (int k = 0; k < 4; ++k) rd2.calibrate(ramp, 22, k + 1);
        EXPECT(rd2.s->calibrationAvg == kBase && rd2.s->zeroMiss == 4);
        std::printf("(d) never-settling ramp: still kept after %u in a row\n", rd2.s->zeroMiss);
    }
#endif

    // slow tails that fit the logged 22 A line as well as the 40 ms one; the remaining error is not bounded by the
    // settle check (it sees a mean that moves less than its tolerance per block)
    std::printf("slow tails from 22 A, 20 seeds (error = adopted - true zero; tail = A e^-t/tau at the decision):\n");
    struct { float A, tau; } slow[] = {{.06f, 2.f}, {.075f, .5f}, {.1f, .2f}};
    for (auto &sl: slow) {
        Trace tr{sl.A, sl.tau, 0, 1};
        float eMin = 1, eMax = 0, msMax = 0;
        int kept = 0, done = 0;
        for (int seed = 1; seed <= 20; ++seed) {
            auto r = run(tr, kBase, 22, seed);
            done += r.done;
            if (r.offset == kBase) { ++kept; continue; }
            const float e = r.offset - kBase;
            eMin = fminf(eMin, e), eMax = fmaxf(eMax, e), msMax = fmaxf(msMax, r.ms);
        }
        std::printf("  A=%.3f A tau=%.1f s: done %d/20, kept %d/20, error %+.4f..%+.4f A (by %.0f ms; tail then %.4f A)\n",
                    sl.A, sl.tau, done, kept, kept < 20 ? eMin : 0.f, kept < 20 ? eMax : 0.f, msMax,
                    sl.A * expf(-msMax / 1e3f / sl.tau));
#ifndef IZERO_OLD
        EXPECT(done == 20);
#endif
    }
    std::printf("  KNOWN LIMITATION: these errors remain; the real tail shape is unknown until a raw capture across a\n"
                "  loaded stop. Without lengthening every stop they could shrink by bounding how far a loaded\n"
                "  calibration may move the zero from the last unloaded one (boot, dawn), by extrapolating the block\n"
                "  means once the tail shape is known, or by waiting longer only while the block means still trend.\n");

    std::printf("%d/%d passed\n", g_run - g_fail, g_run);
    return g_fail != 0;
}
