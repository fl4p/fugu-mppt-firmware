// Host test for the Iout zero (offset) calibration in src/adc/sampling.h::handleSensorCalib.
// Replays a post-stop Iout trace at the INA226 rate and checks which zero is adopted.
//   test/host-stub/izero-test.sh            (new code)
//   IZERO_OLD=1 test/host-stub/izero-test.sh (same traces against HEAD's sampling.h)

#include "adc/sampling.h"

#include "sim_alloc.h"

#include <cstdio>
#include <random>

time_us loopWallClockUs_ = 0;
SimRtAlloc g_simRtAlloc{};

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
constexpr float kMaxAbs = 0.05f * 32; // limits Iout_max 32 A

// raw Iout t s after the stop: base + A e^(-t/tau) + noise of sd idle + B e^(-t/tauN)
struct Trace {
    float A, tau, B, tauN, drift = 0; // drift: A/s ramp that never settles
    float at(float t, std::mt19937 &r) const {
        std::normal_distribution<float> n(0.f, 1.f);
        return kBase + A * expf(-t / tau) + drift * t + n(r) * (kIdleSd + B * expf(-t / tauN));
    }
};

struct Result {
    float offset; // zero in use afterwards
    float ms;     // time from the calibration start until this sensor finished
    bool failed, done;
    float ewmStd;
};

// prevOffset: zero adopted by the previous calibration (0 = none, e.g. boot)
// preI: Iout before the stop (NAN = no samples yet, as at boot)
Result run(const Trace &tr, float prevOffset, float preI, int seed = 1, float maxS = 3.f) {
    NullADC adc;
    ADC_Sampler sampler{};
    auto *s = const_cast<Sensor *>(sampler.addSensor(
            &adc, SensorParams{0, {1.f, 0.f}, {kMaxAbs, .1f, true}, "iout", 'A', false}, 32.f, 20));
    s->calibrationAvg = prevOffset;
    s->calibrationComplete = prevOffset != 0;
    if (!std::isnan(preI))
        for (int i = 0; i < 50; ++i) s->add_sample(preI + kBase);
    loopWallClockUs_ = 1000000;
    sampler.startCalibration();
    sampler.update();
    std::mt19937 rng(seed);
    Result res{NAN, NAN, false, false, NAN};
    for (int i = 0; i < (int) (maxS * kFs); ++i) {
        const float t = (float) i / kFs;
        loopWallClockUs_ = 1000000 + (time_us) (t * 1e6f);
        s->add_sample(tr.at(t, rng));
        float std = s->ewm.std.get();
        auto r = sampler.handleSensorCalib(*s);
        if (r == ADC_Sampler::UpdateRet::CalibFailure) {
            res.failed = true;
            res.ms = t * 1e3f;
            break;
        }
        if (s->calibrationComplete) {
            res.done = true;
            res.ms = t * 1e3f;
            res.ewmStd = std;
            break;
        }
    }
    res.offset = s->calibrationAvg;
    return res;
}

void show(const char *name, const Result &r) {
    std::printf("%-36s offset %+.4f A  after %6.1f ms  ewm.std %.4f%s\n", name, r.offset, r.ms, r.ewmStd,
                r.failed ? "  CalibFailure" : r.done ? "" : "  NOT DONE");
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
    const float kOldMs = 200 * 1e3f / kFs + 1;     // HEAD: 100 samples settle + 101 averaged

    auto rb = run(idle, 0, NAN);
    show("boot (no samples, no previous zero)", rb);
    auto rd = run(idle, kBase, .5f);
    show("dawn restart from 0.5 A", rd);
    auto r22 = run(from22, kBase, 22);
    show("from 22 A, previous zero 0.012", r22);
    auto r22b = run(from22, 0, 22);
    show("from 22 A, no previous zero", r22b);
    auto r11 = run(from11, kBase, 11);
    show("from 11 A, previous zero 0.012", r11);
    auto rr = run(ramp, kBase, 22);
    show("never settles, previous 0.012", rr);
    auto rr0 = run(ramp, 0, 22);
    show("never settles, no previous", rr0);
    auto rf = run(flat, kBase, 11);
    show("flat +0.056 (not detectable)", rf);

#ifdef IZERO_OLD
    EXPECT(r22.done && r22.offset > .1f); // known-bad: HEAD adopts the 22 A tail
    EXPECT(r11.done && r11.offset > .05f);
#else
    const float errMax = .008f; // HEAD: 0.12 A
    EXPECT(rb.done && fabsf(rb.offset - kBase) < .004f && rb.ms < kOldMs);
    EXPECT(rd.done && fabsf(rd.offset - kBase) < .004f);
    EXPECT(rd.ms < kOldMs); // no wait after a low-current stop
    EXPECT(r22.done && fabsf(r22.offset - kBase) < errMax && r22.ms < 600.f);
    EXPECT(r22b.done && fabsf(r22b.offset - kBase) < errMax);
    EXPECT(r11.done && fabsf(r11.offset - kBase) < errMax);
    EXPECT(rr.done && rr.offset == kBase); // kept
    EXPECT(rr.ms >= 1200.f && rr.ms < 1200.f + 50 * 1e3f / kFs + 1);
    EXPECT(rr0.done && rr0.offset > kBase + .5f); // nothing to keep: latest block, logged

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
    // unevaluable input: non-finite samples never become the zero
    const Trace nanT{NAN, 1, 0, 1};
    auto rn = run(nanT, kBase, 22);
    show("NaN samples, previous 0.012", rn);
    EXPECT(rn.done && rn.offset == kBase);
    auto rn0 = run(nanT, 0, 22);
    show("NaN samples, no previous", rn0);
    EXPECT(rn0.failed && rn0.offset == 0);
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
    std::printf("idle noise, low-current stop: %d/100 done within HEAD's sample count\n", firstPair);
    EXPECT(firstPair >= 95);
#endif
    std::printf("%d/%d passed\n", g_run - g_fail, g_run);
    return g_fail != 0;
}
