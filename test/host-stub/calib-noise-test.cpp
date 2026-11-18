// Host test for the calibration noise check in src/adc/sampling.h::handleSensorCalib (maxStddev, sensor units).
// Drives the real ADC_Sampler calibration with quantised Gaussian noise and checks pass/fail.
//   test/host-stub/calib-noise-test.sh
//   CALIB_REF=<rev> test/host-stub/calib-noise-test.sh   (same cases against another revision, results printed)

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

constexpr float kFs = 1470.f, kQ = .0025f; // INA226 current LSB order

enum class Res { Pass, Fail, Open };

// one calibration of a fresh sensor: mean + N(0, sd) rounded to kQ (+ slope in units/s)
Res calibrate(CalibrationConstraints c, float mean, float sd, int seed, float slope = 0) {
    NullADC adc;
    ADC_Sampler sampler{};
    auto s = const_cast<Sensor *>(sampler.addSensor(&adc, SensorParams{0, {1.f, 0.f}, c, "x", ' ', false}, 100.f, 10));
    loopWallClockUs_ = 1000000;
    sampler.startCalibration();
    sampler.update();
    std::mt19937 rng(seed);
    std::normal_distribution<float> n(0.f, 1.f);
    for (int i = 0; i < (int) (3 * kFs); ++i) {
        const float t = (float) i / kFs;
        loopWallClockUs_ = 1000000 + (time_us) (t * 1e6f);
        const float x = roundf((mean + slope * t + sd * n(rng)) / kQ) * kQ;
        auto r = sampler._addSensorSample(s, x);
        if (r == ADC_Sampler::UpdateRet::CalibFailure) return Res::Fail;
        if (s->calibrationComplete && !sampler.isCalibrating()) return Res::Pass;
    }
    return Res::Open;
}

int count(CalibrationConstraints c, float mean, float sd, int seeds, Res want, float slope = 0) {
    int k = 0;
    for (int seed = 1; seed <= seeds; ++seed) k += calibrate(c, mean, sd, seed, slope) == want;
    return k;
}
} // namespace

int main() {
    const CalibrationConstraints iout{.maxAbsValue = 1.6f, .maxStddev = .1f, .calibrateOffset = true},
            zeroPlain{.maxAbsValue = 1.6f, .maxStddev = .1f, .calibrateOffset = false},
            vin{.maxAbsValue = 80.f, .maxStddev = 1.8f, .calibrateOffset = false};
    constexpr int kSeeds = 20;

    for (float despike: {0.f, 8.f}) {
        Sensor::despikeK = despike;
        std::printf("despike=%.0f\n", despike);

        // (a) zero-mean quantised noise well below maxStddev at an offset of ~0 passes
        for (float mean: {0.f, .004f, -.004f}) {
            const int pi = count(iout, mean, .03f, kSeeds, Res::Pass), pz = count(zeroPlain, mean, .03f, kSeeds, Res::Pass);
            std::printf("  (a) mean %+.3f sd 0.03: offset sensor %2d/%d pass, plain sensor %2d/%d pass\n", mean, pi,
                        kSeeds, pz, kSeeds);
            EXPECT(pi == kSeeds);
            EXPECT(pz == kSeeds);
        }
        const int pv = count(vin, 60.f, .05f, kSeeds, Res::Pass), pr = count(vin, 65.f, .1f, kSeeds, Res::Pass, .03f);
        std::printf("  (a) vin 60 V sd 0.05: %d/%d pass; 65 V ramp sd 0.1: %d/%d pass\n", pv, kSeeds, pr, kSeeds);
        EXPECT(pv == kSeeds);
        EXPECT(pr == kSeeds);

        // (b) noise above maxStddev fails
        for (float sd: {.15f, .3f}) {
            const int fi = count(iout, 0, sd, kSeeds, Res::Fail), fz = count(zeroPlain, 0, sd, kSeeds, Res::Fail);
            std::printf("  (b) sd %.2f > 0.1: offset sensor %2d/%d fail, plain sensor %2d/%d fail\n", sd, fi, kSeeds,
                        fz, kSeeds);
            EXPECT(fi == kSeeds);
            EXPECT(fz == kSeeds);
        }
        const int fv = count(vin, 60.f, 2.5f, kSeeds, Res::Fail);
        std::printf("  (b) vin 60 V sd 2.5 > 1.8: %d/%d fail\n", fv, kSeeds);
        EXPECT(fv == kSeeds);

        // (c) monotonic in the noise: once a seed fails it never passes again, far tail included
        int flips = 0, open = 0;
        const CalibrationConstraints cs[] = {zeroPlain, iout};
        for (int seed = 1; seed <= kSeeds; ++seed) {
            bool failed[2] = {};
            for (float sd: {.0f, .01f, .03f, .05f, .07f, .09f, .11f, .13f, .2f, .5f, 1.f, 3.f, 10.f, 100.f, 1e4f}) {
                for (int i = 0; i < 2; ++i) {
                    const Res r = calibrate(cs[i], 0, sd, seed);
                    open += r == Res::Open;
                    if (r != Res::Pass) failed[i] = true;
                    else if (failed[i]) ++flips;
                }
            }
        }
        std::printf("  (c) sd 0..1e4 x %d seeds: %d pass-after-no-pass flips, %d still open\n", kSeeds, flips, open);
        EXPECT(flips == 0);
    }

#ifndef CALIB_OLD
    // unevaluable: no sample pair is NaN, and NaN / inf never pass
    EXPECT(std::isnan(ADC_Sampler::calibNoiseSd(0, 0)));
    EXPECT(!ADC_Sampler::calibNoiseOk(NAN, .1f));
    EXPECT(!ADC_Sampler::calibNoiseOk(INFINITY, .1f));
    EXPECT(ADC_Sampler::calibNoiseOk(ADC_Sampler::calibNoiseSd(0, 100), .1f));
#endif

    std::printf("%d/%d passed\n", g_run - g_fail, g_run);
    return g_fail ? 1 : 0;
}
