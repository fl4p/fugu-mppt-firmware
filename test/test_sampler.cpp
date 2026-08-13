#include <unity.h>
#include "adc/sampling.h"

#include "adc/adc_dummy.h"

void test_LinearTransform() {
    {
        LinearTransform t0{10, 0};
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f * 10.f, t0.apply(1.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.f * 10.f, t0.apply(5.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f / 10.f, t0.apply_inverse(1.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.f / 10.f, t0.apply_inverse(5.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f, t0.apply(t0.apply_inverse(1.f)));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, t0.apply(t0.apply_inverse(2.f)));
    }

    {
        LinearTransform t1{10, 1};
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, t1.apply(1.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, (5.f - 1) * 10.f, t1.apply(5.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f / 10.f + 1, t1.apply_inverse(1.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f, t1.apply(t1.apply_inverse(1.f)));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, t1.apply(t1.apply_inverse(2.f)));
    }
}

#if 0
void test_ADCSampler() {

    std::vector<std::vector<float>> samples{
            {12.4f, 12.3f, 12.2f, 12.15f},
            {0.f,   0.1f,  0.2f,  0.25f},
    };
    ADC_Dummy adc{samples};

    adc.startReading(0);
    TEST_ASSERT_TRUE(adc.hasData());
    TEST_ASSERT_EQUAL(12.4f, adc.getSample());

    adc.startReading(1);
    TEST_ASSERT_TRUE(adc.hasData());
    TEST_ASSERT_EQUAL(0.f, adc.getSample());

    adc.startReading(0);
    TEST_ASSERT_TRUE(adc.hasData());
    TEST_ASSERT_EQUAL(12.3f, adc.getSample());

    adc.startReading(1);
    TEST_ASSERT_TRUE(adc.hasData());
    TEST_ASSERT_EQUAL(0.1f, adc.getSample());

    adc.resetPointer();

    ADC_Sampler sampler{};
    sampler.setADC(&adc);
    auto &sensorU(*sampler.addSensor(0, {1, 0}, 15, {15, 1, false}, "u"));
    auto &sensorI(*sampler.addSensor(1, {2.5, 0}, 1, {1, 1, false}, "i"));
    sampler.begin(1);

    TEST_ASSERT_EQUAL(0, sensorU.adcCh);
    TEST_ASSERT_EQUAL(1, sensorI.adcCh);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 15, adc.maxExpectedVoltages[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f / 2.5f, adc.maxExpectedVoltages[1]);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_EQUAL(1, sensorU.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.4f, sensorU.last);
    TEST_ASSERT_EQUAL(0, sensorI.numSamples);
    TEST_ASSERT_FLOAT_IS_NAN(sensorI.last);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_EQUAL(1, sensorU.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.4f, sensorU.last);
    TEST_ASSERT_EQUAL(1, sensorI.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sensorI.transform.apply(.0f), sensorI.last);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_EQUAL(2, sensorU.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.3f, sensorU.last);
    TEST_ASSERT_EQUAL(1, sensorI.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sensorI.transform.apply(.0f), sensorI.last);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_EQUAL(2, sensorU.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.3f, sensorU.last);
    TEST_ASSERT_EQUAL(2, sensorI.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sensorI.transform.apply(.1f), sensorI.last);

    sampler.startCalibration();
    TEST_ASSERT_TRUE(sampler.isCalibrating());

    TEST_ASSERT_EQUAL(0, sensorU.numSamples);
    TEST_ASSERT_FLOAT_IS_NAN(sensorU.last);
    TEST_ASSERT_FLOAT_WITHIN(0, 0, sensorU.calibrationAvg);
    TEST_ASSERT_EQUAL(0, sensorI.numSamples);
    TEST_ASSERT_FLOAT_IS_NAN(sensorI.last);
    TEST_ASSERT_FLOAT_WITHIN(0, 0, sensorI.calibrationAvg);


    adc.resetPointer();
    TEST_ASSERT_FALSE(sampler.update());
    TEST_ASSERT_FALSE(sampler.update());

    TEST_ASSERT_FALSE(sampler.update());
    TEST_ASSERT_FALSE(sampler.update());

    TEST_ASSERT_FALSE(sampler.update());
    TEST_ASSERT_TRUE(sampler.update());

    struct Med3Ewm {
        RunningMedian3<float> med3;
        EWM<float> ewm;

        Med3Ewm(int span) : ewm(span) {}

        void add(float v) { ewm.add(med3.next(v)); }
    };

    Med3Ewm ewmU{1}, ewmI{1};
    ewmU.add(samples[0][0]);
    ewmU.add(samples[0][1]);
    ewmU.add(samples[0][2]);
    ewmI.add(sensorI.transform.apply(samples[1][0]));
    ewmI.add(sensorI.transform.apply(samples[1][1]));
    ewmI.add(sensorI.transform.apply(samples[1][2]));

    TEST_ASSERT_FLOAT_WITHIN(1e-6, ewmU.ewm.avg.get(), sensorU.calibrationAvg);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, ewmI.ewm.avg.get(), sensorI.calibrationAvg);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_TRUE(sampler.update());

    ewmU.add(samples[0][3]);
    ewmI.add(sensorI.transform.apply(samples[1][3]));

    TEST_ASSERT_FLOAT_WITHIN(1e-6, ewmI.ewm.std.get(), sensorI.ewm.std.get());
}
#endif

namespace {
// Muxed ADC mock (one channel per poll) that records the startReading() sequence, so the Vout
// interleave in ADC_Sampler can be tested without ADS1x15 hardware.
class MuxRrMockADC : public AsyncADC<float> {
public:
    std::vector<uint8_t> reads;

    [[nodiscard]] AdcReadMode readMode() const override { return AdcReadMode::MuxedRoundRobin; }
    bool init(const ConfFile &) override { return true; }
    void deinit() override {}
    void startReading(uint8_t ch) override { reads.push_back(ch); }
    bool hasData() override { return true; }
    float getSample() override { return 1.0f; }
    void setMaxExpectedVoltage(uint8_t, float) override {}
    float getInputImpedance(uint8_t) override { return 1e6f; }
    float getSamplingRate(uint8_t) override { return 100.0f; } // uniform per-channel rate
};

SensorParams mkp(uint8_t ch, const char *name) {
    return SensorParams{ch, {1.f, 0.f}, {1e9f, 1e9f, false}, name, 'u', false};
}
}

// Vout (added last) is read every other poll: c0, vout, c1, vout, ...
void test_vout_interleave_poll_order() {
    MuxRrMockADC adc;
    ADC_Sampler sampler{};
    sampler.ignoreCalibrationConstraints = true;
    sampler.addSensor(&adc, mkp(0, "c0"), 10.f, 1);
    sampler.addSensor(&adc, mkp(1, "c1"), 10.f, 1);
    sampler.addSensor(&adc, mkp(2, "vout"), 10.f, 1); // Vout last
    sampler.begin();
    for (int i = 0; i < 7; ++i) sampler.update();

    static const uint8_t expected[8] = {0, 2, 1, 2, 0, 2, 1, 2};
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(8, adc.reads.size());
    for (int i = 0; i < 8; ++i)
        TEST_ASSERT_EQUAL_UINT8(expected[i], adc.reads[i]);

    int nVout = 0, nC0 = 0;
    for (int i = 0; i < 8; ++i) {
        if (adc.reads[i] == 2) ++nVout;
        if (adc.reads[i] == 0) ++nC0;
    }
    TEST_ASSERT_EQUAL_INT(4, nVout); // twice as often as the others
    TEST_ASSERT_EQUAL_INT(2, nC0);
}

// Interleaving makes Vout's effective rate 2x and the others' 0.5x; effectiveSampleRate() must
// compensate so the notch is tuned to the real per-channel cadence (base=100, N=3, pollLen=4).
void test_vout_interleave_notch_rate() {
    MuxRrMockADC adc;
    ADC_Sampler sampler{};
    sampler.ignoreCalibrationConstraints = true;
    auto c0 = sampler.addSensor(&adc, mkp(0, "c0"), 10.f, 1);
    sampler.addSensor(&adc, mkp(1, "c1"), 10.f, 1);
    auto vout = sampler.addSensor(&adc, mkp(2, "vout"), 10.f, 1);
    sampler.begin();

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 150.f, sampler.effectiveSampleRate((PhysicalSensor *) vout)); // 100*3*2/4
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 75.f, sampler.effectiveSampleRate((PhysicalSensor *) c0));     // 100*3*1/4
}

// With <=2 channels there is nothing to interleave: plain round-robin, rate unchanged.
void test_cycle_no_interleave_two_channels() {
    MuxRrMockADC adc;
    ADC_Sampler sampler{};
    sampler.ignoreCalibrationConstraints = true;
    auto c0 = sampler.addSensor(&adc, mkp(0, "c0"), 10.f, 1);
    auto vout = sampler.addSensor(&adc, mkp(1, "vout"), 10.f, 1);
    sampler.begin();
    for (int i = 0; i < 5; ++i) sampler.update();

    static const uint8_t expected[6] = {0, 1, 0, 1, 0, 1};
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(6, adc.reads.size());
    for (int i = 0; i < 6; ++i)
        TEST_ASSERT_EQUAL_UINT8(expected[i], adc.reads[i]);

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.f, sampler.effectiveSampleRate((PhysicalSensor *) vout));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.f, sampler.effectiveSampleRate((PhysicalSensor *) c0));
}

void test_calibration_counts_each_sensor_once() {
    MuxRrMockADC adc;
    ADC_Sampler sampler{};
    sampler.ignoreCalibrationConstraints = true;
    auto *fast = const_cast<Sensor *>(sampler.addSensor(&adc, mkp(0, "fast"), 100.f, 1));
    auto *slow = const_cast<Sensor *>(sampler.addSensor(&adc, mkp(1, "slow"), 100.f, 1));

    sampler.begin();
    sampler.startCalibration();
    sampler.update();

    auto feed = [&](Sensor *sensor, float value, int count) {
        for (int i = 0; i < count; ++i) {
            sensor->add_sample(value);
            sampler.handleSensorCalib(*sensor);
        }
    };

    feed(fast, 10.f, 400);
    TEST_ASSERT_TRUE(sampler.isCalibrating());
    TEST_ASSERT_TRUE(fast->calibrationComplete);
    TEST_ASSERT_FALSE(slow->calibrationComplete);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 10.f, fast->calibrationAvg);

    feed(slow, 20.f, 200);
    TEST_ASSERT_FALSE(sampler.isCalibrating());
    TEST_ASSERT_TRUE(slow->calibrationComplete);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.f, slow->calibrationAvg);
}

void test_calibration_reset_is_deferred_to_update() {
    MuxRrMockADC adc;
    ADC_Sampler sampler{};
    sampler.ignoreCalibrationConstraints = true;
    auto *sensor = const_cast<Sensor *>(sampler.addSensor(&adc, mkp(0, "sensor"), 100.f, 1));
    sampler.begin();

    sensor->add_sample(42.f);
    sensor->calibrationAvg = 3.f;
    sensor->calibrationComplete = true;
    sampler.startCalibration();

    TEST_ASSERT_TRUE(sampler.isCalibrating());
    TEST_ASSERT_EQUAL_UINT32(1, sensor->numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.f, sensor->calibrationAvg);
    TEST_ASSERT_TRUE(sensor->calibrationComplete);

    sampler.update();
    TEST_ASSERT_EQUAL_UINT32(1, sensor->numSamples); // reset, then one ADC sample in update()
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.f, sensor->calibrationAvg);
    TEST_ASSERT_FALSE(sensor->calibrationComplete);
}

void test_calibration_restart_is_deferred_to_update() {
    MuxRrMockADC adc;
    ADC_Sampler sampler{};
    sampler.ignoreCalibrationConstraints = true;
    auto *sensor = const_cast<Sensor *>(sampler.addSensor(&adc, mkp(0, "sensor"), 100.f, 1));
    sampler.begin();
    sampler.startCalibration();
    sampler.update();
    sensor->add_sample(42.f);
    const auto samplesBeforeRestart = sensor->numSamples;

    sampler.startCalibration();
    TEST_ASSERT_TRUE(sampler.isCalibrating());
    TEST_ASSERT_EQUAL_UINT32(samplesBeforeRestart, sensor->numSamples);

    sampler.update();
    TEST_ASSERT_EQUAL_UINT32(1, sensor->numSamples); // reset, then one ADC sample in update()
    TEST_ASSERT_FALSE(sensor->calibrationComplete);
}

void test_calibration_cancel_is_deferred_to_update() {
    MuxRrMockADC adc;
    ADC_Sampler sampler{};
    sampler.ignoreCalibrationConstraints = true;
    auto *sensor = const_cast<Sensor *>(sampler.addSensor(&adc, mkp(0, "sensor"), 100.f, 1));
    sampler.begin();
    sampler.startCalibration();
    sampler.update();
    sensor->add_sample(42.f);
    const auto samplesBeforeCancel = sensor->numSamples;

    sampler.cancelCalibration();
    TEST_ASSERT_TRUE(sampler.isCalibrating());
    TEST_ASSERT_EQUAL_UINT32(samplesBeforeCancel, sensor->numSamples);

    sampler.update();
    TEST_ASSERT_FALSE(sampler.isCalibrating());
    TEST_ASSERT_EQUAL_UINT32(1, sensor->numSamples); // reset, then one ADC sample in update()
}

namespace {
// StreamedCallback mock mirroring ADC_ESP32_Cont's no-sample watchdog: read() is the ONLY place
// that delivers samples and (here, like lastDataUs_) clears `good`. Lets us regression-test that
// _updateAdc drains read() even while isGood() is false.
class StreamedMockADC : public AsyncADC<float> {
public:
    bool good = true;
    bool dataPending = false;
    float nextValue = 0;
    int readCalls = 0;

    [[nodiscard]] AdcReadMode readMode() const override { return AdcReadMode::StreamedCallback; }
    bool init(const ConfFile &) override { return true; }
    void deinit() override {}
    void startReading(uint8_t) override {}
    float getSample() override { return 0; }
    void setMaxExpectedVoltage(uint8_t, float) override {}
    float getInputImpedance(uint8_t) override { return 1e6f; }
    float getSamplingRate(uint8_t) override { return 100.0f; }
    bool hasData() override { return dataPending; }
    bool isGood() override { return good; }

    uint32_t read(SampleCallback &&cb) override {
        ++readCalls;
        if (!dataPending) return 0;
        cb(0, nextValue);
        dataPending = false;
        good = true; // delivering data clears the no-sample watchdog (mirrors lastDataUs_ refresh)
        return 1;
    }
};
}

// Regression for the 81c9c10 deadlock: a tripped no-sample watchdog (isGood()==false) must NOT gate
// read() for a StreamedCallback ADC. read() is the only thing that drains the DMA and clears the
// watchdog, so gating it self-latches a dead ADC forever. _updateAdc must drain read() first.
void test_streamed_watchdog_does_not_deadlock_read() {
    StreamedMockADC adc;
    ADC_Sampler sampler{};
    sampler.ignoreCalibrationConstraints = true;
    auto s = sampler.addSensor(&adc, mkp(0, "vin"), 10.f, 1);
    sampler.begin();

    // Watchdog has tripped (stale) but the DMA has a fresh sample queued.
    adc.good = false;
    adc.dataPending = true;
    adc.nextValue = 42.0f;

    sampler.update();

    // Fixed: read() ran despite isGood()==false, delivered the sample, and the ADC recovered.
    // Pre-fix: the isGood() gate returned AdcError before read(), so good stayed false forever.
    TEST_ASSERT_GREATER_OR_EQUAL_INT(1, adc.readCalls);
    TEST_ASSERT_TRUE(adc.good);
    TEST_ASSERT_EQUAL_UINT32(1, s->numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 42.0f, s->last);
}

// Regression for the boot "ADC error" (fry-brk1): TaskNotification::wait() must report a *burst* of
// pending notifications as one wakeup, not starve read(). At boot loopRT arms the watchdog in
// start(), then sits in delay(1000) (CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS) while the conv-done
// ISR piles up thousands of notifications. The old (pdFALSE, ==1) wait() returned false for any
// count != 1, so hasData() was false, read() never ran, lastDataUs_ never refreshed, and the
// no-sample watchdog tripped. Clear-on-exit (pdTRUE, !=0) collapses the burst into one wakeup.
void test_tasknotification_burst_reads_as_one_wakeup() {
    TaskNotification n;
    n.subscribe();          // bind this (the Unity/loop) task
    n.wait(0);              // drain any stale count from prior tests

    for (int i = 0; i < 2000; ++i) n.notify();   // burst, as a 1 s ISR pileup would

    TEST_ASSERT_TRUE(n.wait(10));   // pre-fix: false (count==2000 != 1) -> read() starved
    TEST_ASSERT_FALSE(n.wait(1));   // clear-on-exit drained the whole burst in one wakeup
}

// A single notification still wakes once and only once.
void test_tasknotification_single_wakeup_then_empty() {
    TaskNotification n;
    n.subscribe();
    n.wait(0);
    n.notify();
    TEST_ASSERT_TRUE(n.wait(10));
    TEST_ASSERT_FALSE(n.wait(1));   // timed out: nothing left pending
}
